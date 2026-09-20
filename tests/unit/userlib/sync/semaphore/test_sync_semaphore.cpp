#include <cyros/sync/semaphore.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <common/guarded_stack.hpp>

#include "gtest/gtest.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

using namespace cyros;

static_assert(config::cores >= 4, "Test suite is designed for (at least) quad-core configuration");

// A semaphore must stay constant-initialisable, so a static one lands in .bss
// with no startup code and no initialisation-order surface. This is the reason
// the park list is a named-but-unusable type rather than opaque storage: opaque
// storage would need placement-new in the constructor and break this.
constinit static sync::semaphore constant_initialised_semaphore{1};

static constexpr auto STACK_SIZE = thread::min_stack_size + (16 * 1024);

/* ============================================================================
 * Semaphore contract, black-box
 *
 * The semaphore is barge-permitting by design (built on plain wake, not
 * transfer), so this suite deliberately asserts only what that design
 * promises: token accounting is exact, acquire on a zero count BLOCKS until a
 * release supplies a token, releases wake enough waiters, and no token is
 * ever lost or double-granted. It deliberately does NOT assert grant order
 * between concurrent waiters, a woken waiter races fresh arrivals for the
 * token and may lose, which is the documented barging trade.
 *
 * NOTE: GivenZeroCount_WhenAcquire_ThenBlocksUntilRelease and the
 * producer/consumer accounting test are the regression tests for the
 * unconditional-decrement acquire() bug: an acquire that never blocks wraps
 * the counter and both tests fail loudly (accounting) or via the recorded
 * order flag (blocking) rather than by hanging.
 * ========================================================================= */

namespace
{

constexpr int contract_reps = 3;

// Producer/consumer accounting volume per lifecycle. All inside one
// lifecycle, so cheap to raise.
constexpr std::uint64_t tokens_total = 20'000;

class SyncSemaphore_Test : public ::testing::Test {};

}  // namespace

/* ============================================================================
 * Counting contract, single thread
 *
 * Pure state machine, no timing: n tokens grant exactly n try_acquires, the
 * n+1th fails, release restores exactly what it adds, and acquire on a
 * positive count takes the fast path without blocking (reaching the flag is
 * the proof, a wrongly parked thread would prevent quiescence).
 * ========================================================================= */
TEST_F(SyncSemaphore_Test, GivenTokens_WhenSingleThreadDrivesTheCount_ThenAccountingIsExact)
{
   kernel::initialise();

   cyros::test::guarded_stack driver_stack;

   struct state
   {
      semaphore sem{3};
      std::atomic<int>  takes_granted{0};
      std::atomic<bool> fourth_rejected{false};
      std::atomic<bool> fast_acquire_returned{false};
      std::atomic<std::size_t> peek_after_release{0};
   } s;

   thread driver(
      [&s]{
         int granted = 0;
         for (int i = 0; i < 3; ++i) {
            if (s.sem.try_acquire()) ++granted;
         }
         s.takes_granted.store(granted, std::memory_order_release);
         s.fourth_rejected.store(!s.sem.try_acquire(), std::memory_order_release);

         s.sem.release(2);
         s.peek_after_release.store(s.sem.peek(), std::memory_order_release);

         // Positive count: acquire must take the fast path and return.
         s.sem.acquire();
         s.sem.acquire();
         s.fast_acquire_returned.store(true, std::memory_order_release);
      },
      driver_stack,
      thread::priority(0),
      core0
   );

   kernel::start();

   EXPECT_EQ(s.takes_granted.load(), 3)        << "3 tokens did not grant exactly 3 try_acquires";
   EXPECT_TRUE(s.fourth_rejected.load())       << "try_acquire succeeded on an empty semaphore";
   EXPECT_EQ(s.peek_after_release.load(), 2u)  << "release(2) on empty did not read back as 2";
   EXPECT_TRUE(s.fast_acquire_returned.load()) << "acquire on a positive count failed to return";

   kernel::finalise();
}

/* ============================================================================
 * acquire on zero blocks until release
 *
 * The property the current unconditional-decrement acquire() violates. The
 * consumer must NOT get past acquire() before the producer's release, which
 * is proven by ordering, not timing: the producer sets released_first before
 * releasing, and the consumer records whether that flag was up when acquire
 * returned. A non-blocking acquire returns immediately, long before the
 * producer's gate opens, and records the violation.
 * ========================================================================= */
TEST_F(SyncSemaphore_Test, GivenZeroCount_WhenAcquire_ThenBlocksUntilRelease)
{
   for (int rep = 0; rep < contract_reps; ++rep) {
      SCOPED_TRACE("blocking rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<cyros::test::guarded_stack, 3> stacks;

      struct state
      {
         semaphore sem{0};
         std::atomic<bool> consumer_waiting{false};
         std::atomic<bool> released_first{false};
         std::atomic<bool> acquire_after_release{false};
      } s;

      thread consumer(
         [&s]{
            s.sem.acquire(); // must park: count is zero
            s.acquire_after_release.store(s.released_first.load(std::memory_order_acquire),
                                          std::memory_order_release);
         },
         stacks[0],
         thread::priority(1),
         core0
      );

      // Same core as the consumer, strictly lower priority: runs only once
      // the consumer has vacated the core by parking inside acquire, so its
      // store is the attestation the producer gates on. A broken,
      // non-blocking acquire lets the consumer terminate instead, which ALSO
      // lets this run, and the ordering flag then records the violation.
      thread witness(
         [&s]{ s.consumer_waiting.store(true, std::memory_order_release); },
         stacks[1],
         thread::priority(2),
         core0
      );

      thread producer(
         [&s]{
            while (!s.consumer_waiting.load(std::memory_order_acquire)) {
               this_core::cpu_relax();
            }
            s.released_first.store(true, std::memory_order_release);
            s.sem.release();
         },
         stacks[2],
         thread::priority(0),
         core1
      );

      kernel::start();

      EXPECT_TRUE(s.acquire_after_release.load())
         << "acquire returned before any token was released (non-blocking acquire bug)";

      kernel::finalise();
   }
}

/* ============================================================================
 * release(n) frees n parked waiters
 *
 * Three waiters park on a zero semaphore, one per core, each attested by a
 * same-core lower-priority witness. A single release(3) must let all three
 * complete. Completing at all is the wake-sufficiency proof (an under-waking
 * release hangs the stragglers and the framework timeout catches it), and
 * the count arithmetic proves no waiter consumed more than one token.
 * ========================================================================= */
TEST_F(SyncSemaphore_Test, GivenThreeParkedWaiters_WhenReleaseThree_ThenAllProceedOnExactlyOneTokenEach)
{
   for (int rep = 0; rep < contract_reps; ++rep) {
      SCOPED_TRACE("multi-wake rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<cyros::test::guarded_stack, 7> stacks;

      struct state
      {
         semaphore sem{0};
         std::atomic<int> parked{0};
         std::atomic<int> proceeded{0};
         std::atomic<std::size_t> final_peek{~std::size_t{0}};
      } s;

      constexpr core_affinity waiter_core[3] = { core0, core1, core2 };

      std::array<thread, 3> waiters{};
      std::array<thread, 3> witnesses{};
      for (std::size_t i = 0; i < 3; ++i) {
         waiters[i] = thread(
            [&s]{
               s.sem.acquire();
               s.proceeded.fetch_add(1, std::memory_order_acq_rel);
            },
            stacks[i], thread::priority(1), waiter_core[i]);

         // Lower priority on the waiter's own core: proof of park.
         witnesses[i] = thread(
            [&s]{ s.parked.fetch_add(1, std::memory_order_acq_rel); },
            stacks[3 + i], thread::priority(2), waiter_core[i]);
      }

      thread releaser(
         [&s]{
            while (s.parked.load(std::memory_order_acquire) < 3) {
               this_core::cpu_relax();
            }
            s.sem.release(3);
            while (s.proceeded.load(std::memory_order_acquire) < 3) {
               this_core::cpu_relax();
            }
            s.final_peek.store(s.sem.peek(), std::memory_order_release);
         },
         stacks[6], thread::priority(0), core3);

      kernel::start();

      EXPECT_EQ(s.proceeded.load(), 3)   << "release(3) did not free all three parked waiters";
      EXPECT_EQ(s.final_peek.load(), 0u) << "waiters consumed a token count other than three";

      kernel::finalise();
   }
}
/* ============================================================================
 * release(0) is a no-op
 *
 * release(n) adds n and wakes n times, so n == 0 must add nothing and satisfy
 * nobody. What is pinned here is exactly that: the count does not move and the
 * parked waiter does not proceed. A stray wake_one() on top of that is NOT
 * detectable from the public API, because the waitable contract makes a
 * spurious wake legal, the waiter simply re-polls and re-parks, so do not read
 * this test as proof that nothing was woken. The parked waiter is proved parked
 * by a same-core witness of lower urgency: it can only run if the waiter is not
 * runnable.
 * ========================================================================= */
TEST_F(SyncSemaphore_Test, GivenAParkedWaiter_WhenReleasingZero_ThenNothingIsWokenAndTheCountIsUnchanged)
{
   kernel::initialise();

   static std::array<cyros::test::guarded_stack, 2> stacks;

   struct state
   {
      semaphore sem{0};
      std::atomic<bool>        waiter_done{false};
      std::atomic<bool>        waiter_ran_early{false};
      std::atomic<std::size_t> peek_after_zero{~std::size_t{0}};
   } s;

   thread waiter(
      [&s]{
         s.sem.acquire();
         s.waiter_done.store(true, std::memory_order_release);
      },
      stacks[0], thread::priority(1), core0
   );

   // Less urgent and on the same core, so reaching this body at all proves the
   // waiter is parked rather than merely descheduled.
   thread witness(
      [&s]{
         s.sem.release(0);

         s.peek_after_zero.store(s.sem.peek(), std::memory_order_release);
         s.waiter_ran_early.store(s.waiter_done.load(std::memory_order_acquire),
                                  std::memory_order_release);

         s.sem.release(1); // now let it go, so the kernel can quiesce
      },
      stacks[1], thread::priority(2), core0
   );

   kernel::start();

   EXPECT_EQ(s.peek_after_zero.load(), 0u)  << "release(0) changed the count";
   EXPECT_FALSE(s.waiter_ran_early.load())  << "release(0) satisfied a waiter";
   EXPECT_TRUE(s.waiter_done.load())        << "the waiter never got its token";
   EXPECT_EQ(s.sem.peek(), 0u)              << "the single real release left a surplus";

   kernel::finalise();
}

/* ============================================================================
 * A release larger than the demand leaves the surplus in the count
 *
 * release(3) against one waiter grants that waiter exactly one token and keeps
 * two. The extra wake_one calls land on an empty queue and must be harmless.
 * ========================================================================= */
TEST_F(SyncSemaphore_Test, GivenOneWaiter_WhenReleasingMoreThanIsDemanded_ThenTheSurplusRemains)
{
   kernel::initialise();

   static std::array<cyros::test::guarded_stack, 2> stacks;

   struct state
   {
      semaphore sem{0};
      std::atomic<bool> waiter_done{false};
   } s;

   thread waiter(
      [&s]{
         s.sem.acquire();
         s.waiter_done.store(true, std::memory_order_release);
      },
      stacks[0], thread::priority(1), core0
   );

   thread releaser(
      [&s]{ s.sem.release(3); },
      stacks[1], thread::priority(2), core0
   );

   kernel::start();

   EXPECT_TRUE(s.waiter_done.load());
   EXPECT_EQ(s.sem.peek(), 2u) << "three released against one waiter did not leave two";

   kernel::finalise();
}

/* ============================================================================
 * Barging: a more urgent releaser may take back the token it just released
 *
 * The suite header says grant ORDER between concurrent waiters is not asserted.
 * This is the other half of that trade and it IS a property worth pinning,
 * because it is what "built on plain wake, not transfer" buys: waking a waiter
 * reserves nothing for it.
 *
 * Deterministic on one core. The releaser is MORE urgent than the waiter, so
 * readying the waiter does not preempt it, and its own try_acquire runs first
 * and succeeds. The waiter then runs, finds the count empty, and must re-park
 * rather than return with nothing, which is the mandatory re-poll loop around a
 * spurious wake. A second release finally satisfies it.
 *
 * If the semaphore is ever changed to a barge-free handoff, this test fails,
 * and that is the correct signal: it is a deliberate change of the documented
 * trade, not a regression to paper over.
 * ========================================================================= */
TEST_F(SyncSemaphore_Test, GivenAMoreUrgentReleaser_WhenItRetakesItsOwnRelease_ThenTheWaiterReParksAndIsSatisfiedLater)
{
   kernel::initialise();

   static std::array<cyros::test::guarded_stack, 2> stacks;

   struct state
   {
      semaphore sem{0};
      std::atomic<bool> barged{false};
      std::atomic<bool> waiter_done_before_second_release{false};
      std::atomic<bool> waiter_done{false};
   } s;

   thread waiter(
      [&s]{
         s.sem.acquire();
         s.waiter_done.store(true, std::memory_order_release);
      },
      stacks[0], thread::priority(2), core0
   );

   thread releaser(
      [&s]{
         // The waiter is parked (it is more urgent, so it ran first and blocked).
         s.sem.release(1);

         // Still running, because readying a less urgent thread does not
         // preempt. Take the token straight back.
         s.barged.store(s.sem.try_acquire(), std::memory_order_release);

         s.waiter_done_before_second_release.store(
            s.waiter_done.load(std::memory_order_acquire), std::memory_order_release);

         s.sem.release(1);
      },
      stacks[1], thread::priority(1), core0
   );

   kernel::start();

   EXPECT_TRUE(s.barged.load())
      << "the releaser could not retake its own token, so the wake reserved it";
   EXPECT_FALSE(s.waiter_done_before_second_release.load())
      << "the waiter completed on a token that had already been taken";
   EXPECT_TRUE(s.waiter_done.load()) << "the waiter never re-acquired after re-parking";
   EXPECT_EQ(s.sem.peek(), 0u);

   kernel::finalise();
}
