/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

#include <cyros/sync/handoff_semaphore.hpp>
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

// Constant-initialisable, as semaphore is, so a static one needs no startup code.
constinit static sync::handoff_semaphore constant_initialised_handoff_semaphore{1};

/* ============================================================================
 * handoff_semaphore contract, black-box
 *
 * semaphore's counting contract, plus what barging gives up: a release with a
 * waiter queued hands the token to the most urgent waiter, so nothing that
 * runs before that waiter, the releaser included, can take it. Each test
 * that parks a waiter proves the park with a same-core witness of lower
 * urgency, which can run only while the waiter cannot.
 * ========================================================================= */

namespace
{

constexpr int contract_reps = 3;

// Accounting volume per lifecycle, for the stress test.
constexpr std::uint64_t tokens_per_producer = 20'000;
constexpr int stress_reps = 5;

class SyncHandoffSemaphore_Test : public ::testing::Test {};

}  // namespace

TEST_F(SyncHandoffSemaphore_Test, GivenTokens_WhenSingleThreadDrivesTheCount_ThenAccountingIsExact)
{
   kernel::initialise();

   cyros::test::guarded_stack driver_stack;

   struct state
   {
      handoff_semaphore sem{3};
      std::atomic<int>  takes_granted{0};
      std::atomic<bool> fourth_rejected{false};
      std::atomic<bool> fast_acquire_returned{false};
      std::atomic<std::size_t> peek_after_release{0};
      std::atomic<std::size_t> peek_after_release_zero{~std::size_t{0}};
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
         s.sem.release(0);
         s.peek_after_release_zero.store(s.sem.peek(), std::memory_order_release);

         // Positive count: acquire takes the fast path and returns.
         s.sem.acquire();
         s.sem.acquire();
         s.fast_acquire_returned.store(true, std::memory_order_release);
      },
      driver_stack,
      thread::priority(0),
      core0
   );

   kernel::start();

   EXPECT_EQ(s.takes_granted.load(), 3)            << "3 tokens did not grant exactly 3 try_acquires";
   EXPECT_TRUE(s.fourth_rejected.load())           << "try_acquire succeeded on an empty semaphore";
   EXPECT_EQ(s.peek_after_release.load(), 2u)      << "release(2) on empty did not read back as 2";
   EXPECT_EQ(s.peek_after_release_zero.load(), 2u) << "release(0) changed the count";
   EXPECT_TRUE(s.fast_acquire_returned.load())     << "acquire on a positive count failed to return";
   EXPECT_EQ(s.sem.peek(), 0u);

   kernel::finalise();
}

TEST_F(SyncHandoffSemaphore_Test, GivenZeroCount_WhenAcquire_ThenBlocksUntilRelease)
{
   for (int rep = 0; rep < contract_reps; ++rep) {
      SCOPED_TRACE("blocking rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<cyros::test::guarded_stack, 3> stacks;

      struct state
      {
         handoff_semaphore sem{0};
         std::atomic<bool> consumer_waiting{false};
         std::atomic<bool> released_first{false};
         std::atomic<bool> acquire_after_release{false};
      } s;

      thread consumer(
         [&s]{
            s.sem.acquire();
            s.acquire_after_release.store(s.released_first.load(std::memory_order_acquire),
                                          std::memory_order_release);
         },
         stacks[0], thread::priority(1), core0);

      thread witness(
         [&s]{ s.consumer_waiting.store(true, std::memory_order_release); },
         stacks[1], thread::priority(2), core0);

      thread producer(
         [&s]{
            while (!s.consumer_waiting.load(std::memory_order_acquire)) {
               this_core::cpu_relax();
            }
            s.released_first.store(true, std::memory_order_release);
            s.sem.release();
         },
         stacks[2], thread::priority(0), core1);

      kernel::start();

      EXPECT_TRUE(s.acquire_after_release.load())
         << "acquire returned before any token was released";
      EXPECT_EQ(s.sem.peek(), 0u) << "the token handed over was counted as well";

      kernel::finalise();
   }
}

/* ============================================================================
 * No barging: the releaser cannot take back a token it handed over
 *
 * The mirror of semaphore's GivenAMoreUrgentReleaser test, which pins the
 * opposite. On one core: the releaser, more urgent than the waiter, waits on
 * a gate until the waiter has parked, then releases. Readying a less urgent
 * thread does not preempt, so the releaser runs on, and its try_acquire must
 * fail: the token is the waiter's. The waiter then returns from acquire
 * holding it, and nothing is counted.
 * ========================================================================= */
TEST_F(SyncHandoffSemaphore_Test, GivenAParkedWaiter_WhenAMoreUrgentReleaserTriesToRetakeItsToken_ThenItFailsAndTheWaiterHasIt)
{
   for (int rep = 0; rep < contract_reps; ++rep) {
      SCOPED_TRACE("no-barge rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<cyros::test::guarded_stack, 3> stacks;

      struct state
      {
         handoff_semaphore sem{0};
         semaphore gate{0};
         std::atomic<bool> retaken{true};
         std::atomic<std::size_t> peek_after_release{~std::size_t{0}};
         std::atomic<bool> waiter_done_before_releaser_left{true};
         std::atomic<bool> waiter_done{false};
      } s;

      thread releaser(
         [&s]{
            s.gate.acquire();   // until the waiter has parked
            s.sem.release(1);
            s.peek_after_release.store(s.sem.peek(), std::memory_order_release);
            s.retaken.store(s.sem.try_acquire(), std::memory_order_release);
            s.waiter_done_before_releaser_left.store(s.waiter_done.load(std::memory_order_acquire),
                                                     std::memory_order_release);
         },
         stacks[0], thread::priority(1), core0);

      thread waiter(
         [&s]{
            s.sem.acquire();
            s.waiter_done.store(true, std::memory_order_release);
         },
         stacks[1], thread::priority(2), core0);

      // Runs only once the releaser and the waiter have both parked.
      thread witness(
         [&s]{ s.gate.release(); },
         stacks[2], thread::priority(3), core0);

      kernel::start();

      EXPECT_FALSE(s.retaken.load()) << "the releaser took back the token it handed over (barging)";
      EXPECT_EQ(s.peek_after_release.load(), 0u) << "the token handed over was counted";
      EXPECT_FALSE(s.waiter_done_before_releaser_left.load())
         << "the less urgent waiter ran before the releaser blocked or left";
      EXPECT_TRUE(s.waiter_done.load()) << "the waiter never returned with the token handed to it";
      EXPECT_EQ(s.sem.peek(), 0u);

      kernel::finalise();
   }
}

/* ============================================================================
 * The most urgent waiter gets the token, and the other is not overtaken
 *
 * On one core, the less urgent waiter parks first. One release goes to the
 * more urgent waiter. The releaser then tries to take a token for itself, as
 * a newcomer would, and must fail while the other waiter is still queued. A
 * second release then completes the less urgent waiter.
 * ========================================================================= */
TEST_F(SyncHandoffSemaphore_Test, GivenTwoParkedWaiters_WhenReleasingOne_ThenTheMoreUrgentHasItAndANewcomerCannotOvertakeTheOther)
{
   for (int rep = 0; rep < contract_reps; ++rep) {
      SCOPED_TRACE("order rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<cyros::test::guarded_stack, 5> stacks;

      struct state
      {
         handoff_semaphore sem{0};
         semaphore gate{0};
         semaphore second_gate{0};
         std::atomic<int> order{0};
         std::atomic<int> urgent_finished_at{-1};
         std::atomic<int> lesser_finished_at{-1};
         std::atomic<bool> newcomer_took{true};
      } s;

      thread releaser(
         [&s]{
            s.gate.acquire();          // until both waiters have parked
            s.sem.release(1);
            s.second_gate.acquire();   // until the more urgent waiter has run
            s.newcomer_took.store(s.sem.try_acquire(), std::memory_order_release);
            s.sem.release(1);
         },
         stacks[0], thread::priority(1), core0);

      // The less urgent waiter is created first, so it would park first even
      // if creation order decided.
      thread lesser(
         [&s]{
            s.sem.acquire();
            s.lesser_finished_at.store(s.order.fetch_add(1), std::memory_order_release);
         },
         stacks[1], thread::priority(3), core0);

      thread urgent(
         [&s]{
            s.sem.acquire();
            s.urgent_finished_at.store(s.order.fetch_add(1), std::memory_order_release);
            s.second_gate.release();
         },
         stacks[2], thread::priority(2), core0);

      thread witness(
         [&s]{ s.gate.release(); },
         stacks[3], thread::priority(4), core0);

      kernel::start();

      EXPECT_EQ(s.urgent_finished_at.load(), 0) << "the more urgent waiter did not get the first token";
      EXPECT_EQ(s.lesser_finished_at.load(), 1) << "the less urgent waiter did not get the second token";
      EXPECT_FALSE(s.newcomer_took.load()) << "a try_acquire found a token while a waiter was queued";
      EXPECT_EQ(s.sem.peek(), 0u);

      kernel::finalise();
   }
}

/* ============================================================================
 * A release of more tokens than are waited for counts the surplus
 * ========================================================================= */
TEST_F(SyncHandoffSemaphore_Test, GivenOneParkedWaiter_WhenReleasingThree_ThenItProceedsAndTwoAreCounted)
{
   for (int rep = 0; rep < contract_reps; ++rep) {
      SCOPED_TRACE("surplus rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<cyros::test::guarded_stack, 3> stacks;

      struct state
      {
         handoff_semaphore sem{0};
         std::atomic<bool> parked{false};
         std::atomic<bool> done{false};
      } s;

      thread waiter(
         [&s]{
            s.sem.acquire();
            s.done.store(true, std::memory_order_release);
         },
         stacks[0], thread::priority(1), core0);

      thread witness(
         [&s]{ s.parked.store(true, std::memory_order_release); },
         stacks[1], thread::priority(2), core0);

      thread releaser(
         [&s]{
            while (!s.parked.load(std::memory_order_acquire)) {
               this_core::cpu_relax();
            }
            s.sem.release(3);
         },
         stacks[2], thread::priority(0), core1);

      kernel::start();

      EXPECT_TRUE(s.done.load()) << "the parked waiter did not proceed";
      EXPECT_EQ(s.sem.peek(), 2u) << "the surplus was not counted exactly";

      kernel::finalise();
   }
}

/* ============================================================================
 * Exact accounting under contention, across cores
 *
 * Two producers on two cores release, and two consumers on the other two
 * acquire exactly what was released between them. On each consumer's core a
 * less urgent newcomer takes counted tokens with try_acquire and puts them
 * back, whenever its consumer has parked. Every way a token moves runs here
 * at once: counted without the lock, taken back and handed over when a waiter
 * armed meanwhile, handed to a waiter whose first poll had already taken a
 * counted one, which gives one back. A token lost leaves a consumer parked,
 * and the suite's timeout reports it. A token made from nothing shows in the
 * final count.
 * ========================================================================= */
TEST_F(SyncHandoffSemaphore_Test, GivenProducersAndConsumersOnFourCores_WhenTheyContend_ThenNoTokenIsLostOrMade)
{
   for (int rep = 0; rep < stress_reps; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<cyros::test::guarded_stack, 6> stacks;

      struct state
      {
         handoff_semaphore sem{0};
         std::atomic<std::uint64_t> consumed{0};
         std::atomic<std::uint64_t> taken_by_newcomers{0};
         std::atomic<int> consumers_done{0};
      } s;

      constexpr core_affinity producer_core[2] = { core0, core1 };
      constexpr core_affinity consumer_core[2] = { core2, core3 };

      std::array<thread, 2> producers{};
      std::array<thread, 2> consumers{};
      std::array<thread, 2> newcomers{};
      for (std::size_t i = 0; i < 2; ++i) {
         producers[i] = thread(
            [&s]{
               // A delay that varies, so a release finds a consumer
               // parked, arming or polling as often as finding none.
               for (std::uint64_t n = 0; n < tokens_per_producer; ++n) {
                  for (std::uint64_t spin = 0; spin < (n * 7) % 64; ++spin) {
                     this_core::cpu_relax();
                  }
                  s.sem.release();
               }
            },
            stacks[i], thread::priority(1), producer_core[i]);

         consumers[i] = thread(
            [&s]{
               for (std::uint64_t n = 0; n < tokens_per_producer; ++n) {
                  s.sem.acquire();
                  s.consumed.fetch_add(1, std::memory_order_acq_rel);
               }
               s.consumers_done.fetch_add(1, std::memory_order_acq_rel);
            },
            stacks[2 + i], thread::priority(1), consumer_core[i]);

         newcomers[i] = thread(
            [&s]{
               while (s.consumers_done.load(std::memory_order_acquire) < 2) {
                  if (s.sem.try_acquire()) {
                     s.taken_by_newcomers.fetch_add(1, std::memory_order_acq_rel);
                     s.sem.release();
                  }
                  // A yield, not a spin: on the cooperative port its
                  // consumer, handed a token, runs only when this yields.
                  this_thread::yield();
               }
            },
            stacks[4 + i], thread::priority(2), consumer_core[i]);
      }

      kernel::start();

      EXPECT_EQ(s.consumed.load(), 2 * tokens_per_producer) << "the consumers did not take every token";
      EXPECT_EQ(s.sem.peek(), 0u) << "a token was made: counted after every released one was taken";

      kernel::finalise();
   }
}
