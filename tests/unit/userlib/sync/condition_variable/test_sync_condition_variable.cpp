#include <cyros/sync/condition_variable.hpp>
#include <cyros/sync/mutex.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/config/config.hpp>

#include <common/guarded_stack.hpp>

#include "gtest/gtest.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>

using namespace cyros;

static_assert(config::cores >= 4, "Test suite is designed for (at least) quad-core configuration");

// Constant-initialisable for the same reason as the semaphore: a static one
// lands in .bss with no initialisation-order surface.
constinit static sync::condition_variable constant_initialised_condition_variable{};

/* ============================================================================
 * condition_variable contract, black-box
 *
 * What is asserted: wait releases the mutex while parked and returns owning
 * it, a notify with nobody waiting is not banked, notify_one hands exactly one
 * wake to the most urgent waiter, notify_all releases every waiter, and no
 * notify is lost when the notifier does not hold the mutex, which is the case
 * the standard allows and a sequence-number design gets wrong.
 *
 * A lone waiter's park is proved by a same-core witness of lower priority,
 * which can only run once the waiter has vacated the core. Several waiters
 * sharing a mutex are proved to be waiting by await_waiters instead. Every
 * spin gate waits on a flag set from a different core.
 * ========================================================================= */

namespace
{

constexpr int contract_reps = 3;

class SyncConditionVariable_Test : public ::testing::Test {};

/* A shutdown that fails by NAME rather than by the runner's timeout. Every
 * caller sits alone on its core, so spinning here holds up nothing. */
void wait_or_abort(std::atomic<bool> const& done, char const* what)
{
   using clock = std::chrono::steady_clock;
   auto const deadline = clock::now() + std::chrono::seconds(20);
   while (!done.load(std::memory_order_acquire)) {
      if (clock::now() > deadline) {
         std::fprintf(stderr, "watchdog: %s\n", what);
         std::fflush(stderr);
         std::abort();
      }
      this_core::cpu_relax();
   }
}

/* Block until @p count threads are inside wait on @p cv.
 *
 * Each waiter bumps its counter under the mutex and calls wait before
 * unlocking, and the only thing that unlocks for it is wait's release, which
 * happens after it has armed. So reading the full count under the mutex
 * proves every waiter is armed. A same-core witness cannot prove
 * that here: with several waiters sharing one mutex, a witness also runs when
 * its waiter is parked in lock() and has not reached wait at all. */
void await_waiters(sync::mutex& m, int const& waiting, int count)
{
   while (true) {
      m.lock();
      int const now = waiting;
      m.unlock();
      if (now == count) return;
      this_core::cpu_relax();
   }
}

}  // namespace

/* ============================================================================
 * wait releases the mutex, and returns owning it
 *
 * The notifier can take the mutex only if the parked waiter released it, and
 * the waiter's unlock after wait is a hard error unless it owns the mutex
 * again. The predicate is written under the lock by the notifier, so the
 * waiter reading it true is the handshake the whole type exists for.
 * ========================================================================= */
TEST_F(SyncConditionVariable_Test, GivenAParkedWaiter_WhenNotified_ThenWaitHadReleasedTheMutexAndReturnsOwningIt)
{
   for (int rep = 0; rep < contract_reps; ++rep) {
      SCOPED_TRACE("handshake rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<cyros::test::guarded_stack, 3> stacks;

      struct state
      {
         sync::mutex              m;
         sync::condition_variable cv;
         bool                     ready{false}; // guarded by m
         std::atomic<bool>        parked{false};
         std::atomic<bool>        mutex_was_free{false};
         std::atomic<bool>        saw_ready{false};
      } s;

      thread waiter(
         [&s]{
            s.m.lock();
            while (!s.ready) {
               s.cv.wait(s.m);
            }
            s.saw_ready.store(s.ready, std::memory_order_release);
            s.m.unlock(); // a hard error unless wait handed the mutex back
         },
         stacks[0], thread::priority(1), core0);

      thread witness(
         [&s]{ s.parked.store(true, std::memory_order_release); },
         stacks[1], thread::priority(2), core0);

      thread notifier(
         [&s]{
            while (!s.parked.load(std::memory_order_acquire)) {
               this_core::cpu_relax();
            }
            // try_lock, not lock: a wait that kept the mutex must fail here
            // rather than hang the lifecycle.
            bool const got = s.m.try_lock();
            s.mutex_was_free.store(got, std::memory_order_release);
            if (!got) {
               std::fprintf(stderr, "wait parked still owning the mutex\n");
               std::abort();
            }
            s.ready = true;
            s.cv.notify_one();
            s.m.unlock();
         },
         stacks[2], thread::priority(0), core1);

      kernel::start();

      EXPECT_TRUE(s.mutex_was_free.load()) << "the mutex was still held while its waiter was parked";
      EXPECT_TRUE(s.saw_ready.load())      << "the waiter returned without the notifier's write";

      kernel::finalise();
   }
}

/* ============================================================================
 * wait arms before it releases
 *
 * The release inside wait hands the mutex to a more urgent thread on the same
 * core, which therefore runs the instant it is released and notifies at once.
 * That notify can only reach the waiter if the waiter was already armed when
 * it let go of the mutex. A wait that released first and armed after would
 * find the notify gone and park for good, which the watchdog names. Made
 * deterministic by priority rather than left to a race: under load the same
 * window is a few hundred nanoseconds wide and a stress test rarely hits it.
 * ========================================================================= */
TEST_F(SyncConditionVariable_Test, GivenAMoreUrgentLockerOnTheSameCore_WhenWaitReleasesToIt_ThenItsImmediateNotifyIsNotLost)
{
   for (int rep = 0; rep < contract_reps; ++rep) {
      SCOPED_TRACE("arm-before-release rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<cyros::test::guarded_stack, 3> stacks;

      struct state
      {
         sync::mutex              m;
         sync::condition_variable cv;
         sync::semaphore          go{0};
         bool                     ready{false}; // guarded by m
         std::atomic<bool>        waiter_returned{false};
      } s;

      thread waiter(
         [&s]{
            s.m.lock();
            // The locker preempts here, then parks on the mutex we hold.
            s.go.release();
            while (!s.ready) {
               s.cv.wait(s.m);
            }
            s.m.unlock();
            s.waiter_returned.store(true, std::memory_order_release);
         },
         stacks[0], thread::priority(2), core0);

      thread locker(
         [&s]{
            s.go.acquire();
            s.m.lock(); // granted by the release inside wait, not before
            s.ready = true;
            s.cv.notify_one();
            s.m.unlock();
         },
         stacks[1], thread::priority(1), core0);

      thread watchdog(
         [&s]{ wait_or_abort(s.waiter_returned, "a notify issued at wait's release was lost"); },
         stacks[2], thread::priority(0), core1);

      kernel::start();

      EXPECT_TRUE(s.waiter_returned.load());

      kernel::finalise();
   }
}

/* ============================================================================
 * A notify with nobody waiting is not banked
 *
 * Unlike a semaphore release, a notify that finds no waiter is gone. The
 * waiter starts only after both notifies have run, so its wait must park, and
 * the witness attests the park before the one notify that should free it.
 * ========================================================================= */
TEST_F(SyncConditionVariable_Test, GivenNoWaiter_WhenNotified_ThenALaterWaitStillBlocks)
{
   kernel::initialise();

   static std::array<cyros::test::guarded_stack, 3> stacks;

   struct state
   {
      sync::mutex              m;
      sync::condition_variable cv;
      std::atomic<bool>        early_notifies_done{false};
      std::atomic<bool>        parked{false};
      std::atomic<bool>        returned{false};
      std::atomic<bool>        returned_early{false};
   } s;

   thread waiter(
      [&s]{
         while (!s.early_notifies_done.load(std::memory_order_acquire)) {
            this_core::cpu_relax();
         }
         s.m.lock();
         s.cv.wait(s.m);
         s.returned.store(true, std::memory_order_release);
         s.m.unlock();
      },
      stacks[0], thread::priority(1), core0);

   thread witness(
      [&s]{ s.parked.store(true, std::memory_order_release); },
      stacks[1], thread::priority(2), core0);

   thread notifier(
      [&s]{
         s.cv.notify_one();
         s.cv.notify_all();
         s.early_notifies_done.store(true, std::memory_order_release);

         while (!s.parked.load(std::memory_order_acquire)) {
            this_core::cpu_relax();
         }
         s.returned_early.store(s.returned.load(std::memory_order_acquire), std::memory_order_release);
         s.cv.notify_one();
      },
      stacks[2], thread::priority(0), core1);

   kernel::start();

   EXPECT_FALSE(s.returned_early.load()) << "a notify issued before anyone waited released a later wait";
   EXPECT_TRUE(s.returned.load())        << "the waiter never returned";

   kernel::finalise();
}

/* ============================================================================
 * notify_one hands one wake to the most urgent waiter
 *
 * Three waiters wait on three cores at priorities 3, 1 and 2, then notify_one
 * runs three times from a fourth core without the mutex. Each wait has no
 * predicate loop, so every return is visible: they must come back most urgent
 * first, and one per notify. The settle spin before each notify gives a wake
 * that freed two waiters time to show up as a count ahead of the notifies.
 * ========================================================================= */
TEST_F(SyncConditionVariable_Test, GivenWaitersOfDifferentPriority_WhenNotifiedOneAtATime_ThenOneReturnsPerNotifyMostUrgentFirst)
{
   for (int rep = 0; rep < contract_reps; ++rep) {
      SCOPED_TRACE("priority rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<cyros::test::guarded_stack, 4> stacks;

      struct state
      {
         sync::mutex                m;
         sync::condition_variable   cv;
         std::array<std::uint8_t, 3> order{};  // guarded by m
         int                        waiting{0}; // guarded by m
         std::atomic<int>           returned{0};
         std::atomic<int>           returned_ahead_of_notifies{0};
      } s;

      struct waiter_spec { std::uint8_t priority; core_affinity core; };
      static constexpr std::array<waiter_spec, 3> spec{{
         { 3, core0 }, { 1, core1 }, { 2, core2 },
      }};

      std::array<thread, 3> waiters{};
      for (std::size_t i = 0; i < spec.size(); ++i) {
         waiters[i] = thread(
            [&s, rank = spec[i].priority]{
               s.m.lock();
               ++s.waiting;
               s.cv.wait(s.m);
               auto const at = s.returned.load(std::memory_order_relaxed);
               s.order[static_cast<std::size_t>(at)] = rank;
               s.returned.store(at + 1, std::memory_order_release);
               s.m.unlock();
            },
            stacks[i], thread::priority(spec[i].priority), spec[i].core);
      }

      thread notifier(
         [&s]{
            await_waiters(s.m, s.waiting, 3);
            for (int n = 0; n < 3; ++n) {
               for (int settle = 0; settle < 200'000; ++settle) {
                  this_core::cpu_relax();
               }
               if (s.returned.load(std::memory_order_acquire) != n) {
                  s.returned_ahead_of_notifies.store(s.returned.load(), std::memory_order_release);
               }
               s.cv.notify_one();
               while (s.returned.load(std::memory_order_acquire) < n + 1) {
                  this_core::cpu_relax();
               }
            }
         },
         stacks[3], thread::priority(0), core3);

      kernel::start();

      EXPECT_EQ(s.returned_ahead_of_notifies.load(), 0) << "a single notify_one released more than one waiter";
      EXPECT_EQ(s.order[0], 1) << "the first notify did not go to the most urgent waiter";
      EXPECT_EQ(s.order[1], 2);
      EXPECT_EQ(s.order[2], 3);

      kernel::finalise();
   }
}

/* ============================================================================
 * notify_all releases every waiter
 * ========================================================================= */
TEST_F(SyncConditionVariable_Test, GivenThreeWaiters_WhenNotifyAll_ThenEveryOneReturns)
{
   for (int rep = 0; rep < contract_reps; ++rep) {
      SCOPED_TRACE("broadcast rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<cyros::test::guarded_stack, 4> stacks;

      struct state
      {
         sync::mutex              m;
         sync::condition_variable cv;
         int                      waiting{0}; // guarded by m
         std::atomic<int>         returned{0};
         std::atomic<bool>        all_returned{false};
      } s;

      constexpr core_affinity waiter_core[3] = { core0, core1, core2 };

      std::array<thread, 3> waiters{};
      for (std::size_t i = 0; i < 3; ++i) {
         waiters[i] = thread(
            [&s]{
               s.m.lock();
               ++s.waiting;
               s.cv.wait(s.m);
               s.m.unlock();
               if (s.returned.fetch_add(1, std::memory_order_acq_rel) + 1 == 3) {
                  s.all_returned.store(true, std::memory_order_release);
               }
            },
            stacks[i], thread::priority(1), waiter_core[i]);
      }

      thread notifier(
         [&s]{
            await_waiters(s.m, s.waiting, 3);
            s.cv.notify_all();
            wait_or_abort(s.all_returned, "notify_all left a waiter parked");
         },
         stacks[3], thread::priority(0), core3);

      kernel::start();

      EXPECT_EQ(s.returned.load(), 3);

      kernel::finalise();
   }
}

/* ============================================================================
 * No notify is lost, under load, with the notifier outside the mutex
 *
 * Three consumers on three cores take items; a producer on the fourth adds a
 * burst of one to three items, notifying after it unlocks, then waits for the
 * burst to drain before the next. A consumer that just drained comes straight
 * back to wait, so late waiters race every notify, which is the shape where a
 * wake can be stolen. A notify lost from the tail of a burst strands an item
 * with every consumer parked, and the burst never drains: the watchdog names
 * it. Every seventh notify is a notify_all.
 * ========================================================================= */
TEST_F(SyncConditionVariable_Test, GivenConsumersRacingBackToWait_WhenNotifiedOutsideTheMutex_ThenNoItemIsStranded)
{
   constexpr int bursts = 5'000;

   kernel::initialise();

   static std::array<cyros::test::guarded_stack, 4> stacks;

   struct state
   {
      sync::mutex              m;
      sync::condition_variable cv;
      int                      available{0}; // guarded by m
      bool                     finished{false}; // guarded by m
      std::atomic<int>         consumed{0};
      int                      produced{0};  // producer only
   } s;

   constexpr core_affinity consumer_core[3] = { core0, core1, core2 };

   std::array<thread, 3> consumers{};
   for (std::size_t i = 0; i < 3; ++i) {
      consumers[i] = thread(
         [&s]{
            s.m.lock();
            while (true) {
               s.cv.wait(s.m, [&s]{ return s.available > 0 || s.finished; });
               if (s.available == 0) break;
               --s.available;
               s.consumed.fetch_add(1, std::memory_order_acq_rel);
            }
            s.m.unlock();
         },
         stacks[i], thread::priority(1), consumer_core[i]);
   }

   thread producer(
      [&s]{
         using clock = std::chrono::steady_clock;
         for (int b = 0; b < bursts; ++b) {
            int const burst = 1 + (b % 3);
            for (int k = 0; k < burst; ++k) {
               s.m.lock();
               ++s.available;
               s.m.unlock();
               ++s.produced;
               if (s.produced % 7 == 0) {
                  s.cv.notify_all();
               } else {
                  s.cv.notify_one();
               }
            }
            auto const deadline = clock::now() + std::chrono::seconds(10);
            while (s.consumed.load(std::memory_order_acquire) != s.produced) {
               if (clock::now() > deadline) {
                  std::fprintf(stderr, "watchdog: burst %d stranded %d item(s) with every consumer parked\n",
                               b, s.produced - s.consumed.load());
                  std::fflush(stderr);
                  std::abort();
               }
               this_core::cpu_relax();
            }
         }
         s.m.lock();
         s.finished = true;
         s.m.unlock();
         s.cv.notify_all();
      },
      stacks[3], thread::priority(0), core3);

   kernel::start();

   EXPECT_EQ(s.consumed.load(), s.produced);
   EXPECT_EQ(s.available, 0);

   kernel::finalise();
}
