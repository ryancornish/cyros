/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/kernel/waitable.hpp>
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

static constexpr auto STACK_SIZE = thread::min_stack_size + (16 * 1024);

int main(int argc, char** argv)
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

namespace
{

// Tune this knob to trade test speed for race-detection confidence.
// 200 catches most races with reasonable test runtime; bump for harder
// hunting, drop when iterating on unrelated work.
constexpr int stress_repetitions = 200;

// Each test owns its own kernel lifecycle per rep, so the fixture is empty.
// Inheriting from ::testing::Test (rather than using TEST instead of TEST_F)
// keeps the test naming consistent with the other multicore suites.
class MultiCoreWaitables_Test : public ::testing::Test {};

/**
 * @brief Test waitable with an externally-driven atomic condition.
 *
 * Mirrors the one used by single-core waitable tests. wake_one/wake_all
 * are exposed via thin wrappers so signaler threads can drive them
 * directly without needing protected access.
 */
class TestWaitable final : public waitable
{
public:
   std::atomic<bool> condition{false};

   void set_and_wake_one() noexcept
   {
      condition.store(true, std::memory_order_release);
      wake_one();
   }

   void set_and_wake_all() noexcept
   {
      condition.store(true, std::memory_order_release);
      wake_all();
   }

   void wake_one_no_set() noexcept { wake_one(); }
   void wake_all_no_set() noexcept { wake_all(); }

protected:
   bool try_satisfy(waiter_record*) noexcept override
   {
      return condition.load(std::memory_order_acquire);
   }
};

/**
 * @brief A counting waitable, one wake per token as a semaphore's release,
 *        whose poll can hold one named thread just after it takes a token.
 *
 * The hold stands for a preemption or an interrupt landing between a
 * waiter's satisfied poll and its disarm, the window a wake can still take
 * the waiter's node in.
 */
class HeldCountingWaitable final : public waitable
{
public:
   std::atomic<int>        tokens{0};
   std::atomic<thread::id> hold_for{0};
   std::atomic<bool>       took{false};
   std::atomic<bool>       may_continue{false};

   void release() noexcept
   {
      tokens.fetch_add(1, std::memory_order_acq_rel);
      wake_one();
   }

protected:
   bool try_satisfy(waiter_record*) noexcept override
   {
      int current = tokens.load(std::memory_order_acquire);
      while (current > 0) {
         if (tokens.compare_exchange_weak(current, current - 1,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed)) {
            if (hold_for.load(std::memory_order_acquire) == this_thread::id()) {
               took.store(true, std::memory_order_release);
               while (!may_continue.load(std::memory_order_acquire)) {
                  this_core::cpu_relax();
               }
            }
            return true;
         }
      }
      return false;
   }
};

}  // namespace

/* ============================================================================
 * Cross-core wake: waiter on one core, signaler on another
 *
 * The most basic SMP waitable property: a wake originating on a different
 * core than the waiter must reach the waiter via the intake + IPI path.
 * Replicating this under stress builds confidence that the cross-core
 * routing is reliable across many timing windows.
 * ========================================================================= */
TEST_F(MultiCoreWaitables_Test,
       GivenWaiterOnCore0AndSignalerOnCore1_WhenSignalerWakes_ThenWaiterReturns)
{
   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      cyros::test::guarded_stack waiter_stack;
      cyros::test::guarded_stack signaler_stack;

      TestWaitable w;
      std::atomic<bool> waiter_completed{false};

      thread waiter(
         [&]{
            this_thread::wait_on(w);
            waiter_completed.store(true, std::memory_order_release);
         },
         waiter_stack,
         thread::priority(0),
         core0
      );

      thread signaler(
         [&]{
            w.set_and_wake_one();
         },
         signaler_stack,
         thread::priority(0),
         core1
      );

      kernel::start();

      EXPECT_TRUE(waiter_completed.load(std::memory_order_acquire));
      kernel::finalise();
   }
}

/* ============================================================================
 * Cross-core cancel of a waiter parked with a cancel signal
 * ========================================================================= */
TEST_F(MultiCoreWaitables_Test,
       GivenWaiterOnCore0WithACancel_WhenCore1RaisesIt_ThenTheWaitReturnsFalse)
{
   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      cyros::test::guarded_stack waiter_stack;
      cyros::test::guarded_stack signaler_stack;

      TestWaitable w;
      cancel_signal cancel;
      std::atomic<int> result{-1};

      thread waiter(
         [&]{
            result.store(this_thread::wait_on(w, cancel) ? 1 : 0, std::memory_order_release);
         },
         waiter_stack,
         thread::priority(0),
         core0
      );

      thread signaler(
         [&]{
            cancel.raise();
         },
         signaler_stack,
         thread::priority(0),
         core1
      );

      kernel::start();

      EXPECT_EQ(result.load(std::memory_order_acquire), 0) << "the cancel did not end the wait";
      kernel::finalise();
   }
}

/* ============================================================================
 * Two signalers on different cores both wake the same waiter
 *
 * This is the double-wake idempotency case: two cores nearly simultaneously
 * call wake_one on the same waitable. The first wake unlinks the waiter;
 * the second wake finds the queue empty and is a no-op. The waiter must
 * not be enqueued twice into the ready matrix.
 *
 * If double-enqueue happened, the most likely symptom would be a corrupt
 * ready list - either an assert in the scheduler, an infinite loop, or a
 * use-after-free. So "test passes" here is "kernel survives the scenario."
 * ========================================================================= */
TEST_F(MultiCoreWaitables_Test,
       GivenTwoSignalersOnDifferentCores_WhenBothWakeSameWaitable_ThenWaiterReturnsOnce)
{
   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      cyros::test::guarded_stack waiter_stack;
      cyros::test::guarded_stack s1_stack;
      cyros::test::guarded_stack s2_stack;

      TestWaitable w;
      std::atomic<int> completion_count{0};

      thread waiter(
         [&]{
            this_thread::wait_on(w);
            completion_count.fetch_add(1, std::memory_order_acq_rel);
         },
         waiter_stack,
         thread::priority(0),
         core0
      );

      thread signaler1(
         [&]{
            w.set_and_wake_one();
         },
         s1_stack,
         thread::priority(0),
         core1
      );

      thread signaler2(
         [&]{
            // Don't set condition again - just wake. If the first wake
            // already woke the waiter, this is a wake against an empty
            // queue (no-op). If somehow the queue still has the waiter,
            // this re-wakes (which would also be fine, just redundant).
            w.wake_one_no_set();
         },
         s2_stack,
         thread::priority(0),
         core2
      );

      kernel::start();

      EXPECT_EQ(completion_count.load(std::memory_order_acquire), 1)
         << "waiter must complete exactly once, no matter how many wakes target it";
      kernel::finalise();
   }
}

/* ============================================================================
 * wake_all across cores: three waiters on different cores, one signaler
 *
 * Exercises the multi-core fan-out of wake_all: the signaler's wake_all
 * posts set_thread_ready into three different schedulers' intakes (or
 * directly readies its own core's waiter, depending on placement). All
 * three waiters must return.
 * ========================================================================= */
TEST_F(MultiCoreWaitables_Test,
       GivenThreeWaitersOnThreeCores_WhenSignalerOnFourthWakesAll_ThenAllThreeReturn)
{
   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      cyros::test::guarded_stack w0_stack;
      cyros::test::guarded_stack w1_stack;
      cyros::test::guarded_stack w2_stack;
      cyros::test::guarded_stack signaler_stack;

      TestWaitable w;
      std::atomic<int> woke_count{0};

      thread waiter0(
         [&]{ this_thread::wait_on(w); woke_count.fetch_add(1, std::memory_order_acq_rel); },
         w0_stack, thread::priority(0), core0);

      thread waiter1(
         [&]{ this_thread::wait_on(w); woke_count.fetch_add(1, std::memory_order_acq_rel); },
         w1_stack, thread::priority(0), core1);

      thread waiter2(
         [&]{ this_thread::wait_on(w); woke_count.fetch_add(1, std::memory_order_acq_rel); },
         w2_stack, thread::priority(0), core2);

      thread signaler(
         [&]{
            // Yield a few times to maximise the chance all three waiters
            // are parked before we fire. This isn't strictly required for
            // correctness - a waiter that hasn't yet armed will see the
            // condition true on its first check - but exercising the
            // "all three are blocked" path is more interesting.
            for (int i = 0; i < 5; ++i) this_thread::yield();
            w.set_and_wake_all();
         },
         signaler_stack,
         thread::priority(0),
         core3
      );

      kernel::start();

      EXPECT_EQ(woke_count.load(std::memory_order_acquire), 3);
      kernel::finalise();
   }
}

/* ============================================================================
 * Cross-core wake racing the waiter's own arm
 *
 * This is the scenario the "case ready" scheduler tolerance was designed
 * for. The signaler on core1 calls set_and_wake_one in a loop; the waiter
 * on core0 calls wait_on in a loop. The wake may fire BEFORE the waiter
 * has finished arming and yielded - the intake routing must serialise this
 * so the wake isn't lost.
 *
 * We can't deterministically trigger the inner-window race on a
 * cooperative port, but running many iterations gives the OS scheduler
 * many chances to interleave the two pthreads adversarially.
 * ========================================================================= */
TEST_F(MultiCoreWaitables_Test,
       GivenWakeRacingWithArm_WhenStressed_ThenNoWakeIsLost)
{
   constexpr int iterations_per_rep = 50;

   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));
      //std::printf("\nTEST ITERATION: %d\n", rep);

      kernel::initialise();

      cyros::test::guarded_stack waiter_stack;
      cyros::test::guarded_stack signaler_stack;

      TestWaitable w;
      std::atomic<int> rounds_completed{0};

      thread waiter(
         [&]{
            for (int i = 0; i < iterations_per_rep; ++i) {
               this_thread::wait_on(w);
               // Reset for next round. The signaler will set it again.
               w.condition.store(false, std::memory_order_release);
               rounds_completed.fetch_add(1, std::memory_order_acq_rel);
            }
         },
         waiter_stack,
         thread::priority(0),
         core0
      );

      thread signaler(
         [&]{
            for (int i = 0; i < iterations_per_rep; ++i) {
               // Wait until the waiter has consumed the previous wake.
               // Without this, the signaler races ahead and sets the
               // condition before the waiter has cleared it.
               while (w.condition.load(std::memory_order_acquire)) {
                  this_thread::yield();
               }
               w.set_and_wake_one();
            }
         },
         signaler_stack,
         thread::priority(0),
         core1
      );

      kernel::start();

      EXPECT_EQ(rounds_completed.load(std::memory_order_acquire), iterations_per_rep)
         << "waiter completed fewer rounds than the signaler sent wakes - lost wake";
      kernel::finalise();
   }
}

/* ============================================================================
 * A wake that takes a satisfied waiter's node is passed on
 *
 * A wake takes the head node, whatever its owner is doing. Between a
 * waiter's satisfied poll and its disarm its node is still queued, so a wake
 * landing there spends itself on a thread that needs nothing, and a waiter
 * behind it stays parked beside the token that wake was for. The engine must
 * pass such a wake on.
 *
 *    core1  U    parks, the only waiter
 *    core1  R1   releases: pops U, readies it (R1 outranks U, U waits)
 *    core0  T    arms (U's node is gone, so T heads the queue), polls, takes
 *                R1's token, and is held there by the poll
 *    core1  U    runs: its poll fails, it re-arms behind T, parks
 *    core1  W1   releases: the head is T's node, still queued
 *    core0  T    disarms, finds its node taken, returns
 *
 * Without the pass-on U stays parked with one token unclaimed, and W1, which
 * runs only once U has parked, finds exactly that. T waits directly rather
 * than through a fast path, which is R1's release landing between a fast
 * path's failed take and its arm. Deterministic, so a few lifecycles.
 * ========================================================================= */
TEST_F(MultiCoreWaitables_Test,
       GivenSatisfiedWaiterStillQueued_WhenAWakeTakesItsNode_ThenTheWakeReachesTheParkedWaiter)
{
   constexpr int lifecycles = 3;

   for (int rep = 0; rep < lifecycles; ++rep) {
      SCOPED_TRACE("lifecycle " + std::to_string(rep));

      kernel::initialise();

      cyros::test::guarded_stack r1_stack;
      cyros::test::guarded_stack u_stack;
      cyros::test::guarded_stack w1_stack;
      cyros::test::guarded_stack t_stack;

      struct state
      {
         HeldCountingWaitable w;
         TestWaitable first_gate;   // holds R1 until U has parked
         TestWaitable second_gate;  // holds R1 once T has the token, so U runs
         std::atomic<bool> r1_released{false};
         std::atomic<bool> t_done{false};
         std::atomic<bool> u_acquired{false};
         std::atomic<bool> u_stranded{false};
         std::atomic<int>  tokens_left{-1};
      } s;

      // core1, in priority order: R1 first to its gate, then U onto w, then W1.
      thread r1(
         [&s]{
            this_thread::wait_on(s.first_gate);
            s.w.release();
            s.r1_released.store(true, std::memory_order_release);
            while (!s.w.took.load(std::memory_order_acquire)) {
               this_core::cpu_relax();
            }
            this_thread::wait_on(s.second_gate);
         },
         r1_stack, thread::priority(1), core1);

      thread u(
         [&s]{
            this_thread::wait_on(s.w);
            s.u_acquired.store(true, std::memory_order_release);
         },
         u_stack, thread::priority(5), core1);

      thread w1(
         [&s]{
            // Runs only once R1 and U are parked, and again only once R1 is
            // parked at its second gate and U parked behind T.
            s.first_gate.set_and_wake_one();
            s.w.release();
            s.w.may_continue.store(true, std::memory_order_release);
            while (!s.t_done.load(std::memory_order_acquire)) {
               this_core::cpu_relax();
            }
            // A wake passed on to U was posted to this core before T
            // returned. The yield drains it, and U outranks us, so U has run
            // by the time this thread does again.
            this_thread::yield();
            s.tokens_left.store(s.w.tokens.load(std::memory_order_acquire), std::memory_order_release);
            bool const stranded = !s.u_acquired.load(std::memory_order_acquire);
            s.u_stranded.store(stranded, std::memory_order_release);
            if (stranded) {
               s.w.release(); // free U, so the kernel quiesces
            }
            s.second_gate.set_and_wake_one();
         },
         w1_stack, thread::priority(9), core1);

      thread t(
         [&s]{
            while (!s.r1_released.load(std::memory_order_acquire)) {
               this_core::cpu_relax();
            }
            s.w.hold_for.store(this_thread::id(), std::memory_order_release);
            this_thread::wait_on(s.w);
            s.t_done.store(true, std::memory_order_release);
         },
         t_stack, thread::priority(3), core0);

      kernel::start();

      EXPECT_FALSE(s.u_stranded.load(std::memory_order_acquire))
         << "U parked beside an unclaimed token: the wake for it was spent on T";
      EXPECT_EQ(s.tokens_left.load(std::memory_order_acquire), 0);
      kernel::finalise();
   }
}

/* ============================================================================
 * A waitable and a cancel signal woken by different cores
 *
 * The waiter on core0 waits on w until a cancel. Each has its own signaler on
 * a different core, and exactly one fires per rep, so the result must say
 * which.
 * ========================================================================= */
TEST_F(MultiCoreWaitables_Test,
       GivenWaitWithACancel_WhenWOrTheCancelFiresFromAnotherCore_ThenTheResultSaysWhich)
{
   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      cyros::test::guarded_stack waiter_stack;
      cyros::test::guarded_stack s_for_w_stack;
      cyros::test::guarded_stack s_for_cancel_stack;

      TestWaitable w;
      cancel_signal cancel;
      std::atomic<int> result{-1};

      // Alternate which one fires per rep so both outcomes get exercised.
      const bool fire_w_this_rep = (rep % 2 == 0);

      thread waiter(
         [&]{
            result.store(this_thread::wait_on(w, cancel) ? 1 : 0, std::memory_order_release);
         },
         waiter_stack,
         thread::priority(0),
         core0
      );

      thread signaler_w(
         [&]{
            if (fire_w_this_rep) {
               w.set_and_wake_one();
            }
         },
         s_for_w_stack,
         thread::priority(0),
         core1
      );

      thread signaler_cancel(
         [&]{
            if (!fire_w_this_rep) {
               cancel.raise();
            }
         },
         s_for_cancel_stack,
         thread::priority(0),
         core2
      );

      kernel::start();

      EXPECT_EQ(result.load(std::memory_order_acquire), fire_w_this_rep ? 1 : 0);
      kernel::finalise();
   }
}

/* ============================================================================
 * Cross-core join: target on one core, joiner on another
 *
 * thread::join() is implemented via wait_on(termination), so this is a
 * waitable test in disguise. With the target on core0 and the joiner on
 * core1, the raise of the target's termination signal (running on core0)
 * must wake the joiner parked on core1.
 * ========================================================================= */
TEST_F(MultiCoreWaitables_Test,
       GivenTargetOnCore0AndJoinerOnCore1_WhenTargetTerminates_ThenJoinerReturns)
{
   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      cyros::test::guarded_stack target_stack;
      cyros::test::guarded_stack joiner_stack;

      std::atomic<bool> joiner_returned{false};

      thread target(
         [&]{
            // Just terminate cleanly. The joiner is parked on our termination.
         },
         target_stack,
         thread::priority(0),
         core0
      );

      thread joiner(
         [&]{
            target.join();
            joiner_returned.store(true, std::memory_order_release);
         },
         joiner_stack,
         thread::priority(0),
         core1
      );

      kernel::start();

      EXPECT_TRUE(joiner_returned.load(std::memory_order_acquire));
      kernel::finalise();
   }
}


/* ============================================================================
 * Heavy fan-in: one core drives w and another raises the cancel, at one waiter
 *
 * The waiter on core0 loops on wait_on(w, cancel). One signaler drives w in a
 * handshake loop: wait until the waiter has consumed its last set, set, wake.
 * Another does the same with the cancel: wait until the waiter has reset it,
 * raise. The waiter consumes whichever ended each wait, clearing w on true
 * and resetting the cancel on false, and tallies it.
 *
 * The handshakes mean every set and every raise is consumed exactly once, so
 * the waiter performs exactly 2 * iterations_per_rep waits and each side's
 * tally must reach iterations_per_rep. Both often hold at once here, and then
 * w wins and the cancel ends the next wait. A lost wake, or a node disarmed
 * wrongly under adversarial interleaving across three cores, shows up as a
 * hang or as a tally short of its count.
 * ========================================================================= */
TEST_F(MultiCoreWaitables_Test,
       GivenOneCoreDrivingWAndAnotherRaisingTheCancel_WhenStressed_ThenEveryEventIsSeenOnce)
{
   constexpr int iterations_per_rep = 40;
   constexpr int total_waits = 2 * iterations_per_rep;

   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      cyros::test::guarded_stack waiter_stack;
      cyros::test::guarded_stack w_stack;
      cyros::test::guarded_stack cancel_stack;

      struct state
      {
         TestWaitable     w;
         cancel_signal    cancel;
         std::atomic<int> by_w{0};
         std::atomic<int> by_cancel{0};
      } s;

      thread waiter(
         [&s]{
            for (int i = 0; i < total_waits; ++i) {
               if (this_thread::wait_on(s.w, s.cancel)) {
                  s.w.condition.store(false, std::memory_order_release);
                  s.by_w.fetch_add(1, std::memory_order_acq_rel);
               } else {
                  s.cancel.reset();
                  s.by_cancel.fetch_add(1, std::memory_order_acq_rel);
               }
            }
         },
         waiter_stack, thread::priority(0), core0);

      thread w_signaler(
         [&s]{
            for (int i = 0; i < iterations_per_rep; ++i) {
               while (s.w.condition.load(std::memory_order_acquire)) {
                  this_thread::yield();
               }
               s.w.set_and_wake_one();
            }
         },
         w_stack, thread::priority(0), core1);

      thread cancel_signaler(
         [&s]{
            for (int i = 0; i < iterations_per_rep; ++i) {
               while (s.cancel.raised()) {
                  this_thread::yield();
               }
               s.cancel.raise();
            }
         },
         cancel_stack, thread::priority(0), core2);

      kernel::start();

      EXPECT_EQ(s.by_w.load(std::memory_order_acquire), iterations_per_rep);
      EXPECT_EQ(s.by_cancel.load(std::memory_order_acquire), iterations_per_rep);
      kernel::finalise();
   }
}
