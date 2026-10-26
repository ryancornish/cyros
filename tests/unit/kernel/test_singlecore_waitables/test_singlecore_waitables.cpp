/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/kernel/waitable.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include "gtest/gtest.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

using namespace cyros;

static_assert(config::cores == 1, "Test suite is designed for single core configuration only");

static constexpr auto STACK_SIZE = thread::min_stack_size + (16 * 1024);

int main(int argc, char** argv)
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

class SingleCoreWaitables_Test : public ::testing::Test
{
   void SetUp() override    { kernel::initialise(); }
   void TearDown() override { kernel::finalise(); }
};

/**
 * @brief Test waitable with an externally-driven condition.
 *
 * Backed by a plain atomic flag the test can flip. wake_one/wake_all are
 * exposed via thin wrappers so tests can drive them directly. The flag-based
 * model mirrors how real primitives work (e.g. cancel_signal's flag).
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

   // For tests that need to drive wakes without changing the condition (to
   // exercise spurious-wake behaviour explicitly).
   void wake_one_no_set() noexcept { wake_one(); }
   void wake_all_no_set() noexcept { wake_all(); }

   // The reschedule policy is part of the wake surface and is otherwise only
   // ever used at its default, so these expose the other two arms.
   void set_and_wake_one(reschedule_policy policy) noexcept
   {
      condition.store(true, std::memory_order_release);
      wake_one(policy);
   }

protected:
   bool try_satisfy(waiter_record*) noexcept override
   {
      return condition.load(std::memory_order_acquire);
   }
};

/* ============================================================================
 * Single-waitable wait_on
 * ========================================================================= */

TEST_F(SingleCoreWaitables_Test,
       GivenOneWaiter_WhenConditionSetAndWakeOne_ThenWaiterReturns)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waiter_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> signaler_stack{};

   TestWaitable w;
   bool waiter_completed = false;

   thread waiter(
      [&]{
         this_thread::wait_on(w);
         waiter_completed = true;
      },
      waiter_stack,
      thread::priority(0),
      core0
   );

   thread signaler(
      [&]{
         // Waiter has already blocked by the time we run (lower priority).
         w.set_and_wake_one();
      },
      signaler_stack,
      thread::priority(1),
      core0
   );

   kernel::start();

   ASSERT_TRUE(waiter_completed);
}

TEST_F(SingleCoreWaitables_Test,
       GivenConditionAlreadyTrue_WhenWaiterCallsWaitOn_ThenWaiterDoesNotBlock)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waiter_stack{};

   TestWaitable w;
   w.condition.store(true, std::memory_order_release);

   bool waiter_completed = false;

   thread waiter(
      [&]{
         this_thread::wait_on(w);   // condition already true; should return immediately
         waiter_completed = true;
      },
      waiter_stack,
      thread::priority(0),
      core0
   );

   kernel::start();

   // No signaler thread; if wait_on did not respect the already-true condition,
   // the kernel would hang in idle. Reaching here proves the early-return path.
   ASSERT_TRUE(waiter_completed);
}

TEST_F(SingleCoreWaitables_Test,
       GivenWaiter_WhenWokenButConditionStillFalse_ThenWaiterReblocks)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waiter_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> signaler_stack{};

   TestWaitable w;
   bool waiter_completed = false;

   thread waiter(
      [&]{
         this_thread::wait_on(w);
         waiter_completed = true;
      },
      waiter_stack,
      thread::priority(0),
      core0
   );

   thread signaler(
      [&]{
         // First wake: condition is still false. Waiter must re-block.
         w.wake_one_no_set();

         // Second wake: now set the condition. Waiter should observe and return.
         w.set_and_wake_one();
      },
      signaler_stack,
      thread::priority(1),
      core0
   );

   kernel::start();

   ASSERT_TRUE(waiter_completed);
}

/* ============================================================================
 * wait_on with a cancel signal
 * ========================================================================= */

TEST_F(SingleCoreWaitables_Test,
       GivenAWaitWithACancel_WhenTheCancelIsRaised_ThenTheWaitReturnsFalse)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waiter_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> signaler_stack{};

   TestWaitable w;
   cancel_signal cancel;

   int result = -1;

   thread waiter(
      [&]{
         result = this_thread::wait_on(w, cancel) ? 1 : 0;
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
      thread::priority(1),
      core0
   );

   kernel::start();

   ASSERT_EQ(result, 0);
}

TEST_F(SingleCoreWaitables_Test,
       GivenWAndTheCancelBothHoldAlready_WhenWaiting_ThenWWins)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waiter_stack{};

   TestWaitable w;
   cancel_signal cancel;
   w.condition.store(true, std::memory_order_release);
   cancel.raise();

   int result = -1;

   thread waiter(
      [&]{
         // Both hold before the wait starts. w is polled first.
         result = this_thread::wait_on(w, cancel) ? 1 : 0;
      },
      waiter_stack,
      thread::priority(0),
      core0
   );

   kernel::start();

   ASSERT_EQ(result, 1);
}

/* ============================================================================
 * wake_one with multiple waiters: priority order
 * ========================================================================= */

TEST_F(SingleCoreWaitables_Test,
       GivenTwoWaitersDifferentPriority_WhenWakeOne_ThenHighestPriorityWakesFirst)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> hi_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> lo_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> signaler_stack{};

   TestWaitable w;
   std::atomic<int> wake_sequence{0};
   int hi_wake_position = 0;
   int lo_wake_position = 0;

   thread hi(
      [&]{
         this_thread::wait_on(w);
         hi_wake_position = wake_sequence.fetch_add(1) + 1;
      },
      hi_stack,
      thread::priority(0),   // highest priority (numerically smallest)
      core0
   );

   thread lo(
      [&]{
         this_thread::wait_on(w);
         lo_wake_position = wake_sequence.fetch_add(1) + 1;
      },
      lo_stack,
      thread::priority(3),
      core0
   );

   thread signaler(
      [&]{
         // First wake should target the highest-priority waiter (hi).
         w.set_and_wake_one();
         // Second wake should target the remaining waiter (lo). The condition
         // is already true from the first call; just wake_one again.
         w.wake_one_no_set();
      },
      signaler_stack,
      thread::priority(10),
      core0
   );

   kernel::start();

   ASSERT_EQ(hi_wake_position, 1) << "high-priority waiter should wake first";
   ASSERT_EQ(lo_wake_position, 2) << "low-priority waiter should wake second";
}

/* ============================================================================
 * wake_all
 * ========================================================================= */

TEST_F(SingleCoreWaitables_Test,
       GivenThreeWaiters_WhenWakeAll_ThenAllThreeReturn)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> a_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> b_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> c_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> signaler_stack{};

   TestWaitable w;
   std::atomic<int> woke_count{0};

   thread a([&]{ this_thread::wait_on(w); woke_count.fetch_add(1); }, a_stack, thread::priority(2), core0);
   thread b([&]{ this_thread::wait_on(w); woke_count.fetch_add(1); }, b_stack, thread::priority(1), core0);
   thread c([&]{ this_thread::wait_on(w); woke_count.fetch_add(1); }, c_stack, thread::priority(3), core0);

   thread signaler(
      [&]{
         w.set_and_wake_all();
      },
      signaler_stack,
      thread::priority(7),
      core0
   );

   kernel::start();

   ASSERT_EQ(woke_count.load(), 3);
}

/* ============================================================================
 * Spurious wake re-check loop on a wait with a cancel
 * ========================================================================= */

TEST_F(SingleCoreWaitables_Test,
       GivenAWaitWithACancel_WhenWIsWokenButNotSatisfied_ThenTheWaiterParksAgainUntilTheCancel)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waiter_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> signaler_stack{};

   TestWaitable w;
   cancel_signal cancel;
   int result = -1;

   thread waiter(
      [&]{
         result = this_thread::wait_on(w, cancel) ? 1 : 0;
      },
      waiter_stack,
      thread::priority(0),
      core0
   );

   thread signaler(
      [&]{
         // Spurious-style wake: poke w's queue without setting its condition.
         // The waiter must re-check, find nothing satisfied, and re-block.
         w.wake_one_no_set();
         // Then the cancel, which is what the waiter should return on.
         cancel.raise();
      },
      signaler_stack,
      thread::priority(1),
      core0
   );

   kernel::start();

   ASSERT_EQ(result, 0) << "a spurious wake of w must not be reported as w";
}

/* ============================================================================
 * A wake of w that lands as the cancel wins is not lost
 *
 * The cancel is already raised when T waits, so T's first poll of w fails and
 * the cancel's succeeds. A release of w landing between w's poll and its
 * disarm takes T's node, since T heads w's queue, though that wake was for
 * the token the release brought. U waits on w behind T. The wait must poll w
 * after its disarm and before the cancel may end it: T then takes the token
 * and returns true. Ending on the cancel instead leaves U parked beside it.
 *
 * w's own poll makes the release, after it has looked and found nothing,
 * which puts it in exactly that window. One core, deterministic: T is gated
 * until U has parked, and the judge, least urgent, looks once T is done.
 * ========================================================================= */

namespace
{

class LateReleaseWaitable final : public waitable
{
public:
   std::atomic<int>        tokens{0};
   std::atomic<thread::id> release_late_for{0};

   void release() noexcept
   {
      tokens.fetch_add(1, std::memory_order_acq_rel);
      wake_one();
   }

protected:
   bool try_satisfy(waiter_record*) noexcept override
   {
      bool const took = take();
      if (!took && release_late_for.load(std::memory_order_acquire) == this_thread::id()) {
         release_late_for.store(0, std::memory_order_relaxed);
         release(); // after this poll looked, before the waiter's disarm
      }
      return took;
   }

private:
   bool take() noexcept
   {
      int current = tokens.load(std::memory_order_acquire);
      while (current > 0) {
         if (tokens.compare_exchange_weak(current, current - 1,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed)) {
            return true;
         }
      }
      return false;
   }
};

}  // namespace

TEST_F(SingleCoreWaitables_Test,
       GivenARaisedCancel_WhenAWakeOfWLandsAfterItsPoll_ThenTheWaitStillTakesWAndNobodyIsStranded)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> t_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> u_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> judge_stack{};

   struct state
   {
      LateReleaseWaitable w;
      cancel_signal       cancel;
      TestWaitable        gate;
      int  t_result{-1};
      int  tokens_left{-1};
      bool u_acquired{false};
      bool u_stranded{false};
   } s;
   s.cancel.raise();

   thread t(
      [&s]{
         this_thread::wait_on(s.gate);
         s.w.release_late_for.store(this_thread::id(), std::memory_order_release);
         s.t_result = this_thread::wait_on(s.w, s.cancel) ? 1 : 0;
      },
      t_stack, thread::priority(1), core0);

   thread u(
      [&s]{
         this_thread::wait_on(s.w);
         s.u_acquired = true;
      },
      u_stack, thread::priority(2), core0);

   thread judge(
      [&s]{
         // T outranks this, and nothing in its wait can park it, so it has
         // finished by the time this runs again.
         s.gate.set_and_wake_one();
         s.tokens_left = s.w.tokens.load(std::memory_order_acquire);
         s.u_stranded  = !s.u_acquired && s.tokens_left > 0;
         if (!s.u_acquired) {
            s.w.release(); // free U, so the kernel quiesces
         }
      },
      judge_stack, thread::priority(3), core0);

   kernel::start();

   EXPECT_EQ(s.t_result, 1) << "the cancel ended a wait whose w had a token waiting for it";
   EXPECT_FALSE(s.u_stranded) << "U parked beside the token the late release brought";
   EXPECT_EQ(s.tokens_left, 0);
}

/* ============================================================================
 * Sanity: condition is read on the waiter's thread (caller TCB passed through)
 * ========================================================================= */

class IdentityCheckingWaitable final : public waitable
{
public:
   std::atomic<thread::id> seen_caller_id{0};
   std::atomic<bool>       go{false};

   void release() noexcept
   {
      go.store(true, std::memory_order_release);
      wake_one();
   }

protected:
   bool try_satisfy(waiter_record*) noexcept override
   {
      // try_satisfy runs in the calling thread's context, so this_thread::id()
      // is the waiter's identity. That is the contract a transfer-shaped
      // primitive relies on to recognise ownership, so pin it here.
      seen_caller_id.store(this_thread::id(), std::memory_order_relaxed);
      return go.load(std::memory_order_acquire);
   }
};

TEST_F(SingleCoreWaitables_Test,
       GivenWaiter_WhenTrySatisfyRuns_ThenThisThreadIdIsTheWaitingThread)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waiter_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> signaler_stack{};

   IdentityCheckingWaitable w;
   thread::id captured_waiter_id = 0;

   thread waiter(
      [&]{
         captured_waiter_id = this_thread::id();
         this_thread::wait_on(w);
      },
      waiter_stack,
      thread::priority(0),
      core0
   );

   thread signaler(
      [&]{
         w.release();
      },
      signaler_stack,
      thread::priority(1),
      core0
   );

   kernel::start();

   ASSERT_EQ(w.seen_caller_id.load(), captured_waiter_id);
}


/* ============================================================================
 * reschedule_policy decides who keeps the core after a wake
 *
 * wake_one takes a policy and every caller in the tree uses the default, so the
 * other two arms had no consumer at all. On one core the three are directly
 * observable, and the difference is not cosmetic: it decides whether the waker
 * runs on after waking somebody.
 *
 *   automatic : reschedule only if the woken thread is a better pick than the
 *               running one. An EQUAL-priority wake is therefore not a better
 *               pick and the waker keeps the core.
 *   always    : reschedule regardless. The waker goes to the back of its
 *               priority's FIFO run queue, so an equal-priority woken thread
 *               takes the core immediately. A more urgent one preempts it,
 *               and that preemption is the one reschedule: the waker keeps
 *               the front of its level.
 *   never     : do not reschedule even when the woken thread IS more urgent.
 *               It stays ready until the next scheduling point the waker
 *               reaches on its own.
 * ========================================================================= */

TEST_F(SingleCoreWaitables_Test,
       GivenAnEqualPriorityWaiter_WhenWokenWithAutomatic_ThenTheWakerKeepsTheCore)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waiter_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waker_stack{};

   TestWaitable w;
   std::atomic<bool> waiter_done{false};
   std::atomic<bool> waiter_ran_before_waker_finished{false};

   thread waiter(
      [&]{
         this_thread::wait_on(w);
         waiter_done.store(true, std::memory_order_release);
      },
      waiter_stack, thread::priority(1), core0
   );

   thread waker(
      [&]{
         w.set_and_wake_one(); // automatic
         waiter_ran_before_waker_finished.store(waiter_done.load(std::memory_order_acquire),
                                                std::memory_order_release);
      },
      waker_stack, thread::priority(1), core0
   );

   kernel::start();

   EXPECT_FALSE(waiter_ran_before_waker_finished.load())
      << "an equal-priority wake preempted the waker under the automatic policy";
   EXPECT_TRUE(waiter_done.load());
}

TEST_F(SingleCoreWaitables_Test,
       GivenAnEqualPriorityWaiter_WhenWokenWithAlways_ThenTheWakerYieldsTheCore)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waiter_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waker_stack{};

   TestWaitable w;
   std::atomic<bool> waiter_done{false};
   std::atomic<bool> waiter_ran_before_waker_finished{false};

   thread waiter(
      [&]{
         this_thread::wait_on(w);
         waiter_done.store(true, std::memory_order_release);
      },
      waiter_stack, thread::priority(1), core0
   );

   thread waker(
      [&]{
         w.set_and_wake_one(reschedule_policy::always);
         waiter_ran_before_waker_finished.store(waiter_done.load(std::memory_order_acquire),
                                                std::memory_order_release);
      },
      waker_stack, thread::priority(1), core0
   );

   kernel::start();

   EXPECT_TRUE(waiter_ran_before_waker_finished.load())
      << "the always policy did not hand the core to the equal-priority waiter";
   EXPECT_TRUE(waiter_done.load());
}

TEST_F(SingleCoreWaitables_Test,
       GivenAMoreUrgentWaiter_WhenWokenWithAlways_ThenTheWakerReschedulesOnceAndKeepsItsTurn)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waiter_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waker_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> equal_stack{};

   TestWaitable w;
   std::atomic<bool> waiter_done{false};
   std::atomic<bool> equal_ran{false};
   std::atomic<bool> waiter_ran_first{false};
   std::atomic<bool> equal_ran_before_waker_resumed{false};

   // MORE urgent than the waker, so the wake itself preempts it.
   thread waiter(
      [&]{
         this_thread::wait_on(w);
         waiter_done.store(true, std::memory_order_release);
      },
      waiter_stack, thread::priority(0), core0
   );

   thread waker(
      [&]{
         w.set_and_wake_one(reschedule_policy::always);

         // `always` asks for one reschedule, and the wake that preempted the
         // waker was it. The waker was preempted, not rotated, so it is back
         // at the front of its level and runs on ahead of its equal.
         waiter_ran_first.store(waiter_done.load(std::memory_order_acquire), std::memory_order_release);
         equal_ran_before_waker_resumed.store(equal_ran.load(std::memory_order_acquire),
                                              std::memory_order_release);
      },
      waker_stack, thread::priority(1), core0
   );

   // Behind the waker at its level.
   thread equal(
      [&]{ equal_ran.store(true, std::memory_order_release); },
      equal_stack, thread::priority(1), core0
   );

   kernel::start();

   EXPECT_TRUE(waiter_ran_first.load())
      << "the always policy did not hand the core to the more urgent waiter";
   EXPECT_FALSE(equal_ran_before_waker_resumed.load())
      << "the waker was rescheduled twice: a second reschedule after the preempting one sent it behind its equal";
   EXPECT_TRUE(equal_ran.load());
}

TEST_F(SingleCoreWaitables_Test,
       GivenAMoreUrgentWaiter_WhenWokenWithNever_ThenItWaitsForTheWakersNextSchedulingPoint)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waiter_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waker_stack{};

   TestWaitable w;
   std::atomic<bool> waiter_done{false};
   std::atomic<bool> waiter_ran_immediately{false};

   // MORE urgent than the waker, so under any other policy it would preempt.
   thread waiter(
      [&]{
         this_thread::wait_on(w);
         waiter_done.store(true, std::memory_order_release);
      },
      waiter_stack, thread::priority(0), core0
   );

   thread waker(
      [&]{
         w.set_and_wake_one(reschedule_policy::never);

         // Still running despite having readied a more urgent thread.
         waiter_ran_immediately.store(waiter_done.load(std::memory_order_acquire),
                                      std::memory_order_release);

         // A voluntary scheduling point: now the more urgent thread must run.
         this_thread::yield();
      },
      waker_stack, thread::priority(1), core0
   );

   kernel::start();

   EXPECT_FALSE(waiter_ran_immediately.load())
      << "the never policy still preempted the waker";
   EXPECT_TRUE(waiter_done.load())
      << "the woken thread never ran, so never lost the wake rather than deferring it";
}

/* ============================================================================
 * Per-waiter records, and chosen()
 *
 * A waiter_record names one waitable and is built on the waiting thread, which
 * installs it. An ordinary wait on that waitable then hands the record to its
 * polls and to no other waitable's, and the waiter's own disarm marks it
 * chosen() when a wake of that waitable took its node. chosen() rests on two
 * facts documented at wait_engine: a node leaves its queue only by one disarm
 * or one wake per pass, and each node is disarmed exactly once per pass. These
 * cases pin both, the binding by identity, and that a record is gone once
 * destroyed.
 *
 * RecordWaitable answers from the record alone: the flag it points at, or
 * chosen() when answer_by_chosen is set, as a condition variable does. The
 * bail flag lets a broken kernel still quiesce instead of hanging.
 * ========================================================================= */

namespace
{

struct tagged_record : waiter_record
{
   tagged_record(waitable& source, std::atomic<bool>& ready) noexcept
      : waiter_record(source), ready(&ready) {}

   std::atomic<bool>* ready;
};

class RecordWaitable final : public waitable
{
public:
   std::atomic<bool>           bail{false};
   bool                        answer_by_chosen{false};
   std::atomic<waiter_record*> last_polled{nullptr};
   std::atomic<waiter_record*> wake_during_poll_of{nullptr}; ///< a wake lands inside this record's next poll

   void wake_best() noexcept { wake_one(); }
   void wake_everyone() noexcept { wake_all(); }

   /// wake_all_where with the poll's own answer, noting each record it is handed.
   std::array<waiter_record const*, 4> handed{};
   std::size_t                         handed_count{0};
   void wake_where_ready() noexcept
   {
      handed_count = 0;
      wake_all_where([this](waiter_record const* record) noexcept {
         if (handed_count < handed.size()) handed[handed_count++] = record;
         return record != nullptr
             && static_cast<tagged_record const*>(record)->ready->load(std::memory_order_acquire);
      });
   }

   void release_everyone() noexcept
   {
      bail.store(true, std::memory_order_release);
      wake_all();
   }

protected:
   bool try_satisfy(waiter_record* record) noexcept override
   {
      last_polled.store(record, std::memory_order_relaxed);
      if (record != nullptr && record == wake_during_poll_of.load(std::memory_order_acquire)) {
         wake_during_poll_of.store(nullptr, std::memory_order_relaxed);
         wake_one();
      }
      if (bail.load(std::memory_order_acquire)) return true;
      if (record == nullptr) return false;
      if (answer_by_chosen) return record->chosen();
      return static_cast<tagged_record*>(record)->ready->load(std::memory_order_acquire);
   }
};

}  // namespace

TEST_F(SingleCoreWaitables_Test,
       GivenARecordNamingW_WhenTheCancelEndsTheWait_ThenOnlyWSawItAndItIsNotChosen)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waker_stack{};

   struct state
   {
      RecordWaitable    self;
      RecordWaitable    other;
      cancel_signal     cancel;
      std::atomic<bool> never{false};
      int               result{-1};
      bool              self_saw_record{false};
      bool              chosen_by_cancel{true};
      waiter_record*    seen_after_destroy{reinterpret_cast<waiter_record*>(1)};
      waiter_record*    seen_with_foreign_record{reinterpret_cast<waiter_record*>(1)};
   } s;

   thread poller(
      [&s]{
         {
            // Neither w nor the cancel holds, so this parks until the waker
            // raises the cancel, whose wake must not mark the record.
            tagged_record record(s.self, s.never);
            s.result           = this_thread::wait_on(s.self, s.cancel) ? 1 : 0;
            s.self_saw_record  = s.self.last_polled.load() == &record;
            s.chosen_by_cancel = record.chosen();
         }

         // Destroyed, so no longer installed: a plain wait gets null.
         s.self.bail = true;
         this_thread::wait_on(s.self);
         s.seen_after_destroy = s.self.last_polled.load();

         // A record naming another waitable reaches no poll of this one: the
         // binding is by identity.
         {
            tagged_record record(s.other, s.never);
            this_thread::wait_on(s.self);
            s.seen_with_foreign_record = s.self.last_polled.load();
         }
      },
      stack, thread::priority(0), core0);

   // Less urgent on the same core, so it runs only once the poller has parked.
   thread waker([&s]{ s.cancel.raise(); }, waker_stack, thread::priority(1), core0);

   kernel::start();

   EXPECT_EQ(s.result, 0)                   << "the wait did not report the cancel";
   EXPECT_TRUE(s.self_saw_record)           << "w's poll did not receive the record naming it";
   EXPECT_FALSE(s.chosen_by_cancel)         << "the cancel's wake marked w's record chosen";
   EXPECT_EQ(s.seen_after_destroy, nullptr) << "a destroyed record still reached a poll";
   EXPECT_EQ(s.seen_with_foreign_record, nullptr) << "a record naming another waitable reached this one's poll";
}

TEST_F(SingleCoreWaitables_Test,
       GivenWaitersWithTheirOwnRecords_WhenAllAreWoken_ThenEachPollAnswersForItsOwnWaiter)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s0{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s1{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s2{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> sd{};

   struct state
   {
      RecordWaitable w;
      std::array<std::atomic<bool>, 3> ready{};
      std::array<bool, 3> done{};
      int  returned{0};
      bool only_middle{false};
   } s;

   std::array<std::array<std::byte, STACK_SIZE>*, 3> stacks{ &s0, &s1, &s2 };
   std::array<thread, 3> waiters{};
   for (std::size_t i = 0; i < 3; ++i) {
      waiters[i] = thread(
         [&s, i]{
            {
               tagged_record record(s.w, s.ready[i]);
               this_thread::wait_on(s.w);
            }
            s.done[i] = true;
            ++s.returned;
         },
         *stacks[i], thread::priority(static_cast<std::uint8_t>(1 + i)), core0);
   }

   // Least urgent on the only core, so it runs once all three are parked. Every
   // waiter is woken, but only the one whose record is ready may return: the
   // others must each read their OWN record and park again.
   thread driver(
      [&s]{
         s.ready[1] = true;
         s.w.wake_everyone();
         this_thread::yield();
         s.only_middle = s.returned == 1 && s.done[1] && !s.done[0] && !s.done[2];

         s.ready[0] = true;
         s.ready[2] = true;
         s.w.wake_everyone();
         this_thread::yield();
         s.w.release_everyone(); // quiesce whatever a broken poll stranded
      },
      sd, thread::priority(4), core0);

   kernel::start();

   EXPECT_TRUE(s.only_middle) << "a poll answered from a record other than its own waiter's";
   EXPECT_EQ(s.returned, 3);
}

TEST_F(SingleCoreWaitables_Test,
       GivenRecordsNamingThisAndAnotherWaitable_WhenWakeAllWhere_ThenEachWaiterIsAskedWithTheRecordItsPollGets)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s0{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s1{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s2{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> sd{};

   struct state
   {
      RecordWaitable    w;
      RecordWaitable    other;
      std::atomic<bool> ready{false};
      std::atomic<bool> never{false};
      waiter_record*    own{nullptr};
      std::array<bool, 3> done{};
      int  returned{0};
      bool only_first{false};
   } s;

   // In queue order: a record naming w, a record naming another waitable on a
   // waiter of w, and no record at all. The filter must be handed the first,
   // then null twice, exactly what each one's poll of w gets.
   thread named(
      [&s]{
         {
            tagged_record record(s.w, s.ready);
            s.own = &record;
            this_thread::wait_on(s.w);
         }
         s.done[0] = true;
         ++s.returned;
      },
      s0, thread::priority(1), core0);
   thread elsewhere(
      [&s]{
         {
            tagged_record record(s.other, s.never);
            this_thread::wait_on(s.w);
         }
         s.done[1] = true;
         ++s.returned;
      },
      s1, thread::priority(2), core0);
   thread bare(
      [&s]{
         this_thread::wait_on(s.w);
         s.done[2] = true;
         ++s.returned;
      },
      s2, thread::priority(3), core0);

   // Least urgent on the only core, so it runs once all three are parked.
   thread driver(
      [&s]{
         s.ready = true;
         s.w.wake_where_ready();
         this_thread::yield();
         s.only_first = s.returned == 1 && s.done[0] && !s.done[1] && !s.done[2];
         s.w.release_everyone();
      },
      sd, thread::priority(4), core0);

   kernel::start();

   ASSERT_EQ(s.w.handed_count, 3u) << "the filter was not asked once about each queued waiter";
   EXPECT_EQ(s.w.handed[0], s.own)   << "the waiter whose record names w was not asked with it";
   EXPECT_EQ(s.w.handed[1], nullptr) << "a record naming another waitable reached w's filter";
   EXPECT_EQ(s.w.handed[2], nullptr);
   EXPECT_TRUE(s.only_first) << "wake_all_where woke a waiter its filter refused, or not the one it accepted";
   EXPECT_EQ(s.returned, 3);
}

TEST_F(SingleCoreWaitables_Test,
       GivenWaitersThatAnswerByChosen_WhenWakeOne_ThenOnlyTheBestWaiterIsChosenAndReturns)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s0{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s1{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s2{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> sd{};

   struct state
   {
      RecordWaitable w;
      std::array<std::atomic<bool>, 3> never{};
      std::array<int, 3>  order{};
      std::array<bool, 3> was_chosen{};
      int returned{0};
      int returned_after_first{-1};
   } s;

   s.w.answer_by_chosen = true;

   // Priorities 3, 1, 2, so the wakes must choose waiter 1, then 2, then 0.
   constexpr std::array<std::uint8_t, 3> prio{ 3, 1, 2 };
   std::array<std::array<std::byte, STACK_SIZE>*, 3> stacks{ &s0, &s1, &s2 };
   std::array<thread, 3> waiters{};
   for (std::size_t i = 0; i < 3; ++i) {
      waiters[i] = thread(
         [&s, i]{
            bool chosen = false;
            {
               tagged_record record(s.w, s.never[i]);
               this_thread::wait_on(s.w);
               chosen = record.chosen();
            }
            s.was_chosen[i] = chosen;
            s.order[static_cast<std::size_t>(s.returned++)] = static_cast<int>(i);
         },
         *stacks[i], thread::priority(prio[i]), core0);
   }

   // Least urgent on the only core, so it runs once all three are parked, and
   // each woken waiter, being more urgent, runs before this resumes.
   thread driver(
      [&s]{
         s.w.wake_best();
         s.returned_after_first = s.returned;
         s.w.wake_best();
         s.w.wake_best();
         s.w.release_everyone(); // quiesce whatever a broken mark stranded
      },
      sd, thread::priority(4), core0);

   kernel::start();

   EXPECT_EQ(s.returned_after_first, 1) << "one wake_one released a count other than one";
   EXPECT_EQ(s.order[0], 1) << "the first wake did not choose the most urgent waiter";
   EXPECT_EQ(s.order[1], 2);
   EXPECT_EQ(s.order[2], 0);
   EXPECT_TRUE(s.was_chosen[0] && s.was_chosen[1] && s.was_chosen[2])
      << "a waiter a wake returned was not marked chosen";
}

TEST_F(SingleCoreWaitables_Test,
       GivenAWaiterSatisfiedByItsOwnPoll_WhenItReturns_ThenItIsNotChosen)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> stack{};

   struct state
   {
      RecordWaitable    w;
      std::atomic<bool> ready{true};
      bool              chosen{true};
   } s;

   // Satisfied on the first poll, so the node is disarmed once, and must not
   // be disarmed again: a second disarm would find it gone and mark a wake
   // that never happened.
   thread waiter(
      [&s]{
         tagged_record record(s.w, s.ready);
         this_thread::wait_on(s.w);
         s.chosen = record.chosen();
      },
      stack, thread::priority(0), core0);

   kernel::start();

   EXPECT_FALSE(s.chosen) << "a waiter nobody woke was marked chosen (a node disarmed twice)";
}

TEST_F(SingleCoreWaitables_Test,
       GivenARecordBoundToW_WhenAWakeTakesTheNodeOfASatisfiedPoll_ThenTheRecordGetsItAndNobodyElseIsWoken)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> t_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> u_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> judge_stack{};

   struct state
   {
      RecordWaitable    w;
      TestWaitable      gate;
      std::atomic<bool> ready{true};
      std::atomic<bool> never{false};
      waiter_record*    t_record{nullptr};
      waiter_record*    last_polled_after_t{nullptr};
      bool              t_chosen{false};
   } s;

   // A wake takes T's node inside a poll that satisfies T, the window where an
   // ordinary wait passes the wake on to the waiter behind. But a record bound
   // to w owns its wakes: it is marked chosen, as a condition variable learns
   // of its notify, and passing the wake on as well would deliver it twice.
   // U, behind T, must not be woken, so the last poll w sees is still T's.
   thread t(
      [&s]{
         this_thread::wait_on(s.gate);
         tagged_record record(s.w, s.ready);
         s.t_record = &record;
         s.w.wake_during_poll_of.store(&record, std::memory_order_release);
         this_thread::wait_on(s.w);
         s.t_chosen = record.chosen();
      },
      t_stack, thread::priority(1), core0);

   thread u(
      [&s]{
         tagged_record record(s.w, s.never);
         this_thread::wait_on(s.w);
      },
      u_stack, thread::priority(2), core0);

   thread judge(
      [&s]{
         // Runs once T is gated and U parked. T outranks this and cannot
         // park, and U outranks this too, so both have run if woken.
         s.gate.set_and_wake_one();
         s.last_polled_after_t = s.w.last_polled.load();
         s.w.release_everyone(); // quiesce U
      },
      judge_stack, thread::priority(3), core0);

   kernel::start();

   EXPECT_TRUE(s.t_chosen) << "the wake that took T's node was not recorded on T's record";
   EXPECT_EQ(s.last_polled_after_t, s.t_record) << "the wake was passed on to U as well";
}
