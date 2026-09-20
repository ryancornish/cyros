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
 * model mirrors how real primitives work (e.g. thread_termination's flag).
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
   bool try_satisfy() noexcept override
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
 * wait_on_any over two waitables
 * ========================================================================= */

TEST_F(SingleCoreWaitables_Test,
       GivenTwoWaitables_WhenSecondIsSatisfied_ThenWinnerIndexIs1)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waiter_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> signaler_stack{};

   TestWaitable w0;
   TestWaitable w1;

   std::size_t winner = static_cast<std::size_t>(-1);

   thread waiter(
      [&]{
         winner = this_thread::wait_on_any(w0, w1);
      },
      waiter_stack,
      thread::priority(0),
      core0
   );

   thread signaler(
      [&]{
         w1.set_and_wake_one();
      },
      signaler_stack,
      thread::priority(1),
      core0
   );

   kernel::start();

   ASSERT_EQ(winner, 1U);
}

TEST_F(SingleCoreWaitables_Test,
       GivenTwoWaitablesBothSatisfied_WhenWaiterChecks_ThenLowestIndexWins)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waiter_stack{};

   TestWaitable w0;
   TestWaitable w1;
   w0.condition.store(true, std::memory_order_release);
   w1.condition.store(true, std::memory_order_release);

   std::size_t winner = static_cast<std::size_t>(-1);

   thread waiter(
      [&]{
         // Both conditions are already true when wait_on_any is called.
         // Tie-break by lowest index = 0.
         winner = this_thread::wait_on_any(w0, w1);
      },
      waiter_stack,
      thread::priority(0),
      core0
   );

   kernel::start();

   ASSERT_EQ(winner, 0U);
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
 * Spurious wake re-check loop on wait_on_any
 * ========================================================================= */

TEST_F(SingleCoreWaitables_Test,
       GivenWaitOnAnyWokenByOneSourceButConditionFalse_ThenWaiterReblocks)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waiter_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> signaler_stack{};

   TestWaitable w0;
   TestWaitable w1;
   std::size_t winner = static_cast<std::size_t>(-1);

   thread waiter(
      [&]{
         winner = this_thread::wait_on_any(w0, w1);
      },
      waiter_stack,
      thread::priority(0),
      core0
   );

   thread signaler(
      [&]{
         // Spurious-style wake: poke w0's queue without setting any condition.
         // The waiter must re-check, find nothing satisfied, and re-block.
         w0.wake_one_no_set();
         // Real wake: set w1, which is what the waiter should ultimately return on.
         w1.set_and_wake_one();
      },
      signaler_stack,
      thread::priority(1),
      core0
   );

   kernel::start();

   ASSERT_EQ(winner, 1U) << "spurious wake on w0 must not be reported as the winner";
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
   bool try_satisfy() noexcept override
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
 *               takes the core immediately.
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
