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

/* ============================================================================
 * Per-waiter records, and chosen()
 *
 * A waiter_record names one waitable and is built on the waiting thread, which
 * installs it. An ordinary wait that includes that waitable then hands the
 * record to that waitable's polls and to no other source's, wherever it sits
 * in the group, and the waiter's own disarm marks it chosen() when a wake of
 * that waitable took its node. chosen() rests on two facts documented at
 * waitable_arm_guard: a node leaves its queue only by one disarm or one wake
 * per pass, and each node is disarmed at most once per pass. These cases pin
 * both, the binding by identity, and that a record is gone once destroyed.
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

   void wake_best() noexcept { wake_one(); }
   void wake_everyone() noexcept { wake_all(); }

   void release_everyone() noexcept
   {
      bail.store(true, std::memory_order_release);
      wake_all();
   }

protected:
   bool try_satisfy(waiter_record* record) noexcept override
   {
      last_polled.store(record, std::memory_order_relaxed);
      if (bail.load(std::memory_order_acquire)) return true;
      if (record == nullptr) return false;
      if (answer_by_chosen) return record->chosen();
      return static_cast<tagged_record*>(record)->ready->load(std::memory_order_acquire);
   }
};

}  // namespace

TEST_F(SingleCoreWaitables_Test,
       GivenARecordNamingOneWaitable_WhenAnotherSourceWakes_ThenOnlyTheNamedWaitableSeesItAndItIsNotChosen)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waker_stack{};

   struct state
   {
      RecordWaitable    self;
      RecordWaitable    other;
      std::atomic<bool> never{false};
      std::size_t       index{99};
      bool              self_saw_record{false};
      bool              other_saw_null{false};
      bool              chosen_by_other_wake{true};
      waiter_record*    seen_after_destroy{reinterpret_cast<waiter_record*>(1)};
   } s;

   thread poller(
      [&s]{
         {
            // Named waitable listed SECOND: the binding is by identity. Neither
            // source is satisfied, so this parks until the waker wakes `other`,
            // whose polls must never see the record, and whose wake must not
            // mark it.
            tagged_record record(s.self, s.never);
            s.index = this_thread::wait_on_any(s.other, s.self);
            s.self_saw_record      = s.self.last_polled.load() == &record;
            s.other_saw_null       = s.other.last_polled.load() == nullptr;
            s.chosen_by_other_wake = record.chosen();
         }

         // Destroyed, so no longer installed: a plain wait gets null.
         s.self.bail = true;
         this_thread::wait_on(s.self);
         s.seen_after_destroy = s.self.last_polled.load();
      },
      stack, thread::priority(0), core0);

   // Less urgent on the same core, so it runs only once the poller has parked.
   thread waker([&s]{ s.other.release_everyone(); }, waker_stack, thread::priority(1), core0);

   kernel::start();

   EXPECT_EQ(s.index, 0u)               << "the wait did not return the other source's index";
   EXPECT_TRUE(s.self_saw_record)       << "the named waitable did not receive the record in position 1";
   EXPECT_TRUE(s.other_saw_null)        << "a source the record does not name received it";
   EXPECT_FALSE(s.chosen_by_other_wake) << "a wake of another source marked the record chosen";
   EXPECT_EQ(s.seen_after_destroy, nullptr) << "a destroyed record still reached a poll";
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

   // Satisfied on the first poll, so the node is left early and then must not
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
