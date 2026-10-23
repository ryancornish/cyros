/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_singlecore_suspend.cpp
 * @brief this_thread::suspend and thread::resume on one core.
 *
 * Subject: suspend and resume (L4), built on scheduling as a wait is
 * Trusts:  bring-up and scheduling, and waitable for the one test that
 *          resumes a thread waiting on something else
 *
 * Priority 0 is the most urgent. Each test orders its threads by priority, so
 * a less urgent thread runs only once every more urgent one has parked or
 * finished, and that is how a test knows where the others are. A thread that
 * would stay parked if the subject were wrong is resumed by a least urgent
 * thread that records it first, so a failure is an assertion, not a hang.
 */

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

using namespace cyros;

static_assert(config::cores == 1, "Test suite is designed for single core configuration only");

static constexpr auto STACK_SIZE = thread::min_stack_size + (16 * 1024);

int main(int argc, char** argv)
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

namespace
{

class SingleCoreSuspend_Test : public ::testing::Test
{
   void SetUp() override    { kernel::initialise(); }
   void TearDown() override { kernel::finalise(); }
};

alignas(CYROS_PORT_STACK_ALIGN) std::array<std::byte, STACK_SIZE> stack_a{};
alignas(CYROS_PORT_STACK_ALIGN) std::array<std::byte, STACK_SIZE> stack_b{};
alignas(CYROS_PORT_STACK_ALIGN) std::array<std::byte, STACK_SIZE> stack_c{};

/// A waitable a test opens by hand, for a thread that waits on something else.
class TestWaitable final : public waitable
{
public:
   std::atomic<bool> condition{false};

   void set_and_wake_one() noexcept
   {
      condition.store(true, std::memory_order_release);
      wake_one();
   }

protected:
   bool try_satisfy(waiter_record*) noexcept override
   {
      return condition.load(std::memory_order_acquire);
   }
};

} // namespace

TEST_F(SingleCoreSuspend_Test,
       GivenASuspendedThread_WhenAnotherThreadResumesIt_ThenItRunsAtOnce)
{
   struct state
   {
      int steps = 0;
      int steps_before_resume = -1;
      int steps_after_resume = -1;
   } s;

   thread suspender(
      [&s]{
         s.steps = 1;
         this_thread::suspend();
         s.steps = 2;
      },
      stack_a, thread::priority(0), core0
   );

   thread resumer(
      [&s, &suspender]{
         s.steps_before_resume = s.steps;
         suspender.resume();
         // More urgent, so it ran inside the resume
         s.steps_after_resume = s.steps;
      },
      stack_b, thread::priority(1), core0
   );

   kernel::start();

   EXPECT_EQ(s.steps_before_resume, 1); // It parked
   EXPECT_EQ(s.steps_after_resume, 2);
}

TEST_F(SingleCoreSuspend_Test,
       GivenAResumeBeforeTheSuspend_WhenTheThreadSuspends_ThenItReturnsAtOnce)
{
   struct state
   {
      bool returned = false;
      bool parked = false;
   } s;

   thread suspender(
      [&s]{
         this_thread::suspend();
         s.returned = true;
      },
      stack_a, thread::priority(1), core0
   );

   thread resumer(
      [&suspender]{ suspender.resume(); }, // It has not run yet
      stack_b, thread::priority(0), core0
   );

   thread rescuer(
      [&s, &suspender]{
         s.parked = !s.returned;
         if (s.parked) suspender.resume();
      },
      stack_c, thread::priority(2), core0
   );

   kernel::start();

   EXPECT_FALSE(s.parked); // The early resume was kept
   EXPECT_TRUE(s.returned);
}

TEST_F(SingleCoreSuspend_Test,
       GivenTwoResumesBeforeASuspend_WhenTheThreadSuspendsTwice_ThenTheSecondParks)
{
   struct state
   {
      int returns = 0;
      int returns_before_third = -1;
   } s;

   thread suspender(
      [&s]{
         this_thread::suspend();
         ++s.returns;
         this_thread::suspend();
         ++s.returns;
      },
      stack_a, thread::priority(1), core0
   );

   thread resumer(
      [&suspender]{
         suspender.resume();
         suspender.resume();
      },
      stack_b, thread::priority(0), core0
   );

   thread third(
      [&s, &suspender]{
         s.returns_before_third = s.returns;
         suspender.resume();
      },
      stack_c, thread::priority(2), core0
   );

   kernel::start();

   EXPECT_EQ(s.returns_before_third, 1); // One, not counted
   EXPECT_EQ(s.returns, 2);
}

TEST_F(SingleCoreSuspend_Test,
       GivenAThreadWaitingOnAWaitable_WhenItIsResumed_ThenItKeepsWaitingAndItsNextSuspendReturns)
{
   TestWaitable w;
   struct state
   {
      bool past_wait = false;
      bool past_suspend = false;
      bool past_wait_after_resume = true;
      bool past_suspend_after_open = false;
   } s;

   thread waiter(
      [&s, &w]{
         this_thread::wait_on(w);
         s.past_wait = true;
         this_thread::suspend();
         s.past_suspend = true;
      },
      stack_a, thread::priority(0), core0
   );

   thread resumer(
      [&s, &w, &waiter]{
         waiter.resume(); // Lands on the wait, which polls and parks again
         s.past_wait_after_resume = s.past_wait;
         w.set_and_wake_one();
         s.past_suspend_after_open = s.past_suspend;
      },
      stack_b, thread::priority(1), core0
   );

   kernel::start();

   EXPECT_FALSE(s.past_wait_after_resume);
   EXPECT_TRUE(s.past_suspend_after_open); // The resume was kept for it
}

TEST_F(SingleCoreSuspend_Test,
       GivenTwoThreadsResumingEachOther_WhenTheyAlternate_ThenEveryResumeReleasesOneSuspend)
{
   constexpr int rounds = 1000;
   struct state
   {
      thread* a = nullptr;
      thread* b = nullptr;
      int a_turns = 0;
      int b_turns = 0;
      bool in_step = true;
   } s;

   // Equal priority, so neither preempts the other: each resumes the other and
   // then parks, and the other runs only once it has.
   thread a(
      [&s]{
         for (int i = 0; i < rounds; ++i) {
            s.b->resume();
            this_thread::suspend();
            ++s.a_turns;
            if (s.a_turns != s.b_turns) s.in_step = false;
         }
         s.b->resume();
      },
      stack_a, thread::priority(1), core0
   );

   thread b(
      [&s]{
         for (int i = 0; i < rounds; ++i) {
            this_thread::suspend();
            ++s.b_turns;
            s.a->resume();
         }
         this_thread::suspend();
      },
      stack_b, thread::priority(1), core0
   );

   s.a = &a;
   s.b = &b;

   kernel::start();

   EXPECT_EQ(s.a_turns, rounds);
   EXPECT_EQ(s.b_turns, rounds);
   EXPECT_TRUE(s.in_step);
}
