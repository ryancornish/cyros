/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_singlecore_interrupt_wake.cpp
 * @brief A wake made in an interrupt decides its switch there, on
 *        linux_preempt, whose interrupt is its timer signal.
 *
 * Subject: an interrupt's wake (scheduler::interrupt_reschedule) and
 *          linux_preempt's half of it, cyros_port_interrupted_at_baseline and
 *          cyros_port_owe_switch
 * Trusts:  layers 0 to 4: the port, the periodic time driver, the kernel's
 *          threads, suspend and resume, and a waitable's wake
 *
 * The twin of test_target_interrupt_wake, which says what each case is for. The
 * interrupt is the timer's signal, through a callback time::schedule_at files.
 * In each case the driver, the least urgent thread, spins until the callback
 * has run, so the signal always takes it at baseline (in the second case, with
 * preemption disabled), and the threads the callback wakes note themselves in
 * one shared order when they run.
 */

#include <cyros/kernel/core.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/waitable.hpp>
#include <cyros/time/time.hpp>
#include <cyros/config/config.hpp>

#include <common/guarded_stack.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <vector>

using namespace cyros;

namespace
{

constexpr std::uint32_t tick_hz = 1'000;

constexpr std::uint32_t x1 = 1u;          /* priority 10, created before x2 */
constexpr std::uint32_t x2 = 2u;          /* priority 10                    */
constexpr std::uint32_t b = 3u;           /* priority 5                     */
constexpr std::uint32_t y = 4u;           /* priority 3, waits on the gate  */
constexpr std::uint32_t driver = 9u;      /* priority 20, the interrupted   */

/* A batch wake, which pends one reschedule after readying its waiters. */
class gate final : public waitable
{
public:
   void open() noexcept
   {
      opens.fetch_add(1u, std::memory_order_acq_rel);
      wake_all();
   }

   void pass() noexcept { this_thread::wait_on(*this); }

protected:
   bool try_satisfy(waiter_record*) noexcept override
   {
      std::uint32_t current = opens.load(std::memory_order_acquire);
      while (current > 0u) {
         if (opens.compare_exchange_weak(current, current - 1u, std::memory_order_acq_rel)) {
            return true;
         }
      }
      return false;
   }

private:
   std::atomic<std::uint32_t> opens{0u};
};

struct shared_state
{
   std::vector<std::uint32_t> notes;
   std::atomic<bool>          fired{false};
   bool                       ran{false};
   bool                       still_the_driver{false};
   thread::id                 driver_id{0};
   gate                       the_gate;
   thread*                    thread_x1{nullptr};
   thread*                    thread_x2{nullptr};
   thread*                    thread_b{nullptr};
};

shared_state* state = nullptr;

/* Under a critical section, so the signal cannot split it. */
void note(std::uint32_t who)
{
   this_core::critical_guard const guard;
   state->notes.push_back(who);
}

/* Has the timer's signal run CALLBACK two ticks from now, spinning until it
 * has, for a second at most. */
bool run_and_wait(time::callback callback)
{
   time::handle const handle = time::schedule_at(time::now() + time::duration{2}, callback, nullptr);
   if (handle.id == 0) {
      return false;
   }
   auto const deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
   for (std::uint32_t i = 0;; ++i) {
      if (state->fired.load(std::memory_order_acquire)) return true;
      if ((i & 1023u) == 0 && std::chrono::steady_clock::now() > deadline) return false;
   }
}

class SingleCoreInterruptWake_Test : public ::testing::Test
{
protected:
   void SetUp() override
   {
      state = &s;
      kernel::initialise();
      time::initialise(tick_hz);
   }

   void TearDown() override
   {
      time::finalise();
      kernel::finalise();
      state = nullptr;
   }

   shared_state s;
};

}  // namespace

TEST_F(SingleCoreInterruptWake_Test, GivenAnInterruptThatWakesThreeThreads_WhenTheLastOvertakesTheFirst_ThenTheyRunMostUrgentFirstAndTheOvertakenKeepsTheFrontOfItsLevel)
{
   static test::guarded_stack x1_stack, x2_stack, b_stack, driver_stack;

   thread tx1([] { this_thread::suspend(); note(x1); }, x1_stack, thread::priority(10), core0);
   thread tx2([] { this_thread::suspend(); note(x2); }, x2_stack, thread::priority(10), core0);
   thread tb([] { this_thread::suspend(); note(b); }, b_stack, thread::priority(5), core0);
   s.thread_x1 = &tx1;
   s.thread_x2 = &tx2;
   s.thread_b = &tb;

   thread td([] {
         time::start();
         state->ran = run_and_wait([](void*) {
            state->thread_x1->resume();
            state->thread_x2->resume();
            state->thread_b->resume();
            state->fired.store(true, std::memory_order_release);
         });
         note(driver);
      },
      driver_stack, thread::priority(20), core0);

   kernel::start();

   EXPECT_TRUE(s.ran) << "the timer's callback never ran";
   EXPECT_EQ(s.notes, (std::vector<std::uint32_t>{b, x1, x2, driver}))
      << "x1, decided first and then overtaken by b, belongs at the front of its level, ahead of its equal x2";
}

TEST_F(SingleCoreInterruptWake_Test, GivenPreemptionDisabled_WhenAnInterruptWakesAMoreUrgentThread_ThenItRunsOnlyAtTheEnable)
{
   static test::guarded_stack b_stack, driver_stack;

   thread tb([] { this_thread::suspend(); note(b); }, b_stack, thread::priority(5), core0);
   s.thread_b = &tb;

   thread td([] {
         time::start();
         {
            this_core::preemption_guard const no_preemption;
            state->ran = run_and_wait([](void*) {
               state->thread_b->resume();
               state->fired.store(true, std::memory_order_release);
            });
            state->still_the_driver = this_thread::id() == state->driver_id;
            note(driver);
         }
      },
      driver_stack, thread::priority(20), core0);
   s.driver_id = td.get_id();

   kernel::start();

   EXPECT_TRUE(s.ran) << "the timer's callback never ran";
   EXPECT_TRUE(s.still_the_driver)
      << "the kernel counted b running inside the driver's preempt-disabled region: a switch was decided where none can be made";
   EXPECT_EQ(s.notes, (std::vector<std::uint32_t>{driver, b}))
      << "b ran inside the driver's preempt-disabled region";
}

TEST_F(SingleCoreInterruptWake_Test, GivenAThreadInsideACriticalSection_WhenItWakesAMoreUrgentThread_ThenItKeepsRunningUntilTheSectionEnds)
{
   static test::guarded_stack b_stack, driver_stack;

   thread tb([] { this_thread::suspend(); note(b); }, b_stack, thread::priority(5), core0);
   s.thread_b = &tb;

   thread td([] {
         {
            this_core::critical_guard const masked;
            state->thread_b->resume();
            state->still_the_driver = this_thread::id() == state->driver_id;
            note(driver);
         }
      },
      driver_stack, thread::priority(20), core0);
   s.driver_id = td.get_id();

   kernel::start();

   EXPECT_TRUE(s.still_the_driver)
      << "the kernel counted b running inside the driver's critical section";
   EXPECT_EQ(s.notes, (std::vector<std::uint32_t>{driver, b}))
      << "b ran inside the driver's critical section";
}

TEST_F(SingleCoreInterruptWake_Test, GivenAnInterruptThatOwesASwitch_WhenItAlsoAsksForAReschedule_ThenTheSwitchIsMadeAndTheRescheduleFollows)
{
   static test::guarded_stack b_stack, y_stack, driver_stack;

   thread ty([] { state->the_gate.pass(); note(y); }, y_stack, thread::priority(3), core0);
   thread tb([] { this_thread::suspend(); note(b); }, b_stack, thread::priority(5), core0);
   s.thread_b = &tb;

   thread td([] {
         time::start();
         state->ran = run_and_wait([](void*) {
            state->thread_b->resume();
            state->the_gate.open();
            state->fired.store(true, std::memory_order_release);
         });
         note(driver);
      },
      driver_stack, thread::priority(20), core0);

   kernel::start();

   EXPECT_TRUE(s.ran) << "the timer's callback never ran";
   EXPECT_EQ(s.notes, (std::vector<std::uint32_t>{y, b, driver}))
      << "y, readied by the gate's batch wake after b was decided, must run before b";
}
