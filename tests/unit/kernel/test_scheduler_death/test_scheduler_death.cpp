/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_scheduler_death.cpp
 * @brief A thread that yields or blocks off baseline stops the system.
 *
 * Subject: the precondition of the thread's road (L3): a yield and a block
 *          switch on the calling thread, so it must be at baseline, outside
 *          any critical section and with preemption enabled
 * Trusts:  the kernel harness (L2) and this_core's guards (L1)
 *
 * The thread's road decides the switch and hands it to the port under the
 * kernel's mask, trusting that releasing that mask returns the core to
 * baseline. Off baseline that is false, and on Cortex-M the kernel would then
 * count the next thread as running while this one runs on, until the region
 * ends: corrupt books, not a crash. So the check is a CYROS_REQUIRE, kept in
 * every build that keeps preconditions.
 *
 * A block is reached through this_thread::suspend, the one wait thread.hpp
 * offers, so nothing above L2 is needed to make one.
 *
 * Every test requires the panic's file and failing line as well as SIGABRT
 * (`common/death.hpp`).
 */

#include <cyros/kernel/core.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>

#include <common/death.hpp>
#include <common/guarded_stack.hpp>

#include <gtest/gtest.h>

using namespace cyros;

namespace
{

/* Each misuse runs on a thread of its own, in the death-test child. */
template<typename Misuse>
void run_on_a_thread(Misuse misuse)
{
   kernel::initialise();
   static test::guarded_stack stack;
   thread t(misuse, stack, thread::priority(1), core0);
   kernel::start();
   kernel::finalise();
}

void yield_in_a_critical_section()
{
   run_on_a_thread([] {
      this_core::critical_guard guard;
      this_thread::yield();
   });
}

void yield_with_preemption_disabled()
{
   run_on_a_thread([] {
      this_core::preemption_guard guard;
      this_thread::yield();
   });
}

void block_in_a_critical_section()
{
   run_on_a_thread([] {
      this_core::critical_guard guard;
      this_thread::suspend();
   });
}

void block_with_preemption_disabled()
{
   run_on_a_thread([] {
      this_core::preemption_guard guard;
      this_thread::suspend();
   });
}

}  // namespace

TEST(SchedulerDeath_Test, GivenACriticalSection_WhenTheThreadYields_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(yield_in_a_critical_section(),
                      test::panicked_at("scheduler.cpp", "yield() off baseline"));
}

TEST(SchedulerDeath_Test, GivenPreemptionDisabled_WhenTheThreadYields_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(yield_with_preemption_disabled(),
                      test::panicked_at("scheduler.cpp", "yield() off baseline"));
}

TEST(SchedulerDeath_Test, GivenACriticalSection_WhenTheThreadBlocks_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(block_in_a_critical_section(),
                      test::panicked_at("scheduler.cpp", "Blocked off baseline"));
}

TEST(SchedulerDeath_Test, GivenPreemptionDisabled_WhenTheThreadBlocks_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(block_with_preemption_disabled(),
                      test::panicked_at("scheduler.cpp", "Blocked off baseline"));
}
