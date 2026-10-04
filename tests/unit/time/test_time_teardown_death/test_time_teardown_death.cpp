/**
 * @file test_time_teardown_death.cpp
 * @brief Time teardown refuses to run while the kernel is running.
 *
 * Subject: cyros_port_time_teardown() on linux_preempt (L1), reached through
 *          time::finalise()
 * Trusts:  kernel bring-up and threads (L2), as harness only, declared as debt
 *
 * time::finalise() documents that it runs after the kernel has stopped. Called
 * from a thread mid-run it deletes every core's timer under a live kernel, and
 * the port would then hand the timer signal back to the thread the kernel
 * borrowed, opening it on a running core. Before this check the first went
 * unnoticed and the second was a silent no-op. Mutation-verified by deleting
 * the check.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/time/time.hpp>

#include <common/death.hpp>
#include <common/guarded_stack.hpp>

#include <gtest/gtest.h>

using namespace cyros;

namespace
{

void finalise_time_from_a_running_thread()
{
   kernel::initialise();
   time::initialise(1'000);

   static test::guarded_stack stack;
   thread t([] {
      time::start();
      time::finalise();
   }, stack, thread::priority(0), core0);

   kernel::start();
}

}  // namespace

TEST(TimeTeardownDeath_Test, GivenARunningKernel_WhenTimeIsFinalisedFromAThread_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(finalise_time_from_a_running_thread(),
                      test::panicked_at("port_linux_preempt.cpp", "time::finalise() while the kernel runs"))
      << "time teardown ran under a live kernel without stopping it";
}
