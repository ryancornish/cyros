/**
 * @file test_linux_preempt_adopt.cpp
 * @brief The port must ADOPT the calling thread's signal mask, not assume it.
 *
 * Subject: cyros_port_init's adoption of the OS thread (L0)
 * Trusts:  the harness floor, declared as debt in test.toml
 *
 * The preempt port's central invariant is that the OS signal mask is a view of
 * its own depth counters, and `assert_mask_matches_depths()` checks it on every
 * masking call. That invariant is only TRUE if something establishes it once,
 * and until 2026-09-24 nothing did. It held by luck in two common cases and
 * failed in a third:
 *
 *   fresh process        signals unblocked, counters zero      agrees
 *   second run in-proc   counters left raised by the first run agrees
 *   INHERITED mask       signals blocked, counters zero        FAILS
 *
 * The third is not exotic. The kernel leaves both of its signals blocked on the
 * thread it borrowed as core 0, so ANY fork plus exec from a process that has
 * run cyros hands the child a mask with them blocked while its counters start
 * at zero. The child then panics on its first critical section, nowhere near
 * the cause. An application that blocks SIGURG or a realtime signal for its own
 * reasons hits exactly the same wall without forking at all.
 *
 * This test reproduces the fork case, which is the one that was actually
 * observed: a gtest death test in a binary where any earlier test had run the
 * kernel. Doing it that way rather than by blocking named signals keeps the
 * test from having to know which signals the port chose.
 *
 * HOW IT WORKS. `threadsafe` death test style re-executes this binary for the
 * child, so the child runs this test body from the top with the parent's mask
 * inherited and its own counters fresh. That is the failing combination, and it
 * is reached before the statement below is ever evaluated: a regression shows
 * up as the child dying rather than exiting 0.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>

#include <common/guarded_stack.hpp>

#include <gtest/gtest.h>

#include <cstdlib>

using namespace cyros;

namespace
{

/* Borrowing the calling thread as core 0 is what dirties its mask, so a full
 * lifecycle is the smallest thing that sets up the next run to fail. */
void run_one_lifecycle()
{
   kernel::initialise();

   static test::guarded_stack stack;
   thread t([] {}, stack, thread::priority(0), core0);

   kernel::start();
   kernel::finalise();
}

void run_one_lifecycle_and_exit()
{
   run_one_lifecycle();
   std::exit(0);
}

}  // namespace

TEST(LinuxPreemptAdopt_Test, GivenAThreadThatAlreadyRanTheKernel_WhenAChildInheritsItsMask_ThenTheKernelStillRuns)
{
   GTEST_FLAG_SET(death_test_style, "threadsafe");

   // Leaves this thread with the port's signals blocked, which is what the
   // child is about to inherit.
   run_one_lifecycle();

   /* An assertion that the child SURVIVES, which is the unusual direction for
    * this macro and the reason it is EXPECT_EXIT rather than EXPECT_DEATH. A
    * plain in-process check could not be written: the failure is a panic, and
    * a panic takes the whole suite with it. */
   EXPECT_EXIT(run_one_lifecycle_and_exit(), ::testing::ExitedWithCode(0), "")
      << "a process that inherited the mask of a thread which had run the kernel "
         "could not run the kernel itself, so cyros_port_init is assuming the mask "
         "rather than adopting it";
}
