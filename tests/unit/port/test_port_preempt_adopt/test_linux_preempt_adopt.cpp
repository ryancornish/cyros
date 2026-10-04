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
 * The third is not exotic. An application that blocks SIGURG or a realtime
 * signal for its own reasons hands the kernel exactly that, and so does any
 * fork plus exec from a process that has, since the child inherits the mask
 * while its counters start at zero. The child then panics on its first
 * critical section, nowhere near the cause. It was found through a gtest death
 * test, in a binary where an earlier test had run the kernel, back when the
 * kernel handed its caller's thread back with both of its signals blocked.
 * It no longer does (test_port_preempt_return), so this test blocks them
 * itself.
 *
 * HOW IT WORKS. The parent blocks EVERY signal, which keeps the test from
 * having to know which ones the port chose. `threadsafe` death test style
 * re-executes this binary for the child, so the child runs the statement below
 * with that mask inherited and its own counters fresh. That is the failing
 * combination: a regression shows up as the child dying rather than exiting 0.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>

#include <common/guarded_stack.hpp>

#include <gtest/gtest.h>

#include <csignal>
#include <cstdlib>
#include <pthread.h>

using namespace cyros;

namespace
{

/* The smallest thing that takes a critical section on the adopted thread. */
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

/// @brief Block every signal on this thread for the scope, then put the mask back.
struct everything_blocked
{
   sigset_t prior{};

   everything_blocked()
   {
      sigset_t all;
      sigfillset(&all);
      pthread_sigmask(SIG_BLOCK, &all, &prior);
   }
   ~everything_blocked() { pthread_sigmask(SIG_SETMASK, &prior, nullptr); }

   everything_blocked(everything_blocked const&)            = delete;
   everything_blocked& operator=(everything_blocked const&) = delete;
};

}  // namespace

TEST(LinuxPreemptAdopt_Test, GivenAThreadWithEverySignalBlocked_WhenAChildInheritsItsMask_ThenTheKernelStillRuns)
{
   GTEST_FLAG_SET(death_test_style, "threadsafe");

   // What the child is about to inherit.
   everything_blocked const blocked;

   /* An assertion that the child SURVIVES, which is the unusual direction for
    * this macro and the reason it is EXPECT_EXIT rather than EXPECT_DEATH. A
    * plain in-process check could not be written: the failure is a panic, and
    * a panic takes the whole suite with it. */
   EXPECT_EXIT(run_one_lifecycle_and_exit(), ::testing::ExitedWithCode(0), "")
      << "a process that inherited a mask with the port's signals blocked could "
         "not run the kernel, so cyros_port_init is assuming the mask rather than "
         "adopting it";
}
