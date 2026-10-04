/**
 * @file test_linux_preempt_return.cpp
 * @brief The thread the port borrowed as core 0 must not be handed back broken.
 *
 * Subject: what cyros_port_start_cores leaves on the calling OS thread (L0)
 * Trusts:  the harness floor, declared as debt in test.toml
 *
 * The preempt port runs core 0 on the caller's own thread, and gives each core
 * an alternate signal stack of its own for the reschedule and timer handlers.
 * sigaltstack is per THREAD and stays registered until changed. Until
 * 2026-09-24 the port unmapped that memory at shutdown and left it registered,
 * so the caller got its thread back with an alternate signal stack pointing at
 * nothing. Measured on the Arch box: any SA_ONSTACK handler the application
 * then ran on that thread, for a signal cyros never touches, killed the process
 * with SIGSEGV. Crash reporters and stack-overflow handlers use SA_ONSTACK, so
 * this is the ordinary case, not an exotic one.
 *
 * The same night showed the alternate stack was one of three leftovers. The
 * sigctx interceptor's per-thread config still named the unmapped handler
 * stack, and the reschedule signal's process-wide disposition was still the
 * interceptor, so the first such signal delivered after unblocking it died too.
 * Since 2026-09-25 the port calls sigctx_intercept_uninstall() before freeing
 * the stacks, which undoes all three.
 *
 * The last leftover was the signal MASK: the thread came back with both of the
 * port's signals blocked. It now comes back with the mask it had, signal by
 * signal, including one it had blocked itself, and taking a critical section
 * afterwards must not disturb that. With no time driver in this test, nothing
 * holds the timer signal open, so both go back as the cores stop. The case
 * where a live tick keeps the timer signal blocked until time teardown is
 * test_time_teardown_preempt's.
 *
 * Checks come in pairs, the state and then its consequence, and every
 * consequence runs in a child so the failure it guards against cannot take the
 * suite with it.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/port/port.h>

#include <common/guarded_stack.hpp>

#include <gtest/gtest.h>

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

using namespace cyros;

namespace
{

void run_one_lifecycle()
{
   kernel::initialise();

   static test::guarded_stack stack;
   thread t([] {}, stack, thread::priority(0), core0);

   kernel::start();
   kernel::finalise();
}

volatile sig_atomic_t handled = 0;

/* Every signal's disposition, so a lifetime can be shown to leave them all as it
 * found them without the test naming the signals the port chose. */
struct dispositions
{
   struct sigaction of[NSIG];

   static dispositions now()
   {
      dispositions d{};
      for (int signo = 1; signo < NSIG; ++signo) {
         sigaction(signo, nullptr, &d.of[signo]);
      }
      return d;
   }
};

/* The calling thread's blocked signals, as a list, so a mismatch names them
 * without the test naming the signals the port chose. */
std::string blocked_signals()
{
   sigset_t mask;
   pthread_sigmask(SIG_BLOCK, nullptr, &mask);

   std::string numbers;
   for (int signo = 1; signo < NSIG; ++signo) {
      if (sigismember(&mask, signo) == 1) numbers += std::to_string(signo) + " ";
   }
   return numbers;
}

/* A lifetime, then a critical section of each grade on the same thread, as an
 * application might take after the kernel returns. A port that still derived
 * the mask from its depths for a signal it had handed back would re-block it
 * here, or trip its own mask assert. */
void run_one_lifecycle_then_mask_and_unmask()
{
   run_one_lifecycle();

   cyros_mask_token_t const irq = cyros_port_irq_save();
   cyros_port_irq_restore(irq);
   cyros_mask_token_t const preempt = cyros_port_preempt_disable();
   cyros_port_preempt_enable(preempt);
}

/* The starting mask is set explicitly, never snapshotted: a child inherits its
 * parent's mask across exec, so a snapshot would already carry whatever an
 * earlier run in the parent left blocked, and a regression would pass. */
void mask_survives_a_lifetime_and_exit(bool block_everything_first)
{
   sigset_t start;
   if (block_everything_first) sigfillset(&start);
   else                        sigemptyset(&start);
   pthread_sigmask(SIG_SETMASK, &start, nullptr);
   std::string const before = blocked_signals();

   run_one_lifecycle_then_mask_and_unmask();

   std::string const after = blocked_signals();
   if (after != before) {
      std::fprintf(stderr, "blocked before: [%s] after: [%s]\n", before.c_str(), after.c_str());
      std::exit(1);
   }
   std::exit(0);
}

/* The signal a socket raises for out-of-band data, and the one this port
 * borrows for rescheduling. An application that meets it after the kernel has
 * run must not crash. */
void unblock_and_raise_sigurg_and_exit()
{
   sigset_t urg;
   sigemptyset(&urg);
   sigaddset(&urg, SIGURG);
   pthread_sigmask(SIG_UNBLOCK, &urg, nullptr);
   raise(SIGURG);
   std::exit(0);
}

/* The application's own handler for a signal cyros never uses. */
void run_an_application_onstack_handler_and_exit()
{
   struct sigaction sa;
   std::memset(&sa, 0, sizeof(sa));
   sa.sa_handler = [](int) { handled = 1; };
   sa.sa_flags   = SA_ONSTACK;
   sigaction(SIGUSR1, &sa, nullptr);

   raise(SIGUSR1);
   std::exit(handled == 1 ? 0 : 1);
}

/* The state checks run in a child too, and not only for safety. A lifetime run
 * earlier in the same process leaves exactly the state under test behind, and a
 * "before" snapshot taken after it proves nothing. That is not hypothetical:
 * with the uninstall deleted, an in-process version of the disposition check
 * still passed, because an earlier test had already installed the handler. */
void alternate_stack_survives_a_lifetime_and_exit()
{
   stack_t before;
   sigaltstack(nullptr, &before);

   run_one_lifecycle();

   stack_t after;
   sigaltstack(nullptr, &after);
   bool const same = after.ss_flags == before.ss_flags
                  && ((before.ss_flags & SS_DISABLE) != 0
                      || (after.ss_sp == before.ss_sp && after.ss_size == before.ss_size));
   std::exit(same ? 0 : 1);
}

void dispositions_survive_a_lifetime_and_exit()
{
   dispositions const before = dispositions::now();

   run_one_lifecycle();

   dispositions const after = dispositions::now();
   for (int signo = 1; signo < NSIG; ++signo) {
      if (after.of[signo].sa_sigaction != before.of[signo].sa_sigaction) {
         std::fprintf(stderr, "signal %d still has the handler the kernel installed\n", signo);
         std::exit(1);
      }
   }
   std::exit(0);
}

}  // namespace

TEST(LinuxPreemptReturn_Test, GivenAThreadThatRanTheKernel_ThenItsAlternateSignalStackIsTheOneItHadBefore)
{
   GTEST_FLAG_SET(death_test_style, "threadsafe");
   EXPECT_EXIT(alternate_stack_survives_a_lifetime_and_exit(), ::testing::ExitedWithCode(0), "")
      << "the kernel left an alternate signal stack registered on its caller's thread";
}

TEST(LinuxPreemptReturn_Test, GivenAThreadThatRanTheKernel_ThenEverySignalDispositionIsTheOneItHadBefore)
{
   GTEST_FLAG_SET(death_test_style, "threadsafe");
   EXPECT_EXIT(dispositions_survive_a_lifetime_and_exit(), ::testing::ExitedWithCode(0), "");
}

TEST(LinuxPreemptReturn_Test, GivenAThreadThatRanTheKernel_WhenSigurgArrives_ThenNothingCrashes)
{
   GTEST_FLAG_SET(death_test_style, "threadsafe");

   EXPECT_EXIT((run_one_lifecycle(), unblock_and_raise_sigurg_and_exit()),
               ::testing::ExitedWithCode(0), "")
      << "a SIGURG after the kernel crashed, most likely in the interceptor on a "
         "handler stack the port had already unmapped";
}

TEST(LinuxPreemptReturn_Test, GivenAThreadThatRanTheKernel_WhenTheApplicationRunsAnOnstackHandler_ThenItRuns)
{
   GTEST_FLAG_SET(death_test_style, "threadsafe");

   // In the child, run a lifetime and then the application's handler. The
   // parent stays clean, so a regression kills only the child.
   EXPECT_EXIT((run_one_lifecycle(), run_an_application_onstack_handler_and_exit()),
               ::testing::ExitedWithCode(0), "")
      << "an SA_ONSTACK handler run after the kernel crashed, most likely on an "
         "alternate signal stack the port had already unmapped";
}

TEST(LinuxPreemptReturn_Test, GivenAThreadWithNothingBlocked_WhenItRunsTheKernel_ThenItsMaskIsTheOneItHadBefore)
{
   GTEST_FLAG_SET(death_test_style, "threadsafe");
   EXPECT_EXIT(mask_survives_a_lifetime_and_exit(false), ::testing::ExitedWithCode(0), "")
      << "the kernel handed its caller's thread back with a different signal mask";
}

TEST(LinuxPreemptReturn_Test, GivenAThreadWithEverythingBlocked_WhenItRunsTheKernel_ThenItsMaskIsTheOneItHadBefore)
{
   /* The other direction. A port that handed every signal back unblocked would
    * pass the case above and open signals here that the application chose to
    * keep closed. */
   GTEST_FLAG_SET(death_test_style, "threadsafe");
   EXPECT_EXIT(mask_survives_a_lifetime_and_exit(true), ::testing::ExitedWithCode(0), "")
      << "the kernel handed its caller's thread back with a different signal mask";
}
