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
 * Checks come in pairs, the state and then its consequence, and every
 * consequence runs in a child so the failure it guards against cannot take the
 * suite with it. The mask is still handed back blocked (roadmap P3), so the
 * children unblock what they raise themselves.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>

#include <common/guarded_stack.hpp>

#include <gtest/gtest.h>

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

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
