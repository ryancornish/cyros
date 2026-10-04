/**
 * @file test_time_teardown_preempt.cpp
 * @brief time::finalise() must stop and delete every core's timer.
 *
 * Subject: cyros_port_time_teardown() on linux_preempt (L1), reached through
 *          the periodic driver's time::finalise()
 * Trusts:  kernel bring-up and threads (L2), as harness only, declared as debt
 *
 * Until 2026-09-24 the time port contract had no teardown. time::start()
 * creates a POSIX timer per core and nothing ever deleted one, so every timer
 * outlived the kernel run that made it. There are two kinds, and this test
 * reproduces both:
 *
 *   core 0     targets the calling thread, which survives the run, and KEEPS
 *              FIRING at it. The signal only sits harmlessly pending because
 *              the port keeps it blocked on that thread until teardown.
 *   cores 1-3  target threads that have exited. They deliver nothing, but stay
 *              alive as kernel objects for the rest of the process.
 *
 * Nobody calls time::stop() here, on purpose. Most kernel tests in this suite
 * do not either, so a tick still running when kernel::start() returns is the
 * normal case, not a contrived one. kernel::finalise() runs BEFORE
 * time::finalise() for the same reason: teardown must not depend on anything
 * the kernel has already released.
 *
 * It also requires every signal's disposition, and the thread's signal mask,
 * to be back as it found them once both finalises have run, so neither the
 * reschedule handler nor the timer handler outlives the run that installed it,
 * and the thread the kernel borrowed is not left with signals blocked. The mask
 * is checked in BOTH finalise orders, because the timer signal must stay
 * blocked from kernel::start() returning until time teardown, whichever
 * finalise the application calls first.
 *
 * The timer count comes from /proc/self/timers, because from inside the
 * process a deleted timer and a disarmed one look the same. It is checked once
 * while the ticks are live, so a probe that cannot see timers fails there
 * instead of passing the zero check for free. The pending check gets the same
 * treatment: a tick must be seen queued BEFORE finalise, or "nothing pending
 * after" would prove nothing about the drain.
 */

#include <cyros/config/config.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/time/time.hpp>

#include <common/guarded_stack.hpp>
#include <common/posix_timers.hpp>

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

using namespace cyros;
using namespace std::chrono_literals;

namespace
{

constexpr std::uint32_t tick_hz = 1'000u;   // 1 kHz, so the wait below spans 20 ticks

/// @brief Every signal whose handler differs from `before`, as a readable list.
std::string changed_dispositions(std::array<struct sigaction, NSIG> const& before)
{
   std::string numbers;
   for (int signo = 1; signo < NSIG; ++signo) {
      struct sigaction now;
      sigaction(signo, nullptr, &now);
      if (now.sa_sigaction != before[static_cast<std::size_t>(signo)].sa_sigaction) {
         numbers += std::to_string(signo) + " ";
      }
   }
   return numbers;
}

/// @brief Every signal blocked on the calling thread, as a readable list.
std::string blocked_signals()
{
   sigset_t mask;
   pthread_sigmask(SIG_BLOCK, nullptr, &mask);

   std::string numbers;
   for (int signo = 1; signo < NSIG; ++signo) {
      if (sigismember(&mask, signo) == 1) {
         numbers += std::to_string(signo) + " ";
      }
   }
   return numbers;
}

/// @brief Every signal pending on the calling thread, as a readable list.
std::string pending_signals()
{
   sigset_t pending;
   sigpending(&pending);

   std::string numbers;
   for (int signo = 1; signo <= SIGRTMAX; ++signo) {
      if (sigismember(&pending, signo) == 1) {
         numbers += std::to_string(signo) + " ";
      }
   }
   return numbers;
}

}  // namespace

TEST(TimeTeardownPreempt_Test, GivenEveryCoreLeftItsTickRunning_WhenTimeIsFinalised_ThenNoTimerSurvivesAndNothingIsPending)
{
   int const before = test::live_posix_timers();
   if (before < 0) GTEST_SKIP() << "/proc/self/timers is unreadable on this kernel";

   std::array<struct sigaction, NSIG> dispositions_before{};
   for (int signo = 1; signo < NSIG; ++signo) {
      sigaction(signo, nullptr, &dispositions_before[static_cast<std::size_t>(signo)]);
   }
   std::string const mask_before = blocked_signals();

   kernel::initialise();
   time::initialise(tick_hz);

   static std::array<test::guarded_stack, config::cores> stacks;
   thread t0([] { time::start(); }, stacks[0], thread::priority(0), core0);
   thread t1([] { time::start(); }, stacks[1], thread::priority(0), core1);
   thread t2([] { time::start(); }, stacks[2], thread::priority(0), core2);
   thread t3([] { time::start(); }, stacks[3], thread::priority(0), core3);

   kernel::start();

   EXPECT_EQ(test::live_posix_timers(), before + static_cast<int>(config::cores))
      << "every core should own a live timer here, or the checks below prove nothing";

   // Let core 0's tick run on for a few periods first. The port keeps the
   // timer signal blocked on this thread until teardown, so a tick QUEUES here,
   // and that queued tick is what teardown's drain exists for. Finalising
   // straight away usually beats the next tick and leaves the drain untested:
   // measured on 2026-09-24, removing the drain failed this test 1 run in 40
   // without the wait. The check makes the precondition explicit rather than
   // hoped for.
   std::this_thread::sleep_for(5ms);
   EXPECT_NE(pending_signals(), "")
      << "no tick queued on this thread: either the timer is not running, and the "
         "drain in teardown is not being tested, or the port unblocked the timer "
         "signal while its ticks were live, delivering them to a stopped kernel";

   kernel::finalise();
   time::finalise();

   EXPECT_EQ(test::live_posix_timers(), before)
      << "time::finalise() left POSIX timers alive";

   // Twenty tick periods. A timer that was only disarmed would fire again in
   // here, and a signal queued before its timer was deleted would still show.
   std::this_thread::sleep_for(20ms);
   EXPECT_EQ(pending_signals(), "")
      << "signals still pending on the thread the kernel borrowed as core 0";

   // And the handlers cyros installed are gone: the reschedule signal's by the
   // core port as the cores stopped, the timer signal's by time teardown.
   EXPECT_EQ(changed_dispositions(dispositions_before), "")
      << "signals whose disposition is still the one cyros installed";

   EXPECT_EQ(blocked_signals(), mask_before)
      << "the thread the kernel borrowed came back with a different signal mask";
}

TEST(TimeTeardownPreempt_Test, GivenCore0LeftItsTickRunning_WhenTimeIsFinalisedBeforeTheKernel_ThenTheMaskIsHandedBackOnlyAtTeardown)
{
   /* The other order. The tick must still be held off between kernel::start()
    * returning and time::finalise(), and the mask must still come back whole,
    * now from teardown rather than after the kernel.
    *
    * In a child, starting from an EMPTY mask set explicitly. A snapshot would
    * not do: the child inherits its parent's mask across exec, and the test
    * above has already run a kernel on the parent's thread, so a regression
    * that leaves signals blocked would leave them in "before" too and pass.
    * Measured: it did, until this was explicit. A fresh child has nothing
    * pending, so unblocking delivers nothing. */
   GTEST_FLAG_SET(death_test_style, "threadsafe");

   EXPECT_EXIT(
      {
         sigset_t none;
         sigemptyset(&none);
         pthread_sigmask(SIG_SETMASK, &none, nullptr);
         std::string const mask_before = blocked_signals();

         kernel::initialise();
         time::initialise(tick_hz);

         static test::guarded_stack stack;
         thread t0([] { time::start(); }, stack, thread::priority(0), core0);

         kernel::start();

         std::this_thread::sleep_for(5ms);
         if (pending_signals().empty()) {
            std::fprintf(stderr, "no tick held pending after the kernel returned, so "
                                 "the timer is not running or its signal was unblocked "
                                 "while live\n");
            std::exit(2);
         }

         time::finalise();
         if (blocked_signals() != mask_before) {
            std::fprintf(stderr, "after time teardown: [%s], before: [%s]\n",
                         blocked_signals().c_str(), mask_before.c_str());
            std::exit(3);
         }

         kernel::finalise();
         if (blocked_signals() != mask_before) {
            std::fprintf(stderr, "after kernel::finalise: [%s], before: [%s]\n",
                         blocked_signals().c_str(), mask_before.c_str());
            std::exit(4);
         }
         std::exit(pending_signals().empty() ? 0 : 5);
      },
      ::testing::ExitedWithCode(0), "");
}
