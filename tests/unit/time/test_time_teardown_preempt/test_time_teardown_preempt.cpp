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
 *              the kernel also leaves that thread with its signals blocked.
 *   cores 1-3  target threads that have exited. They deliver nothing, but stay
 *              alive as kernel objects for the rest of the process.
 *
 * Nobody calls time::stop() here, on purpose. Most kernel tests in this suite
 * do not either, so a tick still running when kernel::start() returns is the
 * normal case, not a contrived one. kernel::finalise() runs BEFORE
 * time::finalise() for the same reason: teardown must not depend on anything
 * the kernel has already released.
 *
 * It also requires every signal's disposition to be back as it found it once
 * both finalises have run, so neither the reschedule handler nor the timer
 * handler outlives the run that installed it.
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

   // Let core 0's tick run on for a few periods first. The kernel hands this
   // thread back with its signals blocked, so a tick QUEUES here, and that
   // queued tick is what teardown's drain exists for. Finalising straight away
   // usually beats the next tick and leaves the drain untested: measured on
   // 2026-09-24, removing the drain failed this test 1 run in 40 without the
   // wait. The check makes the precondition explicit rather than hoped for.
   std::this_thread::sleep_for(5ms);
   EXPECT_NE(pending_signals(), "")
      << "no tick queued on this thread, so the drain in teardown is not being tested";

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
}
