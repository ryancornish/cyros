/**
 * @file test_chrono_alarm_preempt.cpp
 * @brief The alarm's real-ISR path: preempt port, periodic driver.
 *
 * The sim suite proves the semantics at exact ticks. This suite proves the
 * mechanism under a real asynchronous timer: the callback runs in a timer ISR
 * that interrupted whatever was running, and the wake it issues crosses back
 * into thread context through the ISR-safe wake path. Assertions are order and
 * at-or-after bounds only. Wall-clock upper bounds are deliberately absent, a
 * sleep that never wakes fails as a suite hang, not as a flaky margin.
 *
 * time::start() enables the CALLING core's tick and schedule_at files into the
 * calling core's timetable, so the most urgent thread starts time before
 * anything arms. Single core, so there is exactly one timetable in play.
 *
 * UNITS, and this has already produced vacuous assertions here. time::now()
 * counts PORT TICKS, and a tick is one microsecond on both linux ports, not one
 * millisecond and not the frequency passed to time::initialise() (that value
 * configures the tick, while from_milliseconds() converts against
 * cyros_port_time_freq_hz()). So a tick delta must be compared against
 * from_milliseconds(n).value, never against a bare n: three assertions below
 * read "at least 10" after a 10 MILLISECOND wait and were satisfied by 10
 * microseconds, i.e. they could not fail. Fixed 2026-09-18.
 */

#include <cyros/chrono/alarm.hpp>
#include <cyros/chrono/chrono.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/waitable.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/time/time.hpp>

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstddef>

using namespace cyros;

namespace
{

constexpr auto frequency = 1'000u; // 1 kHz tick, millisecond granularity

class ChronoAlarmPreempt_Test : public ::testing::Test
{
protected:
   void SetUp() override
   {
      kernel::initialise();
      time::initialise(frequency);
   }

   void TearDown() override
   {
      time::finalise();
      kernel::finalise();
   }
};

alignas(16) std::array<std::byte, 128 * 1024> s_a;
alignas(16) std::array<std::byte, 128 * 1024> s_b;

} // namespace

TEST_F(ChronoAlarmPreempt_Test, GivenBlockedWaiter_WhenTheAlarmFiresInTheTimerIsr_ThenItWakesAtOrAfterTheDeadline)
{
   uint64_t deadline = 0;
   uint64_t woke_at = 0;
   bool was_expired = false;

   thread waiter(
      [&]{
         time::start();
         chrono::alarm a;
         a.arm_in(time::from_milliseconds(10));
         deadline = (time::now() + time::from_milliseconds(10)).value;

         this_thread::wait_on(a);

         woke_at = time::now().value;
         was_expired = a.expired();
      },
      s_a, thread::priority(1)
   );

   kernel::start();

   EXPECT_TRUE(was_expired);
   EXPECT_GE(woke_at + 1, deadline); // +1 absorbs the arm-vs-deadline read race
}

TEST_F(ChronoAlarmPreempt_Test, GivenSleepFor_WhenRealTimeElapses_ThenAtLeastTheDurationPassed)
{
   uint64_t before = 0;
   uint64_t after = 0;

   thread sleeper(
      [&]{
         time::start();
         before = time::now().value;
         this_thread::sleep_for(time::from_milliseconds(20));
         after = time::now().value;
      },
      s_a, thread::priority(1)
   );

   kernel::start();

   EXPECT_GE(after - before, time::from_milliseconds(20).value);
}

TEST_F(ChronoAlarmPreempt_Test, GivenTimedWait_WhenAnotherThreadReleasesFirst_ThenTheSemaphoreIndexWins)
{
   sync::semaphore sem(0);
   std::size_t index = 999;

   thread waiter(
      [&]{
         time::start();
         chrono::alarm deadline;
         deadline.arm_in(time::from_milliseconds(500));
         index = this_thread::wait_on_any(sem, deadline);
         deadline.disarm();
      },
      s_a, thread::priority(1)
   );

   thread releaser(
      [&]{
         this_thread::sleep_for(time::from_milliseconds(5));
         sem.release();
      },
      s_b, thread::priority(2)
   );

   kernel::start();

   EXPECT_EQ(index, 0u); // the semaphore, well inside the deadline
}

TEST_F(ChronoAlarmPreempt_Test, GivenTimedWait_WhenNothingReleases_ThenTheAlarmIndexWins)
{
   sync::semaphore sem(0);
   std::size_t index = 999;

   thread waiter(
      [&]{
         time::start();
         chrono::alarm deadline;
         deadline.arm_in(time::from_milliseconds(10));
         index = this_thread::wait_on_any(sem, deadline);
      },
      s_a, thread::priority(1)
   );

   kernel::start();

   EXPECT_EQ(index, 1u); // the deadline
}

TEST_F(ChronoAlarmPreempt_Test, GivenABusySpinner_WhenTheAlarmFires_ThenTheWokenSleeperPreemptsIt)
{
   // The ISR wake must not just ready the sleeper, it must get it RUNNING
   // ahead of a lower-urgency thread that never yields. This is the preempt
   // port earning its name on the alarm path.
   std::atomic<bool> woke{false};
   std::atomic<bool> spinner_saw_wake{false};

   thread sleeper(
      [&]{
         time::start();
         this_thread::sleep_for(time::from_milliseconds(10));
         woke.store(true, std::memory_order_release);
      },
      s_a, thread::priority(1)
   );

   thread spinner(
      [&]{
         auto const spin_until = time::now() + time::from_milliseconds(200);
         while (time::now() < spin_until) {
            if (woke.load(std::memory_order_acquire)) {
               spinner_saw_wake.store(true, std::memory_order_relaxed);
               break;
            }
         }
      },
      s_b, thread::priority(5)
   );

   kernel::start();

   EXPECT_TRUE(woke.load());
   EXPECT_TRUE(spinner_saw_wake.load()); // observed mid-spin, so the wake preempted
}

TEST_F(ChronoAlarmPreempt_Test, GivenTimedAcquire_WhenNothingReleases_ThenItTimesOutUnderTheRealTimer)
{
   sync::semaphore sem(0);
   bool got = true;
   uint64_t before = 0;
   uint64_t after = 0;

   thread waiter(
      [&]{
         time::start();
         before = time::now().value;
         got = sem.try_acquire_for(time::from_milliseconds(10));
         after = time::now().value;
      },
      s_a, thread::priority(1)
   );

   kernel::start();

   EXPECT_FALSE(got);
   EXPECT_GE(after - before, time::from_milliseconds(10).value);
}

/* ============================================================================
 * The timed acquire's non-timeout paths
 *
 * Only the expiry path above had a consumer, which left the primary contract
 * (a token arriving before the deadline) and both degenerate-deadline fast
 * paths untested. Those fast paths exist so that "already too late" does not
 * stretch into "up to one tick late", so the assertion that matters is that
 * they do not block, not merely that they return the right bool.
 * ========================================================================= */

TEST_F(ChronoAlarmPreempt_Test, GivenTimedAcquire_WhenAReleaseArrivesBeforeTheDeadline_ThenItSucceedsEarly)
{
   sync::semaphore sem(0);
   bool got = false;
   bool took_a_second = true;
   uint64_t before = 0;
   uint64_t after = 0;

   thread waiter(
      [&]{
         time::start();
         before = time::now().value;
         got = sem.try_acquire_for(time::from_milliseconds(500));
         after = time::now().value;
         // Exactly one token existed, so the deadline path must not have
         // granted a second one.
         took_a_second = sem.try_acquire();
      },
      s_a, thread::priority(1)
   );

   thread releaser(
      [&]{
         this_thread::sleep_for(time::from_milliseconds(5));
         sem.release();
      },
      s_b, thread::priority(2)
   );

   kernel::start();

   EXPECT_TRUE(got)            << "a token released well inside the deadline was not taken";
   EXPECT_FALSE(took_a_second) << "one release satisfied two acquires";
   EXPECT_LT(after - before, time::from_milliseconds(400).value)
      << "returned at the deadline rather than on the release";
}

TEST_F(ChronoAlarmPreempt_Test, GivenADeadlineAtNow_WhenTimedAcquiring_ThenItDegradesToTheNonBlockingTake)
{
   sync::semaphore empty(0);
   sync::semaphore stocked(1);
   bool got_empty = true;
   bool got_stocked = false;
   uint64_t elapsed = ~uint64_t{0};

   thread waiter(
      [&]{
         time::start();
         auto const before = time::now().value;

         // now() is not < now(), so both calls take the degenerate path.
         got_empty = empty.try_acquire_until(time::now());
         got_stocked = stocked.try_acquire_until(time::now());

         elapsed = time::now().value - before;
      },
      s_a, thread::priority(1)
   );

   kernel::start();

   EXPECT_FALSE(got_empty)  << "an expired deadline granted a token that did not exist";
   EXPECT_TRUE(got_stocked) << "an expired deadline refused an available token";
   EXPECT_LT(elapsed, time::from_milliseconds(1).value)
      << "the expired-deadline path blocked instead of degrading";
}

TEST_F(ChronoAlarmPreempt_Test, GivenAZeroDuration_WhenTimedAcquiring_ThenItDegradesToTheNonBlockingTake)
{
   sync::semaphore empty(0);
   sync::semaphore stocked(1);
   bool got_empty = true;
   bool got_stocked = false;
   uint64_t elapsed = ~uint64_t{0};

   thread waiter(
      [&]{
         time::start();
         auto const before = time::now().value;

         got_empty = empty.try_acquire_for(time::duration{0});
         got_stocked = stocked.try_acquire_for(time::duration{0});

         elapsed = time::now().value - before;
      },
      s_a, thread::priority(1)
   );

   kernel::start();

   EXPECT_FALSE(got_empty);
   EXPECT_TRUE(got_stocked);
   EXPECT_LT(elapsed, time::from_milliseconds(1).value)
      << "a zero duration blocked instead of degrading";
}
