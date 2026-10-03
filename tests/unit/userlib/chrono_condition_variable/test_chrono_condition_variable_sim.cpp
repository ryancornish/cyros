/**
 * @file test_chrono_condition_variable_sim.cpp
 * @brief The timed condition_variable methods, on the simulation driver.
 *
 * Same model as test_chrono_alarm_sim: virtual time moves only when a thread
 * advances it, the pumper is the least urgent thread, and a woken waiter runs
 * before time moves again, so every return tick is exact.
 *
 * What is asserted: a wait with no notify times out at exactly its deadline
 * and returns owning the mutex, the mutex is free while the wait is parked, a
 * notify before the deadline returns no_timeout at the notify's tick, a notify
 * landing on the deadline's own tick counts as a notify, an expired deadline
 * does not block, and the predicate forms return the predicate's final value.
 */

#include <cyros/chrono/chrono.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/condition_variable.hpp>
#include <cyros/sync/mutex.hpp>
#include <cyros/time/time.hpp>
#include <cyros/time/simulation.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>

using namespace cyros;

namespace
{

constexpr auto frequency = 1'000u; // 1 kHz, one tick per virtual millisecond

class ChronoConditionVariableSim_Test : public ::testing::Test
{
protected:
   void SetUp() override
   {
      kernel::initialise();
      time::initialise(frequency);
      time::simulation::set_mode(time::simulation::mode::virtual_time);
      time::start();
   }

   void TearDown() override
   {
      time::stop();
      time::finalise();
      kernel::finalise();
   }
};

void pump_until(time::time_point deadline)
{
   while (time::now() < deadline) {
      time::simulation::advance_by(time::duration{1});
      this_thread::yield();
   }
}

alignas(16) std::array<std::byte, 64 * 1024> s_a;
alignas(16) std::array<std::byte, 64 * 1024> s_b;
alignas(16) std::array<std::byte, 64 * 1024> s_pump;

} // namespace

TEST_F(ChronoConditionVariableSim_Test, GivenNoNotify_WhenWaitUntil_ThenItTimesOutAtTheDeadlineOwningTheMutex)
{
   struct state
   {
      sync::mutex m;
      sync::condition_variable cv;
      cv_status status = cv_status::no_timeout;
      std::uint64_t returned_at = 0;
      bool free_while_waiting = false;
   } s;

   thread waiter(
      [&s]{
         s.m.lock();
         s.status = s.cv.wait_until(s.m, time::time_point{40});
         s.returned_at = time::now().value;
         s.m.unlock(); // a hard error unless the timed wait handed the mutex back
      },
      s_a, thread::priority(1)
   );

   // Runs only while the waiter is parked, and must find the mutex free.
   thread prober(
      [&s]{
         pump_until(time::time_point{20});
         s.free_while_waiting = s.m.try_lock();
         if (s.free_while_waiting) s.m.unlock();
      },
      s_b, thread::priority(10)
   );

   thread pumper([]{ pump_until(time::time_point{80}); }, s_pump, thread::priority(20));

   kernel::start();

   EXPECT_EQ(s.status, cv_status::timeout);
   EXPECT_EQ(s.returned_at, 40u);
   EXPECT_TRUE(s.free_while_waiting) << "the mutex was held while its timed waiter was parked";
}

TEST_F(ChronoConditionVariableSim_Test, GivenANotifyBeforeTheDeadline_WhenWaitFor_ThenItReturnsNoTimeoutAtTheNotify)
{
   struct state
   {
      sync::mutex m;
      sync::condition_variable cv;
      cv_status status = cv_status::timeout;
      std::uint64_t returned_at = 0;
   } s;

   thread waiter(
      [&s]{
         s.m.lock();
         s.status = s.cv.wait_for(s.m, time::duration{60});
         s.returned_at = time::now().value;
         s.m.unlock();
      },
      s_a, thread::priority(1)
   );

   thread notifier(
      [&s]{
         pump_until(time::time_point{15});
         s.cv.notify_one();
      },
      s_b, thread::priority(10)
   );

   thread pumper([]{ pump_until(time::time_point{100}); }, s_pump, thread::priority(20));

   kernel::start();

   EXPECT_EQ(s.status, cv_status::no_timeout);
   EXPECT_EQ(s.returned_at, 15u);
}

/* The alarm fires and readies the waiter at tick 30, and a more urgent notifier
 * woken on the same tick notifies before the waiter runs. Both are true when
 * the waiter polls, and the notify must win: reporting timeout would swallow a
 * notify_one that was handed to this waiter and to no other. */
TEST_F(ChronoConditionVariableSim_Test, GivenANotifyOnTheDeadlineTick_WhenTheWaiterRuns_ThenTheNotifyWins)
{
   struct state
   {
      sync::mutex m;
      sync::condition_variable cv;
      cv_status status = cv_status::timeout;
   } s;

   thread waiter(
      [&s]{
         s.m.lock();
         s.status = s.cv.wait_until(s.m, time::time_point{30});
         s.m.unlock();
      },
      s_a, thread::priority(5)
   );

   thread notifier(
      [&s]{
         this_thread::sleep_until(time::time_point{30});
         s.cv.notify_one();
      },
      s_b, thread::priority(1)
   );

   thread pumper([]{ pump_until(time::time_point{60}); }, s_pump, thread::priority(20));

   kernel::start();

   EXPECT_EQ(s.status, cv_status::no_timeout);
}

TEST_F(ChronoConditionVariableSim_Test, GivenAnExpiredDeadline_WhenWaitUntil_ThenItTimesOutWithoutBlocking)
{
   struct state
   {
      sync::mutex m;
      sync::condition_variable cv;
      cv_status until = cv_status::no_timeout;
      cv_status zero  = cv_status::no_timeout;
   } s;

   thread caller(
      [&s]{
         time::simulation::advance_to(time::time_point{20});
         s.m.lock();
         s.until = s.cv.wait_until(s.m, time::time_point{5});
         s.zero  = s.cv.wait_for(s.m, time::duration{0});
         s.m.unlock();
      },
      s_a, thread::priority(1)
   );

   // No pumper: a wait that blocked here would hang the suite.
   kernel::start();

   EXPECT_EQ(s.until, cv_status::timeout);
   EXPECT_EQ(s.zero, cv_status::timeout);
}

TEST_F(ChronoConditionVariableSim_Test, GivenAPredicate_WhenTimedWaits_ThenTheyReturnItsFinalValue)
{
   struct state
   {
      sync::mutex m;
      sync::condition_variable cv;
      bool ready = false; // guarded by m
      bool met = false;
      bool never = true;
      std::uint64_t met_at = 0;
      std::uint64_t never_at = 0;
   } s;

   thread waiter(
      [&s]{
         s.m.lock();
         s.met = s.cv.wait_for(s.m, time::duration{50}, [&s]{ return s.ready; });
         s.met_at = time::now().value;
         s.never = s.cv.wait_until(s.m, time::time_point{70}, [&s]{ return !s.ready; });
         s.never_at = time::now().value;
         s.m.unlock();
      },
      s_a, thread::priority(1)
   );

   // A notify with the predicate still false must not end the wait early, so
   // the first notify at tick 5 is spurious from the predicate's point of view.
   thread notifier(
      [&s]{
         pump_until(time::time_point{5});
         s.cv.notify_one();
         pump_until(time::time_point{12});
         s.m.lock();
         s.ready = true;
         s.m.unlock();
         s.cv.notify_one();
      },
      s_b, thread::priority(10)
   );

   thread pumper([]{ pump_until(time::time_point{100}); }, s_pump, thread::priority(20));

   kernel::start();

   EXPECT_TRUE(s.met);
   EXPECT_EQ(s.met_at, 12u);
   EXPECT_FALSE(s.never);
   EXPECT_EQ(s.never_at, 70u);
}
