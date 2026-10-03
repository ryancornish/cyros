/**
 * @file test_chrono_event_flags_sim.cpp
 * @brief The timed event_flags methods, on the simulation driver.
 *
 * Same model as test_chrono_alarm_sim: virtual time moves only when a thread
 * advances it, the pumper is the least urgent thread, and a woken waiter runs
 * before time moves again, so every return tick is exact.
 */

#include <cyros/chrono/chrono.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/event_flags.hpp>
#include <cyros/time/time.hpp>
#include <cyros/time/simulation.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>

using namespace cyros;

namespace
{

constexpr auto frequency = 1'000u;

class ChronoEventFlagsSim_Test : public ::testing::Test
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

TEST_F(ChronoEventFlagsSim_Test, GivenNoMatchingSet_WhenTryWaitUntil_ThenItReturnsZeroAtTheDeadline)
{
   struct state
   {
      sync::event_flags f{0b100};
      std::uint32_t got{~0u};
      std::uint64_t at{0};
   } s;

   thread waiter(
      [&s]{
         s.got = s.f.try_wait_until(0b011, time::time_point{40});
         s.at = time::now().value;
      },
      s_a, thread::priority(1));

   thread pumper([]{ pump_until(time::time_point{80}); }, s_pump, thread::priority(20));

   kernel::start();

   EXPECT_EQ(s.got, 0u);
   EXPECT_EQ(s.at, 40u);
}

TEST_F(ChronoEventFlagsSim_Test, GivenASetBeforeTheDeadline_WhenTryWaitFor_ThenItReturnsTheBitsAtTheSet)
{
   struct state
   {
      sync::event_flags f{0};
      std::uint32_t got{0};
      std::uint64_t at{0};
   } s;

   thread waiter(
      [&s]{
         s.got = s.f.try_wait_for(0b011, time::duration{60}, flags_match::all, flags_exit::consume);
         s.at = time::now().value;
      },
      s_a, thread::priority(1));

   thread setter(
      [&s]{
         pump_until(time::time_point{10});
         s.f.set(0b001);
         pump_until(time::time_point{15});
         s.f.set(0b110);
      },
      s_b, thread::priority(10));

   thread pumper([]{ pump_until(time::time_point{100}); }, s_pump, thread::priority(20));

   kernel::start();

   EXPECT_EQ(s.got, 0b011u);
   EXPECT_EQ(s.at, 15u) << "an ALL wait returned before its last bit";
   EXPECT_EQ(s.f.peek(), 0b100u) << "consume cleared bits outside the ones it matched";
}

/* The alarm and a more urgent setter both fire at tick 30. The setter sets
 * before the waiter runs, and the bits are polled before the alarm, so the
 * wait must report them rather than a timeout. */
TEST_F(ChronoEventFlagsSim_Test, GivenASetOnTheDeadlineTick_WhenTheWaiterRuns_ThenTheBitsWin)
{
   struct state
   {
      sync::event_flags f{0};
      std::uint32_t got{0};
   } s;

   thread waiter([&s]{ s.got = s.f.try_wait_until(0b1, time::time_point{30}); }, s_a, thread::priority(5));

   thread setter(
      [&s]{
         this_thread::sleep_until(time::time_point{30});
         s.f.set(0b1);
      },
      s_b, thread::priority(1));

   thread pumper([]{ pump_until(time::time_point{60}); }, s_pump, thread::priority(20));

   kernel::start();

   EXPECT_EQ(s.got, 0b1u);
}

TEST_F(ChronoEventFlagsSim_Test, GivenAnExpiredDeadline_WhenTryWaitUntil_ThenItDegradesToTryWait)
{
   struct state
   {
      sync::event_flags f{0b10};
      std::uint32_t hit{0};
      std::uint32_t miss{~0u};
      std::uint32_t zero{~0u};
   } s;

   thread caller(
      [&s]{
         time::simulation::advance_to(time::time_point{20});
         s.hit  = s.f.try_wait_until(0b10, time::time_point{5});
         s.miss = s.f.try_wait_until(0b01, time::time_point{5});
         s.zero = s.f.try_wait_for(0b01, time::duration{0});
      },
      s_a, thread::priority(1));

   // No pumper: a wait that blocked here would hang the suite.
   kernel::start();

   EXPECT_EQ(s.hit, 0b10u);
   EXPECT_EQ(s.miss, 0u);
   EXPECT_EQ(s.zero, 0u);
}
