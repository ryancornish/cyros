/**
 * @file test_chrono_death.cpp
 * @brief Arming an alarm wrongly stops the system, and says whose fault it is.
 *
 * Subject: chrono::alarm::arm_at (L8)
 * Trusts:  the periodic driver's timetable (L1). No kernel and no tick: both
 *          misuses die inside arm_at() before either would matter.
 *
 * alarm.cpp holds one check of each public kind, which makes it the natural
 * place to pin that both kinds actually fire:
 *
 *   CYROS_REQUIRE, the caller's bug
 *      re-arming a pending alarm. The timer it already holds would be
 *      orphaned, still scheduled against an alarm that has forgotten it.
 *   CYROS_FATAL, nobody's bug
 *      the driver's timer table is full, a limit sized by configuration.
 *
 * Both are mutation-verified by deleting the check.
 */

#include <cyros/chrono/alarm.hpp>
#include <cyros/time/time.hpp>

#include <common/death.hpp>

#include <gtest/gtest.h>

#include <array>

using namespace cyros;

namespace
{

constexpr std::uint32_t frequency = 1'000;

void arm_a_pending_alarm()
{
   time::initialise(frequency);
   static chrono::alarm a;
   a.arm_in(time::from_milliseconds(100));
   a.arm_in(time::from_milliseconds(100));   // still pending
}

/* One more alarm than the periodic driver has slots for. The driver's table is
 * a fixed 16, and nothing here fires, so every slot stays taken. */
void arm_more_alarms_than_the_driver_has_slots()
{
   time::initialise(frequency);
   static std::array<chrono::alarm, 17> alarms;
   for (auto& a : alarms) {
      a.arm_in(time::from_milliseconds(100));
   }
}

}  // namespace

TEST(ChronoDeath_Test, GivenAPendingAlarm_WhenArmedAgain_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(arm_a_pending_alarm(),
                      test::panicked_at("alarm.cpp", "CYROS_REQUIRE(!pending())"))
      << "re-arming orphaned the alarm's first timer instead of stopping";
}

TEST(ChronoDeath_Test, GivenAFullTimerTable_WhenAnAlarmIsArmed_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(arm_more_alarms_than_the_driver_has_slots(),
                      test::panicked_at("alarm.cpp", "Timer table full"))
      << "an alarm was armed with no timer behind it, so it could never fire";
}
