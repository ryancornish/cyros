/**
 * @file test_tickless_driver.cpp
 * @brief Unit tests for the tickless (one-shot) time driver.
 *
 * The tickless driver implements the free-function API declared in
 * <cortos/time/time.hpp>. A test binary links exactly ONE time driver, so this
 * file targets the tickless implementation only (selected via test.toml
 * [components].time_driver = "tickless").
 *
 * Model
 * -----
 * Like the periodic driver, the tickless driver does not own time; now()
 * forwards to the Linux port counter. The difference is arming: the tickless
 * driver calls cortos_port_time_arm()/disarm() to request a one-shot interrupt
 * at the earliest pending deadline. The Linux port records the armed deadline
 * but, in unit tests, does not autonomously deliver IRQs -- we model an IRQ by
 * advancing port time and calling on_timer_isr() directly.
 *
 * Coverage goal: 100% branch coverage of the tickless translation unit, with
 * one documented exception: at the port's fixed 1 MHz frequency the duration
 * conversions are always exact, so the round-up arm of ceil_div_u64 is not
 * reachable from this suite (see the time::duration conversion section).
 *
 * Lifecycle note:
 *   The tickless driver keeps its state in a constinit global. The fixture
 *   pairs initialise()/finalise() per test; finalise() does `ds = driver_state{}`,
 *   which is what resets the slot table between tests. Earlier revisions of
 *   this file skipped initialise() to dodge a driver bug (initialise() not
 *   setting ds.initialised); that bug is fixed, and skipping initialise()
 *   would now leak scheduled slots from one test into the next.
 */

#include <cortos/time/time.hpp>
#include <cortos/port/port.h>

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <vector>

using namespace cortos;

// Linux-only deterministic test hook, implemented in the linux_boost port.
extern "C" void cortos_port_time_advance(uint64_t delta);

namespace
{

// Mirrors the tickless driver's internal MAX_SCHEDULED_CALLBACKS (currently
// 16). Not exported via a public header; update if the driver's limit changes.
constexpr uint32_t kMaxScheduledCallbacks = 16;

void counting_callback(void* arg) noexcept
{
   static_cast<std::atomic<int>*>(arg)->fetch_add(1, std::memory_order_relaxed);
}

class TicklessDriverTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      cortos_port_time_reset(0);

      // initialise() asserts it has not already been initialised, and
      // finalise() (run in TearDown) does `ds = driver_state{}`, fully resetting
      // the slot table and next_id. Pairing them per test is what keeps the
      // driver's constinit global state from leaking between tests -- without
      // it, slots scheduled by one test remain occupied for the next.
      cortos::time::initialise(1'000'000 /* Hz */);
   }

   void TearDown() override
   {
      // Stop the driver (disarms the one-shot) before finalising.
      // stop() is idempotent, so this is safe whether or not the test started.
      cortos::time::stop();
      cortos::time::finalise();
   }

   // Advance port time and deliver one timer ISR (models a one-shot firing).
   static void advance_and_pump(uint64_t delta_ticks)
   {
      cortos_port_time_advance(delta_ticks);
      cortos::time::on_timer_isr();
   }

   static void pump()
   {
      cortos::time::on_timer_isr();
   }
};

/* ============================================================================
 * Lifecycle
 * ========================================================================= */

// start() first call: started == false branch -> setup + initial rearm.
// start() second call: started == true branch  -> early return.
TEST_F(TicklessDriverTest, StartIsIdempotent)
{
   cortos::time::start();
   cortos::time::start();  // hits the `started` early-return branch
   SUCCEED();
}

// stop() not started: started == false branch -> early return.
// stop() started:     started == true branch  -> disarm + teardown.
TEST_F(TicklessDriverTest, StopIsIdempotent)
{
   cortos::time::stop();   // not started: early-return branch

   cortos::time::start();
   cortos::time::stop();   // started: teardown branch
   cortos::time::stop();   // stopped again: early-return branch
   SUCCEED();
}

// start() with no callbacks scheduled exercises rearm_locked()'s
// "earliest == UINT64_MAX" branch -> the driver disarms the one-shot.
TEST_F(TicklessDriverTest, StartWithNoCallbacksDisarms)
{
   cortos::time::start();  // rearm_locked() finds nothing -> disarm path
   SUCCEED();
}

// now() forwards the port counter unchanged.
TEST_F(TicklessDriverTest, NowReflectsPortCounter)
{
   cortos::time::start();
   EXPECT_EQ(cortos::time::now().value, 0u);

   cortos_port_time_advance(123);
   EXPECT_EQ(cortos::time::now().value, 123u);

   cortos_port_time_advance(7);
   EXPECT_EQ(cortos::time::now().value, 130u);
}

/* ============================================================================
 * schedule_at
 * ========================================================================= */

// Null callback: the `!cb` branch returns an invalid handle.
TEST_F(TicklessDriverTest, ScheduleNullCallbackReturnsInvalidHandle)
{
   cortos::time::start();
   EXPECT_EQ(cortos::time::schedule_at(time::time_point{100}, nullptr, nullptr).id, 0u);
}

// Valid callback: takes the first free slot, rearms, returns a valid handle.
TEST_F(TicklessDriverTest, ScheduleValidCallbackReturnsValidHandle)
{
   cortos::time::start();
   std::atomic<int> count{0};
   EXPECT_NE(cortos::time::schedule_at(time::time_point{100}, counting_callback, &count).id, 0u);
}

// Scheduling before start(): schedule_at() does not require the driver to be
// started; the slot is taken and rearm_locked() runs. A later start() will
// rearm again. This covers schedule_at()'s rearm call independent of start().
TEST_F(TicklessDriverTest, ScheduleBeforeStartStillSchedules)
{
   std::atomic<int> count{0};
   time::handle h = cortos::time::schedule_at(time::time_point{100}, counting_callback, &count);
   EXPECT_NE(h.id, 0u);

   cortos::time::start();
   advance_and_pump(100);
   EXPECT_EQ(count.load(), 1);
}

// Filling all slots then scheduling once more exercises the loop's
// "no free slot" exit -> invalid handle.
TEST_F(TicklessDriverTest, ScheduleBeyondCapacityReturnsInvalidHandle)
{
   cortos::time::start();

   std::atomic<int> count{0};
   std::vector<time::handle> handles;
   handles.reserve(kMaxScheduledCallbacks);

   for (uint32_t i = 0; i < kMaxScheduledCallbacks; ++i)
   {
      time::handle h = cortos::time::schedule_at(
         time::time_point{static_cast<uint64_t>(i) + 1000}, counting_callback, &count);
      ASSERT_NE(h.id, 0u) << "slot " << i << " should still be free";
      handles.push_back(h);
   }

   EXPECT_EQ(cortos::time::schedule_at(time::time_point{9999}, counting_callback, &count).id, 0u);

   // Freeing a slot makes room again (and rearms).
   ASSERT_TRUE(cortos::time::cancel(handles.front()));
   EXPECT_NE(cortos::time::schedule_at(time::time_point{9999}, counting_callback, &count).id, 0u);
}

/* ============================================================================
 * Firing semantics
 * ========================================================================= */

// Future deadline: fires once exactly when now >= deadline, not before.
TEST_F(TicklessDriverTest, CallbackFiresWhenDeadlineReached)
{
   cortos::time::start();

   std::atomic<int> count{0};
   ASSERT_NE(cortos::time::schedule_at(time::time_point{100}, counting_callback, &count).id, 0u);

   advance_and_pump(99);   // `when <= now` false -> no fire
   EXPECT_EQ(count.load(), 0);

   advance_and_pump(1);    // `when <= now` true  -> fires once
   EXPECT_EQ(count.load(), 1);

   advance_and_pump(1000); // slot freed -> no re-fire
   EXPECT_EQ(count.load(), 1);
}

// Deadline exactly equal to now fires (boundary of `when <= now`).
TEST_F(TicklessDriverTest, CallbackFiresAtExactDeadline)
{
   cortos::time::start();

   std::atomic<int> count{0};
   ASSERT_NE(cortos::time::schedule_at(time::time_point{10}, counting_callback, &count).id, 0u);

   advance_and_pump(10);
   EXPECT_EQ(count.load(), 1);
}

// Deadline already in the past at schedule time fires on the next ISR even
// with no time advance.
TEST_F(TicklessDriverTest, CallbackScheduledInPastFiresOnNextIsr)
{
   cortos::time::start();

   cortos_port_time_advance(100);
   ASSERT_EQ(cortos::time::now().value, 100u);

   std::atomic<int> count{0};
   ASSERT_NE(cortos::time::schedule_at(time::time_point{50}, counting_callback, &count).id, 0u);

   pump();  // deadline 50 <= now 100
   EXPECT_EQ(count.load(), 1);
}

// Distinct deadlines each fire once at their own time. As callbacks fire and
// free their slots, on_timer_isr() rearms to the next-earliest deadline; the
// final fire leaves nothing pending and rearm hits the disarm branch.
TEST_F(TicklessDriverTest, MultipleCallbacksFireInDeadlineOrder)
{
   cortos::time::start();

   std::atomic<int> a{0}, b{0}, c{0};
   ASSERT_NE(cortos::time::schedule_at(time::time_point{50},  counting_callback, &a).id, 0u);
   ASSERT_NE(cortos::time::schedule_at(time::time_point{100}, counting_callback, &b).id, 0u);
   ASSERT_NE(cortos::time::schedule_at(time::time_point{150}, counting_callback, &c).id, 0u);

   advance_and_pump(60);
   EXPECT_EQ(a.load(), 1);
   EXPECT_EQ(b.load(), 0);
   EXPECT_EQ(c.load(), 0);

   advance_and_pump(60);   // now = 120
   EXPECT_EQ(b.load(), 1);
   EXPECT_EQ(c.load(), 0);

   advance_and_pump(80);   // now = 200, last one fires, rearm -> disarm
   EXPECT_EQ(c.load(), 1);
}

// Several callbacks due on the same ISR all fire together.
TEST_F(TicklessDriverTest, AllDueCallbacksFireOnSameIsr)
{
   cortos::time::start();

   std::atomic<int> count{0};
   for (int i = 0; i < 5; ++i)
   {
      ASSERT_NE(cortos::time::schedule_at(time::time_point{100}, counting_callback, &count).id, 0u);
   }

   advance_and_pump(200);
   EXPECT_EQ(count.load(), 5);
}

// An ISR with nothing due: fire_due_isr visits an occupied-but-not-due slot
// (`when <= now` false) and also exercises the all-empty case.
TEST_F(TicklessDriverTest, IsrWithNothingDueFiresNothing)
{
   cortos::time::start();

   advance_and_pump(500);  // no callbacks at all

   std::atomic<int> count{0};
   ASSERT_NE(cortos::time::schedule_at(time::time_point{10'000}, counting_callback, &count).id, 0u);
   advance_and_pump(500);  // occupied slot, not due
   EXPECT_EQ(count.load(), 0);
}

/* ============================================================================
 * cancel
 * ========================================================================= */

// cancel() of an invalid handle: `h.id == 0` branch -> false.
TEST_F(TicklessDriverTest, CancelInvalidHandleReturnsFalse)
{
   cortos::time::start();
   EXPECT_FALSE(cortos::time::cancel(time::handle{0}));
}

// cancel() of an unknown non-zero id: loop finds no match -> false.
TEST_F(TicklessDriverTest, CancelUnknownHandleReturnsFalse)
{
   cortos::time::start();
   EXPECT_FALSE(cortos::time::cancel(time::handle{999999}));
}

// cancel() before firing: slot matched and cleared, driver rearms -> true.
TEST_F(TicklessDriverTest, CancelBeforeFiringPreventsCallback)
{
   cortos::time::start();

   std::atomic<int> count{0};
   time::handle h = cortos::time::schedule_at(time::time_point{100}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   EXPECT_TRUE(cortos::time::cancel(h));

   advance_and_pump(500);
   EXPECT_EQ(count.load(), 0);
}

// Cancelling the only pending callback makes rearm_locked() take its disarm
// branch (nothing left to arm).
TEST_F(TicklessDriverTest, CancelLastCallbackDisarms)
{
   cortos::time::start();

   std::atomic<int> count{0};
   time::handle h = cortos::time::schedule_at(time::time_point{100}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   EXPECT_TRUE(cortos::time::cancel(h));  // rearm_locked() -> disarm
   SUCCEED();
}

// cancel() after firing: slot already freed -> no match -> false.
TEST_F(TicklessDriverTest, CancelAfterFiringReturnsFalse)
{
   cortos::time::start();

   std::atomic<int> count{0};
   time::handle h = cortos::time::schedule_at(time::time_point{50}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   advance_and_pump(50);
   EXPECT_EQ(count.load(), 1);

   EXPECT_FALSE(cortos::time::cancel(h));
}

// Cancelling twice: second call finds no match.
TEST_F(TicklessDriverTest, CancelTwiceReturnsFalseSecondTime)
{
   cortos::time::start();

   std::atomic<int> count{0};
   time::handle h = cortos::time::schedule_at(time::time_point{100}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   EXPECT_TRUE(cortos::time::cancel(h));
   EXPECT_FALSE(cortos::time::cancel(h));
}

// Cancelling one of several leaves the others; the cancel rearms to the new
// earliest deadline.
TEST_F(TicklessDriverTest, CancelOneOfManyLeavesOthers)
{
   cortos::time::start();

   std::atomic<int> count{0};
   time::handle h1 = cortos::time::schedule_at(time::time_point{100}, counting_callback, &count);
   time::handle h2 = cortos::time::schedule_at(time::time_point{200}, counting_callback, &count);
   time::handle h3 = cortos::time::schedule_at(time::time_point{300}, counting_callback, &count);
   ASSERT_NE(h1.id, 0u);
   ASSERT_NE(h2.id, 0u);
   ASSERT_NE(h3.id, 0u);

   EXPECT_TRUE(cortos::time::cancel(h2));

   advance_and_pump(400);
   EXPECT_EQ(count.load(), 2);
}

/* ============================================================================
 * time::duration conversion
 *
 * The tickless driver converts using its frequency_hz field, set by
 * initialise() -- 1'000'000 Hz in this fixture (1 tick = 1 us).
 *
 * NOTE on the ceil_div round-up branch: at 1 MHz both conversions are exact
 * (ms -> ms*1000 ticks, us -> us*1 ticks), so the rounding-up arm of
 * ceil_div_u64 (remainder != 0) is NOT reachable from this suite. Exercising
 * it would require a sub-MHz frequency. The simulation driver suite, which
 * runs at 1 kHz, does cover the round-up arm. For the tickless TU this is an
 * accepted coverage exclusion on that one branch.
 * ========================================================================= */

TEST_F(TicklessDriverTest, FromMillisecondsConvertsExactly)
{
   // 10 ms at 1 MHz = 10'000 ticks.
   EXPECT_EQ(cortos::time::from_milliseconds(10).value, 10'000u);
}

TEST_F(TicklessDriverTest, FromMillisecondsZeroIsZero)
{
   // 0 ms: numerator is 0, ceil_div yields 0 (the a == 0 arm).
   EXPECT_EQ(cortos::time::from_milliseconds(0).value, 0u);
}

TEST_F(TicklessDriverTest, FromMicrosecondsConvertsExactly)
{
   // 500 us at 1 MHz = 500 ticks.
   EXPECT_EQ(cortos::time::from_microseconds(500).value, 500u);
}

TEST_F(TicklessDriverTest, FromMicrosecondsZeroIsZero)
{
   EXPECT_EQ(cortos::time::from_microseconds(0).value, 0u);
}

/* ============================================================================
 * Lifecycle: initialise / finalise
 *
 * Every test in this suite already exercises the initialise()/finalise() pair
 * via the fixture. This test makes the round-trip explicit and also documents
 * the previously-broken behaviour (initialise() not setting ds.initialised),
 * which is now fixed.
 * ========================================================================= */

TEST_F(TicklessDriverTest, InitialiseFinaliseRoundTrips)
{
   // The fixture has already called initialise(). finalise() then re-
   // initialise() must round-trip cleanly: finalise() asserts on the
   // initialised flag (set by initialise()) and resets ds; the following
   // initialise() asserts the flag is clear again.
   cortos::time::finalise();
   cortos::time::initialise(1'000'000);

   // Driver is usable again after the round-trip.
   std::atomic<int> count{0};
   EXPECT_NE(cortos::time::schedule_at(time::time_point{100}, counting_callback, &count).id, 0u);

   // TearDown() will stop() + finalise() as normal.
}

}  // namespace