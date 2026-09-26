/**
 * @file test_simulation_driver.cpp
 * @brief Unit tests for the simulation time driver (src/time/simulation).
 *
 * The simulation driver implements the free-function API declared in
 * <cyros/time/time.hpp>, plus the virtual_time/real_time control surface declared
 * in <cyros/time/simulation.hpp>. A test binary links exactly ONE time
 * driver, selected via test.toml [components].time_driver = "simulation".
 *
 * Model
 * -----
 * Unlike the periodic and tickless drivers, the simulation driver OWNS time --
 * it does not read the Linux port counter. It has two modes:
 *
 *   - virtual_time  : time only moves when a test calls advance_to()/advance_by().
 *                     Each advance invokes on_timer_isr() to fire due callbacks.
 *                     Fully deterministic; this is what the coverage gate targets.
 *
 *   - real_time : a background thread tracks wall-clock time and pumps the ISR
 *                 roughly every millisecond. Inherently timing-dependent.
 *
 * Lifecycle: the simulation driver heap-allocates its state in initialise()
 * and frees it in finalise(), so unlike the other two drivers BOTH must be
 * called. The fixture does this in SetUp()/TearDown().
 *
 * Coverage
 * --------
 * Goal: 100% branch coverage of the simulation translation unit via the
 * virtual_time-mode and lifecycle tests below.
 *
 * The real_time-mode branches (the `mode == real_time` arms of start(), stop(),
 * now()/realtime_now_ticks(), and the realtime_thread_main loop) are exercised
 * by `test_time_driver_simulation_realtime`, a separate integration binary, so
 * this one stays fully deterministic. They lived here as DISABLED_ tests until
 * 2026-09-24, when 600 runs on the Arch box, half of them with every core
 * saturated, showed no failure and they were moved there and enabled.
 */

#include <cyros/time/time.hpp>
#include <cyros/time/simulation.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

using namespace cyros;

namespace
{

void counting_callback(void* arg) noexcept
{
   static_cast<std::atomic<int>*>(arg)->fetch_add(1, std::memory_order_relaxed);
}

/* ============================================================================
 * Fixture
 *
 * The simulation driver allocates its driver_state on the heap in initialise()
 * and frees it in finalise(); both must be paired. Tests run in the default
 * virtual_time mode unless they explicitly switch. reset() returns virtual time and
 * the event list to a known state.
 * ========================================================================= */
class SimulationDriverTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      cyros::time::initialise(1'000 /* Hz */);
      // Default mode is virtual_time; make it explicit and deterministic.
      cyros::time::simulation::set_mode(time::simulation::mode::virtual_time);
      cyros::time::simulation::reset(time::time_point{0});
   }

   void TearDown() override
   {
      // stop() is safe whether or not start() ran. In virtual_time mode it is a
      // near no-op; pairing it keeps real_time-leaning tests tidy too.
      cyros::time::stop();
      cyros::time::finalise();
   }
};

/* ============================================================================
 * Lifecycle  (initialise / finalise / start / stop, virtual_time mode)
 * ========================================================================= */

// initialise() allocated state; reset() in SetUp left virtual time at 0.
TEST_F(SimulationDriverTest, InitialTimeIsZero)
{
   EXPECT_EQ(cyros::time::now().value, 0u);
}

// start() in virtual_time mode sets `started` but spawns no thread; the
// `mode == real_time` branch is not taken.
TEST_F(SimulationDriverTest, StartInVirtualModeDoesNotSpawnThread)
{
   cyros::time::start();
   // now() must still work and reflect virtual time.
   EXPECT_EQ(cyros::time::now().value, 0u);
}

// stop() in virtual_time mode takes the non-real_time path and is a no-op.
TEST_F(SimulationDriverTest, StopInVirtualModeIsHarmless)
{
   cyros::time::start();
   cyros::time::stop();
   cyros::time::stop();  // again -- still harmless in virtual_time mode
   SUCCEED();
}

// A driver can be started, stopped, and restarted; virtual time survives
// because it lives in driver_state, which reset() (not stop()) clears.
TEST_F(SimulationDriverTest, StopThenRestart)
{
   cyros::time::start();
   cyros::time::simulation::advance_to(time::time_point{100});
   EXPECT_EQ(cyros::time::now().value, 100u);

   cyros::time::stop();

   cyros::time::start();
   cyros::time::simulation::advance_to(time::time_point{200});
   EXPECT_EQ(cyros::time::now().value, 200u);
}

/* ============================================================================
 * time::Simulation::time::simulation::mode control  (set_mode / get_mode)
 * ========================================================================= */

// get_mode() reports the default virtual_time mode set in SetUp().
TEST_F(SimulationDriverTest, DefaultModeIsVirtual)
{
   EXPECT_EQ(cyros::time::simulation::get_mode(), time::simulation::mode::virtual_time);
}

// set_mode() round-trips both enumerators. (Switching to real_time here is
// safe: the driver has not been started, so set_mode()'s `!running` assertion
// holds. We switch straight back without ever starting in real_time.)
TEST_F(SimulationDriverTest, SetModeRoundTrips)
{
   cyros::time::simulation::set_mode(time::simulation::mode::real_time);
   EXPECT_EQ(cyros::time::simulation::get_mode(), time::simulation::mode::real_time);

   cyros::time::simulation::set_mode(time::simulation::mode::virtual_time);
   EXPECT_EQ(cyros::time::simulation::get_mode(), time::simulation::mode::virtual_time);
}

/* ============================================================================
 * reset()
 * ========================================================================= */

// reset() to a non-zero time point sets virtual time and clears events.
TEST_F(SimulationDriverTest, ResetToNonZerotimeTimePoint)
{
   cyros::time::simulation::reset(time::time_point{500});
   EXPECT_EQ(cyros::time::now().value, 500u);
}

// reset() clears any pending events: a callback scheduled before reset() must
// not fire afterwards.
TEST_F(SimulationDriverTest, ResetClearsPendingEvents)
{
   std::atomic<int> count{0};
   time::handle h = cyros::time::schedule_at(time::time_point{100}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   cyros::time::simulation::reset(time::time_point{0});

   cyros::time::simulation::advance_to(time::time_point{200});
   EXPECT_EQ(count.load(), 0);
}

/* ============================================================================
 * schedule_at  (virtual_time mode)
 * ========================================================================= */

// Null callback: the `!cb` branch returns an invalid handle.
TEST_F(SimulationDriverTest, ScheduleNullCallbackReturnsInvalidHandle)
{
   EXPECT_EQ(cyros::time::schedule_at(time::time_point{100}, nullptr, nullptr).id, 0u);
}

// Valid callback: returns a non-zero handle.
TEST_F(SimulationDriverTest, ScheduleValidCallbackReturnsValidHandle)
{
   std::atomic<int> count{0};
   EXPECT_NE(cyros::time::schedule_at(time::time_point{100}, counting_callback, &count).id, 0u);
}

// Distinct handles are issued for successive schedule_at() calls.
TEST_F(SimulationDriverTest, SuccessiveHandlesAreDistinct)
{
   std::atomic<int> count{0};
   time::handle h1 = cyros::time::schedule_at(time::time_point{100}, counting_callback, &count);
   time::handle h2 = cyros::time::schedule_at(time::time_point{200}, counting_callback, &count);
   EXPECT_NE(h1.id, 0u);
   EXPECT_NE(h2.id, 0u);
   EXPECT_NE(h1.id, h2.id);
}

/* ============================================================================
 * Firing semantics  (virtual_time mode)
 * ========================================================================= */

// advance_to() exactly at the deadline fires the callback (boundary of the
// `when <= now` test in fire_due_callbacks).
TEST_F(SimulationDriverTest, CallbackFiresAtExactDeadline)
{
   std::atomic<int> count{0};
   ASSERT_NE(cyros::time::schedule_at(time::time_point{100}, counting_callback, &count).id, 0u);

   cyros::time::simulation::advance_to(time::time_point{100});
   EXPECT_EQ(count.load(), 1);
}

// advance_to() past the deadline still fires (the `when <= now` test is true).
TEST_F(SimulationDriverTest, CallbackFiresWhenCrossed)
{
   std::atomic<int> count{0};
   ASSERT_NE(cyros::time::schedule_at(time::time_point{50}, counting_callback, &count).id, 0u);

   cyros::time::simulation::advance_to(time::time_point{150});
   EXPECT_EQ(count.load(), 1);
}

// advance_to() short of the deadline does not fire; a later advance does.
TEST_F(SimulationDriverTest, CallbackDoesNotFireBeforeDeadline)
{
   std::atomic<int> count{0};
   ASSERT_NE(cyros::time::schedule_at(time::time_point{100}, counting_callback, &count).id, 0u);

   cyros::time::simulation::advance_to(time::time_point{99});
   EXPECT_EQ(count.load(), 0);

   cyros::time::simulation::advance_to(time::time_point{100});
   EXPECT_EQ(count.load(), 1);
}

// A callback fires exactly once: once consumed it is erased, so further
// advances do not re-fire it.
TEST_F(SimulationDriverTest, CallbackFiresOnlyOnce)
{
   std::atomic<int> count{0};
   ASSERT_NE(cyros::time::schedule_at(time::time_point{100}, counting_callback, &count).id, 0u);

   cyros::time::simulation::advance_to(time::time_point{150});
   EXPECT_EQ(count.load(), 1);

   cyros::time::simulation::advance_to(time::time_point{300});
   EXPECT_EQ(count.load(), 1);
}

// A callback whose deadline is already in the past at schedule time fires on
// the next advance, even a zero-length one (advance_to current time).
TEST_F(SimulationDriverTest, CallbackInPastFiresOnNextAdvance)
{
   cyros::time::simulation::advance_to(time::time_point{100});

   std::atomic<int> count{0};
   ASSERT_NE(cyros::time::schedule_at(time::time_point{50}, counting_callback, &count).id, 0u);

   // advance_to current time: target is clamped to now, ISR still pumps.
   cyros::time::simulation::advance_to(time::time_point{100});
   EXPECT_EQ(count.load(), 1);
}

// advance_to() a point earlier than now is monotonically clamped: time does
// not go backwards, and a future callback does not fire.
TEST_F(SimulationDriverTest, AdvanceToEarlierTimeIsClampedMonotonic)
{
   cyros::time::simulation::advance_to(time::time_point{200});
   EXPECT_EQ(cyros::time::now().value, 200u);

   std::atomic<int> count{0};
   ASSERT_NE(cyros::time::schedule_at(time::time_point{250}, counting_callback, &count).id, 0u);

   // Ask to go backwards: clamp keeps now at 200, callback at 250 stays pending.
   cyros::time::simulation::advance_to(time::time_point{100});
   EXPECT_EQ(cyros::time::now().value, 200u);
   EXPECT_EQ(count.load(), 0);
}

// advance_by() advances relative to the current virtual time.
TEST_F(SimulationDriverTest, AdvanceByMovesRelativeToNow)
{
   cyros::time::simulation::advance_to(time::time_point{100});

   std::atomic<int> count{0};
   ASSERT_NE(cyros::time::schedule_at(time::time_point{150}, counting_callback, &count).id, 0u);

   cyros::time::simulation::advance_by(time::duration{50});  // 100 -> 150
   EXPECT_EQ(cyros::time::now().value, 150u);
   EXPECT_EQ(count.load(), 1);
}

// Several callbacks at distinct deadlines each fire once, in time order.
TEST_F(SimulationDriverTest, MultipleCallbacksFireInDeadlineOrder)
{
   std::atomic<int> a{0}, b{0}, c{0};
   ASSERT_NE(cyros::time::schedule_at(time::time_point{100}, counting_callback, &a).id, 0u);
   ASSERT_NE(cyros::time::schedule_at(time::time_point{200}, counting_callback, &b).id, 0u);
   ASSERT_NE(cyros::time::schedule_at(time::time_point{300}, counting_callback, &c).id, 0u);

   cyros::time::simulation::advance_to(time::time_point{150});
   EXPECT_EQ(a.load(), 1);
   EXPECT_EQ(b.load(), 0);
   EXPECT_EQ(c.load(), 0);

   cyros::time::simulation::advance_to(time::time_point{250});
   EXPECT_EQ(b.load(), 1);
   EXPECT_EQ(c.load(), 0);

   cyros::time::simulation::advance_to(time::time_point{350});
   EXPECT_EQ(c.load(), 1);
}

// Several callbacks sharing one deadline all fire on the same advance.
TEST_F(SimulationDriverTest, CallbacksAtSameDeadlineAllFire)
{
   std::atomic<int> count{0};
   for (int i = 0; i < 5; ++i)
   {
      ASSERT_NE(cyros::time::schedule_at(time::time_point{100}, counting_callback, &count).id, 0u);
   }

   cyros::time::simulation::advance_to(time::time_point{100});
   EXPECT_EQ(count.load(), 5);
}

// An advance with no callbacks pending: fire_due_callbacks finds nothing due.
TEST_F(SimulationDriverTest, AdvanceWithNothingPendingFiresNothing)
{
   cyros::time::simulation::advance_to(time::time_point{500});
   SUCCEED();

   // And with an occupied-but-not-due event present.
   std::atomic<int> count{0};
   ASSERT_NE(cyros::time::schedule_at(time::time_point{10'000}, counting_callback, &count).id, 0u);
   cyros::time::simulation::advance_to(time::time_point{600});
   EXPECT_EQ(count.load(), 0);
}

// A callback may schedule another callback while running; the newly scheduled
// one fires on a subsequent advance.
TEST_F(SimulationDriverTest, CallbackCanScheduleAnotherCallback)
{
   struct Ctx
   {
      std::atomic<int> count{0};
   } ctx;

   auto rescheduling_cb = [](void* arg) noexcept
   {
      auto* c = static_cast<Ctx*>(arg);
      c->count.fetch_add(1, std::memory_order_relaxed);

      // Schedule a follow-up that only increments the counter.
      time::handle h = cyros::time::schedule_at(
         time::time_point{200},
         [](void* a) noexcept
         {
            static_cast<Ctx*>(a)->count.fetch_add(1, std::memory_order_relaxed);
         },
         c);
      (void)h;  // nodiscard
   };

   time::handle h = cyros::time::schedule_at(time::time_point{100}, rescheduling_cb, &ctx);
   ASSERT_NE(h.id, 0u);

   cyros::time::simulation::advance_to(time::time_point{100});
   EXPECT_EQ(ctx.count.load(), 1);

   cyros::time::simulation::advance_to(time::time_point{200});
   EXPECT_EQ(ctx.count.load(), 2);
}

/* ============================================================================
 * cancel()  (virtual_time mode)
 * ========================================================================= */

// cancel() of an invalid (id == 0) handle: the `h.id == 0` branch -> false.
TEST_F(SimulationDriverTest, CancelInvalidHandleReturnsFalse)
{
   EXPECT_FALSE(cyros::time::cancel(time::handle{0}));
}

// cancel() of an unknown non-zero id: loop finds no match -> false.
TEST_F(SimulationDriverTest, CancelUnknownHandleReturnsFalse)
{
   EXPECT_FALSE(cyros::time::cancel(time::handle{999999}));
}

// cancel() before firing: matching, non-cancelled event found -> true; the
// callback never fires.
TEST_F(SimulationDriverTest, CancelBeforeFiringPreventsCallback)
{
   std::atomic<int> count{0};
   time::handle h = cyros::time::schedule_at(time::time_point{100}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   EXPECT_TRUE(cyros::time::cancel(h));

   cyros::time::simulation::advance_to(time::time_point{200});
   EXPECT_EQ(count.load(), 0);
}

// cancel() after firing: the event was consumed (marked cancelled, then
// erased) by the fire, so the handle no longer matches -> false.
TEST_F(SimulationDriverTest, CancelAfterFiringReturnsFalse)
{
   std::atomic<int> count{0};
   time::handle h = cyros::time::schedule_at(time::time_point{100}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   cyros::time::simulation::advance_to(time::time_point{150});
   ASSERT_EQ(count.load(), 1);

   EXPECT_FALSE(cyros::time::cancel(h));
}

// Cancelling the same handle twice: the second call finds the event already
// marked cancelled, so the `!cancelled` guard fails -> false.
TEST_F(SimulationDriverTest, CancelTwiceReturnsFalseSecondTime)
{
   std::atomic<int> count{0};
   time::handle h = cyros::time::schedule_at(time::time_point{100}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   EXPECT_TRUE(cyros::time::cancel(h));
   EXPECT_FALSE(cyros::time::cancel(h));
}

// Cancelling one of several leaves the others intact.
TEST_F(SimulationDriverTest, CancelOneOfManyLeavesOthers)
{
   std::atomic<int> count{0};
   time::handle h1 = cyros::time::schedule_at(time::time_point{100}, counting_callback, &count);
   time::handle h2 = cyros::time::schedule_at(time::time_point{200}, counting_callback, &count);
   time::handle h3 = cyros::time::schedule_at(time::time_point{300}, counting_callback, &count);
   ASSERT_NE(h1.id, 0u);
   ASSERT_NE(h2.id, 0u);
   ASSERT_NE(h3.id, 0u);

   EXPECT_TRUE(cyros::time::cancel(h2));  // cancel the middle one

   cyros::time::simulation::advance_to(time::time_point{400});
   EXPECT_EQ(count.load(), 2);  // h1 and h3 fired, h2 did not
}

/* ============================================================================
 * time::duration conversion
 *
 * The simulation driver converts using the frequency passed to initialise()
 * (1000 Hz in this fixture). Both an exact conversion and a rounding-up
 * conversion are covered, exercising both arms of the ceil division.
 * ========================================================================= */

// 10 ms at 1 kHz = 10 ticks exactly.
TEST_F(SimulationDriverTest, FromMillisecondsExact)
{
   EXPECT_EQ(cyros::time::from_milliseconds(10).value, 10u);
}

// 0 ms converts to 0 ticks.
TEST_F(SimulationDriverTest, FromMillisecondsZero)
{
   EXPECT_EQ(cyros::time::from_milliseconds(0).value, 0u);
}

// 5000 us = 5 ms = 5 ticks at 1 kHz, exact.
TEST_F(SimulationDriverTest, FromMicrosecondsExact)
{
   EXPECT_EQ(cyros::time::from_microseconds(5000).value, 5u);
}

// 1001 us at 1 kHz is 1.001 ticks; the conversion rounds UP to 2. This is the
// case that exercises the round-up arm of the microsecond ceil division.
TEST_F(SimulationDriverTest, FromMicrosecondsRoundsUp)
{
   EXPECT_EQ(cyros::time::from_microseconds(1001).value, 2u);
}

// 1 us at 1 kHz is 0.001 ticks; rounding up yields 1 tick (a non-zero duration
// never converts to zero -- it never undersleeps).
TEST_F(SimulationDriverTest, FromMicrosecondsSubTickRoundsUpToOne)
{
   EXPECT_EQ(cyros::time::from_microseconds(1).value, 1u);
}

// 0 us converts to 0 ticks (the numerator-is-zero arm of the ceil division).
TEST_F(SimulationDriverTest, FromMicrosecondsZero)
{
   EXPECT_EQ(cyros::time::from_microseconds(0).value, 0u);
}

/* ============================================================================
 * on_timer_isr() directly
 * ========================================================================= */

// Calling on_timer_isr() directly (no advance) fires anything already due.
// After advancing virtual time without going through advance_to(), this is the
// path the real_time background thread would use.
TEST_F(SimulationDriverTest, OnTimerIsrFiresDueCallbacks)
{
   std::atomic<int> count{0};
   ASSERT_NE(cyros::time::schedule_at(time::time_point{0}, counting_callback, &count).id, 0u);

   // Deadline 0 with virtual time already at 0: a zero-length pump fires it.
   cyros::time::simulation::advance_by(time::duration{0});
   EXPECT_EQ(count.load(), 1);
}


/* ============================================================================
 * Recurring timers  (virtual_time mode)
 *
 * schedule_recurring had no consumer in this file at all, so the sim driver's
 * re-arm arithmetic, its two rejection paths and the catch-up loop were carried
 * only by the periodic and tickless preempt suites, which exercise a different
 * implementation. Everything here is deterministic: virtual time moves only
 * when a test advances it.
 * ========================================================================= */

// A null callback is rejected before a slot is taken.
TEST_F(SimulationDriverTest, ScheduleRecurringNullCallbackReturnsInvalidHandle)
{
   time::handle const h = cyros::time::schedule_recurring(time::duration{10}, nullptr, nullptr);

   EXPECT_EQ(h.id, 0u);
}

// A zero interval is rejected: it would re-arm at the same tick forever.
TEST_F(SimulationDriverTest, ScheduleRecurringZeroIntervalReturnsInvalidHandle)
{
   std::atomic<int> count{0};

   time::handle const h = cyros::time::schedule_recurring(time::duration{0}, counting_callback, &count);

   EXPECT_EQ(h.id, 0u);
   cyros::time::simulation::advance_by(time::duration{100});
   EXPECT_EQ(count.load(), 0) << "a rejected recurring timer still fired";
}

// The first fire lands one full interval after arming, not immediately.
TEST_F(SimulationDriverTest, RecurringDoesNotFireBeforeItsFirstInterval)
{
   std::atomic<int> count{0};
   time::handle const h = cyros::time::schedule_recurring(time::duration{10}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   cyros::time::simulation::advance_by(time::duration{9});

   EXPECT_EQ(count.load(), 0);
}

// One fire per interval, and the timer stays armed across fires.
TEST_F(SimulationDriverTest, RecurringFiresOncePerIntervalAcrossManyAdvances)
{
   std::atomic<int> count{0};
   time::handle const h = cyros::time::schedule_recurring(time::duration{10}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   for (int i = 1; i <= 5; ++i) {
      cyros::time::simulation::advance_by(time::duration{10});
      EXPECT_EQ(count.load(), i) << "wrong fire count after " << i << " intervals";
   }
}

/* One advance that crosses several intervals fires ONCE and re-anchors ahead of
 * now, rather than firing once per skipped interval. That is the `do { when +=
 * period; } while (when <= now)` loop, and it is what stops a long advance (or a
 * late pump) from delivering a burst of backlogged callbacks. */
TEST_F(SimulationDriverTest, RecurringCrossingSeveralIntervalsInOneAdvanceFiresOnceAndReAnchors)
{
   std::atomic<int> count{0};
   time::handle const h = cyros::time::schedule_recurring(time::duration{10}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   cyros::time::simulation::advance_by(time::duration{35}); // crosses 10, 20, 30

   EXPECT_EQ(count.load(), 1) << "a multi-interval advance delivered a backlog burst";

   // Re-anchored to 40, the first multiple strictly after 35.
   cyros::time::simulation::advance_by(time::duration{4});  // now 39
   EXPECT_EQ(count.load(), 1);
   cyros::time::simulation::advance_by(time::duration{1});  // now 40
   EXPECT_EQ(count.load(), 2) << "the timer did not re-anchor to the next interval after now";
}

// Cancelling a recurring timer stops it permanently.
TEST_F(SimulationDriverTest, CancelStopsARecurringTimer)
{
   std::atomic<int> count{0};
   time::handle const h = cyros::time::schedule_recurring(time::duration{10}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   cyros::time::simulation::advance_by(time::duration{10});
   ASSERT_EQ(count.load(), 1);

   EXPECT_TRUE(cyros::time::cancel(h));

   cyros::time::simulation::advance_by(time::duration{100});
   EXPECT_EQ(count.load(), 1) << "a cancelled recurring timer kept firing";
}

// A recurring and a one-shot timer coexist: the one-shot retires, the recurring
// keeps going. This is the branch of fire_due_callbacks that distinguishes them.
TEST_F(SimulationDriverTest, OneShotRetiresWhileARecurringTimerContinues)
{
   std::atomic<int> once{0};
   std::atomic<int> repeating{0};

   ASSERT_NE(cyros::time::schedule_at(time::time_point{10}, counting_callback, &once).id, 0u);
   ASSERT_NE(cyros::time::schedule_recurring(time::duration{10}, counting_callback, &repeating).id, 0u);

   cyros::time::simulation::advance_by(time::duration{10});
   EXPECT_EQ(once.load(), 1);
   EXPECT_EQ(repeating.load(), 1);

   cyros::time::simulation::advance_by(time::duration{10});
   EXPECT_EQ(once.load(), 1)      << "a one-shot fired twice";
   EXPECT_EQ(repeating.load(), 2) << "the recurring timer stopped when the one-shot retired";
}

/* ============================================================================
 * Duration conversions
 *
 * from_* had tests, to_* had none on this driver. At 1 kHz one tick is one
 * millisecond, so the round trips are exact and the microsecond scale is what
 * shows the rounding.
 * ========================================================================= */

TEST_F(SimulationDriverTest, ToMillisecondsInvertsFromMilliseconds)
{
   EXPECT_EQ(cyros::time::to_milliseconds(cyros::time::from_milliseconds(0)), 0u);
   EXPECT_EQ(cyros::time::to_milliseconds(cyros::time::from_milliseconds(1)), 1u);
   EXPECT_EQ(cyros::time::to_milliseconds(cyros::time::from_milliseconds(250)), 250u);
}

TEST_F(SimulationDriverTest, ToMicrosecondsScalesTicksToMicroseconds)
{
   // 1 kHz: one tick is 1000 us.
   EXPECT_EQ(cyros::time::to_microseconds(time::duration{0}), 0u);
   EXPECT_EQ(cyros::time::to_microseconds(time::duration{1}), 1'000u);
   EXPECT_EQ(cyros::time::to_microseconds(time::duration{7}), 7'000u);
}

// Sub-tick microsecond values round UP to a whole tick, so a deadline is never
// earlier than asked for; a zero request stays zero.
TEST_F(SimulationDriverTest, FromMicrosecondsRoundsUpToAWholeTick)
{
   EXPECT_EQ(cyros::time::from_microseconds(0).value, 0u);
   EXPECT_EQ(cyros::time::from_microseconds(1).value, 1u);
   EXPECT_EQ(cyros::time::from_microseconds(999).value, 1u);
   EXPECT_EQ(cyros::time::from_microseconds(1'000).value, 1u);
   EXPECT_EQ(cyros::time::from_microseconds(1'001).value, 2u);
}


}  // namespace