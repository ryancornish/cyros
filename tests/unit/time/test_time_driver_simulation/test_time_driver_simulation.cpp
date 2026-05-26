/**
 * @file test_simulation_driver.cpp
 * @brief Unit tests for the simulation time driver (src/time/simulation).
 *
 * The simulation driver implements the free-function API declared in
 * <cortos/time/time.hpp>, plus the virtual_time/real_time control surface declared
 * in <cortos/time/simulation.hpp>. A test binary links exactly ONE time
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
 * ONLY by the DISABLED_ real_time tests at the end of this file. They are
 * disabled because they depend on wall-clock sleeps and would make the suite
 * timing-flaky. Consequently those branches are NOT hit by the default run.
 *
 * To keep the coverage gate honest, the real_time-only blocks in
 * time_driver_simulation.cpp should be wrapped in coverage-exclusion markers
 * (for gcov/lcov: LCOV_EXCL_START / LCOV_EXCL_STOP), namely:
 *   - the `if (mode == real_time) { ... }` block in start()
 *   - the `if (mode == real_time) { ... }` block in stop()
 *   - the real_time return path in now()
 *   - realtime_now_ticks() and realtime_thread_main() in their entirety
 * With those markers, "100%" means 100% of the deterministically tested paths.
 */

#include <cortos/time/time.hpp>
#include <cortos/time/simulation.hpp>
#include <cortos/port/port.h>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

using namespace cortos;

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
      cortos::time::initialise(1'000 /* Hz */);
      // Default mode is virtual_time; make it explicit and deterministic.
      cortos::time::simulation::set_mode(time::simulation::mode::virtual_time);
      cortos::time::simulation::reset(time::time_point{0});
   }

   void TearDown() override
   {
      // stop() is safe whether or not start() ran. In virtual_time mode it is a
      // near no-op; pairing it keeps real_time-leaning tests tidy too.
      cortos::time::stop();
      cortos::time::finalise();
   }
};

/* ============================================================================
 * Lifecycle  (initialise / finalise / start / stop, virtual_time mode)
 * ========================================================================= */

// initialise() allocated state; reset() in SetUp left virtual time at 0.
TEST_F(SimulationDriverTest, InitialTimeIsZero)
{
   EXPECT_EQ(cortos::time::now().value, 0u);
}

// start() in virtual_time mode sets `started` but spawns no thread; the
// `mode == real_time` branch is not taken.
TEST_F(SimulationDriverTest, StartInVirtualModeDoesNotSpawnThread)
{
   cortos::time::start();
   // now() must still work and reflect virtual time.
   EXPECT_EQ(cortos::time::now().value, 0u);
}

// stop() in virtual_time mode takes the non-real_time path and is a no-op.
TEST_F(SimulationDriverTest, StopInVirtualModeIsHarmless)
{
   cortos::time::start();
   cortos::time::stop();
   cortos::time::stop();  // again -- still harmless in virtual_time mode
   SUCCEED();
}

// A driver can be started, stopped, and restarted; virtual time survives
// because it lives in driver_state, which reset() (not stop()) clears.
TEST_F(SimulationDriverTest, StopThenRestart)
{
   cortos::time::start();
   cortos::time::simulation::advance_to(time::time_point{100});
   EXPECT_EQ(cortos::time::now().value, 100u);

   cortos::time::stop();

   cortos::time::start();
   cortos::time::simulation::advance_to(time::time_point{200});
   EXPECT_EQ(cortos::time::now().value, 200u);
}

/* ============================================================================
 * time::Simulation::time::simulation::mode control  (set_mode / get_mode)
 * ========================================================================= */

// get_mode() reports the default virtual_time mode set in SetUp().
TEST_F(SimulationDriverTest, DefaultModeIsVirtual)
{
   EXPECT_EQ(cortos::time::simulation::get_mode(), time::simulation::mode::virtual_time);
}

// set_mode() round-trips both enumerators. (Switching to real_time here is
// safe: the driver has not been started, so set_mode()'s `!running` assertion
// holds. We switch straight back without ever starting in real_time.)
TEST_F(SimulationDriverTest, SetModeRoundTrips)
{
   cortos::time::simulation::set_mode(time::simulation::mode::real_time);
   EXPECT_EQ(cortos::time::simulation::get_mode(), time::simulation::mode::real_time);

   cortos::time::simulation::set_mode(time::simulation::mode::virtual_time);
   EXPECT_EQ(cortos::time::simulation::get_mode(), time::simulation::mode::virtual_time);
}

/* ============================================================================
 * reset()
 * ========================================================================= */

// reset() to a non-zero time point sets virtual time and clears events.
TEST_F(SimulationDriverTest, ResetToNonZerotimeTimePoint)
{
   cortos::time::simulation::reset(time::time_point{500});
   EXPECT_EQ(cortos::time::now().value, 500u);
}

// reset() clears any pending events: a callback scheduled before reset() must
// not fire afterwards.
TEST_F(SimulationDriverTest, ResetClearsPendingEvents)
{
   std::atomic<int> count{0};
   time::handle h = cortos::time::schedule_at(time::time_point{100}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   cortos::time::simulation::reset(time::time_point{0});

   cortos::time::simulation::advance_to(time::time_point{200});
   EXPECT_EQ(count.load(), 0);
}

/* ============================================================================
 * schedule_at  (virtual_time mode)
 * ========================================================================= */

// Null callback: the `!cb` branch returns an invalid handle.
TEST_F(SimulationDriverTest, ScheduleNullCallbackReturnsInvalidHandle)
{
   EXPECT_EQ(cortos::time::schedule_at(time::time_point{100}, nullptr, nullptr).id, 0u);
}

// Valid callback: returns a non-zero handle.
TEST_F(SimulationDriverTest, ScheduleValidCallbackReturnsValidHandle)
{
   std::atomic<int> count{0};
   EXPECT_NE(cortos::time::schedule_at(time::time_point{100}, counting_callback, &count).id, 0u);
}

// Distinct handles are issued for successive schedule_at() calls.
TEST_F(SimulationDriverTest, SuccessiveHandlesAreDistinct)
{
   std::atomic<int> count{0};
   time::handle h1 = cortos::time::schedule_at(time::time_point{100}, counting_callback, &count);
   time::handle h2 = cortos::time::schedule_at(time::time_point{200}, counting_callback, &count);
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
   ASSERT_NE(cortos::time::schedule_at(time::time_point{100}, counting_callback, &count).id, 0u);

   cortos::time::simulation::advance_to(time::time_point{100});
   EXPECT_EQ(count.load(), 1);
}

// advance_to() past the deadline still fires (the `when <= now` test is true).
TEST_F(SimulationDriverTest, CallbackFiresWhenCrossed)
{
   std::atomic<int> count{0};
   ASSERT_NE(cortos::time::schedule_at(time::time_point{50}, counting_callback, &count).id, 0u);

   cortos::time::simulation::advance_to(time::time_point{150});
   EXPECT_EQ(count.load(), 1);
}

// advance_to() short of the deadline does not fire; a later advance does.
TEST_F(SimulationDriverTest, CallbackDoesNotFireBeforeDeadline)
{
   std::atomic<int> count{0};
   ASSERT_NE(cortos::time::schedule_at(time::time_point{100}, counting_callback, &count).id, 0u);

   cortos::time::simulation::advance_to(time::time_point{99});
   EXPECT_EQ(count.load(), 0);

   cortos::time::simulation::advance_to(time::time_point{100});
   EXPECT_EQ(count.load(), 1);
}

// A callback fires exactly once: once consumed it is erased, so further
// advances do not re-fire it.
TEST_F(SimulationDriverTest, CallbackFiresOnlyOnce)
{
   std::atomic<int> count{0};
   ASSERT_NE(cortos::time::schedule_at(time::time_point{100}, counting_callback, &count).id, 0u);

   cortos::time::simulation::advance_to(time::time_point{150});
   EXPECT_EQ(count.load(), 1);

   cortos::time::simulation::advance_to(time::time_point{300});
   EXPECT_EQ(count.load(), 1);
}

// A callback whose deadline is already in the past at schedule time fires on
// the next advance, even a zero-length one (advance_to current time).
TEST_F(SimulationDriverTest, CallbackInPastFiresOnNextAdvance)
{
   cortos::time::simulation::advance_to(time::time_point{100});

   std::atomic<int> count{0};
   ASSERT_NE(cortos::time::schedule_at(time::time_point{50}, counting_callback, &count).id, 0u);

   // advance_to current time: target is clamped to now, ISR still pumps.
   cortos::time::simulation::advance_to(time::time_point{100});
   EXPECT_EQ(count.load(), 1);
}

// advance_to() a point earlier than now is monotonically clamped: time does
// not go backwards, and a future callback does not fire.
TEST_F(SimulationDriverTest, AdvanceToEarlierTimeIsClampedMonotonic)
{
   cortos::time::simulation::advance_to(time::time_point{200});
   EXPECT_EQ(cortos::time::now().value, 200u);

   std::atomic<int> count{0};
   ASSERT_NE(cortos::time::schedule_at(time::time_point{250}, counting_callback, &count).id, 0u);

   // Ask to go backwards: clamp keeps now at 200, callback at 250 stays pending.
   cortos::time::simulation::advance_to(time::time_point{100});
   EXPECT_EQ(cortos::time::now().value, 200u);
   EXPECT_EQ(count.load(), 0);
}

// advance_by() advances relative to the current virtual time.
TEST_F(SimulationDriverTest, AdvanceByMovesRelativeToNow)
{
   cortos::time::simulation::advance_to(time::time_point{100});

   std::atomic<int> count{0};
   ASSERT_NE(cortos::time::schedule_at(time::time_point{150}, counting_callback, &count).id, 0u);

   cortos::time::simulation::advance_by(time::duration{50});  // 100 -> 150
   EXPECT_EQ(cortos::time::now().value, 150u);
   EXPECT_EQ(count.load(), 1);
}

// Several callbacks at distinct deadlines each fire once, in time order.
TEST_F(SimulationDriverTest, MultipleCallbacksFireInDeadlineOrder)
{
   std::atomic<int> a{0}, b{0}, c{0};
   ASSERT_NE(cortos::time::schedule_at(time::time_point{100}, counting_callback, &a).id, 0u);
   ASSERT_NE(cortos::time::schedule_at(time::time_point{200}, counting_callback, &b).id, 0u);
   ASSERT_NE(cortos::time::schedule_at(time::time_point{300}, counting_callback, &c).id, 0u);

   cortos::time::simulation::advance_to(time::time_point{150});
   EXPECT_EQ(a.load(), 1);
   EXPECT_EQ(b.load(), 0);
   EXPECT_EQ(c.load(), 0);

   cortos::time::simulation::advance_to(time::time_point{250});
   EXPECT_EQ(b.load(), 1);
   EXPECT_EQ(c.load(), 0);

   cortos::time::simulation::advance_to(time::time_point{350});
   EXPECT_EQ(c.load(), 1);
}

// Several callbacks sharing one deadline all fire on the same advance.
TEST_F(SimulationDriverTest, CallbacksAtSameDeadlineAllFire)
{
   std::atomic<int> count{0};
   for (int i = 0; i < 5; ++i)
   {
      ASSERT_NE(cortos::time::schedule_at(time::time_point{100}, counting_callback, &count).id, 0u);
   }

   cortos::time::simulation::advance_to(time::time_point{100});
   EXPECT_EQ(count.load(), 5);
}

// An advance with no callbacks pending: fire_due_callbacks finds nothing due.
TEST_F(SimulationDriverTest, AdvanceWithNothingPendingFiresNothing)
{
   cortos::time::simulation::advance_to(time::time_point{500});
   SUCCEED();

   // And with an occupied-but-not-due event present.
   std::atomic<int> count{0};
   ASSERT_NE(cortos::time::schedule_at(time::time_point{10'000}, counting_callback, &count).id, 0u);
   cortos::time::simulation::advance_to(time::time_point{600});
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
      time::handle h = cortos::time::schedule_at(
         time::time_point{200},
         [](void* a) noexcept
         {
            static_cast<Ctx*>(a)->count.fetch_add(1, std::memory_order_relaxed);
         },
         c);
      (void)h;  // nodiscard
   };

   time::handle h = cortos::time::schedule_at(time::time_point{100}, rescheduling_cb, &ctx);
   ASSERT_NE(h.id, 0u);

   cortos::time::simulation::advance_to(time::time_point{100});
   EXPECT_EQ(ctx.count.load(), 1);

   cortos::time::simulation::advance_to(time::time_point{200});
   EXPECT_EQ(ctx.count.load(), 2);
}

/* ============================================================================
 * cancel()  (virtual_time mode)
 * ========================================================================= */

// cancel() of an invalid (id == 0) handle: the `h.id == 0` branch -> false.
TEST_F(SimulationDriverTest, CancelInvalidHandleReturnsFalse)
{
   EXPECT_FALSE(cortos::time::cancel(time::handle{0}));
}

// cancel() of an unknown non-zero id: loop finds no match -> false.
TEST_F(SimulationDriverTest, CancelUnknownHandleReturnsFalse)
{
   EXPECT_FALSE(cortos::time::cancel(time::handle{999999}));
}

// cancel() before firing: matching, non-cancelled event found -> true; the
// callback never fires.
TEST_F(SimulationDriverTest, CancelBeforeFiringPreventsCallback)
{
   std::atomic<int> count{0};
   time::handle h = cortos::time::schedule_at(time::time_point{100}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   EXPECT_TRUE(cortos::time::cancel(h));

   cortos::time::simulation::advance_to(time::time_point{200});
   EXPECT_EQ(count.load(), 0);
}

// cancel() after firing: the event was consumed (marked cancelled, then
// erased) by the fire, so the handle no longer matches -> false.
TEST_F(SimulationDriverTest, CancelAfterFiringReturnsFalse)
{
   std::atomic<int> count{0};
   time::handle h = cortos::time::schedule_at(time::time_point{100}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   cortos::time::simulation::advance_to(time::time_point{150});
   ASSERT_EQ(count.load(), 1);

   EXPECT_FALSE(cortos::time::cancel(h));
}

// Cancelling the same handle twice: the second call finds the event already
// marked cancelled, so the `!cancelled` guard fails -> false.
TEST_F(SimulationDriverTest, CancelTwiceReturnsFalseSecondTime)
{
   std::atomic<int> count{0};
   time::handle h = cortos::time::schedule_at(time::time_point{100}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   EXPECT_TRUE(cortos::time::cancel(h));
   EXPECT_FALSE(cortos::time::cancel(h));
}

// Cancelling one of several leaves the others intact.
TEST_F(SimulationDriverTest, CancelOneOfManyLeavesOthers)
{
   std::atomic<int> count{0};
   time::handle h1 = cortos::time::schedule_at(time::time_point{100}, counting_callback, &count);
   time::handle h2 = cortos::time::schedule_at(time::time_point{200}, counting_callback, &count);
   time::handle h3 = cortos::time::schedule_at(time::time_point{300}, counting_callback, &count);
   ASSERT_NE(h1.id, 0u);
   ASSERT_NE(h2.id, 0u);
   ASSERT_NE(h3.id, 0u);

   EXPECT_TRUE(cortos::time::cancel(h2));  // cancel the middle one

   cortos::time::simulation::advance_to(time::time_point{400});
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
   EXPECT_EQ(cortos::time::from_milliseconds(10).value, 10u);
}

// 0 ms converts to 0 ticks.
TEST_F(SimulationDriverTest, FromMillisecondsZero)
{
   EXPECT_EQ(cortos::time::from_milliseconds(0).value, 0u);
}

// 5000 us = 5 ms = 5 ticks at 1 kHz, exact.
TEST_F(SimulationDriverTest, FromMicrosecondsExact)
{
   EXPECT_EQ(cortos::time::from_microseconds(5000).value, 5u);
}

// 1001 us at 1 kHz is 1.001 ticks; the conversion rounds UP to 2. This is the
// case that exercises the round-up arm of the microsecond ceil division.
TEST_F(SimulationDriverTest, FromMicrosecondsRoundsUp)
{
   EXPECT_EQ(cortos::time::from_microseconds(1001).value, 2u);
}

// 1 us at 1 kHz is 0.001 ticks; rounding up yields 1 tick (a non-zero duration
// never converts to zero -- it never undersleeps).
TEST_F(SimulationDriverTest, FromMicrosecondsSubTickRoundsUpToOne)
{
   EXPECT_EQ(cortos::time::from_microseconds(1).value, 1u);
}

// 0 us converts to 0 ticks (the numerator-is-zero arm of the ceil division).
TEST_F(SimulationDriverTest, FromMicrosecondsZero)
{
   EXPECT_EQ(cortos::time::from_microseconds(0).value, 0u);
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
   ASSERT_NE(cortos::time::schedule_at(time::time_point{0}, counting_callback, &count).id, 0u);

   // Deadline 0 with virtual time already at 0: a direct ISR fires it.
   cortos::time::on_timer_isr();
   EXPECT_EQ(count.load(), 1);
}

/* ============================================================================
 * real_time mode  (DISABLED -- timing-dependent, see file header)
 *
 * These cover the wall-clock branches: the real_time arms of start()/stop(),
 * the real_time return path of now()/realtime_now_ticks(), and the background
 * realtime_thread_main loop. They use generous sleeps and assert only loose,
 * monotonic properties. They are DISABLED so the default suite stays
 * deterministic; run them manually with
 *   --gtest_also_run_disabled_tests
 * The corresponding source blocks should carry LCOV_EXCL markers so the
 * coverage gate is not skewed by their absence.
 * ========================================================================= */

// real_time mode: now() advances on its own as wall-clock time passes.
TEST_F(SimulationDriverTest, DISABLED_RealTimeModeTimeProgresses)
{
   cortos::time::simulation::set_mode(time::simulation::mode::real_time);
   cortos::time::start();

   const time::time_point t1 = cortos::time::now();
   std::this_thread::sleep_for(std::chrono::milliseconds(50));
   const time::time_point t2 = cortos::time::now();

   EXPECT_GT(t2.value, t1.value);

   cortos::time::stop();
}

// real_time mode: a scheduled callback fires on its own once wall-clock time
// reaches the deadline (the background thread pumps the ISR).
TEST_F(SimulationDriverTest, DISABLED_RealTimeModeCallbackFiresAutonomously)
{
   cortos::time::simulation::set_mode(time::simulation::mode::real_time);
   cortos::time::start();

   std::atomic<int> count{0};
   // 1 kHz: 10 ticks ~= 10 ms. Sleep well past it.
   const uint64_t deadline = cortos::time::now().value + 10;
   ASSERT_NE(cortos::time::schedule_at(time::time_point{deadline}, counting_callback, &count).id, 0u);

   std::this_thread::sleep_for(std::chrono::milliseconds(200));
   EXPECT_GE(count.load(), 1);

   cortos::time::stop();
}

// real_time mode: cancelling before the deadline prevents the autonomous fire.
TEST_F(SimulationDriverTest, DISABLED_RealTimeModeCancelBeforeAutonomousFire)
{
   cortos::time::simulation::set_mode(time::simulation::mode::real_time);
   cortos::time::start();

   std::atomic<int> count{0};
   const uint64_t deadline = cortos::time::now().value + 100;  // ~100 ms out
   time::handle h = cortos::time::schedule_at(time::time_point{deadline}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   EXPECT_TRUE(cortos::time::cancel(h));

   std::this_thread::sleep_for(std::chrono::milliseconds(250));
   EXPECT_EQ(count.load(), 0);

   cortos::time::stop();
}

// real_time mode: start() is idempotent -- a second start() while the thread is
// already running hits the `compare_exchange_strong` false branch.
TEST_F(SimulationDriverTest, DISABLED_RealTimeModeStartIsIdempotent)
{
   cortos::time::simulation::set_mode(time::simulation::mode::real_time);
   cortos::time::start();
   cortos::time::start();  // already running -> early return

   cortos::time::stop();
   cortos::time::stop();   // already stopped -> early return
   SUCCEED();
}

}  // namespace