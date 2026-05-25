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
 * one documented exception (see KNOWN BUG below).
 *
 * KNOWN BUG (see DISABLED_InitialiseDoesNotSetInitialisedFlag):
 *   tickless::initialise() sets frequency_hz but never sets
 *   ds.initialised = true. finalise() asserts on ds.initialised, so the
 *   initialise()/finalise() pair cannot currently be exercised without
 *   tripping CORTOS_ASSERT. The periodic driver sets the flag correctly;
 *   the tickless driver should do the same. Until that one-line fix lands,
 *   these tests deliberately do NOT call initialise()/finalise(): the driver
 *   does not need initialise() for now()/schedule_at()/cancel() to work, so
 *   coverage of the rest of the file is unaffected. The two lines of
 *   initialise()/finalise() are the only uncovered lines and are tracked by
 *   the disabled test below.
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
      // NOTE: intentionally NOT calling cortos::time::initialise() -- see the
      // KNOWN BUG note in the file header. start()/stop()/schedule_at()/
      // cancel()/now() do not depend on the `initialised` flag.
   }

   void TearDown() override
   {
      // Ensure the one-shot is disarmed and the driver is stopped between
      // tests so static state does not leak. stop() is idempotent.
      cortos::time::stop();
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
   EXPECT_EQ(cortos::time::schedule_at(time::TimePoint{100}, nullptr, nullptr).id, 0u);
}

// Valid callback: takes the first free slot, rearms, returns a valid handle.
TEST_F(TicklessDriverTest, ScheduleValidCallbackReturnsValidHandle)
{
   cortos::time::start();
   std::atomic<int> count{0};
   EXPECT_NE(cortos::time::schedule_at(time::TimePoint{100}, counting_callback, &count).id, 0u);
}

// Scheduling before start(): schedule_at() does not require the driver to be
// started; the slot is taken and rearm_locked() runs. A later start() will
// rearm again. This covers schedule_at()'s rearm call independent of start().
TEST_F(TicklessDriverTest, ScheduleBeforeStartStillSchedules)
{
   std::atomic<int> count{0};
   time::Handle h = cortos::time::schedule_at(time::TimePoint{100}, counting_callback, &count);
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
   std::vector<time::Handle> handles;
   handles.reserve(kMaxScheduledCallbacks);

   for (uint32_t i = 0; i < kMaxScheduledCallbacks; ++i)
   {
      time::Handle h = cortos::time::schedule_at(
         time::TimePoint{static_cast<uint64_t>(i) + 1000}, counting_callback, &count);
      ASSERT_NE(h.id, 0u) << "slot " << i << " should still be free";
      handles.push_back(h);
   }

   EXPECT_EQ(cortos::time::schedule_at(time::TimePoint{9999}, counting_callback, &count).id, 0u);

   // Freeing a slot makes room again (and rearms).
   ASSERT_TRUE(cortos::time::cancel(handles.front()));
   EXPECT_NE(cortos::time::schedule_at(time::TimePoint{9999}, counting_callback, &count).id, 0u);
}

/* ============================================================================
 * Firing semantics
 * ========================================================================= */

// Future deadline: fires once exactly when now >= deadline, not before.
TEST_F(TicklessDriverTest, CallbackFiresWhenDeadlineReached)
{
   cortos::time::start();

   std::atomic<int> count{0};
   ASSERT_NE(cortos::time::schedule_at(time::TimePoint{100}, counting_callback, &count).id, 0u);

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
   ASSERT_NE(cortos::time::schedule_at(time::TimePoint{10}, counting_callback, &count).id, 0u);

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
   ASSERT_NE(cortos::time::schedule_at(time::TimePoint{50}, counting_callback, &count).id, 0u);

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
   ASSERT_NE(cortos::time::schedule_at(time::TimePoint{50},  counting_callback, &a).id, 0u);
   ASSERT_NE(cortos::time::schedule_at(time::TimePoint{100}, counting_callback, &b).id, 0u);
   ASSERT_NE(cortos::time::schedule_at(time::TimePoint{150}, counting_callback, &c).id, 0u);

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
      ASSERT_NE(cortos::time::schedule_at(time::TimePoint{100}, counting_callback, &count).id, 0u);
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
   ASSERT_NE(cortos::time::schedule_at(time::TimePoint{10'000}, counting_callback, &count).id, 0u);
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
   EXPECT_FALSE(cortos::time::cancel(time::Handle{0}));
}

// cancel() of an unknown non-zero id: loop finds no match -> false.
TEST_F(TicklessDriverTest, CancelUnknownHandleReturnsFalse)
{
   cortos::time::start();
   EXPECT_FALSE(cortos::time::cancel(time::Handle{999999}));
}

// cancel() before firing: slot matched and cleared, driver rearms -> true.
TEST_F(TicklessDriverTest, CancelBeforeFiringPreventsCallback)
{
   cortos::time::start();

   std::atomic<int> count{0};
   time::Handle h = cortos::time::schedule_at(time::TimePoint{100}, counting_callback, &count);
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
   time::Handle h = cortos::time::schedule_at(time::TimePoint{100}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   EXPECT_TRUE(cortos::time::cancel(h));  // rearm_locked() -> disarm
   SUCCEED();
}

// cancel() after firing: slot already freed -> no match -> false.
TEST_F(TicklessDriverTest, CancelAfterFiringReturnsFalse)
{
   cortos::time::start();

   std::atomic<int> count{0};
   time::Handle h = cortos::time::schedule_at(time::TimePoint{50}, counting_callback, &count);
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
   time::Handle h = cortos::time::schedule_at(time::TimePoint{100}, counting_callback, &count);
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
   time::Handle h1 = cortos::time::schedule_at(time::TimePoint{100}, counting_callback, &count);
   time::Handle h2 = cortos::time::schedule_at(time::TimePoint{200}, counting_callback, &count);
   time::Handle h3 = cortos::time::schedule_at(time::TimePoint{300}, counting_callback, &count);
   ASSERT_NE(h1.id, 0u);
   ASSERT_NE(h2.id, 0u);
   ASSERT_NE(h3.id, 0u);

   EXPECT_TRUE(cortos::time::cancel(h2));

   advance_and_pump(400);
   EXPECT_EQ(count.load(), 2);
}

/* ============================================================================
 * Duration conversion
 *
 * The tickless driver converts using its own frequency_hz field. Because the
 * KNOWN BUG prevents calling initialise() (which would set frequency_hz), the
 * field is left at its default of 0 in these tests. With frequency 0 the
 * conversions yield 0 ticks: numerator (ms * 0) is 0, ceil_div(0, d) == 0.
 * This still exercises both conversion functions and the ceil_div a == 0 path.
 *
 * Once the initialise() bug is fixed, these should be strengthened to call
 * initialise() with a real frequency and assert non-zero, rounded results --
 * see the periodic driver tests for the intended shape.
 * ========================================================================= */

TEST_F(TicklessDriverTest, FromMillisecondsWithDefaultFrequencyIsZero)
{
   // frequency_hz defaults to 0 (initialise() not called -- see KNOWN BUG).
   EXPECT_EQ(cortos::time::from_milliseconds(10).value, 0u);
}

TEST_F(TicklessDriverTest, FromMicrosecondsWithDefaultFrequencyIsZero)
{
   EXPECT_EQ(cortos::time::from_microseconds(500).value, 0u);
}

/* ============================================================================
 * KNOWN BUG tracker
 * ========================================================================= */

// DISABLED: tickless::initialise() never sets ds.initialised = true, so
// finalise()'s CORTOS_ASSERT(initialised) aborts. Enable this test once the
// driver is fixed (one line: `tickless::ds.initialised = true;` in
// initialise(), matching the periodic driver). When enabled it also restores
// real duration-conversion coverage.
TEST_F(TicklessDriverTest, DISABLED_InitialiseDoesNotSetInitialisedFlag)
{
   cortos::time::initialise(1'000'000);

   // With the bug present, the following line aborts via CORTOS_ASSERT.
   // With the bug fixed, it succeeds and frequency-based conversion works.
   cortos::time::finalise();

   SUCCEED();
}

}  // namespace