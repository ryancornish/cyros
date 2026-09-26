/**
 * @file test_time_driver_simulation_realtime.cpp
 * @brief The simulation driver's real_time mode, against the wall clock.
 *
 * Subject: the real_time arms of the simulation driver (L1): start() and stop(),
 *          the real_time path of now(), and the background thread that pumps
 *          the ISR.
 *
 * The deterministic suite, `test_time_driver_simulation`, runs in virtual_time
 * mode only. These cover the rest, which cannot be deterministic because time
 * moves on its own here. With them the driver's coverage goes from 133 to 156
 * of 157 lines (coop coverage profile, 2026-09-24). The one line left is the
 * handle-id wrap-around skip, which needs 2^32 schedules to reach. They assert only loose, one-sided properties
 * (time moved, a callback 10 ms out fired within 200 ms, a cancelled one did
 * not within 250 ms), so a slow machine delays them rather than failing them.
 *
 * They were DISABLED_ in the deterministic binary until 2026-09-24. Soaked
 * before they were moved and enabled, Arch box: 300 runs at 3-way load, then 300
 * more with every core saturated by busy loops, zero failures.
 */

#include <cyros/time/time.hpp>
#include <cyros/time/simulation.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

using namespace cyros;

namespace
{

void counting_callback(void* arg) noexcept
{
   static_cast<std::atomic<int>*>(arg)->fetch_add(1, std::memory_order_relaxed);
}

/* Each test switches to real_time itself. The driver's state is heap allocated
 * in initialise() and freed in finalise(), so both are always paired. */
class SimulationRealTimeTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      cyros::time::initialise(1'000 /* Hz */);
   }

   void TearDown() override
   {
      cyros::time::stop();
      cyros::time::finalise();
   }
};

// real_time mode: now() advances on its own as wall-clock time passes.
TEST_F(SimulationRealTimeTest, RealTimeModeTimeProgresses)
{
   cyros::time::simulation::set_mode(time::simulation::mode::real_time);
   cyros::time::start();

   const time::time_point t1 = cyros::time::now();
   std::this_thread::sleep_for(std::chrono::milliseconds(50));
   const time::time_point t2 = cyros::time::now();

   EXPECT_GT(t2.value, t1.value);

   cyros::time::stop();
}

// real_time mode: a scheduled callback fires on its own once wall-clock time
// reaches the deadline (the background thread pumps the ISR).
TEST_F(SimulationRealTimeTest, RealTimeModeCallbackFiresAutonomously)
{
   cyros::time::simulation::set_mode(time::simulation::mode::real_time);
   cyros::time::start();

   std::atomic<int> count{0};
   // 1 kHz: 10 ticks ~= 10 ms. Sleep well past it.
   const uint64_t deadline = cyros::time::now().value + 10;
   ASSERT_NE(cyros::time::schedule_at(time::time_point{deadline}, counting_callback, &count).id, 0u);

   std::this_thread::sleep_for(std::chrono::milliseconds(200));
   EXPECT_GE(count.load(), 1);

   cyros::time::stop();
}

// real_time mode: cancelling before the deadline prevents the autonomous fire.
TEST_F(SimulationRealTimeTest, RealTimeModeCancelBeforeAutonomousFire)
{
   cyros::time::simulation::set_mode(time::simulation::mode::real_time);
   cyros::time::start();

   std::atomic<int> count{0};
   const uint64_t deadline = cyros::time::now().value + 100;  // ~100 ms out
   time::handle h = cyros::time::schedule_at(time::time_point{deadline}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   EXPECT_TRUE(cyros::time::cancel(h));

   std::this_thread::sleep_for(std::chrono::milliseconds(250));
   EXPECT_EQ(count.load(), 0);

   cyros::time::stop();
}

// real_time mode: start() is idempotent -- a second start() while the thread is
// already running hits the `compare_exchange_strong` false branch.
TEST_F(SimulationRealTimeTest, RealTimeModeStartIsIdempotent)
{
   cyros::time::simulation::set_mode(time::simulation::mode::real_time);
   cyros::time::start();
   cyros::time::start();  // already running -> early return

   cyros::time::stop();
   cyros::time::stop();   // already stopped -> early return
   SUCCEED();
}

// real_time mode: before start() the clock has not begun, so now() reads zero
// rather than whatever the wall clock says.
TEST_F(SimulationRealTimeTest, RealTimeModeReadsZeroBeforeStart)
{
   cyros::time::simulation::set_mode(time::simulation::mode::real_time);
   EXPECT_EQ(cyros::time::now().value, 0u);
}

}  // namespace
