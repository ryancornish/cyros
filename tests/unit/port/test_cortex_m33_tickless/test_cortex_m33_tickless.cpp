/**
 * @file test_cortex_m33_tickless.cpp
 * @brief Tickless SysTick: a free-running clock and one-shot deadlines.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the cortex_m33 port's tickless implementation of `port_time.h`,
 *          driven through the `tickless` time driver.
 * Trusts:  layers 0 to 2, and the periodic path that
 *          `test_cortex_m33_systick` proves.
 * Proves:  that `now()` is a monotonic cycle counter that survives a 24-bit
 *          hardware WRAP, that its absolute rate is the real clock, and that a
 *          one-shot deadline is delivered at the requested time rather than
 *          merely eventually.
 *
 *
 * THE WRAP IS THE POINT
 * =====================
 * SysTick is a 24-bit down counter, so at 20 MHz a full period is 839 ms and
 * anything longer is reached across several wraps with the high bits carried in
 * software. Every interesting bug in a tickless port lives at that boundary:
 * a lost wrap freezes the clock for a period, a double-counted one jumps it
 * forward, and both are invisible to a test that runs for a few milliseconds.
 *
 * So the monotonicity check below deliberately runs long enough to cross at
 * least one wrap, and verifies the crossing happened rather than assuming it.
 *
 * The re-arm path has the same hazard from the other side. `arm()` folds the
 * running interval into the software counter and then CANCELS any pending
 * SysTick exception, because that exception's effect has just been accounted
 * for and letting it run would add the period twice. The repeated-arm check
 * below exercises exactly that, many times, at intervals chosen to land near
 * the boundary.
 *
 *
 * WHY THE ABSOLUTE RATE IS CHECKED HERE TOO
 * =========================================
 * In tickless mode `freq_hz()` reports the COUNTER rate rather than a tick
 * rate, which is a different code path from the periodic one, and the
 * conversion helpers in the driver are built on it. Semihosting's SYS_ELAPSED
 * is the only reference on this bench that does not come from the thing under
 * test.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/time/time.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_time.h>
#include <cyros/port/port_traits.h>

#include <common/arm/bench.hpp>

#include <cstddef>
#include <cstdint>

using namespace cyros;

/* The port's weak board-clock symbol. In tickless mode a port tick IS a counter
 * cycle, so this is also the driver's conversion base. */
extern "C" std::uint32_t const cyros_port_systick_clock_hz;

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 2048;
alignas(CYROS_PORT_STACK_ALIGN) std::byte worker_stack[stack_size];

/* One full 24-bit SysTick period, in counter cycles. Crossing this is what
 * the software high word exists for. */
constexpr std::uint64_t hardware_period = 0x1000000ull;

volatile int  fired_count = 0;
volatile std::uint64_t fired_at = 0;

void on_deadline(void*)
{
   fired_count = fired_count + 1;
   fired_at = time::now().value;
}


void test_tickless_reports_the_counter_rate()
{
   cyros::bench::start("tickless freq_hz is the counter rate, not a tick rate");

   std::uint64_t const freq = cyros_port_time_freq_hz();
   cyros::bench::print("  freq_hz = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(freq));
   cyros::bench::print("\n");

   /* Must be the clock feeding SysTick, which is what makes now() a cycle
    * count. A periodic-style tick rate here would be a few kHz. */
   CYROS_CHECK(freq > 1'000'000ull);
}

void test_now_is_monotonic_across_a_hardware_wrap()
{
   cyros::bench::start("now() is monotonic across a 24-bit wrap");

   std::uint64_t const start = time::now().value;
   std::uint64_t previous = start;
   bool monotonic = true;
   std::uint64_t backwards_at = 0;

   /* Two full hardware periods, so at least one wrap is guaranteed to land
    * inside the loop rather than being hoped for. */
   std::uint64_t const target = start + (2u * hardware_period);

   while (previous < target) {
      std::uint64_t const current = time::now().value;
      if (current < previous) {
         monotonic = false;
         backwards_at = previous;
         break;
      }
      previous = current;
   }

   CYROS_CHECK(monotonic);
   if (!monotonic) {
      cyros::bench::print("  went backwards near ");
      cyros::bench::print_hex(static_cast<std::uint32_t>(backwards_at));
      cyros::bench::print("\n");
   }

   /* The whole point: confirm the wrap actually happened. A clock that never
    * crossed one would pass the check above for the wrong reason. */
   std::uint64_t const travelled = previous - start;
   cyros::bench::print("  cycles travelled = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(travelled >> 32));
   cyros::bench::print_hex(static_cast<std::uint32_t>(travelled));
   cyros::bench::print("\n");
   CYROS_CHECK(travelled >= hardware_period);
}

void test_the_counter_rate_is_real()
{
   cyros::bench::start("the tickless counter rate is the real one");

   if (!cyros::bench::elapsed_ns_is_available()) {
      cyros::bench::print("  no SYS_ELAPSED reference, skipping\n");
      return;
   }

   std::uint64_t const t0 = time::now().value;
   std::uint64_t const ns0 = cyros::bench::elapsed_ns();
   while ((cyros::bench::elapsed_ns() - ns0) < 200'000'000ull) { }
   std::uint64_t const ns = cyros::bench::elapsed_ns() - ns0;
   std::uint64_t const cycles = time::now().value - t0;

   std::uint64_t const measured_hz = (cycles * 1'000'000'000ull) / ns;
   std::uint64_t const claimed_hz  = cyros_port_time_freq_hz();

   cyros::bench::print("  claimed = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(claimed_hz));
   cyros::bench::print("  measured = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(measured_hz));
   cyros::bench::print("\n");

   CYROS_CHECK(measured_hz >= (claimed_hz * 90u) / 100u);
   CYROS_CHECK(measured_hz <= (claimed_hz * 110u) / 100u);
}

void test_a_one_shot_fires_at_the_requested_time()
{
   cyros::bench::start("a one-shot deadline is delivered, and on time");

   fired_count = 0;
   fired_at = 0;

   std::uint64_t const freq = cyros_port_time_freq_hz();
   /* A quarter of a hardware period, so the deadline is reached inside a
    * single interval and the arm path does the interesting re-timing. */
   std::uint64_t const delay = hardware_period / 4u;

   std::uint64_t const armed_at = time::now().value;
   auto const handle = time::schedule_at(time::time_point{armed_at + delay}, on_deadline, nullptr);
   (void)handle;
   (void)freq;

   /* Bounded wait: a deadline that never arrives must fail, not hang. */
   std::uint64_t const give_up = armed_at + (4u * hardware_period);
   while (fired_count == 0 && time::now().value < give_up) { }

   CYROS_CHECK(fired_count == 1);
   if (fired_count == 0) {
      cyros::bench::print("  the deadline never fired\n");
      return;
   }

   std::uint64_t const actual = fired_at - armed_at;
   cyros::bench::print("  requested = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(delay));
   cyros::bench::print("  actual = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(actual));
   cyros::bench::print("\n");

   /* Late is expected: the ISR and the driver's own bookkeeping cost cycles.
    * EARLY is a defect, and is what a mis-sized interval produces. */
   CYROS_CHECK(actual >= delay);
   CYROS_CHECK(actual < delay + (delay / 4u));
}

void test_a_deadline_beyond_one_period_still_fires()
{
   cyros::bench::start("a deadline further away than one hardware period fires");

   fired_count = 0;

   /* Longer than a full 24-bit period, so it cannot be reached inside one
    * interval and must survive the wrap-and-reprogram path. */
   std::uint64_t const delay = hardware_period + (hardware_period / 2u);
   std::uint64_t const armed_at = time::now().value;

   auto const handle = time::schedule_at(time::time_point{armed_at + delay}, on_deadline, nullptr);
   (void)handle;

   std::uint64_t const give_up = armed_at + (6u * hardware_period);
   while (fired_count == 0 && time::now().value < give_up) { }

   CYROS_CHECK(fired_count == 1);
   if (fired_count != 0) {
      std::uint64_t const actual = fired_at - armed_at;
      cyros::bench::print("  requested = ");
      cyros::bench::print_hex(static_cast<std::uint32_t>(delay));
      cyros::bench::print("  actual = ");
      cyros::bench::print_hex(static_cast<std::uint32_t>(actual));
      cyros::bench::print("\n");
      CYROS_CHECK(actual >= delay);
      CYROS_CHECK(actual < delay + (delay / 4u));
   }
}

void test_repeated_arming_does_not_corrupt_the_clock()
{
   cyros::bench::start("repeated re-arming keeps now() honest");

   /* Each schedule_at re-times the running interval, which folds the elapsed
    * part into the software counter and cancels a possibly-pending wrap
    * exception. Doing that many times is how a double-count or a lost period
    * shows up: the clock would jump or stall, and the elapsed cycles would
    * stop matching elapsed host time. */
   if (!cyros::bench::elapsed_ns_is_available()) {
      cyros::bench::print("  no SYS_ELAPSED reference, skipping\n");
      return;
   }

   std::uint64_t const t0 = time::now().value;
   std::uint64_t const ns0 = cyros::bench::elapsed_ns();

   bool monotonic = true;
   std::uint64_t previous = t0;

   for (int i = 0; i < 200; ++i) {
      auto const h = time::schedule_at(
         time::time_point{time::now().value + hardware_period / 2u}, on_deadline, nullptr);
      std::uint64_t const current = time::now().value;
      if (current < previous) { monotonic = false; break; }
      previous = current;
      (void)time::cancel(h);
   }

   std::uint64_t const cycles = time::now().value - t0;
   std::uint64_t const ns = cyros::bench::elapsed_ns() - ns0;
   std::uint64_t const measured_hz = ns ? (cycles * 1'000'000'000ull) / ns : 0;
   std::uint64_t const claimed_hz = cyros_port_time_freq_hz();

   CYROS_CHECK(monotonic);

   cyros::bench::print("  after 200 re-arms, measured rate = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(measured_hz));
   cyros::bench::print("\n");

   /* A double-counted wrap inflates the cycle count against host time, a lost
    * one deflates it. Either shows here. */
   CYROS_CHECK(measured_hz >= (claimed_hz * 80u) / 100u);
   CYROS_CHECK(measured_hz <= (claimed_hz * 120u) / 100u);
}

void worker()
{
   /* The frequency argument is the driver's conversion base. In tickless mode
    * the port reports the counter rate, so hand the driver the same thing. */
   time::initialise(cyros_port_systick_clock_hz);
   time::start();

   test_tickless_reports_the_counter_rate();
   test_now_is_monotonic_across_a_hardware_wrap();
   test_the_counter_rate_is_real();
   test_a_one_shot_fires_at_the_requested_time();
   test_a_deadline_beyond_one_period_still_fires();
   test_repeated_arming_does_not_corrupt_the_clock();

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("cortex_m33 tickless SysTick\n\n");

   kernel::initialise();
   thread worker_thread(worker, worker_stack, thread::priority(0), core0);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
