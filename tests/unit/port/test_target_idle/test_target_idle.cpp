/**
 * @file test_target_idle.cpp
 * @brief The idle path: a core with nothing to run waits in WFI, and a sleeping
 *        thread is woken out of it on time.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the idle path on a target port (cortex_m, riscv_virt), end to
 *          end. When every thread sleeps the scheduler runs the idle thread,
 *          idle_task calls cyros_port_idle, the core waits in WFI, and the
 *          time driver's interrupt wakes it and the sleeper. Built twice from
 *          this one source, against the periodic driver (test_target_idle)
 *          and the tickless one (test_target_idle_tickless).
 * Trusts:  layers 0 to 7, the timer paths that test_target_tick and the
 *          port's tickless test prove from a spinning thread, and the chrono
 *          feature's sleep, which this is the first on-target test to use.
 * Proves:  that idle is reached, with interrupts enabled, that the core waits
 *          between interrupts rather than spinning, that a sleep wakes no
 *          earlier than its deadline and not much later, for deadlines already
 *          due, short, and further away than one hardware period, that two
 *          sleepers wake in deadline order, and that no sleep already due
 *          makes now() leap.
 *
 *
 * WHY THIS EXISTS
 * ===============
 * Until it, no on-target test let the idle thread run, because every one spins.
 * So cyros_port_idle, the idle thread's stack (the only stack that runs on
 * exactly min_stack_size, and it runs under the stack guard), and the tickless
 * driver's wake out of WFI were exercised by nothing. Two tickless defects hid
 * there (arm-port-notes 10b, items 4 and 5): a deadline already due stopped the
 * clock for good, and then made now() leap a whole hardware period.
 *
 *
 * HOW IDLE IS OBSERVED
 * ====================
 * The link wraps cyros_port_idle (`-Wl,--wrap` in test.toml), so every call the
 * kernel makes to it reaches __wrap_cyros_port_idle below first. That counts
 * the call, notes whether interrupts were enabled, and forwards to the port.
 * It sits on the port contract's own boundary, so nothing inside the kernel is
 * reached into.
 *
 * The count is what tells waiting from spinning, which nothing else on QEMU
 * can see. A core in WFI enters idle about once per interrupt: once per tick on
 * the periodic driver, once per deadline or hardware period on the tickless
 * one. A core spinning through the idle loop enters it thousands of times a
 * millisecond.
 *
 *
 * A FAILURE THAT CAN ONLY HANG
 * ============================
 * A lost wake leaves every thread asleep and the core in WFI with no interrupt
 * coming, so nothing in the image can report it. The runner's timeout does, and
 * the last "===" line printed names the case.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/time/time.hpp>
#include <cyros/chrono/chrono.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_mcu.h>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <cstddef>
#include <cstdint>

using namespace cyros;

/* The tickless counter's clock, a board fact, which in tickless mode is also
 * the tick rate. And the longest interval that counter can time in one go,
 * which on SysTick is a full 24-bit period and is what a long sleep has to
 * cross with the core idle. MTIME is 64 bits and is never restarted, so it
 * has no such limit. Which counter the board's port runs on is the board's
 * toolchain's to say (CYROS_BENCH_MTIME), because it is not the ISA's: the
 * RP2350's Cortex-M33 targets run on MTIME too. */
#if defined(CYROS_BENCH_MTIME)
extern "C" std::uint32_t cyros_port_mtime_clock_hz(void);
inline std::uint32_t counter_clock_hz() { return cyros_port_mtime_clock_hz(); }
inline constexpr std::uint64_t hardware_period = 0u;
#else
extern "C" std::uint32_t cyros_port_systick_clock_hz(void);
inline std::uint32_t counter_clock_hz() { return cyros_port_systick_clock_hz(); }
inline constexpr std::uint64_t hardware_period = 0x1000000ull;
#endif

namespace
{

/* Which driver this build has. Each of the two tests' config headers says,
 * because the drivers want different arguments to time::initialise and nothing
 * else in the build tells them apart. The first case checks it against what the
 * port reports, so a config that names the wrong driver fails. */
constexpr bool tickless = CYROS_TEST_IDLE_TICKLESS != 0;

/* The periodic driver's tick, a typical RTOS rate. */
constexpr std::uint32_t tick_hz = 1'000;

constexpr std::size_t stack_size = thread::min_stack_size + 2048;
alignas(CYROS_PORT_STACK_ALIGN) std::byte worker_stack[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte early_stack[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte late_stack[stack_size];

/* Written only by the idle thread, through the wrapper below. */
volatile std::uint32_t idle_entries = 0;
volatile std::uint32_t idle_entries_masked = 0;

} // namespace

extern "C" void __real_cyros_port_idle(void);

extern "C" void __wrap_cyros_port_idle(void)
{
   idle_entries = idle_entries + 1;
   if (!cyros_port_interrupts_enabled()) {
      idle_entries_masked = idle_entries_masked + 1;
   }
   __real_cyros_port_idle();
}

namespace
{

/* Interrupts the driver needs to deliver a sleep of @p d, which is how often
 * a waiting core should enter idle. Every tick on the periodic driver. On the
 * tickless one, the deadline plus one per hardware period crossed. */
std::uint64_t interrupts_for(time::duration d)
{
   if (tickless) {
      return hardware_period == 0u ? 1u : (d.value / hardware_period) + 1u;
   }
   return d.value + 1u;
}

/* How late a wake may be and still count as on time. Early is never allowed.
 * Late is within the contract, so the bound only has to catch a wake that
 * waited for the wrong interrupt: a whole tickless period, or a tick that was
 * never armed. A quarter of the sleep, plus 5 ms and two ticks for an emulator
 * whose interrupt delivery depends on the host's scheduling. */
std::uint64_t allowed_lateness(time::duration d)
{
   return (d.value / 4u) + time::from_milliseconds(5).value + 2u;
}

void print_ticks(char const* label, std::uint64_t value)
{
   cyros::bench::print(label);
   cyros::bench::print_dec(value);
}

void test_the_driver_is_the_one_the_config_names()
{
   cyros::bench::start("the build's time driver is the one its config names");

   std::uint64_t const expected = tickless ? counter_clock_hz() : tick_hz;
   print_ticks(tickless ? "  tickless, freq_hz = " : "  periodic, freq_hz = ", cyros_port_time_freq_hz());
   cyros::bench::print("\n");
   CYROS_CHECK(cyros_port_time_freq_hz() == expected);
}

void test_a_sleeping_thread_lets_the_core_wait_in_idle()
{
   cyros::bench::start("a sleeping thread lets the core idle, and it waits rather than spins");

   constexpr std::uint32_t sleeps = 10;
   time::duration const d = time::from_milliseconds(20);

   std::uint32_t const entries_before = idle_entries;
   std::uint32_t const masked_before  = idle_entries_masked;
   for (std::uint32_t i = 0; i < sleeps; ++i) {
      this_thread::sleep_for(d);
   }
   std::uint32_t const entries = idle_entries - entries_before;
   std::uint32_t const masked  = idle_entries_masked - masked_before;

   /* Twice the interrupts the driver needs, and a few for the wake itself. A
    * spinning idle is orders of magnitude past this. */
   std::uint64_t const allowed = sleeps * ((2u * interrupts_for(d)) + 4u);

   print_ticks("  idle entries over ", sleeps);
   print_ticks(" sleeps = ", entries);
   print_ticks(", allowed = ", allowed);
   cyros::bench::print("\n");

   CYROS_CHECK(entries >= sleeps);       // every sleep reached the port's idle
   CYROS_CHECK(entries <= allowed);      // and waited there, once per interrupt
   CYROS_CHECK(masked == 0u);            // with interrupts enabled
}

void check_a_wake_from_idle(char const* label, time::duration d)
{
   time::time_point const deadline = time::now() + d;
   this_thread::sleep_until(deadline);
   time::time_point const woke = time::now();

   bool const early = woke < deadline;
   std::uint64_t const late = early ? 0u : woke.value - deadline.value;

   cyros::bench::print(label);
   print_ticks(": slept ", d.value);
   print_ticks(" ticks, woke ", late);
   cyros::bench::print(early ? " ticks EARLY\n" : " ticks late\n");

   CYROS_CHECK(!early);
   CYROS_CHECK(late <= allowed_lateness(d));
}

void test_a_sleep_wakes_from_idle_on_time()
{
   cyros::bench::start("a sleep wakes from idle on time, short and long");

   check_a_wake_from_idle("  1 ms", time::from_milliseconds(1));
   check_a_wake_from_idle("  7 ms", time::from_milliseconds(7));
   check_a_wake_from_idle("  50 ms", time::from_milliseconds(50));

   /* On the SysTick tickless driver this one crosses a hardware wrap with the
    * core waiting in idle, which the tickless test only ever does from a
    * spinning thread. Anywhere else it is simply a long sleep. */
   time::duration const beyond = (tickless && hardware_period != 0u)
      ? time::duration{hardware_period + (hardware_period / 2u)}
      : time::from_milliseconds(300);
   check_a_wake_from_idle("  past one hardware period", beyond);
}

void test_sleeps_already_due_return_and_the_clock_stays_honest()
{
   cyros::bench::start("sleeps already due return, and no round makes now() leap");

   /* Each of these asks the driver for the shortest interval it has, or for one
    * already past, with every thread then asleep. That is the shape of 10b
    * item 4, where the clock stopped and this loop would hang, and of item 5,
    * where now() leapt a whole hardware period, and of item 7, the same leap by
    * another route. Item 7 needs QEMU to leave VAL reading zero after a write
    * for longer than usual, which its host timing decides: before the fix this
    * loop caught it in about a third of runs (Arch box), and a longer loop did
    * no better, the leaps all coming early.
    *
    * A leap is judged PER ROUND, so the round that leapt is named. A round
    * takes microseconds, so now() moving by half a hardware period in one is a
    * leap, less whatever the host clock says really passed, where there is a
    * host clock (QEMU). A host stall then cannot pass for a leap, and a board,
    * which has no host clock, still gets the check. */
   constexpr std::uint32_t rounds = 200;

   /* Half a hardware period on the SysTick tickless driver, whose leap is a
    * whole one. The periodic driver counts ticks, so the same constant would
    * be hours of them and the check could not fire. A due sleep there waits a
    * tick or two, so 100 ms is far outside anything honest, and it is the
    * bound on MTIME too, which has no hardware period to leap by. */
   std::uint64_t const leap_bound_us = (tickless && hardware_period != 0u)
      ? time::to_microseconds(time::duration{hardware_period / 2u})
      : 100'000u;

   bool const have_host_time = cyros::bench::elapsed_ns_is_available();
   bool monotonic = true;
   std::uint32_t leaps = 0;
   std::uint64_t worst_excess_us = 0;

   for (std::uint32_t i = 0; i < rounds; ++i) {
      std::uint64_t const ns0 = have_host_time ? cyros::bench::elapsed_ns() : 0u;
      time::time_point const before = time::now();
      switch (i % 4u) {
         case 0:  this_thread::sleep_until(time::now());          break;
         case 1:  this_thread::sleep_until(time::time_point{0});  break;
         case 2:  this_thread::sleep_for(time::duration{1});      break;
         default: this_thread::sleep_for(time::duration{2});      break;
      }
      time::time_point const after = time::now();
      std::uint64_t const host_us = have_host_time ? (cyros::bench::elapsed_ns() - ns0) / 1000u : 0u;

      if (after < before) {
         monotonic = false;
         continue;
      }
      std::uint64_t const guest_us = time::to_microseconds(time::duration_between(after, before));
      std::uint64_t const excess_us = guest_us > host_us ? guest_us - host_us : 0u;
      if (excess_us > worst_excess_us) {
         worst_excess_us = excess_us;
      }
      if (excess_us > leap_bound_us) {
         if (leaps == 0) {
            print_ticks("  first leap in round ", i);
            print_ticks(": now() moved ", guest_us);
            print_ticks(" us, host ", host_us);
            cyros::bench::print(have_host_time ? " us\n" : " us (no host clock)\n");
         }
         ++leaps;
      }
   }

   print_ticks("  rounds that leapt = ", leaps);
   print_ticks(", worst excess over host time = ", worst_excess_us);
   cyros::bench::print(have_host_time ? " us\n" : " us (no host clock, so all of it)\n");

   CYROS_CHECK(monotonic);
   CYROS_CHECK(leaps == 0u);
}

struct sleeper
{
   time::time_point deadline{};
   time::time_point woke{};
   std::uint32_t    order{0};
};

std::uint32_t wakes_so_far = 0;

void sleep_and_record(sleeper& s)
{
   this_thread::sleep_until(s.deadline);
   s.woke = time::now();
   wakes_so_far = wakes_so_far + 1;
   s.order = wakes_so_far;
}

void test_two_sleepers_wake_in_deadline_order()
{
   cyros::bench::start("two sleepers wake from idle in deadline order");

   /* Started latest-deadline first, so creation order and deadline order
    * disagree. The driver has to re-arm for the later deadline from inside
    * the interrupt that delivered the earlier one, with the core idle. */
   wakes_so_far = 0;
   time::time_point const t0 = time::now();
   sleeper late_one{t0 + time::from_milliseconds(30), {}, 0};
   sleeper early_one{t0 + time::from_milliseconds(10), {}, 0};

   thread late_thread([&late_one] { sleep_and_record(late_one); },
                      late_stack, thread::priority(1), core0);
   thread early_thread([&early_one] { sleep_and_record(early_one); },
                       early_stack, thread::priority(1), core0);
   late_thread.join();
   early_thread.join();

   print_ticks("  early woke ", early_one.woke.value - early_one.deadline.value);
   print_ticks(" ticks late, order ", early_one.order);
   print_ticks(". late woke ", late_one.woke.value - late_one.deadline.value);
   print_ticks(" ticks late, order ", late_one.order);
   cyros::bench::print("\n");

   CYROS_CHECK(early_one.order == 1u);
   CYROS_CHECK(late_one.order == 2u);
   CYROS_CHECK(early_one.woke >= early_one.deadline);
   CYROS_CHECK(late_one.woke >= late_one.deadline);
   CYROS_CHECK(late_one.woke.value - late_one.deadline.value
               <= allowed_lateness(time::duration_between(late_one.deadline, t0)));
}

void worker()
{
   /* The periodic driver takes a tick rate. The tickless one converts against
    * this argument and counts counter cycles, so it takes the counter rate. */
   time::initialise(tickless ? counter_clock_hz() : tick_hz);
   time::start();

   test_the_driver_is_the_one_the_config_names();
   test_a_sleeping_thread_lets_the_core_wait_in_idle();
   test_a_sleep_wakes_from_idle_on_time();
   test_sleeps_already_due_return_and_the_clock_stays_honest();
   test_two_sleepers_wake_in_deadline_order();

   cyros::bench::finish();
}

} // namespace

extern "C" int cyros_bench_main()
{
   cyros::bench::print(tickless ? "idle, tickless\n\n" : "idle, periodic\n\n");

   kernel::initialise();
   thread worker_thread(worker, worker_stack, thread::priority(0), core0);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
