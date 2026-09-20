/**
 * @file test_cortex_m33_systick.cpp
 * @brief The SysTick time source, and masking under a real interrupt.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the cortex_m33 port's `port_time.h` implementation, driving the
 *          periodic time driver from a real SysTick interrupt.
 * Trusts:  layer 0 (the port's masking contract) and the kernel bring-up that
 *          layer 2 proves, which is why this test declares a harness debt.
 * Proves:  that SysTick advances a monotonic clock, and - the part that could
 *          not be written before now - that preemption-disable leaves that
 *          clock running while interrupt-masking stops it.
 *
 *
 * THE CHECK THIS TEST EXISTS FOR
 * ==============================
 * `port.h` draws a sharp line between its two facilities: interrupt masking
 * blocks the hardware, preemption disabling blocks only the scheduler, and
 * "ISRs still fire and run" while preemption is off. On the Linux ports that
 * distinction is a pair of depth counters feeding a signal mask, and the
 * consequence of getting it wrong is subtle. Here it is directly observable:
 * if preemption-disable were implemented with PRIMASK rather than BASEPRI, or
 * if SysTick had been given the same priority level as PendSV, then every
 * critical section in the kernel would silently stop the clock.
 *
 * `test_cortex_m33_port` checks the priority NUMBERS are distinct. This checks
 * the consequence, with an interrupt actually firing.
 *
 *
 * WHY THE HARNESS DEBT
 * ====================
 * `time::start()` is per-core and documents itself as running after
 * `kernel::start()` in a core's context. So the time source cannot be observed
 * until the kernel is up, even though its subject sits at layer 1. That is the
 * same shape as `test_spinlock` on the host side, and it gets the same
 * treatment rather than being quietly relabelled as a layer-2 test.
 *
 *
 * WHAT IS NOT CHECKED, AND WHY
 * ============================
 * The ABSOLUTE tick rate. `cyros_port_systick_clock_hz` is a board fact an
 * application supplies, the port's default is a placeholder, and QEMU's TCG
 * does not model real time anyway, so a wall-clock assertion here would be
 * measuring the emulator's scheduling rather than the port. Everything below
 * is expressed in ticks and in orderings, which are the properties the port is
 * actually responsible for.
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

namespace
{

constexpr std::uint32_t tick_hz = 10'000;

constexpr std::size_t stack_size = thread::min_stack_size + 2048;
alignas(CYROS_PORT_STACK_ALIGN) std::byte worker_stack[stack_size];

/* Bounded, so a stopped clock fails the test instead of hanging the suite.
 * Generous because the guest's notion of elapsed time is QEMU's business. */
constexpr std::uint32_t spin_limit = 40'000'000;

/**
 * @brief Spin until the clock moves, or give up.
 * @return the number of ticks it advanced, 0 if it never did.
 */
std::uint64_t spin_for_a_tick()
{
   std::uint64_t const start = time::now().value;
   for (std::uint32_t i = 0; i < spin_limit; ++i) {
      std::uint64_t const current = time::now().value;
      if (current != start) { return current - start; }
   }
   return 0;
}

/**
 * @brief Burn `iterations` of a loop the compiler may not remove.
 *
 * An empty asm barrier rather than a volatile counter: incrementing a volatile
 * is deprecated since C++20 and -Werror rejects it.
 */
void burn(std::uint32_t iterations)
{
   for (std::uint32_t i = 0; i < iterations; ++i) {
      asm volatile("" ::: "memory");
   }
}

/* How many iterations of `burn` reliably span at least one tick, measured at
 * runtime rather than guessed. Set by calibrate() below. */
std::uint32_t iterations_per_tick = 0;

/**
 * @brief Find out how long a FULL tick period is, in units of `burn`.
 *
 * This has to be measured. The masked test below needs a spin KNOWN to be
 * longer than a tick, so that "the clock did not advance" means the clock was
 * stopped rather than that the spin was too short.
 *
 * TWO BUGS HAVE ALREADY LIVED HERE, both producing false failures:
 *
 *  - A hardcoded 2,000,000 iterations. The guest's speed relative to SysTick is
 *    QEMU's business on the bench and the clock tree's on real silicon, so a
 *    constant cannot know.
 *  - Measuring without ALIGNING FIRST. Counting from wherever the call happens
 *    to land measures the REMAINDER of the current tick, not a whole one, and
 *    landing just before a boundary returns nearly zero. That flaked roughly 1
 *    run in 12, which is exactly the rate that gets mistaken for a real
 *    intermittent kernel defect.
 *
 * So: wait for a boundary, then measure a whole period from it.
 */
void calibrate()
{
   constexpr std::uint32_t step = 200;

   /* Align. Everything measured before this point is a partial period. */
   std::uint64_t const before = time::now().value;
   for (std::uint32_t i = 0; i < spin_limit; ++i) {
      if (time::now().value != before) { break; }
   }

   /* Now a full one. */
   std::uint64_t const base = time::now().value;
   std::uint32_t total = 0;
   while (total < spin_limit) {
      burn(step);
      total += step;
      if (time::now().value != base) { break; }
   }

   /* Eight times, because the margin is free and a false failure is not. */
   iterations_per_tick = (total == 0) ? 0 : total * 8;
}

/**
 * @brief Spin for comfortably longer than a tick, and report the advance.
 *
 * A fixed count rather than "until something happens", because the caller
 * EXPECTS nothing to happen and has to come back rather than spin forever.
 */
std::uint64_t clock_advance_over_a_calibrated_spin()
{
   std::uint64_t const start = time::now().value;
   burn(iterations_per_tick);
   return time::now().value - start;
}


void test_systick_advances_a_monotonic_clock()
{
   cyros::bench::start("SysTick advances the clock");

   time::initialise(tick_hz);
   time::start();

   CYROS_CHECK_EQ(cyros_port_time_freq_hz(), tick_hz);

   std::uint64_t const advanced = spin_for_a_tick();
   CYROS_CHECK(advanced > 0);
   if (advanced == 0) {
      cyros::bench::print("  clock never moved, SysTick is not firing\n");
      return;
   }

   cyros::bench::start("a tick interval can be measured");
   calibrate();
   cyros::bench::print("  iterations per tick (x4 margin) = ");
   cyros::bench::print_hex(iterations_per_tick);
   cyros::bench::print("\n");
   CYROS_CHECK(iterations_per_tick > 0);

   cyros::bench::start("the clock never goes backwards");
   bool monotonic = true;
   std::uint64_t previous = time::now().value;
   for (int i = 0; i < 20'000; ++i) {
      std::uint64_t const current = time::now().value;
      if (current < previous) { monotonic = false; break; }
      previous = current;
   }
   CYROS_CHECK(monotonic);
}

void test_preempt_disable_leaves_the_clock_running()
{
   cyros::bench::start("a preempt-disabled section does NOT stop the clock");

   /* port.h: disabling preemption blocks the scheduler, not the hardware. On
    * this port that is BASEPRI raised to PendSV's level, and SysTick sits a
    * full AIRCR.PRIGROUP preemption group above it so that it keeps being
    * delivered here. "A group above", not "a priority value above": that
    * distinction is the bug this test found. */
   /* Waits for the tick it EXPECTS, rather than burning a guessed duration and
    * asking whether one happened. A test whose success case is "something
    * occurs" should never be able to fail by not waiting long enough. The
    * masked test below keeps the fixed spin, because its success case is that
    * nothing occurs and it therefore has to come back. */
   cyros_mask_token_t const token = cyros_port_preempt_disable();
   std::uint64_t const start = time::now().value;
   std::uint64_t advanced = 0;
   for (std::uint32_t i = 0; i < spin_limit; ++i) {
      advanced = time::now().value - start;
      if (advanced != 0) { break; }
   }
   cyros_port_preempt_enable(token);

   CYROS_CHECK(advanced > 0);
   if (advanced == 0) {
      cyros::bench::print(
         "  the clock stopped inside a critical section. Candidates, in the\n"
         "  order they have actually gone wrong: SysTick and PendSV share an\n"
         "  AIRCR.PRIGROUP preemption group, preemption is masking with PRIMASK\n"
         "  rather than BASEPRI, or SysTick was given PendSV's priority.\n");
   }
}

void test_irq_masking_does_stop_the_clock()
{
   cyros::bench::start("an interrupt-masked section DOES stop the clock");

   /* The other half, and the one that makes the check above mean something. If
    * the clock advanced in both cases, this test would prove nothing about
    * which register preempt-disable uses. */
   cyros_mask_token_t const token = cyros_port_irq_save();
   std::uint64_t const advanced = clock_advance_over_a_calibrated_spin();
   cyros_port_irq_restore(token);

   CYROS_CHECK_EQ(advanced, 0u);

   cyros::bench::start("and the clock resumes once the mask is lifted");
   CYROS_CHECK(spin_for_a_tick() > 0);
}

void worker()
{
   test_systick_advances_a_monotonic_clock();
   test_preempt_disable_leaves_the_clock_running();
   test_irq_masking_does_stop_the_clock();

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("cortex_m33 SysTick time source\n\n");

   kernel::initialise();
   thread worker_thread(worker, worker_stack, thread::priority(0), core0);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
