/**
 * @file test_cortex_m33_smp_time.cpp
 * @brief Time on the dual-M33 target, as BOTH cores see it (L1).
 *
 * Subject: the cortex_m33_smp target's `port_mcu.h` time half, driving the
 *          periodic driver from core 0's SysTick
 * Trusts:  SMP bring-up (L2) as harness, declared as debt in test.toml. The two
 *          cores hand off through release/acquire flags rather than a
 *          semaphore, so the test stands on nothing above layer 2. Spinning is
 *          sound here because each flag is set from the OTHER core.
 * Proves:  that core 1 reads the clock core 0 keeps, that time never runs
 *          backwards across a hand-off between the cores, that a timer fires
 *          whichever core scheduled it, that core 1's own tick still fires
 *          while core 1 has preemption disabled, that the clock runs at its
 *          configured rate with both cores ticking, and that time::finalise()
 *          stops the calling core's SysTick
 *
 *
 * WHY THIS HAD TO EXIST
 * =====================
 * SysTick is core-private, so this target keeps time on core 0 alone and every
 * other core reads a counter core 0's ISR increments (`arm-port-notes.md` 16f).
 * The whole design rests on the claim that the counter "reads correctly from
 * either core". Until 2026-09-24 nothing had ever run time on this target at
 * all: the other SMP tests build the periodic driver and never start it.
 *
 *
 * THE CASE THAT USED TO FAIL
 * ==========================
 * A timer scheduled FROM core 1. The periodic driver files it in core 1's own
 * timetable, which only core 1's tick services. Until 2026-09-25 core 1 had no
 * tick, and a temporary experiment in this file measured such a timer never
 * firing, so every timed wait on core 1 hung. Every core now runs its own
 * SysTick and only core 0's counts (`arm-port-notes.md` 16j), and that timer is
 * asserted below.
 *
 * Core 1 also gets the check the single-core SysTick test makes on core 0: its
 * tick must keep firing under preemption-disable. AIRCR is banked per core and
 * core 1 never took a SysTick before, so the section 6.0 bug class had never
 * been possible there until now.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/time/time.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_mcu.h>
#include <cyros/port/port_traits.h>

#include <common/arm/bench.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>

using namespace cyros;

static_assert(config::cores == 2, "This test is dual core");
static_assert(CYROS_PORT_CORE_COUNT == 2, "Needs the cortex_m33_smp port");

namespace
{

constexpr std::uint32_t tick_hz = 1'000;

constexpr std::size_t stack_size = thread::min_stack_size + 2048;
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_core0[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_core1[stack_size];

/* Bounded, so a stopped clock fails the test instead of hanging the suite.
 * Generous because the guest's notion of elapsed time is QEMU's business. */
constexpr std::uint32_t spin_limit = 40'000'000;

std::atomic<bool> time_is_live{false};   // core 0 -> core 1
std::atomic<bool> core1_done{false};     // core 1 -> core 0

/* Wait for a flag the other core sets. Bounded, so a lost hand-off fails by
 * name rather than hanging the runner. */
bool wait_for(std::atomic<bool> const& flag)
{
   for (std::uint32_t i = 0; i < spin_limit; ++i) {
      if (flag.load(std::memory_order_acquire)) return true;
   }
   return false;
}

/* 64-bit values are PLAIN, not std::atomic: std::atomic<std::uint64_t> is not
 * lock-free on ARMv8-M (no LDREXD) and would need libatomic. Each is written
 * before a release store of a flag and read after the matching acquire load,
 * which is all the ordering they need.
 *
 * Core 0's reading of the clock just before it lets core 1 go. */
std::uint64_t core0_reading = 0;

/* Results from core 1, reported by core 0 so only one core prints. */
std::uint64_t core1_advance = 0;
std::uint64_t core1_first_reading = 0;
std::atomic<std::uint32_t> core1_ran_on{0xFFFFFFFFu};

std::atomic<bool> timer_fired{false};
std::atomic<bool> core1_timer_fired{false};
std::atomic<bool> core1_timer_fired_while_preempt_disabled{false};
std::atomic<bool> core1_preempt_disabled_timer_flag{false};
std::atomic<bool> core1_timer_scheduled{false};

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
 * @brief How far the clock moves across a fixed burn, which is long enough to
 *        span many ticks on the bench. Used before AND after finalise, so the
 *        "stopped" check is measured against a burn proven to see ticks.
 */
std::uint64_t advance_over_a_fixed_burn()
{
   std::uint64_t const start = cyros_port_time_now();
   for (std::uint32_t i = 0; i < 4'000'000; ++i) {
      asm volatile("" ::: "memory");
   }
   return cyros_port_time_now() - start;
}

void on_timer(void*) noexcept
{
   timer_fired.store(true, std::memory_order_relaxed);
}

void on_flag_timer(void* flag) noexcept
{
   static_cast<std::atomic<bool>*>(flag)->store(true, std::memory_order_relaxed);
}

/* Schedule a one-shot three ticks out on the CALLING core and spin until it
 * fires or the budget runs out. The spin is bounded because the failure this
 * guards against is a timer that never fires. */
bool schedule_and_wait_here(std::atomic<bool>& flag)
{
   time::handle const h = time::schedule_at(time::now() + time::duration{3}, &on_flag_timer, &flag);
   if (h.id == 0) return false;
   for (std::uint32_t i = 0; i < spin_limit; ++i) {
      if (flag.load(std::memory_order_relaxed)) return true;
   }
   return false;
}

void thread_on_core1()
{
   core1_ran_on.store(this_core::id(), std::memory_order_relaxed);

   /* Every core starts its own time, as cyros-policies.md asks. Before
    * 2026-09-25 this panicked on core 1 of this target. */
   time::start();

   if (!wait_for(time_is_live)) return;   // core 0 reports the missing result

   /* Read first, then compare with what core 0 read BEFORE releasing us. The
    * release/acquire pair orders them, so a smaller value here would be time
    * running backwards across the hand-off. */
   core1_first_reading = time::now().value;

   /* Core 1 has no SysTick of its own. If this advances, it is reading the
    * counter core 0's ISR keeps, which is the claim under test. */
   core1_advance = spin_for_a_tick();

   /* A timer scheduled HERE, serviced by core 1's own tick. */
   core1_timer_scheduled.store(true, std::memory_order_relaxed);
   (void)schedule_and_wait_here(core1_timer_fired);

   /* And again with preemption disabled on core 1. BASEPRI then masks PendSV's
    * group, and core 1's SysTick has to sit in a group above it to be taken. */
   /* The verdict is taken INSIDE the window. Reading the flag afterwards would
    * pass even with SysTick masked: the tick left pending fires the moment
    * preemption is re-enabled and sets it then. A mutation putting SysTick in
    * PendSV's group got through exactly that way on the first draft. */
   cyros_mask_token_t const token = cyros_port_preempt_disable();
   bool const fired_inside = schedule_and_wait_here(core1_preempt_disabled_timer_flag);
   cyros_port_preempt_enable(token);
   core1_timer_fired_while_preempt_disabled.store(fired_inside, std::memory_order_relaxed);

   core1_done.store(true, std::memory_order_release);
}

void thread_on_core0()
{
   cyros::bench::start("SysTick advances the clock on core 0");
   time::start();
   CYROS_CHECK_EQ(cyros_port_time_freq_hz(), tick_hz);
   CYROS_CHECK(spin_for_a_tick() > 0);

   core0_reading = time::now().value;
   time_is_live.store(true, std::memory_order_release);

   cyros::bench::start("core 1 finished its readings");
   CYROS_CHECK(wait_for(core1_done));

   cyros::bench::start("the other thread really was on core 1");
   CYROS_CHECK_EQ(core1_ran_on.load(std::memory_order_relaxed), 1u);

   cyros::bench::start("core 1 reads the clock core 0 keeps, and sees it advance");
   CYROS_CHECK(core1_advance > 0);

   cyros::bench::start("time does not run backwards across a hand-off between cores");
   CYROS_CHECK(core1_first_reading >= core0_reading);

   cyros::bench::start("a timer scheduled on core 1 fires, serviced by core 1's tick");
   CYROS_CHECK(core1_timer_scheduled.load(std::memory_order_relaxed));
   CYROS_CHECK(core1_timer_fired.load(std::memory_order_relaxed));

   cyros::bench::start("core 1's tick still fires while core 1 has preemption disabled");
   CYROS_CHECK(core1_timer_fired_while_preempt_disabled.load(std::memory_order_relaxed));

   /* Absolute, against semihosting's own clock, and taken while BOTH cores
    * tick. Every relative check here passes if each core's tick advanced the
    * counter, and the clock then ran at twice its rate. */
   cyros::bench::start("the clock runs at the configured rate with both cores ticking");
   if (cyros::bench::elapsed_ns_is_available()) {
      std::uint64_t const t0  = time::now().value;
      std::uint64_t const ns0 = cyros::bench::elapsed_ns();
      while ((cyros::bench::elapsed_ns() - ns0) < 200'000'000ull) { }
      std::uint64_t const ns    = cyros::bench::elapsed_ns() - ns0;
      std::uint64_t const ticks = time::now().value - t0;
      std::uint64_t const measured_hz = (ticks * 1'000'000'000ull) / ns;
      cyros::bench::print("  measured = ");
      cyros::bench::print_hex(static_cast<std::uint32_t>(measured_hz));
      cyros::bench::print(" Hz\n");
      /* Ten per cent either side, the same band the single-core test uses:
       * an emulator is not a frequency standard, and double counting is 100. */
      CYROS_CHECK(measured_hz >= (std::uint64_t{tick_hz} * 90u) / 100u);
      CYROS_CHECK(measured_hz <= (std::uint64_t{tick_hz} * 110u) / 100u);
   } else {
      cyros::bench::print("  no SYS_ELAPSED reference, skipping the absolute check\n");
   }

   cyros::bench::start("a timer scheduled on core 0 fires");
   time::handle const h = time::schedule_at(time::now() + time::duration{3}, &on_timer, nullptr);
   CYROS_CHECK(h.id != 0);
   for (std::uint32_t i = 0; i < spin_limit && !timer_fired.load(std::memory_order_relaxed); ++i) { }
   CYROS_CHECK(timer_fired.load(std::memory_order_relaxed));

   cyros::bench::start("the burn used below spans ticks while time is running");
   CYROS_CHECK(advance_over_a_fixed_burn() > 0);

   cyros::bench::start("time::finalise() stops the calling core's SysTick, and with it the clock");
   time::finalise();
   CYROS_CHECK_EQ(advance_over_a_fixed_burn(), 0u);

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("cortex_m33_smp time, as both cores see it, mps2-an521\n\n");

   kernel::initialise();
   time::initialise(tick_hz);

   thread t0(thread_on_core0, stack_core0, thread::priority(0), core0);
   thread t1(thread_on_core1, stack_core1, thread::priority(0), core1);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
