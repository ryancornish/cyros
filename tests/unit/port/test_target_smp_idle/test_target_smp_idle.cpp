/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_target_smp_idle.cpp
 * @brief Both cores sleep at once, and each is woken out of its own WFI by its
 *        own timer.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the idle path on a two-core port (cortex_m33_smp, riscv_virt_smp)
 *          when the wake is a timer: a sleep on core N is serviced by core N's
 *          timer (SysTick, or the hart's MTIMECMP), which has to wake core N
 *          out of WFI, while the other core is doing the same. Built against
 *          the periodic driver here, and against the tickless one by
 *          test_target_smp_idle_tickless, which only RISC-V's shared MTIME
 *          allows on two cores (the SSE-200 target refuses it).
 * Trusts:  layers 0 to 7, the per-core ticks that test_target_smp_time
 *          proves from spinning threads, and the doorbell wake out of WFI that
 *          test_target_smp_ipi already covers, idle included.
 * Proves:  that each core reaches idle with interrupts enabled, waits there
 *          rather than spinning, and wakes its own sleeper no earlier than the
 *          deadline and not much later, with both cores asleep together, and
 *          that a deadline armed on one core leaves the other core's alone.
 *
 *
 * What is new against the IPI test
 * ================================
 * test_target_smp_ipi parks each core in WFI and wakes it by doorbell.
 * Nothing on this target slept before this test, so no core was ever woken
 * out of WFI by its OWN timer, which is the path port_time_systick_smp.cpp
 * adds: every core ticks for itself and services its own timers, while only
 * core 0 advances the shared count.
 *
 * Idle is observed as in test_target_idle, through a link-time wrap of
 * cyros_port_idle, counted per core.
 *
 *
 * Each core's deadline is its own
 * ================================
 * The sleeps above are short and alike on both cores, so a deadline one core
 * overwrote on the other would cost milliseconds, inside their tolerance: the
 * rp2040_smp target with both cores on one TIMER alarm passed them
 * (arm-port-notes.md 19h). The last phase arms a near deadline on one core
 * and then a far one on the other, each way round. Sharing one compare, the
 * far arm replaces the near one and the near sleeper wakes with the far.
 *
 *
 * How this fails
 * ==============
 * A sleep whose wake never comes leaves its core in WFI with nothing to wake
 * it, so the case hangs and the runner's timeout reports it. Core 0 prints
 * which phase it reached. A core that never reports is caught by name: core 0
 * gives up waiting for it after a bounded number of sleeps.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/time/time.hpp>
#include <cyros/chrono/chrono.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>

using namespace cyros;

static_assert(config::cores == 2, "This test is dual core");
static_assert(CYROS_PORT_CORE_COUNT == 2, "Needs a two-core port");

/* Which driver this build has, from the config header, as in
 * test_target_idle. A tickless port tick is one counter count, so the rate
 * the driver is given is the counter's, a board fact. Only an MTIME port has
 * tickless on two cores, and the board's toolchain says it has one
 * (CYROS_BENCH_MTIME). */
#if defined(CYROS_BENCH_MTIME)
extern "C" std::uint32_t cyros_port_mtime_clock_hz(void);
#endif

namespace
{

constexpr bool tickless = CYROS_TEST_IDLE_TICKLESS != 0;

constexpr std::uint32_t tick_hz = 1'000;

std::uint32_t driver_rate()
{
#if defined(CYROS_BENCH_MTIME)
   return tickless ? cyros_port_mtime_clock_hz() : tick_hz;
#else
   static_assert(!tickless, "the cortex_m33_smp target refuses tickless (arm-port-notes.md 16f)");
   return tick_hz;
#endif
}

constexpr std::size_t stack_size = thread::min_stack_size + 2048;
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_core0[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_core1[stack_size];

/* Indexed by core, each written only by that core's idle thread. */
std::atomic<std::uint32_t> idle_entries[2]{};
std::atomic<std::uint32_t> idle_entries_masked[2]{};

} // namespace

extern "C" void __real_cyros_port_idle(void);

extern "C" void __wrap_cyros_port_idle(void)
{
   std::uint32_t const core = this_core::id();
   idle_entries[core].fetch_add(1u, std::memory_order_relaxed);
   if (!cyros_port_interrupts_enabled()) {
      idle_entries_masked[core].fetch_add(1u, std::memory_order_relaxed);
   }
   __real_cyros_port_idle();
}

namespace
{

/* What one core measured. Core 1 fills its own and publishes it with
 * `core1_done`, core 0 reads it after acquiring that flag. */
struct core_report
{
   std::uint32_t ran_on{0xFFFFFFFFu};
   std::uint32_t entries{0};
   std::uint32_t allowed{0};
   std::uint32_t masked{0};
   std::uint32_t early_wakes{0};
   std::uint32_t late_wakes{0};
   std::uint64_t worst_late{0};
};

core_report reports[2];
std::atomic<bool> core1_done{false};

/* The last phase (see the header). Round r puts core r near and the other
 * core far. Each flag holds the round it last reached, plus one. */
constexpr std::uint32_t near_ms = 20;
constexpr std::uint32_t far_ms  = 200;
std::atomic<std::uint32_t> far_ready{0};
std::atomic<std::uint32_t> near_announced{0};
std::atomic<std::uint32_t> far_arming{0};
std::atomic<std::uint32_t> far_done{0};
std::uint64_t near_late[2] = {};
bool near_early[2] = {};
bool near_overlapped[2] = {};

/* Polls by sleeping, bounded, so a core that never arrives fails by name. */
bool wait_for(std::atomic<std::uint32_t> const& flag, std::uint32_t value)
{
   for (std::uint32_t i = 0; i < 2'000u; ++i) {
      if (flag.load(std::memory_order_acquire) == value) { return true; }
      this_thread::sleep_for(time::from_milliseconds(1));
   }
   return false;
}

/* Polls by spinning, for the far core, which must arm well inside the near
 * sleep. A sleep here would be a deadline of its own, and one the defect
 * under test could stretch. */
bool spin_for(std::atomic<std::uint32_t> const& flag, std::uint32_t value)
{
   time::time_point const limit = time::now() + time::from_milliseconds(2'000);
   while (flag.load(std::memory_order_acquire) != value) {
      if (time::now() > limit) { return false; }
      this_core::cpu_relax();
   }
   return true;
}

void deadlines_are_per_core()
{
   std::uint32_t const core = this_core::id();
   for (std::uint32_t round = 0; round < 2u; ++round) {
      if (round == core) {
         /* Near: sleep once the far core is listening. */
         if (!wait_for(far_ready, round + 1u)) { return; }
         near_announced.store(round + 1u, std::memory_order_release);
         time::time_point const deadline = time::now() + time::from_milliseconds(near_ms);
         this_thread::sleep_until(deadline);
         time::time_point const woke = time::now();
         near_early[core] = woke < deadline;
         near_late[core] = near_early[core] ? 0u : woke.value - deadline.value;
         /* The far core armed while this core slept, or the case proved
          * nothing: a far core slower than the near sleep arms after it. */
         near_overlapped[core] = far_arming.load(std::memory_order_acquire) == round + 1u;
         if (!wait_for(far_done, round + 1u)) { return; }
      } else {
         /* Far: arm after the near core has, well inside its sleep. */
         far_ready.store(round + 1u, std::memory_order_release);
         if (!spin_for(near_announced, round + 1u)) { return; }
         time::time_point const settle = time::now() + time::from_milliseconds(2);
         while (time::now() < settle) { this_core::cpu_relax(); }
         far_arming.store(round + 1u, std::memory_order_release);
         this_thread::sleep_until(time::now() + time::from_milliseconds(far_ms));
         far_done.store(round + 1u, std::memory_order_release);
      }
   }
}

/* As in test_target_idle: early is never allowed, and late only has to stay
 * clear of waiting for the wrong interrupt. */
std::uint64_t allowed_lateness(time::duration d)
{
   return (d.value / 4u) + time::from_milliseconds(5).value + 2u;
}

void sleep_and_measure(core_report& r)
{
   std::uint32_t const core = this_core::id();
   r.ran_on = core;

   /* Waiting rather than spinning: ten sleeps of 20 ms. Periodic, one idle
    * entry per tick on this core's own timer, so about 200. Tickless, one per
    * deadline, so about 10. */
   constexpr std::uint32_t sleeps = 10;
   time::duration const d = time::from_milliseconds(20);
   std::uint32_t const entries_before = idle_entries[core].load(std::memory_order_relaxed);
   std::uint32_t const masked_before  = idle_entries_masked[core].load(std::memory_order_relaxed);
   for (std::uint32_t i = 0; i < sleeps; ++i) {
      this_thread::sleep_for(d);
   }
   r.entries = idle_entries[core].load(std::memory_order_relaxed) - entries_before;
   std::uint32_t const interrupts = tickless ? 1u : static_cast<std::uint32_t>(d.value + 1u);
   r.allowed = sleeps * ((2u * interrupts) + 4u);
   r.masked  = idle_entries_masked[core].load(std::memory_order_relaxed) - masked_before;

   /* On time, from idle. Different lengths on the two cores, so their wakes do
    * not line up and each core's tick delivers its own. */
   std::uint32_t const lengths_ms[] = {1, 7, 50, core == 0u ? 33u : 41u};
   for (std::uint32_t ms : lengths_ms) {
      time::duration const length = time::from_milliseconds(ms);
      time::time_point const deadline = time::now() + length;
      this_thread::sleep_until(deadline);
      time::time_point const woke = time::now();
      if (woke < deadline) {
         ++r.early_wakes;
         continue;
      }
      std::uint64_t const late = woke.value - deadline.value;
      if (late > r.worst_late) {
         r.worst_late = late;
      }
      if (late > allowed_lateness(length)) {
         ++r.late_wakes;
      }
   }
}

void thread_on_core1()
{
   time::start();
   sleep_and_measure(reports[1]);
   deadlines_are_per_core();
   core1_done.store(true, std::memory_order_release);
}

void report(std::uint32_t core, core_report const& r)
{
   cyros::bench::print("  core ");
   cyros::bench::print_dec(core);
   cyros::bench::print(": idle entries = ");
   cyros::bench::print_dec(r.entries);
   cyros::bench::print(" (allowed ");
   cyros::bench::print_dec(r.allowed);
   cyros::bench::print("), worst wake ");
   cyros::bench::print_dec(r.worst_late);
   cyros::bench::print(" ticks late\n");

   CYROS_CHECK_EQ(r.ran_on, core);
   CYROS_CHECK(r.entries >= 10u);          // every sleep reached the port's idle
   CYROS_CHECK(r.entries <= r.allowed);    // and waited there, once per interrupt
   CYROS_CHECK_EQ(r.masked, 0u);           // with interrupts enabled
   CYROS_CHECK_EQ(r.early_wakes, 0u);
   CYROS_CHECK_EQ(r.late_wakes, 0u);
}

void thread_on_core0()
{
   time::start();
   CYROS_CHECK_EQ(cyros_port_time_freq_hz(), std::uint64_t{driver_rate()});

   cyros::bench::print("  both cores sleeping...\n");
   sleep_and_measure(reports[0]);
   deadlines_are_per_core();

   /* Waiting for core 1 by sleeping, so core 0 keeps idling. Bounded, so a
    * core 1 that never finishes fails by name. */
   bool core1_finished = false;
   for (std::uint32_t i = 0; i < 2'000u && !core1_finished; ++i) {
      core1_finished = core1_done.load(std::memory_order_acquire);
      if (!core1_finished) {
         this_thread::sleep_for(time::from_milliseconds(1));
      }
   }

   cyros::bench::start("core 1 finished its sleeps");
   CYROS_CHECK(core1_finished);

   cyros::bench::start("core 0 idled in WFI and its own tick woke its sleeps on time");
   report(0u, reports[0]);

   if (core1_finished) {
      cyros::bench::start("core 1 idled in WFI and its own tick woke its sleeps on time");
      report(1u, reports[1]);

      cyros::bench::start("a far deadline armed on one core leaves the other's near one alone");
      for (std::uint32_t c = 0; c < 2u; ++c) {
         cyros::bench::print("  core ");
         cyros::bench::print_dec(c);
         cyros::bench::print(" near wake, the other core's far arm after it: ");
         cyros::bench::print_dec(near_late[c]);
         cyros::bench::print(" ticks late\n");
         CYROS_CHECK(near_overlapped[c]);
         CYROS_CHECK(!near_early[c]);
         CYROS_CHECK(near_late[c] <= allowed_lateness(time::from_milliseconds(near_ms)));
      }
   }

   cyros::bench::finish();
}

} // namespace

extern "C" int cyros_bench_main()
{
   cyros::bench::print(tickless ? "two-core idle, tickless, both cores asleep\n\n"
                                : "two-core idle, periodic, both cores asleep\n\n");

   kernel::initialise();
   time::initialise(driver_rate());

   thread t0(thread_on_core0, stack_core0, thread::priority(0), core0);
   thread t1(thread_on_core1, stack_core1, thread::priority(0), core1);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
