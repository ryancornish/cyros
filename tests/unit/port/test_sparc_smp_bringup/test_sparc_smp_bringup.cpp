/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_sparc_smp_bringup.cpp
 * @brief Two LEON3s, one kernel (L2). Runs on QEMU's leon3_generic, `-smp 2`.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: multicore bring-up on the sparcv8 core layer: the target's start
 *          (start_cores writing the IRQMP's multiprocessor status), the
 *          board's reset path on the second core, and
 *          cyros_port_secondary_core_entry.
 * Trusts:  the single-core port, layers 0 to 2 (test_sparc_port,
 *          test_sparc_bringup).
 * Proves:  that core 1 is started, joins the same kernel and runs a thread
 *          pinned to it, and that it ran the per-core init: the trap table in
 *          its TBR, one window invalid, the reschedule routed to it in its own
 *          IRQMP mask with none left pending, its own interrupt stack, and its
 *          TLS pointer its own.
 *
 * The SPARC sibling of test_armv8r_smp_bringup. A LEON3 that never ran the
 * per-core init would never take a reschedule: its mask register is its own.
 * The interrupt stack is checked from the device hook, the application's
 * view of where its handlers run: each thread forces an otherwise unused
 * level on its own core, and the hook records its sp.
 *
 * Ending: as the other SMP bring-ups, core 0 reports, after a bounded wait on
 * a flag core 1 sets once and never clears.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_core.h>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>

using namespace cyros;

/* The port's trap table (sparcv8.hpp), which the board points TBR at. */
extern "C" std::uint32_t const cyros_sparc_trap_table[];

static_assert(config::cores == 2, "This bring-up is dual core");
static_assert(CYROS_PORT_CORE_COUNT == 2, "Needs the leon3_smp port");

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 2048;

alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_core0[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_core1[stack_size];

/* QEMU's leon3_generic: the IRQMP's per-core mask and force registers. */
constexpr std::uintptr_t irqmp        = 0x80000200u;
constexpr std::uint32_t  device_level = 5u;

volatile std::uint32_t& reg(std::uintptr_t address) { return *reinterpret_cast<volatile std::uint32_t*>(address); }

std::uint32_t read_psr()   { std::uint32_t v; asm volatile("rd %%psr, %0" : "=r"(v) :: "memory"); return v; }
std::uint32_t read_wim()   { std::uint32_t v; asm volatile("rd %%wim, %0" : "=r"(v) :: "memory"); return v; }
std::uint32_t read_tbr()   { std::uint32_t v; asm volatile("rd %%tbr, %0" : "=r"(v) :: "memory"); return v; }
std::uint32_t read_asr17() { std::uint32_t v; asm volatile("rd %%asr17, %0" : "=r"(v) :: "memory"); return v; }

/* What each core's thread saw of its own core, read from the thread itself. */
struct core_view
{
   std::uint32_t id;
   std::uint32_t index;
   std::uint32_t tbr;
   std::uint32_t wim;
   std::uint32_t psr;
   std::uint32_t mask;
   std::uint32_t force;
   std::uintptr_t handler_sp;
   std::uintptr_t thread_sp;
   std::uintptr_t tls;
};

core_view views[2] = {};
std::atomic<bool> core1_finished{false};

volatile std::uintptr_t device_sp[2] = {};

} // namespace

/* The application's device hook: where this core's handlers run. */
extern "C" void cyros_sparc_device_interrupt(std::uint32_t level)
{
   if (level == device_level) {
      device_sp[read_asr17() >> 28] = reinterpret_cast<std::uintptr_t>(cyros_port_get_stack_pointer());
   }
}

namespace
{

void look_at_this_core(core_view& view)
{
   view.id    = this_core::id();
   view.index = read_asr17() >> 28;
   view.tbr   = read_tbr() & ~0xFFFu;
   view.wim   = read_wim();
   view.psr   = read_psr();
   view.mask  = reg(irqmp + 0x40u + view.index * 4u);
   view.force = reg(irqmp + 0x80u + view.index * 4u);
   view.tls   = reinterpret_cast<std::uintptr_t>(cyros_port_get_tls_pointer());
   view.thread_sp = reinterpret_cast<std::uintptr_t>(cyros_port_get_stack_pointer());

   /* Route the device level to this core and raise it here. */
   std::uintptr_t const mask = irqmp + 0x40u + view.index * 4u;
   reg(mask) = reg(mask) | (1u << device_level);
   reg(irqmp + 0x80u + view.index * 4u) = 1u << device_level;
   for (int i = 0; i < 64 && device_sp[view.index] == 0u; ++i) { asm volatile("nop"); }
   reg(mask) = reg(mask) & ~(1u << device_level);
   view.handler_sp = device_sp[view.index];
}

/* Bounded in yields, as the other SMP bring-ups, so a core that never joins
 * is reported rather than timed out. */
constexpr std::uint32_t rendezvous_limit = 100'000u;

bool wait_for_core1()
{
   for (std::uint32_t spin = 0; spin < rendezvous_limit; ++spin) {
      if (core1_finished.load(std::memory_order_acquire)) { return true; }
      this_thread::yield();
   }
   return false;
}

void thread_on_core1()
{
   look_at_this_core(views[1]);
   core1_finished.store(true, std::memory_order_release);
}

void print_view(char const* label, core_view const& view)
{
   cyros::bench::print(label);
   cyros::bench::print(" id ");
   cyros::bench::print_hex(view.id);
   cyros::bench::print(" wim ");
   cyros::bench::print_hex(view.wim);
   cyros::bench::print(" psr ");
   cyros::bench::print_hex(view.psr);
   cyros::bench::print(" mask ");
   cyros::bench::print_hex(view.mask);
   cyros::bench::print(" handler sp ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(view.handler_sp));
   cyros::bench::print(" tls ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(view.tls));
   cyros::bench::print("\n");
}

void thread_on_core0()
{
   look_at_this_core(views[0]);
   bool const rendezvous = wait_for_core1();

   core_view const& c0 = views[0];
   core_view const& c1 = views[1];
   print_view("core 0:", c0);
   print_view("core 1:", c1);

   cyros::bench::start("the second core reached its thread");
   CYROS_CHECK(rendezvous);

   cyros::bench::start("the kernel reports two cores");
   CYROS_CHECK_EQ(kernel::core_count(), 2u);

   cyros::bench::start("each thread runs on the core it was pinned to");
   CYROS_CHECK_EQ(c0.id, 0u);
   CYROS_CHECK_EQ(c1.id, 1u);
   CYROS_CHECK_EQ(c0.index, 0u);
   CYROS_CHECK_EQ(c1.index, 1u);

   cyros::bench::start("each core traps through the port's table, one window invalid");
   CYROS_CHECK_EQ(c0.tbr, reinterpret_cast<std::uintptr_t>(cyros_sparc_trap_table));
   CYROS_CHECK_EQ(c1.tbr, c0.tbr);
   CYROS_CHECK(c0.wim != 0u && (c0.wim & (c0.wim - 1u)) == 0u);
   CYROS_CHECK(c1.wim != 0u && (c1.wim & (c1.wim - 1u)) == 0u);

   cyros::bench::start("each core runs its thread supervisor, traps on, at PIL 0");
   CYROS_CHECK_EQ(c0.psr & 0xA0u, 0xA0u);
   CYROS_CHECK_EQ(c1.psr & 0xA0u, 0xA0u);
   CYROS_CHECK_EQ((c0.psr >> 8) & 0xFu, 0u);
   CYROS_CHECK_EQ((c1.psr >> 8) & 0xFu, 0u);

   cyros::bench::start("each core has the reschedule routed to it, none left pending");
   CYROS_CHECK_EQ((c0.mask >> 1) & 1u, 1u);
   CYROS_CHECK_EQ((c1.mask >> 1) & 1u, 1u);
   CYROS_CHECK_EQ((c0.force >> 1) & 1u, 0u);
   CYROS_CHECK_EQ((c1.force >> 1) & 1u, 0u);

   cyros::bench::start("each core has an interrupt stack of its own, off its thread's");
   std::uintptr_t const apart = c0.handler_sp > c1.handler_sp ? c0.handler_sp - c1.handler_sp
                                                              : c1.handler_sp - c0.handler_sp;
   CYROS_CHECK(c0.handler_sp != 0u);
   CYROS_CHECK(c1.handler_sp != 0u);
   CYROS_CHECK(apart >= 4096u);
   auto const off = [](std::uintptr_t handler, std::uintptr_t thread_sp, std::byte const* stack) {
      auto const base = reinterpret_cast<std::uintptr_t>(stack);
      return (handler < base || handler >= base + stack_size) && handler != thread_sp;
   };
   CYROS_CHECK(off(c0.handler_sp, c0.thread_sp, stack_core0));
   CYROS_CHECK(off(c1.handler_sp, c1.thread_sp, stack_core1));

   cyros::bench::start("each thread has its own TLS pointer");
   CYROS_CHECK(c0.tls != 0u);
   CYROS_CHECK(c1.tls != 0u);
   CYROS_CHECK(c0.tls != c1.tls);

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("sparcv8 two-core kernel bring-up\n\n");

   /* Core 1 is powered down until start_cores, so only core 0 runs here. */
   cyros::bench::start("the bootstrap core identifies itself as core 0");
   CYROS_CHECK_EQ(this_core::id(), 0u);

   kernel::initialise();

   cyros::bench::start("one thread pinned to each core registers");
   thread t0(thread_on_core0, stack_core0, thread::priority(0), core0);
   thread t1(thread_on_core1, stack_core1, thread::priority(0), core1);
   CYROS_CHECK_EQ(kernel::active_threads(), 2u);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
