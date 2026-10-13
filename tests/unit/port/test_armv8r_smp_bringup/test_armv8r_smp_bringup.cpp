/**
 * @file test_armv8r_smp_bringup.cpp
 * @brief Two Cortex-R52s, one kernel (L2). Runs on QEMU's mps3-an536,
 *        `-smp 2`.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: multicore bring-up on the armv8r core layer: the board's park, the
 *          target's release (start_cores setting the release word and sending
 *          an event) and cyros_port_secondary_core_entry.
 * Trusts:  the single-core port, layers 0 to 2 (test_armv8r_port,
 *          test_armv8r_bringup).
 * Proves:  that core 1 is released, joins the same kernel and runs a thread
 *          pinned to it, and that it ran the per-core init: its own
 *          redistributor awake with the reschedule enabled, its CPU interface
 *          taking Group 1 with the priority mask open, its own interrupt
 *          stack, its TLS pointer its own, and its own MPU guarding the stack
 *          of the thread it runs.
 *
 * The R-profile sibling of test_riscv_smp_bringup. A secondary core that never
 * ran the per-core init comes up with reset state, a sleeping redistributor
 * and a closed CPU interface, and would simply never take a reschedule. Most
 * of that state is private to each core (a redistributor's SGI registers are
 * banked by which redistributor is addressed, the CPU interface is system
 * registers), so each thread reads it on its own core.
 *
 * The interrupt stack is checked directly, as on RISC-V: each core's handlers
 * run on the Supervisor-mode stack its first thread started from, and two
 * cores sharing one would corrupt each other's handler frames on the first
 * pair of simultaneous interrupts, which a bring-up test does not provoke.
 *
 * Ending: as test_riscv_smp_bringup, core 0 reports, after a bounded wait on a
 * flag core 1 sets once and never clears.
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

static_assert(config::cores == 2, "This bring-up is dual core");
static_assert(CYROS_PORT_CORE_COUNT == 2, "Needs the cortex_r52_smp port");

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 2048;

alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_core0[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_core1[stack_size];

/* What each core's thread saw of its own core, read from the thread itself. */
struct core_view
{
   std::uint32_t id;
   std::uint32_t aff0;
   std::uint32_t vbar;
   std::uint32_t waker;
   std::uint32_t isenabler0;
   std::uint32_t sgi0_priority;
   std::uint32_t pmr;
   std::uint32_t igrpen1;
   std::uint32_t sp_svc;
   std::uintptr_t tls;
   std::uint32_t sctlr;
   std::uint32_t prbar15;
   std::uint32_t prlar15;
};

core_view views[2] = {};
std::atomic<bool> core1_finished{false};

std::uint32_t read_mpidr()   { std::uint32_t v; asm volatile("mrc p15, 0, %0, c0, c0, 5" : "=r"(v)); return v; }
std::uint32_t read_vbar()    { std::uint32_t v; asm volatile("mrc p15, 0, %0, c12, c0, 0" : "=r"(v)); return v; }
std::uint32_t read_cbar()    { std::uint32_t v; asm volatile("mrc p15, 1, %0, c15, c3, 0" : "=r"(v)); return v; }
std::uint32_t read_pmr()     { std::uint32_t v; asm volatile("mrc p15, 0, %0, c4, c6, 0" : "=r"(v)); return v; }
std::uint32_t read_igrpen1() { std::uint32_t v; asm volatile("mrc p15, 0, %0, c12, c12, 7" : "=r"(v)); return v; }
std::uint32_t read_sp_svc()  { std::uint32_t v; asm volatile("mrs %0, sp_svc" : "=r"(v)); return v; }
std::uint32_t read_sctlr()   { std::uint32_t v; asm volatile("mrc p15, 0, %0, c1, c0, 0" : "=r"(v)); return v; }
std::uint32_t read_prbar15() { std::uint32_t v; asm volatile("mrc p15, 0, %0, c6, c15, 4" : "=r"(v)); return v; }
std::uint32_t read_prlar15() { std::uint32_t v; asm volatile("mrc p15, 0, %0, c6, c15, 5" : "=r"(v)); return v; }

/* The guard the core layer gives a thread on `buffer` (its file comment):
 * 128 bytes from the bottom rounded up to the MPU's 64-byte granule. */
std::uint32_t guard_base(std::byte const* buffer)
{
   auto const base = reinterpret_cast<std::uintptr_t>(buffer);
   return static_cast<std::uint32_t>((base + 63u) & ~std::uintptr_t{63u});
}

std::uint32_t load(std::uintptr_t address) { return *reinterpret_cast<volatile std::uint32_t*>(address); }

void look_at_this_core(core_view& view)
{
   view.id   = this_core::id();
   view.aff0 = read_mpidr() & 0xFFu;
   view.vbar = read_vbar();

   /* This core's redistributor: the R52 puts them 1 MB above PERIPHBASE,
    * 128 kB apart, with the SGI frame 64 kB in. */
   std::uintptr_t const rd = (read_cbar() & 0xFFE00000u) + 0x100000u + view.aff0 * 0x20000u;
   view.waker         = load(rd + 0x14u);
   view.isenabler0    = load(rd + 0x10000u + 0x100u);
   view.sgi0_priority = *reinterpret_cast<volatile std::uint8_t*>(rd + 0x10000u + 0x400u);

   view.pmr     = read_pmr();
   view.igrpen1 = read_igrpen1();
   view.sp_svc  = read_sp_svc();
   view.tls     = reinterpret_cast<std::uintptr_t>(cyros_port_get_tls_pointer());
   view.sctlr   = read_sctlr();
   view.prbar15 = read_prbar15();
   view.prlar15 = read_prlar15();
}

/* Bounded in yields, as test_riscv_smp_bringup's, so a core that never joins
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
   cyros::bench::print(" waker ");
   cyros::bench::print_hex(view.waker);
   cyros::bench::print(" isenabler0 ");
   cyros::bench::print_hex(view.isenabler0);
   cyros::bench::print(" pmr ");
   cyros::bench::print_hex(view.pmr);
   cyros::bench::print(" sp_svc ");
   cyros::bench::print_hex(view.sp_svc);
   cyros::bench::print(" tls ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(view.tls));
   cyros::bench::print("\n");
}

void thread_on_core0()
{
   look_at_this_core(views[0]);
   bool const rendezvous = wait_for_core1();

   cyros::bench::start("the second core reached its thread");
   CYROS_CHECK(rendezvous);

   cyros::bench::start("the kernel reports two cores");
   CYROS_CHECK_EQ(kernel::core_count(), 2u);

   core_view const& c0 = views[0];
   core_view const& c1 = views[1];
   print_view("  core 0:", c0);
   print_view("  core 1:", c1);

   cyros::bench::start("each thread runs on the core it was pinned to");
   CYROS_CHECK_EQ(c0.id, 0u);
   CYROS_CHECK_EQ(c1.id, 1u);
   CYROS_CHECK_EQ(c0.aff0, 0u);
   CYROS_CHECK_EQ(c1.aff0, 1u);

   cyros::bench::start("each core takes its exceptions through the board's table");
   CYROS_CHECK(c0.vbar != 0u);
   CYROS_CHECK_EQ(c1.vbar, c0.vbar);

   /* ProcessorSleep and ChildrenAsleep clear: the redistributor forwards. */
   cyros::bench::start("each core's redistributor is awake, the reschedule enabled");
   CYROS_CHECK_EQ(c0.waker & 6u, 0u);
   CYROS_CHECK_EQ(c1.waker & 6u, 0u);
   CYROS_CHECK_EQ(c0.isenabler0 & 1u, 1u);
   CYROS_CHECK_EQ(c1.isenabler0 & 1u, 1u);
   CYROS_CHECK_EQ(c1.sgi0_priority, c0.sgi0_priority);

   cyros::bench::start("each core's CPU interface takes Group 1, the reschedule let through");
   CYROS_CHECK_EQ(c0.igrpen1 & 1u, 1u);
   CYROS_CHECK_EQ(c1.igrpen1 & 1u, 1u);
   CYROS_CHECK(c0.sgi0_priority < c0.pmr);
   CYROS_CHECK(c1.sgi0_priority < c1.pmr);

   /* Supervisor mode's SP is the top of the core's interrupt stack whenever no
    * handler is running, which is always, from a thread. Apart, not merely
    * different: each grows down from its top, and 4 kB is several times what
    * the deepest handler chain here uses. */
   std::uint32_t const apart = c0.sp_svc > c1.sp_svc ? c0.sp_svc - c1.sp_svc : c1.sp_svc - c0.sp_svc;
   cyros::bench::start("each core has an interrupt stack of its own");
   cyros::bench::print("  the two interrupt stack tops are ");
   cyros::bench::print_hex(apart);
   cyros::bench::print(" bytes apart\n");
   CYROS_CHECK(c0.sp_svc != 0u);
   CYROS_CHECK(c1.sp_svc != 0u);
   CYROS_CHECK(apart >= 4096u);

   cyros::bench::start("each thread has its own TLS pointer");
   CYROS_CHECK(c0.tls != 0u);
   CYROS_CHECK(c1.tls != 0u);
   CYROS_CHECK(c0.tls != c1.tls);

   /* The MPU is per core, so core 1's guard exists only if its own bring-up
    * turned the MPU on and its own switch placed the region. */
   cyros::bench::start("each core guards the stack of the thread it runs");
   CYROS_CHECK_EQ(c0.sctlr & 1u, 1u);
   CYROS_CHECK_EQ(c1.sctlr & 1u, 1u);
   CYROS_CHECK_EQ(c0.prbar15 & ~63u, guard_base(stack_core0));
   CYROS_CHECK_EQ(c1.prbar15 & ~63u, guard_base(stack_core1));
   CYROS_CHECK_EQ(c0.prlar15 & 1u, 1u);
   CYROS_CHECK_EQ(c1.prlar15 & 1u, 1u);

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("armv8r two-core kernel bring-up\n\n");

   /* Core 1 is parked by the board until start_cores, so only core 0 runs
    * here. */
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
