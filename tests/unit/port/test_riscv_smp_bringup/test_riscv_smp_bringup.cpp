/**
 * @file test_riscv_smp_bringup.cpp
 * @brief Two RISC-V harts, one kernel (L2). Runs on QEMU's virt, `-smp 2`.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: multicore bring-up on the rv32 core layer: the board's park, the
 *          target's release (start_cores raising the soft IRQ) and
 *          cyros_port_secondary_core_entry.
 * Trusts:  the single-hart port, layers 0 to 2 (test_riscv_port,
 *          test_riscv_bringup).
 * Proves:  that hart 1 is released, joins the same kernel and runs a thread
 *          pinned to it, and that it ran the per-hart init: its own trap
 *          vector, its reschedule enabled, its own interrupt stack, its cycle
 *          counter running, and its TLS pointer its own.
 *
 * The RISC-V sibling of test_cortex_m33_smp_bringup, which checks PendSV's
 * priority and the FPU on core 1 for the same reason: a secondary that never
 * ran the per-core init comes up with reset state, and nothing else in a
 * bring-up test notices. Here that state is CSRs, private to each hart.
 *
 * The one check with no ARM counterpart is the interrupt stack. Each hart's
 * trap path moves onto the stack the hart started its first thread from,
 * through mscratch (port_core_rv32.cpp). Two harts sharing one, or two whose
 * stacks overlap, would corrupt each other's handler frames on the first pair
 * of simultaneous interrupts, which a bring-up test does not provoke, so it
 * is checked directly.
 *
 * Ending: as test_cortex_m33_smp_bringup, core 0 reports, after a bounded wait
 * on a flag core 1 sets once and never clears.
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

static_assert(config::cores == 2, "This bring-up is dual hart");
static_assert(CYROS_PORT_CORE_COUNT == 2, "Needs the riscv_virt_smp port");

extern "C" void cyros_port_trap_entry(void);

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 2048;

alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_core0[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_core1[stack_size];

/* What each hart's thread saw of its own hart, read from the thread itself,
 * because every one of these CSRs answers for the hart that reads it. */
struct hart_view
{
   std::uint32_t id;
   std::uint32_t mtvec;
   std::uint32_t mie;
   std::uint32_t mscratch;
   std::uint32_t mcountinhibit;
   std::uintptr_t tls;
   std::uint64_t cycles_moved;
};

hart_view views[2] = {};
std::atomic<bool> core1_finished{false};

std::uint32_t read_mtvec()         { std::uint32_t v; asm volatile("csrr %0, mtvec" : "=r"(v)); return v; }
std::uint32_t read_mie()           { std::uint32_t v; asm volatile("csrr %0, mie" : "=r"(v)); return v; }
std::uint32_t read_mscratch()      { std::uint32_t v; asm volatile("csrr %0, mscratch" : "=r"(v)); return v; }
std::uint32_t read_mcountinhibit() { std::uint32_t v; asm volatile("csrr %0, mcountinhibit" : "=r"(v)); return v; }

std::uint64_t read_mcycle()
{
   std::uint32_t high, low, again;
   do {
      asm volatile("csrr %0, mcycleh" : "=r"(high));
      asm volatile("csrr %0, mcycle"  : "=r"(low));
      asm volatile("csrr %0, mcycleh" : "=r"(again));
   } while (high != again);
   return (static_cast<std::uint64_t>(high) << 32) | low;
}

void look_at_this_hart(hart_view& view)
{
   view.id            = this_core::id();
   view.mtvec         = read_mtvec();
   view.mie           = read_mie();
   view.mscratch      = read_mscratch();
   view.mcountinhibit = read_mcountinhibit();
   view.tls           = reinterpret_cast<std::uintptr_t>(cyros_port_get_tls_pointer());
   std::uint64_t const before = read_mcycle();
   for (int i = 0; i < 1000; ++i) { asm volatile("" ::: "memory"); }
   view.cycles_moved = read_mcycle() - before;
}

/* Bounded in yields, as test_cortex_m33_smp_bringup's, so a hart that never
 * joins is reported rather than timed out. */
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
   look_at_this_hart(views[1]);
   core1_finished.store(true, std::memory_order_release);
}

void print_view(char const* label, hart_view const& view)
{
   cyros::bench::print(label);
   cyros::bench::print(" id ");
   cyros::bench::print_hex(view.id);
   cyros::bench::print(" mtvec ");
   cyros::bench::print_hex(view.mtvec);
   cyros::bench::print(" mie ");
   cyros::bench::print_hex(view.mie);
   cyros::bench::print(" mscratch ");
   cyros::bench::print_hex(view.mscratch);
   cyros::bench::print(" tp ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(view.tls));
   cyros::bench::print("\n");
}

void thread_on_core0()
{
   look_at_this_hart(views[0]);
   bool const rendezvous = wait_for_core1();

   cyros::bench::start("the second hart reached its thread");
   CYROS_CHECK(rendezvous);

   cyros::bench::start("the kernel reports two cores");
   CYROS_CHECK_EQ(kernel::core_count(), 2u);

   hart_view const& h0 = views[0];
   hart_view const& h1 = views[1];
   print_view("  hart 0:", h0);
   print_view("  hart 1:", h1);

   cyros::bench::start("each thread runs on the hart it was pinned to");
   CYROS_CHECK_EQ(h0.id, 0u);
   CYROS_CHECK_EQ(h1.id, 1u);

   auto const trap_entry = reinterpret_cast<std::uint32_t>(&cyros_port_trap_entry);
   cyros::bench::start("each hart traps into the port, in direct mode");
   CYROS_CHECK_EQ(h0.mtvec, trap_entry);
   CYROS_CHECK_EQ(h1.mtvec, trap_entry);

   cyros::bench::start("each hart has its reschedule interrupt enabled");
   CYROS_CHECK((h0.mie & 8u) != 0u);
   CYROS_CHECK((h1.mie & 8u) != 0u);

   /* mscratch holds the top of the hart's interrupt stack whenever no trap is
    * being handled, which is always, from a thread. */
   /* Apart, not merely different: each grows down from its top, so two tops
    * closer than a handler chain's depth overlap. 4 kB is several times what
    * the deepest chain here uses. */
   std::uint32_t const apart = h0.mscratch > h1.mscratch ? h0.mscratch - h1.mscratch
                                                         : h1.mscratch - h0.mscratch;
   cyros::bench::start("each hart has an interrupt stack of its own");
   cyros::bench::print("  the two interrupt stack tops are ");
   cyros::bench::print_hex(apart);
   cyros::bench::print(" bytes apart\n");
   CYROS_CHECK(h0.mscratch != 0u);
   CYROS_CHECK(h1.mscratch != 0u);
   CYROS_CHECK(apart >= 4096u);

   cyros::bench::start("each hart's cycle counter runs");
   CYROS_CHECK_EQ(h1.mcountinhibit & 1u, 0u);
   CYROS_CHECK(h1.cycles_moved > 0u);

   cyros::bench::start("each thread has its own TLS pointer");
   CYROS_CHECK(h0.tls != 0u);
   CYROS_CHECK(h1.tls != 0u);
   CYROS_CHECK(h0.tls != h1.tls);

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("rv32 two-hart kernel bring-up\n\n");

   /* Hart 1 is parked by the board until start_cores, so only hart 0 runs
    * here. */
   cyros::bench::start("the bootstrap hart identifies itself as core 0");
   CYROS_CHECK_EQ(this_core::id(), 0u);

   kernel::initialise();

   cyros::bench::start("one thread pinned to each hart registers");
   thread t0(thread_on_core0, stack_core0, thread::priority(0), core0);
   thread t1(thread_on_core1, stack_core1, thread::priority(0), core1);
   CYROS_CHECK_EQ(kernel::active_threads(), 2u);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
