/**
 * @file test_riscv_stack_guard.cpp
 * @brief The rv32 core layer's stack guard: PMP entry 0 over the bottom of the
 *        running thread's buffer.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the rv32 core layer's stack guard, placed by cyros_port_context_init,
 *          moved by every cyros_port_switch, and bound to machine mode by the
 *          target (Smepmp on QEMU's virt, Xh3pmpm on the RP2350's Hazard3).
 * Trusts:  layer 0 and the kernel bring-up layer 2 proves.
 * Proves:  that entry 0 is a 128-byte no-access region in the running
 *          thread's own buffer, 256 bytes above its bottom, that loads and
 *          stores into it fault and its neighbours do not, that it follows
 *          the switch to another thread, and that running a thread off the
 *          bottom of its stack ends in an access fault with nothing below the
 *          buffer touched, the trap entry's own spill staying in the buffer's
 *          spill zone.
 *
 *
 * HOW IT SEES A FAULT
 * ===================
 * The core layer sends every fault to cyros_riscv_fault_handler, which is weak
 * so that an application can take faults, as an ARM application owns its fault
 * vectors. This test defines it. A probe (one load or store at a chosen
 * address) is answered by stepping the saved mepc past the instruction and
 * returning, which resumes the thread. The overrun cannot be resumed, so its
 * fault ends the test from inside the handler, as on ARM. Any fault the test
 * did not ask for is a failure.
 *
 *
 * THE SPILL ZONE
 * ==============
 * The trap entry saves its frame on the interrupted stack, so a thread that
 * faults on the guard makes the entry fault again, 128 bytes lower each time,
 * until its stores clear the guard: at most 252 bytes below it. The guard
 * therefore sits 256 bytes above the buffer's bottom. The overrun checks both
 * halves of that promise: the canary below the buffer is untouched, and the
 * deepest byte the spill changed is inside the zone.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <cstddef>
#include <cstdint>

using namespace cyros;

/* The core layer's fault hook (riscv.hpp, private to the port). frame[0] is
 * the saved mepc. */
extern "C" void cyros_riscv_fault_handler(std::uint32_t mcause, std::uint32_t* frame);

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 2048;

/* The geometry the core layer promises. */
constexpr std::uintptr_t guard_bytes = 128u;
constexpr std::uintptr_t spill_bytes = 256u;

constexpr std::uint32_t cause_load_access  = 5u;
constexpr std::uint32_t cause_store_access = 7u;

alignas(CYROS_PORT_STACK_ALIGN) std::byte probe_stack[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte other_stack[stack_size];

/* The overrun thread's buffer with a canary directly below it, one object so
 * nothing else can sit between them. */
constexpr std::uint8_t canary_byte = 0xa5u;
constexpr std::uint8_t spill_byte  = 0x5au;
struct alignas(CYROS_PORT_STACK_ALIGN) canaried
{
   std::uint8_t canary[512];
   std::byte    stack[stack_size];
};
canaried overrun_area;

/* --------------------------------------------------------------------------
 * The fault handler
 * ----------------------------------------------------------------------- */

enum class expecting : std::uint32_t { nothing, probe, overrun };

volatile expecting expect = expecting::nothing;
volatile std::uint32_t probe_faults = 0u;
volatile std::uint32_t probe_cause = 0u;

void overrun_ended(std::uint32_t mcause, std::uint32_t const* frame);

}  // namespace

extern "C" void cyros_riscv_fault_handler(std::uint32_t mcause, std::uint32_t* frame)
{
   switch (expect) {
   case expecting::probe:
      /* A probe is one uncompressed load or store: resume after it. */
      probe_faults = probe_faults + 1u;
      probe_cause = mcause;
      frame[0] += 4u;
      return;
   case expecting::overrun:
      overrun_ended(mcause, frame);
      break;
   case expecting::nothing:
      break;
   }
   cyros::bench::print("\n*** a fault the test did not ask for, mcause ");
   cyros::bench::print_hex(mcause);
   cyros::bench::print(", mepc ");
   cyros::bench::print_hex(frame[0]);
   cyros::bench::print(" ***\n");
   cyros::bench::record(false, "unexpected fault", __FILE__, __LINE__);
   cyros::bench::finish();
}

namespace
{

/* --------------------------------------------------------------------------
 * Reading the guard back, and probing
 * ----------------------------------------------------------------------- */

std::uint32_t read_pmpcfg0()
{
   std::uint32_t value;
   asm volatile("csrr %0, pmpcfg0" : "=r"(value));
   return value;
}

std::uint32_t read_pmpaddr0()
{
   std::uint32_t value;
   asm volatile("csrr %0, pmpaddr0" : "=r"(value));
   return value;
}

std::uintptr_t read_sp()
{
   std::uintptr_t value;
   asm volatile("mv %0, sp" : "=r"(value));
   return value;
}

/* pmpaddr0 for a NAPOT guard at `bottom`: the base over four, with the
 * size's low ones. Compared with the two lowest bits forced, because Hazard3
 * reads them back as zero (what it stores, and enforces, is the full value,
 * which the probes below show). So the size is proved by probing the region's
 * edges rather than decoded. */
std::uint32_t napot(std::uintptr_t bottom)
{
   return static_cast<std::uint32_t>(bottom >> 2) | static_cast<std::uint32_t>(guard_bytes / 8u - 1u);
}

bool guard_is_at(std::uintptr_t bottom)
{
   return (read_pmpaddr0() | 3u) == napot(bottom);
}

/* One load or store, uncompressed so the handler can step over it. Returns
 * whether it faulted, and with which cause through probe_cause. */
bool load_faults(std::uintptr_t address)
{
   probe_faults = 0u;
   expect = expecting::probe;
   asm volatile(
      ".option push \n"
      ".option norvc \n"
      "lw t0, 0(%0) \n"
      ".option pop \n"
      :
      : "r"(address)
      : "t0", "memory");
   expect = expecting::nothing;
   return probe_faults != 0u;
}

bool store_faults(std::uintptr_t address)
{
   probe_faults = 0u;
   expect = expecting::probe;
   asm volatile(
      ".option push \n"
      ".option norvc \n"
      "sw zero, 0(%0) \n"
      ".option pop \n"
      :
      : "r"(address)
      : "memory");
   expect = expecting::nothing;
   return probe_faults != 0u;
}

/* The guard this thread should have, from its own buffer. */
std::uintptr_t expected_guard(std::byte const* buffer)
{
   auto const base = reinterpret_cast<std::uintptr_t>(buffer);
   return (base + spill_bytes + guard_bytes - 1u) & ~(guard_bytes - 1u);
}

/* --------------------------------------------------------------------------
 * The threads. All three share one priority and hand over by yielding, in
 * the order probe, other, overrun.
 * ----------------------------------------------------------------------- */

volatile bool probe_done = false;
volatile bool other_done = false;
volatile std::uintptr_t probe_guard = 0u;

void probe_thread()
{
   cyros::bench::start("entry 0 is a no-access NAPOT region");
   std::uint32_t const cfg = read_pmpcfg0() & 0xffu;
   cyros::bench::print("  pmpcfg0 entry 0 = ");
   cyros::bench::print_hex(cfg);
   cyros::bench::print("\n");
   CYROS_CHECK_EQ((cfg >> 3) & 3u, 3u);   /* NAPOT              */
   CYROS_CHECK_EQ(cfg & 7u, 0u);          /* no R, W or X        */

   cyros::bench::start("entry 0 sits 256 bytes above this thread's buffer bottom");
   std::uintptr_t const g = expected_guard(probe_stack);
   std::uintptr_t const sp = read_sp();
   cyros::bench::print("  buffer   = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(probe_stack)));
   cyros::bench::print("\n  pmpaddr0 = ");
   cyros::bench::print_hex(read_pmpaddr0());
   cyros::bench::print("\n  guard    = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(g));
   cyros::bench::print("\n  sp       = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(sp));
   cyros::bench::print("\n");
   CYROS_CHECK(guard_is_at(g));
   CYROS_CHECK(sp > g + guard_bytes);
   probe_guard = g;

   cyros::bench::start("loads and stores across all 128 bytes fault, as access faults");
   CYROS_CHECK(load_faults(g));
   CYROS_CHECK_EQ(probe_cause, cause_load_access);
   CYROS_CHECK(store_faults(g));
   CYROS_CHECK_EQ(probe_cause, cause_store_access);
   CYROS_CHECK(load_faults(g + guard_bytes - 4u));
   CYROS_CHECK_EQ(probe_cause, cause_load_access);
   CYROS_CHECK(store_faults(g + guard_bytes - 4u));
   CYROS_CHECK_EQ(probe_cause, cause_store_access);

   cyros::bench::start("the words either side of it do not");
   CYROS_CHECK(!load_faults(g + guard_bytes));
   CYROS_CHECK(!store_faults(g + guard_bytes));
   CYROS_CHECK(!load_faults(g - 4u));
   CYROS_CHECK(!store_faults(g - 4u));

   probe_done = true;
   while (!other_done) { this_thread::yield(); }
}

void other_thread()
{
   while (!probe_done) { this_thread::yield(); }

   cyros::bench::start("the guard moved to this thread's buffer at the switch");
   std::uintptr_t const g = expected_guard(other_stack);
   CYROS_CHECK(guard_is_at(g));

   cyros::bench::start("so this thread faults on its own guard, not the other's");
   CYROS_CHECK(store_faults(g));
   CYROS_CHECK(!store_faults(probe_guard));
   CYROS_CHECK(!load_faults(probe_guard));

   other_done = true;
   while (true) { this_thread::yield(); }
}

/* Deep enough to run off any buffer: each level writes the whole of its own
 * frame, and the call is not a tail call. The limit is far past any buffer
 * here, and volatile so the compiler cannot see the recursion is meant to
 * run away. */
volatile std::uint32_t dive_sink = 0u;
volatile std::uint32_t dive_limit = 1000000u;

[[gnu::noinline]] void dive(std::uint32_t depth)
{
   volatile std::uint32_t pad[24];
   for (std::uint32_t i = 0; i < 24u; ++i) { pad[i] = depth; }
   if (depth < dive_limit) { dive(depth + 1u); }
   dive_sink = pad[0] + pad[23];
}

void overrun_thread()
{
   while (!other_done) { this_thread::yield(); }

   /* Mark everything below the guard: the canary under the buffer, and the
    * spill zone inside it. */
   for (auto& b : overrun_area.canary) { b = canary_byte; }
   auto* const bottom = reinterpret_cast<std::uint8_t*>(overrun_area.stack);
   std::uintptr_t const guard = expected_guard(overrun_area.stack);
   for (auto* p = bottom; reinterpret_cast<std::uintptr_t>(p) < guard; ++p) { *p = spill_byte; }

   cyros::bench::start("running off the bottom of the stack faults");
   expect = expecting::overrun;
   dive(0u);

   cyros::bench::record(false, "stack overrun did not fault", __FILE__, __LINE__);
   cyros::bench::finish();
}

void overrun_ended(std::uint32_t mcause, std::uint32_t const* frame)
{
   expect = expecting::nothing;
   cyros::bench::print("  mcause = ");
   cyros::bench::print_hex(mcause);
   cyros::bench::print(", mepc = ");
   cyros::bench::print_hex(frame[0]);
   cyros::bench::print("\n");
   CYROS_CHECK(mcause == cause_store_access || mcause == cause_load_access);

   cyros::bench::start("nothing below the buffer was written");
   std::uint32_t touched = 0u;
   for (auto b : overrun_area.canary) { touched += b != canary_byte ? 1u : 0u; }
   CYROS_CHECK_EQ(touched, 0u);

   cyros::bench::start("the trap entry's spill stayed in the spill zone");
   auto const* const bottom = reinterpret_cast<std::uint8_t const*>(overrun_area.stack);
   std::uintptr_t const guard = expected_guard(overrun_area.stack);
   std::uintptr_t deepest = guard;
   for (auto const* p = bottom; reinterpret_cast<std::uintptr_t>(p) < guard; ++p) {
      if (*p != spill_byte) {
         deepest = reinterpret_cast<std::uintptr_t>(p);
         break;
      }
   }
   cyros::bench::print("  spill reached ");
   cyros::bench::print_dec(guard - deepest);
   cyros::bench::print(" bytes below the guard\n");
   CYROS_CHECK(guard - deepest <= spill_bytes);

   cyros::bench::finish();
}

}  // namespace

extern "C" int cyros_bench_main()
{
   cyros::bench::print("rv32 stack guard\n\n");

   kernel::initialise();
   thread probe(probe_thread, probe_stack, thread::priority(1), core0);
   thread other(other_thread, other_stack, thread::priority(1), core0);
   thread overrun(overrun_thread, overrun_area.stack, thread::priority(1), core0);

   /* Does not return on a target port. The overrun's fault ends the image. */
   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
