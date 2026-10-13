/**
 * @file test_armv8r_stack_guard.cpp
 * @brief The armv8r core layer's stack guard: EL1 MPU region 15, read-only,
 *        over the bottom of the running thread's buffer.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the armv8r core layer's stack guard, placed by
 *          cyros_port_context_init and moved by every cyros_port_switch.
 * Trusts:  layer 0 and the kernel bring-up layer 2 proves.
 * Proves:  that region 15 is a 128-byte read-only, execute-never region at
 *          the bottom of the running thread's own buffer, with the MPU and its
 *          background region on, that stores into it fault and its neighbours
 *          do not, that it follows the switch to another thread, and that
 *          running a thread off the bottom of its stack ends in a Data Abort
 *          on a write into the guard with nothing below the buffer touched.
 *
 *
 * READ-ONLY, NOT NO-ACCESS
 * ========================
 * PMSAv8-32 has no access permission that denies EL1, where cyros runs, so the
 * guard is read-only to it. An overrun is a write, so that is enough to stop
 * one before it reaches whatever lies below the buffer, and a load from the
 * guard is allowed, which the probes check too.
 *
 *
 * HOW IT SEES A FAULT
 * ===================
 * The board's vector table sends a Data Abort to DataAbort_Handler, which is
 * weak so that a test can take it, as test_cortex_m_stack_guard takes
 * MemManage on M-profile. This test defines it. A probe is one 32-bit store,
 * answered by returning past it. The overrun cannot be resumed, so its fault
 * ends the test from inside the handler. Any fault the test did not ask for
 * is a failure.
 *
 * No spill zone, unlike RISC-V: a store into the guard made while an exception
 * entry saves its frame is a Data Abort like any other, taken in Abort mode on
 * that mode's own stack, so it never stacks another frame below the guard.
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

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 2048;

/* The geometry the core layer promises: 128 bytes from the buffer's bottom
 * rounded up to the MPU's 64-byte granule. */
constexpr std::uintptr_t guard_bytes = 128u;

alignas(CYROS_PORT_STACK_ALIGN) std::byte probe_stack[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte other_stack[stack_size];

/* The overrun thread's buffer with a canary directly below it, one object so
 * nothing else can sit between them. */
constexpr std::uint8_t canary_byte = 0xa5u;
struct alignas(64) canaried
{
   std::uint8_t canary[512];
   std::byte    stack[stack_size];
};
canaried overrun_area;

/* --------------------------------------------------------------------------
 * The Data Abort
 * ----------------------------------------------------------------------- */

enum class expecting : std::uint32_t { nothing, probe, overrun };

volatile expecting expect = expecting::nothing;
volatile std::uint32_t probe_faults = 0u;
volatile std::uint32_t probe_dfsr = 0u;
volatile std::uint32_t probe_dfar = 0u;

std::uint32_t read_dfsr() { std::uint32_t v; asm volatile("mrc p15, 0, %0, c5, c0, 0" : "=r"(v)); return v; }
std::uint32_t read_dfar() { std::uint32_t v; asm volatile("mrc p15, 0, %0, c6, c0, 0" : "=r"(v)); return v; }

void overrun_ended(std::uint32_t dfsr, std::uint32_t dfar, std::uint32_t lr);

}  // namespace

/* Returns to resume after the faulting store, which is only ever a probe. */
extern "C" void guard_test_data_abort(std::uint32_t lr)
{
   std::uint32_t const dfsr = read_dfsr();
   std::uint32_t const dfar = read_dfar();
   switch (expect) {
   case expecting::probe:
      probe_faults = probe_faults + 1u;
      probe_dfsr = dfsr;
      probe_dfar = dfar;
      return;
   case expecting::overrun:
      overrun_ended(dfsr, dfar, lr);
      break;
   case expecting::nothing:
      break;
   }
   cyros::bench::print("\n*** a Data Abort the test did not ask for, DFSR ");
   cyros::bench::print_hex(dfsr);
   cyros::bench::print(", DFAR ");
   cyros::bench::print_hex(dfar);
   cyros::bench::print(", lr ");
   cyros::bench::print_hex(lr);
   cyros::bench::print(" ***\n");
   cyros::bench::record(false, "unexpected fault", __FILE__, __LINE__);
   cyros::bench::finish();
}

/* lr is the faulting store plus 8, and a probe's store is 32 bits, so the
 * next instruction is at lr - 4. */
extern "C" [[gnu::naked]] void DataAbort_Handler(void)
{
   asm volatile(
      "push   {r0-r3, r12, lr}        \n"
      "mov    r0, lr                  \n"
      "bl     guard_test_data_abort   \n"
      "pop    {r0-r3, r12, lr}        \n"
      "subs   pc, lr, #4              \n");
}

namespace
{

/* --------------------------------------------------------------------------
 * Reading the guard back, and probing
 * ----------------------------------------------------------------------- */

std::uint32_t read_prbar15() { std::uint32_t v; asm volatile("mrc p15, 0, %0, c6, c15, 4" : "=r"(v)); return v; }
std::uint32_t read_prlar15() { std::uint32_t v; asm volatile("mrc p15, 0, %0, c6, c15, 5" : "=r"(v)); return v; }
std::uint32_t read_sctlr()   { std::uint32_t v; asm volatile("mrc p15, 0, %0, c1, c0, 0" : "=r"(v)); return v; }

std::uintptr_t read_sp()
{
   std::uintptr_t value;
   asm volatile("mov %0, sp" : "=r"(value));
   return value;
}

/* PRBAR: BASE[31:6], SH[4:3], AP[2:1], XN[0]. PRLAR: LIMIT[31:6], the last
 * 64-byte block included, AttrIndx[3:1], EN[0]. */
bool guard_is_at(std::uintptr_t bottom)
{
   return (read_prbar15() & ~63u) == bottom
       && (read_prlar15() & ~63u) == bottom + guard_bytes - 64u
       && (read_prlar15() & 1u) == 1u;
}

/* One 32-bit store, so the handler can step over it. Returns whether it
 * faulted. */
bool store_faults(std::uintptr_t address)
{
   probe_faults = 0u;
   expect = expecting::probe;
   asm volatile("str.w %1, [%0]" : : "r"(address), "r"(0x5a5a5a5au) : "memory");
   expect = expecting::nothing;
   return probe_faults != 0u;
}

bool load_faults(std::uintptr_t address)
{
   probe_faults = 0u;
   expect = expecting::probe;
   std::uint32_t value;
   asm volatile("ldr.w %0, [%1]" : "=r"(value) : "r"(address) : "memory");
   (void)value;
   expect = expecting::nothing;
   return probe_faults != 0u;
}

/* The guard this thread should have, from its own buffer. */
std::uintptr_t expected_guard(std::byte const* buffer)
{
   auto const base = reinterpret_cast<std::uintptr_t>(buffer);
   return (base + 63u) & ~std::uintptr_t{63u};
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
   cyros::bench::start("the MPU is on, with the background region for everything else");
   std::uint32_t const sctlr = read_sctlr();
   CYROS_CHECK_EQ(sctlr & 1u, 1u);                 /* M  */
   CYROS_CHECK_EQ((sctlr >> 17) & 1u, 1u);         /* BR */

   cyros::bench::start("region 15 is read-only and execute-never");
   std::uint32_t const prbar = read_prbar15();
   cyros::bench::print("  PRBAR15 = ");
   cyros::bench::print_hex(prbar);
   cyros::bench::print("  PRLAR15 = ");
   cyros::bench::print_hex(read_prlar15());
   cyros::bench::print("\n");
   CYROS_CHECK_EQ((prbar >> 1) & 3u, 2u);          /* AP: read-only at EL1 */
   CYROS_CHECK_EQ(prbar & 1u, 1u);                 /* XN */

   cyros::bench::start("it covers the bottom 128 bytes of this thread's buffer");
   std::uintptr_t const g = expected_guard(probe_stack);
   CYROS_CHECK(guard_is_at(g));
   CYROS_CHECK(read_sp() > g + guard_bytes);
   probe_guard = g;

   cyros::bench::start("stores across all 128 bytes fault, as writes");
   CYROS_CHECK(store_faults(g));
   CYROS_CHECK_EQ(probe_dfar, g);
   CYROS_CHECK_EQ((probe_dfsr >> 11) & 1u, 1u);    /* WnR: a write */
   CYROS_CHECK(store_faults(g + guard_bytes - 4u));
   CYROS_CHECK_EQ(probe_dfar, g + guard_bytes - 4u);

   cyros::bench::start("loads from it do not, nor stores either side of it");
   CYROS_CHECK(!load_faults(g));
   CYROS_CHECK(!store_faults(g + guard_bytes));
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

   for (auto& b : overrun_area.canary) { b = canary_byte; }

   cyros::bench::start("running off the bottom of the stack faults");
   expect = expecting::overrun;
   dive(0u);

   cyros::bench::record(false, "stack overrun did not fault", __FILE__, __LINE__);
   cyros::bench::finish();
}

void overrun_ended(std::uint32_t dfsr, std::uint32_t dfar, std::uint32_t lr)
{
   expect = expecting::nothing;
   std::uintptr_t const g = expected_guard(overrun_area.stack);
   cyros::bench::print("  DFSR = ");
   cyros::bench::print_hex(dfsr);
   cyros::bench::print(", DFAR = ");
   cyros::bench::print_hex(dfar);
   cyros::bench::print(", lr = ");
   cyros::bench::print_hex(lr);
   cyros::bench::print(", guard = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(g));
   cyros::bench::print("\n");
   CYROS_CHECK((dfsr >> 11) & 1u);                 /* a write */
   CYROS_CHECK(dfar >= g && dfar < g + guard_bytes);

   cyros::bench::start("nothing below the buffer was written");
   std::uint32_t touched = 0u;
   for (auto b : overrun_area.canary) { touched += b != canary_byte ? 1u : 0u; }
   CYROS_CHECK_EQ(touched, 0u);

   cyros::bench::finish();
}

}  // namespace

extern "C" int cyros_bench_main()
{
   cyros::bench::print("armv8r stack guard\n\n");

   kernel::initialise();
   thread probe(probe_thread, probe_stack, thread::priority(1), core0);
   thread other(other_thread, other_stack, thread::priority(1), core0);
   thread overrun(overrun_thread, overrun_area.stack, thread::priority(1), core0);

   /* Does not return on a target port. The overrun's fault ends the image. */
   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
