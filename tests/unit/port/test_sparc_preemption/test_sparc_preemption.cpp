/**
 * @file test_sparc_preemption.cpp
 * @brief Every integer register, Y, the condition codes and the register
 *        windows survive an interrupt that switches threads.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the sparcv8 layer's trap exit when the interrupt switched: the
 *          flush of the outgoing thread's windows still in the register file,
 *          and the fill of the incoming thread's interrupted window.
 * Trusts:  layers 0 to 5, the semaphore (test_sync_semaphore), and the plain
 *          context switch test_sparc_bringup proves.
 * Proves:  that a thread interrupted with a pattern in every global, out,
 *          local and in it can hold, in Y and in the condition codes, at the
 *          top of a window chain, comes back with all of it after the
 *          interrupt's reschedule ran a more urgent thread that overwrote
 *          every register and reused every window, and that every caller's
 *          window comes back with it.
 *
 * test_sparc_port proves the same registers across a trap that does NOT
 * switch, and test_sparc_bringup a window across a YIELD, which flushes at
 * entry. Only an interrupt that switches reaches the exit's flush walk, where
 * the outgoing thread's windows are stored only then, so this is its test.
 *
 * The window: with interrupts masked the test releases the semaphore a more
 * urgent thread waits on, which pends the reschedule, then loads its pattern
 * and clears PIL's interrupt field inside one asm block. The reschedule is
 * taken there, the urgent thread runs, loads its own values, churns every
 * window, blocks again, and the window reads back. Half the rounds call it
 * from twelve calls down, where every window is in use.
 *
 * Then a trap inside a handler, which only a running kernel can show, since
 * only then is there an interrupt stack: a device level forced on this core,
 * whose hook takes a software trap. The nested trap must stay below the
 * handler it interrupted, on the same stack, never restart at the top of the
 * interrupt stack over that handler's frames.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <cstddef>
#include <cstdint>

using namespace cyros;

static_assert(config::cores == 1, "This test is single core");

/* ---------------------------------------------------------------------------
 * The window, in assembly: test_sparc_port's register check with PIL's
 * interrupt field cleared in place of a pended trap. Masked on entry and on
 * return.
 *
 * out[n]: 1-7 g1-g7, 8-15 o0-o7, 16-23 l0-l7, 24-31 i0-i7, 32 Y, 33 PSR.
 * ------------------------------------------------------------------------ */

extern "C" void sparc_registers_across_a_switch(std::uint32_t* out);

asm(
   ".pushsection .text.sparc_registers_across_a_switch, \"ax\"  \n"
   ".global sparc_registers_across_a_switch                     \n"
   ".type   sparc_registers_across_a_switch, #function          \n"
   "sparc_registers_across_a_switch:                            \n"
   "   save  %sp, -256, %sp                                     \n"
   "   st    %i0, [%sp + 96]                                    \n"
   "   st    %g5, [%sp + 108]                                   \n"
   "   st    %g6, [%sp + 112]                                   \n"
   "   st    %g7, [%sp + 116]                                   \n"
   "   rd    %psr, %l7                                          \n"
   "   andn  %l7, 0xe00, %l7                                    \n"   /* the interrupt field */
   "   set   0x00f00000, %g1                                    \n"
   "   or    %l7, %g1, %l7                                      \n"   /* NZVC */
   "   set   0xA500003F, %g1                                    \n"
   "   wr    %g1, %y                                            \n"
   ".irp n, 1,2,3,4,5,6,7                                       \n"
   "   set   0xA5000000 + \\n, %g\\n                            \n"
   ".endr                                                       \n"
   ".irp n, 0,1,2,3,4,5,7                                       \n"
   "   set   0xA5000008 + \\n, %o\\n                            \n"
   ".endr                                                       \n"
   ".irp n, 0,1,2,3,4,5,6                                       \n"
   "   set   0xA5000010 + \\n, %l\\n                            \n"
   ".endr                                                       \n"
   ".irp n, 0,1,2,3,4,5                                         \n"
   "   set   0xA5000018 + \\n, %i\\n                            \n"
   ".endr                                                       \n"
   "   wr    %l7, %psr                                          \n"
   "   nop                                                      \n"   /* the reschedule is taken here */
   "   nop                                                      \n"
   "   nop                                                      \n"
   ".irp n, 1,2,3,4,5,6,7                                       \n"
   "   st    %g\\n, [%sp + 120 + 4 * \\n]                       \n"
   ".endr                                                       \n"
   ".irp n, 0,1,2,3,4,5,6,7                                     \n"
   "   st    %o\\n, [%sp + 152 + 4 * \\n]                       \n"
   "   st    %l\\n, [%sp + 184 + 4 * \\n]                       \n"
   "   st    %i\\n, [%sp + 216 + 4 * \\n]                       \n"
   ".endr                                                       \n"
   "   rd    %y, %l0                                            \n"
   "   st    %l0, [%sp + 248]                                   \n"
   "   rd    %psr, %l0                                          \n"
   "   st    %l0, [%sp + 252]                                   \n"
   "   or    %l0, 0xe00, %l0                                    \n"
   "   wr    %l0, %psr                                          \n"
   "   nop                                                      \n"
   "   nop                                                      \n"
   "   nop                                                      \n"
   "   ld    [%sp + 108], %g5                                   \n"
   "   ld    [%sp + 112], %g6                                   \n"
   "   ld    [%sp + 116], %g7                                   \n"
   "   ld    [%sp + 96], %l1                                    \n"
   "   add   %sp, 120, %l2                                      \n"
   "   mov   0, %l3                                             \n"
   "1: ld    [%l2 + %l3], %l4                                   \n"
   "   st    %l4, [%l1 + %l3]                                   \n"
   "   add   %l3, 4, %l3                                        \n"
   "   cmp   %l3, 136                                           \n"
   "   bne   1b                                                 \n"
   "    nop                                                     \n"
   "   ret                                                      \n"
   "   restore                                                  \n"
   ".size sparc_registers_across_a_switch, . - sparc_registers_across_a_switch \n"

   /* The urgent thread's own values in every register a window and the
    * globals give it, its own g5 to g7 put back (g7 is its TLS). */
   ".global sparc_overwrite_registers                           \n"
   ".type   sparc_overwrite_registers, #function                \n"
   "sparc_overwrite_registers:                                  \n"
   "   save  %sp, -96, %sp                                      \n"
   "   mov   %g5, %l5                                           \n"
   "   mov   %g6, %l6                                           \n"
   "   mov   %g7, %l7                                           \n"
   "   set   0x5C00003F, %g1                                    \n"
   "   wr    %g1, %y                                            \n"
   ".irp n, 1,2,3,4,5,6,7                                       \n"
   "   set   0x5C000000 + \\n, %g\\n                            \n"
   ".endr                                                       \n"
   ".irp n, 0,1,2,3,4,5                                         \n"
   "   set   0x5C000008 + \\n, %o\\n                            \n"
   "   set   0x5C000018 + \\n, %i\\n                            \n"
   ".endr                                                       \n"
   ".irp n, 0,1,2,3,4                                           \n"
   "   set   0x5C000010 + \\n, %l\\n                            \n"
   ".endr                                                       \n"
   "   subcc %g0, 1, %g0                                        \n"   /* N and C, Z and V clear */
   "   mov   %l5, %g5                                           \n"
   "   mov   %l6, %g6                                           \n"
   "   mov   %l7, %g7                                           \n"
   "   ret                                                      \n"
   "   restore                                                  \n"
   ".size sparc_overwrite_registers, . - sparc_overwrite_registers \n"
   ".popsection                                                 \n");

extern "C" void sparc_overwrite_registers();

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 4096;

alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_a[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_urgent[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_reporter[stack_size];

constexpr int rounds = 32;
constexpr int deep = 12;

sync::semaphore urgent_wake{0};

/* QEMU's leon3_generic: core 0's IRQMP mask and force registers, and a level
 * nothing there uses. */
constexpr std::uintptr_t irqmp_mask0  = 0x80000200u + 0x40u;
constexpr std::uintptr_t irqmp_force0 = 0x80000200u + 0x80u;
constexpr std::uint32_t  device_level = 5u;

volatile std::uint32_t& reg(std::uintptr_t address) { return *reinterpret_cast<volatile std::uint32_t*>(address); }

volatile std::uintptr_t device_sp = 0;
volatile std::uintptr_t nested_sp = 0;
volatile std::uint32_t  device_sentinel_wrong = 0xFFFFFFFFu;
volatile bool fault_expected = false;

volatile int urgent_ran = 0;
volatile int switched_rounds = 0;
volatile std::uint32_t registers_wrong = 0;
volatile std::uint32_t sums_wrong = 0;
volatile bool a_done = false;

std::uint32_t count_wrong(std::uint32_t const (&out)[34])
{
   std::uint32_t wrong = 0;
   auto expect = [&](std::uint32_t index, std::uint32_t value) {
      if (out[index] != value) {
         ++wrong;
         cyros::bench::print("  wrong: out[");
         cyros::bench::print_hex(index);
         cyros::bench::print("] = ");
         cyros::bench::print_hex(out[index]);
         cyros::bench::print("\n");
      }
   };
   for (std::uint32_t n = 1; n <= 7; ++n) { expect(n, 0xA5000000u + n); }
   for (std::uint32_t n = 0; n <= 7; ++n) { if (n != 6) { expect(8 + n, 0xA5000008u + n); } }
   for (std::uint32_t n = 0; n <= 6; ++n) { expect(16 + n, 0xA5000010u + n); }
   for (std::uint32_t n = 0; n <= 5; ++n) { expect(24 + n, 0xA5000018u + n); }
   expect(32, 0xA500003Fu);
   if ((out[33] & 0x00F00000u) != 0x00F00000u) {
      ++wrong;
      cyros::bench::print("  wrong: the condition codes\n");
   }
   return wrong;
}

/* The window `depth` calls down, each with a live local. The sum comes out
 * right only if every caller's window came back. */
[[gnu::noinline]] std::uint32_t at_depth(int depth, std::uint32_t* out)
{
   volatile std::uint32_t local = 0x4000u + static_cast<std::uint32_t>(depth);
   if (depth == 0) {
      sparc_registers_across_a_switch(out);
      return local;
   }
   return at_depth(depth - 1, out) + local;
}

std::uint32_t expected_sum(int depth)
{
   std::uint32_t sum = 0;
   for (int d = 0; d <= depth; ++d) { sum += 0x4000u + static_cast<std::uint32_t>(d); }
   return sum;
}

[[gnu::noinline]] std::uint32_t churn(int depth)
{
   volatile std::uint32_t local = static_cast<std::uint32_t>(depth) * 0x01010101u;
   return depth == 0 ? local : churn(depth - 1) ^ local;
}

void urgent()
{
   while (true) {
      urgent_wake.acquire();
      sparc_overwrite_registers();
      (void)churn(20);
      urgent_ran = urgent_ran + 1;
   }
}

} // namespace

/* The device hook: a handler with a live local across a trap of its own. */
extern "C" void cyros_sparc_device_interrupt(std::uint32_t level)
{
   if (level != device_level) {
      cyros::bench::print("\n*** unexpected device level\n");
      cyros::bench::host_exit(3u);
   }
   volatile std::uint32_t sentinel = 0x5E17E1u;
   device_sp = reinterpret_cast<std::uintptr_t>(cyros_port_get_stack_pointer());
   fault_expected = true;
   asm volatile("ta 0x20" ::: "memory");
   fault_expected = false;
   device_sentinel_wrong = sentinel == 0x5E17E1u ? 0u : 1u;
}

/* The fault hook: records where it ran and skips the ta. Any other fault
 * fails the run. */
extern "C" void cyros_sparc_fault_handler(std::uint32_t tt, std::uint32_t* frame)
{
   if (!fault_expected || tt != 0xA0u) {
      cyros::bench::print("\n*** unexpected trap ");
      cyros::bench::print_hex(tt);
      cyros::bench::print("\n");
      cyros::bench::host_exit(3u);
   }
   nested_sp = reinterpret_cast<std::uintptr_t>(cyros_port_get_stack_pointer());
   frame[1] = frame[2];
   frame[2] = frame[2] + 4u;
}

namespace
{

void trap_inside_a_handler()
{
   reg(irqmp_mask0) = reg(irqmp_mask0) | (1u << device_level);
   reg(irqmp_force0) = 1u << device_level;
   for (int i = 0; i < 64 && device_sp == 0u; ++i) { asm volatile("nop"); }
   reg(irqmp_mask0) = reg(irqmp_mask0) & ~(1u << device_level);
}

void thread_a()
{
   for (int i = 0; i < rounds; ++i) {
      int const depth = (i % 2 == 0) ? 0 : deep;
      int const before = urgent_ran;
      std::uint32_t out[34] = {};

      cyros_mask_token_t const token = cyros_port_irq_save();
      urgent_wake.release();
      std::uint32_t const sum = at_depth(depth, out);
      cyros_port_irq_restore(token);

      if (urgent_ran == before + 1) {
         switched_rounds = switched_rounds + 1;
      }
      registers_wrong = registers_wrong + count_wrong(out);
      if (sum != expected_sum(depth)) {
         sums_wrong = sums_wrong + 1;
      }
   }
   trap_inside_a_handler();
   a_done = true;
}

void reporter()
{
   cyros::bench::start("each interrupt ran the urgent thread inside the window");
   CYROS_CHECK(a_done);
   CYROS_CHECK_EQ(switched_rounds, rounds);
   CYROS_CHECK_EQ(urgent_ran, rounds);

   cyros::bench::start("every register, Y and the codes survive an interrupt that switched");
   cyros::bench::print("  wrong registers = ");
   cyros::bench::print_hex(registers_wrong);
   cyros::bench::print("\n");
   CYROS_CHECK_EQ(registers_wrong, 0u);

   cyros::bench::start("... and every caller's window, half of them from twelve calls down");
   CYROS_CHECK_EQ(sums_wrong, 0u);

   cyros::bench::start("a trap inside a handler stays below it, on the same stack");
   cyros::bench::print("  device handler sp ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(device_sp));
   cyros::bench::print(", nested trap's handler sp ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(nested_sp));
   cyros::bench::print("\n");
   CYROS_CHECK(device_sp != 0u);
   CYROS_CHECK(nested_sp != 0u);
   CYROS_CHECK(nested_sp < device_sp);
   CYROS_CHECK(device_sp - nested_sp < 2048u);
   CYROS_CHECK_EQ(device_sentinel_wrong, 0u);

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("sparcv8 preemption\n\n");

   kernel::initialise();

   /* The urgent thread outranks A, and the reporter runs only once A has
    * finished and the urgent thread is blocked, so it reports last. */
   thread u(urgent, stack_urgent, thread::priority(1), core0);
   thread a(thread_a, stack_a, thread::priority(2), core0);
   thread r(reporter, stack_reporter, thread::priority(3), core0);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
