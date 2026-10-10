/**
 * @file test_cortex_m_bringup.cpp
 * @brief First kernel bring-up on the target. Runs under QEMU.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the Cortex-M ports' context switch, Mainline (cortex_m,
 *          rp2350_m33) and Baseline (cortex_m0), as exercised through the
 *          kernel's own scheduler.
 * Trusts:  layer 0, the port's masking contract (test_cortex_m_port, or
 *          test_cortex_m0_port on ARMv6-M).
 * Proves:  that cyros_port_context_init builds a frame the hardware can enter,
 *          that cyros_port_start_first reaches it, and that PendSV carries a
 *          thread out and back with its registers and stack intact.
 *
 *
 * WHAT MAKES THIS DIFFERENT FROM THE HOST EQUIVALENT
 * ==================================================
 * On Linux the context switch is a library call (fcontext) or a signal return.
 * Here it is the real thing: the hardware stacks half the frame on exception
 * entry, the port's PendSV prologue stacks the rest, and an exception return
 * unstacks it onto a DIFFERENT thread's stack than the one it came from. There
 * is no equivalent of that on the host ports, so nothing in the existing suite
 * covers it.
 *
 * The register check below is the part that earns its place. A switcher that
 * saves and restores the wrong register set still runs, still switches, and
 * still looks correct, right up to the point where a caller's live value is
 * quietly replaced. Writing known values into r4-r11, yielding, and reading
 * them back is the cheapest way to make that failure loud.
 *
 *
 * A NOTE ON HOW IT EXITS
 * ======================
 * kernel::start() does not return on this port, because cyros_port_start_first
 * enters the first thread and never comes back. So the checks cannot live after
 * it as they do in the host tests. The last thread to run reports and exits the
 * image through semihosting instead, which is why bench::finish() is called
 * from inside a thread rather than from the entry point.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

using namespace cyros;

static_assert(config::cores == 1, "This bring-up is single core");

namespace
{

/* .bss rather than the entry point's frame, which is how an MCU application
 * would place them, and keeps the main stack for exceptions and the scheduler. */
constexpr std::size_t stack_size = thread::min_stack_size + 2048;

alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_a[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_b[stack_size];

/* Execution trace. Plain ints: this is single core, and every writer runs in
 * thread context with no preemption source configured in this profile. */
constexpr std::size_t trace_capacity = 16;
int trace[trace_capacity];
std::size_t trace_length = 0;

void mark(int value)
{
   if (trace_length < trace_capacity) { trace[trace_length++] = value; }
}

bool trace_is(std::initializer_list<int> expected)
{
   if (trace_length != expected.size()) { return false; }
   std::size_t index = 0;
   for (int value : expected) {
      if (trace[index++] != value) { return false; }
   }
   return true;
}

void print_trace()
{
   cyros::bench::print("  trace = [");
   for (std::size_t i = 0; i < trace_length; ++i) {
      cyros::bench::print_hex(static_cast<std::uint32_t>(trace[i]));
      if (i + 1 < trace_length) { cyros::bench::print(", "); }
   }
   cyros::bench::print("]\n");
}


/* ---------------------------------------------------------------------------
 * The callee-saved register check
 *
 * r4-r11 are the registers the PendSV prologue has to stack by hand, because
 * the hardware does not. Writing a recognisable pattern into all eight, giving
 * up the CPU, and checking them afterwards is a direct test of that half of the
 * switch. `yield` is kept opaque to the compiler by the asm barrier, so the
 * values genuinely live in registers across it rather than being rematerialised
 * from the stack afterwards.
 * ------------------------------------------------------------------------ */

bool registers_survived_a_yield()
{
   std::uint32_t result = 0;

#if defined(__ARM_ARCH_6M__)
   /* The same check in Thumb-1, for the armv6m layer, whose PendSV moves r8 to
    * r11 through low registers. No MOVW and no IT block, so the patterns are
    * byte immediates and each mismatch sets its bit by a branch around an ADDS
    * (the bits are distinct, so adding is OR). Unified syntax, which GCC does
    * not select for Thumb-1 inline assembly on its own. */
   asm volatile(
      ".syntax unified           \n"
      "push  {r4-r7}             \n"
      "mov   r4, r8              \n"
      "mov   r5, r9              \n"
      "mov   r6, r10             \n"
      "mov   r7, r11             \n"
      "push  {r4-r7}             \n"
      "movs  r0, #0x88           \n"
      "mov   r8, r0              \n"
      "movs  r0, #0x99           \n"
      "mov   r9, r0              \n"
      "movs  r0, #0xAA           \n"
      "mov   r10, r0             \n"
      "movs  r0, #0xBB           \n"
      "mov   r11, r0             \n"
      "movs  r4, #0x44           \n"
      "movs  r5, #0x55           \n"
      "movs  r6, #0x66           \n"
      "movs  r7, #0x77           \n"
      "bl    cyros_bench_yield   \n"
      "movs  r0, #0              \n"
      "cmp   r4, #0x44           \n"
      "beq   1f                  \n"
      "adds  r0, #1              \n"
      "1:                        \n"
      "cmp   r5, #0x55           \n"
      "beq   2f                  \n"
      "adds  r0, #2              \n"
      "2:                        \n"
      "cmp   r6, #0x66           \n"
      "beq   3f                  \n"
      "adds  r0, #4              \n"
      "3:                        \n"
      "cmp   r7, #0x77           \n"
      "beq   4f                  \n"
      "adds  r0, #8              \n"
      "4:                        \n"
      "mov   r1, r8              \n"
      "cmp   r1, #0x88           \n"
      "beq   5f                  \n"
      "adds  r0, #16             \n"
      "5:                        \n"
      "mov   r1, r9              \n"
      "cmp   r1, #0x99           \n"
      "beq   6f                  \n"
      "adds  r0, #32             \n"
      "6:                        \n"
      "mov   r1, r10             \n"
      "cmp   r1, #0xAA           \n"
      "beq   7f                  \n"
      "adds  r0, #64             \n"
      "7:                        \n"
      "mov   r1, r11             \n"
      "cmp   r1, #0xBB           \n"
      "beq   8f                  \n"
      "adds  r0, #128            \n"
      "8:                        \n"
      /* Restore before handing the result out, as below. */
      "pop   {r4-r7}             \n"
      "mov   r8, r4              \n"
      "mov   r9, r5              \n"
      "mov   r10, r6             \n"
      "mov   r11, r7             \n"
      "pop   {r4-r7}             \n"
      "mov   %[out], r0          \n"
      : [out] "=r"(result)
      :
      : "r0", "r1", "r2", "r3", "r12", "lr", "cc", "memory");
#else
   asm volatile(
      "push  {r4-r11}            \n"
      "movw  r4,  #0x4444        \n"
      "movw  r5,  #0x5555        \n"
      "movw  r6,  #0x6666        \n"
      "movw  r7,  #0x7777        \n"
      "movw  r8,  #0x8888        \n"
      "movw  r9,  #0x9999        \n"
      "movw  r10, #0xAAAA        \n"
      "movw  r11, #0xBBBB        \n"
      "bl    cyros_bench_yield   \n"
      /* Accumulate every mismatch into r0 so one comparison covers all eight. */
      "movs  r0, #0              \n"
      "movw  r1, #0x4444         \n"
      "cmp   r4,  r1             \n"
      "it    ne                  \n"
      "orrne r0, r0, #1          \n"
      "movw  r1, #0x5555         \n"
      "cmp   r5,  r1             \n"
      "it    ne                  \n"
      "orrne r0, r0, #2          \n"
      "movw  r1, #0x6666         \n"
      "cmp   r6,  r1             \n"
      "it    ne                  \n"
      "orrne r0, r0, #4          \n"
      "movw  r1, #0x7777         \n"
      "cmp   r7,  r1             \n"
      "it    ne                  \n"
      "orrne r0, r0, #8          \n"
      "movw  r1, #0x8888         \n"
      "cmp   r8,  r1             \n"
      "it    ne                  \n"
      "orrne r0, r0, #16         \n"
      "movw  r1, #0x9999         \n"
      "cmp   r9,  r1             \n"
      "it    ne                  \n"
      "orrne r0, r0, #32         \n"
      "movw  r1, #0xAAAA         \n"
      "cmp   r10, r1             \n"
      "it    ne                  \n"
      "orrne r0, r0, #64         \n"
      "movw  r1, #0xBBBB         \n"
      "cmp   r11, r1             \n"
      "it    ne                  \n"
      "orrne r0, r0, #128        \n"
      /* Restore BEFORE handing the result out. %[out] is register-allocated by
       * the compiler and may well land in r4-r11, in which case writing it
       * first and popping second overwrites the answer with the saved value.
       * That is not hypothetical: it is what this test did on its first run,
       * and it reported a "clobbered mask" of 0x100038d0, a code address. */
      "pop   {r4-r11}            \n"
      "mov   %[out], r0          \n"
      : [out] "=r"(result)
      :
      : "r0", "r1", "r2", "r3", "r12", "lr", "cc", "memory");
#endif

   if (result != 0) {
      cyros::bench::print("  clobbered register mask = ");
      cyros::bench::print_hex(result);
      cyros::bench::print("  (bit 0 is r4, bit 7 is r11)\n");
   }
   return result == 0;
}

} // namespace


/* Called from the assembly above. Not static, and extern "C", so the `bl` finds
 * an unmangled symbol the linker can reach. */
extern "C" void cyros_bench_yield()
{
   this_thread::yield();
}


namespace
{

/* The other half of the check: thread_a holds DIFFERENT values in r4 to r11
 * across the yield that thread_b's check switches to. Without it, thread_b's
 * yield would find no other thread ready and come straight back, so PendSV
 * would save and restore the same thread's registers, and a switch that
 * dropped a register from both its save and its restore would pass. Its own
 * callee-saved registers are put back before it returns. */
void scramble_registers_across_a_yield()
{
#if defined(__ARM_ARCH_6M__)
   asm volatile(
      ".syntax unified           \n"
      "push  {r4-r7}             \n"
      "mov   r4, r8              \n"
      "mov   r5, r9              \n"
      "mov   r6, r10             \n"
      "mov   r7, r11             \n"
      "push  {r4-r7}             \n"
      "movs  r0, #0x18           \n"
      "mov   r8, r0              \n"
      "movs  r0, #0x19           \n"
      "mov   r9, r0              \n"
      "movs  r0, #0x1A           \n"
      "mov   r10, r0             \n"
      "movs  r0, #0x1B           \n"
      "mov   r11, r0             \n"
      "movs  r4, #0x14           \n"
      "movs  r5, #0x15           \n"
      "movs  r6, #0x16           \n"
      "movs  r7, #0x17           \n"
      "bl    cyros_bench_yield   \n"
      "pop   {r4-r7}             \n"
      "mov   r8, r4              \n"
      "mov   r9, r5              \n"
      "mov   r10, r6             \n"
      "mov   r11, r7             \n"
      "pop   {r4-r7}             \n"
      :
      :
      : "r0", "r1", "r2", "r3", "r12", "lr", "cc", "memory");
#else
   asm volatile(
      "push  {r4-r11}            \n"
      "movw  r4,  #0x1414        \n"
      "movw  r5,  #0x1515        \n"
      "movw  r6,  #0x1616        \n"
      "movw  r7,  #0x1717        \n"
      "movw  r8,  #0x1818        \n"
      "movw  r9,  #0x1919        \n"
      "movw  r10, #0x1A1A        \n"
      "movw  r11, #0x1B1B        \n"
      "bl    cyros_bench_yield   \n"
      "pop   {r4-r11}            \n"
      :
      :
      : "r0", "r1", "r2", "r3", "r12", "lr", "cc", "memory");
#endif
}

void thread_a()
{
   mark(1);
   this_thread::yield();
   mark(3);
   /* Ready, with other values in r4 to r11, when thread_b's check yields. */
   scramble_registers_across_a_yield();
}

void thread_b()
{
   mark(2);
   this_thread::yield();
   mark(4);

   /* Last thing to run. Everything below reports and exits the image. */
   cyros::bench::start("both threads ran, in registration order, across a yield");
   print_trace();
   CYROS_CHECK(trace_is({1, 2, 3, 4}));

   cyros::bench::start("callee-saved registers survive a context switch");
   CYROS_CHECK(registers_survived_a_yield());

   cyros::bench::start("kernel still reports its threads");
   CYROS_CHECK(kernel::core_count() == 1u);

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("cortex_m kernel bring-up\n\n");

   kernel::initialise();

   cyros::bench::start("threads register before the kernel starts");
   thread a(thread_a, stack_a, thread::priority(0), core0);
   thread b(thread_b, stack_b, thread::priority(0), core0);
   CYROS_CHECK_EQ(kernel::active_threads(), 2u);

   /* Does not return on this port. thread_b exits the image. */
   kernel::start();

   /* Only reachable if start() returned, which would itself be the defect. */
   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
