/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_armv8r_fpu.cpp
 * @brief Floating-point state survives both ways into the armv8r layer's
 *        exception path, across a switch.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the armv8r layer's FP handling: the FPU enable, and the FP half of
 *          every exception frame (FPSCR, d0-d15, and d16-d31 where MVFR0 says
 *          they exist).
 * Trusts:  layers 0 to 5, the semaphore (test_sync_semaphore), and the plain
 *          context switch test_armv8r_bringup proves.
 * Proves:  that a thread's whole FP register file and FPSCR come back after a
 *          yield (the SVC path) that ran another thread holding different
 *          values, and after an interrupt (the IRQ path) whose reschedule ran
 *          a more urgent thread that overwrote every FP register.
 *
 *
 * WHY A SEPARATE TEST, WHEN THE SUITE ALREADY PASSES HARD-FLOAT
 * ============================================================
 * test_cortex_m_fpu's reason: nothing in the kernel or the other tests holds a
 * floating-point value across a switch, so a port that saved no FP state at
 * all would look as green. Each window below is one asm block, so the values
 * live in the REGISTERS across the switch and nowhere else.
 *
 * The IRQ path's window: with interrupts masked the test releases the
 * semaphore a more urgent thread waits on, which pends the reschedule, then
 * loads its pattern and unmasks inside the block. The reschedule is taken
 * there, the urgent thread runs and loads its own pattern into every FP
 * register, blocks again, and the window reads back.
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

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 4096;

alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_a[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_b[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_urgent[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_reporter[stack_size];

constexpr int rounds = 32;

/* d0-d31 as 64 words, then FPSCR. Raw bits, as test_cortex_m_fpu's: what
 * matters is that the exact bits come back. FPSCR's pattern sets N, Z, C and
 * V and a rounding mode, which a lost FPSCR would reset. */
struct fp_state
{
   std::uint32_t d[64];
   std::uint32_t fpscr;
};

alignas(8) fp_state pattern_a{};
alignas(8) fp_state pattern_b{};
alignas(8) fp_state pattern_urgent{};
alignas(8) fp_state readback_a{};
alignas(8) fp_state readback_b{};

void fill(fp_state& state, std::uint32_t seed, std::uint32_t fpscr)
{
   for (std::uint32_t i = 0; i < 64u; ++i) {
      state.d[i] = seed + (i * 0x01010101u);
   }
   state.fpscr = fpscr;
}

int compare(fp_state const& lhs, fp_state const& rhs)
{
   int bad = 0;
   for (int i = 0; i < 64; ++i) {
      if (lhs.d[i] != rhs.d[i]) { ++bad; }
   }
   /* The writable control and flag bits only. */
   if ((lhs.fpscr & 0xF0C00000u) != (rhs.fpscr & 0xF0C00000u)) { ++bad; }
   return bad;
}

sync::semaphore urgent_wake{0};

volatile int a_mismatches = 0;
volatile int b_mismatches = 0;
volatile int irq_mismatches = 0;
volatile int a_rounds = 0;
volatile int b_rounds = 0;
volatile int irq_rounds = 0;
volatile int urgent_ran = 0;
volatile bool a_done = false;
volatile bool b_done = false;
volatile bool irq_done = false;

} // namespace


/* Called from the asm below. extern "C" so the `bl` finds an unmangled name, and
 * used because no call the compiler can see reaches it, so LTO would drop it. */
extern "C" [[gnu::used]] void cyros_fpu_yield()
{
   this_thread::yield();
}


namespace
{

#define CYROS_TEST_FP_CLOBBERS                                                    \
   "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",                                \
   "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15",                          \
   "d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",                        \
   "d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31"

/** Load @p pattern into d0-d31 and FPSCR, yield, read them all back. */
void round_trip_by_yield(fp_state const* pattern, fp_state* readback)
{
   asm volatile(
      "ldr    r0, [%[pat], #256]    \n"
      "vmsr   fpscr, r0             \n"
      "vldmia %[pat], {d0-d15}      \n"
      "add    r0, %[pat], #128      \n"
      "vldmia r0, {d16-d31}         \n"
      "bl     cyros_fpu_yield       \n"
      "vstmia %[out], {d0-d15}      \n"
      "add    r0, %[out], #128      \n"
      "vstmia r0, {d16-d31}         \n"
      "vmrs   r0, fpscr             \n"
      "str    r0, [%[out], #256]    \n"
      :
      : [pat] "r"(pattern), [out] "r"(readback)
      : "r0", "r1", "r2", "r3", "r12", "lr", "cc", "memory", CYROS_TEST_FP_CLOBBERS);
}

/** The same across an interrupt: masked on entry, with a reschedule pended
 * that runs the urgent thread, which is taken at the unmask inside. */
void round_trip_by_interrupt(fp_state const* pattern, fp_state* readback)
{
   asm volatile(
      "ldr    r0, [%[pat], #256]    \n"
      "vmsr   fpscr, r0             \n"
      "vldmia %[pat], {d0-d15}      \n"
      "add    r0, %[pat], #128      \n"
      "vldmia r0, {d16-d31}         \n"
      "cpsie  i                     \n"   /* the reschedule is taken here */
      "isb                          \n"
      "nop                          \n"
      "nop                          \n"
      "cpsid  i                     \n"
      "vstmia %[out], {d0-d15}      \n"
      "add    r0, %[out], #128      \n"
      "vstmia r0, {d16-d31}         \n"
      "vmrs   r0, fpscr             \n"
      "str    r0, [%[out], #256]    \n"
      :
      : [pat] "r"(pattern), [out] "r"(readback)
      : "r0", "cc", "memory", CYROS_TEST_FP_CLOBBERS);
}

/** The urgent thread's own values, in every FP register. */
void load_everything(fp_state const* pattern)
{
   asm volatile(
      "ldr    r0, [%[pat], #256]    \n"
      "vmsr   fpscr, r0             \n"
      "vldmia %[pat], {d0-d15}      \n"
      "add    r0, %[pat], #128      \n"
      "vldmia r0, {d16-d31}         \n"
      :
      : [pat] "r"(pattern)
      : "r0", "memory", CYROS_TEST_FP_CLOBBERS);
}

void urgent()
{
   while (true) {
      urgent_wake.acquire();
      load_everything(&pattern_urgent);
      urgent_ran = urgent_ran + 1;
   }
}

void thread_a()
{
   for (int i = 0; i < rounds; ++i) {
      round_trip_by_yield(&pattern_a, &readback_a);
      a_mismatches = a_mismatches + compare(pattern_a, readback_a);
      a_rounds = a_rounds + 1;
   }
   a_done = true;
   while (!b_done) { this_thread::yield(); }

   /* The IRQ path. B is idle in a yield loop of its own, at A's priority, so
    * only the urgent thread runs inside the window. */
   for (int i = 0; i < rounds; ++i) {
      int const before = urgent_ran;
      cyros_mask_token_t const token = cyros_port_irq_save();
      urgent_wake.release();
      round_trip_by_interrupt(&pattern_a, &readback_a);
      cyros_port_irq_restore(token);
      if (urgent_ran == before + 1) {
         irq_rounds = irq_rounds + 1;
      }
      irq_mismatches = irq_mismatches + compare(pattern_a, readback_a);
   }
   irq_done = true;
}

void thread_b()
{
   /* A DIFFERENT pattern, and that is the point: a frame that dropped the FP
    * state would hand each thread the other's values. */
   for (int i = 0; i < rounds; ++i) {
      round_trip_by_yield(&pattern_b, &readback_b);
      b_mismatches = b_mismatches + compare(pattern_b, readback_b);
      b_rounds = b_rounds + 1;
   }
   b_done = true;
   while (!irq_done) { this_thread::yield(); }
}

void reporter()
{
   cyros::bench::start("both yielding threads completed their rounds");
   CYROS_CHECK_EQ(a_rounds, rounds);
   CYROS_CHECK_EQ(b_rounds, rounds);

   cyros::bench::start("d0-d31 and FPSCR survive a yield, per thread");
   cyros::bench::print("  thread A mismatches = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(a_mismatches));
   cyros::bench::print("\n  thread B mismatches = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(b_mismatches));
   cyros::bench::print("\n");
   CYROS_CHECK_EQ(a_mismatches, 0);
   CYROS_CHECK_EQ(b_mismatches, 0);

   cyros::bench::start("each interrupt ran the urgent thread inside the window");
   CYROS_CHECK_EQ(irq_rounds, rounds);
   CYROS_CHECK_EQ(urgent_ran, rounds);

   cyros::bench::start("d0-d31 and FPSCR survive an interrupt that ran another thread");
   cyros::bench::print("  mismatches = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(irq_mismatches));
   cyros::bench::print("\n");
   CYROS_CHECK_EQ(irq_mismatches, 0);

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("armv8r FPU context\n\n");

   fill(pattern_a,      0xA0000001u, 0xF0400000u);   /* NZCV, round to +inf  */
   fill(pattern_b,      0xB0000002u, 0x50800000u);   /* Z and V, round to -inf */
   fill(pattern_urgent, 0xC0000003u, 0x00C00000u);   /* round to zero        */

   kernel::initialise();

   /* The urgent thread outranks everything, A and B share a priority and
    * hand over by yielding, and the reporter runs only once A and B have
    * finished and the urgent thread is blocked, so it reports last. */
   thread u(urgent, stack_urgent, thread::priority(1), core0);
   thread a(thread_a, stack_a, thread::priority(2), core0);
   thread b(thread_b, stack_b, thread::priority(2), core0);
   thread r(reporter, stack_reporter, thread::priority(3), core0);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
