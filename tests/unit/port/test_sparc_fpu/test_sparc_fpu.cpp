/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_sparc_fpu.cpp
 * @brief Floating-point state survives both ways into the sparcv8 layer's
 *        trap path, across a switch.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the sparcv8 layer's FP handling: the FP half of every trap frame
 *          (FSR and f0-f31), saved and restored when the PSR has EF.
 * Trusts:  layers 0 to 5, the semaphore (test_sync_semaphore), and the plain
 *          context switch test_sparc_bringup proves.
 * Proves:  that a thread's whole FP register file and FSR come back after a
 *          yield (`ta 9`) that ran another thread holding different values,
 *          and after an interrupt whose reschedule ran a more urgent thread
 *          that overwrote every FP register.
 *
 *
 * WHY A SEPARATE TEST, WHEN THE SUITE ALREADY PASSES HARD-FLOAT
 * ============================================================
 * test_armv8r_fpu's reason: nothing in the kernel or the other tests holds a
 * floating-point value across a switch, so a port that saved no FP state at
 * all would look as green. Each window below is one asm block, so the values
 * live in the REGISTERS across the switch and nowhere else.
 *
 * The interrupt path's window: with interrupts masked the test releases the
 * semaphore a more urgent thread waits on, which pends the reschedule, then
 * loads its pattern and clears PIL's interrupt field inside the block. The
 * reschedule is taken there, the urgent thread runs and loads its own pattern
 * into every FP register, blocks again, and the window reads back.
 *
 * FSR's pattern sets the rounding direction, the FP condition codes and the
 * accrued and current exception flags, which a lost FSR would reset. Its trap
 * enables stay clear.
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

/* f0-f31 as 32 words, then FSR. Raw bits, as test_armv8r_fpu's: what
 * matters is that the exact bits come back. */
struct fp_state
{
   std::uint32_t f[32];
   std::uint32_t fsr;
};

alignas(8) fp_state pattern_a{};
alignas(8) fp_state pattern_b{};
alignas(8) fp_state pattern_urgent{};
alignas(8) fp_state readback_a{};
alignas(8) fp_state readback_b{};

void fill(fp_state& state, std::uint32_t seed, std::uint32_t fsr)
{
   for (std::uint32_t i = 0; i < 32u; ++i) {
      state.f[i] = seed + (i * 0x01010101u);
   }
   state.fsr = fsr;
}

int compare(fp_state const& lhs, fp_state const& rhs)
{
   int bad = 0;
   for (int i = 0; i < 32; ++i) {
      if (lhs.f[i] != rhs.f[i]) { ++bad; }
   }
   /* The writable bits FSR's pattern uses: the rounding direction (31:30), the
    * FP condition codes (11:10) and the accrued and current exceptions. */
   if ((lhs.fsr & 0xC0000FFFu) != (rhs.fsr & 0xC0000FFFu)) { ++bad; }
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


/* Called from the asm below. extern "C" so the `call` finds an unmangled name. */
extern "C" void cyros_fpu_yield()
{
   this_thread::yield();
}


namespace
{

#define CYROS_TEST_FP_CLOBBERS                                                    \
   "f0", "f1", "f2", "f3", "f4", "f5", "f6", "f7",                                \
   "f8", "f9", "f10", "f11", "f12", "f13", "f14", "f15",                          \
   "f16", "f17", "f18", "f19", "f20", "f21", "f22", "f23",                        \
   "f24", "f25", "f26", "f27", "f28", "f29", "f30", "f31"

/* A call clobbers the outs and g1 to g4, so naming them keeps the compiler's
 * pattern and readback pointers in locals across it. */
#define CYROS_TEST_CALL_CLOBBERS                                                  \
   "o0", "o1", "o2", "o3", "o4", "o5", "o7", "g1", "g2", "g3", "g4"

#define CYROS_TEST_FP_LOAD                                                        \
   "ld     [%[pat] + 128], %%fsr        \n"                                      \
   ".irp n, 0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30 \n"                       \
   "ldd    [%[pat] + 4 * \\n], %%f\\n   \n"                                      \
   ".endr                               \n"

#define CYROS_TEST_FP_STORE                                                       \
   ".irp n, 0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30 \n"                       \
   "std    %%f\\n, [%[out] + 4 * \\n]   \n"                                      \
   ".endr                               \n"                                      \
   "st     %%fsr, [%[out] + 128]        \n"

/** Load @p pattern into f0-f31 and FSR, yield, read them all back. */
void round_trip_by_yield(fp_state const* pattern, fp_state* readback)
{
   asm volatile(
      CYROS_TEST_FP_LOAD
      "call   cyros_fpu_yield              \n"
      " nop                                \n"
      CYROS_TEST_FP_STORE
      :
      : [pat] "r"(pattern), [out] "r"(readback)
      : "cc", "memory", CYROS_TEST_CALL_CLOBBERS, CYROS_TEST_FP_CLOBBERS);
}

/** The same across an interrupt: masked on entry, with a reschedule pended
 * that runs the urgent thread, which is taken when PIL's interrupt field
 * clears inside. */
void round_trip_by_interrupt(fp_state const* pattern, fp_state* readback)
{
   asm volatile(
      CYROS_TEST_FP_LOAD
      "rd     %%psr, %%g1                  \n"
      "andn   %%g1, 0xe00, %%g1            \n"
      "wr     %%g1, %%psr                  \n"   /* the reschedule is taken here */
      "nop                                 \n"
      "nop                                 \n"
      "nop                                 \n"
      "rd     %%psr, %%g1                  \n"
      "or     %%g1, 0xe00, %%g1            \n"
      "wr     %%g1, %%psr                  \n"
      "nop                                 \n"
      "nop                                 \n"
      "nop                                 \n"
      CYROS_TEST_FP_STORE
      :
      : [pat] "r"(pattern), [out] "r"(readback)
      : "g1", "cc", "memory", CYROS_TEST_FP_CLOBBERS);
}

/** The urgent thread's own values, in every FP register. */
void load_everything(fp_state const* pattern)
{
   asm volatile(
      "ld     [%[pat] + 128], %%fsr        \n"
      ".irp n, 0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30 \n"
      "ldd    [%[pat] + 4 * \\n], %%f\\n   \n"
      ".endr                               \n"
      :
      : [pat] "r"(pattern)
      : "memory", CYROS_TEST_FP_CLOBBERS);
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

   cyros::bench::start("f0-f31 and FSR survive a yield, per thread");
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

   cyros::bench::start("f0-f31 and FSR survive an interrupt that ran another thread");
   cyros::bench::print("  mismatches = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(irq_mismatches));
   cyros::bench::print("\n");
   CYROS_CHECK_EQ(irq_mismatches, 0);

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("sparcv8 FPU context\n\n");

   /* FSR: rounding direction (31:30), fcc (11:10), aexc (9:5), cexc (4:0). */
   fill(pattern_a,      0xA0000001u, 0x80000C3Au);   /* to +inf, fcc 3      */
   fill(pattern_b,      0xB0000002u, 0xC0000425u);   /* to -inf, fcc 1      */
   fill(pattern_urgent, 0xC0000003u, 0x40000800u);   /* to zero, fcc 2      */

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
