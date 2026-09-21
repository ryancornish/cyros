/**
 * @file test_cortex_m33_fpu.cpp
 * @brief Floating-point state survives a context switch.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the cortex_m33 port's FP handling: the FPU enable in
 *          cyros_port_init, and PendSV's conditional save of s16-s31.
 * Trusts:  layers 0 to 2, and the plain context switch that
 *          test_cortex_m33_bringup proves.
 * Proves:  that two threads can each hold different values in the FP register
 *          file across arbitrarily many switches without seeing each other's.
 *
 *
 * WHY A SEPARATE TEST, WHEN THE SUITE ALREADY PASSES HARD-FLOAT
 * ============================================================
 * Turning on `-mfloat-abi=hard` made every existing ARM test compile and pass
 * without touching the port's FP code at all, because NOTHING in the kernel or
 * in those tests uses a floating-point value. A port that saved no FP state
 * whatsoever would have looked exactly as green. The FP path has to be
 * exercised deliberately or it is not covered.
 *
 *
 * WHAT THE HARDWARE DOES, AND WHAT THE PORT HAS TO DO
 * ===================================================
 * The split is the thing to hold on to:
 *
 *   s0-s15, FPSCR    CALLER-saved. The hardware stacks these itself, into an
 *                    EXTENDED exception frame, and only for a thread that has
 *                    actually used the FPU. EXC_RETURN bit 4 (FType) records
 *                    which kind of frame a thread got.
 *   s16-s31          CALLEE-saved. The hardware does NOT touch them, so PendSV
 *                    must, exactly as it does for r4-r11.
 *
 * Both halves are checked below, because they fail differently: losing
 * s16-s31 is a missing instruction in the port, while losing s0-s15 means the
 * FPU was never enabled or the frame was the wrong kind.
 *
 * Lazy stacking is left at its reset default (FPCCR.LSPEN=1). The port's
 * conditional `vstmdb {s16-s31}` is itself an FP instruction, so executing it
 * forces any deferred save out to FPCAR, which still points at the outgoing
 * thread's frame, before anything switches.
 *
 *
 * WHY THE REGISTER WINDOW IS ONE ASM BLOCK
 * ========================================
 * The values have to live in the REGISTERS across the yield, not in memory
 * either side of it. Splitting load, yield and read-back into separate
 * statements would let the compiler keep them anywhere it liked, and the test
 * would then prove nothing about the register file. One asm block with the
 * `bl` inside it is the same shape the r4-r11 check in
 * test_cortex_m33_bringup uses, and for the same reason.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <common/arm/bench.hpp>

#include <cstddef>
#include <cstdint>

using namespace cyros;

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 2048;

alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_a[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_b[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_reporter[stack_size];

constexpr int rounds = 64;

/* Raw bit patterns rather than float literals: what matters is that the exact
 * 32 bits come back, and a NaN or a denormal would compare oddly as a float
 * while being a perfectly good thing for a register to hold. */
alignas(8) std::uint32_t pattern_a[16];
alignas(8) std::uint32_t pattern_b[16];
alignas(8) std::uint32_t readback_a[16];
alignas(8) std::uint32_t readback_b[16];

alignas(8) std::uint32_t low_pattern[16];
alignas(8) std::uint32_t low_readback[16];

volatile int a_mismatches = 0;
volatile int b_mismatches = 0;
volatile int low_mismatches = 0;
volatile int a_rounds = 0;
volatile int b_rounds = 0;
volatile bool low_done = false;

void fill(std::uint32_t* dst, std::uint32_t seed)
{
   for (std::uint32_t i = 0; i < 16u; ++i) {
      dst[i] = seed + (i * 0x01010101u);
   }
}

int compare(std::uint32_t const* lhs, std::uint32_t const* rhs)
{
   int bad = 0;
   for (int i = 0; i < 16; ++i) {
      if (lhs[i] != rhs[i]) { ++bad; }
   }
   return bad;
}

} // namespace


/* Called from the asm below. extern "C" so the `bl` finds an unmangled name. */
extern "C" void cyros_fpu_yield()
{
   this_thread::yield();
}


namespace
{

/**
 * @brief Put @p pattern in s16-s31, give up the CPU, read them back.
 *
 * s16-s31 are clobber-listed so the compiler knows the block destroys them and
 * saves anything of its own first. The values under test are loaded and read
 * back INSIDE the block, so that does not weaken the check.
 */
void round_trip_high(std::uint32_t const* pattern, std::uint32_t* readback)
{
   asm volatile(
      "vldmia %[pat], {s16-s31}  \n"
      "bl     cyros_fpu_yield    \n"
      "vstmia %[out], {s16-s31}  \n"
      :
      : [pat] "r"(pattern), [out] "r"(readback)
      : "r0", "r1", "r2", "r3", "r12", "lr", "cc", "memory",
        "s16", "s17", "s18", "s19", "s20", "s21", "s22", "s23",
        "s24", "s25", "s26", "s27", "s28", "s29", "s30", "s31");
}

/**
 * @brief The same, for s0-s15, which the HARDWARE is responsible for.
 *
 * These ride in the extended exception frame rather than being saved by the
 * port. If they come back wrong, the FPU was never enabled or the frame was
 * the standard one.
 */
void round_trip_low(std::uint32_t const* pattern, std::uint32_t* readback)
{
   asm volatile(
      "vldmia %[pat], {s0-s15}   \n"
      "bl     cyros_fpu_yield    \n"
      "vstmia %[out], {s0-s15}   \n"
      :
      : [pat] "r"(pattern), [out] "r"(readback)
      : "r0", "r1", "r2", "r3", "r12", "lr", "cc", "memory",
        "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
        "s8", "s9", "s10", "s11", "s12", "s13", "s14", "s15");
}

void thread_a()
{
   for (int i = 0; i < rounds; ++i) {
      round_trip_high(pattern_a, readback_a);
      a_mismatches = a_mismatches + compare(pattern_a, readback_a);
      a_rounds = a_rounds + 1;
   }

   /* The caller-saved half, once the contended part is done. */
   round_trip_low(low_pattern, low_readback);
   low_mismatches = compare(low_pattern, low_readback);
   low_done = true;
}

void thread_b()
{
   /* A DIFFERENT pattern, and that is the point. If PendSV dropped s16-s31,
    * each thread would read back the other's values rather than garbage, so
    * the two patterns must not overlap. */
   for (int i = 0; i < rounds; ++i) {
      round_trip_high(pattern_b, readback_b);
      b_mismatches = b_mismatches + compare(pattern_b, readback_b);
      b_rounds = b_rounds + 1;
   }
}

void reporter()
{
   cyros::bench::start("both threads completed their rounds");
   CYROS_CHECK_EQ(a_rounds, rounds);
   CYROS_CHECK_EQ(b_rounds, rounds);
   CYROS_CHECK(low_done);

   cyros::bench::start("s16-s31 survive a switch, per thread");
   cyros::bench::print("  thread A mismatches = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(a_mismatches));
   cyros::bench::print("\n  thread B mismatches = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(b_mismatches));
   cyros::bench::print("\n");
   CYROS_CHECK_EQ(a_mismatches, 0);
   CYROS_CHECK_EQ(b_mismatches, 0);

   cyros::bench::start("s0-s15 survive a switch, via the extended frame");
   cyros::bench::print("  mismatches = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(low_mismatches));
   cyros::bench::print("\n");
   CYROS_CHECK_EQ(low_mismatches, 0);

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("cortex_m33 FPU context\n\n");

   fill(pattern_a,   0xA0000001u);
   fill(pattern_b,   0xB0000002u);
   fill(low_pattern, 0xC0000003u);

   kernel::initialise();

   thread a(thread_a, stack_a, thread::priority(0), core0);
   thread b(thread_b, stack_b, thread::priority(0), core0);
   /* Lowest priority, so it runs only once both workers have finished. */
   thread r(reporter, stack_reporter, thread::priority(7), core0);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
