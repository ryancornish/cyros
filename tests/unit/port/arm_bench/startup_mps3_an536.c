/**
 * @file startup_mps3_an536.c
 * @brief Reset path, vector table and console for cyros's on-target tests on
 *        QEMU's mps3-an536, two Cortex-R52s, the R-profile sibling of
 *        startup_mps2_an505.c.
 *
 * QEMU starts EVERY core at the image's entry, in EL2 (Hyp mode) with IRQ,
 * FIQ and asynchronous aborts masked. Each core:
 *
 *  1. lets EL1 keep what the port needs, which the R52 already allows at reset
 *     but firmware on a board may not: the physical counter and timer
 *     (CNTHCTL), the GIC's system registers (ICC_HSRE) and the FPU (HCPTR),
 *  2. drops to EL1, Supervisor mode, still masked,
 *  3. in a hard-float image, enables the FPU, before any compiled code: GCC
 *     moves 64-bit values through d registers (fpu_at_reset.h's reason on
 *     M-profile),
 *  4. points VBAR at the vector table below and gives each mode its stack:
 *     Abort and Undefined their own, Supervisor a small one that serves as the
 *     interrupt stack until the port names the real one, System the stack
 *     main and then the kernel's bring-up run on,
 *  5. core 0 runs the image, in System mode. Every other core parks in WFE
 *     until the port's release word is set (cortex_r52_smp's start_cores) and
 *     then joins the kernel through cyros_port_secondary_core_entry. A
 *     single-core image has neither symbol, and its other cores park for good.
 *
 * The park touches no memory but the release word, because core 0 is zeroing
 * .bss under it.
 *
 * The vector table is the BOARD's, as on M-profile: IRQ and SVC go to the
 * port's entries, the faults to weak handlers here that report and exit, which
 * a test may replace. Output and the exit status are semihosting (HLT), which
 * QEMU services when run with -semihosting-config.
 */

#include <stdint.h>

extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern int cyros_bench_main(void);
extern void (*__init_array_start[])(void);
extern void (*__init_array_end[])(void);

/* MTIME is the generic timer here (cortex_r52's port_mcu), and its rate is
 * whatever the counter's frequency register says. QEMU sets 62.5 MHz. */
uint32_t cyros_port_mtime_clock_hz(void)
{
   uint32_t hz;
   __asm__ volatile("mrc p15, 0, %0, c14, c0, 0" : "=r"(hz));
   return hz;
}

/* ---------------------------------------------------------------------------
 * Semihosting, open-coded so the reset path depends on nothing
 * ------------------------------------------------------------------------ */

/* Also bench.hpp's semihosting on this board. */
__attribute__((noinline)) long cyros_bench_semihost(long op, void volatile* arg)
{
   register long r0 __asm__("r0") = op;
   register void volatile* r1 __asm__("r1") = arg;
   __asm__ volatile("hlt 0x3c" : "+r"(r0) : "r"(r1) : "memory");
   return r0;
}

void cyros_bench_write(char const* text)
{
   cyros_bench_semihost(0x04, (void volatile*)text);
}

__attribute__((noreturn)) void cyros_bench_exit(uint32_t code)
{
   volatile uint32_t block[2] = { 0x20026u, code };   /* ADP_Stopped_ApplicationExit */
   cyros_bench_semihost(0x20, block);                 /* SYS_EXIT_EXTENDED           */
   for (;;) { __asm__ volatile("wfi"); }
}

/* ---------------------------------------------------------------------------
 * Faults
 * ------------------------------------------------------------------------ */

static void write_hex(uint32_t value)
{
   char buffer[11] = { '0', 'x' };
   for (int i = 0; i < 8; ++i) {
      uint32_t const nibble = (value >> ((7 - i) * 4)) & 0xFu;
      buffer[2 + i] = (char)(nibble < 10 ? '0' + nibble : 'a' + (nibble - 10));
   }
   buffer[10] = '\0';
   cyros_bench_write(buffer);
}

enum fault_kind { fault_undefined, fault_prefetch, fault_data, fault_fiq, fault_unused };

__attribute__((noreturn, used)) void an536_fault_report(uint32_t kind, uint32_t lr, uint32_t spsr)
{
   static char const* const names[] = { "Undefined", "Prefetch Abort", "Data Abort", "FIQ", "unused vector" };
   uint32_t core;
   __asm__ volatile("mrc p15, 0, %0, c0, c0, 5" : "=r"(core));
   cyros_bench_write("\n*** FAULT on core ");
   write_hex(core & 0xFFu);
   cyros_bench_write(": ");
   cyros_bench_write(names[kind]);
   cyros_bench_write(", lr ");
   write_hex(lr);
   cyros_bench_write(", spsr ");
   write_hex(spsr);
   if (kind == fault_data) {
      uint32_t dfsr, dfar;
      __asm__ volatile("mrc p15, 0, %0, c5, c0, 0" : "=r"(dfsr));
      __asm__ volatile("mrc p15, 0, %0, c6, c0, 0" : "=r"(dfar));
      cyros_bench_write(", DFSR ");
      write_hex(dfsr);
      cyros_bench_write(", DFAR ");
      write_hex(dfar);
   }
   if (kind == fault_prefetch) {
      uint32_t ifsr, ifar;
      __asm__ volatile("mrc p15, 0, %0, c5, c0, 1" : "=r"(ifsr));
      __asm__ volatile("mrc p15, 0, %0, c6, c0, 2" : "=r"(ifar));
      cyros_bench_write(", IFSR ");
      write_hex(ifsr);
      cyros_bench_write(", IFAR ");
      write_hex(ifar);
   }
   cyros_bench_write(" ***\n");
   cyros_bench_exit(3u);
}

/* Each reports its kind, the exception's lr and the SPSR. Weak, so a test may
 * take a fault itself, as test_cortex_m_stack_guard does on M-profile. */
#define AN536_FAULT(name, kind)                                                  \
   __attribute__((naked, weak)) void name(void)                                 \
   {                                                                            \
      __asm__ volatile("movs r0, #" #kind " \n"                                  \
                       "mov  r1, lr         \n"                                  \
                       "mrs  r2, spsr       \n"                                  \
                       "b    an536_fault_report \n");                            \
   }

AN536_FAULT(Undefined_Handler,     0)
AN536_FAULT(PrefetchAbort_Handler, 1)
AN536_FAULT(DataAbort_Handler,     2)
AN536_FAULT(FIQ_Handler,           3)
AN536_FAULT(Unused_Handler,        4)

/* ---------------------------------------------------------------------------
 * The vector table, in ARM state (SCTLR.TE clear), each slot loading its
 * handler's address from the word eight slots on, which interworks into the
 * Thumb handlers. VBAR needs 32-byte alignment.
 * ------------------------------------------------------------------------ */

extern void cyros_port_irq_entry(void);
extern void cyros_port_svc_entry(void);

__asm__(
   ".pushsection .vectors, \"ax\", %progbits \n"
   ".arm                                    \n"
   ".balign 32                              \n"
   ".global an536_vectors                   \n"
   "an536_vectors:                          \n"
   "   ldr pc, [pc, #24]                    \n"   /* reset: never taken at EL1 */
   "   ldr pc, [pc, #24]                    \n"   /* undefined instruction     */
   "   ldr pc, [pc, #24]                    \n"   /* SVC                       */
   "   ldr pc, [pc, #24]                    \n"   /* prefetch abort            */
   "   ldr pc, [pc, #24]                    \n"   /* data abort                */
   "   ldr pc, [pc, #24]                    \n"   /* unused (Hyp trap at EL2)  */
   "   ldr pc, [pc, #24]                    \n"   /* IRQ                       */
   "   ldr pc, [pc, #24]                    \n"   /* FIQ                       */
   "   .word Unused_Handler                 \n"
   "   .word Undefined_Handler              \n"
   "   .word cyros_port_svc_entry           \n"
   "   .word PrefetchAbort_Handler          \n"
   "   .word DataAbort_Handler              \n"
   "   .word Unused_Handler                 \n"
   "   .word cyros_port_irq_entry           \n"
   "   .word FIQ_Handler                    \n"
   ".thumb                                  \n"
   ".popsection                             \n");

/* ---------------------------------------------------------------------------
 * Reset
 * ------------------------------------------------------------------------ */

__attribute__((noreturn, used)) void reset_c(void)
{
   for (uint32_t* target = &__bss_start__; target < &__bss_end__; ) {
      *target++ = 0u;
   }
   for (void (**ctor)(void) = __init_array_start; ctor != __init_array_end; ++ctor) {
      (*ctor)();
   }
   cyros_bench_exit((uint32_t)cyros_bench_main());
}

/* Per-core stacks. Core 0's System stack is the linker script's __stack_top,
 * which becomes its interrupt stack once cyros starts its first thread. */
#define MAX_CORES          2
#define MODE_STACK_BYTES   1024
#define SVC_STACK_BYTES    4096
#define SYS_STACK_BYTES    16384

__attribute__((aligned(8), used)) uint8_t an536_abt_stacks[MAX_CORES * MODE_STACK_BYTES];
__attribute__((aligned(8), used)) uint8_t an536_und_stacks[MAX_CORES * MODE_STACK_BYTES];
__attribute__((aligned(8), used)) uint8_t an536_irq_stacks[MAX_CORES * MODE_STACK_BYTES];
__attribute__((aligned(8), used)) uint8_t an536_svc_stacks[MAX_CORES * SVC_STACK_BYTES];
__attribute__((aligned(8), used)) uint8_t an536_sys_stacks[(MAX_CORES - 1) * SYS_STACK_BYTES];

/* Weak, so a single-core image links without them. Their addresses are
 * loaded absolutely and tested for zero before use. */
extern void cyros_port_secondary_core_entry(void) __attribute__((weak));
extern volatile uint32_t cyros_armv8r_release __attribute__((weak));

#define STR_(x) #x
#define STR(x)  STR_(x)

/* CPACR's CP10 and CP11 at full access, then FPEXC.EN. Nothing in a soft-float
 * image may touch the FPU, so there the board leaves it off, as the port does. */
#if defined(__ARM_FP)
#define AN536_FPU_ON                                     \
   "mrc   p15, 0, r0, c1, c0, 2                \n"      \
   "orr   r0, r0, #(0xf << 20)                 \n"      \
   "mcr   p15, 0, r0, c1, c0, 2                \n"      \
   "isb                                       \n"      \
   "mov   r0, #(1 << 30)                       \n"      \
   "vmsr  fpexc, r0                           \n"      \
   "isb                                       \n"
#else
#define AN536_FPU_ON ""
#endif

/* EL1 from here: core number in r4. */
__attribute__((naked, used)) void an536_el1_start(void)
{
   __asm__ volatile(
      ".weak cyros_armv8r_release                \n"
      ".weak cyros_port_secondary_core_entry     \n"
      /* VBAR, and SCTLR's V (high vectors) and TE (Thumb exceptions) clear. */
      "ldr   r0, =an536_vectors                  \n"
      "mcr   p15, 0, r0, c12, c0, 0              \n"
      "mrc   p15, 0, r0, c1, c0, 0               \n"
      "bic   r0, r0, #(1 << 13)                  \n"
      "bic   r0, r0, #(1 << 30)                  \n"
      "mcr   p15, 0, r0, c1, c0, 0               \n"
      "isb                                       \n"
      AN536_FPU_ON
      /* r5 = core + 1: each mode's stack top is its array + (core + 1) * size. */
      "add   r5, r4, #1                          \n"
      "cps   #0x17                               \n"   /* Abort      */
      "ldr   r0, =an536_abt_stacks               \n"
      "mov   r1, #" STR(MODE_STACK_BYTES) "      \n"
      "mla   r0, r1, r5, r0                      \n"
      "mov   sp, r0                              \n"
      "cps   #0x1b                               \n"   /* Undefined  */
      "ldr   r0, =an536_und_stacks               \n"
      "mla   r0, r1, r5, r0                      \n"
      "mov   sp, r0                              \n"
      "cps   #0x12                               \n"   /* IRQ, unused by the port */
      "ldr   r0, =an536_irq_stacks               \n"
      "mla   r0, r1, r5, r0                      \n"
      "mov   sp, r0                              \n"
      "cps   #0x13                               \n"   /* Supervisor */
      "ldr   r0, =an536_svc_stacks               \n"
      "mov   r1, #" STR(SVC_STACK_BYTES) "       \n"
      "mla   r0, r1, r5, r0                      \n"
      "mov   sp, r0                              \n"
      "cps   #0x1f                               \n"   /* System     */
      "cbnz  r4, 1f                              \n"
      "ldr   sp, =__stack_top                    \n"
      "b     reset_c                             \n"
      /* Any other core: its System stack, then the park. */
      "1:                                        \n"
      "ldr   r0, =an536_sys_stacks               \n"
      "ldr   r1, =" STR(SYS_STACK_BYTES) "       \n"
      "mla   r0, r1, r4, r0                      \n"
      "mov   sp, r0                              \n"
      "ldr   r2, =cyros_armv8r_release           \n"
      "cbz   r2, 3f                              \n"
      "2:                                        \n"
      "wfe                                       \n"
      "ldr   r0, [r2]                            \n"
      "cmp   r0, #0                              \n"
      "beq   2b                                  \n"
      "ldr   r1, =cyros_port_secondary_core_entry \n"
      "cbz   r1, 3f                              \n"
      "bx    r1                                  \n"
      "3:                                        \n"
      "wfi                                       \n"
      "b     3b                                  \n"
      ".ltorg                                    \n");
}

__attribute__((naked, section(".text.reset"))) void _start(void)
{
   __asm__ volatile(
      "mrc   p15, 0, r4, c0, c0, 5               \n"   /* MPIDR       */
      "and   r4, r4, #0xff                       \n"   /* Aff0, the core */
      /* EL2: EL1 may use the physical counter and timer, the GIC's system
       * registers and the FPU. CNTVOFF zero, should anything read the
       * virtual count. */
      "mrc   p15, 4, r0, c14, c1, 0              \n"   /* CNTHCTL     */
      "orr   r0, r0, #3                          \n"   /* EL1PCTEN, EL1PCEN */
      "mcr   p15, 4, r0, c14, c1, 0              \n"
      "mrc   p15, 4, r0, c12, c9, 5              \n"   /* ICC_HSRE    */
      "orr   r0, r0, #9                          \n"   /* SRE, Enable */
      "mcr   p15, 4, r0, c12, c9, 5              \n"
      "mrc   p15, 4, r0, c1, c1, 2               \n"   /* HCPTR       */
      "bic   r0, r0, #(3 << 10)                  \n"   /* TCP10, TCP11 */
      "bic   r0, r0, #(1 << 15)                  \n"   /* TASE        */
      "mcr   p15, 4, r0, c1, c1, 2               \n"
      "mov   r0, #0                              \n"
      "mov   r1, #0                              \n"
      "mcrr  p15, 4, r0, r1, c14                 \n"   /* CNTVOFF     */
      "isb                                       \n"
      /* To EL1 Supervisor, masked, in Thumb. */
      "ldr   r0, =an536_el1_start                \n"
      "bic   r0, r0, #1                          \n"
      "msr   elr_hyp, r0                         \n"
      "movw  r0, #0x1f3                          \n"   /* SVC | T | F | I | A */
      "msr   spsr_hyp, r0                        \n"
      "eret                                      \n"
      ".ltorg                                    \n");
}
