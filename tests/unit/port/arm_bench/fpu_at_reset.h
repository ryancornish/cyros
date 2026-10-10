/**
 * @file fpu_at_reset.h
 * @brief The FPU on before any compiled code runs, for the ARM benches' reset
 *        handlers.
 *
 * A hard-float image may execute an FP instruction anywhere: GCC at -O2 moves
 * 64-bit values through the d registers, in the kernel and the port as much
 * as in a test. The port turns the FPU on only in cyros_port_init, so code
 * before that, a reset handler or a test driving the port before the kernel,
 * faults NOCP. Turning it on is the reset path's job, as CMSIS's SystemInit
 * does, and it has to happen before any C, so each reset handler is a naked
 * entry that does this and then continues in C. A soft-float build (the M3)
 * has no FPU to turn on, and the entry only branches.
 *
 * Found on the RP2350 (~/cyros-claude/rp2350-notes.md 11c), whose board does
 * the same in startup_rp2350_m33.c.
 */
#ifndef CYROS_BENCH_FPU_AT_RESET_H
#define CYROS_BENCH_FPU_AT_RESET_H

#if defined(__ARM_FP)
#  define BENCH_FPU_ON                   \
      "ldr   r0, =0xE000ED88       \n"  \
      "ldr   r1, [r0]              \n"  \
      "orr   r1, r1, #(0xF << 20)  \n"  \
      "str   r1, [r0]              \n"  \
      "dsb                         \n"  \
      "isb                         \n"
#else
#  define BENCH_FPU_ON ""
#endif

/* A naked entry `entry` that turns the FPU on and branches to `in_c`, which
 * must be an external, used, noreturn C function. */
#define BENCH_RESET_ENTRY(entry, in_c)                                  \
   __attribute__((naked, noreturn)) void entry(void)                    \
   {                                                                    \
      __asm__ volatile(BENCH_FPU_ON "b " #in_c "\n.ltorg\n");           \
   }

#endif /* CYROS_BENCH_FPU_AT_RESET_H */
