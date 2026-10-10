/**
 * @file startup_rp2350_hazard3.c
 * @brief Reset path for cyros's on-target tests on the Pico 2 W's Hazard3 core
 *        0, the RISC-V sibling of ../u575/startup_stm32u575.c.
 *
 * The image runs from SRAM. It is entered at _entry either by the boot ROM,
 * which finds the IMAGE_DEF block below (`picotool load -x`, the boot path a
 * product takes and the runner's default), or by the debugger after a reset
 * halt (RP2350_BOOT=swd). _entry sets sp and mtvec before any C runs, because
 * a debugger start leaves whatever the boot ROM had. Core 1 stays in the boot
 * ROM's holding pen until the rp2350_hazard3_smp target launches it on
 * core1_stack below.
 *
 * Then, in order:
 *
 *   1. the console, and rtt_ready, so the runner can read it;
 *   2. the runner's go word. OpenOCD's examination of the harts halts them,
 *      and so does its handling of the boot's reset, which also writes a
 *      stale s0 into each hart (rp2350-notes.md 7b). Nothing may run before
 *      both are over, so the runner says go only then;
 *   3. 150 MHz from the crystal, and MTIME counting it (FULLSPEED);
 *   4. constructors, then cyros_bench_main.
 *
 * MTIME_CTRL keeps DBGPAUSE0, so a debug halt of core 0 stops time with the
 * core. It clears DBGPAUSE1, because OpenOCD halts the idle core 1 too.
 */
#include "board.h"
#include "rp2350.h"

#include <stdint.h>

extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __stack_top;
extern int cyros_bench_main(void);
extern void (*__init_array_start[])(void);
extern void (*__init_array_end[])(void);

/* Read by the runner: the exit status, and that there is one. */
volatile uint32_t bench_exit_code;
volatile uint32_t bench_done;

/* Written by the runner once it is attached and OpenOCD has settled
 * (drive_test.py). _entry waits for it. */
volatile uint32_t host_go;

/* Core 1's stack, for the rp2350_hazard3_smp target's launch: core 1 runs on
 * it from the boot ROM's handoff, and it becomes core 1's interrupt stack. */
#define CORE1_STACK_BYTES 16384
__attribute__((aligned(16))) static uint8_t core1_stack[CORE1_STACK_BYTES];

void* cyros_port_core1_stack_top(void)
{
   return core1_stack + CORE1_STACK_BYTES;
}

/* MTIME's rate, for the port's MTIME time source. */
uint32_t cyros_port_mtime_clock_hz(void)
{
   return BOARD_SYSCLK_HZ;
}

/* Moves MTIME, for a test that needs it near a boundary. The SIO takes a
 * write only while MTIME_CTRL.EN is clear. Only ever before time starts. */
void cyros_bench_mtime_set(uint64_t value)
{
   uint32_t const ctrl = REG(SIO_MTIME_CTRL);
   REG(SIO_MTIME_CTRL) = ctrl & ~MTIME_EN;
   REG(SIO_MTIME) = 0u;
   REG(SIO_MTIMEH) = (uint32_t)(value >> 32);
   REG(SIO_MTIME) = (uint32_t)value;
   REG(SIO_MTIME_CTRL) = ctrl;
}

/* bench.hpp's semihosting: never trapped here. OpenOCD's RISC-V semihosting
 * on this board writes a stale value into s0 at every call (rp2350-notes.md
 * 7b), and it supports neither elapsed-time operation, so this is its answer
 * without the trap. The tests then skip their absolute-rate checks, as they do
 * on the ARM boards. */
long cyros_bench_semihost(long op, void volatile* arg)
{
   (void)op;
   (void)arg;
   return -1;
}

static void write_decimal(uint32_t value)
{
   char digits[11];
   char* p = &digits[sizeof digits - 1];
   *p = '\0';
   do {
      *--p = (char)('0' + value % 10u);
      value /= 10u;
   } while (value != 0u);
   cyros_bench_write(p);
}

static void write_hex(uint32_t value)
{
   char s[11] = "0x00000000";
   for (int i = 0; i < 8; ++i) {
      uint32_t const n = (value >> (28 - 4 * i)) & 0xFu;
      s[2 + i] = (char)(n < 10u ? '0' + n : 'a' + n - 10u);
   }
   cyros_bench_write(s);
}

__attribute__((noreturn)) void cyros_bench_exit(uint32_t code)
{
   cyros_bench_write("EXIT: ");
   write_decimal(code);
   cyros_bench_write("\n");
   /* Bounded, so a runner that never started RTT still sees bench_done. */
   (void)board_console_drain(150000000u);
   bench_exit_code = code;
   __asm__ volatile("fence rw, rw" ::: "memory");
   bench_done = 1u;
   for (;;) {
      __asm__ volatile("wfi");
   }
}

/* ---------------------------------------------------------------------------
 * Faults before cyros owns the trap vector
 * ------------------------------------------------------------------------ */

__attribute__((noreturn, used)) void early_trap_report(uint32_t cause, uint32_t epc)
{
   /* mtval is hardwired to zero on Hazard3 (rp2350-notes.md 6a). */
   cyros_bench_write("\n*** TRAP before cyros owns mtvec: mcause ");
   write_hex(cause);
   cyros_bench_write(", mepc ");
   write_hex(epc);
   cyros_bench_write(" ***\n");
   cyros_bench_exit(3u);
}

__attribute__((naked, aligned(4), used)) void early_trap(void);
__attribute__((naked, aligned(4), used)) void early_trap(void)
{
   __asm__ volatile(
      "csrr a0, mcause         \n"
      "csrr a1, mepc           \n"
      "j    early_trap_report  \n");
}

/* ---------------------------------------------------------------------------
 * Reset
 * ------------------------------------------------------------------------ */

/* Before the go word: .bss and the console. */
__attribute__((used)) void board_early(void)
{
   for (uint32_t* target = &__bss_start__; target < &__bss_end__; ) {
      *target++ = 0u;
   }
   board_console_init();
}

__attribute__((noreturn, used)) void board_no_go(void)
{
   cyros_bench_write("\n*** the runner never said go ***\n");
   cyros_bench_exit(6u);
}

/* After it: the clock, MTIME, constructors, the test. */
__attribute__((noreturn, used)) void reset_c(void)
{
   if (!clocks_150mhz()) {
      cyros_bench_write("\n*** the 150 MHz clock did not come up ***\n");
      cyros_bench_exit(7u);
   }
   /* DBGPAUSE0 only. With DBGPAUSE1 too, MTIME stopped while OpenOCD had the
    * idle core 1 halted: 0 of 153,622 core cycles counted in one window
    * (rp2350-notes.md 7b). Core 1 runs nothing of this image. */
   REG(SIO_MTIME_CTRL) = MTIME_EN | MTIME_FULLSPEED | MTIME_DBGPAUSE0;

   for (void (**ctor)(void) = __init_array_start; ctor != __init_array_end; ++ctor) {
      (*ctor)();
   }
   cyros_bench_exit((uint32_t)cyros_bench_main());
}

/* Interrupts off, the image's stack and a trap vector that reports, then
 * board_early, then the wait for the runner's go word, then reset_c. Linked
 * first in the image (rp2350_hazard3.ld).
 *
 * The wait is here, in assembly on temporaries, because OpenOCD halts the
 * harts once while it is attached to take the boot's reset report, and its
 * resume writes a stale value into s0 (rp2350-notes.md 7b). The runner makes
 * that happen during this wait, where nothing lives in s0. About a minute at
 * the boot clock, then board_no_go. */
__attribute__((naked, section(".text.board_entry"))) void _entry(void)
{
   __asm__ volatile(
      ".option push              \n"
      ".option norelax           \n"
      "csrci mstatus, 8          \n"
      "la    sp, __stack_top     \n"
      "la    t0, early_trap      \n"
      "csrw  mtvec, t0           \n"
      "call  board_early         \n"
      "la    t0, host_go         \n"
      "li    t1, 0x60606060      \n"
      "li    t2, 400000000       \n"
      "1:                        \n"
      "lw    t3, 0(t0)           \n"
      "beq   t3, t1, 2f          \n"
      "addi  t2, t2, -1          \n"
      "bnez  t2, 1b              \n"
      "j     board_no_go         \n"
      "2:                        \n"
      "fence rw, rw              \n"
      "j     reset_c             \n"
      ".option pop               \n");
}

/* The boot ROM's IMAGE_DEF, which it looks for in the first 4 kB: a RISC-V
 * executable for the RP2350, entered at _entry with sp at __stack_top. One
 * block, linked to itself. */
__asm__(
   "   .section .embedded_block, \"a\"            \n"
   "   .p2align 2                                 \n"
   "embedded_block:                               \n"
   "   .word 0xffffded3                           \n"   /* PICOBIN_BLOCK_MARKER_START       */
   "   .byte 0x42, 0x01                           \n"   /* IMAGE_TYPE, 1 word               */
   "   .hword 0x1101                              \n"   /* EXE, CPU_RISCV, CHIP_RP2350      */
   "   .byte 0x44, 0x03, 0, 0                     \n"   /* ENTRY_POINT, 3 words             */
   "   .word _entry                               \n"
   "   .word __stack_top                          \n"
   "   .byte 0xff                                 \n"   /* LAST                             */
   "   .hword (embedded_block_end - embedded_block - 16) / 4 \n"
   "   .byte 0                                    \n"
   "   .word 0                                    \n"   /* a loop of one block              */
   "   .word 0xab123579                           \n"   /* PICOBIN_BLOCK_MARKER_END         */
   "embedded_block_end:                           \n"
   "   .previous                                  \n");
