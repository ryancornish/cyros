/**
 * @file startup_rp2040.c
 * @brief Reset path and vector table for cyros's on-target tests on the RP2040
 *        Pico's Cortex-M0+ core 0, the ARMv6-M sibling of
 *        ../rp2350/startup_rp2350_m33.c.
 *
 * Serves the rp2040 target, which keeps time on the TIMER through the MTIME
 * time source (the toolchain says so to the tests, CYROS_BENCH_MTIME). The
 * generic cortex_m0 target runs on this chip through
 * ~/cyros-claude/tools-rp2040-loop/cyros_board instead. The handlers the port
 * defines are linked over weak defaults below.
 *
 * The image runs from SRAM (rp2040.ld) and is entered at _entry by the
 * debugger after a reset halt (run_test.sh), which resets the cores alone and
 * leaves the boot ROM's MSP and a VTOR of 0. _entry sets both before any C
 * runs. Core 1 stays in the boot ROM. Then, in order:
 *
 *   1. .bss and the console, and rtt_ready, so the runner can read it.
 *   2. The runner's go word, so nothing timed starts before OpenOCD is
 *      attached, as on the Pico 2 W board.
 *   3. Every unused peripheral into reset, 125 MHz from the crystal, and the
 *      TIMER at 1 MHz (rp2040.h).
 *   4. Constructors, then cyros_bench_main.
 *
 * What ARMv6-M changes from the Pico 2 W board: no MSPLIM, no FPU, and no
 * configurable faults, so every fault is a HardFault, and its report adds the
 * stacked PC since there is no CFSR to say why. Thumb-1's B reaches 2 kB, so
 * the naked entries reach C through a register.
 */
#include "board.h"
#include "rp2040.h"

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

/* Written by the runner once it is attached (drive_test.py). */
volatile uint32_t host_go;
#define HOST_GO 0x60606060u

/* The TIMER's rate, for the port's MTIME time source. */
uint32_t cyros_port_mtime_clock_hz(void)
{
   return BOARD_TIMER_HZ;
}

/* Moves the TIMER, for a test that needs it near a boundary. The low word
 * waits in TIMELW until TIMEHW is written. Only ever before time starts. */
void cyros_bench_mtime_set(uint64_t value)
{
   REG(TIMER_TIMELW) = (uint32_t)value;
   REG(TIMER_TIMEHW) = (uint32_t)(value >> 32);
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
   /* Bounded, so a runner that never read the console still sees bench_done. */
   (void)board_console_drain(125000000u);
   bench_exit_code = code;
   __asm__ volatile("dmb" ::: "memory");
   bench_done = 1u;
   for (;;) {
      __asm__ volatile("wfi");
   }
}

/* ---------------------------------------------------------------------------
 * Reset
 * ------------------------------------------------------------------------ */

__attribute__((noreturn, used)) void reset_c(void)
{
   for (uint32_t* target = &__bss_start__; target < &__bss_end__; ) {
      *target++ = 0u;
   }
   board_console_init();

   for (uint32_t spins = 0; host_go != HOST_GO; ) {
      if (++spins == 400000000u) {
         cyros_bench_write("\n*** the runner never said go ***\n");
         cyros_bench_exit(6u);
      }
   }
   __asm__ volatile("dmb" ::: "memory");

   peripherals_quiet();
   if (!clocks_125mhz()) {
      cyros_bench_write("\n*** the 125 MHz clock did not come up ***\n");
      cyros_bench_exit(7u);
   }
   if (!board_timer_init()) {
      cyros_bench_write("\n*** the TIMER did not leave reset ***\n");
      cyros_bench_exit(7u);
   }

   for (void (**ctor)(void) = __init_array_start; ctor != __init_array_end; ++ctor) {
      (*ctor)();
   }
   cyros_bench_exit((uint32_t)cyros_bench_main());
}

/* MSP and VTOR first: a debugger start leaves the boot ROM's. */
__attribute__((naked, noreturn)) void _entry(void)
{
   __asm__ volatile(
      "cpsid i                 \n"
      "ldr   r0, =__stack_top  \n"
      "msr   msp, r0           \n"
      "ldr   r0, =vector_table \n"
      "ldr   r1, =0xE000ED08   \n"
      "str   r0, [r1]          \n"
      "dsb                     \n"
      "isb                     \n"
      "cpsie i                 \n"
      "ldr   r0, =reset_c      \n"
      "bx    r0                \n"
      ".ltorg                  \n");
}

/* ---------------------------------------------------------------------------
 * Faults and the vector table
 * ------------------------------------------------------------------------ */

/* Reached from Fault_Handler with the stacked frame, so not static. */
__attribute__((noreturn, used)) void fault_c(uint32_t const* frame);
__attribute__((noreturn)) void fault_c(uint32_t const* frame)
{
   uint32_t ipsr;
   __asm__ volatile("mrs %0, ipsr" : "=r"(ipsr));
   cyros_bench_write("\n*** FAULT on core ");
   write_decimal(REG(SIO_CPUID));
   cyros_bench_write(", exception ");
   write_decimal(ipsr & 0x3fu);
   cyros_bench_write(", stacked pc ");
   write_hex(frame[6]);
   cyros_bench_write(", lr ");
   write_hex(frame[5]);
   cyros_bench_write(" ***\n");
   cyros_bench_exit(3u);
}

/* The frame is on whichever stack the faulting code used: EXC_RETURN bit 2. */
__attribute__((naked, noreturn)) void Fault_Handler(void)
{
   __asm__ volatile(
      "movs  r0, #4       \n"
      "mov   r1, lr       \n"
      "tst   r0, r1       \n"
      "beq   1f           \n"
      "mrs   r0, psp      \n"
      "b     2f           \n"
      "1: mrs r0, msp     \n"
      "2: ldr r1, =fault_c \n"
      "bx    r1           \n"
      ".ltorg             \n");
}

/* Not declared noreturn, although it never returns, because the handlers
 * aliased to it below are not. */
void Default_Handler(void)
{
   uint32_t ipsr;
   __asm__ volatile("mrs %0, ipsr" : "=r"(ipsr));
   cyros_bench_write("\n*** unexpected exception ");
   write_decimal(ipsr & 0x3fu);
   cyros_bench_write(" ***\n");
   cyros_bench_exit(4u);
}

/* Named and weak, as on every ARM bench, so a test can replace one. */
__attribute__((noreturn)) void NMI_Handler(void)       __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void HardFault_Handler(void) __attribute__((weak, alias("Fault_Handler")));

/* The port defines these: PendSV in the core layer, the TIMER's alarm in the
 * rp2040 target. Weak, so a missing one is a report rather than a link error,
 * which the port's own tests then explain. */
void PendSV_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void TIMER_ALARM_Handler(void) __attribute__((weak, alias("Default_Handler")));

/* Device interrupts, by IRQ number, weak so an application or test defines
 * the ones it uses, the names the Pico 2 W board's tables use. 3 is cyros's. */
void isr_irq0(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq1(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq2(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq4(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq5(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq6(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq7(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq8(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq9(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq10(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq11(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq12(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq13(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq14(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq15(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq16(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq17(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq18(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq19(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq20(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq21(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq22(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq23(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq24(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq25(void) __attribute__((weak, alias("Default_Handler")));

typedef union
{
   void (*handler)(void);
   uintptr_t value;
} vector_entry;

#define DEFAULT { .handler = Default_Handler }

/* Sixteen system entries and the RP2040's 26 device IRQs. ARMv6-M reserves 4
 * to 10, 12 and 13. VTOR keeps bits 31:8, so the table is 256-aligned. */
__attribute__((section(".vectors"), used, aligned(256)))
vector_entry const vector_table[] = {
   { .value   = (uintptr_t)&__stack_top },  /*  0 initial MSP                */
   { .handler = _entry },                   /*  1 Reset                      */
   { .handler = NMI_Handler },              /*  2 NMI                        */
   { .handler = HardFault_Handler },        /*  3 HardFault                  */
   { .value = 0u }, { .value = 0u }, { .value = 0u }, { .value = 0u },
   { .value = 0u }, { .value = 0u }, { .value = 0u },
   DEFAULT,                                 /* 11 SVCall                     */
   { .value = 0u }, { .value = 0u },
   { .handler = PendSV_Handler },           /* 14 PendSV                     */
   DEFAULT,                                 /* 15 SysTick, unused here       */
   { .handler = isr_irq0 },
   { .handler = isr_irq1 },
   { .handler = isr_irq2 },
   { .handler = TIMER_ALARM_Handler },      /* IRQ 3, TIMER_IRQ_3, cyros's   */
   { .handler = isr_irq4 },
   { .handler = isr_irq5 },
   { .handler = isr_irq6 },
   { .handler = isr_irq7 },
   { .handler = isr_irq8 },
   { .handler = isr_irq9 },
   { .handler = isr_irq10 },
   { .handler = isr_irq11 },
   { .handler = isr_irq12 },
   { .handler = isr_irq13 },
   { .handler = isr_irq14 },
   { .handler = isr_irq15 },
   { .handler = isr_irq16 },
   { .handler = isr_irq17 },
   { .handler = isr_irq18 },
   { .handler = isr_irq19 },
   { .handler = isr_irq20 },
   { .handler = isr_irq21 },
   { .handler = isr_irq22 },
   { .handler = isr_irq23 },
   { .handler = isr_irq24 },
   { .handler = isr_irq25 },
};

_Static_assert(sizeof vector_table == (16u + 26u) * 4u, "16 system entries and 26 IRQs");
