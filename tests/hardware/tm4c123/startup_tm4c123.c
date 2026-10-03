/**
 * @file startup_tm4c123.c
 * @brief Reset path and vector table for the EK-TM4C123GXL board.
 *
 * The U575 startup's ARMv7-M sibling, and NOTHING in libcyros.a differs from
 * the mps2-an386 bench's: the port is core-level, and the core is the same
 * Cortex-M4F on both.
 *
 * The clock is the board's choice. Reset_Handler brings it to BOARD_SYSCLK_HZ
 * (board.h), 16 MHz from the crystal or 80 MHz from the PLL, and board_clock.c
 * tells cyros the same number. Then the console comes up, UART0 to the ICDI's
 * virtual COM port, and everything the image prints goes there, faults and the
 * final exit status included (console.c).
 *
 * One image serves both of run_test.sh's ways in: programmed into flash, or
 * loaded into SRAM by OpenOCD with flash untouched. The linker script is the
 * only difference, which is why Reset_Handler points VTOR at this table rather
 * than trusting that the table is at address 0.
 */

#include "board.h"

#include <stdint.h>

/* Provided by the linker script. */
extern uint32_t __etext;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __stack_top;

/* The two handlers cyros owns, from the port. */
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);

/* The test's entry point. Deliberately NOT called main: this is a freestanding
 * image with no hosted runtime, ISO C++ forbids a program from calling main at
 * all, and naming it plainly says that nothing here is the C startup contract. */
extern int cyros_bench_main(void);

/* C++ static constructors. The kernel has file-scope objects, and while most
 * are constant-initialised, nothing guarantees that for all of them and a
 * missed constructor is a silent zero rather than a link error. */
extern void (*__init_array_start[])(void);
extern void (*__init_array_end[])(void);

/**
 * @brief End the image, bench.hpp's host_exit on this board.
 *
 * There is no host to hand a status to, so the status is PRINTED, as the last
 * line the image ever writes, and run_test.sh reads it. Then the core parks. It
 * does not trap: an unserviced BKPT would become a HardFault the moment no
 * debugger is attached.
 */
__attribute__((noreturn)) void cyros_bench_exit(uint32_t code)
{
   char digits[11];
   char* p = &digits[sizeof digits - 1];
   *p = '\0';
   do {
      *--p = (char)('0' + code % 10u);
      code /= 10u;
   } while (code != 0u);
   cyros_bench_write("EXIT: ");
   cyros_bench_write(p);
   cyros_bench_write("\n");
   board_console_drain();
   for (;;) {
      /* TI's SYSCTL#04: one instruction after a WFI, as cyros_port_idle has. */
      __asm__ volatile("wfi\n nop");
   }
}

typedef union
{
   void (*handler)(void);
   uintptr_t value;
} vector_entry;

extern vector_entry const vector_table[];

__attribute__((noreturn)) void Reset_Handler(void)
{
   /* Exceptions vector through THIS table, wherever the image was linked. An
    * SRAM image is entered by the debugger with flash's own table at 0. */
   *(volatile uint32_t*)0xE000ED08u = (uint32_t)(uintptr_t)vector_table;
   __asm__ volatile("dsb\n isb" ::: "memory");

   uint32_t const* source = &__etext;
   for (uint32_t* target = &__data_start__; target < &__data_end__; ) {
      *target++ = *source++;
   }
   for (uint32_t* target = &__bss_start__; target < &__bss_end__; ) {
      *target++ = 0u;
   }

   /* Enable the configurable faults. Without these, a MemManage, BusFault or
    * UsageFault ESCALATES to HardFault, and every one of them reports as
    * exception 3 with no indication of which it was. An INVSTATE UsageFault
    * (branching to a non-Thumb address) and an MPU guard hit both land in that
    * bucket, and they want completely different investigations. */
   *(volatile uint32_t*)0xE000ED24u |= (1u << 16) | (1u << 17) | (1u << 18);

   /* The clock first, because the console's baud divisor is computed from it,
    * and both before any constructor, which may print. */
   board_clock_init();
   board_console_init();

   for (void (**ctor)(void) = __init_array_start; ctor != __init_array_end; ++ctor) {
      (*ctor)();
   }

   cyros_bench_exit((uint32_t)cyros_bench_main());
}

/**
 * @brief Every fault lands here.
 *
 * Reporting the exception number matters more than it looks. A bad context
 * switch shows up as a UsageFault or a HardFault at an address that means
 * nothing on its own, and the number is the first thing that narrows it: 3 is
 * HardFault, 4 MemManage, 5 BusFault, 6 UsageFault. On ARMv7-M a thread
 * overrunning its stack writes into the port's MPU guard region and arrives
 * as a MemManage fault with DACCVIOL set.
 */
/* Weakly aliased one by one, as on the benches and for the same reason:
 * test_cortex_m_stack_guard replaces the two handlers a stack overflow can
 * arrive at, and every other fault has to go on reporting as a fault. */
__attribute__((noreturn)) void Fault_Handler(void);

__attribute__((noreturn)) void NMI_Handler(void)         __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void HardFault_Handler(void)   __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void MemManage_Handler(void)   __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void BusFault_Handler(void)    __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void UsageFault_Handler(void)  __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void DebugMon_Handler(void)    __attribute__((weak, alias("Fault_Handler")));

__attribute__((noreturn)) void Fault_Handler(void)
{
   uint32_t ipsr;
   __asm__ volatile("mrs %0, ipsr" : "=r"(ipsr));

   cyros_bench_write("\n*** FAULT, exception number ");
   char digits[4];
   digits[0] = (char)('0' + ((ipsr / 100u) % 10u));
   digits[1] = (char)('0' + ((ipsr / 10u) % 10u));
   digits[2] = (char)('0' + (ipsr % 10u));
   digits[3] = '\0';
   cyros_bench_write(digits);
   cyros_bench_write(" ***\n");

   cyros_bench_exit(3u);
}

__attribute__((noreturn)) static void Default_Handler(void)
{
   cyros_bench_write("\n*** unexpected interrupt ***\n");
   cyros_bench_exit(4u);
}

/* The ARMv7-M vector table. Sixteen system entries, then the TM4C123's
 * device IRQs, of which the tests need none.
 *
 * A union rather than an array of function pointers because entry 0 is the
 * initial stack POINTER, not a handler, and ISO C forbids converting an object
 * pointer to a function pointer. The union states the real shape of the table
 * instead of casting past the rule. */
__attribute__((section(".vectors"), used))
vector_entry const vector_table[] = {
   { .value   = (uintptr_t)&__stack_top }, /*  0 initial MSP              */
   { .handler = Reset_Handler },           /*  1 Reset                    */
   { .handler = NMI_Handler },             /*  2 NMI                      */
   { .handler = HardFault_Handler },       /*  3 HardFault                */
   { .handler = MemManage_Handler },       /*  4 MemManage                */
   { .handler = BusFault_Handler },        /*  5 BusFault                 */
   { .handler = UsageFault_Handler },      /*  6 UsageFault               */
   { .value   = 0u },                      /*  7 reserved                 */
   { .value   = 0u },                      /*  8 reserved                 */
   { .value   = 0u },                      /*  9 reserved                 */
   { .value   = 0u },                      /* 10 reserved                 */
   { .handler = Default_Handler },         /* 11 SVCall                   */
   { .handler = DebugMon_Handler },        /* 12 DebugMonitor             */
   { .value   = 0u },                      /* 13 reserved                 */
   { .handler = PendSV_Handler },          /* 14 PendSV, cyros owns this  */
   { .handler = SysTick_Handler },         /* 15 SysTick, cyros owns this */
};
