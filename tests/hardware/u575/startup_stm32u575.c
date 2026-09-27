/**
 * @file startup_stm32u575.c
 * @brief Reset path and vector table for the NUCLEO-U575ZI-Q board.
 *
 * THIS IS NOT PART OF THE PORT, and this file is the proof of that claim.
 * It is the bench's startup with the board-specific parts changed and NOTHING
 * in libcyros.a touched: the port is core-level (PendSV, SysTick, NVIC,
 * BASEPRI, PRIMASK, PSPLIM), and all of that is identical on the MPS2 model
 * and on real U575 silicon.
 *
 * The clock is the board's choice, not the port's. Reset_Handler brings it to
 * BOARD_SYSCLK_HZ (board.h), either the 4 MHz reset clock or 160 MHz from
 * PLL1, and board_clock.c tells cyros the same number through
 * cyros_port_systick_clock_hz. Then the console comes up, USART1 to the
 * ST-LINK's virtual COM port, and everything the image prints goes there,
 * faults and the final exit status included (console.c).
 *
 * It follows that libcyros.a contains no vector table and no reset handler. An
 * application supplies those, exactly as it would with any other RTOS, and only
 * has to route two entries at cyros.
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
      __asm__ volatile("wfi");
   }
}

__attribute__((noreturn)) void Reset_Handler(void)
{
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
    * (branching to a non-Thumb address) and a PSPLIM stack overflow both land
    * in that bucket, and they want completely different investigations. */
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
 * HardFault, 4 MemManage, 5 BusFault, 6 UsageFault. On ARMv8-M a PSPLIM
 * violation arrives as UsageFault with STKOF set, which is the stack-overflow
 * signal this bench exists to be able to see.
 */
/* Each fault gets its own weakly-aliased symbol so a test can replace ONE of
 * them and leave the rest reporting. test_cortex_m33_psplim overrides
 * UsageFault_Handler, because a PSPLIM stack overflow arrives there and the
 * whole point of that test is that the fault HAPPENS. Without the split it
 * would have to replace the reporting for every fault at once, and then a
 * genuine BusFault during the test would look like success. */
__attribute__((noreturn)) void Fault_Handler(void);

__attribute__((noreturn)) void NMI_Handler(void)         __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void HardFault_Handler(void)   __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void MemManage_Handler(void)   __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void BusFault_Handler(void)    __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void UsageFault_Handler(void)  __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void SecureFault_Handler(void) __attribute__((weak, alias("Fault_Handler")));
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

/* The ARMv8-M vector table. Sixteen system entries, then device IRQs, of which
 * the bench needs none.
 *
 * A union rather than an array of function pointers because entry 0 is the
 * initial stack POINTER, not a handler, and ISO C forbids converting an object
 * pointer to a function pointer. The union states the real shape of the table
 * instead of casting past the rule. */
typedef union
{
   void (*handler)(void);
   uintptr_t value;
} vector_entry;

__attribute__((section(".vectors"), used))
vector_entry const vector_table[] = {
   { .value   = (uintptr_t)&__stack_top }, /*  0 initial MSP              */
   { .handler = Reset_Handler },           /*  1 Reset                    */
   { .handler = NMI_Handler },             /*  2 NMI                      */
   { .handler = HardFault_Handler },       /*  3 HardFault                */
   { .handler = MemManage_Handler },       /*  4 MemManage                */
   { .handler = BusFault_Handler },        /*  5 BusFault                 */
   { .handler = UsageFault_Handler },      /*  6 UsageFault               */
   { .handler = SecureFault_Handler },     /*  7 SecureFault              */
   { .value   = 0u },                      /*  8 reserved                 */
   { .value   = 0u },                      /*  9 reserved                 */
   { .value   = 0u },                      /* 10 reserved                 */
   { .handler = Default_Handler },         /* 11 SVCall                   */
   { .handler = DebugMon_Handler },        /* 12 DebugMonitor             */
   { .value   = 0u },                      /* 13 reserved                 */
   { .handler = PendSV_Handler },          /* 14 PendSV, cyros owns this  */
   { .handler = SysTick_Handler },         /* 15 SysTick, cyros owns this */
};
