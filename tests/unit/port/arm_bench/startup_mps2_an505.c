/**
 * @file startup_mps2_an505.c
 * @brief Reset path and vector table for the QEMU mps2-an505 bench.
 *
 * THIS IS NOT PART OF THE PORT, and the split is deliberate. cyros's port is
 * core-level: PendSV, SysTick, NVIC, BASEPRI, PRIMASK, PSPLIM. Startup is
 * BOARD-level: where flash and RAM are, how .data gets copied, what sits in the
 * vector table. Keeping them apart is what lets the same port binary serve both
 * the bench and an STM32U575, whose startup differs completely (clock tree,
 * flash latency, a full device vector table) while its core does not.
 *
 * It follows that libcyros.a contains no vector table and no reset handler. An
 * application supplies those, exactly as it would with any other RTOS, and only
 * has to route two entries at cyros.
 */

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
 * @brief What drives SysTick on this board.
 *
 * The port declares this and deliberately supplies no default, so every image
 * has to say. MEASURED 2026-09-21 against semihosting's nanosecond reference:
 * 20,000,051 Hz with CLKSOURCE=1 and 19,999,929 with CLKSOURCE=0, both within
 * 50 ppm of exactly 20 MHz.
 *
 * A literal is correct here because nothing on the bench ever changes it.
 */
uint32_t cyros_port_systick_clock_hz(void)
{
   return 20000000u;
}

/* Semihosting, open-coded so the reset path depends on nothing. */
#define SYS_EXIT_EXTENDED 0x20
#define SYS_WRITE0        0x04
#define ADP_STOPPED_APPLICATION_EXIT 0x20026u

static void bench_write0(char const* text)
{
   register long r0 __asm__("r0") = SYS_WRITE0;
   register void const* r1 __asm__("r1") = text;
   __asm__ volatile("bkpt 0xAB" : "+r"(r0) : "r"(r1) : "memory");
}

__attribute__((noreturn)) static void bench_exit(unsigned code)
{
   volatile unsigned block[2] = { ADP_STOPPED_APPLICATION_EXIT, code };
   register long r0 __asm__("r0") = SYS_EXIT_EXTENDED;
   register volatile unsigned* r1 __asm__("r1") = block;
   __asm__ volatile("bkpt 0xAB" : "+r"(r0) : "r"(r1) : "memory");
   __builtin_unreachable();
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

   for (void (**ctor)(void) = __init_array_start; ctor != __init_array_end; ++ctor) {
      (*ctor)();
   }

   bench_exit((unsigned)cyros_bench_main());
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

   bench_write0("\n*** FAULT, exception number ");
   char digits[4];
   digits[0] = (char)('0' + ((ipsr / 100u) % 10u));
   digits[1] = (char)('0' + ((ipsr / 10u) % 10u));
   digits[2] = (char)('0' + (ipsr % 10u));
   digits[3] = '\0';
   bench_write0(digits);
   bench_write0(" ***\n");

   bench_exit(3u);
}

__attribute__((noreturn)) static void Default_Handler(void)
{
   bench_write0("\n*** unexpected interrupt ***\n");
   bench_exit(4u);
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
