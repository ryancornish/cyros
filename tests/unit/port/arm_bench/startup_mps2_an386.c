/**
 * @file startup_mps2_an386.c
 * @brief Reset path and vector table for the QEMU mps2-an386 and mps2-an385 benches.
 *
 * The an505 startup's ARMv7-M sibling, for the Cortex-M4F image on the AN386
 * and the soft-float Cortex-M3 image on the AN385. QEMU gives the two machines
 * one memory map and one SysTick clock, so one board file serves both, and it
 * touches no FP register, so it builds either way. Everything said in the
 * an505 startup about the split applies here: this is BOARD code, not port
 * code, and libcyros.a contains no vector table and no reset handler.
 *
 * Two things differ, the SysTick reference and entry 7 of the vector table,
 * which is SecureFault on ARMv8-M and reserved here.
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
 * 25 MHz, the processor clock the port selects (CLKSOURCE=1), against the
 * AN505's 20. Measured against semihosting's nanosecond reference over three
 * two-second windows: 24,999,947 to 25,001,285 Hz. Unlike the AN505, the
 * external reference (CLKSOURCE=0) is a different clock here, 999,998 Hz, with
 * SYST_CALIB reading 9999. The AN385 is the same 25 MHz:
 * test_cortex_m_systick's absolute-rate check passes there (2026-10-04).
 */
uint32_t cyros_port_systick_clock_hz(void)
{
   return 25000000u;
}

/* Semihosting, open-coded so the reset path depends on nothing. */
#define SYS_EXIT_EXTENDED 0x20
#define SYS_WRITE0        0x04
#define ADP_STOPPED_APPLICATION_EXIT 0x20026u

/* bench.hpp's two board hooks. On this bench both are semihosting, which QEMU
 * services natively and cheaply, and the exit status is what the runner reads. */
void cyros_bench_write(char const* text)
{
   register long r0 __asm__("r0") = SYS_WRITE0;
   register void const* r1 __asm__("r1") = text;
   __asm__ volatile("bkpt 0xAB" : "+r"(r0) : "r"(r1) : "memory");
}

__attribute__((noreturn)) void cyros_bench_exit(uint32_t code)
{
   volatile uint32_t block[2] = { ADP_STOPPED_APPLICATION_EXIT, code };
   register long r0 __asm__("r0") = SYS_EXIT_EXTENDED;
   register volatile uint32_t* r1 __asm__("r1") = block;
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
    * (branching to a non-Thumb address) and an MPU guard hit both land in that
    * bucket, and they want completely different investigations. */
   *(volatile uint32_t*)0xE000ED24u |= (1u << 16) | (1u << 17) | (1u << 18);

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
/* Weakly aliased one by one, as on the an505 bench and for the same reason:
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

/* The ARMv7-M vector table. Sixteen system entries, then device IRQs, of which
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
