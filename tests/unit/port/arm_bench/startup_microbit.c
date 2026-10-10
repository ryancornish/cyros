/**
 * @file startup_microbit.c
 * @brief Reset path and vector table for QEMU's microbit, the ARMv6-M bench.
 *
 * The mps2 startups' Baseline sibling, for a Cortex-M0. BOARD code, not port
 * code: libcyros.a has no vector table and no reset handler. What differs
 * from the Mainline benches:
 *
 *   - no FPU to turn on, so the reset handler is plain C,
 *   - no configurable faults (MemManage, BusFault, UsageFault): every fault
 *     is a HardFault on ARMv6-M, so there is no SHCSR enable to set,
 *   - .data is copied out of flash, which the core cannot write,
 *   - the nRF51's 32 device interrupts, all unexpected here.
 */

#include <stdint.h>

/* Provided by the linker script. */
extern uint32_t const __data_load__;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __stack_top;

/* The two handlers cyros owns, from the port. */
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);

extern int cyros_bench_main(void);

extern void (*__init_array_start[])(void);
extern void (*__init_array_end[])(void);

/**
 * @brief What drives SysTick on this board.
 *
 * 16 MHz, the nRF51's HCLK, on either clock source (arm-port-notes.md 2,
 * probed 2026-10-04). test_target_tick's absolute-rate check holds it to
 * semihosting's clock.
 */
uint32_t cyros_port_systick_clock_hz(void)
{
   return 16000000u;
}

#define SYS_EXIT_EXTENDED 0x20
#define SYS_WRITE0        0x04
#define ADP_STOPPED_APPLICATION_EXIT 0x20026u

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
   uint32_t const* source = &__data_load__;
   for (uint32_t* target = &__data_start__; target < &__data_end__; ) {
      *target++ = *source++;
   }
   for (uint32_t* target = &__bss_start__; target < &__bss_end__; ) {
      *target++ = 0u;
   }
   for (void (**ctor)(void) = __init_array_start; ctor != __init_array_end; ++ctor) {
      (*ctor)();
   }
   cyros_bench_exit((uint32_t)cyros_bench_main());
}

static void write_decimal3(uint32_t value)
{
   char digits[4];
   digits[0] = (char)('0' + ((value / 100u) % 10u));
   digits[1] = (char)('0' + ((value / 10u) % 10u));
   digits[2] = (char)('0' + (value % 10u));
   digits[3] = '\0';
   cyros_bench_write(digits);
}

/* Weak, so a test can take one. */
__attribute__((noreturn)) void Fault_Handler(void);
__attribute__((noreturn)) void NMI_Handler(void)       __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void HardFault_Handler(void) __attribute__((weak, alias("Fault_Handler")));

__attribute__((noreturn)) void Fault_Handler(void)
{
   uint32_t ipsr;
   __asm__ volatile("mrs %0, ipsr" : "=r"(ipsr));
   cyros_bench_write("\n*** FAULT, exception number ");
   write_decimal3(ipsr & 0x3fu);
   cyros_bench_write(" ***\n");
   cyros_bench_exit(3u);
}

__attribute__((noreturn)) static void Default_Handler(void)
{
   uint32_t ipsr;
   __asm__ volatile("mrs %0, ipsr" : "=r"(ipsr));
   cyros_bench_write("\n*** unexpected exception ");
   write_decimal3(ipsr & 0x3fu);
   cyros_bench_write(" ***\n");
   cyros_bench_exit(4u);
}

typedef union
{
   void (*handler)(void);
   uintptr_t value;
} vector_entry;

#define DEFAULT  { .handler = Default_Handler }
#define DEFAULT4 DEFAULT, DEFAULT, DEFAULT, DEFAULT

/* Sixteen system entries and the nRF51's 32 device interrupts. ARMv6-M
 * reserves 4 to 10, 12 and 13. */
__attribute__((section(".vectors"), used))
vector_entry const vector_table[] = {
   { .value   = (uintptr_t)&__stack_top },  /*  0 initial MSP */
   { .handler = Reset_Handler },            /*  1 Reset       */
   { .handler = NMI_Handler },              /*  2 NMI         */
   { .handler = HardFault_Handler },        /*  3 HardFault   */
   { .value = 0u }, { .value = 0u }, { .value = 0u }, { .value = 0u },
   { .value = 0u }, { .value = 0u }, { .value = 0u },
   DEFAULT,                                 /* 11 SVCall      */
   { .value = 0u }, { .value = 0u },
   { .handler = PendSV_Handler },           /* 14 PendSV      */
   { .handler = SysTick_Handler },          /* 15 SysTick     */
   DEFAULT4, DEFAULT4, DEFAULT4, DEFAULT4,
   DEFAULT4, DEFAULT4, DEFAULT4, DEFAULT4,
};

_Static_assert(sizeof vector_table == (16u + 32u) * 4u, "16 system entries and 32 IRQs");
