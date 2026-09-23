/**
 * @file startup_mps2_an521.c
 * @brief Reset path and vector tables for the QEMU mps2-an521 dual-M33 bench.
 *
 * The an505 startup's dual-core sibling. Everything said there about the split
 * applies here: this is BOARD code, not port code, and libcyros.a contains no
 * vector table and no reset handler.
 *
 * What is genuinely different is that there are TWO vector tables.
 *
 * CPU1 comes out of reset held. cyros releases it by pointing INITSVTOR1 at a
 * table and clearing CPUWAIT, at which point CPU1 loads its initial MSP from
 * word 0 of that table and branches to word 1. So the two tables must differ
 * in exactly those two words and may be identical everywhere else: the fault
 * handlers, PendSV and the doorbell are the same code on both cores.
 *
 * THE TWO CORES MAY NOT SHARE ONE TABLE, and the reason is word 0. A shared
 * table gives both cores the same initial MSP, so both push exception frames
 * to the same main stack and the first interrupt on either core corrupts the
 * other. The rest of the table is shared through a macro rather than by
 * pointing both cores at the same array.
 *
 * SysTick appears in both tables even though only core 0 ever enables it. An
 * entry costs four bytes and a core that somehow took an unexpected SysTick
 * would otherwise fetch a null handler rather than report.
 */

#include <stdint.h>

extern uint32_t __etext;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __stack_top;

/* The three handlers cyros owns on this target. MHU_Handler is the inter-core
 * doorbell, and is what makes this bench dual-core rather than two independent
 * kernels sharing an image. */
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);
extern void MHU_Handler(void);

/* Where a released secondary core joins the kernel. */
extern void cyros_port_secondary_core_entry(void);

extern int cyros_bench_main(void);

extern void (*__init_array_start[])(void);
extern void (*__init_array_end[])(void);

/**
 * @brief What drives SysTick on this board.
 *
 * The AN521 runs the same 20 MHz reference the AN505 does, measured there
 * against semihosting's nanosecond clock.
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

typedef union
{
   void (*handler)(void);
   uintptr_t value;
} vector_entry;

__attribute__((noreturn)) void Fault_Handler(void);
__attribute__((noreturn)) void Reset_Handler(void);
__attribute__((noreturn)) static void CPU1_Reset_Handler(void);

__attribute__((noreturn)) void NMI_Handler(void)         __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void HardFault_Handler(void)   __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void MemManage_Handler(void)   __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void BusFault_Handler(void)    __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void UsageFault_Handler(void)  __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void SecureFault_Handler(void) __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void DebugMon_Handler(void)    __attribute__((weak, alias("Fault_Handler")));

__attribute__((noreturn)) static void Default_Handler(void);

/* CPU1's main stack. The linker script places the primary core's at the top of
 * RAM, and a secondary core needs its own object. It is only ever the boot
 * path, the exception handlers and the scheduler that run on it, the same as
 * for core 0, so it is sized like a handler stack rather than a thread one. */
static uint32_t cpu1_main_stack[512] __attribute__((aligned(8)));

/* Everything after the first two words. Identical on both cores. */
#define SHARED_VECTORS                                       \
   { .handler = NMI_Handler },        /*  2 NMI           */ \
   { .handler = HardFault_Handler },  /*  3 HardFault     */ \
   { .handler = MemManage_Handler },  /*  4 MemManage     */ \
   { .handler = BusFault_Handler },   /*  5 BusFault      */ \
   { .handler = UsageFault_Handler }, /*  6 UsageFault    */ \
   { .handler = SecureFault_Handler },/*  7 SecureFault   */ \
   { .value   = 0u },                 /*  8 reserved      */ \
   { .value   = 0u },                 /*  9 reserved      */ \
   { .value   = 0u },                 /* 10 reserved      */ \
   { .handler = Default_Handler },    /* 11 SVCall        */ \
   { .handler = DebugMon_Handler },   /* 12 DebugMonitor  */ \
   { .value   = 0u },                 /* 13 reserved      */ \
   { .handler = PendSV_Handler },     /* 14 PendSV        */ \
   { .handler = SysTick_Handler },    /* 15 SysTick       */ \
   /* Device IRQs. Only 6 is used, and the rest report rather than         */ \
   /* fetching whatever follows the table.                                 */ \
   { .handler = Default_Handler },    /* IRQ 0            */ \
   { .handler = Default_Handler },    /* IRQ 1            */ \
   { .handler = Default_Handler },    /* IRQ 2            */ \
   { .handler = Default_Handler },    /* IRQ 3            */ \
   { .handler = Default_Handler },    /* IRQ 4            */ \
   { .handler = Default_Handler },    /* IRQ 5            */ \
   { .handler = MHU_Handler },        /* IRQ 6, MHU0      */ \
   { .handler = Default_Handler }     /* IRQ 7            */

/* ARMv8-M requires a vector table aligned to its own size rounded up to a
 * power of two, minimum 128 bytes. 24 entries is 96 bytes, so 128 would do,
 * but 512 leaves room to grow the device entries without a silent
 * misalignment. */
__attribute__((section(".vectors"), used))
vector_entry const vector_table[] = {
   { .value   = (uintptr_t)&__stack_top },
   { .handler = Reset_Handler },
   SHARED_VECTORS
};

__attribute__((used, aligned(512)))
static vector_entry const cpu1_vector_table[] = {
   { .value   = (uintptr_t)(cpu1_main_stack + 512) },
   { .handler = CPU1_Reset_Handler },
   SHARED_VECTORS
};

/**
 * @brief Where cyros finds CPU1's vector table.
 *
 * Declared by the port, defined here, exactly like cyros_port_systick_clock_hz
 * above: the port states what it needs from the board and the board answers.
 */
void const* cyros_port_cpu1_vector_table(void)
{
   return cpu1_vector_table;
}

/**
 * @brief CPU1 out of reset.
 *
 * Deliberately does almost nothing. .data and .bss were copied and zeroed by
 * core 0 long before this runs, and doing either again would wipe state the
 * kernel is already using. The configurable faults are enabled per core
 * because SHCSR is core-private, and then this hands over to the port.
 */
__attribute__((noreturn)) static void CPU1_Reset_Handler(void)
{
   *(volatile uint32_t*)0xE000ED24u |= (1u << 16) | (1u << 17) | (1u << 18);

   cyros_port_secondary_core_entry();
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

   *(volatile uint32_t*)0xE000ED24u |= (1u << 16) | (1u << 17) | (1u << 18);

   for (void (**ctor)(void) = __init_array_start; ctor != __init_array_end; ++ctor) {
      (*ctor)();
   }

   bench_exit((unsigned)cyros_bench_main());
}

/**
 * @brief Every fault lands here, on either core.
 *
 * Reports the core as well as the exception number, because on two cores the
 * number alone does not say where to look and the two cores fail for different
 * reasons. Reads the SSE-200 CPU identity block directly rather than calling
 * into the port: a fault handler should depend on as little as possible.
 */
__attribute__((noreturn)) void Fault_Handler(void)
{
   uint32_t ipsr;
   __asm__ volatile("mrs %0, ipsr" : "=r"(ipsr));
   uint32_t const core = *(volatile uint32_t*)0x5001F000u;

   bench_write0("\n*** FAULT on core ");
   char digit[2];
   digit[0] = (char)('0' + (core % 10u));
   digit[1] = '\0';
   bench_write0(digit);

   bench_write0(", exception number ");
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
