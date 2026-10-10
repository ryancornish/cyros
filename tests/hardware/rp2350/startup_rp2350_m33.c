/**
 * @file startup_rp2350_m33.c
 * @brief Reset path and vector table for cyros's on-target tests on the Pico 2
 *        W's Cortex-M33 cores, the Arm sibling of startup_rp2350_hazard3.c.
 *
 * Serves the rp2350_m33 and rp2350_m33_smp targets, which keep time on MTIME
 * (the toolchain says so to the tests, CYROS_BENCH_MTIME). The generic
 * cortex_m target runs on this chip through ~/cyros-claude/tools-rp2350-probe
 * instead. The handlers each port defines are linked over weak defaults
 * below.
 *
 * The image runs from SRAM (rp2350_m33.ld) or in place from flash
 * (rp2350_m33_flash.ld, the toolchain arm-none-eabi-rp2350-flash.toml). It is
 * entered at _entry either by the boot ROM, through the IMAGE_DEF block below
 * (`picotool load -x`, the runner's default, which writes a flash image to
 * flash first), or by an SRAM image's debugger after a reset halt
 * (RP2350_BOOT=swd). _entry
 * sets MSP, MSPLIM and VTOR before any C runs, because a debugger start leaves
 * the boot ROM's (rp2350-notes.md 2b). Core 1 stays in the boot ROM's holding
 * pen until the rp2350_m33_smp target launches it, with this same vector table
 * and core1_stack below.
 *
 * Then, in order:
 *
 *   1. a flash image's .data, .bss and the console, and rtt_ready, so the
 *      runner can read it;
 *   2. the runner's go word, so nothing timed starts before OpenOCD is
 *      attached, as on the Hazard3 board;
 *   3. 150 MHz from the crystal, the flash clock slowed first (rp2350.h),
 *      and MTIME counting it (FULLSPEED);
 *   4. core 1's RCP salt, seeded if a debugger start skipped the boot ROM;
 *   5. constructors, then cyros_bench_main.
 */
#include "board.h"
#include "rp2350.h"

#include <stdint.h>

extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t const __data_load__;
extern uint32_t __stack_top;
extern int cyros_bench_main(void);
extern void (*__init_array_start[])(void);
extern void (*__init_array_end[])(void);

#define SCB_VTOR   0xE000ED08u
#define SCB_SHCSR  0xE000ED24u
#define SCB_CFSR   0xE000ED28u
#define SCB_CPACR  0xE000ED88u

/* Read by the runner: the exit status, and that there is one. */
volatile uint32_t bench_exit_code;
volatile uint32_t bench_done;

/* Written by the runner once it is attached (drive_test.py). */
volatile uint32_t host_go;
#define HOST_GO 0x60606060u

/* Core 1's stack, for the rp2350_m33_smp target's launch. It stays core 1's
 * MSP, its handler stack. */
#define CORE1_STACK_BYTES 16384
__attribute__((aligned(16))) static uint8_t core1_stack[CORE1_STACK_BYTES];

void* cyros_port_core1_stack_top(void)
{
   return core1_stack + CORE1_STACK_BYTES;
}

/* MTIME's rate, for the port's MTIME time source: FULLSPEED, the core clock. */
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
   (void)board_console_drain(150000000u);
   bench_exit_code = code;
   __asm__ volatile("dmb" ::: "memory");
   bench_done = 1u;
   for (;;) {
      __asm__ volatile("wfi");
   }
}

/* ---------------------------------------------------------------------------
 * Core 1's RCP salt
 * ------------------------------------------------------------------------ */

/* Core 1's boot ROM pen sleeps until its RCP salt is valid, and only core 0's
 * boot ROM seeds it, so after a debugger start the launch would time out with
 * core 1 asleep (rp2350-notes.md 2c). This is the boot ROM's own check and
 * seed (varm_boot_path.c step 4), and does nothing after a real boot. */
static void seed_rcp_if_unseeded(void)
{
   REG(SCB_CPACR) |= 3u << 14;   /* CP7, the RCP */
   __asm__ volatile("dsb\n\tisb" ::: "memory");
   uint32_t apsr;
   __asm__ volatile("mrc p7, #1, APSR_nzcv, c0, c0, #0\n\tmrs %0, apsr" : "=r"(apsr) :: "cc");
   if ((apsr >> 31) != 0u) {
      return;
   }
   __asm__ volatile("mcrr p7, #8, %0, %1, c0\n\t"
                    "mcrr p7, #8, %2, %3, c1\n\t"
                    "sev"
                    :: "r"(0x9e3779b9u), "r"(0x7f4a7c15u), "r"(0xf39cc060u), "r"(0x5ced8a2bu));
}

/* ---------------------------------------------------------------------------
 * Reset
 * ------------------------------------------------------------------------ */

/* Before the go word: .data from flash in a flash image (its load and run
 * addresses are one in an SRAM image), .bss, and the console. */
__attribute__((used)) void board_early(void)
{
   uint32_t const* source = &__data_load__;
   if (source != &__data_start__) {
      for (uint32_t* target = &__data_start__; target < &__data_end__; ) {
         *target++ = *source++;
      }
   }
   for (uint32_t* target = &__bss_start__; target < &__bss_end__; ) {
      *target++ = 0u;
   }
   board_console_init();
}

__attribute__((noreturn, used)) void reset_c(void)
{
   board_early();
   for (uint32_t spins = 0; host_go != HOST_GO; ) {
      if (++spins == 400000000u) {
         cyros_bench_write("\n*** the runner never said go ***\n");
         cyros_bench_exit(6u);
      }
   }
   __asm__ volatile("dmb" ::: "memory");

   /* MemManage, BusFault and UsageFault as themselves, not as HardFault. */
   REG(SCB_SHCSR) |= (1u << 16) | (1u << 17) | (1u << 18);

   if (!clocks_150mhz()) {
      cyros_bench_write("\n*** the 150 MHz clock did not come up ***\n");
      cyros_bench_exit(7u);
   }
   /* DBGPAUSE0 only, as on the Hazard3 board: a debug halt of core 0 stops
    * time with it, and one of core 1 does not. */
   REG(SIO_MTIME_CTRL) = MTIME_EN | MTIME_FULLSPEED | MTIME_DBGPAUSE0;

   seed_rcp_if_unseeded();

   for (void (**ctor)(void) = __init_array_start; ctor != __init_array_end; ++ctor) {
      (*ctor)();
   }
   cyros_bench_exit((uint32_t)cyros_bench_main());
}

/* The FPU on before any compiled code runs, when this image is hard float.
 * GCC at -O2 moves 64-bit values through d registers, in the kernel and the
 * port as much as here, and the port enables the FPU only in cyros_port_init,
 * so a test that drives the time source before the kernel faulted NOCP at -O2
 * while passing at -Og. Enabling it is the reset path's job on every hard-float
 * Cortex-M image (CMSIS's SystemInit does it), the port's enable then finds it
 * done. */
#if defined(__ARM_FP)
#  define ENABLE_FPU                     \
      "ldr   r0, =0xE000ED88       \n"  \
      "ldr   r1, [r0]              \n"  \
      "orr   r1, r1, #(0xF << 20)  \n"  \
      "str   r1, [r0]              \n"
#else
#  define ENABLE_FPU ""
#endif

/* MSP, MSPLIM and VTOR first: a debugger start leaves the boot ROM's. Then
 * the FPU. */
__attribute__((naked, noreturn)) void _entry(void)
{
   __asm__ volatile(
      "cpsid i              \n"
      "movs  r0, #0         \n"
      "msr   msplim, r0     \n"
      "ldr   r0, =__stack_top \n"
      "msr   msp, r0        \n"
      "ldr   r0, =vector_table \n"
      "ldr   r1, =0xE000ED08 \n"
      "str   r0, [r1]       \n"
      ENABLE_FPU
      "dsb                  \n"
      "isb                  \n"
      "cpsie i              \n"
      "b     reset_c        \n"
      ".ltorg               \n");
}

/* ---------------------------------------------------------------------------
 * Faults and the vector table
 * ------------------------------------------------------------------------ */

__attribute__((noreturn)) void Fault_Handler(void)
{
   uint32_t ipsr;
   __asm__ volatile("mrs %0, ipsr" : "=r"(ipsr));
   cyros_bench_write("\n*** FAULT on core ");
   write_decimal(REG(0xd0000000u));
   cyros_bench_write(", exception ");
   write_decimal(ipsr & 0x1ffu);
   cyros_bench_write(", CFSR ");
   write_hex(REG(SCB_CFSR));
   cyros_bench_write(" ***\n");
   cyros_bench_exit(3u);
}

/* Not declared noreturn, although it never returns, because the handlers
 * aliased to it below are not. */
void Default_Handler(void)
{
   uint32_t ipsr;
   __asm__ volatile("mrs %0, ipsr" : "=r"(ipsr));
   cyros_bench_write("\n*** unexpected exception ");
   write_decimal(ipsr & 0x1ffu);
   cyros_bench_write(" ***\n");
   cyros_bench_exit(4u);
}

/* Named and weak, as on every ARM bench, so a test can replace one:
 * test_cortex_m_stack_guard takes UsageFault and MemManage. */
__attribute__((noreturn)) void NMI_Handler(void)         __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void HardFault_Handler(void)   __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void MemManage_Handler(void)   __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void BusFault_Handler(void)    __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void UsageFault_Handler(void)  __attribute__((weak, alias("Fault_Handler")));
__attribute__((noreturn)) void SecureFault_Handler(void) __attribute__((weak, alias("Fault_Handler")));

/* The port defines these: PendSV in the core layer, the SIO doorbell and
 * MTIMECMP in the rp2350_m33 targets. Weak, so a missing one is a report
 * rather than a link error, which the port's own tests then explain. */
void PendSV_Handler(void)       __attribute__((weak, alias("Default_Handler")));
void SIO_BELL_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void SIO_MTIMECMP_Handler(void) __attribute__((weak, alias("Default_Handler")));

/* Device interrupts, by IRQ number, weak so an application or test defines
 * the ones it uses. The same names the Hazard3 board's table uses, so one
 * test serves both ISAs. 26 and 29 are cyros's own. */
void isr_irq0(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq1(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq2(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq3(void) __attribute__((weak, alias("Default_Handler")));
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
void isr_irq27(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq28(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq30(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq31(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq32(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq33(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq34(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq35(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq36(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq37(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq38(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq39(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq40(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq41(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq42(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq43(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq44(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq45(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq46(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq47(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq48(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq49(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq50(void) __attribute__((weak, alias("Default_Handler")));
void isr_irq51(void) __attribute__((weak, alias("Default_Handler")));

typedef union
{
   void (*handler)(void);
   uintptr_t value;
} vector_entry;

#define DEFAULT { .handler = Default_Handler }

/* Sixteen system entries and the RP2350's 52 device IRQs, aligned to the
 * table's size rounded up to a power of two, which VTOR requires. Both cores
 * use it. The boot ROM enters through words 0 and 1 (the IMAGE_DEF below). */
__attribute__((section(".vectors"), used, aligned(512)))
vector_entry const vector_table[] = {
   { .value   = (uintptr_t)&__stack_top },  /*  0 initial MSP                */
   { .handler = _entry },                   /*  1 Reset                      */
   { .handler = NMI_Handler },              /*  2 NMI                        */
   { .handler = HardFault_Handler },        /*  3 HardFault                  */
   { .handler = MemManage_Handler },        /*  4 MemManage                  */
   { .handler = BusFault_Handler },         /*  5 BusFault                   */
   { .handler = UsageFault_Handler },       /*  6 UsageFault                 */
   { .handler = SecureFault_Handler },      /*  7 SecureFault                */
   { .value = 0u },                         /*  8 reserved                   */
   { .value = 0u },                         /*  9 reserved                   */
   { .value = 0u },                         /* 10 reserved                   */
   DEFAULT,                                 /* 11 SVCall                     */
   DEFAULT,                                 /* 12 DebugMonitor               */
   { .value = 0u },                         /* 13 reserved                   */
   { .handler = PendSV_Handler },           /* 14 PendSV                     */
   DEFAULT,                                 /* 15 SysTick, unused here       */
   { .handler = isr_irq0 },
   { .handler = isr_irq1 },
   { .handler = isr_irq2 },
   { .handler = isr_irq3 },
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
   { .handler = SIO_BELL_Handler },        /* IRQ 26, SIO doorbell, cyros's */
   { .handler = isr_irq27 },
   { .handler = isr_irq28 },
   { .handler = SIO_MTIMECMP_Handler },    /* IRQ 29, MTIMECMP, cyros's     */
   { .handler = isr_irq30 },
   { .handler = isr_irq31 },
   { .handler = isr_irq32 },
   { .handler = isr_irq33 },
   { .handler = isr_irq34 },
   { .handler = isr_irq35 },
   { .handler = isr_irq36 },
   { .handler = isr_irq37 },
   { .handler = isr_irq38 },
   { .handler = isr_irq39 },
   { .handler = isr_irq40 },
   { .handler = isr_irq41 },
   { .handler = isr_irq42 },
   { .handler = isr_irq43 },
   { .handler = isr_irq44 },
   { .handler = isr_irq45 },
   { .handler = isr_irq46 },
   { .handler = isr_irq47 },
   { .handler = isr_irq48 },
   { .handler = isr_irq49 },
   { .handler = isr_irq50 },
   { .handler = isr_irq51 },
};

_Static_assert(sizeof vector_table == (16u + 52u) * 4u, "16 system entries and 52 IRQs");

/* What the rp2350_m33_smp target launches core 1 with. */
void const* cyros_port_core1_vector_table(void)
{
   return vector_table;
}

/* The boot ROM's IMAGE_DEF, which it looks for in the first 4 kB: a Secure
 * Arm executable for the RP2350, entered through vector_table's first two
 * words. One block, linked to itself. The SDK's embedded block for a no_flash
 * binary. */
__asm__(
   "   .section .embedded_block, \"a\"            \n"
   "   .p2align 2                                 \n"
   "embedded_block:                               \n"
   "   .word 0xffffded3                           \n"   /* PICOBIN_BLOCK_MARKER_START          */
   "   .byte 0x42, 0x01                           \n"   /* IMAGE_TYPE, 1 word                  */
   "   .hword 0x1021                              \n"   /* EXE, SECURITY_S, CPU_ARM, RP2350    */
   "   .byte 0x03, 0x02                           \n"   /* VECTOR_TABLE, 2 words               */
   "   .hword 0                                   \n"
   "   .word vector_table                         \n"
   "   .byte 0xff                                 \n"   /* LAST                                */
   "   .hword (embedded_block_end - embedded_block - 16) / 4 \n"
   "   .byte 0                                    \n"
   "   .word 0                                    \n"   /* a loop of one block                 */
   "   .word 0xab123579                           \n"   /* PICOBIN_BLOCK_MARKER_END            */
   "embedded_block_end:                           \n"
   "   .previous                                  \n");
