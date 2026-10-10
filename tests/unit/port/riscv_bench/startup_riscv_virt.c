/**
 * @file startup_riscv_virt.c
 * @brief Reset path and console for cyros's on-target tests on QEMU's RISC-V
 *        `virt` machine, the RISC-V sibling of ../arm_bench/startup_mps2_an505.c.
 *
 * `-bios none` starts EVERY hart at 0x80000000 with nothing set up. Hart 0 runs
 * the image. Every other hart parks on a stack of its own, interrupts masked
 * and mie.MSIE set, in WFI until its soft IRQ is raised. That is the port's
 * release (riscv_virt_smp's start_cores), and the hart then joins the kernel
 * through cyros_port_secondary_core_entry. A single-hart image has no such
 * function and its harts beyond 0, if QEMU starts any, stay parked for good.
 *
 * The park touches nothing in memory, because hart 0 is zeroing .bss under
 * it, the secondary stacks included.
 *
 * Until cyros_port_init installs the port's trap vector, a trap lands in
 * early_trap below, which reports it and exits. Output and the exit status are
 * semihosting, which QEMU services when run with -semihosting-config.
 *
 * The C library and C++ runtime functions the bridge toolchain lacks are in
 * runtime_stubs.c, which every RISC-V board links.
 */

#include <stdint.h>

extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern int cyros_bench_main(void);
extern void (*__init_array_start[])(void);
extern void (*__init_array_end[])(void);

/* MTIME's rate on QEMU's virt: its device tree's timebase-frequency. */
uint32_t cyros_port_mtime_clock_hz(void)
{
   return 10000000u;
}

/* Moves MTIME, for a test that needs it near a boundary. QEMU's CLINT takes a
 * write to MTIME (the RP2350's SIO MTIME does too). Low word to zero first, so
 * no carry lands between the two halves. Only ever forward, before time starts:
 * a jump back would break every clock built on it. */
void cyros_bench_mtime_set(uint64_t value)
{
   uint32_t volatile* const mtime = (uint32_t volatile*)0x0200BFF8u;
   mtime[0] = 0u;
   mtime[1] = (uint32_t)(value >> 32);
   mtime[0] = (uint32_t)value;
}

/* ---------------------------------------------------------------------------
 * Semihosting, open-coded so the reset path depends on nothing
 * ------------------------------------------------------------------------ */

/* Also bench.hpp's semihosting on this board: an ebreak framed by two marker
 * instructions, uncompressed and inside one page, which the alignment
 * guarantees. */
__attribute__((noinline)) long cyros_bench_semihost(long op, void volatile* arg)
{
   register long a0 __asm__("a0") = op;
   register void volatile* a1 __asm__("a1") = arg;
   __asm__ volatile(
      ".option push       \n"
      ".option norvc      \n"
      ".p2align 4         \n"
      "slli x0, x0, 0x1f  \n"
      "ebreak             \n"
      "srai x0, x0, 7     \n"
      ".option pop        \n"
      : "+r"(a0)
      : "r"(a1)
      : "memory");
   return a0;
}

void cyros_bench_write(char const* text)
{
   cyros_bench_semihost(0x04, (void volatile*)text);
}

__attribute__((noreturn)) void cyros_bench_exit(uint32_t code)
{
   volatile uint32_t block[2] = { 0x20026u, code };   /* ADP_Stopped_ApplicationExit */
   cyros_bench_semihost(0x20, block);                 /* SYS_EXIT_EXTENDED           */
   for (;;) { __asm__ volatile("wfi"); }
}

/* ---------------------------------------------------------------------------
 * Faults before cyros owns the trap vector
 * ------------------------------------------------------------------------ */

static void write_hex(uint32_t value)
{
   char buffer[11] = { '0', 'x' };
   for (int i = 0; i < 8; ++i) {
      uint32_t const nibble = (value >> ((7 - i) * 4)) & 0xFu;
      buffer[2 + i] = (char)(nibble < 10 ? '0' + nibble : 'a' + (nibble - 10));
   }
   buffer[10] = '\0';
   cyros_bench_write(buffer);
}

__attribute__((noreturn, used)) void early_trap_report(uint32_t cause, uint32_t epc, uint32_t tval)
{
   cyros_bench_write("\n*** TRAP before cyros owns mtvec: mcause ");
   write_hex(cause);
   cyros_bench_write(", mepc ");
   write_hex(epc);
   cyros_bench_write(", mtval ");
   write_hex(tval);
   cyros_bench_write(" ***\n");
   cyros_bench_exit(3u);
}

__attribute__((naked, aligned(4), used)) void early_trap(void);
__attribute__((naked, aligned(4), used)) void early_trap(void)
{
   __asm__ volatile(
      "csrr a0, mcause         \n"
      "csrr a1, mepc           \n"
      "csrr a2, mtval          \n"
      "j    early_trap_report  \n");
}

/* ---------------------------------------------------------------------------
 * Reset
 * ------------------------------------------------------------------------ */

__attribute__((noreturn, used)) void reset_c(void)
{
   for (uint32_t* target = &__bss_start__; target < &__bss_end__; ) {
      *target++ = 0u;
   }
   for (void (**ctor)(void) = __init_array_start; ctor != __init_array_end; ++ctor) {
      (*ctor)();
   }
   cyros_bench_exit((uint32_t)cyros_bench_main());
}

/* Stacks for harts 1 to max_harts - 1. Hart h's is the h-th, counting from 1,
 * and its top is secondary_stacks + h * secondary_stack_bytes. */
#define MAX_HARTS             4
#define SECONDARY_STACK_BYTES 16384
__attribute__((aligned(16), used)) uint8_t secondary_stacks[(MAX_HARTS - 1) * SECONDARY_STACK_BYTES];

/* Weak, so a single-hart image links without it. Its address is taken
 * absolutely below, because a pc-relative reference to an undefined weak
 * symbol overflows from 0x80000000. */
extern void cyros_port_secondary_core_entry(void) __attribute__((weak));

#define STR_(x) #x
#define STR(x)  STR_(x)

__attribute__((naked, section(".text.reset"))) void _start(void)
{
   __asm__ volatile(
      "csrr t0, mhartid                    \n"
      "bnez t0, 1f                         \n"
      "la   sp, __stack_top                \n"
      "la   t0, early_trap                 \n"
      "csrw mtvec, t0                      \n"
      "j    reset_c                        \n"
      /* Any other hart: park, released by its soft IRQ. */
      "1:                                  \n"
      "li   t1, " STR(MAX_HARTS) "         \n"
      "bgeu t0, t1, 3f                     \n"
      "li   t1, " STR(SECONDARY_STACK_BYTES) " \n"
      "mul  t1, t0, t1                     \n"
      "la   sp, secondary_stacks           \n"
      "add  sp, sp, t1                     \n"
      "la   t1, early_trap                 \n"
      "csrw mtvec, t1                      \n"
      "li   t1, 8                          \n"   /* mie.MSIE, with mstatus.MIE clear */
      "csrw mie, t1                        \n"
      "2:                                  \n"
      "wfi                                 \n"
      "csrr t1, mip                        \n"
      "andi t1, t1, 8                      \n"
      "beqz t1, 2b                         \n"
      ".option push                        \n"
      ".option norelax                     \n"
      "lui  t1, %hi(cyros_port_secondary_core_entry)      \n"
      "addi t1, t1, %lo(cyros_port_secondary_core_entry)  \n"
      ".option pop                         \n"
      "beqz t1, 3f                         \n"
      "jr   t1                             \n"
      "3:                                  \n"
      "csrw mie, zero                      \n"
      "wfi                                 \n"
      "j    3b                             \n");
}
