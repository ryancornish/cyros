/**
 * @file startup_leon3.c
 * @brief Reset path, console, exit and host time for cyros's on-target tests
 *        on QEMU's leon3_generic, one LEON3 or several.
 *
 * QEMU loads the ELF into RAM and starts core 0 through a few instructions of
 * its own in the PROM, which enable the APBUART, set GPTIMER's prescaler and
 * jump to the entry. Every other core stays powered down until the IRQMP
 * starts it, and then QEMU sends it straight to the entry. Each arrives with
 * traps disabled (PSR.ET 0), the FPU off (PSR.EF 0), CWP 0, WIM 1, TBR 0 and
 * no stack (~/cyros-claude/sparc-port-plan.md has them measured). Each core:
 *
 *  1. points TBR at the port's trap table before anything that could trap,
 *     because any call of compiled code may need a window trap,
 *  2. marks the window above its current one invalid, so it has every other,
 *  3. takes its stack: core 0 the linker script's __stack_top, which becomes
 *     its interrupt stack when the kernel starts, any other its own,
 *  4. enables traps at PIL 15, masked, with the FPU on in a hard-float image,
 *     before any compiled code, because the port saves FP state in every
 *     trap frame and a compiled handler may use FP registers,
 *  5. core 0 runs the image. Any other core joins the kernel through
 *     cyros_port_secondary_core_entry, which only a multicore target has, and
 *     only its start_cores starts another core.
 *
 * The board sets GPTIMER's prescaler itself, from the system clock it knows
 * (QEMU's 40 MHz), rather than trust QEMU's PROM code, and states the rate
 * the port's time source runs at.
 *
 * Output is the APBUART. QEMU has no semihosting on SPARC and exits with 0
 * whatever the guest did, so the exit prints "EXIT: n" and halts the core,
 * and the runner (run_leon3.py) exits with n. The runner also answers the
 * bench's two elapsed-time calls with host time, over the same UART: the
 * only time reference here that is not the thing under test (bench.hpp).
 */

#include <stdint.h>

extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern int cyros_bench_main(void);
extern void (*__init_array_start[])(void);
extern void (*__init_array_end[])(void);

/* QEMU's leon3_generic: the system clock, and where its peripherals are. The
 * port finds the last three for itself through plug and play. The board
 * knows its own machine. */
#define LEON3_SYSTEM_CLOCK_HZ 40000000u
#define LEON3_TIMER_HZ        1000000u
#define LEON3_APBUART         0x80000100u
#define LEON3_GPTIMER         0x80000300u

#define REG(address) (*(volatile uint32_t*)(address))

/* MTIME is GPTIMER here (leon3's port_mcu), counting at the prescaler's
 * rate, which reset_c sets. */
uint32_t cyros_port_mtime_clock_hz(void)
{
   return LEON3_TIMER_HZ;
}

/* ---------------------------------------------------------------------------
 * Console and exit
 * ------------------------------------------------------------------------ */

#define UART_DATA          (LEON3_APBUART + 0x0u)
#define UART_STATUS        (LEON3_APBUART + 0x4u)
#define UART_CONTROL       (LEON3_APBUART + 0x8u)
#define UART_DATA_READY    (1u << 0)
#define UART_TRANSMIT_FULL (1u << 9)

static void uart_put(char c)
{
   while ((REG(UART_STATUS) & UART_TRANSMIT_FULL) != 0u) {}
   REG(UART_DATA) = (uint8_t)c;
}

void cyros_bench_write(char const* text)
{
   for (; *text != '\0'; ++text) {
      uart_put(*text);
   }
}

static void write_dec(uint32_t value)
{
   char buffer[11];
   int at = 10;
   buffer[at] = '\0';
   do {
      buffer[--at] = (char)('0' + value % 10u);
      value /= 10u;
   } while (value != 0u);
   cyros_bench_write(&buffer[at]);
}

/* A trap with traps disabled is the error state, which halts the core. `ta 0`
 * is QEMU's shutdown, of every core. */
__attribute__((noreturn)) static void halt(void)
{
   __asm__ volatile("rd   %%psr, %%g1      \n"
                    "andn %%g1, 0x20, %%g1 \n"
                    "wr   %%g1, %%psr      \n"
                    "nop                   \n"
                    "nop                   \n"
                    "nop                   \n"
                    "ta   0                \n"
                    ::: "g1", "memory");
   for (;;) {}
}

__attribute__((noreturn)) void cyros_bench_exit(uint32_t code)
{
   cyros_bench_write("\nEXIT: ");
   write_dec(code);
   cyros_bench_write("\n");
   halt();
}

/* ---------------------------------------------------------------------------
 * Host time, from the runner (bench.hpp's elapsed_ns)
 * ---------------------------------------------------------------------------
 *
 * The board asks with a line of its own on the UART, "@@cyros-host <what>",
 * and the runner answers with a decimal number and a newline on the UART's
 * receive side: ticks per second for SYS_TICKFREQ (always 1e9), nanoseconds
 * since QEMU started for SYS_ELAPSED. With no runner, nothing answers within
 * a second, and the call reports itself unsupported, as on a board whose
 * debugger does not implement it. A round trip costs the host's pipe latency,
 * well under a millisecond, so it belongs at the edges of a measurement.
 */

static int read_reply(uint64_t* value)
{
   /* Timed on GPTIMER's timer 1, whatever its reload, by the modular
    * difference of its down-count: a second at the prescaler's rate. */
   uint32_t const start = REG(LEON3_GPTIMER + 0x10u);
   uint64_t number = 0u;
   int digits = 0;
   while ((uint32_t)(start - REG(LEON3_GPTIMER + 0x10u)) < LEON3_TIMER_HZ) {
      if ((REG(UART_STATUS) & UART_DATA_READY) == 0u) {
         continue;
      }
      char const c = (char)REG(UART_DATA);
      if (c >= '0' && c <= '9') {
         number = number * 10u + (uint64_t)(c - '0');
         ++digits;
      } else if (c == '\n' && digits > 0) {
         *value = number;
         return 1;
      }
   }
   return 0;
}

__attribute__((noinline)) long cyros_bench_semihost(long op, void volatile* arg)
{
   uint64_t value = 0u;
   if (op == 0x31) {   /* SYS_TICKFREQ */
      cyros_bench_write("@@cyros-host tickfreq\n");
      return read_reply(&value) ? (long)value : -1;
   }
   if (op == 0x30) {   /* SYS_ELAPSED, into a two-word block, low word first */
      cyros_bench_write("@@cyros-host elapsed\n");
      if (!read_reply(&value)) {
         return -1;
      }
      volatile uint32_t* const block = (volatile uint32_t*)arg;
      block[0] = (uint32_t)value;
      block[1] = (uint32_t)(value >> 32);
      return 0;
   }
   return -1;
}

/* ---------------------------------------------------------------------------
 * Reset
 * ------------------------------------------------------------------------ */

__attribute__((noreturn, used)) void reset_c(void)
{
   for (uint32_t* target = &__bss_start__; target < &__bss_end__; ) {
      *target++ = 0u;
   }

   /* The UART's transmitter and receiver, and GPTIMER's prescaler at the
    * rate cyros_port_mtime_clock_hz states: the scaler reload is the system
    * clock's divisor less one. */
   REG(UART_CONTROL) = 0x3u;
   REG(LEON3_GPTIMER + 0x4u) = LEON3_SYSTEM_CLOCK_HZ / LEON3_TIMER_HZ - 1u;

   for (void (**ctor)(void) = __init_array_start; ctor != __init_array_end; ++ctor) {
      (*ctor)();
   }
   cyros_bench_exit((uint32_t)cyros_bench_main());
}

/* Every core but 0 joins the kernel here. Weak, so a single-core image links
 * without it: its other cores are never started. */
extern void cyros_port_secondary_core_entry(void) __attribute__((weak));

/* Per-core stacks for every core but 0, whose stack is the linker script's. */
#define MAX_CORES         4
#define CORE_STACK_BYTES  16384

__attribute__((aligned(8), used)) uint8_t leon3_core_stacks[(MAX_CORES - 1) * CORE_STACK_BYTES];

#define STR_(x) #x
#define STR(x)  STR_(x)

/* S, PS, ET and PIL 15, and EF in a hard-float image. */
#if defined(_SOFT_FLOAT)
#define LEON3_PSR_BITS "0xfe0"
#else
#define LEON3_PSR_BITS "0x1fe0"
#endif

__asm__(
   ".pushsection .text.reset, \"ax\"              \n"
   ".global _start                                \n"
   ".type   _start, #function                     \n"
   "_start:                                       \n"
   /* 1. The trap table, before anything that could trap. */
   "   set   cyros_sparc_trap_table, %g1          \n"
   "   wr    %g1, %tbr                            \n"
   /* 2. WIM: the window above this one (CWP + 1) is the invalid one. */
   "   rd    %psr, %g2                            \n"
   "   and   %g2, 0x1f, %g3                       \n"
   "   add   %g3, 1, %g3                          \n"
   "   and   %g3, 7, %g3                          \n"
   "   mov   1, %g4                               \n"
   "   sll   %g4, %g3, %g4                        \n"
   "   wr    %g4, %wim                            \n"
   /* 3. The stack: this core's index from ASR17. */
   "   rd    %asr17, %g5                          \n"
   "   srl   %g5, 28, %g5                         \n"
   "   cmp   %g5, 0                               \n"
   "   bne   1f                                   \n"
   "    nop                                       \n"
   "   set   __stack_top, %g1                     \n"
   "   ba    2f                                   \n"
   "    nop                                       \n"
   "1: set   leon3_core_stacks, %g1               \n"
   "   set   " STR(CORE_STACK_BYTES) ", %g3       \n"
   "   umul  %g5, %g3, %g3                        \n"   /* index * size is the top of core index - 1's */
   "   add   %g1, %g3, %g1                        \n"
   "2: sub   %g1, 96, %sp                         \n"
   "   mov   %g0, %fp                             \n"
   /* 4. Traps on, masked, keeping CWP. */
   "   rd    %psr, %g2                            \n"
   "   and   %g2, 0x1f, %g2                       \n"
   "   set   " LEON3_PSR_BITS ", %g3              \n"
   "   or    %g2, %g3, %g2                        \n"
   "   wr    %g2, %psr                            \n"
   "   nop                                        \n"
   "   nop                                        \n"
   "   nop                                        \n"
   /* 5. Core 0 runs the image, any other joins the kernel. */
   "   cmp   %g5, 0                               \n"
   "   bne   3f                                   \n"
   "    nop                                       \n"
   "   call  reset_c                              \n"
   "    nop                                       \n"
   "3: set   cyros_port_secondary_core_entry, %g1 \n"
   "   cmp   %g1, 0                               \n"
   "   be    4f                                   \n"
   "    nop                                       \n"
   "   call  %g1                                  \n"
   "    nop                                       \n"
   "4: wr    %g0, %asr19                          \n"   /* power down, for good */
   "   ba    4b                                   \n"
   "    nop                                       \n"
   ".weak cyros_port_secondary_core_entry         \n"
   ".size   _start, . - _start                    \n"
   ".popsection                                   \n");
