/**
 * @file board_clock.c
 * @brief What this board's clock tree produces, and how it gets there.
 *
 * The port declares `cyros_port_systick_clock_hz()` and deliberately supplies
 * NO default, so every image must answer. The answer here is BOARD_SYSCLK_HZ,
 * the same constant board_clock_init() configures, so the two cannot drift
 * apart. Configure the clock BEFORE `time::start()`, which reads the rate once.
 * Reset_Handler does it before any constructor.
 *
 * The part resets onto its 16 MHz internal oscillator, which TI's erratum
 * ELEC#05 allows to be more than 3 per cent off. That would scale every
 * duration in the system by up to 3 per cent with nothing to catch it, so both
 * clocks here come from the board's 16 MHz crystal (the main oscillator,
 * MOSC).
 *
 * THE SEQUENCE IS TI's, line for line: TivaWare's SysCtlClockSet
 * (driverlib/sysctl.c) for one configuration word, open-coded so the board
 * depends on no vendor library. The word is TivaWare's own encoding, so each
 * step can be checked against that source.
 *
 * Do not "simplify" it. A hand-written version following the datasheet's five
 * steps (SPMS376E, 5.3) booted at 12.5 MHz about one time in ten, the PLL's
 * 200 MHz over RCC's /16, while RCC2 read back the 80 MHz configuration in every
 * field. A second, nearer TI's order but merging two of its writes and dropping
 * its final delay, came up wrong on its first boot. This one: 60 boots out of
 * 60 at 80 MHz (DWT cycles against a host second, 2026-10-03). Which of TI's
 * details matters was not isolated.
 *
 * At 16 MHz the crystal drives SYSCLK directly, bypassed and undivided, and RCC2
 * stays unused. Flash needs no wait states set: the TM4C123's prefetch buffer
 * covers 80 MHz by itself. PC0 to PC3 are JTAG and nothing here touches them
 * (GPIO#01).
 */

#include "board.h"

#include <stdint.h>

#define REG(address) (*(volatile uint32_t*)(address))

#define SYSCTL_RIS     REG(0x400FE050u)
#define SYSCTL_MISC    REG(0x400FE058u)
#define SYSCTL_RCC     REG(0x400FE060u)
#define SYSCTL_RCC2    REG(0x400FE070u)
#define SYSCTL_PLLSTAT REG(0x400FE168u)

/* inc/hw_sysctl.h */
#define RCC_SYSDIV_M      0x07800000u
#define RCC_USESYSDIV     0x00400000u
#define RCC_PWRDN         0x00002000u
#define RCC_BYPASS        0x00000800u
#define RCC_XTAL_M        0x000007C0u
#define RCC_OSCSRC_M      0x00000030u
#define RCC_MOSCDIS       0x00000001u
#define RCC2_USERCC2      0x80000000u
#define RCC2_DIV400       0x40000000u
#define RCC2_SYSDIV2_M    0x1F800000u
#define RCC2_SYSDIV2LSB   0x00400000u
#define RCC2_PWRDN2       0x00002000u
#define RCC2_BYPASS2      0x00000800u
#define RCC2_OSCSRC2_M    0x00000070u
#define RIS_MOSCPUPRIS    0x00000100u
#define MISC_MOSCPUPMIS   0x00000100u
#define MISC_PLLLMIS      0x00000040u
#define PLLSTAT_LOCK      0x00000001u

/* driverlib/sysctl.h: SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN with, at 80 MHz,
 * SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL and, at 16 MHz, SYSCTL_SYSDIV_1 |
 * SYSCTL_USE_OSC. */
#if BOARD_SYSCLK_HZ == 80000000u
#define CLOCK_CONFIG (0xC1000000u | 0x00000000u | 0x00000540u)
#else
#define CLOCK_CONFIG (0x07800000u | 0x00003800u | 0x00000540u)
#endif

void board_clock_init(void)
{
   uint32_t const config = CLOCK_CONFIG;
   uint32_t rcc  = SYSCTL_RCC;
   uint32_t rcc2 = SYSCTL_RCC2;

   /* Bypass the PLL and the system divider for now. */
   rcc |= RCC_BYPASS;
   rcc &= ~RCC_USESYSDIV;
   rcc2 |= RCC2_BYPASS2;
   SYSCTL_RCC  = rcc;
   SYSCTL_RCC2 = rcc2;

   /* The crystal, if it is off, which it is after reset. TivaWare gives up
    * after a bounded wait and stays on the old clock. Here nothing could
    * report that, so it waits. */
   if ((rcc & RCC_MOSCDIS) != 0u) {
      rcc &= ~RCC_MOSCDIS;
      SYSCTL_MISC = MISC_MOSCPUPMIS;
      SYSCTL_RCC = rcc;
      while ((SYSCTL_RIS & RIS_MOSCPUPRIS) == 0u) {}
   }

   /* Crystal value and oscillator source. */
   rcc &= ~(RCC_XTAL_M | RCC_OSCSRC_M);
   rcc |= config & (RCC_XTAL_M | RCC_OSCSRC_M);
   rcc2 &= ~(RCC2_USERCC2 | RCC2_OSCSRC2_M);
   rcc2 |= config & (RCC2_USERCC2 | RCC_OSCSRC_M);
   rcc2 |= (config & 0x00000008u) << 3;
   SYSCTL_RCC  = rcc;
   SYSCTL_RCC2 = rcc2;

   /* PLL power, written RCC2 first when RCC2 is in charge. */
   rcc &= ~RCC_PWRDN;
   rcc |= config & RCC_PWRDN;
   rcc2 &= ~RCC2_PWRDN2;
   rcc2 |= config & RCC2_PWRDN2;
   SYSCTL_MISC = MISC_PLLLMIS;
   if ((rcc2 & RCC2_USERCC2) != 0u) {
      SYSCTL_RCC2 = rcc2;
      SYSCTL_RCC  = rcc;
   } else {
      SYSCTL_RCC  = rcc;
      SYSCTL_RCC2 = rcc2;
   }

   /* The divider, not written yet. */
   rcc &= ~(RCC_SYSDIV_M | RCC_USESYSDIV | RCC_MOSCDIS);
   rcc |= config & (RCC_SYSDIV_M | RCC_USESYSDIV | RCC_MOSCDIS);
   rcc2 &= ~RCC2_SYSDIV2_M;
   rcc2 |= config & RCC2_SYSDIV2_M;
   if ((config & RCC2_DIV400) != 0u) {
      rcc |= RCC_USESYSDIV;
      rcc2 &= ~RCC2_SYSDIV2LSB;
      rcc2 |= config & (RCC2_DIV400 | RCC2_SYSDIV2LSB);
   } else {
      rcc2 &= ~RCC2_DIV400;
   }

   /* Wait for lock, then stop bypassing. TivaWare bounds the wait and goes
    * ahead regardless. This waits, for the same reason as the crystal. */
   if ((config & RCC_BYPASS) == 0u) {
      while ((SYSCTL_PLLSTAT & PLLSTAT_LOCK) == 0u) {}
      rcc &= ~RCC_BYPASS;
      rcc2 &= ~RCC2_BYPASS2;
   }

   SYSCTL_RCC  = rcc;
   SYSCTL_RCC2 = rcc2;

   /* "Delay for a little bit so that the system divider takes effect":
    * SysCtlDelay(16), three cycles a loop. */
   for (uint32_t i = 0; i < 16u; ++i) {
      __asm__ volatile("");
   }
}

uint32_t cyros_port_systick_clock_hz(void)
{
   return BOARD_SYSCLK_HZ;
}
