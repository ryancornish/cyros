/**
 * @file rp2040.h
 * @brief The RP2040 registers this board uses, the 125 MHz clock bring-up, and
 *        the 1 us tick that drives the TIMER.
 *
 * Addresses are from the RP2040 datasheet. The sequence is the one
 * ~/cyros-claude/tools-rp2040-loop/cyros_board measured on this board
 * (arm-port-notes.md 19f): an image starts after a debugger's reset halt,
 * which resets only the cores, so it switches clocks glitch-free from
 * whatever the boot ROM or the last image left, and holds every peripheral it
 * does not use in reset.
 */
#ifndef CYROS_RP2040_H
#define CYROS_RP2040_H

#include <stdbool.h>
#include <stdint.h>

#define REG(a) (*(volatile uint32_t*)(uintptr_t)(a))
#define SET(a) REG((a) + 0x2000u)   /* atomic set alias   */
#define CLR(a) REG((a) + 0x3000u)   /* atomic clear alias */

#define CLK_REF_CTRL         0x40008030u
#define CLK_REF_DIV          0x40008034u
#define CLK_REF_SELECTED     0x40008038u
#define CLK_SYS_CTRL         0x4000803cu
#define CLK_SYS_DIV          0x40008040u
#define CLK_SYS_SELECTED     0x40008044u
#define CLK_SYS_RESUS_CTRL   0x40008078u
#define RESETS_RESET         0x4000c000u
#define RESETS_DONE          0x4000c008u
#define XOSC_CTRL            0x40024000u
#define XOSC_STATUS          0x40024004u
#define XOSC_STARTUP         0x4002400cu
#define PLL_CS               0x40028000u
#define PLL_PWR              0x40028004u
#define PLL_FBDIV_INT        0x40028008u
#define PLL_PRIM             0x4002800cu
#define TIMER_TIMEHW         0x40054000u
#define TIMER_TIMELW         0x40054004u
#define TIMER_DBGPAUSE       0x4005402cu
#define WATCHDOG_CTRL        0x40058000u
#define WATCHDOG_TICK        0x4005802cu
#define SIO_CPUID            0xd0000000u

#define RESET_IO_QSPI        (1u << 6)
#define RESET_PADS_QSPI      (1u << 9)
#define RESET_PLL_SYS        (1u << 12)
#define RESET_PLL_USB        (1u << 13)
#define RESET_SYSCFG         (1u << 18)
#define RESET_TIMER          (1u << 21)
#define RESET_ALL            0x01ffffffu

#define PLL_PWR_PD           (1u << 0)
#define PLL_PWR_POSTDIVPD    (1u << 3)
#define PLL_PWR_VCOPD        (1u << 5)

#define TIMER_DBGPAUSE_DBG0  (1u << 1)

/* 12 MHz x 125 = 1500 MHz VCO, / 6 / 2 = 125 MHz, the SDK's RP2040 default. */
#define PLL_FBDIV    125u
#define PLL_POSTDIV1 6u
#define PLL_POSTDIV2 2u

static inline bool wait_bits(uint32_t addr, uint32_t mask, uint32_t want)
{
   for (uint32_t spins = 0; (REG(addr) & mask) != want; ) {
      if (++spins == 10000000u) {
         return false;
      }
   }
   return true;
}

/* Every peripheral in reset but the flash's pins, SYSCFG and the two PLLs,
 * which the clock tree may still be running from, and the watchdog stopped.
 * The tests use no peripheral but the TIMER, which board_timer_init brings
 * back. A debugger start inherits whatever ran before, the boot ROM's USB in
 * BOOTSEL among it. */
static inline void peripherals_quiet(void)
{
   CLR(WATCHDOG_CTRL) = 1u << 30;                   /* ENABLE */
   SET(RESETS_RESET) = RESET_ALL & ~(RESET_IO_QSPI | RESET_PADS_QSPI | RESET_SYSCFG |
                                     RESET_PLL_SYS | RESET_PLL_USB);
}

/* clk_ref on the 12 MHz crystal and clk_sys on PLL_SYS at 125 MHz, glitch-free
 * from any state: clk_sys onto clk_ref first, then clk_ref onto the crystal,
 * then the PLL from reset, as the SDK's pll_init does it. */
static inline bool clocks_125mhz(void)
{
   REG(CLK_SYS_RESUS_CTRL) = 0u;
   if (!(REG(XOSC_STATUS) & (1u << 31))) {
      REG(XOSC_STARTUP) = 47u;                      /* about 1 ms at 12 MHz */
      REG(XOSC_CTRL) = 0xaa0u | (0xfabu << 12);     /* 1 to 15 MHz range, enabled */
      if (!wait_bits(XOSC_STATUS, 1u << 31, 1u << 31)) {
         return false;
      }
   }

   CLR(CLK_SYS_CTRL) = 1u;
   if (!wait_bits(CLK_SYS_SELECTED, ~0u, 1u)) {
      return false;
   }
   REG(CLK_REF_CTRL) = 2u;
   if (!wait_bits(CLK_REF_SELECTED, ~0u, 1u << 2)) {
      return false;
   }
   REG(CLK_REF_DIV) = 1u << 8;
   REG(CLK_SYS_DIV) = 1u << 8;

   SET(RESETS_RESET) = RESET_PLL_SYS;
   CLR(RESETS_RESET) = RESET_PLL_SYS;
   if (!wait_bits(RESETS_DONE, RESET_PLL_SYS, RESET_PLL_SYS)) {
      return false;
   }
   REG(PLL_CS) = 1u;                                /* refdiv 1 */
   REG(PLL_FBDIV_INT) = PLL_FBDIV;
   CLR(PLL_PWR) = PLL_PWR_PD | PLL_PWR_VCOPD;
   if (!wait_bits(PLL_CS, 1u << 31, 1u << 31)) {
      return false;
   }
   REG(PLL_PRIM) = (PLL_POSTDIV1 << 16) | (PLL_POSTDIV2 << 12);
   CLR(PLL_PWR) = PLL_PWR_POSTDIVPD;

   /* clk_sys: aux = pll_sys while still on clk_ref, then onto aux. */
   REG(CLK_SYS_CTRL) = 0u;
   SET(CLK_SYS_CTRL) = 1u;
   return wait_bits(CLK_SYS_SELECTED, ~0u, 2u);
}

/* The TIMER at 1 MHz: the watchdog's tick generator divides clk_ref, the
 * crystal, by 12, and the TIMER counts its ticks. A debug halt of core 0
 * pauses it and one of core 1 does not, as MTIME on the Pico 2 W board. */
static inline bool board_timer_init(void)
{
   REG(WATCHDOG_TICK) = (1u << 9) | 12u;            /* ENABLE, 12 cycles */
   CLR(RESETS_RESET) = RESET_TIMER;
   if (!wait_bits(RESETS_DONE, RESET_TIMER, RESET_TIMER)) {
      return false;
   }
   REG(TIMER_DBGPAUSE) = TIMER_DBGPAUSE_DBG0;
   return true;
}

#endif /* CYROS_RP2040_H */
