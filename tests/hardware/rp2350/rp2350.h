/**
 * @file rp2350.h
 * @brief The RP2350 registers this board uses, and the 150 MHz clock bring-up.
 *
 * Addresses are from pico-sdk's src/rp2350/hardware_regs headers. The clock
 * sequence is the one ~/cyros-claude/tools-rp2350-probe measured on this board
 * (rp2350-notes.md 2b): it switches glitch-free from whatever the boot ROM or
 * the last image left, because a debugger start resets only the cores and a
 * boot ROM RAM boot leaves the chip on the ring oscillator with the crystal
 * stopped.
 */
#ifndef CYROS_RP2350_H
#define CYROS_RP2350_H

#include <stdbool.h>
#include <stdint.h>

#define REG(a) (*(volatile uint32_t*)(uintptr_t)(a))
#define SET(a) REG((a) + 0x2000u)   /* atomic set alias   */
#define CLR(a) REG((a) + 0x3000u)   /* atomic clear alias */

#define CLOCKS               0x40010000u
#define CLK_REF_CTRL         (CLOCKS + 0x30u)
#define CLK_REF_DIV          (CLOCKS + 0x34u)
#define CLK_REF_SELECTED     (CLOCKS + 0x38u)
#define CLK_SYS_CTRL         (CLOCKS + 0x3cu)
#define CLK_SYS_DIV          (CLOCKS + 0x40u)
#define CLK_SYS_SELECTED     (CLOCKS + 0x44u)
#define CLK_SYS_RESUS_CTRL   (CLOCKS + 0x84u)
#define RESETS_RESET         0x40020000u
#define RESETS_DONE          0x40020008u
#define XOSC_CTRL            0x40048000u
#define XOSC_STATUS          0x40048004u
#define XOSC_STARTUP         0x4004800cu
#define PLL_CS               0x40050000u
#define PLL_PWR              0x40050004u
#define PLL_FBDIV_INT        0x40050008u
#define PLL_PRIM             0x4005000cu

#define QMI_M0_TIMING        0x400d000cu
#define XIP_NOCACHE_BASE     0x14000000u

#define SIO_MTIME_CTRL       0xd00001a4u
#define SIO_MTIME            0xd00001b0u
#define SIO_MTIMEH           0xd00001b4u

#define MTIME_EN             (1u << 0)
#define MTIME_FULLSPEED      (1u << 1)
#define MTIME_DBGPAUSE0      (1u << 2)
#define MTIME_DBGPAUSE1      (1u << 3)

#define RESET_PLL_SYS        (1u << 14)
#define PLL_PWR_PD           (1u << 0)
#define PLL_PWR_POSTDIVPD    (1u << 3)
#define PLL_PWR_VCOPD        (1u << 5)

#define FBDIV                125u   /* 12 MHz * 125 = 1500 MHz VCO */
#define POSTDIV1             5u
#define POSTDIV2             2u     /* 1500 / 5 / 2 = 150 MHz      */
#define XOSC_STARTUP_DELAY   (47u * 64u)   /* about 64 ms, the SDK's RP2350 default */
#define FLASH_CLKDIV_150MHZ  6u     /* SCK 25 MHz at 150 MHz       */

static inline bool wait_bits(uint32_t addr, uint32_t mask, uint32_t want)
{
   for (uint32_t spins = 0; (REG(addr) & mask) != want; ) {
      if (++spins == 10000000u) { return false; }
   }
   return true;
}

/* The flash's SCK is clk_sys over QMI M0_TIMING.CLKDIV. A flash boot leaves
 * the read mode and divisor the boot ROM's scan found working at its own slow
 * clock, as low as 3 (datasheet 5.2.7, table 463): on this board EBh quad at
 * 3 with RXDELAY 2, which at 150 MHz is SCK 50 MHz (rp2350-notes.md 2d).
 * That is inside the W25Q32JV's quad rating, but had the scan settled on 03h
 * at 3 it would sit at that read's limit, so the divisor rises to 6 before the
 * clock does, for margin in any mode at half the miss bandwidth. The divisor
 * may change mid-access, and a read past the cache makes it take effect
 * before the clock does (QMI M0_TIMING). A RAM image finds the QMI at reset
 * (03h, divisor 4) and is raised the same way, harmlessly. */
static inline void flash_clock_for_150mhz(void)
{
   uint32_t const timing = REG(QMI_M0_TIMING);
   uint32_t const clkdiv = timing & 0xffu;             /* 0 encodes 256 */
   if (clkdiv == 0u || clkdiv >= FLASH_CLKDIV_150MHZ) {
      return;
   }
   REG(QMI_M0_TIMING) = (timing & ~0xffu) | FLASH_CLKDIV_150MHZ;
   uint32_t const word = REG(XIP_NOCACHE_BASE);
#if defined(__riscv)
   __asm__ volatile("fence" : : "r"(word) : "memory");
#else
   __asm__ volatile("dsb" : : "r"(word) : "memory");
#endif
}

/* 150 MHz on clk_sys from PLL_SYS, clk_ref on the 12 MHz crystal. False if a
 * step never completed. */
static inline bool clocks_150mhz(void)
{
   REG(CLK_SYS_RESUS_CTRL) = 0;
   if (!(REG(XOSC_STATUS) & (1u << 31))) {
      REG(XOSC_STARTUP) = XOSC_STARTUP_DELAY;
      REG(XOSC_CTRL) = 0xaa0u | (0xfabu << 12);        /* 1 to 15 MHz range, enabled */
      if (!wait_bits(XOSC_STATUS, 1u << 31, 1u << 31)) { return false; }
   }
   /* clk_sys onto clk_ref, then clk_ref onto the crystal, undivided. */
   CLR(CLK_SYS_CTRL) = 1u;
   if (!wait_bits(CLK_SYS_SELECTED, ~0u, 1u)) { return false; }
   REG(CLK_REF_CTRL) = 2u;
   if (!wait_bits(CLK_REF_SELECTED, ~0u, 1u << 2)) { return false; }
   REG(CLK_REF_DIV) = 1u << 16;

   /* PLL_SYS from reset, as the SDK's pll_init does it. */
   SET(RESETS_RESET) = RESET_PLL_SYS;
   CLR(RESETS_RESET) = RESET_PLL_SYS;
   if (!wait_bits(RESETS_DONE, RESET_PLL_SYS, RESET_PLL_SYS)) { return false; }
   REG(PLL_PWR) = 0xffffffffu;
   REG(PLL_FBDIV_INT) = 0;
   REG(PLL_CS) = 1u;                                   /* refdiv 1 */
   REG(PLL_FBDIV_INT) = FBDIV;
   CLR(PLL_PWR) = PLL_PWR_PD | PLL_PWR_VCOPD;
   if (!wait_bits(PLL_CS, 1u << 31, 1u << 31)) { return false; }
   REG(PLL_PRIM) = (POSTDIV1 << 16) | (POSTDIV2 << 12);
   CLR(PLL_PWR) = PLL_PWR_POSTDIVPD;

   /* clk_sys: aux = pll_sys while still on clk_ref, then onto aux. */
   flash_clock_for_150mhz();
   REG(CLK_SYS_DIV) = 1u << 16;
   REG(CLK_SYS_CTRL) = 0;
   SET(CLK_SYS_CTRL) = 1u;
   return wait_bits(CLK_SYS_SELECTED, ~0u, 2u);
}

#endif /* CYROS_RP2350_H */
