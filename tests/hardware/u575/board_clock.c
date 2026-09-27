/**
 * @file board_clock.c
 * @brief What this board's clock tree produces, and how it gets there.
 *
 * The port declares `cyros_port_systick_clock_hz()` and deliberately supplies
 * NO default, so every image must answer. See the declaration in
 * port_time_cortex_m33.cpp for why. The answer here is BOARD_SYSCLK_HZ, the
 * same constant board_clock_init() configures, so the two cannot drift apart.
 *
 * Configure the clock BEFORE `time::start()`. The port reads the rate once,
 * when the timer is programmed. Reset_Handler does it before any constructor.
 *
 * At 4 MHz there is nothing to do: the U575 comes out of reset running from
 * MSIS at 4 MHz. At 160 MHz the sequence is ST's own, open-coded so the board
 * depends on no vendor library. It follows HAL_PWREx_ControlVoltageScaling,
 * HAL_RCC_OscConfig and HAL_RCC_ClockConfig from stm32u5xx_hal_driver:
 *
 *   1. PLL1 from MSIS: 4 MHz in, M = 1, N = 80 for a 320 MHz VCO, R = 2 for
 *      160 MHz. Configured while the EPOD booster is off, because the booster
 *      takes its clock from the PLL source through PLL1MBOOST.
 *   2. Voltage range 1 with the booster on. Range 1 is the only one that
 *      allows 160 MHz, and ST's HAL turns the EPOD booster on whenever it
 *      enters range 1 or 2.
 *   3. PLL1 on, and four flash wait states BEFORE the switch (128 to 160 MHz
 *      in range 1). Raising the clock first would read flash too fast.
 *   4. Wait for the booster, then switch SYSCLK to PLL1.
 *   5. The instruction cache on. Four wait states make every uncached fetch
 *      five cycles, and the cache is what makes 160 MHz worth having.
 *
 * AHB and APB prescalers stay at their reset value of 1, so HCLK, PCLK1 and
 * PCLK2 (the console's clock) are all SYSCLK.
 */

#include "board.h"

#include <stdint.h>

#define REG(address) (*(volatile uint32_t*)(address))

#define RCC_CR          REG(0x46020C00u)
#define RCC_CFGR1       REG(0x46020C1Cu)
#define RCC_PLL1CFGR    REG(0x46020C28u)
#define RCC_PLL1DIVR    REG(0x46020C34u)
#define RCC_AHB3ENR     REG(0x46020C94u)
#define PWR_VOSR        REG(0x4602080Cu)
#define PWR_SVMSR       REG(0x4602083Cu)
#define FLASH_ACR       REG(0x40022000u)
#define ICACHE_CR       REG(0x40030400u)
#define ICACHE_SR       REG(0x40030404u)

#define RCC_CR_PLL1ON          (1u << 24)
#define RCC_CR_PLL1RDY         (1u << 25)
#define RCC_CFGR1_SW_MASK      (3u << 0)
#define RCC_CFGR1_SW_PLL1      (3u << 0)
#define RCC_CFGR1_SWS_MASK     (3u << 2)
#define RCC_CFGR1_SWS_PLL1     (3u << 2)
#define RCC_PLL1CFGR_SRC_MSIS  (1u << 0)
#define RCC_PLL1CFGR_REN       (1u << 18)
#define RCC_AHB3ENR_PWREN      (1u << 2)
#define PWR_VOSR_BOOSTRDY      (1u << 14)
#define PWR_VOSR_VOSRDY        (1u << 15)
#define PWR_VOSR_RANGE1        (3u << 16)
#define PWR_VOSR_BOOSTEN       (1u << 18)
#define PWR_SVMSR_ACTVOSRDY    (1u << 15)
#define FLASH_ACR_LATENCY_MASK 0xFu
#define ICACHE_CR_EN           (1u << 0)
#define ICACHE_SR_BUSYF        (1u << 0)

void board_clock_init(void)
{
#if BOARD_SYSCLK_HZ == 160000000u
   RCC_AHB3ENR |= RCC_AHB3ENR_PWREN;
   (void)RCC_AHB3ENR;

   /* 1. PLL1: source MSIS, input range 4 to 8 MHz (PLL1RGE = 0), M = 1 and
    * PLL1MBOOST = 1 (both fields hold divider minus one, so zero), R output
    * enabled. Dividers hold their value minus one too. */
   RCC_PLL1CFGR = RCC_PLL1CFGR_SRC_MSIS | RCC_PLL1CFGR_REN;
   RCC_PLL1DIVR = (80u - 1u) | ((2u - 1u) << 9) | ((2u - 1u) << 16) | ((2u - 1u) << 24);

   /* 2. Range 1 with the booster, then both ready flags, as the HAL does. */
   PWR_VOSR = (PWR_VOSR & ~((3u << 16) | PWR_VOSR_BOOSTEN)) | PWR_VOSR_RANGE1 | PWR_VOSR_BOOSTEN;
   while ((PWR_VOSR & PWR_VOSR_VOSRDY) == 0u) {}
   while ((PWR_SVMSR & PWR_SVMSR_ACTVOSRDY) == 0u) {}

   /* 3. PLL1 on, and the wait states before the clock rises. Read back, since
    * the new latency is only in force once the register says so. */
   RCC_CR |= RCC_CR_PLL1ON;
   while ((RCC_CR & RCC_CR_PLL1RDY) == 0u) {}
   FLASH_ACR = (FLASH_ACR & ~FLASH_ACR_LATENCY_MASK) | 4u;
   while ((FLASH_ACR & FLASH_ACR_LATENCY_MASK) != 4u) {}

   /* 4. The booster, then the switch. */
   while ((PWR_VOSR & PWR_VOSR_BOOSTRDY) == 0u) {}
   RCC_CFGR1 = (RCC_CFGR1 & ~RCC_CFGR1_SW_MASK) | RCC_CFGR1_SW_PLL1;
   while ((RCC_CFGR1 & RCC_CFGR1_SWS_MASK) != RCC_CFGR1_SWS_PLL1) {}

   /* 5. The cache invalidates itself after reset and refuses EN until done. */
   while ((ICACHE_SR & ICACHE_SR_BUSYF) != 0u) {}
   ICACHE_CR |= ICACHE_CR_EN;
#endif
}

uint32_t cyros_port_systick_clock_hz(void)
{
   return BOARD_SYSCLK_HZ;
}
