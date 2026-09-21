/**
 * @file board_clock.c
 * @brief What this board's clock tree actually produces.
 *
 * The port declares `cyros_port_systick_clock_hz()` and deliberately supplies
 * NO default, so every image must answer. See the declaration in
 * port_time_cortex_m33.cpp for why.
 *
 * The STM32U575 comes out of reset running from MSIS at 4 MHz, and this image
 * does no clock configuration at all, so 4 MHz is the truth here.
 *
 * **WHEN THIS IMAGE GROWS PLL SETUP, THIS FUNCTION MUST CHANGE WITH IT.** The
 * U575 runs to 160 MHz, so an application that configures the PLL and leaves
 * this at 4 MHz gets every duration in the system wrong by a factor of forty,
 * with nothing to report it. Two options at that point, both fine:
 *
 *   - return the literal the clock tree was configured to, or
 *   - read the clock tree and compute it, which cannot drift out of step.
 *
 * Either way, configure the clock BEFORE `time::start()`. The port reads this
 * once, when the timer is programmed.
 */

#include <stdint.h>

uint32_t cyros_port_systick_clock_hz(void)
{
   return 4000000u;
}
