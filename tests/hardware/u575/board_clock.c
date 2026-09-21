/**
 * @file board_clock.c
 * @brief What this board's clock tree actually produces.
 *
 * Overrides the port's weak default, which is the QEMU bench's 20 MHz. The
 * STM32U575 comes out of reset running from MSIS at 4 MHz and this image does
 * no clock configuration, so 4 MHz is the truth here.
 *
 * Getting this wrong does not fail to build or run. It silently scales every
 * duration in the system, which is precisely how the bench value went
 * unnoticed at 25 per cent out for a day. The number is a BOARD fact and this
 * file is where the board states it.
 */

#include <stdint.h>

uint32_t const cyros_port_systick_clock_hz = 4000000u;
