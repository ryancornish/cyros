/**
 * @file board.h
 * @brief The Pico 2 W as cyros's on-target tests see it, on either ISA: its
 *        clock and its console. The sibling of ../u575 and ../tm4c123.
 *
 * 150 MHz from the 12 MHz crystal through PLL_SYS (rp2350.h), MTIME at that
 * rate (FULLSPEED), and a console over SEGGER RTT through the Debug Probe.
 * startup_rp2350_hazard3.c boots the Hazard3 cores, startup_rp2350_m33.c the
 * Cortex-M33s, and everything else here serves both.
 */
#ifndef CYROS_RP2350_BOARD_H
#define CYROS_RP2350_BOARD_H

#include <stdbool.h>
#include <stdint.h>

#define BOARD_SYSCLK_HZ 150000000u

void board_console_init(void);
bool board_console_drain(uint32_t spin_limit);

void cyros_bench_write(char const* text);
__attribute__((noreturn)) void cyros_bench_exit(uint32_t code);

#endif
