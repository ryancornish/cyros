/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file board.h
 * @brief The RP2040 Pico as cyros's on-target tests see it: its clock, its
 *        TIMER's rate and its console. The sibling of ../rp2350.
 *
 * 125 MHz from the 12 MHz crystal through PLL_SYS, the TIMER at 1 MHz from
 * the crystal (rp2040.h), and a console over SEGGER RTT through the Debug
 * Probe.
 */
#ifndef CYROS_RP2040_BOARD_H
#define CYROS_RP2040_BOARD_H

#include <stdbool.h>
#include <stdint.h>

#define BOARD_SYSCLK_HZ 125000000u
#define BOARD_TIMER_HZ  1000000u

void board_console_init(void);
bool board_console_drain(uint32_t spin_limit);

void cyros_bench_write(char const* text);
__attribute__((noreturn)) void cyros_bench_exit(uint32_t code);

#endif
