/**
 * @file board.h
 * @brief The EK-TM4C123GXL as the tests see it: its clock and its console.
 *
 * Every clock-derived setting on this board is computed from BOARD_SYSCLK_HZ:
 * the SysTick rate cyros is told (board_clock.c) and the console's baud
 * divisor (console.c). Changing the clock is therefore one number, which
 * run_test.sh passes with -D.
 */
#ifndef CYROS_TM4C123_BOARD_H
#define CYROS_TM4C123_BOARD_H

#include <stdint.h>

/* Two clocks, both from the board's 16 MHz crystal. 16 MHz is the crystal
 * itself, 80 MHz is the PLL, the part's maximum. Neither uses the 16 MHz
 * internal oscillator the part resets onto, which TI's erratum ELEC#05 allows
 * to be more than 3 per cent off (board_clock.c). */
#ifndef BOARD_SYSCLK_HZ
#define BOARD_SYSCLK_HZ 16000000u
#endif
#if BOARD_SYSCLK_HZ != 16000000u && BOARD_SYSCLK_HZ != 80000000u
#error "BOARD_SYSCLK_HZ must be 16000000 (the crystal) or 80000000 (the PLL)"
#endif

/* The console's rate, the same at both clocks so a terminal never has to be
 * told which clock the image was built for. The divisor is 1.086 at 16 MHz,
 * 0.6 per cent fast, and the ICDI's virtual COM port takes it. */
#ifndef BOARD_CONSOLE_BAUD
#define BOARD_CONSOLE_BAUD 921600u
#endif

/* Bring the clock tree to BOARD_SYSCLK_HZ. Called once from Reset_Handler,
 * before anything is constructed. */
void board_clock_init(void);

/* UART0 on PA0 and PA1, which the board wires to the ICDI's virtual COM port.
 * Called once from Reset_Handler, after the clock is final. */
void board_console_init(void);

/* Wait until everything written has left the wire. */
void board_console_drain(void);

/* The two hooks bench.hpp declares for C++: print to the console (console.c),
 * and end the image with a status (startup_tm4c123.c). */
void cyros_bench_write(char const* text);
__attribute__((noreturn)) void cyros_bench_exit(uint32_t code);

#endif /* CYROS_TM4C123_BOARD_H */
