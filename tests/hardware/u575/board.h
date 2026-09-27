/**
 * @file board.h
 * @brief The NUCLEO-U575ZI-Q as the tests see it: its clock and its console.
 *
 * Every clock-derived setting on this board is computed from BOARD_SYSCLK_HZ:
 * the SysTick rate cyros is told (board_clock.c) and the console's baud
 * divisor (console.c). Changing the clock is therefore one number, which
 * build.sh and run_test.sh pass with -D. A second copy of the clock anywhere
 * else is how "every duration is wrong by a factor of forty" happens.
 */
#ifndef CYROS_U575_BOARD_H
#define CYROS_U575_BOARD_H

#include <stdint.h>

/* Two clocks, and only two, because each is a tested configuration rather than
 * a formula. 4 MHz is the reset clock, MSIS with nothing configured. 160 MHz is
 * PLL1 fed from MSIS, the part's maximum, which needs voltage range 1, the EPOD
 * booster and four flash wait states (board_clock.c). */
#ifndef BOARD_SYSCLK_HZ
#define BOARD_SYSCLK_HZ 4000000u
#endif
#if BOARD_SYSCLK_HZ != 4000000u && BOARD_SYSCLK_HZ != 160000000u
#error "BOARD_SYSCLK_HZ must be 4000000 (the reset clock) or 160000000 (PLL1)"
#endif

/* The console's rate, the same at every clock so a terminal never has to be
 * told which clock the image was built for. 500 kbaud is the fastest the 4 MHz
 * reset clock can produce (oversampling by 8), and the ST-LINK accepts it. */
#ifndef BOARD_CONSOLE_BAUD
#define BOARD_CONSOLE_BAUD 500000u
#endif

/* Bring the clock tree to BOARD_SYSCLK_HZ. Called once from Reset_Handler,
 * before anything is constructed. */
void board_clock_init(void);

/* USART1 on PA9 and PA10, which the board wires to the ST-LINK's virtual COM
 * port. Called once from Reset_Handler, after the clock is final. */
void board_console_init(void);

/* Wait until everything written has left the wire. */
void board_console_drain(void);

/* The two hooks bench.hpp declares for C++: print to the console (console.c),
 * and end the image with a status (startup_stm32u575.c). */
void cyros_bench_write(char const* text);
__attribute__((noreturn)) void cyros_bench_exit(uint32_t code);

#endif /* CYROS_U575_BOARD_H */
