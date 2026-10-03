/**
 * @file console.c
 * @brief The board's console: UART0, which the EK-TM4C123GXL wires to the
 *        ICDI's virtual COM port (PA0 RX, PA1 TX, port control value 1).
 *
 * This is where `cyros::bench::print` ends up on this board. Semihosting
 * through the ICDI halts the core for about 271 ms a call, invisibly to every
 * clock on the chip, so it carries only a panic (~/cyros-claude/
 * arm-port-notes.md 18g).
 *
 * A write is POLLED and blocks only while the 16-byte transmit FIFO is full,
 * for the same reason as on the U575: a transmit interrupt would land inside
 * the masking and timing the tests measure.
 *
 * On the host: `picocom -b 921600 /dev/ttyACM0`.
 */

#include "board.h"

#include <stdint.h>

#define REG(address) (*(volatile uint32_t*)(address))

#define SYSCTL_RCGCGPIO REG(0x400FE608u)
#define SYSCTL_RCGCUART REG(0x400FE618u)
#define SYSCTL_PRGPIO   REG(0x400FEA08u)
#define SYSCTL_PRUART   REG(0x400FEA18u)
#define GPIOA_AFSEL     REG(0x40004420u)
#define GPIOA_PUR       REG(0x40004510u)
#define GPIOA_DEN       REG(0x4000451Cu)
#define GPIOA_PCTL      REG(0x4000452Cu)
#define UART0_DR        REG(0x4000C000u)
#define UART0_FR        REG(0x4000C018u)
#define UART0_IBRD      REG(0x4000C024u)
#define UART0_FBRD      REG(0x4000C028u)
#define UART0_LCRH      REG(0x4000C02Cu)
#define UART0_CTL       REG(0x4000C030u)
#define UART0_CC        REG(0x4000CFC8u)

#define UART_FR_BUSY      (1u << 3)
#define UART_FR_TXFF      (1u << 5)
#define UART_LCRH_FEN     (1u << 4)
#define UART_LCRH_WLEN_8  (3u << 5)
#define UART_CTL_UARTEN   (1u << 0)
#define UART_CTL_TXE      (1u << 8)
#define UART_CTL_RXE      (1u << 9)
#define PINS_PA0_PA1      0x3u

/* Writes before init are dropped rather than spun on, as on the U575: an
 * unclocked UART never reports FIFO space. */
static int console_ready;

void board_console_init(void)
{
   SYSCTL_RCGCGPIO |= 1u << 0;    /* port A */
   SYSCTL_RCGCUART |= 1u << 0;    /* UART0 */
   while ((SYSCTL_PRGPIO & 1u) == 0u || (SYSCTL_PRUART & 1u) == 0u) {}

   /* The divisor in sixty-fourths: BRD = SYSCLK / (16 * baud), integer part
    * in IBRD and the fraction rounded to six bits in FBRD. LCRH is written
    * after both, because that write is what latches them. */
   uint32_t const div64 = (4u * BOARD_SYSCLK_HZ + BOARD_CONSOLE_BAUD / 2u) / BOARD_CONSOLE_BAUD;
   UART0_CTL  = 0u;
   UART0_IBRD = div64 >> 6;
   UART0_FBRD = div64 & 0x3Fu;
   UART0_LCRH = UART_LCRH_WLEN_8 | UART_LCRH_FEN;
   UART0_CC   = 0u;                                   /* the system clock */

   /* The UART is enabled BEFORE the pins are handed to it, so TX is already
    * idling high when it reaches the wire. RX gets a pull-up so an undriven
    * line does not read as a stream of breaks. Neither stops the ICDI seeing a
    * break at every reset, when PA1 is tristated until this runs: it reports
    * that as 0x00 bytes, which run_test.sh drops. */
   UART0_CTL  = UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE;
   GPIOA_PUR  |= 1u << 0;
   GPIOA_PCTL  = (GPIOA_PCTL & ~0xFFu) | 0x11u;
   GPIOA_AFSEL |= PINS_PA0_PA1;
   GPIOA_DEN   |= PINS_PA0_PA1;

   console_ready = 1;
}

void board_console_drain(void)
{
   if (!console_ready) { return; }
   while ((UART0_FR & UART_FR_BUSY) != 0u) {}
}

/* The board's half of bench.hpp's print. */
void cyros_bench_write(char const* text)
{
   if (!console_ready) { return; }
   for (; *text != '\0'; ++text) {
      while ((UART0_FR & UART_FR_TXFF) != 0u) {}
      UART0_DR = (uint8_t)*text;
   }
}
