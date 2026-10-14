/**
 * @file console.c
 * @brief The board's console: USART1, which the NUCLEO wires to the ST-LINK's
 *        virtual COM port (PA9 TX, PA10 RX, alternate function 7).
 *
 * This is where `cyros::bench::print` ends up on this board. It replaced
 * semihosting, which halts the core for about 130 ms a call while OpenOCD
 * services it, invisibly to every clock on the chip (arm-port-notes 15i).
 *
 * A write is POLLED and blocks only while the 8-byte transmit FIFO is full, so
 * a short print costs about a hundred cycles and a long one waits on the wire,
 * 20 microseconds a character at 500 kbaud. Polled rather than interrupt or
 * DMA driven on purpose: a transmit interrupt would land inside the masking and
 * timing the tests measure. Nothing timing-sensitive should print either way.
 *
 * It also carries bench.hpp's host time. OpenOCD answers neither of the two
 * elapsed-time semihosting calls, so the board asks run_test.sh instead, the
 * LEON3 board's protocol: it writes "@@cyros-host tickfreq" or
 * "@@cyros-host elapsed" and reads back a decimal line on RX.
 *
 * On the host: `picocom -b 500000 /dev/ttyACM0`, or console.sh.
 */

#include "board.h"

#include <stdint.h>

#define REG(address) (*(volatile uint32_t*)(address))

#define RCC_AHB2ENR1  REG(0x46020C8Cu)
#define RCC_APB2ENR   REG(0x46020CA4u)
#define GPIOA_MODER   REG(0x42020000u)
#define GPIOA_PUPDR   REG(0x4202000Cu)
#define GPIOA_AFRH    REG(0x42020024u)
#define USART1_CR1    REG(0x40013800u)
#define USART1_BRR    REG(0x4001380Cu)
#define USART1_ISR    REG(0x4001381Cu)
#define USART1_ICR    REG(0x40013820u)
#define USART1_RDR    REG(0x40013824u)
#define USART1_TDR    REG(0x40013828u)

#define USART_CR1_UE      (1u << 0)
#define USART_CR1_RE      (1u << 2)
#define USART_CR1_TE      (1u << 3)
#define USART_CR1_OVER8   (1u << 15)
#define USART_CR1_FIFOEN  (1u << 29)
#define USART_ISR_RXFNE   (1u << 5)
#define USART_ISR_TC      (1u << 6)
#define USART_ISR_TXFNF   (1u << 7)
#define USART_ICR_ORECF   (1u << 3)

/* Writes before init are dropped rather than spun on: an unclocked USART never
 * reports FIFO space, so a fault in the clock bring-up would otherwise hang
 * silently inside its own report. */
static int console_ready;

void board_console_init(void)
{
   RCC_AHB2ENR1 |= 1u << 0;    /* GPIOA */
   RCC_APB2ENR  |= 1u << 14;   /* USART1 */
   (void)RCC_APB2ENR;          /* the enable lands before the first access */

   /* Oversampling by 16 wherever the clock allows it, since it tolerates more
    * clock error. By 8 only when 16 cannot reach the rate, which at 4 MHz means
    * anything above 250 kbaud. */
   uint32_t cr1 = USART_CR1_FIFOEN | USART_CR1_TE | USART_CR1_RE;
   if (BOARD_SYSCLK_HZ / 16u >= BOARD_CONSOLE_BAUD) {
      USART1_BRR = (BOARD_SYSCLK_HZ + BOARD_CONSOLE_BAUD / 2u) / BOARD_CONSOLE_BAUD;
   } else {
      uint32_t const div = (2u * BOARD_SYSCLK_HZ + BOARD_CONSOLE_BAUD / 2u) / BOARD_CONSOLE_BAUD;
      USART1_BRR = (div & ~0xFu) | ((div & 0xFu) >> 1);
      cr1 |= USART_CR1_OVER8;
   }

   /* The USART is enabled BEFORE the pins are handed to it, so TX is already
    * idling high when it reaches the wire. The other order let PA9 float for a
    * moment and put a garbage byte ahead of the first line. RX gets a pull-up
    * so an unconnected line does not read as a stream of breaks. */
   USART1_CR1 = cr1;
   USART1_CR1 = cr1 | USART_CR1_UE;
   GPIOA_PUPDR = (GPIOA_PUPDR & ~(3u << 20)) | (1u << 20);
   GPIOA_AFRH  = (GPIOA_AFRH & ~(0xFFu << 4)) | (0x77u << 4);
   GPIOA_MODER = (GPIOA_MODER & ~(0xFu << 18)) | (0xAu << 18);

   console_ready = 1;
}

void board_console_drain(void)
{
   if (!console_ready) { return; }
   while ((USART1_ISR & USART_ISR_TC) == 0u) {}
}

/* The board's half of bench.hpp's print. */
void cyros_bench_write(char const* text)
{
   if (!console_ready) { return; }
   for (; *text != '\0'; ++text) {
      while ((USART1_ISR & USART_ISR_TXFNF) == 0u) {}
      USART1_TDR = (uint8_t)*text;
   }
}

/* One decimal line from run_test.sh, which answers in about a millisecond
 * (40 at worst, seen once). Bounded, so an image run with no runner attached
 * (console.sh) gives up and its checks skip, after 1.6 s at 160 MHz and 5 s
 * at 4 MHz (measured). */
static int read_reply(uint64_t* value)
{
   uint64_t number = 0u;
   int digits = 0;
   for (uint32_t pass = 0u; pass < BOARD_SYSCLK_HZ / 8u; ++pass) {
      if ((USART1_ISR & USART_ISR_RXFNE) == 0u) {
         continue;
      }
      char const c = (char)USART1_RDR;
      if (c >= '0' && c <= '9') {
         number = number * 10u + (uint64_t)(c - '0');
         ++digits;
      } else if (c == '\n' && digits > 0) {
         *value = number;
         return 1;
      }
   }
   return 0;
}

/* The board's half of bench.hpp's elapsed-time calls (its operation numbers
 * are semihosting's). Anything the runner left unread, and an overrun that
 * would stop reception, is dropped first, so a reply is always this
 * request's. */
long cyros_bench_semihost(long op, void volatile* arg)
{
   if (!console_ready) { return -1; }
   USART1_ICR = USART_ICR_ORECF;
   while ((USART1_ISR & USART_ISR_RXFNE) != 0u) {
      uint32_t const stale = USART1_RDR;
      (void)stale;
   }

   uint64_t value = 0u;
   if (op == 0x31) {   /* SYS_TICKFREQ */
      cyros_bench_write("@@cyros-host tickfreq\n");
      return read_reply(&value) ? (long)value : -1;
   }
   if (op == 0x30) {   /* SYS_ELAPSED, into a two-word block, low word first */
      cyros_bench_write("@@cyros-host elapsed\n");
      if (!read_reply(&value)) {
         return -1;
      }
      volatile uint32_t* const block = (volatile uint32_t*)arg;
      block[0] = (uint32_t)value;
      block[1] = (uint32_t)(value >> 32);
      return 0;
   }
   return -1;
}
