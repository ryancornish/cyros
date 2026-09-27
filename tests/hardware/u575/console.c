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
#define USART1_TDR    REG(0x40013828u)

#define USART_CR1_UE      (1u << 0)
#define USART_CR1_RE      (1u << 2)
#define USART_CR1_TE      (1u << 3)
#define USART_CR1_OVER8   (1u << 15)
#define USART_CR1_FIFOEN  (1u << 29)
#define USART_ISR_TC      (1u << 6)
#define USART_ISR_TXFNF   (1u << 7)

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
