/**
 * @file console_rtt.c
 * @brief The board's console: SEGGER RTT, a ring buffer in SRAM that OpenOCD
 *        reads over SWD and serves on TCP (drive_test.py).
 *
 * The Pico 2 W has no UART wired to the Debug Probe here, and semihosting
 * halts the core for 100 ms a line (rp2350-notes.md 7a), so the tests' output
 * goes through RTT. A write blocks while the buffer is full, as a polled UART
 * does, so nothing is lost when the host is slow.
 */
#include "board.h"

#include <stdint.h>

#define RTT_BUF_SIZE 4096u

struct rtt_buf
{
   char const* name;
   char* buf;
   uint32_t size;
   volatile uint32_t wr;
   volatile uint32_t rd;
   uint32_t flags;
};

struct rtt_cb
{
   char id[16];
   int32_t max_up, max_down;
   struct rtt_buf up[1], down[1];
};

struct rtt_cb rtt_cb;
static char rtt_up[RTT_BUF_SIZE];

/* Read by the runner, which starts RTT once this is set. */
volatile uint32_t rtt_ready;

void board_console_init(void)
{
   rtt_cb.max_up = 1;
   rtt_cb.max_down = 1;
   rtt_cb.up[0] = (struct rtt_buf){ "Terminal", rtt_up, RTT_BUF_SIZE, 0u, 0u, 2u };
   rtt_cb.down[0] = (struct rtt_buf){ "Terminal", 0, 0u, 0u, 0u, 0u };
   /* The ID last and at run time, so the host finds only this copy. */
   static char const id[] = "SEGGER RTT";
   __asm__ volatile("fence rw, rw" ::: "memory");
   for (unsigned i = 0; i < sizeof id; ++i) { rtt_cb.id[i] = id[i]; }
   __asm__ volatile("fence rw, rw" ::: "memory");
   rtt_ready = 1u;
}

void cyros_bench_write(char const* text)
{
   struct rtt_buf* const b = &rtt_cb.up[0];
   while (*text != '\0') {
      uint32_t const wr = b->wr;
      uint32_t const next = (wr + 1u) % b->size;
      while (next == b->rd) {}
      b->buf[wr] = *text++;
      __asm__ volatile("fence rw, rw" ::: "memory");
      b->wr = next;
   }
}

bool board_console_drain(uint32_t spin_limit)
{
   for (uint32_t spins = 0; rtt_cb.up[0].rd != rtt_cb.up[0].wr; ) {
      if (++spins == spin_limit) { return false; }
   }
   return true;
}
