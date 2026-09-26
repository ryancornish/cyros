/**
 * @file port_time_systick_smp.cpp
 * @brief The time half of the MCU contract, for a dual-M33 SSE-200.
 *
 * Separate from the single-core SysTick driver rather than shared with it, and
 * the reason is a hardware fact worth stating plainly:
 *
 *   SYSTICK IS CORE-PRIVATE. Each M33 has its own, in its own System Control
 *   Space, at the same address. Two cores reading 0xE000E018 read two
 *   different counters.
 *
 * The design follows the one every per-core-tick SMP kernel uses: a shared
 * counter for `now()` and a per-core tick for deadlines. The linux_preempt port
 * does the same thing with a POSIX timer per core over one CLOCK_MONOTONIC.
 *
 *
 * Every core ticks, only core 0 counts
 * ====================================
 * EVERY core that calls `time::start()` runs its own SysTick at the same rate,
 * and that tick services its own core's timetable, which is what the time
 * drivers expect. A timer scheduled on core 1 is serviced by core 1's tick.
 *
 * Only core 0's ISR advances the counter behind `now()`. The system has ONE
 * monotonic time, and two independently incremented counters would be two
 * clocks. Every core reads that one counter.
 *
 * Until 2026-09-25 this target gave core 0 the only tick, on the reasoning that
 * SysTick's counter is core-private. That is true and beside the point: only
 * `now()` needs a shared counter. With core 1 tickless in effect, a timer it
 * scheduled was accepted and never fired, so every timed wait on core 1 hung.
 * Zephyr rejected the same design for the same reason (PR #119504).
 * `arm-port-notes.md` 16j.
 *
 *
 * Why periodic and not tickless
 * =============================
 * `cyros_port_time_now` must answer on ANY core. In periodic mode the answer is
 * a software counter core 0's ISR increments, which is ordinary shared memory
 * and reads correctly from either core.
 *
 * Tickless cannot do that. Its answer is a software base plus THE HARDWARE
 * COUNTER, and each core's hardware counter is its own. The read would not
 * fail, it would return a plausible wrong number, which is the worst failure
 * mode available. So tickless is refused here rather than silently supported.
 * Making it work needs a counter genuinely shared between cores, which on a
 * real SSE-200 part means a system timer outside the core. That is an MCU
 * decision, which is exactly why the time contract sits in port_mcu.h.
 *
 *
 * Teardown is not a target feature
 * =================================
 * A core's SysTick can only be reached from that core, and the time contract's
 * teardown runs once, on one core. So `cyros_port_time_teardown()` stops the
 * CALLING core's SysTick and forgets the handler, which makes every other
 * core's tick inert: it still fires, and does nothing. That is enough because on
 * this target `kernel::start()` never returns, so nothing finalises time (Ryan,
 * 2026-09-25: teardown need not be a target feature).
 */

#include <cyros/port/port_mcu.h>

#include "cortex_m.hpp"

#include <cstdint>

namespace cortex_m = cyros::port::cortex_m;

/**
 * @brief What drives SysTick, supplied by the board.
 *
 * Declared by the port and deliberately given no default, so that every image
 * has to state it. See the single-core driver for the full argument.
 */
extern "C" std::uint32_t cyros_port_systick_clock_hz(void);

namespace
{

/* The core whose tick advances the shared counter. Nothing makes this core 0
 * architecturally, it is simply the core that runs bring-up. Every core ticks,
 * only this one counts. */
constexpr std::uint32_t time_core = 0u;

/* Incremented by the time core's SysTick ISR and read by any core. Plain
 * volatile rather than an atomic: it is written by exactly one core and a
 * 32-bit aligned store is single-copy atomic on ARMv8-M, so a reader sees a
 * whole value or the previous whole value, never a torn one.
 *
 * Kept 32-bit deliberately. A 64-bit counter would be two stores and a reader
 * on the other core COULD see half of each, which is precisely the tearing
 * that single-copy atomicity rules out for one word. The 64-bit value the
 * contract wants is assembled from this plus an epoch below. */
volatile std::uint32_t tick_low = 0u;

/* Extends tick_low past its wrap. Written only by the time core, in the same
 * ISR, and only when tick_low wraps to zero. */
volatile std::uint32_t tick_high = 0u;

std::uint32_t configured_tick_hz = 0u;

cyros_port_isr_handler_t isr_handler = nullptr;
void* isr_argument = nullptr;

} // namespace


void cyros_port_time_setup(std::uint32_t tick_hz)
{
   /* Per core, from each core's time::start(). Programs THIS core's SysTick,
    * which is the only one this core can reach. */

   /* tick_hz == 0 is the contract's request for tickless, and tickless needs a
    * hardware counter both cores can read. SysTick is not one. */
   CYROS_ASSERT_OP(tick_hz, >, 0u);

   cortex_m::reg(cortex_m::systick_ctrl) = 0u;   /* stop before reprogramming */

   std::uint32_t const clock_hz = cyros_port_systick_clock_hz();
   CYROS_ASSERT_OP(clock_hz, >, 0u);
   CYROS_ASSERT_OP(tick_hz, <=, clock_hz);

   std::uint32_t const reload = (clock_hz / tick_hz) - 1u;
   CYROS_ASSERT_OP(reload, <=, cortex_m::systick_reload_max);

   /* Every core's tick runs at the rate every core's deadlines are measured
    * in. Written by each core with the same value, so the race is benign. A
    * core asking for a different rate would make one core's ticks another's
    * idea of time, so that is refused. */
   CYROS_ASSERT(configured_tick_hz == 0u || configured_tick_hz == tick_hz);
   configured_tick_hz = tick_hz;

   /* The counter belongs to the time core alone. Resetting it from another
    * core would race that core's ISR. */
   if (cyros_port_get_core_id() == time_core) {
      tick_low  = 0u;
      tick_high = 0u;
   }

   cortex_m::reg(cortex_m::systick_load) = reload;
   cortex_m::reg(cortex_m::systick_val)  = 0u;
   cortex_m::reg(cortex_m::systick_ctrl) =
      cortex_m::systick_ctrl_clksource
      | cortex_m::systick_ctrl_tickint
      | cortex_m::systick_ctrl_enable;

   cortex_m::dsb();
   cortex_m::isb();
}

void cyros_port_time_teardown(void)
{
   /* Stops the CALLING core's SysTick, the only one it can reach, and forgets
    * the handler, which leaves every other core's tick firing and inert. Not a
    * target feature, see the file comment. */
   cyros_mask_token_t const token = cyros_port_irq_save();

   cortex_m::reg(cortex_m::systick_ctrl) = 0u;
   /* A tick that landed while masked would otherwise still run. */
   cortex_m::reg(cortex_m::scb_icsr) = cortex_m::icsr_pendstclr;

   isr_handler  = nullptr;
   isr_argument = nullptr;
   configured_tick_hz = 0u;

   /* Both writes complete before interrupts can be taken again. */
   cortex_m::dsb();
   cortex_m::isb();

   cyros_port_irq_restore(token);
}

std::uint64_t cyros_port_time_now(void)
{
   /* Readable from ANY core, which is the requirement that shaped this file.
    *
    * Read high, low, high again. If the epoch moved between the two high
    * reads, the low value belongs to one side of a wrap and there is no way to
    * tell which, so take the second pair. The owning core cannot wrap twice
    * inside this sequence: that would need 2^32 ticks. */
   for (;;) {
      std::uint32_t const high_before = tick_high;
      std::uint32_t const low         = tick_low;
      std::uint32_t const high_after  = tick_high;

      if (high_before == high_after) {
         return (static_cast<std::uint64_t>(high_after) << 32) | low;
      }
   }
}

std::uint64_t cyros_port_time_freq_hz(void)
{
   /* A port tick IS a SysTick interrupt in periodic mode, so the rate is the
    * configured tick rate. */
   return configured_tick_hz;
}

void cyros_port_time_reset(std::uint64_t time)
{
   CYROS_ASSERT_OP(cyros_port_get_core_id(), ==, time_core);

   cyros_mask_token_t const token = cyros_port_irq_save();

   tick_high = static_cast<std::uint32_t>(time >> 32);
   tick_low  = static_cast<std::uint32_t>(time);

   cortex_m::reg(cortex_m::scb_icsr) = cortex_m::icsr_pendstclr;

   cyros_port_irq_restore(token);
}

void cyros_port_time_register_isr_handler(cyros_port_isr_handler_t handler, void* arg)
{
   cyros_mask_token_t const token = cyros_port_irq_save();
   isr_handler  = handler;
   isr_argument = arg;
   cyros_port_irq_restore(token);
}

void cyros_port_time_irq_enable(void)
{
   /* The calling core's own SysTick, like everything that touches the SCS. */
   cortex_m::reg(cortex_m::systick_ctrl) =
      cortex_m::reg(cortex_m::systick_ctrl) | cortex_m::systick_ctrl_tickint;
}

void cyros_port_time_irq_disable(void)
{
   cortex_m::reg(cortex_m::systick_ctrl) =
      cortex_m::reg(cortex_m::systick_ctrl) & ~cortex_m::systick_ctrl_tickint;
}

void cyros_port_time_arm(std::uint64_t deadline)
{
   /* Tickless only, and setup refuses tickless. Reaching here means the time
    * driver asked for a one-shot on a periodic-only target. */
   (void)deadline;
   CYROS_ASSERT(false);
}

void cyros_port_time_disarm(void)
{
   CYROS_ASSERT(false);
}

void cyros_port_send_time_ipi(std::uint32_t core_id)
{
   /* Every core ticks for itself, so no core has time work to hand to another,
    * and nothing in the kernel calls this. Kept meaningful rather than empty:
    * the doorbell that carries a reschedule is the closest thing this target
    * has to a time interrupt on another core. */
   cyros_port_send_reschedule_ipi(core_id);
}

/**
 * @brief Every core's SysTick. Routed from the application's table, which both
 *        cores share.
 *
 * Runs on whichever core's tick fired, and hands that core's own timetable to
 * the driver through the handler. Only the time core advances the counter, so
 * the counter writes stay single-writer.
 */
extern "C" void SysTick_Handler(void)
{
   if (cyros_port_get_core_id() == time_core) {
      std::uint32_t const next = tick_low + 1u;
      tick_low = next;
      if (next == 0u) {
         tick_high = tick_high + 1u;
      }
   }

   if (isr_handler != nullptr) {
      isr_handler(isr_argument);
   }
}
