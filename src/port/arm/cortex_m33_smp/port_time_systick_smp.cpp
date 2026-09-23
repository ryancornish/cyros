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
 * That single fact decides the whole design.
 *
 *
 * One core owns time
 * ==================
 * Core 0 programs and services SysTick. Secondary cores do not run a timer at
 * all. Any other arrangement gives the system as many clocks as it has cores,
 * and the kernel's contract is for ONE monotonic time.
 *
 *
 * Why periodic and not tickless
 * =============================
 * `cyros_port_time_now` must answer on ANY core, because any core can schedule.
 * In periodic mode the answer is a software counter that the owning core's ISR
 * increments, which is ordinary shared memory and reads correctly from either
 * core.
 *
 * Tickless cannot do that. Its answer is a software base plus THE HARDWARE
 * COUNTER, and on a secondary core that counter belongs to a SysTick which was
 * never started. The read would not fail, it would return a plausible wrong
 * number, which is the worst failure mode available. So tickless is refused
 * here rather than silently supported, and the refusal names the reason.
 *
 * Making tickless work on SMP is not a small change. It needs a time source
 * that is genuinely shared between cores, which on a real SSE-200 part means a
 * system timer outside the core rather than SysTick. That is an MCU decision,
 * which is exactly why the time contract sits in port_mcu.h.
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

/* The core that owns the clock. Nothing makes this core 0 architecturally, it
 * is simply the core that runs bring-up and therefore the one that calls
 * setup. */
constexpr std::uint32_t time_core = 0u;

/* Incremented by the owning core's SysTick ISR and read by any core. Plain
 * volatile rather than an atomic: it is written by exactly one core and a
 * 32-bit aligned store is single-copy atomic on ARMv8-M, so a reader sees a
 * whole value or the previous whole value, never a torn one.
 *
 * Kept 32-bit deliberately. A 64-bit counter would be two stores and a reader
 * on the other core COULD see half of each, which is precisely the tearing
 * that single-copy atomicity rules out for one word. The 64-bit value the
 * contract wants is assembled from this plus an epoch below. */
volatile std::uint32_t tick_low = 0u;

/* Extends tick_low past its wrap. Written only by the owning core, in the same
 * ISR, and only when tick_low wraps to zero. */
volatile std::uint32_t tick_high = 0u;

std::uint32_t configured_tick_hz = 0u;

cyros_port_isr_handler_t isr_handler = nullptr;
void* isr_argument = nullptr;

} // namespace


void cyros_port_time_setup(std::uint32_t tick_hz)
{
   CYROS_ASSERT_OP(cyros_port_get_core_id(), ==, time_core);

   /* The refusal this file exists to make. tick_hz == 0 is the contract's
    * request for tickless, and tickless needs a counter both cores can read.
    * SysTick is not one. */
   CYROS_ASSERT_OP(tick_hz, >, 0u);

   cortex_m::reg(cortex_m::systick_ctrl) = 0u;   /* stop before reprogramming */

   std::uint32_t const clock_hz = cyros_port_systick_clock_hz();
   CYROS_ASSERT_OP(clock_hz, >, 0u);
   CYROS_ASSERT_OP(tick_hz, <=, clock_hz);

   std::uint32_t const reload = (clock_hz / tick_hz) - 1u;
   CYROS_ASSERT_OP(reload, <=, cortex_m::systick_reload_max);

   configured_tick_hz = tick_hz;
   tick_low  = 0u;
   tick_high = 0u;

   cortex_m::reg(cortex_m::systick_load) = reload;
   cortex_m::reg(cortex_m::systick_val)  = 0u;
   cortex_m::reg(cortex_m::systick_ctrl) =
      cortex_m::systick_ctrl_clksource
      | cortex_m::systick_ctrl_tickint
      | cortex_m::systick_ctrl_enable;

   cortex_m::dsb();
   cortex_m::isb();
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
   CYROS_ASSERT_OP(cyros_port_get_core_id(), ==, time_core);
   cortex_m::reg(cortex_m::systick_ctrl) =
      cortex_m::reg(cortex_m::systick_ctrl) | cortex_m::systick_ctrl_tickint;
}

void cyros_port_time_irq_disable(void)
{
   CYROS_ASSERT_OP(cyros_port_get_core_id(), ==, time_core);
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
   /* The kernel's SMP time policy has non-time cores hand work to the core
    * that owns the clock. The doorbell that carries a reschedule carries this
    * too: both end in a pended reschedule on the target, and the receiving
    * core discovers what there is to do from shared state rather than from the
    * signal. A second doorbell would add a message type the kernel never
    * reads. */
   cyros_port_send_reschedule_ipi(core_id);
}

/**
 * @brief SysTick on the owning core. Routed from the application's table.
 *
 * Only core 0 ever enables SysTick, so only core 0 ever arrives here, and the
 * counter writes below are therefore single-writer. The assert states that
 * rather than trusting it.
 */
extern "C" void SysTick_Handler(void)
{
   CYROS_ASSERT_OP(cyros_port_get_core_id(), ==, time_core);

   std::uint32_t const next = tick_low + 1u;
   tick_low = next;
   if (next == 0u) {
      tick_high = tick_high + 1u;
   }

   if (isr_handler != nullptr) {
      isr_handler(isr_argument);
   }
}
