/**
 * @file port_time_mtime.cpp
 * @brief The time half of port_mcu.h on an MTIME timer: one 64-bit counter
 *        every core reads, and a compare register per core. Periodic and
 *        tickless, on any number of cores, on either ISA.
 *
 * Compiled by each target that has one, as linux/common is by each Linux port:
 * QEMU's RISC-V virt, the RP2350's Hazard3 cores and the RP2350's Cortex-M33s.
 * The target says where the registers are and how its core takes the
 * interrupt (mtime.hpp), and the board says how fast MTIME runs.
 *
 *
 * WHY THIS IS SMALLER THAN THE SYSTICK DRIVER
 * ==========================================
 * SysTick is a 24-bit down counter with no compare register, so the ARM
 * driver extends it in software, folds elapsed cycles across every restart and
 * cancels a pending wrap when it re-arms (port_time_systick.cpp). MTIME is a
 * 64-bit up counter that never wraps in practice and has a COMPARE register.
 * Tickless is therefore `now()` = MTIME and `arm()` = one compare write, and
 * nothing is ever folded or lost.
 *
 * And it is SHARED by every core, which is what the SSE-200 SMP target lacked
 * and why that target refuses tickless (arm-port-notes.md 16f). Here a core
 * reads the same counter as every other, so tickless works on every core.
 *
 *
 * TWO MODES
 * =========
 * `setup(tick_hz)` with tick_hz > 0 is PERIODIC: each core's MTIMECMP is
 * stepped one period at a time, a port tick IS a period, and `now()` counts
 * them. Only the time core (0) counts, as on the SMP ARM target, so the count
 * has one writer, and every core reads it.
 *
 * `setup(0)` is TICKLESS: a port tick is one MTIME count, `now()` is MTIME,
 * and a core's compare is armed only for a deadline.
 *
 * In both modes the timer interrupt runs from setup to teardown, and
 * irq_enable/disable gate only the DELIVERY of the driver's callback, per
 * core. In periodic mode that keeps the count moving while delivery is off.
 *
 *
 * 64-BIT VALUES ON A 32-BIT CORE
 * =============================
 * `std::atomic<std::uint64_t>` would need libatomic on both ISAs (no 64-bit
 * LR/SC, no LDREXD on ARMv8-M Mainline), so every 64-bit value below is plain
 * and accessed masked or in the ISR, and the periodic count is published as
 * two words read high-low-high.
 */

#include <cyros/port/port_mcu.h>

#include "mtime.hpp"

#include <cstdint>

namespace mtime = cyros::port::mtime;

namespace
{

enum class timer_mode : std::uint8_t { none, periodic, tickless };

constexpr std::uint64_t never = ~std::uint64_t{0};

/* The core whose interrupt advances the periodic count. Nothing makes it core
 * 0 architecturally, it is the core that runs bring-up. */
constexpr std::uint32_t time_core = 0u;

struct per_core
{
   std::uint64_t next_tick;   /* periodic: the compare value now programmed */
   std::uint64_t armed;       /* tickless: the earliest live deadline        */
   bool          delivery;    /* the driver's irq_enable                     */
};

per_core cores[CYROS_PORT_CORE_COUNT] = {};

timer_mode    active_mode        = timer_mode::none;
std::uint64_t period             = 0u;   /* MTIME counts per periodic tick */
std::uint32_t configured_tick_hz = 0u;   /* port ticks per second          */

/* The periodic count, written only by the time core, in its ISR or in its
 * setup. Two words so any core can read them without a lock. */
volatile std::uint32_t tick_low  = 0u;
volatile std::uint32_t tick_high = 0u;

cyros_port_isr_handler_t isr_handler  = nullptr;
void*                    isr_argument = nullptr;

inline std::uint32_t this_core() noexcept
{
   if constexpr (CYROS_PORT_CORE_COUNT == 1) {
      return 0u;
   } else {
      return cyros_port_get_core_id();
   }
}

std::uint64_t periodic_count() noexcept
{
   /* High, low, high: if the high word moved, the low one belongs to one side
    * of a carry and the pair is retried. */
   while (true) {
      std::uint32_t const high_before = tick_high;
      std::uint32_t const low         = tick_low;
      std::uint32_t const high_after  = tick_high;
      if (high_before == high_after) {
         return (static_cast<std::uint64_t>(high_after) << 32) | low;
      }
   }
}

void advance_count(std::uint32_t ticks) noexcept
{
   std::uint32_t const low  = tick_low;
   std::uint32_t const next = low + ticks;
   if (next < low) {
      tick_high = tick_high + 1u;
   }
   tick_low = next;
}

} // namespace


void cyros_port_time_setup(std::uint32_t tick_hz)
{
   cyros_mask_token_t const token = cyros_port_irq_save();

   std::uint32_t const core = this_core();
   std::uint32_t const clock_hz = cyros_port_mtime_clock_hz();
   CYROS_ASSERT_OP(clock_hz, >, 0u);

   /* Every core runs at the rate every core's deadlines are measured in, so a
    * second core asking for a different mode or rate is refused. */
   timer_mode const wanted = tick_hz == 0u ? timer_mode::tickless : timer_mode::periodic;
   std::uint32_t const wanted_hz = tick_hz == 0u ? clock_hz : tick_hz;
   CYROS_ASSERT(active_mode == timer_mode::none || active_mode == wanted);
   CYROS_ASSERT(configured_tick_hz == 0u || configured_tick_hz == wanted_hz);
   active_mode        = wanted;
   configured_tick_hz = wanted_hz;

   per_core& state = cores[core];
   state.delivery = false;
   state.armed    = never;

   if (wanted == timer_mode::periodic) {
      CYROS_ASSERT_OP(tick_hz, <=, clock_hz);
      period = clock_hz / tick_hz;
      if (core == time_core) {
         tick_low  = 0u;
         tick_high = 0u;
      }
      state.next_tick = mtime::read() + period;
      mtime::compare_write(core, state.next_tick);
   }
   else {
      mtime::compare_write(core, never);
   }

   mtime::interrupt_enable();

   cyros_port_irq_restore(token);
}

void cyros_port_time_teardown(void)
{
   cyros_mask_token_t const token = cyros_port_irq_save();

   /* The calling core's timer, the only compare it owns. Every other core's
    * interrupt keeps firing and finds no handler, which is enough because
    * nothing finalises time on a running multicore target. */
   mtime::interrupt_disable();
   mtime::compare_write(this_core(), never);

   for (per_core& state : cores) {
      state.delivery = false;
      state.armed    = never;
   }
   active_mode        = timer_mode::none;
   configured_tick_hz = 0u;
   period             = 0u;
   isr_handler        = nullptr;
   isr_argument       = nullptr;

   cyros_port_irq_restore(token);
}

std::uint64_t cyros_port_time_now(void)
{
   /* Either answer is readable from any core: MTIME is shared, and the
    * periodic count is published for lock-free reading. */
   return active_mode == timer_mode::tickless ? mtime::read() : periodic_count();
}

std::uint64_t cyros_port_time_freq_hz(void)
{
   return configured_tick_hz;
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
   cores[this_core()].delivery = true;
}

void cyros_port_time_irq_disable(void)
{
   cores[this_core()].delivery = false;
}

void cyros_port_time_arm(std::uint64_t deadline)
{
   CYROS_ASSERT(active_mode == timer_mode::tickless);

   cyros_mask_token_t const token = cyros_port_irq_save();

   std::uint32_t const core = this_core();
   per_core& state = cores[core];
   /* "the port must ensure the earliest deadline is honored". A deadline
    * already passed raises the interrupt at once, which is what it should. */
   if (deadline < state.armed) {
      state.armed = deadline;
      mtime::compare_write(core, deadline);
   }

   cyros_port_irq_restore(token);
}

void cyros_port_time_disarm(void)
{
   CYROS_ASSERT(active_mode == timer_mode::tickless);

   cyros_mask_token_t const token = cyros_port_irq_save();

   std::uint32_t const core = this_core();
   cores[core].armed = never;
   mtime::compare_write(core, never);

   cyros_port_irq_restore(token);
}


namespace cyros::port::mtime
{

/**
 * @brief The compare interrupt, on whichever core's compare fired.
 *
 * Called from the target's handler with interrupts masked, on both ISAs. The
 * RISC-V trap masks by itself. An ARM handler masks for the call, because a
 * device IRQ above the timer's priority could otherwise arm a deadline between
 * this reading `armed` and writing it back, and the earlier deadline would be
 * lost. A reschedule the callback asks for is taken after the handler returns.
 */
void interrupt() noexcept
{
   std::uint32_t const core = this_core();
   per_core& state = cores[core];

   if (active_mode == timer_mode::periodic) {
      /* Step the compare past now. More than one step means ticks were missed
       * (a long masked region), and the count takes all of them, so now()
       * stays true to MTIME. The callback runs once. */
      std::uint64_t const now = read();
      std::uint32_t ticks = 0u;
      while (state.next_tick <= now) {
         state.next_tick += period;
         ++ticks;
      }
      compare_write(core, state.next_tick);
      if (core == time_core) {
         advance_count(ticks);
      }
      if (state.delivery && isr_handler != nullptr) {
         isr_handler(isr_argument);
      }
      return;
   }

   if (active_mode == timer_mode::tickless) {
      bool const reached = state.armed != never && read() >= state.armed;
      if (reached) {
         state.armed = never;
      }
      /* Re-armed from the record, which a spurious or early interrupt leaves
       * where it was. Before the callback, which may arm again. */
      compare_write(core, state.armed);
      if (reached && state.delivery && isr_handler != nullptr) {
         isr_handler(isr_argument);
      }
      return;
   }

   /* Torn down: silence this core's compare for good. */
   compare_write(core, never);
}

} // namespace cyros::port::mtime
