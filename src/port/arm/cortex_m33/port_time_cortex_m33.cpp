/**
 * @file port_time_cortex_m33.cpp
 * @brief SysTick time source for the Cortex-M33 port.
 *
 * PERIODIC ONLY, DELIBERATELY. cyros_port_time_setup(0) selects tickless mode,
 * and this port panics there rather than approximating it. The reason is that
 * SysTick is a 24-bit down-counter with no capture and no second comparator, so
 * a tickless implementation has to synthesise a free-running clock by
 * accumulating reload values across one-shots. That is doable and it is also
 * exactly the kind of code that is subtly wrong at the wrap boundary and passes
 * every short test. It wants a real timer peripheral behind it (the STM32U575's
 * LPTIM, or a CMSDK timer on the bench), which is board-specific and therefore
 * a later, separate piece of work.
 *
 * Choosing to panic rather than approximate: a port that silently returns a
 * plausible-but-wrong monotonic clock would poison every timing assertion above
 * it, and the failure would look like a kernel bug.
 *
 *
 * The 64-bit read problem, which is real on this target
 * =====================================================
 * MEASURED 2026-09-20: std::atomic<std::uint64_t>::is_always_lock_free is FALSE
 * on ARMv8-M, because the profile has no LDREXD. The Linux ports' time sources
 * hold `std::atomic<uint64_t> now` and that is fine on x86-64; copying it here
 * would silently emit a libatomic call taking an address-hashed lock, inside an
 * ISR, on the hottest path in the time layer.
 *
 * So the tick counter is a plain uint64_t and every reader masks interrupts
 * across the read. One masked load of two words is cheaper than a lock, and it
 * is correct by construction rather than by hoping the compiler picked a
 * lock-free path.
 */

#include <cyros/port/port.h>
#include <cyros/port/port_time.h>

#include "cortex_m.hpp"

#include <cstdint>

namespace cortex_m = cyros::port::cortex_m;

/**
 * @brief Frequency feeding SysTick, in Hz.
 *
 * Weak so an application can define its own without a port rebuild, which is
 * how a board says what its clock tree produces. The STM32U575 runs to 160 MHz;
 * QEMU's mps2-an505 is its own thing and has not been measured yet.
 *
 * It is only used to derive the SysTick reload value. Nothing in the kernel
 * reads it, and cyros_port_time_freq_hz reports the TICK rate, not this.
 */
extern "C" [[gnu::weak]] std::uint32_t const cyros_port_systick_clock_hz = 25'000'000u;

namespace
{

/* Plain, not atomic. See the file comment: a 64-bit atomic is not lock-free
 * here, and every access below is already inside a masked region. */
std::uint64_t tick_count = 0;

std::uint64_t configured_tick_hz = 0;

cyros_port_isr_handler_t isr_handler = nullptr;
void*                    isr_argument = nullptr;

} // namespace


/* ============================================================================
 * Time Driver Port Contract
 * ========================================================================= */

void cyros_port_time_setup(std::uint32_t tick_hz)
{
   if (tick_hz == 0u) {
      cyros_port_system_error(
         0, 0, "cortex_m33 port: tickless mode is not implemented, see file comment", 0);
   }

   std::uint32_t const reload = (cyros_port_systick_clock_hz / tick_hz) - 1u;

   /* SysTick's reload field is 24 bits. Without this check a tick_hz too low
    * for the clock truncates and produces a tick rate that is wrong by a
    * factor of anything, reported as if it were correct. */
   CYROS_ASSERT_OP(reload, <=, cortex_m::systick_reload_max);
   CYROS_ASSERT_OP(reload, >, 0u);

   cyros_mask_token_t const token = cyros_port_irq_save();

   cortex_m::reg(cortex_m::systick_ctrl) = 0u;          /* stop before reprogramming */
   cortex_m::reg(cortex_m::systick_load) = reload;
   cortex_m::reg(cortex_m::systick_val)  = 0u;          /* any write clears to reload */

   configured_tick_hz = tick_hz;
   tick_count = 0;

   /* Enabled and counting, but the interrupt stays off until
    * cyros_port_time_irq_enable. The time driver owns that decision. */
   cortex_m::reg(cortex_m::systick_ctrl) = cortex_m::systick_ctrl_clksource | cortex_m::systick_ctrl_enable;

   cyros_port_irq_restore(token);
}

std::uint64_t cyros_port_time_now(void)
{
   /* Masked because the read is two words and SysTick can land between them.
    * The window is a handful of instructions. */
   cyros_mask_token_t const token = cyros_port_irq_save();
   std::uint64_t const value = tick_count;
   cyros_port_irq_restore(token);
   return value;
}

std::uint64_t cyros_port_time_freq_hz(void)
{
   /* Port ticks ARE SysTick interrupts here, so the port tick rate is the
    * configured tick rate and not the CPU clock. */
   return configured_tick_hz;
}

void cyros_port_time_reset(std::uint64_t time)
{
   cyros_mask_token_t const token = cyros_port_irq_save();
   tick_count = time;
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
   cortex_m::reg(cortex_m::systick_ctrl) = cortex_m::reg(cortex_m::systick_ctrl) | cortex_m::systick_ctrl_tickint;
}

void cyros_port_time_irq_disable(void)
{
   cortex_m::reg(cortex_m::systick_ctrl) = cortex_m::reg(cortex_m::systick_ctrl) & ~cortex_m::systick_ctrl_tickint;
}

void cyros_port_time_arm(std::uint64_t deadline)
{
   (void)deadline;
   cyros_port_system_error(
      0, 0, "cortex_m33 port: one-shot arm needs tickless support", 0);
}

void cyros_port_time_disarm(void)
{
   cyros_port_system_error(
      0, 0, "cortex_m33 port: one-shot disarm needs tickless support", 0);
}

void cyros_port_send_time_ipi(std::uint32_t core_id)
{
   /* Single core: the time core is always this core, so there is nothing to
    * notify. port_time.h explicitly permits an empty implementation. */
   CYROS_ASSERT_OP(core_id, ==, 0u);
}


/* ============================================================================
 * The interrupt
 * ========================================================================= */

/**
 * @brief SysTick ISR. Named for the vector table the application supplies.
 *
 * Runs one implemented priority level above PendSV, so it preempts threads and
 * is itself never delayed by a preemption-disable, while any reschedule it
 * requests lands after it returns.
 */
extern "C" void SysTick_Handler(void)
{
   /* Already at interrupt priority, so no masking is needed for this update:
    * SysTick cannot preempt itself, and every thread-context reader masks. */
   ++tick_count;

   if (isr_handler != nullptr) {
      isr_handler(isr_argument);
   }
}
