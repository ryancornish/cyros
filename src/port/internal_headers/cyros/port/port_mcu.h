/**
 * @file port_mcu.h
 * @brief Cyros MCU port contract (C ABI): what a core alone cannot answer.
 *
 * The companion to `port_core.h`. Everything here has an MCU-specific answer
 * even when the processor core is identical, so two targets sharing a core
 * share `port_core.h` and differ here.
 *
 * Two groups:
 *
 *  - **Multicore identity and bring-up.** Which core am I, how are the others
 *    started, and how does one interrupt another.
 *  - **The time source.** SysTick is core-level HARDWARE, but the time
 *    CONTRACT is MCU-level, because a given MCU may answer it with a different
 *    peripheral entirely (an LPTIM, say, which keeps running when the CPU clock
 *    does not). A contract's layer is set by who may vary it, not by what
 *    hardware one implementation happens to use.
 *
 * A target may additionally require symbols from the APPLICATION, for facts
 * only the board knows, such as what frequency is actually reaching its timer.
 * A target declares those itself rather than this header declaring them for
 * everyone, because what a board must supply depends on which target is
 * selected. cyros never implements them.
 */

#ifndef CYROS_PORT_MCU_H
#define CYROS_PORT_MCU_H

#include <cyros/port/port_core.h>

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------------------
 * SMP & Multi-Core Support
 *
 * How a core identifies itself, how secondary cores are started, and how one
 * core interrupts another all have MCU-specific answers and no architectural
 * one. An SSE-200 releases its second core through CPUWAIT and rings an MHU;
 * an RP2350 uses its SIO FIFO for both; a hosted port uses threads. That is why
 * these three live here rather than in port_core.h.
 *
 * All three are trivial when CYROS_PORT_CORE_COUNT is 1, which is exactly why
 * their MCU-dependence is easy to miss.
 * ------------------------------------------------------------------------- */

/**
 * @brief Get the ID of the current CPU core
 * @return Core ID (0-indexed)
 *
 * For single-core systems, always returns 0.
 * For SMP systems, returns which core is executing this code.
 */
uint32_t cyros_port_get_core_id(void);

/**
 * @brief Start (or release) all secondary cores and run entry on every core.
 * @param cores_to_use Number of cores to start
 * @param entry Entry point to run on each core
 *
 * After this call returns on the bootstrap core:
 *  - On embedded: typically never returns because entry will start the first thread.
 *  - On simulation: may return if port_start_first returns (cooperative).
 */
void cyros_port_start_cores(size_t cores_to_use, cyros_port_core_entry_t entry);

/**
 * @brief Send an IPI to another core to trigger a reschedule
 * @param core_id Target core ID
 *
 * Causes the target core to perform a reschedule at its next safe point. This
 * is the cross-core analogue of cyros_port_pend_reschedule(): it carries the
 * same weak guarantee and the receiving core resolves it exactly as a locally
 * pended reschedule would be.
 */
void cyros_port_send_reschedule_ipi(uint32_t core_id);




/* ============================================================================
 * Port Type Definitions
 * ----------------------------------------------------------------------------
 * Various types to support the contract API.
 * ========================================================================= */

/**
 * @brief ISR signature
 */
typedef void (*cyros_port_isr_handler_t)(void* arg);


/* ============================================================================
 * Time Driver Port Contract API
 * ----------------------------------------------------------------------------
 * Provides monotonic time for real drivers (periodic / tickless) in unit
 * tests, plus tickless one-shot arming and ISR delivery when pumped.
 *
 * Note:
 * - The simulation time driver owns time and does NOT use this.
 * - Periodic and tickless driver unit tests pump the ISR via cyros_port_time_fire_isr()
 * ========================================================================= */

/**
 * @brief Configure the underlying timer peripheral(s) used for OS time.
 * @param tick_hz Tick frequency (0 for tickless mode)
 *
 * If tick_hz > 0:
 *   Configure a periodic timer interrupt at tick_hz.
 *   The port must deliver the registered ISR handler once per tick IRQ.
 *
 * If tick_hz == 0:
 *   Configure tickless one-shot mode.
 *   The driver will call cyros_port_time_arm()/disarm() to schedule deadlines.
 *   The port must deliver the registered ISR handler when time_now() >= armed deadline.
 *
 * Called by the selected time driver during start().
 */
void cyros_port_time_setup(uint32_t tick_hz);

/**
 * @brief Monotonic time source
 * @return Current time in port ticks
 *
 * Must be monotonic 64-bit in "port ticks" (opaque unit for whole system).
 */
uint64_t cyros_port_time_now(void);

/**
 * @brief Free-running counter frequency in Hz (ticks per second).
 * @return Frequency in Hz
 *
 * For example:
 *  - DWT_CYCCNT at CPU clock: 168'000'000
 *  - Timer running at 1 MHz: 1'000'000
 *  - Linux steady_clock ns ticks: 1'000'000'000
 */
uint64_t cyros_port_time_freq_hz(void);

/**
 * @brief Reset any internal global time tracking state.
 * @param time Initial time value
 *
 * On embedded targets this is typically meaningless or implemented
 * implicitly by a system reset.
 *
 * Intended primarily for simulation and unit testing to provide
 * deterministic startup conditions.
 * @TODO: SHould I make this extern and invisible in the contract? (Like cyros_port_time_advance)
 */
void cyros_port_time_reset(uint64_t time);

/**
 * @brief Register an ISR handler for timer interrupts
 * @param handler ISR callback function
 * @param arg Argument to pass to handler
 */
void cyros_port_time_register_isr_handler(cyros_port_isr_handler_t handler, void* arg);

/**
 * @brief Enable timer interrupts
 */
void cyros_port_time_irq_enable(void);

/**
 * @brief Disable timer interrupts
 */
void cyros_port_time_irq_disable(void);

/**
 * @brief Arm a one-shot interrupt for the given absolute deadline.
 * @param deadline Absolute time in port ticks
 *
 * If called multiple times before the interrupt fires, the port must ensure
 * the earliest deadline is honored (i.e., effectively min(current, deadline)).
 *
 * Must be safe to call with interrupts disabled.
 */
void cyros_port_time_arm(uint64_t deadline);

/**
 * @brief Disable any pending one-shot.
 */
void cyros_port_time_disarm(void);

/**
 * @brief Notify the time core that there is pending time work.
 * @param core_id Target core ID
 *
 * If unimplemented on a platform, it may be an empty function.
 * Used for SMP policy where non-time cores enqueue requests for the time core.
 */
void cyros_port_send_time_ipi(uint32_t core_id);



#ifdef __cplusplus
}
#endif

#endif /* CYROS_PORT_MCU_H */
