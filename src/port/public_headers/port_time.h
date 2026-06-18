/**
 * @file port_time.h
 * @brief Cyros Timer-Driver Port Layer API (C ABI)
 *
 * TODO: Description
 */

#ifndef CYROS_PORT_TIME_H
#define CYROS_PORT_TIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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
 * - Periodic driver unit tests call on_timer_isr() directly.
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

#endif /* CYROS_PORT_TIME_H */
