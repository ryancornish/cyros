/**
 * @file mtime.hpp
 * @brief What the MTIME time source (port_time_mtime.cpp) needs from a target,
 *        and the one function it gives back.
 *
 * The timer the RISC-V privileged specification describes: one 64-bit MTIME
 * counter that every core reads, and a 64-bit MTIMECMP per core that raises
 * that core's timer interrupt while MTIME >= MTIMECMP. QEMU's virt has it in
 * its CLINT. The RP2350 has it in its SIO and gives it to BOTH of its ISAs:
 * the Hazard3 cores take it as `mtip`, the Cortex-M33s as IRQ 29. So the time
 * source is ISA-neutral, and a target supplies where the registers are and how
 * its core takes the interrupt. The BOARD supplies the rate.
 */
#ifndef CYROS_PORT_COMMON_MTIME_HPP
#define CYROS_PORT_COMMON_MTIME_HPP

#include <cstdint>

namespace cyros::port::mtime
{

/* ============================================================================
 * Supplied by the target
 * ========================================================================= */

/** MTIME, all 64 bits, as one consistent value. Readable from any core. */
std::uint64_t read() noexcept;

/**
 * @brief Set core `core`'s MTIMECMP without a spurious interrupt in between.
 *
 * On a 32-bit core the two halves are separate stores, so a careless order can
 * pass through a value below MTIME for one instruction. A target uses the
 * specification's sequence: low half all ones, high half, low half.
 */
void compare_write(std::uint32_t core, std::uint64_t value) noexcept;

/**
 * @brief Let the calling core take its compare interrupt, or stop it.
 *
 * Enabled, the interrupt must still be taken while preemption is disabled, as
 * SysTick is on ARM: the time source runs under the scheduler, not beside it.
 */
void interrupt_enable() noexcept;
void interrupt_disable() noexcept;

/* ============================================================================
 * Supplied by port_time_mtime.cpp
 * ========================================================================= */

/**
 * @brief The body of the calling core's compare interrupt.
 *
 * The target calls it from that interrupt's handler, with interrupts masked.
 * It rewrites the core's MTIMECMP before returning, which is what lowers the
 * line: the interrupt is level, on both ISAs.
 */
void interrupt() noexcept;

} // namespace cyros::port::mtime

/**
 * @brief MTIME's rate in Hz, supplied by the BOARD. No default.
 *
 * The argument is cyros_port_systick_clock_hz's: a guessed rate silently
 * scales every duration in the system, and a missing one fails to link. QEMU's
 * virt runs MTIME at 10 MHz. The RP2350 runs it at the tick generator's rate,
 * or at clk_sys with FULLSPEED, whichever the board set up.
 */
extern "C" std::uint32_t cyros_port_mtime_clock_hz(void);

#endif /* CYROS_PORT_COMMON_MTIME_HPP */
