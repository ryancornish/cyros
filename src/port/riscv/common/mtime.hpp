/**
 * @file mtime.hpp
 * @brief What the MTIME time source (port_time_mtime.cpp) needs from a target.
 *
 * Every RISC-V target here keeps time on the platform timer the privileged
 * specification describes: one 64-bit MTIME counter shared by every hart, and
 * a 64-bit MTIMECMP per hart that raises that hart's timer interrupt while
 * MTIME >= MTIMECMP. Where the registers are differs (the CLINT on QEMU's
 * virt, the SIO on the RP2350), and so does the rate, so a target supplies the
 * two accessors and the BOARD supplies the rate.
 */
#ifndef CYROS_PORT_RISCV_MTIME_HPP
#define CYROS_PORT_RISCV_MTIME_HPP

#include <cstdint>

namespace cyros::port::riscv
{

/** MTIME, all 64 bits, as one consistent value. Readable from any hart. */
std::uint64_t mtime_read() noexcept;

/**
 * @brief Set hart `core`'s MTIMECMP without a spurious interrupt in between.
 *
 * On a 32-bit hart the two halves are separate stores, so a careless order can
 * pass through a value below MTIME for one instruction. A target uses the
 * specification's sequence: low half all ones, high half, low half.
 */
void mtimecmp_write(std::uint32_t core, std::uint64_t value) noexcept;

} // namespace cyros::port::riscv

/**
 * @brief MTIME's rate in Hz, supplied by the BOARD. No default.
 *
 * The argument is cyros_port_systick_clock_hz's: a guessed rate silently
 * scales every duration in the system, and a missing one fails to link. QEMU's
 * virt runs MTIME at 10 MHz. The RP2350 runs it at the tick generator's rate,
 * or at clk_sys with FULLSPEED, whichever the board set up.
 */
extern "C" std::uint32_t cyros_port_mtime_clock_hz(void);

#endif /* CYROS_PORT_RISCV_MTIME_HPP */
