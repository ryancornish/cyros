/**
 * @file kernel.hpp
 * @brief Entry point and lifecycle API for the kernel
 */

#ifndef CYROS_KERNEL_HPP
#define CYROS_KERNEL_HPP

#include <cyros/kernel/visibility.hpp>

#include <cstdint>


namespace cyros::kernel
{

/**
 * @brief Initialise the kernel
 *
 * Must be called before any threads are created or kernel functions used.
 * Sets up scheduler data structures.
 */
CYROS_PUBLIC void initialise() noexcept;

/**
 * @brief Start the scheduler
 *
 * At least one thread must exist before calling start().
 */
CYROS_PUBLIC void start() noexcept;

CYROS_PUBLIC void finalise() noexcept;

/**
 * @brief Get total number of CPU cores
 * @return Number of cores (1 for single-core)
 */
[[nodiscard]] CYROS_PUBLIC std::uint32_t core_count() noexcept;

/**
 * @brief Get total number of currently registered threads
 *
 * Intended for diagnosis only. All threads that register add to the tally.
 * All threads that terminate substract from the tally.
 */
[[nodiscard]] CYROS_PUBLIC std::uint32_t active_threads() noexcept;

} // namespace cyros::kernel

#endif // CYROS_KERNEL_HPP
