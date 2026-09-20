/**
 * @file kernel.hpp
 * @brief The kernel itself: bringing it up, taking it down, and asking what it
 *        is currently doing.
 *
 * This header declares `cyros::kernel` and nothing else, which is the whole
 * point. Until 2026-09-20 it ALSO acted as an umbrella that included every
 * kernel primitive (threads, waitables, spinlocks, function, core), so there
 * was no way to depend on starting the kernel without depending on all of them.
 *
 * That is a coupling defect in its own right, and it surfaced in the T1 test
 * layering: bring-up sits below waitables in the chain, but its tests could not
 * say so while the only header offering kernel::start() also offered wait_on().
 * The umbrella was removed rather than renamed. Include what you use.
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
