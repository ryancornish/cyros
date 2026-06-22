/**
 * @file kernel.hpp
 * @brief Cyros Kernel API
 *
 * This is the main kernel header. It contains all kernel primitives and APIs.
 */

#ifndef CYROS_KERNEL_HPP
#define CYROS_KERNEL_HPP

#include <cyros/kernel/function.hpp>
#include <cyros/kernel/spinlock.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/waitable.hpp>

#include <cstdint>


namespace cyros::kernel
{

/**
 * @brief Initialise the kernel
 *
 * Must be called before any threads are created or kernel functions used.
 * Sets up scheduler data structures.
 */
void initialise();

/**
 * @brief Start the scheduler
 *
 * At least one thread must exist before calling start().
 */
void start();

void finalise();

/**
 * @brief Request a deferred reschedule on the calling core.
 *
 * Pends a reschedule on the current core, safe to call from an ISR.
 * @note This may return without rescheduling. If so, the reschedule is deferred and resolved
 * at the next safe point.
 */
void pend_reschedule();

/**
 * @brief Get total number of CPU cores
 * @return Number of cores (1 for single-core)
 */
[[nodiscard]] std::uint32_t core_count() noexcept;

/**
 * @brief Get total number of currently registered threads
 *
 * Intended for diagnosis only. All threads that register add to the tally.
 * All threads that terminate substract from the tally.
 */
[[nodiscard]] std::uint32_t active_threads() noexcept;

} // namespace cyros::kernel

#endif // CYROS_KERNEL_HPP
