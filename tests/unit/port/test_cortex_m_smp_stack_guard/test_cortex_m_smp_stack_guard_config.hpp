/**
 * @file test_cortex_m_smp_stack_guard_config.hpp
 * @brief Cyros configuration for the stack guard test on a two-core Cortex-M
 *        target, its threads on core 1.
 */

#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstddef>

/* Which core test_cortex_m_stack_guard.cpp pins its threads to. */
#define CYROS_TEST_GUARD_CORE 1

namespace cyros::config
{

/**
 * @brief How many cores for SMP.
 *
 * Two, as the two-core targets' CYROS_PORT_CORE_COUNT says. Core 0 only idles.
 */
inline constexpr std::size_t cores = 2;

/**
 * @brief Maximum wait nodes per thread.
 */
inline constexpr std::size_t max_wait_nodes = 8;

/**
 * @brief Number of scheduling priorities.
 */
inline constexpr std::size_t max_priorities = 31;

/**
 * @brief Enable trace points around the kernel
 *
 * Disabled when 0. Sizes each cores buffer. Must be a power of 2
 */
inline constexpr std::size_t trace_records_per_core = 0;

}  // namespace cyros::config

#endif
