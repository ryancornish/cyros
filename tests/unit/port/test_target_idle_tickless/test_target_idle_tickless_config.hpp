/**
 * @file test_target_idle_tickless_config.hpp
 * @brief Cyros configuration for the single-core Cortex-M target (`cortex_m`).
 */

#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstddef>

namespace cyros::config
{

/**
 * @brief How many cores for SMP.
 */
inline constexpr std::size_t cores = 1;

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

/* Not a cyros setting. test_target_idle.cpp is built against both time
 * drivers, which want different arguments to time::initialise, and nothing else
 * in the build says which one this is. This test pairs it with the tickless
 * driver, and the test checks that against what the port reports. */
#define CYROS_TEST_IDLE_TICKLESS 1

#endif
