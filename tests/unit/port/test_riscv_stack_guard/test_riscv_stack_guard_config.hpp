/**
 * @file test_riscv_stack_guard_config.hpp
 * @brief Cyros configuration for test_riscv_stack_guard: one core.
 */

#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstddef>

namespace cyros::config
{

/**
 * @brief How many cores for SMP.
 *
 * One. The guard is per hart, and this test watches one.
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

#endif
