/**
 * @file test_target_smp_channel_config.hpp
 * @brief Cyros configuration for the dual-core Cortex-M33 channel test.
 */

#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstddef>

namespace cyros::config
{

/**
 * @brief How many cores for SMP.
 *
 * Two. This test runs on the cortex_m33_smp port under QEMU's mps2-an521,
 * where CYROS_PORT_CORE_COUNT agrees.
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
