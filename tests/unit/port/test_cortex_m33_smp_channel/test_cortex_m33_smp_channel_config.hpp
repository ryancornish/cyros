/**
 * @file test_cortex_m33_smp_channel_config.hpp
 * @brief Cyros configuration for the dual-core Cortex-M33 channel test.
 */

#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstdint>

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

}  // namespace cyros::config

#endif
