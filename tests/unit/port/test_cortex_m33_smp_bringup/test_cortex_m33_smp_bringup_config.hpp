/**
 * @file cortex_m33.hpp
 * @brief Cyros configuration for the single-core Cortex-M33 target.
 */

#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstdint>

namespace cyros::config
{

/**
 * @brief How many cores for SMP.
 *
 * One. The M33 parts this port targets (STM32U575, and the mps2-an505 bench)
 * are single core, and CYROS_PORT_CORE_COUNT agrees. A dual-M33 target is a
 * separate port variant, not a change here.
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
