/**
 * @file cortex_m33.hpp
 * @brief Cyros configuration for the single-core Cortex-M33 target.
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

}  // namespace cyros::config

#endif
