/**
 * @file config.hpp
 * @brief Cyros Configuration header template
 */

#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstdint>

namespace cyros::config
{

/**
 * @brief How many cores for SMP
 */
inline constexpr std::size_t cores = 4;

/**
 * @brief TODO
 */
inline constexpr std::size_t max_wait_nodes = 8;

/**
 * @brief TODO
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
