/**
 * @file config.hpp
 * @brief Cyros Configuration header (tickless time driver test)
 */

#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstdint>

namespace cyros::config
{

/**
 * @brief How many cores for SMP
 */
inline constexpr std::size_t cores = 1;

/**
 * @brief TODO
 */
inline constexpr std::size_t max_wait_nodes = 8;

/**
 * @brief TODO
 */
inline constexpr std::uint32_t time_core_id = 0;

/**
 * @brief TODO
 */
inline constexpr std::size_t max_priorities = 31;

}  // namespace cyros::config

#endif