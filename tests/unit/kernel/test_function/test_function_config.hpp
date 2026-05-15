/**
 * @file config.hpp
 * @brief CoRTOS Configuration header
 */

#ifndef CORTOS_CONFIG_HPP
#define CORTOS_CONFIG_HPP

#include <cstdint>

namespace cortos::config
{

/**
 * @brief How many cores for SMP
 */
static constexpr std::size_t CORES = 1;

/**
 * @brief TODO
 */
static constexpr std::size_t MAX_WAIT_NODES = 8;

/**
 * @brief TODO
 */
static constexpr std::uint32_t TIME_CORE_ID = 0;

/**
 * @brief TODO
 */
static constexpr std::size_t MAX_PRIORITIES = 31;

}  // namespace cortos::config

#endif
