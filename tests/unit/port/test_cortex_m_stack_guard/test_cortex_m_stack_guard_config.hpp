/**
 * @file test_cortex_m_stack_guard_config.hpp
 * @brief Cyros configuration for the single-core Cortex-M target (`cortex_m`).
 */

#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstddef>

namespace cyros::config
{

/**
 * @brief How many cores for SMP.
 *
 * One. The parts this port targets (the STM32U575 and the TM4C123, and the
 * mps2-an505 and mps2-an386 benches) are single core, and CYROS_PORT_CORE_COUNT
 * agrees. A dual-M33 target is a separate port variant, not a change here.
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
