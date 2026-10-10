/**
 * @file test_target_smp_idle_tickless_config.hpp
 * @brief Cyros configuration for the two-hart RISC-V target (`riscv_virt_smp`), tickless.
 */

#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstddef>

namespace cyros::config
{

/** Two harts, as CYROS_PORT_CORE_COUNT says. */
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

/* Not a cyros setting. test_target_smp_idle.cpp is built against both time
 * drivers, and nothing else in the build says which one this is. This test
 * pairs it with the tickless driver. */
#define CYROS_TEST_IDLE_TICKLESS 1

#endif
