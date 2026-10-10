/**
 * @file test_riscv_smp_bringup_config.hpp
 * @brief Cyros configuration for the two-hart RISC-V target (`riscv_virt_smp`).
 */

#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstddef>

namespace cyros::config
{

/** Two harts, as CYROS_PORT_CORE_COUNT says. */
inline constexpr std::size_t cores = 2;

/** Maximum wait nodes per thread. */
inline constexpr std::size_t max_wait_nodes = 8;

/** Number of scheduling priorities. */
inline constexpr std::size_t max_priorities = 31;

/** Trace points off. */
inline constexpr std::size_t trace_records_per_core = 0;

}  // namespace cyros::config

#endif
