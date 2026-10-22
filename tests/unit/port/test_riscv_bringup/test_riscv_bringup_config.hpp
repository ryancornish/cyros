/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_riscv_bringup_config.hpp
 * @brief Cyros configuration for the single-hart RISC-V target (`riscv_virt`).
 */

#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstddef>

namespace cyros::config
{

/** One hart, as CYROS_PORT_CORE_COUNT says. */
inline constexpr std::size_t cores = 1;

/** Number of scheduling priorities. */
inline constexpr std::size_t max_priorities = 31;

/** Trace points off. */
inline constexpr std::size_t trace_records_per_core = 0;

}  // namespace cyros::config

#endif
