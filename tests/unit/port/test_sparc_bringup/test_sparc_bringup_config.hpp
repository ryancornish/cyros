/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_sparc_bringup_config.hpp
 * @brief Cyros configuration for the single-core LEON3 target (`leon3`).
 */

#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstddef>

namespace cyros::config
{

/** One core, as CYROS_PORT_CORE_COUNT says. */
inline constexpr std::size_t cores = 1;

/** Maximum wait nodes per thread. */
inline constexpr std::size_t max_wait_nodes = 8;

/** Number of scheduling priorities. */
inline constexpr std::size_t max_priorities = 31;

/** Trace points off. */
inline constexpr std::size_t trace_records_per_core = 0;

}  // namespace cyros::config

#endif
