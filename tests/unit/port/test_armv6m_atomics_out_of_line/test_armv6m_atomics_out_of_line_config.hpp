/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_armv6m_atomics_out_of_line_config.hpp
 * @brief Cyros configuration for test_armv6m_atomics_out_of_line.
 */

#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstddef>

namespace cyros::config
{

/**
 * @brief How many cores for SMP.
 *
 * One, on every target the test runs on. On rp2040_smp the port is still
 * built for two cores, so its atomics still take the SIO lock, which is the
 * build the flag matters for.
 */
inline constexpr std::size_t cores = 1;

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
