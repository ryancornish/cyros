/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file orbit_demo_config.hpp
 * @brief Cyros configuration for the Orbit demo on the EK-TM4C123GXL.
 */

#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstddef>

namespace cyros::config
{

/** The TM4C123 has one core. */
inline constexpr std::size_t cores = 1;

/** Priorities 1 to 7 are the demo's threads and 15 is the idle thread. */
inline constexpr std::size_t max_priorities = 16;

/** No tracing. */
inline constexpr std::size_t trace_records_per_core = 0;

}  // namespace cyros::config

#endif
