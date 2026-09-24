#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstdint>

namespace cyros::config
{

/* Single core, so there is exactly one timetable in play and the ISR that
 * sends is on the same core as the worker that runs the job. That is the
 * harder arrangement and the realistic one: on a single-core MCU the deferred
 * work has nowhere else to go. */
inline constexpr std::size_t   cores          = 1;
inline constexpr std::size_t   max_wait_nodes = 8;
inline constexpr std::size_t   max_priorities = 31;

}  // namespace cyros::config

#endif // CYROS_CONFIG_HPP
