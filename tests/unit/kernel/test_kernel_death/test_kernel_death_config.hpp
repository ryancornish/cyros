#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstddef>

namespace cyros::config
{

// ONE core, so that a thread pinned to core1 names a core this build does not
// have. Nothing else here needs more.
inline constexpr std::size_t cores                  = 1;
inline constexpr std::size_t max_wait_nodes         = 4;
inline constexpr std::size_t max_priorities         = 31;
inline constexpr std::size_t trace_records_per_core = 0;

}  // namespace cyros::config

#endif // CYROS_CONFIG_HPP
