#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstddef>

namespace cyros::config
{

// Four cores, so four POSIX timers: one targeting the calling thread, which the
// kernel borrows as core 0 and gives back, and three targeting threads that
// have exited by the time the kernel returns. Teardown has to reach both kinds.
inline constexpr std::size_t cores                  = 4;
inline constexpr std::size_t max_wait_nodes         = 4;
inline constexpr std::size_t max_priorities         = 31;
inline constexpr std::size_t trace_records_per_core = 0;

}  // namespace cyros::config

#endif // CYROS_CONFIG_HPP
