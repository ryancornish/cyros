#ifndef CYROS_CONFIG_HPP
#define CYROS_CONFIG_HPP

#include <cstdint>

namespace cyros::config
{

// One core is enough: the subject is what the port leaves on the CALLING
// thread, which it borrows as core 0.
inline constexpr std::size_t   cores          = 1;
inline constexpr std::size_t   max_wait_nodes = 4;
inline constexpr std::size_t   max_priorities = 31;

}  // namespace cyros::config

#endif // CYROS_CONFIG_HPP
