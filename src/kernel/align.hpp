#ifndef CYROS_ALIGN_HPP
#define CYROS_ALIGN_HPP

#include <cstdint>

namespace cyros
{

constexpr std::uintptr_t align_down(std::uintptr_t v, std::size_t a)
{
   return v & ~(static_cast<std::uintptr_t>(a) - 1);
}

constexpr std::uintptr_t align_up(std::uintptr_t v, std::size_t a)
{
   return (v + (a - 1)) & ~(static_cast<std::uintptr_t>(a) - 1);
}

} // namespace cyros

#endif // CYROS_ALIGN_HPP
