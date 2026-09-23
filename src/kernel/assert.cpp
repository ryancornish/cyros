#include <cyros/kernel/assert.hpp>

#include <cyros/port/port.h>

namespace cyros
{

/* The whole reason this function exists rather than the public macros calling
 * cyros_port_system_error directly: that symbol is declared in port_core.h,
 * which A1 made project-internal. A public header cannot name it, and the
 * builder refuses a public header that tries. So the forwarding happens here,
 * in a translation unit that is allowed to see both sides.
 *
 * The cost is one frame between a failing check and the panic, which a release
 * build's LTO collapses into a tail call. */
[[noreturn]] void panic(std::uintptr_t aux1, std::uintptr_t aux2,
                        char const* file, int line) noexcept
{
   cyros_port_system_error(aux1, aux2, file, line);
}

}  // namespace cyros
