#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <cstdint>
#include <limits>

namespace cyros
{

/* ============================================================================
 * Configuration validation
 * ========================================================================= */
static_assert(1 <= config::cores && config::cores <= CYROS_PORT_CORE_COUNT,
              "Port does not support configured amount of cores.");
static_assert(config::max_priorities < std::numeric_limits<uint32_t>::digits,
              "Priorities unsupported by kernel implementation."
              "See 'scheduler::tie_rotor' and 'thread_ready_matrix::bitmap'");
static_assert((config::trace_records_per_core & (config::trace_records_per_core - 1)) == 0,
              "config::trace_records_per_core must be a power of two");

} // namespace cyros
