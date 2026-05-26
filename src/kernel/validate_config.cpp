#include <cortos/config/config.hpp>
#include <cortos/port/port_traits.h>

#include <cstdint>
#include <limits>

namespace cortos
{

/* ============================================================================
 * Configuration validation
 * ========================================================================= */
static_assert(1 <= config::cores && config::cores <= CORTOS_PORT_CORE_COUNT,
              "Port does not support configured amount of cores.");
static_assert(config::time_core_id < config::cores,
              "Time core set to non-existent core.");
static_assert(config::max_priorities < std::numeric_limits<uint32_t>::digits,
              "Priorities unsupported by kernel implementation.");

} // namespace cortos
