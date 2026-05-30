#ifndef CYROS_TIME_SIMULATION_HPP
#define CYROS_TIME_SIMULATION_HPP

#include <cyros/time/time.hpp>

namespace cyros::time::simulation
{

enum class mode
{
   real_time,
   virtual_time,
};

/**
 * @brief Select simulation time progression mode.
 *
 * real_time mode follows wall-clock time.
 * virtual_time mode advances only when explicitly driven by tests.
 */
void set_mode(mode m) noexcept;

/**
 * @brief Get the current simulation mode.
 */
[[nodiscard]] mode get_mode() noexcept;

/**
 * @brief Reset simulation driver state to a known time point.
 *
 * Intended for deterministic tests.
 */
void reset(time_point tp = time_point{0}) noexcept;

/**
 * @brief Advance virtual time to a specific time point.
 *
 * Has no effect in real_time mode.
 */
void advance_to(time_point tp) noexcept;

/**
 * @brief Advance virtual time by a duration.
 *
 * Has no effect in real_time mode.
 */
void advance_by(duration d) noexcept;

} // namespace cyros::time::simulation

#endif
