#ifndef CORTOS_TIME_SIMULATION_HPP
#define CORTOS_TIME_SIMULATION_HPP

#include <cortos/time/time.hpp>

namespace cortos::time::simulation
{

enum class Mode
{
   RealTime,
   Virtual
};

/**
 * @brief Select simulation time progression mode.
 *
 * RealTime mode follows wall-clock time.
 * Virtual mode advances only when explicitly driven by tests.
 */
void set_mode(Mode mode) noexcept;

/**
 * @brief Get the current simulation mode.
 */
[[nodiscard]] Mode get_mode() noexcept;

/**
 * @brief Reset simulation driver state to a known time point.
 *
 * Intended for deterministic tests.
 */
void reset(time_point tp = time_point{0}) noexcept;

/**
 * @brief Advance virtual time to a specific time point.
 *
 * Has no effect in RealTime mode.
 */
void advance_to(time_point tp) noexcept;

/**
 * @brief Advance virtual time by a duration.
 *
 * Has no effect in RealTime mode.
 */
void advance_by(duration d) noexcept;

} // namespace cortos::time::simulation

#endif
