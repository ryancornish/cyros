#ifndef CYROS_CORE_HPP
#define CYROS_CORE_HPP

#include <cstdint>

namespace cyros::this_core
{

/**
   * @brief Get current CPU core ID (0-based)
   */
[[nodiscard]] std::uint32_t id() noexcept;

/**
 * @brief Request a deferred reschedule on the calling core.
 *
 * Pends a reschedule on the current core, safe to call from an ISR.
 * @note This may return without rescheduling. If so, the reschedule is deferred and resolved
 * at the next safe point.
 */
void pend_reschedule() noexcept;

struct [[nodiscard]] preemption_token { std::uint32_t v; };

preemption_token disable_preemption() noexcept;

void enable_preemption(preemption_token token) noexcept;

struct preemption_guard
{
public:
   preemption_guard() noexcept
   {
      token = disable_preemption();
   }

   ~preemption_guard() noexcept
   {
      enable_preemption(token);
   }

   preemption_guard(preemption_guard const&)            = delete;
   preemption_guard& operator=(preemption_guard const&) = delete;
   preemption_guard(preemption_guard&&)                 = delete;
   preemption_guard& operator=(preemption_guard&&)      = delete;

private:
   preemption_token token{};
};

} // namespace cyros::this_core

#endif // CYROS_CORE_HPP
