/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

#include <cyros/kernel/core.hpp>
#include <cyros/port/port.h>

namespace cyros::this_core
{

[[nodiscard]] std::uint32_t id() noexcept
{
   return cyros_port_get_core_id();
}

void pend_reschedule() noexcept
{
   cyros_port_pend_reschedule();
}

void cpu_relax() noexcept
{
   cyros_port_cpu_relax();
}

CYROS_HOT preemption_token disable_preemption() noexcept
{
   return { .v = cyros_port_preempt_disable() };
}

CYROS_HOT void enable_preemption(preemption_token token) noexcept
{
   cyros_port_preempt_enable(token.v);
}

CYROS_HOT critical_token enter_critical() noexcept
{
   return { .v = cyros_port_irq_save() };
}

CYROS_HOT void exit_critical(critical_token token) noexcept
{
   cyros_port_irq_restore(token.v);
}

} // namespace cyros::this_core
