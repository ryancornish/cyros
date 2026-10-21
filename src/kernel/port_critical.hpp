/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

#ifndef CYROS_PORT_CRITICAL_HPP
#define CYROS_PORT_CRITICAL_HPP

#include <cyros/port/port.h>

namespace cyros
{

/**
 * @brief The kernel's own critical section: the port's interrupt mask,
 *        called directly.
 *
 * this_core::critical_guard is the public face of the same mask, and reaches
 * it through core.cpp, a call each way that only an LTO build removes. The
 * switch, wait and wake paths take this one, which saves them that hop in
 * every build (performance-investigation.md 10).
 */
class port_critical_guard
{
public:
   port_critical_guard() noexcept : token(cyros_port_irq_save()) {}
   ~port_critical_guard() noexcept { cyros_port_irq_restore(token); }

   port_critical_guard(port_critical_guard const&)            = delete;
   port_critical_guard& operator=(port_critical_guard const&) = delete;
   port_critical_guard(port_critical_guard&&)                 = delete;
   port_critical_guard& operator=(port_critical_guard&&)      = delete;

private:
   cyros_mask_token_t token;
};

} // namespace cyros

#endif // CYROS_PORT_CRITICAL_HPP
