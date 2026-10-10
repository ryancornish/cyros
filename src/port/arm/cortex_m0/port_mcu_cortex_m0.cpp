/**
 * @file port_mcu_cortex_m0.cpp
 * @brief The generic single-core ARMv6-M target: multicore identity and
 *        bring-up, and the timestamp.
 *
 * cortex_m's Baseline sibling: a Cortex-M0 or M0+ with nothing but core
 * peripherals, which QEMU's microbit is. The core half comes from `armv6m`,
 * the time half from ../common/port_time_systick.cpp, and what is left is the
 * identity trio, answered for one core as cortex_m answers it, and the
 * counter behind cyros_port_timestamp.
 */

#include <cyros/port/port_mcu.h>

#include "cortex_m.hpp"

#include <cstddef>
#include <cstdint>


std::uint32_t cyros_port_get_core_id(void)
{
   return 0u;
}

void cyros_port_start_cores(std::size_t cores_to_use, cyros_port_core_entry_t entry)
{
   /* Single core. A dual-M0+ part (the RP2040) is a separate target. */
   CYROS_ASSERT_OP(cores_to_use, ==, 1u);
   CYROS_ASSERT(entry != nullptr);

   entry();

   /* entry() reaches cyros_port_start_first, which never returns. */
   CYROS_PORT_UNREACHABLE();
}

void cyros_port_send_reschedule_ipi(std::uint32_t core_id)
{
   /* One core: the cross-core request is a local one. */
   CYROS_ASSERT_OP(core_id, ==, 0u);
   cyros_port_pend_reschedule();
}

/* The counter behind cyros_port_timestamp (cortex_m.hpp). ARMv6-M has no
 * cycle counter, and a core-peripherals-only part has nothing else that runs
 * free, so stamps are zero, as QEMU's DWT reads on the Mainline benches. A
 * board with a free-running timer is a target of its own that answers here:
 * the RP2040's 1 us TIMER, for one. */
std::uint64_t cyros::port::cortex_m::timestamp() noexcept
{
   return 0u;
}
