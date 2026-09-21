/**
 * @file port_mcu_cortex_m33.cpp
 * @brief The generic single-core M33 target: multicore identity and bring-up.
 *
 * This target is "an M33 with nothing but core peripherals", which is what
 * QEMU's mps2-an505 bench is and what an STM32U575 looks like before any vendor
 * peripheral is involved. The core half of the contract comes from `armv8m`,
 * which this variant extends, so what is left here is the part of port_mcu.h
 * that a real MCU answers differently:
 *
 *   get_core_id           which of the MCU's cores is executing
 *   start_cores           how a second core is released from reset
 *   send_reschedule_ipi   which mailbox or doorbell reaches another core
 *
 * ARMv8-M says nothing about any of these, which is why they are not in the
 * core layer. A dual-M33 part answers all three differently while running
 * byte-identical core code.
 *
 * The time half of port_mcu.h is in port_time_systick.cpp. It is the half a
 * vendor target replaces on its own, LPTIM instead of SysTick, while leaving
 * these three alone, so it gets a file of its own.
 */

#include <cyros/port/port_mcu.h>

#include <cstddef>
#include <cstdint>


std::uint32_t cyros_port_get_core_id(void)
{
   return 0u;
}

void cyros_port_start_cores(std::size_t cores_to_use, cyros_port_core_entry_t entry)
{
   /* Single core. A dual-M33 part (RP2350, or QEMU's mps2-an521) would start
    * the second core here, and is a separate target variant rather than a
    * switch in this one. */
   CYROS_ASSERT_OP(cores_to_use, ==, 1u);
   CYROS_ASSERT(entry != nullptr);

   entry();

   /* entry() reaches cyros_port_start_first, which never returns. */
   CYROS_PORT_UNREACHABLE();
}

void cyros_port_send_reschedule_ipi(std::uint32_t core_id)
{
   /* On one core the cross-core request degenerates to a local one, and carries
    * the same weak guarantee. cyros_port_pend_reschedule is a CORE function,
    * from the layer this target extends: an MCU layer may freely call the core
    * contract, it just may not implement it. */
   CYROS_ASSERT_OP(core_id, ==, 0u);
   cyros_port_pend_reschedule();
}
