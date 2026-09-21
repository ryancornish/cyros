/**
 * @file port.h
 * @brief Umbrella over the port contract, for the kernel.
 *
 * The contract is two headers, split by which layer may vary each function:
 *
 *   port_core.h   the processor core: switching, masking, PSPLIM, TLS, debug
 *   port_mcu.h    the MCU: multicore identity and bring-up, the IPI, the time
 *                 source
 *
 * This umbrella exists so that the kernel, which legitimately uses both, keeps
 * one include. A PORT IMPLEMENTATION SHOULD NOT USE IT: a core layer includes
 * port_core.h and an MCU layer includes port_mcu.h, which is what keeps a layer
 * from accidentally defining something it does not own.
 */

#ifndef CYROS_PORT_H
#define CYROS_PORT_H

#include <cyros/port/port_core.h>
#include <cyros/port/port_mcu.h>

#endif /* CYROS_PORT_H */
