/**
 * @file port_traits.h
 * @brief Port traits for the rp2040_smp target: both of the RP2040's Cortex-M0+ cores.
 *
 * The core layer is armv6m, so everything but the core count is the rp2040
 * target's, and the cortex_m0 target's port_traits.h says why.
 */
#ifndef CYROS_PORT_TRAITS_H
#define CYROS_PORT_TRAITS_H

/* The saved PSP, the TLS base and the MPU guard's base, as on cortex_m0. */
#define CYROS_PORT_CONTEXT_SIZE  12
#define CYROS_PORT_CONTEXT_ALIGN 4

/* AAPCS. */
#define CYROS_PORT_STACK_ALIGN 8

#define CYROS_PORT_MIN_FRAME 1024

/* The RP2040 has no data cache. 32 keeps two cores' hot words apart, as on
 * the RP2350's targets. */
#define CYROS_PORT_CACHE_LINE 32

/* Two cores, so the core layer's atomics take the target's SIO spinlock
 * (port_atomic_armv6m.cpp). */
#define CYROS_PORT_CORE_COUNT 2

#define CYROS_PORT_CAPTURE_LOCATION 1
#define CYROS_PORT_DEBUG_MODE 1

#endif /* CYROS_PORT_TRAITS_H */
