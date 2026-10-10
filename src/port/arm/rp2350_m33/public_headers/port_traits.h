/**
 * @file port_traits.h
 * @brief Port traits for the rp2350_m33 target: the RP2350's Cortex-M33 core 0 alone.
 *
 * The core layer is armv7m_armv8m, so everything but the core count and the
 * cache line is the cortex_m target's, and its port_traits.h says why.
 */
#ifndef CYROS_PORT_TRAITS_H
#define CYROS_PORT_TRAITS_H

/* The saved PSP, the PSPLIM bound and the TLS base, as on cortex_m. */
#define CYROS_PORT_CONTEXT_SIZE  16
#define CYROS_PORT_CONTEXT_ALIGN 8

/* AAPCS, and PSPLIM's granule. */
#define CYROS_PORT_STACK_ALIGN 8

#define CYROS_PORT_MIN_FRAME 1024

/* The RP2350 has no data cache. 32 is the bus's widest burst and keeps two
 * cores' hot words apart, as on the chip's Hazard3 targets. */
#define CYROS_PORT_CACHE_LINE 32

#define CYROS_PORT_CORE_COUNT 1

#define CYROS_PORT_CAPTURE_LOCATION 1
#define CYROS_PORT_DEBUG_MODE 1

#endif /* CYROS_PORT_TRAITS_H */
