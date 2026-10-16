/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file port_traits.h
 * @brief Port traits for the rp2040 target: the RP2040's Cortex-M0+ core 0 alone.
 *
 * The core layer is armv6m, so everything but the cache line is the cortex_m0
 * target's, and its port_traits.h says why.
 */
#ifndef CYROS_PORT_TRAITS_H
#define CYROS_PORT_TRAITS_H

/* The saved PSP, the TLS base and the MPU guard's base, as on cortex_m0. */
#define CYROS_PORT_CONTEXT_SIZE  12
#define CYROS_PORT_CONTEXT_ALIGN 4

/* AAPCS. */
#define CYROS_PORT_STACK_ALIGN 8

#define CYROS_PORT_MIN_FRAME 1024

/* The RP2040 has no data cache. 32 keeps two cores' hot words apart on the
 * two-core target, rp2040_smp, as on the RP2350's targets. */
#define CYROS_PORT_CACHE_LINE 32

/* Core 0 alone. rp2040_smp is both. */
#define CYROS_PORT_CORE_COUNT 1

#define CYROS_PORT_CAPTURE_LOCATION 1
#define CYROS_PORT_DEBUG_MODE 1

#endif /* CYROS_PORT_TRAITS_H */
