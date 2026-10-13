/**
 * @file port_traits.h
 * @brief Port traits for the cortex_r52_smp target: Cortex-R52, two cores (QEMU's mps3-an536).
 */
#ifndef CYROS_PORT_TRAITS_H
#define CYROS_PORT_TRAITS_H

/* cyros_port_context is three words: a pointer to the thread's exception
 * frame, its TLS base, and its stack guard. Sized with room to spare. */
#define CYROS_PORT_CONTEXT_SIZE  16
#define CYROS_PORT_CONTEXT_ALIGN 8

/* The AAPCS keeps sp 8-byte aligned at every public interface. */
#define CYROS_PORT_STACK_ALIGN 8

/* A thread stack holds its own frames plus one exception frame, 64 bytes, or
 * 328 with the FP state of a hard-float build, and the 128-byte stack guard.
 * Handlers run on the interrupt stack, not here, so 1024 matches the others. */
#define CYROS_PORT_MIN_FRAME 1024

/* The Cortex-R52's cache line. */
#define CYROS_PORT_CACHE_LINE 64

#define CYROS_PORT_CORE_COUNT 2

#define CYROS_PORT_CAPTURE_LOCATION 1
#define CYROS_PORT_DEBUG_MODE 1

#endif /* CYROS_PORT_TRAITS_H */
