/**
 * @file port_traits.h
 * @brief Port traits for the rp2350_hazard3_smp target: the RP2350's two Hazard3
 *        cores.
 */
#ifndef CYROS_PORT_TRAITS_H
#define CYROS_PORT_TRAITS_H

/* cyros_port_context is two words: a pointer to the thread's trap frame,
 * and its stack guard's pmpaddr. Sized with room to spare, as on ARM. */
#define CYROS_PORT_CONTEXT_SIZE  16
#define CYROS_PORT_CONTEXT_ALIGN 8

/* The RISC-V psABI keeps sp 16-byte aligned. */
#define CYROS_PORT_STACK_ALIGN 16

/* One 128-byte trap frame on the thread's stack, handlers on the interrupt
 * stack. As on riscv_virt. */
#define CYROS_PORT_MIN_FRAME 1024

/* The RP2350 has no data cache. 32 is the bus's widest burst and keeps two
 * cores' hot words apart when SMP arrives. */
#define CYROS_PORT_CACHE_LINE 32

#define CYROS_PORT_CORE_COUNT 2

#define CYROS_PORT_CAPTURE_LOCATION 1
#define CYROS_PORT_DEBUG_MODE 1

#endif /* CYROS_PORT_TRAITS_H */
