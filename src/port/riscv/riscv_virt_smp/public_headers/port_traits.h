/**
 * @file port_traits.h
 * @brief Port traits for the riscv_virt_smp target: QEMU's RISC-V virt, two harts.
 */
#ifndef CYROS_PORT_TRAITS_H
#define CYROS_PORT_TRAITS_H

/* cyros_port_context is one pointer, to the thread's trap frame. Sized with
 * room to spare, as on ARM. */
#define CYROS_PORT_CONTEXT_SIZE  16
#define CYROS_PORT_CONTEXT_ALIGN 8

/* The RISC-V psABI keeps sp 16-byte aligned. */
#define CYROS_PORT_STACK_ALIGN 16

/* A thread stack holds its own frames plus one 128-byte trap frame. Handlers
 * run on the interrupt stack, not here, so 1024 matches ARM. */
#define CYROS_PORT_MIN_FRAME 1024

/* QEMU's virt reports 64-byte cache blocks (Zic64b). */
#define CYROS_PORT_CACHE_LINE 64

#define CYROS_PORT_CORE_COUNT 2

#define CYROS_PORT_CAPTURE_LOCATION 1
#define CYROS_PORT_DEBUG_MODE 1

#endif /* CYROS_PORT_TRAITS_H */
