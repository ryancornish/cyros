/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file port_traits.h
 * @brief Port traits for the leon3_smp target: GRLIB LEON3, 2 cores (QEMU's leon3_generic).
 */
#ifndef CYROS_PORT_TRAITS_H
#define CYROS_PORT_TRAITS_H

/* cyros_port_context is one word, a pointer to the thread's trap frame.
 * Sized with room to spare. */
#define CYROS_PORT_CONTEXT_SIZE  8
#define CYROS_PORT_CONTEXT_ALIGN 8

/* The SPARC ABI keeps sp 8-byte aligned, as ldd and std need. */
#define CYROS_PORT_STACK_ALIGN 8

/* A thread stack holds its own frames, each at least 96 bytes because every
 * SPARC frame reserves its register window's save area, plus one trap frame,
 * 80 bytes or 216 with the FP state of a hard-float build. Handlers run on
 * the interrupt stack, not here. */
#define CYROS_PORT_MIN_FRAME 1024

/* LEON3's cache line is built four or eight words long: the larger. */
#define CYROS_PORT_CACHE_LINE 32

#define CYROS_PORT_CORE_COUNT 2

#define CYROS_PORT_CAPTURE_LOCATION 1
#define CYROS_PORT_DEBUG_MODE 1

#endif /* CYROS_PORT_TRAITS_H */
