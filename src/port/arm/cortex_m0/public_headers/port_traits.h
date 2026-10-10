/**
 * @file port_traits.h
 * @brief Generic single-core ARMv6-M port (`cortex_m0`) compile-time traits.
 *
 * Each port must provide exactly one `port_traits.h` defining the constants
 * below. This header:
 *  - must not include any other headers
 *  - must contain only macros
 *
 * The kernel relies on these values at compile time. The port implementation
 * statically verifies that its actual types and behaviour match them.
 */

#ifndef CYROS_PORT_TRAITS_H
#define CYROS_PORT_TRAITS_H

/**
 * @def CYROS_PORT_CONTEXT_SIZE
 * @brief Size of `port_context_t` in bytes.
 *
 * Two words, the saved PSP and the TLS base. No stack guard bound, since
 * ARMv6-M has no stack-limit register and the M0 no MPU, and no FP state.
 * The registers live on the thread's own stack, as on the Mainline layer.
 */
#define CYROS_PORT_CONTEXT_SIZE  8

/**
 * @def CYROS_PORT_CONTEXT_ALIGN
 * @brief Alignment requirement of `port_context_t` in bytes.
 *
 * Must be a power of two.
 */
#define CYROS_PORT_CONTEXT_ALIGN 4

/**
 * @def CYROS_PORT_STACK_ALIGN
 * @brief Required alignment of all thread stacks in bytes.
 *
 * 8, AAPCS's alignment at every public interface, as on the Mainline layer.
 */
#define CYROS_PORT_STACK_ALIGN 8

/**
 * @def CYROS_PORT_MIN_FRAME
 * @brief Stack headroom, in bytes, every thread gets beyond its context and TCB.
 *
 * Feeds thread::min_stack_size, and through it the idle stack every scheduler
 * embeds, so on a part with 32 KB of SRAM it decides which images fit at all.
 * At the Linux ports' 4096 two of the on-target tests needed 53 KB of RAM.
 *
 * The Mainline layer's value, whose reasoning carries over without its MPU
 * guard: idle peaks near 108 bytes there (arm-port-notes.md 18e), and an
 * ARMv6-M exception adds the same 32-byte hardware frame and 32 bytes of
 * PendSV stores. A RAM-starved M0 part could go lower, measured first.
 */
#define CYROS_PORT_MIN_FRAME 1024

/**
 * @def CYROS_PORT_CACHE_LINE
 * @brief Cache line size in bytes.
 *
 * Used for false-sharing avoidance. The M0 has no cache and this port is
 * single core, so nothing can false-share. 32 matches the Mainline ports'
 * value, for the same portability reason given there.
 */
#define CYROS_PORT_CACHE_LINE 32

/**
 * @def CYROS_PORT_CORE_COUNT
 * @brief Number of cores supported by this port.
 *
 * Single core. The dual-M0+ RP2040 is a target of its own, and supplies its
 * own atomics over a hardware lock (port_atomic_armv6m.cpp says why).
 */
#define CYROS_PORT_CORE_COUNT 1

/**
 * @def CYROS_PORT_CAPTURE_LOCATION
 * @brief Capture file and line information on assert.
 *
 * Turning this on can increase code-size as the compiler attaches
 * strings to every CYROS_ASSERT* location. On a flash-constrained part this is
 * the first thing to turn off, and the panic still reports both aux values.
 */
#define CYROS_PORT_CAPTURE_LOCATION 1

/**
 * @def CYROS_PORT_DEBUG_MODE
 * @brief Turn statically-unreachable tails into reported system errors.
 */
#define CYROS_PORT_DEBUG_MODE 1

#endif /* CYROS_PORT_TRAITS_H */
