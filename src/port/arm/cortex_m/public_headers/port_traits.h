/**
 * @file port_traits.h
 * @brief Generic single-core Cortex-M port (`cortex_m`) compile-time traits.
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
 * Three words: the saved PSP, the thread's stack-guard bound (PSPLIM on
 * ARMv8-M, the MPU guard region's base on ARMv7-M), and its TLS base. Rounded
 * to 16 for 8-byte alignment with a word to spare.
 *
 * Worth contrasting with the Linux preempt port's 512, which is dominated by a
 * pointer to a heap-allocated FP save area. Here the registers live on the
 * thread's own stack: the hardware stacks r0-r3/r12/LR/PC/xPSR on exception
 * entry and the PendSV prologue pushes r4-r11 beneath them. Nothing else has
 * to be recorded, so thread::min_stack_size drops accordingly.
 *
 * This grows when hard-float lands. Saving s16-s31 adds 64 bytes to the frame
 * on the STACK, not here, but a lazy-stacking design needs a flag word in the
 * context, and the spare word above is for that.
 */
#define CYROS_PORT_CONTEXT_SIZE  16

/**
 * @def CYROS_PORT_CONTEXT_ALIGN
 * @brief Alignment requirement of `port_context_t` in bytes.
 *
 * Must be a power of two.
 */
#define CYROS_PORT_CONTEXT_ALIGN 8

/**
 * @def CYROS_PORT_STACK_ALIGN
 * @brief Required alignment of all thread stacks in bytes.
 *
 * 8, not the 16 the Linux ports use. AAPCS requires the stack to be 8-byte
 * aligned at every public interface, and ARMv8-M's PSPLIM is 8-byte granular,
 * so 8 is both necessary and sufficient. 16 would be silent waste on every
 * thread. The ARMv7-M guard region needs 128, but the port rounds up to that
 * inside the buffer rather than imposing it on every stack.
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
 * Inside the kernel only the idle thread runs on exactly min_stack_size, and
 * its measured peak is 108 bytes, on both architectures and both time drivers
 * at -Og (2026-10-03, by stack scan after 200 sleeps). The thread's exception
 * frames are in that figure: idle never touches the FPU, so the hardware frame
 * is the basic 32 bytes, and PendSV pushes 36 below it. An FP thread's frames
 * are larger, 104 bytes for the extended frame plus 100 from PendSV.
 *
 * On ARMv7-M the MPU guard also takes its 128 bytes, plus up to 120 more to
 * align it, from the bottom of the buffer. At 1024 that still leaves idle
 * about eight times its peak, and it gives an application a kilobyte as the
 * smallest stack it can ask for. 512 would cover idle too. 256 would not, in
 * the worst alignment on ARMv7-M, although every test passes at 256 because
 * no test ever lets idle run.
 */
#define CYROS_PORT_MIN_FRAME 1024

/**
 * @def CYROS_PORT_CACHE_LINE
 * @brief Cache line size in bytes.
 *
 * Used for false-sharing avoidance. Neither the M33 nor the M4 has a data
 * cache, and this port is single core, so nothing here can actually
 * false-share. 32 is carried as the architectural line size of the M-profile
 * parts that do have one (the M7), which keeps padded structures portable
 * across the family. Dropping it to 4 would reclaim RAM and is a legitimate
 * change for a RAM-constrained build, but it should be a measured decision
 * rather than a default.
 */
#define CYROS_PORT_CACHE_LINE 32

/**
 * @def CYROS_PORT_CORE_COUNT
 * @brief Number of cores supported by this port.
 *
 * The parts this targets are single core. A dual-M33 (RP2350, or QEMU's
 * mps2-an521) is a separate port variant, not a different value here, because
 * it needs real inter-core plumbing this file cannot express.
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
