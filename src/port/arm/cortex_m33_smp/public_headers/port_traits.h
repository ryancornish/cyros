/**
 * @file port_traits.h
 * @brief Cortex-M33 port compile-time traits.
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
 * Three words: the saved PSP, the thread's PSPLIM, and its TLS base. Rounded
 * to 16 for 8-byte alignment with a word to spare.
 *
 * Worth contrasting with the Linux preempt port's 512, which is dominated by a
 * pointer to a heap-allocated FP save area. Here the registers live on the
 * thread's own stack: the hardware stacks r0-r3/r12/LR/PC/xPSR on exception
 * entry and the PendSV prologue pushes r4-r11 beneath them. Nothing else has
 * to be recorded, so thread::min_stack_size drops accordingly.
 *
 * Hard float did not grow it. s16-s31 are saved on the thread's STACK, and
 * whether a thread has FP state is FType in the EXC_RETURN saved in its own
 * frame (arm-port-notes 11), so the word to spare is still spare.
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
 * thread.
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
 * Inside the kernel only the idle thread runs on exactly min_stack_size. On
 * the single-core cortex_m target its measured peak is 108 bytes, on both
 * architectures and both time drivers at -Og (2026-10-03, by stack scan after
 * 200 sleeps). It is unmeasured here: no dual-core test lets either core's
 * idle thread run, and here idle's yield can drain requests posted by the
 * other core, a path single core never takes. The thread's exception
 * frames are in that figure: idle never touches the FPU, so the hardware frame
 * is the basic 32 bytes, and PendSV pushes 36 below it. An FP thread's frames
 * are larger, 104 bytes for the extended frame plus 100 from PendSV.
 *
 * On ARMv7-M the MPU guard also takes its 128 bytes, plus up to 120 more to
 * align it, from the bottom of the buffer, though not on this ARMv8-M target.
 * 1024 leaves idle about ten times the single-core peak, and gives an
 * application a kilobyte as the smallest stack it can ask for.
 */
#define CYROS_PORT_MIN_FRAME 1024

/**
 * @def CYROS_PORT_CACHE_LINE
 * @brief Cache line size in bytes.
 *
 * Used for false-sharing avoidance. This target has two cores that genuinely
 * share memory, but an SSE-200 M33 has no data cache, so two cores writing
 * adjacent words contend for nothing and the padding buys no speed here. It is
 * kept at 32, the architectural line size of the M-profile parts that do have
 * a cache, because a structure laid out for a cached dual-core part stays
 * correct on one without a cache and the reverse is not true.
 */
#define CYROS_PORT_CACHE_LINE 32

/**
 * @def CYROS_PORT_CORE_COUNT
 * @brief Number of cores supported by this port.
 *
 * Two, which is what an SSE-200 subsystem provides and what QEMU's mps2-an521
 * models. The number is not a preference: this target's bring-up releases
 * exactly one secondary core through CPUWAIT and its doorbell has one register
 * pair per core, so a third would need different plumbing rather than a
 * different constant here.
 *
 * Raising this above 1 is what makes the core layer index its per-core state
 * by cyros_port_get_core_id rather than by a constant. See this_core() in
 * port_core_armv7m_armv8m.cpp.
 */
#define CYROS_PORT_CORE_COUNT 2

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
