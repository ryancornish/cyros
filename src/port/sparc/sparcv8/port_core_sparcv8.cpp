/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file port_core_sparcv8.cpp
 * @brief SPARC V8 CORE LAYER (LEON3): all of port_core.h.
 *
 * Bench is QEMU's leon3_generic (one LEON3 or several). No board yet.
 * `~/cyros-claude/sparc-port-plan.md` has the measurements and the reasons.
 *
 *
 * The central decision: ALL SWITCHING HAPPENS IN ONE TRAP PATH
 * ============================================================
 * As on RISC-V and the R52. A reschedule is interrupt level 1, the lowest,
 * raised through the interrupt controller's force register (the target's).
 * It is taken only when PSR.PIL is 0, and every handler runs at PIL 1 or
 * above, so a reschedule pended inside an interrupt, nested or not, is taken
 * when the outermost one returns to its thread, never during it.
 * `cyros_port_thread_yield` is a software trap (`ta 9`), which enters the
 * same path synchronously: a force-register write crosses the bus and lands
 * some cycles later, so a yield built on it could return before the switch.
 *
 * The thread's road (port_core.h) enters the same path by its own trap,
 * `ta 10`. cyros_port_yield_to makes it inside the kernel's mask, a software
 * trap being taken whatever PIL says, with the two contexts in %o0 and %o1,
 * and the handler swaps the frames, as cyros_port_switch does, instead of
 * calling the kernel. So a thread is suspended by one kind of frame whichever
 * road it took, and either road resumes it: a frame cyros_port_yield_to saved
 * has PIL raised, so its thread resumes inside the kernel's mask and releases
 * it itself.
 *
 *
 * Register windows, and what a trap saves
 * =======================================
 * SPARC keeps a function's registers in a WINDOW: `save` moves to a fresh one
 * and `restore` moves back. QEMU's LEON3 has 8. One is always INVALID (WIM),
 * so a `save` into it traps (window overflow, trap 5) for software to spill
 * the oldest window to its owner's stack, and a `restore` into it traps
 * (underflow, trap 6) to load one back. Every window spills to the 64-byte
 * save area the ABI reserves at its own sp, so a thread's spilled windows
 * are on its own stack, wherever they spilled from.
 *
 * A trap moves to the window below the interrupted one WITHOUT checking WIM,
 * so the entry first makes that window valid if it was the invalid one (an
 * overflow by hand). Then it stores the frame on the interrupted thread's
 * stack: globals, Y, PSR, PC, nPC and the interrupted window's outs
 * (sparcv8.hpp has the layout). The interrupted window's locals and ins, and
 * every caller's window above it, stay where they are: in the register file,
 * or already spilled. A suspended thread is a pointer to its frame, so
 * cyros_port_switch is one store, as on RISC-V and the R52.
 *
 * The exit is where windows are settled:
 *  - No switch: if the interrupted window is still in the register file, the
 *    trap returns into it. If the handler's own calls spilled it, it is
 *    loaded back from its stack first (an underflow by hand), because `rett`
 *    into an invalid window with traps off is the error state.
 *  - A switch: the outgoing thread's windows still in the register file are
 *    stored to their stacks (a flush), then the incoming thread's interrupted
 *    window is loaded from its stack, and WIM marks the window above it
 *    invalid, so its callers come back by ordinary underflows.
 *
 * The YIELD flushes the yielding thread's windows at ENTRY, all of them. A
 * terminating thread leaves through a yield, and its stack is its joiner's to
 * reuse once the kernel says it has terminated, which is inside the trap. A
 * window still in the register file would be spilled onto that stack later,
 * by whatever the handler calls. Flushing first means nothing of the thread
 * is left to spill.
 *
 * Handlers run on the core's interrupt stack, the stack the core stood on
 * when it started its first thread, as on RISC-V. Its top reads ZERO before
 * the first thread, so a trap the port test raises during bring-up stays on
 * the stack it arrives on.
 *
 *
 * Nested interrupts
 * =================
 * SPARC prioritises its fifteen levels in hardware: level n is taken only
 * while PIL is below n. So a handler runs at its OWN level, PIL n, and every
 * level above it may interrupt it, while its own and those below wait for it
 * to return. The reschedule, and the yield that stands in for it, run at
 * level 1, so the kernel's reschedule is interruptible by every device and by
 * the timer, as PendSV is on Cortex-M. A fault runs at 15, as everything did
 * before nesting: it is the application's hook, and usually a panic.
 *
 * Only the outermost trap's frame is a thread's, and only it moves to the
 * interrupt stack. A nested trap interrupted a handler, which is on that
 * stack already, so its frame lands below that handler's sp and its handler
 * runs below that, never on a thread's stack and never over the handler it
 * interrupted. The interrupt stack is sized for the deepest nesting: one
 * frame and one handler's calls per level in use. A nested trap may still
 * spill a THREAD's window, to that window's own save area, which the ABI
 * reserves above the thread's sp and every handler's calls already use.
 *
 * Which trap is the outermost is the core's DEPTH, and the entry counts it,
 * records the outermost frame as the thread's, and picks the stack, all with
 * traps still off. The exit counts it back down with traps off again. So no
 * trap can arrive between a trap being taken and its being counted, which is
 * what lets one nest anywhere traps are on: in the handler, in the dispatch
 * around it, in the entry's FP save, and in the exit's FP restore. Level 15,
 * which no PIL masks, can arrive in all of those.
 *
 * The trap path's own code still runs at PIL 15, and lowers it to the
 * handler's level only around the handler. That is not for the count, which
 * is safe with traps on, but for the exit's traps-off write: WRPSR is delayed
 * by up to three instructions, so an interrupt could be taken just after the
 * write that clears ET, and its rett would set ET again under the exit's
 * window handling. At 15 only level 15 can. QEMU applies WRPSR at once, so no
 * test here can show that.
 *
 *
 * Masking: two fields of one PIL
 * ==============================
 *
 *   Interrupt masking  (cyros_port_irq_save/restore)       -> PIL bits 1 to 3
 *   Preemption control (cyros_port_preempt_disable/enable) -> PIL bit 0
 *
 * PSR.PIL masks every interrupt at or below its value, and level 15 is
 * non-maskable. So PIL 14 (bits 1 to 3) masks every maskable level, PIL 1
 * (bit 0) masks the reschedule alone, and the two are independent bit fields:
 * 0 open, 1 preempt-disabled, 14 masked, 15 both, with 14 and 15 masking
 * exactly the same interrupts. Each TOKEN is its own field's previous bits,
 * and a restore rewrites only its own field, so regions compose in any order
 * without memory, as PRIMASK and BASEPRI do. They compose inside a handler
 * too, at whatever bits its level n has: an irq_save there takes PIL to 14
 * or 15 and its restore gives back n's bits 1 to 3, and a preempt-disable at
 * an even level masks the level above it too while it is held, which only
 * delays that level.
 *
 * Baseline priority is PIL 0 outside a trap. WRPSR is delayed by up to three
 * instructions, so every write is followed by three nops: a raised PIL is in
 * force before the region it guards, and a lowered one has let a pending
 * reschedule in before the call returns. Traps are never disabled (PSR.ET) to
 * mask: a trap with ET clear, a window trap included, is the error state.
 *
 *
 * TLS is %g7
 * ==========
 * The SPARC ABI reserves %g7 for the thread pointer, and the frame saves the
 * globals, so it follows its thread with no code at all, as tp on RISC-V.
 *
 *
 * Floating point: eager, in every frame
 * =====================================
 * In a hard-float build every frame has room for FSR and f0 to f31, saved
 * when the interrupted PSR has EF and restored when the resumed one does: a
 * handler's compiled code may use any FP register. The board turns EF on
 * before any compiled code, and every thread starts with it. A soft-float
 * build compiles none of it.
 *
 *
 * No stack guard
 * ==============
 * LEON3 has no MPU. Its watchpoints (ASR24 to 31) would make a cheap one, but
 * QEMU does not model them (`rd %asr24` is `rd %y` there), so no guard can be
 * proven here, and by policy there is none rather than a software proxy.
 */

#include <cyros/port/port_core.h>

/* For cyros_port_get_core_id, as in the other layers: a core layer may call
 * the MCU contract, it may not implement it. */
#include <cyros/port/port_mcu.h>

#include "sparcv8.hpp"

#include <cstddef>
#include <cstdint>

namespace sparcv8 = cyros::port::sparcv8;

#define CYROS_SPARC_STR_(x) #x
#define CYROS_SPARC_STR(x)  CYROS_SPARC_STR_(x)
#define CYROS_SPARC_NWIN    CYROS_SPARC_STR(CYROS_SPARC_NWINDOWS)
#define CYROS_SPARC_NWIN_1  CYROS_SPARC_STR(CYROS_SPARC_NWINDOWS - 1)
#define CYROS_SPARC_OFF(x)  CYROS_SPARC_STR(x)

/* ============================================================================
 * Context and per-core state
 * ========================================================================= */

/** A suspended thread's port state: where its frame is. Everything else is
 * in that frame, or in the window save areas on its stack. */
struct cyros_port_context
{
   std::uint32_t* frame;
};

static_assert(sizeof(cyros_port_context) <= CYROS_PORT_CONTEXT_SIZE,
              "CYROS_PORT_CONTEXT_SIZE in port_traits.h is too small for this port");
static_assert(alignof(cyros_port_context) <= CYROS_PORT_CONTEXT_ALIGN,
              "CYROS_PORT_CONTEXT_ALIGN in port_traits.h is too weak for this port");

/**
 * @brief What each core's trap path keeps. The assembly reads and writes the
 *        first three words, so the layout is fixed (sparcv8.hpp's offsets).
 *
 * `frame` is the RUNNING thread's frame while a trap is in progress, which
 * cyros_port_switch replaces with the incoming thread's and the outermost
 * exit then resumes. `irq_stack` is the top of the stack the outermost trap
 * moves to, zero before the first thread. `depth` is how many traps are in
 * progress, nested ones included, and only the entry and exit change it,
 * with traps off ("Nested interrupts"). No core touches another's.
 */
struct cyros_sparc_core_state
{
   std::uint32_t* frame;
   std::uintptr_t irq_stack;
   std::uint32_t  depth;
   std::uint32_t  unused;   /* the stride the entry indexes by */
};

static_assert(offsetof(cyros_sparc_core_state, frame) == CYROS_SPARC_CORE_FRAME);
static_assert(offsetof(cyros_sparc_core_state, irq_stack) == CYROS_SPARC_CORE_IRQ_STACK);
static_assert(offsetof(cyros_sparc_core_state, depth) == CYROS_SPARC_CORE_DEPTH);
static_assert(sizeof(cyros_sparc_core_state) == (1u << CYROS_SPARC_CORE_STRIDE_LOG2));

extern "C" { cyros_sparc_core_state cyros_sparc_cores[CYROS_PORT_CORE_COUNT] = {}; }

namespace
{

cyros_port_reschedule_t reschedule_handler = nullptr;

inline std::uint32_t this_core() noexcept
{
   if constexpr (CYROS_PORT_CORE_COUNT == 1) {
      return 0u;
   } else {
      return cyros_port_get_core_id();
   }
}

[[noreturn]] void thread_return_trap()
{
   cyros_port_system_error(0, 0, "thread entry returned", 0);
}

/* The reschedule, from inside the trap. The kernel's handler may call
 * cyros_port_switch, which replaces `frame`. */
CYROS_PORT_HOT void run_reschedule(cyros_sparc_core_state& state)
{
   CYROS_ASSERT_OP(state.depth, ==, 1u);   /* never from inside a handler */
   CYROS_ASSERT(reschedule_handler != nullptr);
   reschedule_handler();
}

void write_hex(std::uint32_t value) noexcept
{
   char buffer[11] = { '0', 'x' };
   for (int i = 0; i < 8; ++i) {
      std::uint32_t const nibble = (value >> ((7 - i) * 4)) & 0xFu;
      buffer[2 + i] = static_cast<char>(nibble < 10 ? '0' + nibble : 'a' + (nibble - 10));
   }
   buffer[10] = '\0';
   sparcv8::console_write(buffer);
}

/* The PIL fields (the file comment), in place in the PSR. */
constexpr std::uint32_t pil_irq_bits     = sparcv8::pil_masked << sparcv8::psr_pil_shift;
constexpr std::uint32_t pil_preempt_bits = sparcv8::pil_preempt << sparcv8::psr_pil_shift;
static_assert((pil_irq_bits & pil_preempt_bits) == 0u, "the two grades are disjoint fields of PIL");
static_assert((pil_irq_bits | pil_preempt_bits) == sparcv8::psr_pil_mask, "and together they are all of it");

/* Set `bits` in the PSR, returning the previous PSR. One read and one write,
 * nothing between that can trap. An interrupt between the two returns with
 * this thread's PSR as it was, so the write is still right. */
inline std::uint32_t psr_set(std::uint32_t bits) noexcept
{
   std::uint32_t previous, next;
   asm volatile("rd   %%psr, %0         \n"
                "or   %0, %2, %1        \n"
                "wr   %1, %%psr         \n"
                "nop                    \n"
                "nop                    \n"
                "nop                    \n"
                : "=&r"(previous), "=&r"(next)
                : "r"(bits)
                : "memory", "cc");
   return previous;
}

/* Replace the PSR's `field` with `value`'s bits of it. An interrupt between
 * the read and the write returns with this context's PSR as it was. */
inline void psr_restore(std::uint32_t field, std::uint32_t value) noexcept
{
   std::uint32_t current, next;
   asm volatile("rd   %%psr, %0         \n"
                "andn %0, %2, %0        \n"
                "and  %3, %2, %1        \n"
                "or   %0, %1, %1        \n"
                "wr   %1, %%psr         \n"
                "nop                    \n"
                "nop                    \n"
                "nop                    \n"
                : "=&r"(current), "=&r"(next)
                : "r"(field), "r"(value)
                : "memory", "cc");
}

/* PIL := `level`, both fields at once: the trap path's, around a handler.
 * Lowered, any pending level above it is taken within the write's nops. */
inline void set_pil(std::uint32_t level) noexcept
{
   psr_restore(sparcv8::psr_pil_mask, level << sparcv8::psr_pil_shift);
}

} // namespace


/* ============================================================================
 * The trap table and the window traps
 * ----------------------------------------------------------------------------
 * Each entry is four instructions: the PSR into %l0 and the trap type into
 * %l3, then a jump. The hardware has already put PC and nPC in %l1 and %l2 and
 * moved to the trap window with traps disabled.
 * ========================================================================= */

#define CYROS_SPARC_ENTRY(handler, tt)                                          \
   "rd %psr, %l0; sethi %hi(" handler "), %l4; jmp %l4 + %lo(" handler "); mov " tt ", %l3 \n"

asm(
   ".pushsection .text.cyros_sparc_trap_table, \"ax\"               \n"
   ".balign 4096                                                    \n"
   ".global cyros_sparc_trap_table                                  \n"
   ".type   cyros_sparc_trap_table, #function                       \n"
   "cyros_sparc_trap_table:                                         \n"
   ".set cyros_sparc_tt, 0                                          \n"
   ".rept 256                                                       \n"
   ".if cyros_sparc_tt == 5                                         \n"
   CYROS_SPARC_ENTRY("cyros_sparc_window_overflow", "cyros_sparc_tt")
   ".elseif cyros_sparc_tt == 6                                     \n"
   CYROS_SPARC_ENTRY("cyros_sparc_window_underflow", "cyros_sparc_tt")
   ".elseif cyros_sparc_tt == 0x80 + " CYROS_SPARC_STR(CYROS_SPARC_TA_FLUSH_WINDOWS) " \n"
   CYROS_SPARC_ENTRY("cyros_sparc_flush_windows", "cyros_sparc_tt")
   ".elseif cyros_sparc_tt == 0x80 + " CYROS_SPARC_STR(CYROS_SPARC_TA_YIELD) " \n"
   CYROS_SPARC_ENTRY("cyros_sparc_yield_entry", "cyros_sparc_tt")
   ".elseif cyros_sparc_tt == 0x80 + " CYROS_SPARC_STR(CYROS_SPARC_TA_YIELD_TO) " \n"
   CYROS_SPARC_ENTRY("cyros_sparc_yield_entry", "cyros_sparc_tt")
   ".else                                                           \n"
   CYROS_SPARC_ENTRY("cyros_sparc_trap_entry", "cyros_sparc_tt")
   ".endif                                                          \n"
   ".set cyros_sparc_tt, cyros_sparc_tt + 1                         \n"
   ".endr                                                           \n"
   ".size   cyros_sparc_trap_table, . - cyros_sparc_trap_table      \n"
   ".popsection                                                     \n");

asm(
   ".pushsection .text.cyros_sparc_traps, \"ax\"                   \n"
   ".balign 4                                                       \n"

   /* Window overflow: a `save` met the invalid window, and this trap runs in
    * it. Store the window below it, the oldest valid one, to its own stack,
    * mark that one invalid instead, and re-execute the save. */
   ".global cyros_sparc_window_overflow                             \n"
   ".type   cyros_sparc_window_overflow, #function                  \n"
   "cyros_sparc_window_overflow:                                    \n"
   "   rd    %wim, %l3                                              \n"
   "   mov   %g1, %l7                                               \n"
   "   srl   %l3, 1, %g1                                            \n"
   "   sll   %l3, " CYROS_SPARC_NWIN_1 ", %l4                       \n"
   "   or    %l4, %g1, %g1                                          \n"
   "   save                                                         \n"
   "   wr    %g1, %wim                                              \n"
   "   std   %l0, [%sp + 0]                                         \n"
   "   std   %l2, [%sp + 8]                                         \n"
   "   std   %l4, [%sp + 16]                                        \n"
   "   std   %l6, [%sp + 24]                                        \n"
   "   std   %i0, [%sp + 32]                                        \n"
   "   std   %i2, [%sp + 40]                                        \n"
   "   std   %i4, [%sp + 48]                                        \n"
   "   std   %i6, [%sp + 56]                                        \n"
   "   restore                                                      \n"
   "   mov   %l7, %g1                                               \n"
   "   jmp   %l1                                                    \n"
   "   rett  %l2                                                    \n"
   ".size cyros_sparc_window_overflow, . - cyros_sparc_window_overflow \n"

   /* Window underflow: a `restore` (or rett) met the invalid window W, and
    * this trap runs two below it. Mark the window above W invalid, load W
    * from its stack, and re-execute the restore. */
   ".global cyros_sparc_window_underflow                            \n"
   ".type   cyros_sparc_window_underflow, #function                 \n"
   "cyros_sparc_window_underflow:                                   \n"
   "   rd    %wim, %l3                                              \n"
   "   sll   %l3, 1, %l4                                            \n"
   "   srl   %l3, " CYROS_SPARC_NWIN_1 ", %l5                       \n"
   "   or    %l5, %l4, %l5                                          \n"
   "   wr    %l5, %wim                                              \n"
   "   nop                                                          \n"
   "   nop                                                          \n"
   "   nop                                                          \n"
   "   restore                                                      \n"
   "   restore                                                      \n"
   "   ldd   [%sp + 0], %l0                                         \n"
   "   ldd   [%sp + 8], %l2                                         \n"
   "   ldd   [%sp + 16], %l4                                        \n"
   "   ldd   [%sp + 24], %l6                                        \n"
   "   ldd   [%sp + 32], %i0                                        \n"
   "   ldd   [%sp + 40], %i2                                        \n"
   "   ldd   [%sp + 48], %i4                                        \n"
   "   ldd   [%sp + 56], %i6                                        \n"
   "   save                                                         \n"
   "   save                                                         \n"
   "   jmp   %l1                                                    \n"
   "   rett  %l2                                                    \n"
   ".size cyros_sparc_window_underflow, . - cyros_sparc_window_underflow \n"

   /* `ta 3`, GCC's flush: store every window above the caller's to its stack
    * and mark the one above the caller invalid, so each comes back by an
    * underflow. The caller's own window (the trap window's T+1) stays. */
   ".global cyros_sparc_flush_windows                               \n"
   ".type   cyros_sparc_flush_windows, #function                    \n"
   "cyros_sparc_flush_windows:                                      \n"
   "   mov   %g1, %l3                                               \n"
   "   mov   %g2, %l4                                               \n"
   "   mov   %g3, %l5                                               \n"
   "   mov   %g4, %l6                                               \n"
   "   mov   %l0, %g4                                               \n"   /* the PSR, CWP T, visible from any window */
   "   rd    %wim, %g2                                              \n"
   "   and   %l0, 0x1f, %g1                                         \n"
   "   add   %g1, 1, %g1                                            \n"   /* T+1, the caller's */
   "   cmp   %g1, " CYROS_SPARC_NWIN "                              \n"
   "   bge,a 1f                                                     \n"
   "    mov  0, %g1                                                 \n"
   "1: restore                                                      \n"   /* into T+1, kept */
   "2: add   %g1, 1, %g1                                            \n"
   "   cmp   %g1, " CYROS_SPARC_NWIN "                              \n"
   "   bge,a 3f                                                     \n"
   "    mov  0, %g1                                                 \n"
   "3: mov   1, %g3                                                 \n"
   "   sll   %g3, %g1, %g3                                          \n"
   "   andcc %g3, %g2, %g0                                          \n"
   "   bne   4f                                                     \n"   /* the invalid one: done */
   "    nop                                                         \n"
   "   restore                                                      \n"
   "   std   %l0, [%sp + 0]                                         \n"
   "   std   %l2, [%sp + 8]                                         \n"
   "   std   %l4, [%sp + 16]                                        \n"
   "   std   %l6, [%sp + 24]                                        \n"
   "   std   %i0, [%sp + 32]                                        \n"
   "   std   %i2, [%sp + 40]                                        \n"
   "   std   %i4, [%sp + 48]                                        \n"
   "   std   %i6, [%sp + 56]                                        \n"
   "   ba    2b                                                     \n"
   "    nop                                                         \n"
   "4: wr    %g4, %psr                                              \n"   /* back to T, ET still clear */
   "   nop                                                          \n"
   "   nop                                                          \n"
   "   nop                                                          \n"
   "   and   %g4, 0x1f, %g1                                         \n"
   "   add   %g1, 2, %g1                                            \n"
   "   cmp   %g1, " CYROS_SPARC_NWIN "                              \n"
   "   bge,a 5f                                                     \n"
   "    sub  %g1, " CYROS_SPARC_NWIN ", %g1                         \n"
   "5: mov   1, %g3                                                 \n"
   "   sll   %g3, %g1, %g3                                          \n"
   "   wr    %g3, %wim                                              \n"   /* T+2 invalid */
   "   mov   %l3, %g1                                               \n"
   "   mov   %l4, %g2                                               \n"
   "   mov   %l5, %g3                                               \n"
   "   mov   %l6, %g4                                               \n"
   "   jmp   %l2                                                    \n"   /* past the ta */
   "   rett  %l2 + 4                                                \n"
   ".size cyros_sparc_flush_windows, . - cyros_sparc_flush_windows  \n"
   ".popsection                                                     \n");


/* ============================================================================
 * The trap path
 * ========================================================================= */

#if CYROS_PORT_CORE_COUNT > 1
/* %l7 := this core's cyros_sparc_cores entry, from ASR17.INDEX. */
#define CYROS_SPARC_THIS_CORE_STATE                                             \
   "   rd    %asr17, %l7                                                \n"     \
   "   srl   %l7, 28, %l7                                               \n"     \
   "   sll   %l7, " CYROS_SPARC_OFF(CYROS_SPARC_CORE_STRIDE_LOG2) ", %l7 \n"    \
   "   set   cyros_sparc_cores, %g1                                     \n"     \
   "   add   %g1, %l7, %l7                                              \n"
#else
#define CYROS_SPARC_THIS_CORE_STATE                                             \
   "   set   cyros_sparc_cores, %l7                                     \n"
#endif

#if CYROS_SPARC_HAS_FPU
/* FSR and f0 to f31 into the frame at %l6, when the trapped PSR (%l0) had the
 * FPU on. Traps are on by now, so an FP exception the store completes is a
 * fault through the trap path rather than the error state. */
#define CYROS_SPARC_FP_SAVE                                                     \
   "   set   0x1000, %g1                                                \n"     \
   "   andcc %l0, %g1, %g0                                              \n"     \
   "   be    7f                                                         \n"     \
   "    nop                                                             \n"     \
   "   st    %fsr, [%l6 + " CYROS_SPARC_OFF(CYROS_SPARC_F_FSR) "]       \n"     \
   "   std   %f0,  [%l6 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 0) "]    \n"     \
   "   std   %f2,  [%l6 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 8) "]    \n"     \
   "   std   %f4,  [%l6 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 16) "]   \n"     \
   "   std   %f6,  [%l6 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 24) "]   \n"     \
   "   std   %f8,  [%l6 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 32) "]   \n"     \
   "   std   %f10, [%l6 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 40) "]   \n"     \
   "   std   %f12, [%l6 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 48) "]   \n"     \
   "   std   %f14, [%l6 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 56) "]   \n"     \
   "   std   %f16, [%l6 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 64) "]   \n"     \
   "   std   %f18, [%l6 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 72) "]   \n"     \
   "   std   %f20, [%l6 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 80) "]   \n"     \
   "   std   %f22, [%l6 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 88) "]   \n"     \
   "   std   %f24, [%l6 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 96) "]   \n"     \
   "   std   %f26, [%l6 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 104) "]  \n"     \
   "   std   %f28, [%l6 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 112) "]  \n"     \
   "   std   %f30, [%l6 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 120) "]  \n"     \
   "7:                                                                  \n"
/* The resumed frame's (%l5) FP state, when its PSR has the FPU on. */
#define CYROS_SPARC_FP_RESTORE                                                  \
   "   ld    [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_PSR) "], %g1        \n"     \
   "   set   0x1000, %g2                                                \n"     \
   "   andcc %g1, %g2, %g0                                              \n"     \
   "   be    7f                                                         \n"     \
   "    nop                                                             \n"     \
   /* The FPU on now, whatever the trapped thread had: the incoming one     \
    * may have it where the outgoing one did not. */                        \
   "   rd    %psr, %g1                                                  \n"     \
   "   or    %g1, %g2, %g1                                              \n"     \
   "   wr    %g1, %psr                                                  \n"     \
   "   nop                                                              \n"     \
   "   nop                                                              \n"     \
   "   nop                                                              \n"     \
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 0) "], %f0     \n"     \
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 8) "], %f2     \n"     \
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 16) "], %f4    \n"     \
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 24) "], %f6    \n"     \
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 32) "], %f8    \n"     \
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 40) "], %f10   \n"     \
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 48) "], %f12   \n"     \
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 56) "], %f14   \n"     \
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 64) "], %f16   \n"     \
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 72) "], %f18   \n"     \
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 80) "], %f20   \n"     \
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 88) "], %f22   \n"     \
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 96) "], %f24   \n"     \
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 104) "], %f26  \n"     \
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 112) "], %f28  \n"     \
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_F0 + 120) "], %f30  \n"     \
   "   ld    [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_FSR) "], %fsr       \n"     \
   "7:                                                                  \n"
#else
#define CYROS_SPARC_FP_SAVE    ""
#define CYROS_SPARC_FP_RESTORE ""
#endif

/* (CWP + n) mod NWINDOWS of the window number in `reg`, in place. */
#define CYROS_SPARC_WINDOW_UP(reg, n, label)                                    \
   "   add   " reg ", " #n ", " reg "                                     \n"   \
   "   cmp   " reg ", " CYROS_SPARC_NWIN "                                \n"   \
   "   bge,a " label "f                                                   \n"   \
   "    sub  " reg ", " CYROS_SPARC_NWIN ", " reg "                       \n"   \
   label ":                                                               \n"

asm(
   ".pushsection .text.cyros_sparc_traps, \"ax\"                   \n"
   ".balign 4                                                       \n"

   /* The yield, `ta 9`, and cyros_port_yield_to's `ta 10`: every window of
    * the yielding thread to its stack first (the file comment), then the
    * common entry. Leaves window T+1 invalid, so the exit always loads it back
    * from the stack. */
   ".global cyros_sparc_yield_entry                                 \n"
   ".type   cyros_sparc_yield_entry, #function                      \n"
   "cyros_sparc_yield_entry:                                        \n"
   "   mov   %g1, %l4                                               \n"
   "   mov   %g2, %l5                                               \n"
   "   mov   %g3, %l6                                               \n"
   "   mov   %l0, %g3                                               \n"   /* the PSR, CWP T */
   "   rd    %wim, %g2                                              \n"
   "   and   %l0, 0x1f, %g1                                         \n"   /* T */
   "1:                                                              \n"
   CYROS_SPARC_WINDOW_UP("%g1", 1, "2")
   /* %l7 is scratch in each window the walk visits: T's is free, and every
    * other window's has been stored by the time the next lap reads it. */
   "   mov   1, %l7                                                 \n"
   "   sll   %l7, %g1, %l7                                          \n"
   "   andcc %l7, %g2, %g0                                          \n"
   "   bne   3f                                                     \n"
   "    nop                                                         \n"
   "   restore                                                      \n"
   "   std   %l0, [%sp + 0]                                         \n"
   "   std   %l2, [%sp + 8]                                         \n"
   "   std   %l4, [%sp + 16]                                        \n"
   "   std   %l6, [%sp + 24]                                        \n"
   "   std   %i0, [%sp + 32]                                        \n"
   "   std   %i2, [%sp + 40]                                        \n"
   "   std   %i4, [%sp + 48]                                        \n"
   "   std   %i6, [%sp + 56]                                        \n"
   "   ba    1b                                                     \n"
   "    nop                                                         \n"
   "3: wr    %g3, %psr                                              \n"   /* back to T, ET still clear */
   "   nop                                                          \n"
   "   nop                                                          \n"
   "   nop                                                          \n"
   "   and   %g3, 0x1f, %g1                                         \n"
   CYROS_SPARC_WINDOW_UP("%g1", 1, "4")
   "   mov   1, %g2                                                 \n"
   "   sll   %g2, %g1, %g2                                          \n"
   "   wr    %g2, %wim                                              \n"   /* T+1 invalid: nothing of the thread is left */
   "   mov   %l4, %g1                                               \n"
   "   mov   %l5, %g2                                               \n"
   "   mov   %l6, %g3                                               \n"
   /* Fall into the common entry, whose WIM check now finds T valid. */

   /* Every trap but the window traps and the flush: an interrupt, the yield,
    * a fault. %l0 PSR, %l1 PC, %l2 nPC, %l3 the trap type. */
   ".global cyros_sparc_trap_entry                                  \n"
   ".type   cyros_sparc_trap_entry, #function                       \n"
   "cyros_sparc_trap_entry:                                         \n"
   "   mov   %g1, %l4                                               \n"
   "   mov   %g2, %l5                                               \n"
   /* 1. If the trap window is the invalid one, spill the window below it,
    *    the oldest valid, as the overflow trap would. WIM >> CWP is 1 exactly
    *    then, since WIM has one bit set. */
   "   rd    %wim, %g1                                              \n"
   "   srl   %g1, %l0, %g2                                          \n"
   "   cmp   %g2, 1                                                 \n"
   "   bne   1f                                                     \n"
   "    sll  %g1, " CYROS_SPARC_NWIN_1 ", %g2                       \n"
   "   srl   %g1, 1, %g1                                            \n"
   "   or    %g1, %g2, %g1                                          \n"
   "   save                                                         \n"
   "   wr    %g1, %wim                                              \n"
   "   std   %l0, [%sp + 0]                                         \n"
   "   std   %l2, [%sp + 8]                                         \n"
   "   std   %l4, [%sp + 16]                                        \n"
   "   std   %l6, [%sp + 24]                                        \n"
   "   std   %i0, [%sp + 32]                                        \n"
   "   std   %i2, [%sp + 40]                                        \n"
   "   std   %i4, [%sp + 48]                                        \n"
   "   std   %i6, [%sp + 56]                                        \n"
   "   restore                                                      \n"
   /* 2. The frame, below the interrupted sp, which is the trap window's fp. */
   "1: sub   %fp, " CYROS_SPARC_OFF(CYROS_SPARC_F_BYTES) ", %sp     \n"
   "   std   %l0, [%sp + " CYROS_SPARC_OFF(CYROS_SPARC_F_PSR) "]    \n"   /* PSR, PC */
   "   rd    %y, %g1                                                \n"
   "   st    %l2, [%sp + " CYROS_SPARC_OFF(CYROS_SPARC_F_NPC) "]    \n"
   "   st    %g1, [%sp + " CYROS_SPARC_OFF(CYROS_SPARC_F_Y) "]      \n"
   "   st    %g0, [%sp + " CYROS_SPARC_OFF(CYROS_SPARC_F_G0) "]     \n"
   "   st    %l4, [%sp + " CYROS_SPARC_OFF(CYROS_SPARC_F_G1) "]     \n"
   "   st    %l5, [%sp + " CYROS_SPARC_OFF(CYROS_SPARC_F_G2) "]     \n"
   "   st    %g3, [%sp + " CYROS_SPARC_OFF(CYROS_SPARC_F_G2 + 4) "] \n"
   "   std   %g4, [%sp + " CYROS_SPARC_OFF(CYROS_SPARC_F_G4) "]     \n"
   "   std   %g6, [%sp + " CYROS_SPARC_OFF(CYROS_SPARC_F_G6) "]     \n"
   "   std   %i0, [%sp + " CYROS_SPARC_OFF(CYROS_SPARC_F_O0) "]     \n"
   "   std   %i2, [%sp + " CYROS_SPARC_OFF(CYROS_SPARC_F_O2) "]     \n"
   "   std   %i4, [%sp + " CYROS_SPARC_OFF(CYROS_SPARC_F_O4) "]     \n"
   "   std   %i6, [%sp + " CYROS_SPARC_OFF(CYROS_SPARC_F_O6) "]     \n"
   "   mov   %sp, %l6                                               \n"   /* the entered frame */
   /* 3. Count the trap, traps still off. The outermost one's frame is the
    *    thread's, and it moves to the interrupt stack, if there is one yet.
    *    A nested one stays below the handler it interrupted, which is on that
    *    stack already, so it never lands on a thread's. %l7 keeps the core's
    *    state for the exit. */
   CYROS_SPARC_THIS_CORE_STATE
   "   ld    [%l7 + " CYROS_SPARC_OFF(CYROS_SPARC_CORE_DEPTH) "], %g2 \n"
   "   add   %g2, 1, %g1                                            \n"
   "   st    %g1, [%l7 + " CYROS_SPARC_OFF(CYROS_SPARC_CORE_DEPTH) "] \n"
   "   cmp   %g2, 0                                                 \n"
   "   bne   2f                                                     \n"   /* nested */
   "    mov  %l6, %g1                                               \n"   /* (either way) below the frame */
   "   st    %l6, [%l7 + " CYROS_SPARC_OFF(CYROS_SPARC_CORE_FRAME) "] \n"
   "   ld    [%l7 + " CYROS_SPARC_OFF(CYROS_SPARC_CORE_IRQ_STACK) "], %g2 \n"
   "   cmp   %g2, 0                                                 \n"
   "   bne,a 2f                                                     \n"
   "    mov  %g2, %g1                                               \n"   /* (taken only) the interrupt stack */
   "2: sub   %g1, 96, %sp                                           \n"
   /* 4. PIL 15 and traps on: window traps happen in the C below, and a
    *    level 15 may nest from here. */
   "   or    %l0, 0xf00, %g1                                        \n"
   "   wr    %g1, 0x20, %psr                                        \n"
   "   nop                                                          \n"
   "   nop                                                          \n"
   "   nop                                                          \n"
   CYROS_SPARC_FP_SAVE
   "   mov   %l6, %o0                                               \n"
   "   call  cyros_sparc_trap_dispatch                              \n"
   "    mov  %l3, %o1                                               \n"
   /* 5. The frame to resume, maybe another thread's. */
   "   mov   %o0, %l5                                               \n"
   CYROS_SPARC_FP_RESTORE
   "   rd    %psr, %g1                                              \n"
   "   andn  %g1, 0x20, %g1                                         \n"
   "   wr    %g1, %psr                                              \n"   /* traps off until rett */
   "   nop                                                          \n"
   "   nop                                                          \n"
   "   nop                                                          \n"
   "   ld    [%l7 + " CYROS_SPARC_OFF(CYROS_SPARC_CORE_DEPTH) "], %g2 \n"   /* uncounted, now nothing can nest */
   "   sub   %g2, 1, %g2                                            \n"
   "   st    %g2, [%l7 + " CYROS_SPARC_OFF(CYROS_SPARC_CORE_DEPTH) "] \n"
   "   and   %g1, 0x1f, %g3                                         \n"   /* g3 = T */
   "   rd    %wim, %g2                                              \n"
   "   cmp   %l5, %l6                                               \n"
   "   be    5f                                                     \n"
   "    nop                                                         \n"
   /* 6. A switch: the outgoing thread's windows still in the register file,
    *    from T+1 up to the invalid one, to their stacks. */
   "   mov   %g3, %g4                                               \n"
   "3:                                                              \n"
   CYROS_SPARC_WINDOW_UP("%g4", 1, "33")
   "   mov   1, %g5                                                 \n"
   "   sll   %g5, %g4, %g5                                          \n"
   "   andcc %g5, %g2, %g0                                          \n"
   "   bne   4f                                                     \n"
   "    nop                                                         \n"
   "   restore                                                      \n"
   "   std   %l0, [%sp + 0]                                         \n"
   "   std   %l2, [%sp + 8]                                         \n"
   "   std   %l4, [%sp + 16]                                        \n"
   "   std   %l6, [%sp + 24]                                        \n"
   "   std   %i0, [%sp + 32]                                        \n"
   "   std   %i2, [%sp + 40]                                        \n"
   "   std   %i4, [%sp + 48]                                        \n"
   "   std   %i6, [%sp + 56]                                        \n"
   "   ba    3b                                                     \n"
   "    nop                                                         \n"
   "4: wr    %g1, %psr                                              \n"   /* back to T */
   "   nop                                                          \n"
   "   nop                                                          \n"
   "   nop                                                          \n"
   "   ba    cyros_sparc_trap_fill                                  \n"
   "    nop                                                         \n"
   /* 7. No switch: is the interrupted window still in the register file? */
   "5: mov   %g3, %g4                                               \n"
   CYROS_SPARC_WINDOW_UP("%g4", 1, "55")
   "   mov   1, %g5                                                 \n"
   "   sll   %g5, %g4, %g5                                          \n"
   "   andcc %g5, %g2, %g0                                          \n"
   "   be    cyros_sparc_trap_resume                                \n"
   "    nop                                                         \n"
   /* 8. Load window T+1 from the resumed thread's stack and make T+2 the
    *    invalid one. Entered with traps off, in T (whose number is in %g3),
    *    with the frame in %l5. cyros_port_start_first enters here too. */
   ".global cyros_sparc_trap_fill                                   \n"
   "cyros_sparc_trap_fill:                                          \n"
   "   mov   %g3, %g4                                               \n"
   CYROS_SPARC_WINDOW_UP("%g4", 2, "88")
   "   mov   1, %g5                                                 \n"
   "   sll   %g5, %g4, %g5                                          \n"
   "   wr    %g5, %wim                                              \n"
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_O0) "], %i0     \n"
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_O2) "], %i2     \n"
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_O4) "], %i4     \n"
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_O6) "], %i6     \n"
   "   restore                                                      \n"   /* into T+1, whose sp is the thread's */
   "   ldd   [%sp + 0], %l0                                         \n"
   "   ldd   [%sp + 8], %l2                                         \n"
   "   ldd   [%sp + 16], %l4                                        \n"
   "   ldd   [%sp + 24], %l6                                        \n"
   "   ldd   [%sp + 32], %i0                                        \n"
   "   ldd   [%sp + 40], %i2                                        \n"
   "   ldd   [%sp + 48], %i4                                        \n"
   "   ldd   [%sp + 56], %i6                                        \n"
   "   save                                                         \n"   /* back to T */
   /* 9. The rest of the frame, then into window T+1 with traps on. */
   ".global cyros_sparc_trap_resume                                 \n"
   "cyros_sparc_trap_resume:                                        \n"
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_O0) "], %i0     \n"
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_O2) "], %i2     \n"
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_O4) "], %i4     \n"
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_O6) "], %i6     \n"
   "   ld    [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_Y) "], %g1      \n"
   "   wr    %g1, %y                                                \n"
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_G2) "], %g2     \n"
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_G4) "], %g4     \n"
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_G6) "], %g6     \n"
   "   ldd   [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_PSR) "], %l0    \n"   /* PSR, PC */
   "   ld    [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_NPC) "], %l2    \n"
   /* The thread's PSR with this window's CWP and traps still off. */
   "   rd    %psr, %l3                                              \n"
   "   and   %l3, 0x1f, %l3                                         \n"
   "   andn  %l0, 0x3f, %l0                                         \n"
   "   or    %l0, %l3, %l0                                          \n"
   "   wr    %l0, %psr                                              \n"
   "   nop                                                          \n"
   "   nop                                                          \n"
   "   nop                                                          \n"
   "   ld    [%l5 + " CYROS_SPARC_OFF(CYROS_SPARC_F_G1) "], %g1     \n"
   "   jmp   %l1                                                    \n"
   "   rett  %l2                                                    \n"
   ".size cyros_sparc_trap_entry, . - cyros_sparc_trap_entry        \n"
   ".popsection                                                     \n");


/**
 * @brief The C half of every trap but the window traps. Returns the frame to
 *        resume.
 *
 * An interrupt at the reschedule level is the reschedule, any other level
 * the target's, each run at its own level so that higher ones nest. `ta 9`
 * is a yield, run at the reschedule's, and `ta 10` a switch the kernel has
 * decided (cyros_port_yield_to), which swaps the frames at 15. Anything else
 * is a fault, run at 15.
 * Entered and left at PIL 15 ("Nested interrupts").
 *
 * Used: only the assembly above calls it, which the compiler cannot see, so an
 * LTO build that drops unreferenced internal functions would drop it.
 */
extern "C" [[gnu::used]] CYROS_PORT_HOT std::uint32_t* cyros_sparc_trap_dispatch(std::uint32_t* frame, std::uint32_t tt)
{
   cyros_sparc_core_state& state = cyros_sparc_cores[this_core()];
   /* The entry counted this trap, and recorded the outermost one's frame as
    * the thread's. A nested trap has its own frame, which it resumes, and
    * must not replace the thread's, or the outer trap would resume it. */
   bool const outermost = state.depth == 1u;

   if (tt >= sparcv8::tt_interrupt_first && tt <= sparcv8::tt_interrupt_last) {
      std::uint32_t const level = tt - 0x10u;
      set_pil(level);
      if (level == sparcv8::reschedule_level) {
         /* The interrupt controller dropped the request when the trap was
          * taken, so a reschedule asked for while this one runs (a wake from
          * inside the scheduler, or from a handler nested in it) is kept for
          * the next. */
         run_reschedule(state);
      } else {
         sparcv8::interrupt(level);
      }
      set_pil(sparcv8::pil_masked_preempt);
   }
   else if ((tt == sparcv8::tt_yield || tt == sparcv8::tt_yield_to) && outermost) {
      /* Resume after the ta. Done on THIS thread's frame, before a switch
       * replaces it. */
      frame[sparcv8::frame_pc]  = frame[sparcv8::frame_npc];
      frame[sparcv8::frame_npc] = frame[sparcv8::frame_npc] + 4u;
      if (tt == sparcv8::tt_yield) {
         set_pil(sparcv8::reschedule_level);
         run_reschedule(state);
         set_pil(sparcv8::pil_masked_preempt);
      } else {
         /* The two contexts cyros_port_yield_to passed, in its %o0 and %o1. */
         cyros_port_switch(reinterpret_cast<cyros_port_context*>(frame[sparcv8::frame_o0]),
                           reinterpret_cast<cyros_port_context*>(frame[sparcv8::frame_o0 + 1u]));
      }
   }
   else {
      cyros_sparc_fault_handler(tt, frame);
   }

   return outermost ? state.frame : frame;
}

/**
 * @brief The default fault handler: report, and panic.
 */
extern "C" [[gnu::weak]] CYROS_PORT_LINKED_BY_NAME void cyros_sparc_fault_handler(std::uint32_t tt, std::uint32_t* frame)
{
   sparcv8::console_write("\n*** SPARC TRAP ***\n  tt   = ");
   write_hex(tt);
   sparcv8::console_write("\n  pc   = ");
   write_hex(frame[sparcv8::frame_pc]);
   sparcv8::console_write("\n  npc  = ");
   write_hex(frame[sparcv8::frame_npc]);
   sparcv8::console_write("\n  psr  = ");
   write_hex(frame[sparcv8::frame_psr]);
   sparcv8::console_write("\n");
   cyros_port_system_error(tt, frame[sparcv8::frame_pc], "unexpected trap", 0);
}

/**
 * @brief The default for an interrupt level nobody handles: panic, naming it.
 */
extern "C" [[gnu::weak]] void cyros_sparc_device_interrupt(std::uint32_t level)
{
   cyros_port_system_error(level, 0, "device interrupt with no handler", 0);
}


/* ============================================================================
 * Platform Initialisation
 * ========================================================================= */

namespace cyros::port::sparcv8
{

void init_this_core()
{
   /* Masked for the whole of bring-up, as on every port: nothing may enter
    * the kernel until cyros_port_start_first has a thread to run. */
   psr_set(psr_pil_mask);

   /* The board points TBR at the table before any compiled code, because
    * any call may need a window trap. Checked rather than set: a table set
    * here would be too late to have been right. */
   CYROS_ASSERT_OP(read_tbr() & ~0xFFFu, ==, reinterpret_cast<std::uintptr_t>(cyros_sparc_trap_table));
   CYROS_ASSERT_OP((read_asr17() & 0x1Fu) + 1u, ==, nwindows);
#if CYROS_SPARC_HAS_FPU
   CYROS_ASSERT((read_psr() & psr_ef) != 0u);
#endif

   std::uint32_t const core = core_number();
   CYROS_ASSERT_OP(core, <, static_cast<std::uint32_t>(CYROS_PORT_CORE_COUNT));
   cyros_sparc_core_state& state = cyros_sparc_cores[core];
   state.frame     = nullptr;
   state.depth     = 0u;
   state.irq_stack = 0u;   /* no interrupt stack until the first thread */

   reschedule_setup(core);
}

} // namespace cyros::port::sparcv8

void cyros_port_init(cyros_port_reschedule_t handler)
{
   CYROS_ASSERT(handler != nullptr);
   reschedule_handler = handler;
   sparcv8::init_this_core();
}


/* ============================================================================
 * Interrupt Control
 * ========================================================================= */

bool cyros_port_interrupts_enabled(void)
{
   return (sparcv8::read_psr() & pil_irq_bits) == 0u;
}

CYROS_PORT_HOT cyros_mask_token_t cyros_port_irq_save(void)
{
   return psr_set(pil_irq_bits) & pil_irq_bits;
}

CYROS_PORT_HOT void cyros_port_irq_restore(cyros_mask_token_t token)
{
   /* Restores to MASKED when the token says the caller was already masked,
    * and leaves the preemption field as it is now. */
   psr_restore(pil_irq_bits, token);
}


/* ============================================================================
 * Preemption Control
 * ========================================================================= */

CYROS_PORT_HOT cyros_mask_token_t cyros_port_preempt_disable(void)
{
   return psr_set(pil_preempt_bits) & pil_preempt_bits;
}

CYROS_PORT_HOT void cyros_port_preempt_enable(cyros_mask_token_t token)
{
   /* Clearing the bit with the other field clear takes a pended reschedule
    * within the write's three nops, so before this returns. */
   psr_restore(pil_preempt_bits, token);
}


/* ============================================================================
 * Context Management & Switching
 * ========================================================================= */

void cyros_port_context_init(cyros_port_context* context,
                             void* stack_base,
                             std::size_t stack_size,
                             cyros_port_entry_t entry,
                             void* arg)
{
   CYROS_ASSERT(context != nullptr);
   CYROS_ASSERT(stack_base != nullptr);
   CYROS_ASSERT(entry != nullptr);

   /* The entry function's sp, with its window save area, and the frame below
    * it, which the trap exit consumes as if the thread had been interrupted
    * at the first instruction of `entry`. */
   std::size_t const needed = sparcv8::min_frame_bytes + 2u * sparcv8::frame_bytes;
   CYROS_ASSERT_OP(stack_size, >=, needed);

   auto const base = reinterpret_cast<std::uintptr_t>(stack_base);
   std::uintptr_t const top = (base + stack_size) & ~static_cast<std::uintptr_t>(7u);
   std::uintptr_t const sp  = top - sparcv8::min_frame_bytes;
   CYROS_ASSERT_OP(sp - sparcv8::frame_bytes, >, base);

   /* The save area at sp is the window the exit loads first. Zero, so its
    * frame pointer ends the chain for a debugger. */
   auto* const area = reinterpret_cast<std::uint32_t*>(sp);
   for (std::uint32_t i = 0; i < sparcv8::min_frame_bytes / 4u; ++i) { area[i] = 0u; }

   auto* const frame = reinterpret_cast<std::uint32_t*>(sp - sparcv8::frame_bytes);
   for (std::uint32_t i = 0; i < sparcv8::frame_words; ++i) { frame[i] = 0u; }

   /* Supervisor, traps on at the rett, PIL 0 so the thread starts at
    * baseline priority, and the FPU in a hard-float build. `entry` runs as if
    * called: its argument in o0, its sp in o6, and a return address in o7
    * that leads to the trap for a thread that returns. */
   auto const address = reinterpret_cast<std::uintptr_t>(entry);
   frame[sparcv8::frame_psr] = sparcv8::psr_s | sparcv8::psr_ps
                             | (CYROS_SPARC_HAS_FPU ? sparcv8::psr_ef : 0u);
   frame[sparcv8::frame_pc]  = address;
   frame[sparcv8::frame_npc] = address + 4u;
   frame[sparcv8::frame_o0]  = reinterpret_cast<std::uintptr_t>(arg);
   frame[sparcv8::frame_o6]  = sp;
   frame[sparcv8::frame_o6 + 1u] = reinterpret_cast<std::uintptr_t>(&thread_return_trap) - 8u;

   context->frame = frame;
}

void cyros_port_context_destroy(cyros_port_context* context)
{
   CYROS_ASSERT(context != nullptr);
   /* Nothing is owned. Poisoned, so a resume faults on a null frame. */
   context->frame = nullptr;
}

CYROS_PORT_HOT void cyros_port_switch(cyros_port_context* from, cyros_port_context* to)
{
   cyros_sparc_core_state& state = cyros_sparc_cores[this_core()];

   /* Reachable only from the trap path, where `frame` is the running
    * thread's complete frame. Anywhere else this would save garbage. The
    * windows are the exit's to settle. */
   CYROS_ASSERT_OP(state.depth, ==, 1u);
   CYROS_ASSERT(to != nullptr);
   CYROS_ASSERT(to->frame != nullptr);

   if (from != nullptr) {
      from->frame = state.frame;
   }
   state.frame = to->frame;
}

CYROS_PORT_HOT void cyros_port_yield_to(cyros_port_context* from, cyros_port_context* to, cyros_mask_token_t token)
{
   /* Thread context, masked by the kernel from baseline (port_core.h). The
    * trap is taken masked, and its handler switches (the file comment). */
   register cyros_port_context* o0 asm("o0") = from;
   register cyros_port_context* o1 asm("o1") = to;
   asm volatile("ta " CYROS_SPARC_STR(CYROS_SPARC_TA_YIELD_TO) : "+r"(o0), "+r"(o1) : : "memory");

   /* Back by either road, still masked: the frame kept this thread's PIL. */
   cyros_port_irq_restore(token);
}

CYROS_PORT_HOT bool cyros_port_masked_from_baseline(cyros_mask_token_t token)
{
   /* The token is the interrupt field before the mask. Outside a trap, with
    * it and the preemption field both clear, the caller was at PIL 0. */
   return token == 0u && cyros_sparc_cores[this_core()].depth == 0u &&
          (sparcv8::read_psr() & pil_preempt_bits) == 0u;
}

/* Not marked [[noreturn]], as on the other ports: port_core.h declares it
 * plain. */
void cyros_port_start_first(cyros_port_context* first)
{
   CYROS_ASSERT(first != nullptr);
   CYROS_ASSERT(first->frame != nullptr);

   cyros_sparc_core_state& state = cyros_sparc_cores[this_core()];
   state.depth = 0u;
   state.frame = first->frame;

   /* The stack this core is standing on becomes its interrupt stack. Nothing
    * above it is ever returned to, and everything a handler uses lies below. */
   std::uintptr_t sp;
   asm volatile("mov %%sp, %0" : "=r"(sp));
   state.irq_stack = sp & ~static_cast<std::uintptr_t>(7u);

   /* This window plays the trap window: traps off, then the exit loads the
    * thread's window above it and enters it. */
   asm volatile(
      "mov   %0, %%l5                   \n"
      "rd    %%psr, %%g1                \n"
      "andn  %%g1, 0x20, %%g1           \n"
      "wr    %%g1, %%psr                \n"
      "nop                              \n"
      "nop                              \n"
      "nop                              \n"
      "and   %%g1, 0x1f, %%g3           \n"
      "ba    cyros_sparc_trap_fill      \n"
      " nop                             \n"
      :
      : "r"(first->frame)
      : "l5", "g1", "g3", "memory", "cc");

   __builtin_unreachable();
}


/* ============================================================================
 * Reschedule Requests
 * ========================================================================= */

CYROS_PORT_HOT void cyros_port_thread_yield(void)
{
   /* port.h asks ports that can observe the execution priority to assert the
    * baseline precondition. This one can. */
   CYROS_ASSERT_OP(cyros_sparc_cores[this_core()].depth, ==, 0u);
   CYROS_ASSERT_OP(sparcv8::pil(), ==, sparcv8::pil_open);

   /* Synchronous: the trap path runs the reschedule before the next
    * instruction, and resumes after the ta when this thread is next
    * switched in. */
   asm volatile("ta " CYROS_SPARC_STR(CYROS_SPARC_TA_YIELD) ::: "memory");
}

CYROS_PORT_HOT void cyros_port_pend_reschedule(void)
{
   /* Callable from anywhere. Raising level 1 is the whole implementation: it
    * is taken when PIL is 0, which is port.h's next safe point. */
   sparcv8::reschedule_raise(this_core());
}


/* ============================================================================
 * Thread-Local Storage
 * ========================================================================= */

void cyros_port_set_tls_pointer(void* tls_base)
{
   asm volatile("mov %0, %%g7" : : "r"(tls_base) : "memory");
}

void* cyros_port_get_tls_pointer(void)
{
   void* tls;
   asm volatile("mov %%g7, %0" : "=r"(tls));
   return tls;
}


/* ============================================================================
 * CPU Hints & Idle
 * ========================================================================= */

void cyros_port_cpu_relax(void)
{
   asm volatile("nop" ::: "memory");
}

void cyros_port_idle(void)
{
   /* LEON3's power-down, until an interrupt. The kernel's idle loop calls it
    * at baseline priority, so whatever wakes it is then taken. */
   asm volatile("wr %%g0, %%asr19" ::: "memory");
}


/* ============================================================================
 * Measurement
 * ========================================================================= */

std::uint64_t cyros_port_timestamp(void)
{
   /* The target's counter (sparcv8.hpp). LEON3 has no cycle counter of its
    * own. */
   return sparcv8::timestamp();
}


/* ============================================================================
 * Debug & Diagnostics
 * ========================================================================= */

void cyros_port_system_error(std::uintptr_t auxilary1,
                             std::uintptr_t auxilary2,
                             char const* file_optional,
                             int line_optional)
{
   psr_set(sparcv8::psr_pil_mask);

   sparcv8::console_write("\n*** CYROS PANIC ***\n  aux1 = ");
   write_hex(static_cast<std::uint32_t>(auxilary1));
   sparcv8::console_write("\n  aux2 = ");
   write_hex(static_cast<std::uint32_t>(auxilary2));

   if (file_optional != nullptr && file_optional[0] != '\0') {
      sparcv8::console_write("\n  at   = ");
      sparcv8::console_write(file_optional);
      sparcv8::console_write(":");
      write_hex(static_cast<std::uint32_t>(line_optional));
   }
   sparcv8::console_write("\n");

   /* A trap with traps disabled is the error state, which halts the
    * processor. `ta 0` is QEMU's shutdown. */
   asm volatile("rd   %%psr, %%g1     \n"
                "andn %%g1, 0x20, %%g1 \n"
                "wr   %%g1, %%psr     \n"
                "nop                  \n"
                "nop                  \n"
                "nop                  \n"
                "ta   " CYROS_SPARC_STR(CYROS_SPARC_TA_HALT) " \n"
                ::: "g1", "memory", "cc");
   for (;;) {}
}

void cyros_port_wait_for_debugger(void)
{
   static volatile int resume = 0;
   sparcv8::console_write("cyros: waiting for debugger, set 'resume' to 1\n");
   while (resume == 0) { asm volatile("nop"); }
}

void cyros_port_breakpoint(void)
{
   asm volatile("ta " CYROS_SPARC_STR(CYROS_SPARC_TA_BREAKPOINT) ::: "memory");
}

void* cyros_port_get_stack_pointer(void)
{
   void* sp = nullptr;
   asm volatile("mov %%sp, %0" : "=r"(sp));
   return sp;
}
