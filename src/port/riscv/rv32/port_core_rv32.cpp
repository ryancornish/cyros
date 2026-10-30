/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file port_core_rv32.cpp
 * @brief 32-bit RISC-V machine-mode CORE LAYER: all of port_core.h.
 *
 * Bench is QEMU's `virt` (rv32). Hardware is the RP2350's Hazard3. Both are
 * RV32IMAC in machine mode, with no FPU, which is everything this file assumes.
 * The shape follows the ARM core layer, and where the two differ the reason is
 * the hardware, not taste. `~/cyros-claude/riscv-port-plan.md` has the table.
 *
 *
 * The central decision: ALL SWITCHING HAPPENS IN ONE TRAP PATH
 * ============================================================
 * As on ARM, where it is PendSV. A reschedule is the machine SOFTWARE interrupt
 * (msip), and the trap that services it is the only place a thread is switched
 * out asynchronously. It is taken only when mstatus.MIE and mie.MSIE are both
 * set, and every handler runs with MIE clear, so a reschedule pended inside an
 * interrupt is taken when that interrupt returns, never during it. That is the
 * same property PendSV's lowest priority gives ARM.
 *
 * `cyros_port_thread_yield` enters the same dispatch synchronously, as a CALL
 * that masks and saves only what a call must keep (below). A soft IRQ write is
 * a store to a peripheral and lands a few cycles after the instruction, so a
 * yield built on it could return before the switch, which would break its
 * strong guarantee. A call cannot. A reschedule pended at baseline is the same
 * call, since the soft IRQ would be taken there anyway, a few cycles later and
 * with a whole frame.
 *
 * A reschedule an interrupt handler pends is run before that trap returns,
 * rather than by a second trap the moment its mret unmasks, which would
 * restore the frame only to save it again: what ARM's tail-chaining into
 * PendSV does in hardware.
 *
 *
 * What a trap saves, and where
 * ============================
 * RISC-V saves nothing in hardware but the pc, so the entry below stores the
 * frame itself, on the INTERRUPTED stack (riscv.hpp has the layout), then
 * moves to the core's interrupt stack before calling C, so a thread stack
 * needs room for one frame and never for the deepest handler chain. The
 * interrupt stack is the stack the core was standing on when it started its
 * first thread, which is the role MSP plays on ARM, and `mscratch` holds its
 * top.
 *
 * The frame is 32 words in two halves. THE CLOBBERED HALF is what a call may
 * change, ra, t0 to t6 and a0 to a7, with mepc and mstatus: 18 words. THE
 * KEPT HALF is what compiled code leaves as it found it: s0 to s11 by the
 * psABI, and gp and tp, which nothing compiled writes. The entry stores the
 * clobbered half alone, and the C it calls keeps the other in the registers,
 * so an interrupt that asks for no reschedule restores the clobbered half and
 * returns. The thread's kept half never left the registers.
 *
 * An interrupt that asks for a reschedule stores the kept half as well, from
 * the registers that still hold it, and then runs the reschedule a yield runs
 * (below). A fault stores both halves at entry, so its handler is given every
 * register and may change any. So the kernel only ever switches away from a
 * WHOLE frame: a suspended thread is just a pointer to its frame, and
 * cyros_port_switch is one store, as on ARM. Whole BEFORE the reschedule,
 * never after it, because a reschedule that retires a thread wakes its
 * joiners, and from then on its stack may be theirs.
 *
 * `mscratch` reads ZERO while a trap is being handled. A trap taken inside a
 * handler, which can only be a fault, then stays on the stack it is on rather
 * than restarting the interrupt stack from the top over its own caller.
 *
 * A YIELD stores a CALL FRAME instead: the same 32 words, of which only the
 * return address (in mepc's slot, its bit 0 set to mark the kind), tp and
 * s0 to s11 are written. Everything else is dead across a call by the psABI,
 * so the caller already keeps nothing there. The exit tells the two apart by
 * that bit, which a real mepc never has, and resumes a call frame by
 * restoring tp and s0 to s11 and returning through mret: 14 stores and 14
 * loads where a trap frame costs 32 of each and an exception's entry.
 * A thread's frame is whichever kind it was suspended with, so any exit
 * resumes either.
 *
 *
 * The thread's road: a call that returns into another thread
 * ===========================================================
 * Where the kernel has decided a switch in thread context (port_core.h, the
 * thread's road), cyros_port_direct_switch makes it with no trap path at all. It
 * is a call, made inside the kernel's mask, that saves the call frame a yield
 * saves, marked as a switch's, moves the guard, and resumes the incoming
 * thread on that thread's own stack. A thread cyros_port_direct_switch suspended
 * is resumed by returning from its own call, which unmasks as it returns. Any
 * other frame goes to the trap exit, which resumes anything. The trap exit in
 * turn resumes a switch's frame as it resumes any call frame, unmasked, which
 * is how the switch's own return leaves it. So neither road needs to know how
 * the thread it resumes was suspended.
 *
 *
 * Masking: two bits, not two counters
 * ===================================
 *
 *   Interrupt masking  (cyros_port_irq_save/restore)       -> mstatus.MIE
 *   Preemption control (cyros_port_preempt_disable/enable) -> mie.MSIE
 *
 * MSIE gates the reschedule and nothing else, so the timer and device
 * interrupts still run while no switch can happen. That is port_core.h's
 * distinction, delivered by one CSR bit. The TOKEN carries the previous bit,
 * so regions compose without a depth counter, as PRIMASK and BASEPRI do.
 *
 * Baseline priority is MIE and MSIE both set, outside a trap. Setting either
 * bit back with the other set takes a pended reschedule at once.
 *
 *
 * TLS is the thread pointer
 * =========================
 * ARMv8-M has no TLS register, so the ARM layer keeps the running thread's TLS
 * base in a per-core array and saves it on every switch. RISC-V has `tp`, and
 * the trap frame saves it with every other register, so it follows its thread
 * with no code at all.
 *
 *
 * The stack guard: PMP entry 0, moved on every switch
 * ===================================================
 * cyros hands every thread a caller-owned buffer, so an overrun walks into
 * whatever the application put below it. The policy is a hardware guard where
 * one is cheap and none otherwise. Here it is PMP entry 0, the highest
 * priority, so no entry an application programs can override it: a 128-byte
 * NAPOT region with no permissions near the bottom of the RUNNING thread's
 * buffer, moved by one pmpaddr0 write per switch (3 cycles on Hazard3,
 * rp2350-notes.md 6c). A store or load into it faults.
 *
 * A standard PMP entry binds machine mode only when locked, and a locked one
 * cannot move, so the target has to supply a way (riscv.hpp,
 * stack_guard_setup): Hazard3's Xh3pmpm, or Smepmp's rule-lock bypass on
 * QEMU's virt. A target with neither runs unguarded, by policy.
 *
 * THE SPILL ZONE. The trap entry saves its frame on the INTERRUPTED stack, so
 * a thread that faults on the guard faults again inside the entry, which
 * steps 128 bytes lower each time until its stores clear the guard. Its last
 * frame lies at most 252 bytes below the guard. So the guard sits 256 bytes
 * above the bottom of the buffer, and those writes land in the thread's own
 * buffer rather than below it. The fault handler then runs on the interrupt
 * stack and reports. Guard, spill zone and the guard's 128-byte alignment cost
 * up to 508 bytes of each buffer. The clobbered half holds the frame's lowest
 * word and its highest, mepc and x31, so a frame that reaches into the guard
 * faults while the entry stores that half, and the kept half, stored later,
 * never can.
 *
 * What it cannot catch, as on ARMv7-M: a frame larger than the guard whose
 * stores all miss it, the sp stepping over it in one adjustment.
 *
 * CYROS_RISCV_STACK_GUARD set to 0 builds it out: PMP entry 0 is left to the
 * application, no buffer loses its spill zone, and a switch writes no pmpaddr.
 * It is local to this file, a switch for measuring or ruling the guard out.
 */

#include <cyros/port/port_core.h>

/* For cyros_port_get_core_id, as in the ARM layer: a core layer may call the
 * MCU contract, it may not implement it. On a single-core target the call
 * folds away (this_core below). */
#include <cyros/port/port_mcu.h>

#include "riscv.hpp"

#include <cstddef>
#include <cstdint>

#ifndef CYROS_RISCV_STACK_GUARD
#define CYROS_RISCV_STACK_GUARD 1
#endif

namespace riscv = cyros::port::riscv;

/* ============================================================================
 * Context
 * ========================================================================= */

/**
 * @brief A suspended thread's port state: where its trap frame is.
 *
 * Everything else, the registers, mepc, mstatus and tp, is IN that frame on the
 * thread's own stack.
 */
struct cyros_port_context
{
   std::uint32_t* frame;
   std::uint32_t  guard;   /* pmpaddr0 for this thread's guard, 0 for none */
};

static_assert(sizeof(cyros_port_context) <= CYROS_PORT_CONTEXT_SIZE,
              "CYROS_PORT_CONTEXT_SIZE in port_traits.h is too small for this port");
static_assert(alignof(cyros_port_context) <= CYROS_PORT_CONTEXT_ALIGN,
              "CYROS_PORT_CONTEXT_ALIGN in port_traits.h is too weak for this port");

namespace
{

cyros_port_reschedule_t reschedule_handler = nullptr;

/* The stack guard's geometry (see the file comment). */
constexpr std::uintptr_t guard_bytes = 128u;
constexpr std::uintptr_t spill_bytes = 256u;

constexpr bool guard_built = CYROS_RISCV_STACK_GUARD != 0;

/* PMP entry 0's configuration byte, from the target, or 0 for no guard.
 * Written by the bootstrap core in cyros_port_init, read everywhere after. */
std::uint8_t guard_cfg = 0u;

/* Whether threads are guarded: built in, and the target able. Built out, every
 * use folds away. */
inline bool guarded() noexcept
{
   return guard_built && guard_cfg != 0u;
}

/* pmpaddr for a NAPOT region of guard_bytes at `bottom`, which is aligned to
 * guard_bytes: the base over four, with size/8 - 1 in the low bits. */
constexpr std::uint32_t guard_pmpaddr(std::uintptr_t bottom) noexcept
{
   return static_cast<std::uint32_t>(bottom >> 2) | static_cast<std::uint32_t>(guard_bytes / 8u - 1u);
}

/* Entry 0's byte in pmpcfg0 and nothing else: entries 1 to 3 are the
 * application's. */
inline void set_guard_enabled(std::uint8_t cfg) noexcept
{
   riscv::clear_pmpcfg0(0xffu);
   if (cfg != 0u) {
      riscv::set_pmpcfg0(cfg);
   }
}

/**
 * @brief What each core's trap path keeps.
 *
 * `frame` is the RUNNING thread's frame while a trap is in progress, which
 * cyros_port_switch replaces with the incoming thread's, and which the trap
 * exit then restores. No two cores touch each other's entry.
 */
struct core_state
{
   std::uint32_t* frame;
   std::uintptr_t interrupt_stack_top;
   std::uint32_t  trap_depth;
};

core_state cores[CYROS_PORT_CORE_COUNT] = {};

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

[[noreturn]] void unexpected_trap(std::uint32_t cause, std::uint32_t epc)
{
   riscv::write0("\n*** RISC-V TRAP ***\n  mcause = ");
   riscv::write_hex(cause);
   riscv::write0("\n  mepc   = ");
   riscv::write_hex(epc);
   riscv::write0("\n  mtval  = ");
   riscv::write_hex(riscv::read_mtval());
   riscv::write0("\n");
   cyros_port_system_error(cause, epc, "unexpected trap", 0);
}

/* The reschedule, from inside the trap. The kernel's handler may call
 * cyros_port_switch, which replaces `frame`. */
CYROS_PORT_HOT void run_reschedule(core_state& state)
{
   CYROS_ASSERT_OP(state.trap_depth, ==, 1u);   /* never from inside a handler */
   CYROS_ASSERT(reschedule_handler != nullptr);
   reschedule_handler();
}

/* Whether an interrupt handler pended a reschedule, which then runs inside its
 * trap rather than by the second trap its mret would take (file comment). Only
 * where that trap would be taken: the soft IRQ raised and enabled, mie.MSIE
 * being the interrupted thread's preempt grade, and this the outermost trap,
 * which an interrupt always is. A raise not yet visible in mip is left to that
 * trap, as before. One that is taken is lowered here. */
CYROS_PORT_HOT inline bool take_pended_reschedule() noexcept
{
   if ((riscv::read_mip() & riscv::mip_msip) != 0u && (riscv::read_mie() & riscv::mie_msie) != 0u) {
      riscv::soft_irq_clear(this_core());
      return true;
   }
   return false;
}

/* What an interrupt's dispatch hands its trap's exit: the frame, and whether a
 * reschedule must run before it is resumed. Two words, so the psABI returns
 * them in a0 and a1, where the exit looks. `reschedule` is a whole word, 0 or
 * 1, because a bool's padding would reach a1 unspecified. */
struct interrupt_resume
{
   std::uint32_t* frame;
   std::uint32_t  reschedule;
};

} // namespace


/* ============================================================================
 * The trap path
 * ========================================================================= */

/**
 * @brief Every trap on every core. mtvec points here, direct mode.
 *
 * Naked because the prologue and epilogue ARE the mechanism. Stores the
 * frame's clobbered half on the interrupted stack, moves to the interrupt
 * stack (or stays put when a trap is already in progress, see the file
 * comment), and takes one of three ways out (file comment):
 *
 *  - an interrupt that asks for no reschedule restores the clobbered half and
 *    returns
 *  - one that asks for a reschedule stores the kept half, runs the reschedule
 *    a yield runs, and resumes whatever frame it hands back, which is a
 *    DIFFERENT thread's after a switch
 *  - a fault stores the kept half, runs its handler, and resumes the frame
 *
 * `cyros_port_trap_saved` is the label after the entry's own stores, for the
 * fault handler's overflow check. `cyros_port_trap_exit` resumes any whole
 * frame, which is also how cyros_port_start_first enters a thread for the
 * first time and how a yield leaves. It resumes either kind of frame (file
 * comment).
 */
extern "C" [[gnu::naked, gnu::aligned(4)]] CYROS_PORT_HOT void cyros_port_trap_entry(void)
{
   asm volatile(
      "addi  sp, sp, -128           \n"
      "sw    x1,    4(sp)           \n"   /* the clobbered half */
      "sw    x5,   20(sp)           \n"
      "sw    x6,   24(sp)           \n"
      "sw    x7,   28(sp)           \n"
      "sw    x10,  40(sp)           \n"
      "sw    x11,  44(sp)           \n"
      "sw    x12,  48(sp)           \n"
      "sw    x13,  52(sp)           \n"
      "sw    x14,  56(sp)           \n"
      "sw    x15,  60(sp)           \n"
      "sw    x16,  64(sp)           \n"
      "sw    x17,  68(sp)           \n"
      "sw    x28, 112(sp)           \n"
      "sw    x29, 116(sp)           \n"
      "sw    x30, 120(sp)           \n"
      "sw    x31, 124(sp)           \n"
      "csrr  t0, mepc               \n"
      "sw    t0,    0(sp)           \n"   /* x0's slot   */
      "csrr  t0, mstatus            \n"
      "sw    t0,    8(sp)           \n"   /* x2's slot   */
      ".global cyros_port_trap_saved \n"
      "cyros_port_trap_saved:       \n"
      "mv    a0, sp                 \n"   /* the frame   */
      "csrr  a1, mcause             \n"
      "csrrw t0, mscratch, zero     \n"   /* interrupt stack top, or 0 if nested */
      "bnez  t0, 1f                 \n"
      "mv    t0, sp                 \n"   /* nested: stay below the frame */
      "1:                           \n"
      "mv    sp, t0                 \n"
      "bgez  a1, 3f                 \n"   /* a fault */
      "call  cyros_port_interrupt_dispatch \n" /* the frame in a0, a reschedule asked for in a1 */
      "bnez  a1, 2f                 \n"
      "mv    sp, a0                 \n"   /* none: the clobbered half back */
      "lw    t0,    0(sp)           \n"
      "csrw  mepc, t0               \n"
      "lw    t0,    8(sp)           \n"
      "csrw  mstatus, t0            \n"
      "lw    x1,    4(sp)           \n"
      "lw    x5,   20(sp)           \n"
      "lw    x6,   24(sp)           \n"
      "lw    x7,   28(sp)           \n"
      "lw    x10,  40(sp)           \n"
      "lw    x11,  44(sp)           \n"
      "lw    x12,  48(sp)           \n"
      "lw    x13,  52(sp)           \n"
      "lw    x14,  56(sp)           \n"
      "lw    x15,  60(sp)           \n"
      "lw    x16,  64(sp)           \n"
      "lw    x17,  68(sp)           \n"
      "lw    x28, 112(sp)           \n"
      "lw    x29, 116(sp)           \n"
      "lw    x30, 120(sp)           \n"
      "lw    x31, 124(sp)           \n"
      "addi  sp, sp, 128            \n"
      "mret                         \n"
      "2:                           \n"   /* the kept half, still in the registers */
      "sw    x3,   12(a0)           \n"
      "sw    x4,   16(a0)           \n"
      "sw    x8,   32(a0)           \n"
      "sw    x9,   36(a0)           \n"
      "sw    x18,  72(a0)           \n"
      "sw    x19,  76(a0)           \n"
      "sw    x20,  80(a0)           \n"
      "sw    x21,  84(a0)           \n"
      "sw    x22,  88(a0)           \n"
      "sw    x23,  92(a0)           \n"
      "sw    x24,  96(a0)           \n"
      "sw    x25, 100(a0)           \n"
      "sw    x26, 104(a0)           \n"
      "sw    x27, 108(a0)           \n"
      "call  cyros_port_reschedule_dispatch \n" /* returns the frame to resume in a0 */
      "j     cyros_port_trap_exit   \n"
      "3:                           \n"   /* a fault: the kept half too, for its handler */
      "sw    x3,   12(a0)           \n"
      "sw    x4,   16(a0)           \n"
      "sw    x8,   32(a0)           \n"
      "sw    x9,   36(a0)           \n"
      "sw    x18,  72(a0)           \n"
      "sw    x19,  76(a0)           \n"
      "sw    x20,  80(a0)           \n"
      "sw    x21,  84(a0)           \n"
      "sw    x22,  88(a0)           \n"
      "sw    x23,  92(a0)           \n"
      "sw    x24,  96(a0)           \n"
      "sw    x25, 100(a0)           \n"
      "sw    x26, 104(a0)           \n"
      "sw    x27, 108(a0)           \n"
      "call  cyros_port_fault_dispatch \n" /* returns the frame to resume in a0 */
      ".global cyros_port_trap_exit \n"
      "cyros_port_trap_exit:        \n"
      "mv    sp, a0                 \n"
      "lw    t0,    0(sp)           \n"
      "andi  t1, t0, 1              \n"   /* a yield's call frame */
      "bnez  t1, 2f                 \n"
      "csrw  mepc, t0               \n"
      "lw    t0,    8(sp)           \n"
      "csrw  mstatus, t0            \n"
      "lw    x1,    4(sp)           \n"
      "lw    x3,   12(sp)           \n"
      "lw    x4,   16(sp)           \n"
      "lw    x5,   20(sp)           \n"
      "lw    x6,   24(sp)           \n"
      "lw    x7,   28(sp)           \n"
      "lw    x8,   32(sp)           \n"
      "lw    x9,   36(sp)           \n"
      "lw    x10,  40(sp)           \n"
      "lw    x11,  44(sp)           \n"
      "lw    x12,  48(sp)           \n"
      "lw    x13,  52(sp)           \n"
      "lw    x14,  56(sp)           \n"
      "lw    x15,  60(sp)           \n"
      "lw    x16,  64(sp)           \n"
      "lw    x17,  68(sp)           \n"
      "lw    x18,  72(sp)           \n"
      "lw    x19,  76(sp)           \n"
      "lw    x20,  80(sp)           \n"
      "lw    x21,  84(sp)           \n"
      "lw    x22,  88(sp)           \n"
      "lw    x23,  92(sp)           \n"
      "lw    x24,  96(sp)           \n"
      "lw    x25, 100(sp)           \n"
      "lw    x26, 104(sp)           \n"
      "lw    x27, 108(sp)           \n"
      "lw    x28, 112(sp)           \n"
      "lw    x29, 116(sp)           \n"
      "lw    x30, 120(sp)           \n"
      "lw    x31, 124(sp)           \n"
      "addi  sp, sp, 128            \n"
      "mret                         \n"
      "2:                           \n"   /* the registers a call keeps */
      "lw    x4,   16(sp)           \n"
      "lw    x8,   32(sp)           \n"
      "lw    x9,   36(sp)           \n"
      "lw    x18,  72(sp)           \n"
      "lw    x19,  76(sp)           \n"
      "lw    x20,  80(sp)           \n"
      "lw    x21,  84(sp)           \n"
      "lw    x22,  88(sp)           \n"
      "lw    x23,  92(sp)           \n"
      "lw    x24,  96(sp)           \n"
      "lw    x25, 100(sp)           \n"
      "lw    x26, 104(sp)           \n"
      "lw    x27, 108(sp)           \n"
      "addi  sp, sp, 128            \n"
      /* Through mret like any frame, never a jump: the yield was made at
       * baseline, so it returns there with MIE from MPIE, and mret is what
       * undoes whatever the trap being left did to the interrupt controller
       * (Hazard3 pushes MEICONTEXT's priority stack on an external interrupt
       * and only mret pops it). mepc's bit 0 is always zero, which drops the
       * mark. MPP is machine already, set again for a frame resumed from a
       * yield, which took no trap. */
      "li    t1, 0x1880             \n"   /* MPP machine, MPIE */
      "csrs  mstatus, t1            \n"
      "csrw  mepc, t0               \n"
      "mret                         \n");
}

/**
 * @brief The yield's half of the trap path: a CALL that does what a trap
 *        would, saving a call frame (file comment).
 *
 * Masks, stores the return address (marked) and the registers a call keeps on
 * the yielding thread's stack, moves to the interrupt stack as the trap entry
 * does, runs the reschedule, and leaves through cyros_port_trap_exit with
 * whichever frame it hands back: the trap's own way out once it has asked for
 * a reschedule. `cyros_port_yield_saved` is the label after its last store,
 * for the fault handler's overflow check.
 */
extern "C" [[gnu::naked, gnu::noinline, gnu::aligned(4)]] CYROS_PORT_HOT void cyros_port_yield_entry(void)
{
   asm volatile(
      "csrci mstatus, 8             \n"   /* masked, as a trap would leave it */
      "addi  sp, sp, -128           \n"
      "ori   t0, ra, 1              \n"   /* mepc's slot, marked: a call frame */
      "sw    t0,    0(sp)           \n"
      "sw    zero,  8(sp)           \n"   /* a yield's (riscv.hpp) */
      "sw    x4,   16(sp)           \n"
      "sw    x8,   32(sp)           \n"
      "sw    x9,   36(sp)           \n"
      "sw    x18,  72(sp)           \n"
      "sw    x19,  76(sp)           \n"
      "sw    x20,  80(sp)           \n"
      "sw    x21,  84(sp)           \n"
      "sw    x22,  88(sp)           \n"
      "sw    x23,  92(sp)           \n"
      "sw    x24,  96(sp)           \n"
      "sw    x25, 100(sp)           \n"
      "sw    x26, 104(sp)           \n"
      "sw    x27, 108(sp)           \n"
      ".global cyros_port_yield_saved \n"
      "cyros_port_yield_saved:      \n"
      "mv    a0, sp                 \n"   /* the frame   */
      "csrrw t0, mscratch, zero     \n"   /* interrupt stack top, or 0 before the first thread */
      "bnez  t0, 1f                 \n"
      "mv    t0, sp                 \n"
      "1:                           \n"
      "mv    sp, t0                 \n"
      "call  cyros_port_reschedule_dispatch \n"
      "j     cyros_port_trap_exit   \n");
}

/**
 * @brief The thread's road (file comment): a CALL that saves a call frame and
 *        returns into another, taking no trap.
 *
 * In thread context, masked by the kernel from baseline (port_core.h). Stores
 * the return address (marked) and the registers a call keeps on the outgoing
 * thread's stack, marked as a switch's frame, and points `from` at it. Then
 * moves the guard to the incoming thread and resumes it: by returning from
 * its own switch when a switch suspended it, and through cyros_port_trap_exit
 * otherwise. `cyros_port_direct_switch_saved` is the label after its last
 * store, for the fault handler's overflow check.
 *
 * The mask is what lets a frame be resumed by a return: nothing can trap
 * between the stores and the loads. The token is not read. The kernel masked
 * from baseline, so restoring it sets MIE, and a switch's return sets MIE once
 * it is back on its own stack, as the trap exit's mret sets it for any call
 * frame.
 */
extern "C" [[gnu::naked, gnu::noinline, gnu::aligned(4)]] CYROS_PORT_HOT void cyros_port_direct_switch(cyros_port_context*, cyros_port_context*, cyros_mask_token_t)
{
   asm volatile(
      "addi  sp, sp, -128           \n"
      "ori   t0, ra, 1              \n"   /* mepc's slot, marked: a call frame */
      "sw    t0,    0(sp)           \n"
      "li    t0, 1                  \n"   /* a switch's (riscv.hpp) */
      "sw    t0,    8(sp)           \n"
      "sw    x4,   16(sp)           \n"
      "sw    x8,   32(sp)           \n"
      "sw    x9,   36(sp)           \n"
      "sw    x18,  72(sp)           \n"
      "sw    x19,  76(sp)           \n"
      "sw    x20,  80(sp)           \n"
      "sw    x21,  84(sp)           \n"
      "sw    x22,  88(sp)           \n"
      "sw    x23,  92(sp)           \n"
      "sw    x24,  96(sp)           \n"
      "sw    x25, 100(sp)           \n"
      "sw    x26, 104(sp)           \n"
      "sw    x27, 108(sp)           \n"
      ".global cyros_port_direct_switch_saved \n"
      "cyros_port_direct_switch_saved: \n"
      "sw    sp,    0(a0)           \n"   /* from->frame */
#if CYROS_RISCV_STACK_GUARD
      /* The incoming thread's guard, after the last store to the outgoing
       * stack, which the old guard covered. 0 is no guard. */
      "lw    t0,    4(a1)           \n"   /* to->guard */
      "beqz  t0, 1f                 \n"
      "csrw  pmpaddr0, t0           \n"
      "1:                           \n"
#endif
      "lw    a0,    0(a1)           \n"   /* to->frame */
      "lw    t0,    0(a0)           \n"
      "andi  t1, t0, 1              \n"
      "beqz  t1, 2f                 \n"   /* a trap frame */
      "lw    t1,    8(a0)           \n"
      "beqz  t1, 2f                 \n"   /* a yield's, which resumes at baseline */
      "mv    sp, a0                 \n"   /* a switch's: return into it */
      "lw    x4,   16(sp)           \n"
      "lw    x8,   32(sp)           \n"
      "lw    x9,   36(sp)           \n"
      "lw    x18,  72(sp)           \n"
      "lw    x19,  76(sp)           \n"
      "lw    x20,  80(sp)           \n"
      "lw    x21,  84(sp)           \n"
      "lw    x22,  88(sp)           \n"
      "lw    x23,  92(sp)           \n"
      "lw    x24,  96(sp)           \n"
      "lw    x25, 100(sp)           \n"
      "lw    x26, 104(sp)           \n"
      "lw    x27, 108(sp)           \n"
      "andi  ra, t0, -2             \n"   /* drop the mark */
      "addi  sp, sp, 128            \n"
      "csrsi mstatus, 8             \n"   /* MIE: the token restored */
      "ret                          \n"
      "2:                           \n"
      "j     cyros_port_trap_exit   \n");
}

/**
 * @brief The C half of an interrupt's trap, with the frame's clobbered half
 *        saved. Returns the frame, and whether a reschedule must run before
 *        it is resumed.
 *
 * Dispatched by cause: the soft IRQ is the reschedule, and the timer and
 * external interrupts go to the target, after which a reschedule either pended
 * is asked for. The reschedule itself runs once the trap's exit has made the
 * frame whole (cyros_port_reschedule_dispatch), so nothing here may switch.
 *
 * Used: only the assembly above calls it, which the compiler cannot see, so an
 * LTO build that drops unreferenced internal functions would drop it. The same
 * holds for the two dispatches below.
 */
extern "C" [[gnu::used]] CYROS_PORT_HOT interrupt_resume cyros_port_interrupt_dispatch(std::uint32_t* frame, std::uint32_t cause)
{
   core_state& state = cores[this_core()];
   ++state.trap_depth;

   bool reschedule = false;
   switch (cause & ~riscv::mcause_interrupt) {
   case riscv::cause_soft_interrupt:
      /* Lowered BEFORE the reschedule, so a request made while it runs (a wake
       * from inside the scheduler) is kept for the next one rather than lost
       * to a late clear. */
      riscv::soft_irq_clear(this_core());
      reschedule = true;
      break;
   case riscv::cause_timer_interrupt:
      riscv::timer_interrupt();
      reschedule = take_pended_reschedule();
      break;
   case riscv::cause_ext_interrupt:
      riscv::external_interrupt();
      reschedule = take_pended_reschedule();
      break;
   default:
      unexpected_trap(cause, frame[riscv::frame_mepc]);
   }

   --state.trap_depth;
   if (!reschedule) {
      /* Re-arm the interrupt stack for the next trap. mret is still ahead and
       * MIE stays clear until it, so nothing can take a trap in between. A
       * reschedule re-arms it after itself, not before: until it has run, a
       * fault inside it must stay below it on the interrupt stack, as one
       * inside a handler does, rather than restart from the top over it. No
       * test reaches that (a fault in the kernel's reschedule), so the
       * condition survives its removal, and this is its argument. */
      riscv::write_mscratch(state.interrupt_stack_top);
   }
   return {frame, reschedule ? 1u : 0u};
}

/**
 * @brief The C half of a fault's trap, with the whole frame saved. Returns the
 *        frame to resume, its own.
 *
 * The handler is the application's if it supplied one, and returning from it
 * resumes the frame, with whatever it changed there. A fault inside a handler
 * is a trap inside a trap: it resumes its own frame, and only the outermost
 * re-arms the interrupt stack. The trap it interrupted holds its own frame.
 * Every exception is a fault, an `ecall` included: a yield is a call
 * (cyros_port_yield_entry).
 */
extern "C" [[gnu::used]] std::uint32_t* cyros_port_fault_dispatch(std::uint32_t* frame, std::uint32_t cause)
{
   core_state& state = cores[this_core()];
   ++state.trap_depth;
   cyros_riscv_fault_handler(cause, frame);
   --state.trap_depth;
   if (state.trap_depth == 0u) {
      riscv::write_mscratch(state.interrupt_stack_top);
   }
   return frame;
}

/**
 * @brief The reschedule, on the interrupt stack with a whole frame. Returns
 *        the frame to resume.
 *
 * Reached by a yield, with the call frame it saved, and by an interrupt that
 * asked for one, once the trap's exit has stored the kept half (file comment).
 * Either way no other trap is in progress: a yield is made at baseline
 * (cyros_port_thread_yield checks), and an interrupt is only taken there. So
 * this one is the outermost, and the interrupt stack is re-armed after it.
 */
extern "C" [[gnu::used]] CYROS_PORT_HOT std::uint32_t* cyros_port_reschedule_dispatch(std::uint32_t* frame)
{
   core_state& state = cores[this_core()];
   state.trap_depth = 1u;
   state.frame = frame;
   run_reschedule(state);
   state.trap_depth = 0u;
   riscv::write_mscratch(state.interrupt_stack_top);
   return state.frame;
}


/* The trap entry's, the yield's and cyros_port_direct_switch's own extents, to
 * recognise a fault taken while one saved a frame. Each label follows its
 * last store. The trap entry's is its clobbered half, which spans the frame
 * (file comment). */
extern "C" void cyros_port_trap_saved(void);
extern "C" void cyros_port_yield_saved(void);
extern "C" void cyros_port_direct_switch_saved(void);

/**
 * @brief The default fault handler: report, and panic.
 *
 * Weak, so an application can take faults itself (riscv.hpp). An access fault
 * inside the trap entry, or the yield's or cyros_port_direct_switch's while it saves,
 * means it could not save a frame below the interrupted sp, which with the
 * guard on is a stack overflow: the frame was headed into the guard, and in
 * the trap entry's case the original faulting instruction is lost.
 */
extern "C" [[gnu::weak]] CYROS_PORT_LINKED_BY_NAME void cyros_riscv_fault_handler(std::uint32_t mcause, std::uint32_t* frame)
{
   std::uint32_t const epc = frame[riscv::frame_mepc];
   auto const entry       = reinterpret_cast<std::uint32_t>(&cyros_port_trap_entry);
   auto const entry_saved = reinterpret_cast<std::uint32_t>(&cyros_port_trap_saved);
   auto const yield       = reinterpret_cast<std::uint32_t>(&cyros_port_yield_entry);
   auto const yield_saved = reinterpret_cast<std::uint32_t>(&cyros_port_yield_saved);
   auto const swap        = reinterpret_cast<std::uint32_t>(&cyros_port_direct_switch);
   auto const swap_saved  = reinterpret_cast<std::uint32_t>(&cyros_port_direct_switch_saved);
   bool const access = mcause == riscv::cause_load_access || mcause == riscv::cause_store_access;
   bool const saving = (epc >= entry && epc < entry_saved) || (epc >= yield && epc < yield_saved) ||
                       (epc >= swap && epc < swap_saved);
   if (guarded() && access && saving) {
      riscv::write0("\n*** stack overflow: the trap entry could not save a frame (the guard) ***");
   }
   unexpected_trap(mcause, epc);
}


/* ============================================================================
 * Platform Initialisation
 * ========================================================================= */

namespace cyros::port::riscv
{

void init_this_core()
{
   /* Masked for the whole of bring-up, as on ARM: nothing may trap into the
    * kernel until cyros_port_start_first has a thread to run. */
   clear_mstatus(mstatus_mie);

   write_mtvec(reinterpret_cast<std::uint32_t>(&cyros_port_trap_entry));

   /* No interrupt stack until cyros_port_start_first names one. Zero is the
    * trap entry's "stay on this stack", so a trap taken before the first
    * thread (a reschedule pended during bring-up, or the port test's) runs on
    * the stack it arrives on rather than at whatever mscratch held. */
   write_mscratch(0u);

   /* A reschedule left raised by a previous kernel lifecycle would otherwise
    * fire into the next lifecycle's first thread. */
   soft_irq_clear(this_core());

   /* The reschedule is enabled here and stays enabled except inside a
    * preempt-disable. The timer's enable belongs to the time source. */
   set_mie(mie_msie);

   /* mcycle counts, for cyros_port_timestamp where a target uses it and for
    * anyone measuring. Hazard3 resets with it inhibited (rp2350-notes.md 6a). */
   write_mcountinhibit(0u);

   /* PMP entry 0 binding machine mode, and every hart able to do so if the
    * bootstrap core was. Off until cyros_port_start_first has a thread to
    * guard: the bring-up runs on the boot stack. */
   if constexpr (guard_built) {
      std::uint8_t const cfg = stack_guard_setup();
      CYROS_ASSERT_OP(cfg, ==, guard_cfg);
      set_guard_enabled(0u);
   }
}

} // namespace cyros::port::riscv

void cyros_port_init(cyros_port_reschedule_t handler)
{
   CYROS_ASSERT(handler != nullptr);
   /* The bootstrap core decides whether there is a guard, and every core's
    * init_this_core, this one's included, checks it agrees. */
   if constexpr (guard_built) {
      guard_cfg = riscv::stack_guard_setup();
   }
   reschedule_handler = handler;
   riscv::init_this_core();
}


/* ============================================================================
 * Interrupt Control
 * ========================================================================= */

bool cyros_port_interrupts_enabled(void)
{
   return (riscv::read_mstatus() & riscv::mstatus_mie) != 0u;
}

CYROS_PORT_HOT cyros_mask_token_t cyros_port_irq_save(void)
{
   std::uint32_t previous;
   asm volatile("csrrci %0, mstatus, 8" : "=r"(previous) : : "memory");
   return previous & riscv::mstatus_mie;
}

CYROS_PORT_HOT void cyros_port_irq_restore(cyros_mask_token_t token)
{
   /* Restores to MASKED when the token says the caller was already masked. */
   if ((token & riscv::mstatus_mie) != 0u) {
      riscv::set_mstatus(riscv::mstatus_mie);
   } else {
      riscv::clear_mstatus(riscv::mstatus_mie);
   }
}


/* ============================================================================
 * Preemption Control
 * ========================================================================= */

CYROS_PORT_HOT cyros_mask_token_t cyros_port_preempt_disable(void)
{
   std::uint32_t previous;
   asm volatile("csrrci %0, mie, 8" : "=r"(previous) : : "memory");
   return previous & riscv::mie_msie;
}

CYROS_PORT_HOT void cyros_port_preempt_enable(cyros_mask_token_t token)
{
   /* Setting MSIE back with MIE set takes a pended reschedule at the next
    * instruction boundary. No software safe-point check, as on ARM. */
   if ((token & riscv::mie_msie) != 0u) {
      riscv::set_mie(riscv::mie_msie);
   } else {
      riscv::clear_mie(riscv::mie_msie);
   }
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
   /* One frame plus room to be useful. */
   CYROS_ASSERT_OP(stack_size, >=, 2u * riscv::frame_bytes);

   auto const base = reinterpret_cast<std::uintptr_t>(stack_base);
   std::uintptr_t const top = (base + stack_size) & ~static_cast<std::uintptr_t>(15u);
   CYROS_ASSERT_OP(top - riscv::frame_bytes, >, base);

   /* The guard, when there is one: the spill zone at the bottom, then the
    * guard aligned to its own size, then the stack proper (file comment). The
    * decision is cyros_port_init's, so it must have run. */
   CYROS_ASSERT(reschedule_handler != nullptr);
   std::uint32_t guard = 0u;
   if (guarded()) {
      std::uintptr_t const bottom = (base + spill_bytes + guard_bytes - 1u) & ~(guard_bytes - 1u);
      CYROS_ASSERT_OP(top - riscv::frame_bytes, >, bottom + guard_bytes);
      guard = guard_pmpaddr(bottom);
   }

   /* The frame cyros_port_trap_exit consumes. mret enters `entry` in machine
    * mode with MIE set from MPIE, so the thread starts at baseline priority,
    * and its sp is the frame's top, the 16-aligned top of its stack. */
   auto* frame = reinterpret_cast<std::uint32_t*>(top - riscv::frame_bytes);
   for (std::uint32_t i = 0; i < riscv::frame_words; ++i) { frame[i] = 0u; }

   std::uint32_t gp;
   asm volatile("mv %0, gp" : "=r"(gp));

   frame[riscv::frame_mepc]    = reinterpret_cast<std::uint32_t>(entry);
   frame[riscv::frame_mstatus] = riscv::mstatus_mpp_machine | riscv::mstatus_mpie;
   frame[riscv::frame_ra]      = reinterpret_cast<std::uint32_t>(&thread_return_trap);
   frame[riscv::frame_gp]      = gp;    /* the image's, should it use one */
   frame[riscv::frame_tp]      = 0u;    /* no TLS yet                    */
   frame[riscv::frame_a0]      = reinterpret_cast<std::uint32_t>(arg);

   context->frame = frame;
   context->guard = guard;
}

void cyros_port_context_destroy(cyros_port_context* context)
{
   CYROS_ASSERT(context != nullptr);
   /* Nothing is owned. Poisoned, so a resume faults on a null frame. */
   context->frame = nullptr;
   context->guard = 0u;
}

CYROS_PORT_HOT void cyros_port_switch(cyros_port_context* from, cyros_port_context* to)
{
   core_state& state = cores[this_core()];

   /* Reachable only from the reschedule's dispatch, where `frame` is the
    * running thread's whole frame. Anywhere else this would save garbage. */
   CYROS_ASSERT_OP(state.trap_depth, ==, 1u);
   CYROS_ASSERT(to != nullptr);
   CYROS_ASSERT(to->frame != nullptr);

   if (from != nullptr) {
      from->frame = state.frame;
   }
   state.frame = to->frame;

   /* The incoming thread's guard. The trap runs on the interrupt stack, and
    * its exit reads the new frame, which lies above the new guard, so the
    * moment of the move is free. */
   if (guarded()) {
      riscv::write_pmpaddr0(to->guard);
   }
}

CYROS_PORT_HOT bool cyros_port_masked_from_baseline(cyros_mask_token_t token)
{
   /* MIE set before the mask means no trap was in progress, since handlers
    * run with it clear. MSIE, the preempt grade, no mask touches. */
   return (token & riscv::mstatus_mie) != 0u && (riscv::read_mie() & riscv::mie_msie) != 0u;
}

/* Not marked [[noreturn]], as on ARM: port_core.h declares it plain. */
void cyros_port_start_first(cyros_port_context* first)
{
   CYROS_ASSERT(first != nullptr);
   CYROS_ASSERT(first->frame != nullptr);

   core_state& state = cores[this_core()];
   state.trap_depth = 0u;
   state.frame = first->frame;

   /* The stack this core is standing on becomes its interrupt stack. Nothing
    * above it is ever returned to, and everything a handler uses lies below. */
   std::uintptr_t sp;
   asm volatile("mv %0, sp" : "=r"(sp));
   state.interrupt_stack_top = sp & ~static_cast<std::uintptr_t>(15u);
   riscv::write_mscratch(state.interrupt_stack_top);

   /* The first thread's guard, and entry 0 on from here: only threads run on
    * guarded stacks, never this core's boot or interrupt stack. */
   if (guarded()) {
      riscv::write_pmpaddr0(first->guard);
      set_guard_enabled(guard_cfg);
   }

   asm volatile(
      "mv   a0, %0               \n"
      "j    cyros_port_trap_exit \n"
      :
      : "r"(first->frame)
      : "a0", "memory");

   __builtin_unreachable();
}


/* ============================================================================
 * Reschedule Requests
 * ========================================================================= */

CYROS_PORT_HOT void cyros_port_thread_yield(void)
{
   /* port.h asks ports that can observe the execution priority to assert the
    * baseline precondition. This one can, and the call frame relies on it:
    * its exit unmasks unconditionally. */
   CYROS_ASSERT_OP(cores[this_core()].trap_depth, ==, 0u);
   CYROS_ASSERT((riscv::read_mstatus() & riscv::mstatus_mie) != 0u);
   CYROS_ASSERT((riscv::read_mie() & riscv::mie_msie) != 0u);

   /* Synchronous: the reschedule runs inside the call, which returns when this
    * thread is next switched in. A tail call with the checks built out. */
   cyros_port_yield_entry();
}

CYROS_PORT_HOT void cyros_port_pend_reschedule(void)
{
   /* Callable from anywhere. At baseline the next safe point is now
    * (port_core.h): the soft IRQ would be taken a few cycles after its store,
    * with a trap frame, so the switch is made here instead, as a yield makes
    * it, with a call frame. Anywhere else, raise the soft IRQ, which is taken
    * when MIE and MSIE are next both set. */
   if ((riscv::read_mstatus() & riscv::mstatus_mie) != 0u && (riscv::read_mie() & riscv::mie_msie) != 0u) {
      cyros_port_yield_entry();
      return;
   }
   riscv::soft_irq_raise(this_core());
}


/* ============================================================================
 * Thread-Local Storage
 * ========================================================================= */

void cyros_port_set_tls_pointer(void* tls_base)
{
   asm volatile("mv tp, %0" : : "r"(tls_base) : "memory");
}

void* cyros_port_get_tls_pointer(void)
{
   void* tls;
   asm volatile("mv %0, tp" : "=r"(tls));
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
   /* WFI wakes on any interrupt enabled in mie, whatever MIE says, so this is
    * safe in the kernel's idle loop regardless of masking. */
   asm volatile("wfi" ::: "memory");
}


/* ============================================================================
 * Measurement
 * ========================================================================= */

std::uint64_t cyros_port_timestamp(void)
{
   /* The target's counter (riscv.hpp). mcycle on a target content with
    * per-hart stamps, a shared counter where they must compare. */
   return riscv::timestamp();
}


/* ============================================================================
 * Debug & Diagnostics
 * ========================================================================= */

void cyros_port_system_error(std::uintptr_t auxilary1,
                             std::uintptr_t auxilary2,
                             char const* file_optional,
                             int line_optional)
{
   riscv::clear_mstatus(riscv::mstatus_mie);

   riscv::write0("\n*** CYROS PANIC ***\n  aux1 = ");
   riscv::write_hex(static_cast<std::uint32_t>(auxilary1));
   riscv::write0("\n  aux2 = ");
   riscv::write_hex(static_cast<std::uint32_t>(auxilary2));

   if (file_optional != nullptr && file_optional[0] != '\0') {
      riscv::write0("\n  at   = ");
      riscv::write0(file_optional);
      riscv::write0(":");
      riscv::write_hex(static_cast<std::uint32_t>(line_optional));
   }
   riscv::write0("\n");

   riscv::host_exit(1u);
}

void cyros_port_wait_for_debugger(void)
{
   static volatile int resume = 0;
   riscv::write0("cyros: waiting for debugger, set 'resume' to 1\n");
   while (resume == 0) { asm volatile("nop"); }
}

void cyros_port_breakpoint(void)
{
   asm volatile("ebreak");
}

void* cyros_port_get_stack_pointer(void)
{
   void* sp = nullptr;
   asm volatile("mv %0, sp" : "=r"(sp));
   return sp;
}
