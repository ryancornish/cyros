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
 * `cyros_port_thread_yield` is an `ecall`, which enters the same path
 * synchronously. A soft IRQ write is a store to a peripheral and lands a few
 * cycles after the instruction, so a yield built on it could return before the
 * switch, which would break its strong guarantee. An ecall cannot.
 *
 *
 * What a trap saves, and where
 * ============================
 * RISC-V saves nothing in hardware but the pc, so the entry below stores the
 * whole integer frame, 32 words, on the INTERRUPTED stack (riscv.hpp has the
 * layout). A suspended thread is then just a pointer to its frame, and
 * cyros_port_switch is one store, as on ARM. Then it moves to the core's
 * interrupt stack before calling C, so a thread stack needs room for one
 * frame and never for the deepest handler chain. The interrupt stack is the
 * stack the core was standing on when it started its first thread, which is
 * the role MSP plays on ARM, and `mscratch` holds its top.
 *
 * `mscratch` reads ZERO while a trap is being handled. A trap taken inside a
 * handler, which can only be a fault, then stays on the stack it is on rather
 * than restarting the interrupt stack from the top over its own caller.
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
 * up to 508 bytes of each buffer.
 *
 * What it cannot catch, as on ARMv7-M: a frame larger than the guard whose
 * stores all miss it, the sp stepping over it in one adjustment.
 */

#include <cyros/port/port_core.h>

/* For cyros_port_get_core_id, as in the ARM layer: a core layer may call the
 * MCU contract, it may not implement it. On a single-core target the call
 * folds away (this_core below). */
#include <cyros/port/port_mcu.h>

#include "riscv.hpp"

#include <cstddef>
#include <cstdint>

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

/* PMP entry 0's configuration byte, from the target, or 0 for no guard.
 * Written by the bootstrap core in cyros_port_init, read everywhere after. */
std::uint8_t guard_cfg = 0u;

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
void run_reschedule(core_state& state)
{
   CYROS_ASSERT_OP(state.trap_depth, ==, 1u);   /* never from inside a handler */
   CYROS_ASSERT(reschedule_handler != nullptr);
   reschedule_handler();
}

} // namespace


/* ============================================================================
 * The trap path
 * ========================================================================= */

/**
 * @brief Every trap on every core. mtvec points here, direct mode.
 *
 * Naked because the prologue and epilogue ARE the mechanism. Stores the frame
 * on the interrupted stack, moves to the interrupt stack (or stays put when a
 * trap is already in progress, see the file comment), calls the dispatcher
 * with the frame, and restores whatever frame the dispatcher hands back, which
 * is a DIFFERENT thread's after a switch.
 *
 * `cyros_port_trap_exit` is the second half on its own, which is also how
 * cyros_port_start_first enters a thread for the first time.
 */
extern "C" [[gnu::naked, gnu::aligned(4)]] void cyros_port_trap_entry(void)
{
   asm volatile(
      "addi  sp, sp, -128           \n"
      "sw    x1,    4(sp)           \n"
      "sw    x3,   12(sp)           \n"
      "sw    x4,   16(sp)           \n"
      "sw    x5,   20(sp)           \n"
      "sw    x6,   24(sp)           \n"
      "sw    x7,   28(sp)           \n"
      "sw    x8,   32(sp)           \n"
      "sw    x9,   36(sp)           \n"
      "sw    x10,  40(sp)           \n"
      "sw    x11,  44(sp)           \n"
      "sw    x12,  48(sp)           \n"
      "sw    x13,  52(sp)           \n"
      "sw    x14,  56(sp)           \n"
      "sw    x15,  60(sp)           \n"
      "sw    x16,  64(sp)           \n"
      "sw    x17,  68(sp)           \n"
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
      "sw    x28, 112(sp)           \n"
      "sw    x29, 116(sp)           \n"
      "sw    x30, 120(sp)           \n"
      "sw    x31, 124(sp)           \n"
      "csrr  t0, mepc               \n"
      "sw    t0,    0(sp)           \n"   /* x0's slot   */
      "csrr  t0, mstatus            \n"
      "sw    t0,    8(sp)           \n"   /* x2's slot   */
      "mv    a0, sp                 \n"   /* the frame   */
      "csrrw t0, mscratch, zero     \n"   /* interrupt stack top, or 0 if nested */
      "bnez  t0, 1f                 \n"
      "mv    t0, sp                 \n"   /* nested: stay below the frame */
      "1:                           \n"
      "mv    sp, t0                 \n"
      "call  cyros_port_trap_dispatch \n" /* returns the frame to resume in a0 */
      ".global cyros_port_trap_exit \n"
      "cyros_port_trap_exit:        \n"
      "mv    sp, a0                 \n"
      "lw    t0,    0(sp)           \n"
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
      "mret                         \n");
}

/**
 * @brief The C half of every trap. Returns the frame to resume.
 *
 * Interrupts are dispatched by cause: the soft IRQ is the reschedule, the
 * timer and external interrupts go to the target. An `ecall` is a yield. Any
 * other exception is a fault and ends in a panic.
 */
extern "C" std::uint32_t* cyros_port_trap_dispatch(std::uint32_t* frame)
{
   core_state& state = cores[this_core()];
   ++state.trap_depth;
   state.frame = frame;

   std::uint32_t const cause = riscv::read_mcause();
   if ((cause & riscv::mcause_interrupt) != 0u) {
      switch (cause & ~riscv::mcause_interrupt) {
      case riscv::cause_soft_interrupt:
         /* Lowered BEFORE the reschedule, so a request made while it runs
          * (a wake from inside the scheduler) is kept for the next one rather
          * than lost to a late clear. */
         riscv::soft_irq_clear(this_core());
         run_reschedule(state);
         break;
      case riscv::cause_timer_interrupt:
         riscv::timer_interrupt();
         break;
      case riscv::cause_ext_interrupt:
         riscv::external_interrupt();
         break;
      default:
         unexpected_trap(cause, frame[riscv::frame_mepc]);
      }
   }
   else if (cause == riscv::cause_ecall_machine && state.trap_depth == 1u) {
      /* A yield. Resume after the ecall, which is never compressed. Done on
       * THIS thread's frame, before the switch replaces it. */
      frame[riscv::frame_mepc] += 4u;
      run_reschedule(state);
   }
   else {
      /* A fault. The handler is the application's if it supplied one, and
       * returning from it resumes the frame. */
      cyros_riscv_fault_handler(cause, frame);
   }

   --state.trap_depth;
   if (state.trap_depth == 0u) {
      /* Re-arm the interrupt stack for the next trap. mret is still ahead and
       * MIE stays clear until it, so nothing can take a trap in between. */
      riscv::write_mscratch(state.interrupt_stack_top);
   }
   return state.frame;
}


/* The trap entry's own extent, to recognise a fault taken while it saved a
 * frame. cyros_port_trap_exit is the label after its last store. */
extern "C" void cyros_port_trap_exit(void);

/**
 * @brief The default fault handler: report, and panic.
 *
 * Weak, so an application can take faults itself (riscv.hpp). An access fault
 * inside the trap entry means the entry could not save a frame below the
 * interrupted sp, which with the guard on is a stack overflow: the frame was
 * headed into the guard, and the original faulting instruction is lost.
 */
extern "C" [[gnu::weak]] void cyros_riscv_fault_handler(std::uint32_t mcause, std::uint32_t* frame)
{
   std::uint32_t const epc = frame[riscv::frame_mepc];
   auto const entry = reinterpret_cast<std::uint32_t>(&cyros_port_trap_entry);
   auto const exit  = reinterpret_cast<std::uint32_t>(&cyros_port_trap_exit);
   bool const access = mcause == riscv::cause_load_access || mcause == riscv::cause_store_access;
   if (access && epc >= entry && epc < exit) {
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
   CYROS_ASSERT_OP(stack_guard_setup(), ==, guard_cfg);
   set_guard_enabled(0u);
}

} // namespace cyros::port::riscv

void cyros_port_init(cyros_port_reschedule_t handler)
{
   CYROS_ASSERT(handler != nullptr);
   /* The bootstrap core decides whether there is a guard, and every core's
    * init_this_core, this one's included, checks it agrees. */
   guard_cfg = riscv::stack_guard_setup();
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

cyros_mask_token_t cyros_port_irq_save(void)
{
   std::uint32_t previous;
   asm volatile("csrrci %0, mstatus, 8" : "=r"(previous) : : "memory");
   return previous & riscv::mstatus_mie;
}

void cyros_port_irq_restore(cyros_mask_token_t token)
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

cyros_mask_token_t cyros_port_preempt_disable(void)
{
   std::uint32_t previous;
   asm volatile("csrrci %0, mie, 8" : "=r"(previous) : : "memory");
   return previous & riscv::mie_msie;
}

void cyros_port_preempt_enable(cyros_mask_token_t token)
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
   if (guard_cfg != 0u) {
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

void cyros_port_switch(cyros_port_context* from, cyros_port_context* to)
{
   core_state& state = cores[this_core()];

   /* Reachable only from the trap path, where `frame` is the running thread's
    * complete frame. Anywhere else this would save garbage. */
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
   if (guard_cfg != 0u) {
      riscv::write_pmpaddr0(to->guard);
   }
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
   if (guard_cfg != 0u) {
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

void cyros_port_thread_yield(void)
{
   /* port.h asks ports that can observe the execution priority to assert the
    * baseline precondition. This one can. */
   CYROS_ASSERT_OP(cores[this_core()].trap_depth, ==, 0u);
   CYROS_ASSERT((riscv::read_mstatus() & riscv::mstatus_mie) != 0u);
   CYROS_ASSERT((riscv::read_mie() & riscv::mie_msie) != 0u);

   /* Synchronous: the trap path runs the reschedule before the next
    * instruction, and resumes here when this thread is next switched in. */
   asm volatile("ecall" ::: "memory");
}

void cyros_port_pend_reschedule(void)
{
   /* Callable from anywhere. Raising the soft IRQ is the whole implementation:
    * it is taken when MIE and MSIE are both set, which is port.h's next safe
    * point. */
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

uint64_t cyros_port_timestamp(void)
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
