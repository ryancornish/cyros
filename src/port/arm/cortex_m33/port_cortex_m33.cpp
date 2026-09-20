/**
 * @file port_cortex_m33.cpp
 * @brief Bare-metal Cortex-M33 (ARMv8-M Mainline) port.
 *
 * Bench is QEMU's mps2-an505. Hardware target is an STM32U575. Everything this
 * file touches is architectural, so the two are the same target as far as the
 * port is concerned.
 *
 *
 * The central decision: ALL SWITCHING HAPPENS IN PendSV
 * =====================================================
 * port.h's own implementation notes prescribe this, and the hardware makes it
 * the only shape that is simultaneously correct from thread context and from an
 * ISR. PendSV is configured at the LOWEST exception priority, so it cannot be
 * taken while any other handler is active. That gives two properties for free:
 *
 *  - A reschedule requested from an ISR is delivered when the ISR chain
 *    unwinds, not during it.
 *  - cyros_port_switch() is therefore only ever called from Handler mode, with
 *    the outgoing thread's state already complete on its own PSP stack. It
 *    reduces to swapping which stack PSP points at.
 *
 * The consequence worth stating plainly: the "reschedule pending" FLAG that the
 * Linux ports maintain by hand does not exist here. ICSR.PENDSVSET is that flag,
 * and the hardware resolves it at exactly the safe points port.h defines. This
 * port is smaller than the Linux ones because the silicon already implements
 * most of the contract.
 *
 *
 * Masking: two hardware registers, not two counters
 * =================================================
 * The Linux preempt port derives a signal mask from two depth counters, because
 * a POSIX signal mask cannot express "block the scheduler but not devices". The
 * M-profile can:
 *
 *   Interrupt masking  (cyros_port_irq_save/restore)     -> PRIMASK
 *   Preemption control (cyros_port_preempt_disable/enable) -> BASEPRI
 *
 * PRIMASK blocks everything. BASEPRI raised to PendSV's own priority blocks
 * PendSV and nothing else, because PendSV is alone at the lowest level, so
 * device ISRs continue to run while no context switch can occur. That is
 * precisely port.h's distinction between the two facilities.
 *
 * Both are save/restore rather than counted. The TOKEN carries the previous
 * register value, so nesting composes without a depth counter, and a critical
 * section entered while already masked restores to masked rather than to open.
 * That last case is the one no Linux test can observe, because the token is
 * inert there (see ~/cyros-claude/roadmap.md, P1's "known trap").
 *
 * Baseline priority, in port.h's sense, is PRIMASK == 0 AND BASEPRI == 0 in
 * Thread mode. When the outermost restore of either register reaches that
 * state, a pended PendSV is taken before the next instruction retires. The
 * hardware IS the safe point.
 *
 *
 * Priority numbering, and why it is discovered rather than hardcoded
 * =================================================================
 * Cortex-M implements only the top N bits of each 8-bit priority field, and N
 * is an implementation choice: 3 on the MPS2 bench, 4 on the STM32U575. A
 * hardcoded "SysTick = 0xE0, PendSV = 0xFF" collapses to the SAME level when
 * N is 3, which would let SysTick be masked by a preempt-disable and silently
 * stop the clock inside every critical section. init() writes 0xFF to a
 * priority register and reads back which bits stuck, then derives the two
 * values from that.
 */

#include <cyros/port/port.h>

#include "cortex_m.hpp"

#include <cstddef>
#include <cstdint>

namespace cortex_m = cyros::port::cortex_m;

/* ============================================================================
 * Context
 * ========================================================================= */

/**
 * @brief A suspended thread's port state.
 *
 * Only three words, because the registers themselves live on the thread's own
 * stack: the hardware pushes r0-r3/r12/LR/PC/xPSR on exception entry and the
 * PendSV prologue pushes r4-r11 below them.
 */
struct cyros_port_context
{
   std::uint32_t sp;           /* PSP, pointing at the saved r4 slot */
   std::uint32_t stack_limit;  /* PSPLIM for this thread             */
   void*         tls;          /* saved thread TLS base              */
};

static_assert(sizeof(cyros_port_context) <= CYROS_PORT_CONTEXT_SIZE,
              "CYROS_PORT_CONTEXT_SIZE in port_traits.h is too small for this port");
static_assert(alignof(cyros_port_context) <= CYROS_PORT_CONTEXT_ALIGN,
              "CYROS_PORT_CONTEXT_ALIGN in port_traits.h is too weak for this port");

namespace
{

/* The kernel's reschedule entry point, installed by cyros_port_init. */
cyros_port_reschedule_t reschedule_handler = nullptr;

/* Current thread's TLS base. Swapped by cyros_port_switch so that a thread
 * carries its own across a switch, which the Linux ports get from pthread TLS
 * and this target has to do by hand. */
void* current_tls = nullptr;

/* Derived in cyros_port_init once the implemented priority width is known. */
std::uint32_t pendsv_priority  = 0xFFu;
std::uint32_t systick_priority = 0xFFu;

/* The BASEPRI value that masks PendSV and nothing else. Equal to
 * pendsv_priority: BASEPRI masks exceptions whose priority value is greater
 * than or equal to it, and PendSV is alone at the bottom. */
std::uint32_t preempt_mask_value = 0xFFu;

/**
 * @brief Count the priority bits this implementation actually decodes.
 *
 * Write all ones, read back, and the zero bits are the ones the hardware
 * discards. Architecturally guaranteed to be the top N bits.
 */
std::uint32_t discover_priority_bits() noexcept
{
   std::uint8_t const saved = cortex_m::reg8(cortex_m::shpr_pendsv);
   cortex_m::reg8(cortex_m::shpr_pendsv) = 0xFFu;
   std::uint8_t const readback = cortex_m::reg8(cortex_m::shpr_pendsv);
   cortex_m::reg8(cortex_m::shpr_pendsv) = saved;

   std::uint32_t bits = 0;
   for (std::uint32_t mask = 0x80u; mask != 0u; mask >>= 1) {
      if ((readback & mask) == 0u) { break; }
      ++bits;
   }
   return bits;
}

/**
 * @brief Catch a thread entry point that returns.
 *
 * port.h says a thread entry MUST NOT return and leaves enforcement to the
 * port. Sitting in the initial frame's LR slot costs nothing and turns an
 * otherwise unbounded jump into a named panic.
 */
[[noreturn]] void thread_return_trap()
{
   cyros_port_system_error(0, 0, "thread entry returned", 0);
}

} // namespace


/* ============================================================================
 * Platform Initialisation
 * ========================================================================= */

void cyros_port_init(cyros_port_reschedule_t handler)
{
   CYROS_ASSERT(handler != nullptr);
   reschedule_handler = handler;
   current_tls = nullptr;

   /* Mask everything for the whole of bring-up. Nothing may preempt the kernel
    * between here and cyros_port_start_first, which is the moment the first
    * thread stack exists and PSP becomes meaningful. A SysTick landing in that
    * window would stack an exception frame onto MSP and then try to switch away
    * from a thread that does not exist. */
   cortex_m::disable_irq();
   cortex_m::set_basepri(0u);

   std::uint32_t const bits = discover_priority_bits();
   CYROS_ASSERT_OP(bits, >=, 2u);   /* need at least two distinguishable levels */
   CYROS_ASSERT_OP(bits, <=, 8u);

   /* Granularity comes from TWO independent places and the coarser one wins.
    *
    * The first is how many priority bits the part implements, which is what
    * discover_priority_bits found.
    *
    * The second is AIRCR.PRIGROUP, and MISSING IT IS A REAL BUG THAT THIS PORT
    * SHIPPED FOR AN AFTERNOON. PRIGROUP splits every priority field into a
    * GROUP part and a SUB-priority part, and preemption and BASEPRI masking
    * consider ONLY the group part. At the reset default of 0 the bottom bit is
    * sub-priority, so on this bench, where all 8 bits are implemented, PendSV
    * at 0xFF and SysTick at 0xFE landed in the SAME group. Raising BASEPRI to
    * PendSV's level then masked SysTick too, and every kernel critical section
    * silently stopped the clock.
    *
    * It was invisible to every check that looks at the priority NUMBERS,
    * including this port's own layer-0 test, which asserted only that the two
    * values differ. It took an interrupt actually firing to find, which is why
    * test_cortex_m33_systick exists.
    *
    * PRIGROUP is read rather than written, because an application may have set
    * it for its own device IRQs. The cost is that changing PRIGROUP after
    * cyros_port_init is not supported. */
   std::uint32_t const implemented_step = 1u << (8u - bits);

   std::uint32_t const prigroup =
      (cortex_m::reg(cortex_m::scb_aircr) >> cortex_m::aircr_prigroup_shift) & cortex_m::aircr_prigroup_mask;
   std::uint32_t const group_step = 1u << (prigroup + 1u);

   std::uint32_t const step = implemented_step > group_step ? implemented_step : group_step;

   /* PendSV lowest, SysTick a full preemption group above it. Any device IRQ
    * must be configured at systick_priority or above (numerically lower) to
    * remain serviceable while preemption is disabled. */
   pendsv_priority  = 0xFFu & ~(implemented_step - 1u);
   systick_priority = pendsv_priority - step;
   preempt_mask_value = pendsv_priority;

   cortex_m::reg8(cortex_m::shpr_pendsv)  = static_cast<std::uint8_t>(pendsv_priority);
   cortex_m::reg8(cortex_m::shpr_systick) = static_cast<std::uint8_t>(systick_priority);

   /* Clear any reschedule left pending by a previous kernel lifecycle. The unit
    * tests initialise and finalise repeatedly, and a stale PENDSVSET would fire
    * into the next lifecycle's first thread. */
   cortex_m::reg(cortex_m::scb_icsr) = cortex_m::icsr_pendstclr;

   cortex_m::dsb();
   cortex_m::isb();
}


/* ============================================================================
 * SMP & Multi-Core Support
 * ========================================================================= */

std::uint32_t cyros_port_get_core_id(void)
{
   return 0u;
}

void cyros_port_start_cores(std::size_t cores_to_use, cyros_port_core_entry_t entry)
{
   /* Single core. A dual-M33 part (RP2350, or QEMU's mps2-an521) would start
    * the second core here, and is a separate port variant rather than a switch
    * in this one. */
   CYROS_ASSERT_OP(cores_to_use, ==, 1u);
   CYROS_ASSERT(entry != nullptr);

   entry();

   /* entry() reaches cyros_port_start_first, which never returns. */
   CYROS_PORT_UNREACHABLE();
}

void cyros_port_send_reschedule_ipi(std::uint32_t core_id)
{
   /* On one core the cross-core request degenerates to a local one, and carries
    * the same weak guarantee. */
   CYROS_ASSERT_OP(core_id, ==, 0u);
   cyros_port_pend_reschedule();
}


/* ============================================================================
 * Interrupt Control
 * ========================================================================= */

bool cyros_port_interrupts_enabled(void)
{
   return cortex_m::get_primask() == 0u;
}

cyros_mask_token_t cyros_port_irq_save(void)
{
   cyros_mask_token_t const token = cortex_m::get_primask();
   cortex_m::disable_irq();
   return token;
}

void cyros_port_irq_restore(cyros_mask_token_t token)
{
   /* Restores to MASKED when the token says the caller was already masked.
    * That asymmetry is the whole point of a token, and is what the Linux ports
    * cannot check because theirs carries no state. */
   cortex_m::set_primask(token);
}


/* ============================================================================
 * Preemption Control
 * ========================================================================= */

cyros_mask_token_t cyros_port_preempt_disable(void)
{
   cyros_mask_token_t const token = cortex_m::get_basepri();
   cortex_m::set_basepri(preempt_mask_value);
   /* BASEPRI takes effect immediately for subsequent instructions, but an
    * exception already in flight may still arrive. The barrier is what makes
    * "no switch after this returns" true rather than nearly true. */
   cortex_m::dsb();
   cortex_m::isb();
   return token;
}

void cyros_port_preempt_enable(cyros_mask_token_t token)
{
   cortex_m::set_basepri(token);
   cortex_m::dsb();
   cortex_m::isb();
   /* If this dropped BASEPRI to zero and PRIMASK is clear, a pended PendSV has
    * already been taken by the time the ISB retires. No software safe-point
    * check is needed or wanted. */
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
   /* 16 words of initial frame, plus room to be useful. */
   CYROS_ASSERT_OP(stack_size, >=, 128u);

   auto const base = reinterpret_cast<std::uintptr_t>(stack_base);

   /* AAPCS wants 8-byte alignment at a public interface, and ARMv8-M's PSPLIM
    * is 8-byte granular, so both ends round toward the middle. */
   std::uintptr_t const limit = (base + 7u) & ~static_cast<std::uintptr_t>(7u);
   std::uintptr_t const top   = (base + stack_size) & ~static_cast<std::uintptr_t>(7u);
   CYROS_ASSERT_OP(top, >, limit);

   auto* sp = reinterpret_cast<std::uint32_t*>(top);

   /* The frame PendSV's epilogue and the hardware's exception return will
    * consume, built in the order they are unstacked: highest address first. */
   *--sp = 0x01000000u;                                          /* xPSR, Thumb bit  */
   *--sp = reinterpret_cast<std::uint32_t>(entry) & ~1u;         /* PC               */
   *--sp = reinterpret_cast<std::uint32_t>(&thread_return_trap); /* LR               */
   *--sp = 0u;                                                   /* r12              */
   *--sp = 0u;                                                   /* r3               */
   *--sp = 0u;                                                   /* r2               */
   *--sp = 0u;                                                   /* r1               */
   *--sp = reinterpret_cast<std::uint32_t>(arg);                 /* r0, the argument */

   /* Callee-saved, stored r11 down to r4 so that memory reads r4..r11 upward,
    * matching the stmdb/ldmia pair in the PendSV handler. */
   for (int i = 0; i < 8; ++i) { *--sp = 0u; }

   context->sp          = reinterpret_cast<std::uint32_t>(sp);
   context->stack_limit = static_cast<std::uint32_t>(limit);
   context->tls         = nullptr;
}

void cyros_port_context_destroy(cyros_port_context* context)
{
   CYROS_ASSERT(context != nullptr);

   /* Nothing is owned. Unlike the Linux preempt port, whose context holds a
    * heap-allocated FP area, everything here lives in the caller's stack
    * buffer. Poisoning is worth the three stores: a resume of a destroyed
    * context then faults on a null PSP instead of running stale registers. */
   context->sp          = 0u;
   context->stack_limit = 0u;
   context->tls         = nullptr;
}

void cyros_port_switch(cyros_port_context* from, cyros_port_context* to)
{
   /* Reachable only from the PendSV handler. If this ever fires, something is
    * calling the scheduler's switch from thread context, where PSP does not
    * hold a complete frame and this function would corrupt it. */
   CYROS_ASSERT(cortex_m::in_handler_mode());
   CYROS_ASSERT(to != nullptr);
   CYROS_ASSERT(to->sp != 0u);

   if (from != nullptr) {
      from->sp  = cortex_m::get_psp();
      from->tls = current_tls;
   }

   /* Drop the limit before moving PSP. An MSR to PSP is checked against the
    * OUTGOING thread's PSPLIM, so setting the new stack first would fault
    * whenever the incoming stack sits below the outgoing one. */
   cortex_m::set_psplim(0u);
   cortex_m::set_psp(to->sp);
   cortex_m::set_psplim(to->stack_limit);

   current_tls = to->tls;
}

/**
 * @brief The PendSV handler. Every context switch in the system passes here.
 *
 * Naked because the prologue and epilogue ARE the mechanism: a compiler-emitted
 * frame would save the wrong registers to the wrong stack. The handler runs on
 * MSP while the thread it interrupted is stacked on PSP.
 */
extern "C" [[gnu::naked]] void PendSV_Handler(void)
{
   asm volatile(
      "mrs   r0, psp                    \n"  /* the interrupted thread's stack  */
      "stmdb r0!, {r4-r11}              \n"  /* complete its saved state        */
      "msr   psp, r0                    \n"
      "push  {r3, lr}                   \n"  /* EXC_RETURN, r3 keeps MSP 8-aligned */
      "bl    cyros_port_pendsv_dispatch \n"  /* may call cyros_port_switch      */
      "pop   {r3, lr}                   \n"
      "mrs   r0, psp                    \n"  /* possibly a DIFFERENT stack now  */
      "ldmia r0!, {r4-r11}              \n"
      "msr   psp, r0                    \n"
      "bx    lr                         \n"  /* exception return unstacks the rest */
   );
}

/**
 * @brief The C half of the PendSV handler.
 *
 * Separate so the assembly above stays free of anything the compiler could
 * reorder, and so the scheduler is reached through an ordinary call.
 */
extern "C" void cyros_port_pendsv_dispatch(void)
{
   CYROS_ASSERT(reschedule_handler != nullptr);
   reschedule_handler();
}

/* Not marked [[noreturn]] even though it never returns on this port: port.h
 * declares it plain, and a definition may not add the attribute. The contract
 * allows a cooperative simulation port to return from here, so the declaration
 * is right and this port is simply stricter than it has to be. */
void cyros_port_start_first(cyros_port_context* first)
{
   CYROS_ASSERT(first != nullptr);
   CYROS_ASSERT(first->sp != 0u);

   current_tls = first->tls;

   cortex_m::set_psplim(0u);
   cortex_m::set_psp(first->sp);
   cortex_m::set_psplim(first->stack_limit);

   /* Entering the first thread cannot use an exception return, because we are
    * in Thread mode and EXC_RETURN values are only meaningful in Handler mode.
    * So the frame is unstacked by hand, in the order the hardware would.
    *
    * The one register that cannot survive is r1: it has to carry the entry
    * address across the pop that restores r0-r3. That is architecturally fine.
    * AAPCS defines r0 as the argument and leaves r1-r3 undefined at a function
    * entry point, which is the only place this lands. */
   asm volatile(
      "msr   control, %[spsel]  \n"  /* Thread mode on PSP, still privileged   */
      "isb                      \n"
      "pop   {r4-r11}           \n"  /* callee-saved, now from the thread stack */
      "ldr   r1, [sp, #24]      \n"  /* PC out of the frame                    */
      "str   r1, [sp, #28]      \n"  /* park it in the xPSR slot, which we drop */
      "pop   {r0-r3, r12, lr}   \n"  /* r0 = arg, lr = thread_return_trap      */
      "pop   {r1}               \n"  /* discard the original PC slot           */
      "pop   {r1}               \n"  /* r1 = entry                             */
      /* Set the Thumb bit. The stacked PC has bit 0 CLEAR, which is what an
       * exception return wants because xPSR.T supplies the instruction set
       * there. This path is a plain BX instead, and BX to an address with bit 0
       * clear means "switch to ARM state" - which M-profile does not have. It
       * raises an INVSTATE UsageFault that escalates to HardFault, with the
       * faulting address nowhere in sight. */
      "orr   r1, r1, #1         \n"
      "cpsie i                  \n"  /* the thread begins at baseline priority */
      "bx    r1                 \n"
      :
      : [spsel] "r"(2u)
      : "memory");

   __builtin_unreachable();
}


/* ============================================================================
 * Reschedule Requests
 * ========================================================================= */

void cyros_port_thread_yield(void)
{
   /* port.h asks ports that can observe the execution priority to assert the
    * baseline precondition here. This one can, so it does. */
   CYROS_ASSERT(!cortex_m::in_handler_mode());
   CYROS_ASSERT_OP(cortex_m::get_primask(), ==, 0u);
   CYROS_ASSERT_OP(cortex_m::get_basepri(), ==, 0u);

   cortex_m::reg(cortex_m::scb_icsr) = cortex_m::icsr_pendsvset;
   cortex_m::dsb();
   cortex_m::isb();

   /* At baseline priority PendSV is taken at the ISB above, so by the time
    * control returns here a full reschedule round trip has completed. That is
    * the strong guarantee, delivered by the hardware rather than asserted. */
}

void cyros_port_pend_reschedule(void)
{
   /* Callable from anywhere, including an ISR and inside a critical section.
    * Setting PENDSVSET is the whole implementation: the hardware holds the
    * request until the execution priority drops to PendSV's level, which is
    * exactly port.h's "next safe point". */
   cortex_m::reg(cortex_m::scb_icsr) = cortex_m::icsr_pendsvset;
   cortex_m::dsb();
   cortex_m::isb();
}


/* ============================================================================
 * Thread-Local Storage
 * ========================================================================= */

void cyros_port_set_tls_pointer(void* tls_base)
{
   current_tls = tls_base;
}

void* cyros_port_get_tls_pointer(void)
{
   return current_tls;
}


/* ============================================================================
 * CPU Hints & Idle
 * ========================================================================= */

void cyros_port_cpu_relax(void)
{
   /* YIELD is a hint with no effect on a single-threaded core, but it is the
    * architecturally correct marker for a spin body and costs one cycle. */
   asm volatile("yield" ::: "memory");
}

void cyros_port_idle(void)
{
   /* WFI wakes on any enabled interrupt even while PRIMASK masks it, so this
    * is safe inside the kernel's idle loop regardless of masking state. */
   cortex_m::wfi();
}


/* ============================================================================
 * Debug & Diagnostics
 * ========================================================================= */

void cyros_port_system_error(std::uintptr_t auxilary1,
                             std::uintptr_t auxilary2,
                             char const* file_optional,
                             int line_optional)
{
   /* Mask first. A panic that gets preempted reports the wrong thing, and on
    * this target the report goes out one semihosting trap at a time. */
   cortex_m::disable_irq();

   cortex_m::write0("\n*** CYROS PANIC ***\n  aux1 = ");
   cortex_m::write_hex(static_cast<std::uint32_t>(auxilary1));
   cortex_m::write0("\n  aux2 = ");
   cortex_m::write_hex(static_cast<std::uint32_t>(auxilary2));

   if (file_optional != nullptr && file_optional[0] != '\0') {
      cortex_m::write0("\n  at   = ");
      cortex_m::write0(file_optional);
      cortex_m::write0(":");
      cortex_m::write_hex(static_cast<std::uint32_t>(line_optional));
   }
   cortex_m::write0("\n");

   cortex_m::host_exit(1u);
}

void cyros_port_wait_for_debugger(void)
{
   /* Cleared from gdb with `set var resume = 1`. Volatile so the loop is real
    * and the variable is addressable. */
   static volatile int resume = 0;
   cortex_m::write0("cyros: waiting for debugger, set 'resume' to 1\n");
   while (resume == 0) { cortex_m::nop(); }
}

void cyros_port_breakpoint(void)
{
   /* With no debugger attached this is a HardFault rather than a stop, which
    * is the honest outcome: nothing is listening. */
   asm volatile("bkpt 0x00");
}

void* cyros_port_get_stack_pointer(void)
{
   void* sp = nullptr;
   asm volatile("mov %0, sp" : "=r"(sp));
   return sp;
}
