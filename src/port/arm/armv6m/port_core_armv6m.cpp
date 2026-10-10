/**
 * @file port_core_armv6m.cpp
 * @brief ARMv6-M CORE LAYER: all of port_core.h, for the Cortex-M0 and M0+.
 *
 * Bench: QEMU's microbit (a Cortex-M0, cortex_m0 target). Hardware, when it
 * comes, the RP2040's M0+ (roadmap P1). Everything here is architectural.
 *
 * The Mainline layer (armv7m_armv8m) is the model, and its header says why
 * every switch happens in PendSV at the lowest priority. That holds here
 * unchanged. What ARMv6-M takes away, and what this layer does instead:
 *
 *   no BASEPRI     the preempt grade is a software deferral (below)
 *   Thumb-1 only   PendSV saves r8 to r11 through low registers
 *   no LDREX       std::atomic's read-modify-writes are library calls,
 *                  which port_atomic_armv6m.cpp supplies, single core only
 *   no DWT         the timestamp is the target's counter alone
 *   no FPU         nothing to save, and EXC_RETURN is the same for every
 *                  thread, so unlike the Mainline layer it is not saved
 *   no stack guard the M0 has no MPU. The M0+'s optional one is for the
 *                  RP2040 work, by the policy that a guard is hardware or
 *                  nothing
 *
 *
 * The preempt grade: a software deferral (Ryan, 2026-10-04)
 * ========================================================
 * With no BASEPRI, nothing in hardware masks PendSV alone, and PRIMASK would
 * mask every device and SysTick with it, breaking port.h's line between the
 * grades: a preempt-disable must leave interrupts running. So preemption is
 * a per-core depth, and PendSV consults it:
 *
 *   preempt_disable   raises the depth and returns the old one as the token
 *   PendSV            finding the depth raised, records that a switch is
 *                     owed and returns to the same thread
 *   preempt_enable    restores the depth from the token, and at zero with a
 *                     switch owed, pends PendSV again
 *
 * The token restores rather than decrements, as BASEPRI's does on Mainline,
 * so a region entered while already disabled leaves it disabled.
 *
 * The depth is per core, not per thread, and that is sound because no thread
 * is ever switched out with it raised: PendSV defers instead, and a blocking
 * wait yields at baseline (cyros_port_thread_yield asserts it). Every switch
 * therefore happens at depth zero, and every thread resumes at depth zero.
 *
 * The races are benign by construction. An ISR may disable and enable
 * between a thread's read of the depth and its write, but an ISR's regions
 * are balanced, so the thread's write is still right. PendSV may land between
 * an enable's store of zero and its check of the owed flag: PendSV then sees
 * depth zero, clears the flag and switches, and the enable finds nothing
 * owed, or re-pends one more PendSV, which is a redundant pick and never a
 * wrong one.
 *
 * What it costs over BASEPRI: a reschedule pended inside a preempt-disabled
 * region takes PendSV once to be deferred and once more to be delivered, and
 * the deferred entry still saves and restores r4 to r11.
 */

#include <cyros/port/port_core.h>

/* For cyros_port_get_core_id, as in the Mainline layer: per-core state needs
 * the MCU's answer, and on one core the call folds away. */
#include <cyros/port/port_mcu.h>

#include "cortex_m.hpp"

#include <cstddef>
#include <cstdint>

#if CYROS_CORTEX_M_MAINLINE
#  error "armv6m is the ARMv6-M core layer: a Mainline build uses armv7m_armv8m"
#endif

namespace cortex_m = cyros::port::cortex_m;

/* ============================================================================
 * Context
 * ========================================================================= */

/**
 * @brief A suspended thread's port state.
 *
 * Two words: the registers live on the thread's own stack, the hardware's
 * frame (r0 to r3, r12, LR, PC, xPSR) above the eight PendSV stores below it.
 */
struct cyros_port_context
{
   std::uint32_t sp;    /* PSP, pointing at the saved r4 slot */
   void*         tls;   /* saved thread TLS base              */
};

static_assert(sizeof(cyros_port_context) <= CYROS_PORT_CONTEXT_SIZE,
              "CYROS_PORT_CONTEXT_SIZE in port_traits.h is too small for this port");
static_assert(alignof(cyros_port_context) <= CYROS_PORT_CONTEXT_ALIGN,
              "CYROS_PORT_CONTEXT_ALIGN in port_traits.h is too weak for this port");

namespace
{

cyros_port_reschedule_t reschedule_handler = nullptr;

/* Per core, as the Mainline layer's TLS cache is and for the same reason:
 * one running thread per core. Single core today, so each is one slot. */
void* current_tls[CYROS_PORT_CORE_COUNT] = {};

/* The software preempt grade (see the header). Volatile because PendSV reads
 * them asynchronously to the thread that writes them. */
volatile std::uint32_t preempt_depth[CYROS_PORT_CORE_COUNT] = {};
volatile bool switch_owed[CYROS_PORT_CORE_COUNT] = {};

inline std::uint32_t this_core() noexcept
{
   if constexpr (CYROS_PORT_CORE_COUNT == 1) {
      return 0u;
   } else {
      return cyros_port_get_core_id();
   }
}

/* ARMv6-M implements the top two bits of each priority field, so four
 * levels. PendSV takes the lowest, so it runs only when no other handler
 * does, and SysTick the next, so the clock preempts a switch. No PRIGROUP
 * exists to split them. */
constexpr std::uint32_t pendsv_priority  = 0xC0u;
constexpr std::uint32_t systick_priority = 0x80u;

[[noreturn]] void thread_return_trap()
{
   cyros_port_system_error(0, 0, "thread entry returned", 0);
}

inline void compiler_barrier() noexcept { asm volatile("" ::: "memory"); }

} // namespace


/* ============================================================================
 * Platform Initialisation
 * ========================================================================= */

namespace cyros::port::cortex_m
{

std::uint32_t device_irq_priority()
{
   return systick_priority;
}

void init_this_core()
{
   std::uint32_t const core = this_core();
   current_tls[core] = nullptr;
   preempt_depth[core] = 0u;
   switch_owed[core] = false;

   /* Masked for the whole of bring-up, as on Mainline: there is no thread
    * stack to switch from until cyros_port_start_first. */
   cortex_m::disable_irq();

   /* SHPR3 holds both, and ARMv6-M allows only word access to it. */
   std::uint32_t const shpr3 = cortex_m::reg(cortex_m::scb_shpr3);
   cortex_m::reg(cortex_m::scb_shpr3) =
      (shpr3 & 0x0000FFFFu) | (pendsv_priority << 16) | (systick_priority << 24);
   /* Two implemented bits is architectural. A part that kept fewer would
    * collapse the two levels, and this is the cheapest place to notice. */
   CYROS_ASSERT_OP((cortex_m::reg(cortex_m::scb_shpr3) >> 16) & 0xFFu, ==, pendsv_priority);
   CYROS_ASSERT_OP(cortex_m::reg(cortex_m::scb_shpr3) >> 24, ==, systick_priority);

   /* A reschedule or a SysTick left pending by a previous lifecycle. */
   cortex_m::reg(cortex_m::scb_icsr) = cortex_m::icsr_pendsvclr | cortex_m::icsr_pendstclr;

   cortex_m::dsb();
   cortex_m::isb();
}

} // namespace cyros::port::cortex_m

void cyros_port_init(cyros_port_reschedule_t handler)
{
   CYROS_ASSERT(handler != nullptr);
   reschedule_handler = handler;
   cyros::port::cortex_m::init_this_core();
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
   /* Restores to MASKED when the token says the caller was already masked. */
   cortex_m::set_primask(token);
}


/* ============================================================================
 * Preemption Control: the software deferral (see the header)
 * ========================================================================= */

cyros_mask_token_t cyros_port_preempt_disable(void)
{
   std::uint32_t const core = this_core();
   cyros_mask_token_t const token = preempt_depth[core];
   preempt_depth[core] = token + 1u;
   /* Nothing the caller protects may move above the raise. From here a PendSV
    * defers rather than switching, so "no switch after this returns" holds
    * without a barrier instruction: the store is the mechanism. */
   compiler_barrier();
   return token;
}

void cyros_port_preempt_enable(cyros_mask_token_t token)
{
   std::uint32_t const core = this_core();
   compiler_barrier();
   preempt_depth[core] = token;
   if (token == 0u && switch_owed[core]) {
      switch_owed[core] = false;
      /* Taken at the ISB when PRIMASK is clear, as a Mainline enable's is
       * when BASEPRI drops, and otherwise when the caller unmasks. */
      cortex_m::reg(cortex_m::scb_icsr) = cortex_m::icsr_pendsvset;
      cortex_m::dsb();
      cortex_m::isb();
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
   CYROS_ASSERT_OP(stack_size, >=, 128u);

   auto const base = reinterpret_cast<std::uintptr_t>(stack_base);
   std::uintptr_t const bottom = (base + 7u) & ~static_cast<std::uintptr_t>(7u);
   std::uintptr_t const top    = (base + stack_size) & ~static_cast<std::uintptr_t>(7u);
   CYROS_ASSERT_OP(top, >, bottom + 64u);

   auto* sp = reinterpret_cast<std::uint32_t*>(top);

   /* The hardware frame the exception return unstacks, highest address first. */
   *--sp = 0x01000000u;                                          /* xPSR, Thumb bit  */
   *--sp = reinterpret_cast<std::uint32_t>(entry) & ~1u;         /* PC               */
   *--sp = reinterpret_cast<std::uint32_t>(&thread_return_trap); /* LR               */
   *--sp = 0u;                                                   /* r12              */
   *--sp = 0u;                                                   /* r3               */
   *--sp = 0u;                                                   /* r2               */
   *--sp = 0u;                                                   /* r1               */
   *--sp = reinterpret_cast<std::uint32_t>(arg);                 /* r0, the argument */

   /* r11 down to r4, so memory reads r4 to r11 upward, as PendSV stores them. */
   for (int i = 0; i < 8; ++i) { *--sp = 0u; }

   context->sp  = reinterpret_cast<std::uint32_t>(sp);
   context->tls = nullptr;
}

void cyros_port_context_destroy(cyros_port_context* context)
{
   CYROS_ASSERT(context != nullptr);
   /* Nothing owned. Poisoned so a resume of a destroyed context faults on a
    * null PSP instead of running stale registers. */
   context->sp  = 0u;
   context->tls = nullptr;
}

void cyros_port_switch(cyros_port_context* from, cyros_port_context* to)
{
   /* From PendSV only, where PSP holds the outgoing thread's complete frame. */
   CYROS_ASSERT(cortex_m::in_handler_mode());
   CYROS_ASSERT(to != nullptr);
   CYROS_ASSERT(to->sp != 0u);

   std::uint32_t const core = this_core();
   if (from != nullptr) {
      from->sp  = cortex_m::get_psp();
      from->tls = current_tls[core];
   }
   cortex_m::set_psp(to->sp);
   current_tls[core] = to->tls;
}

/**
 * @brief The PendSV handler. Every context switch in the system passes here.
 *
 * Naked, because the prologue and epilogue are the mechanism. Thumb-1 can
 * store and load only r0 to r7 in a block, so r8 to r11 go through r4 to r7
 * after those are saved, and come back the same way before they are
 * restored. The block is laid out r4 to r11 upward from the saved PSP, which
 * is what cyros_port_context_init builds and cyros_port_start_first reads.
 *
 * PSP is lowered to the block BEFORE the dispatch, so cyros_port_switch saves
 * a PSP that points at it. EXC_RETURN rides across the call on MSP.
 */
extern "C" [[gnu::naked]] void PendSV_Handler(void)
{
   /* GCC hands Thumb-1 inline assembly to gas in divided syntax, where
    * `subs r0, #32` is refused, so each block selects unified itself. GCC
    * selects unified again after the block. */
   asm volatile(
      ".syntax unified                  \n"
      "mrs   r0, psp                    \n"
      "subs  r0, #32                    \n"
      "msr   psp, r0                    \n"  /* the saved PSP: the r4 slot      */
      "stmia r0!, {r4-r7}               \n"
      "mov   r4, r8                     \n"
      "mov   r5, r9                     \n"
      "mov   r6, r10                    \n"
      "mov   r7, r11                    \n"
      "stmia r0!, {r4-r7}               \n"
      "push  {r3, lr}                   \n"  /* EXC_RETURN, MSP kept 8-aligned  */
      "bl    cyros_port_pendsv_dispatch \n"  /* may call cyros_port_switch      */
      "pop   {r2, r3}                   \n"  /* r3 = EXC_RETURN                 */
      "mrs   r0, psp                    \n"  /* possibly a DIFFERENT thread now */
      "adds  r0, #16                    \n"
      "ldmia r0!, {r4-r7}               \n"  /* that thread's r8 to r11         */
      "mov   r8, r4                     \n"
      "mov   r9, r5                     \n"
      "mov   r10, r6                    \n"
      "mov   r11, r7                    \n"
      "msr   psp, r0                    \n"  /* its hardware frame              */
      "subs  r0, #32                    \n"
      "ldmia r0!, {r4-r7}               \n"  /* and its r4 to r7                */
      "bx    r3                         \n"  /* exception return unstacks the rest */
   );
}

/**
 * @brief The C half of PendSV: deliver a reschedule, or owe it.
 */
extern "C" void cyros_port_pendsv_dispatch(void)
{
   CYROS_ASSERT(reschedule_handler != nullptr);
   std::uint32_t const core = this_core();
   if (preempt_depth[core] != 0u) {
      /* The interrupted thread disabled preemption. Return to it, and let its
       * outermost enable pend this again. */
      switch_owed[core] = true;
      return;
   }
   switch_owed[core] = false;
   reschedule_handler();
}

void cyros_port_start_first(cyros_port_context* first)
{
   CYROS_ASSERT(first != nullptr);
   CYROS_ASSERT(first->sp != 0u);
   CYROS_ASSERT_OP(preempt_depth[this_core()], ==, 0u);

   current_tls[this_core()] = first->tls;
   cortex_m::set_psp(first->sp);

   /* Thread mode cannot use an exception return, so the frame is unstacked
    * by hand, in the order PendSV and the hardware would. r4 and r5 carry the
    * entry across the last pops and arrive at it with garbage, which AAPCS
    * allows at a function's entry. */
   asm volatile(
      ".syntax unified          \n"  /* as in PendSV_Handler                   */
      "msr   control, %[spsel]  \n"  /* Thread mode on PSP, still privileged   */
      "isb                      \n"
      "pop   {r4-r7}            \n"
      "pop   {r0-r3}            \n"  /* r8 to r11                              */
      "mov   r8, r0             \n"
      "mov   r9, r1             \n"
      "mov   r10, r2            \n"
      "mov   r11, r3            \n"
      "pop   {r0-r3}            \n"  /* r0 = arg                               */
      "pop   {r4, r5}           \n"  /* r12, LR                                */
      "mov   r12, r4            \n"
      "mov   lr, r5             \n"  /* thread_return_trap                     */
      "pop   {r4, r5}           \n"  /* PC, xPSR                               */
      "movs  r5, #1             \n"
      "orrs  r4, r5             \n"  /* BX needs the Thumb bit the frame lacks */
      "cpsie i                  \n"  /* the thread begins at baseline          */
      "bx    r4                 \n"
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
   CYROS_ASSERT(!cortex_m::in_handler_mode());
   CYROS_ASSERT_OP(cortex_m::get_primask(), ==, 0u);
   CYROS_ASSERT_OP(preempt_depth[this_core()], ==, 0u);

   cortex_m::reg(cortex_m::scb_icsr) = cortex_m::icsr_pendsvset;
   cortex_m::dsb();
   cortex_m::isb();
   /* Taken at the ISB: a full reschedule has run by the time this returns. */
}

void cyros_port_pend_reschedule(void)
{
   /* From anywhere. Inside a preempt-disabled region PendSV runs at once and
    * defers, and the region's enable delivers it. */
   cortex_m::reg(cortex_m::scb_icsr) = cortex_m::icsr_pendsvset;
   cortex_m::dsb();
   cortex_m::isb();
}


/* ============================================================================
 * Thread-Local Storage
 * ========================================================================= */

void cyros_port_set_tls_pointer(void* tls_base)
{
   current_tls[this_core()] = tls_base;
}

void* cyros_port_get_tls_pointer(void)
{
   return current_tls[this_core()];
}


/* ============================================================================
 * CPU Hints & Idle
 * ========================================================================= */

void cyros_port_cpu_relax(void)
{
   asm volatile("yield" ::: "memory");
}

void cyros_port_idle(void)
{
   /* WFI wakes on any enabled interrupt even while PRIMASK masks it. */
   cortex_m::wfi();
}


/* ============================================================================
 * Measurement
 * ========================================================================= */

uint64_t cyros_port_timestamp(void)
{
   /* No cycle counter on ARMv6-M: the target's counter is the only one. */
   return cortex_m::timestamp();
}


/* ============================================================================
 * Debug & Diagnostics
 * ========================================================================= */

void cyros_port_system_error(std::uintptr_t auxilary1,
                             std::uintptr_t auxilary2,
                             char const* file_optional,
                             int line_optional)
{
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
   static volatile int resume = 0;
   cortex_m::write0("cyros: waiting for debugger, set 'resume' to 1\n");
   while (resume == 0) { cortex_m::nop(); }
}

void cyros_port_breakpoint(void)
{
   asm volatile("bkpt 0x00");
}

void* cyros_port_get_stack_pointer(void)
{
   void* sp = nullptr;
   asm volatile("mov %0, sp" : "=r"(sp));
   return sp;
}
