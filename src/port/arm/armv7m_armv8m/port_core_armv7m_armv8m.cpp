/**
 * @file port_core_armv7m_armv8m.cpp
 * @brief ARMv7-M and ARMv8-M Mainline CORE LAYER: all of port_core.h.
 *
 * Benches are QEMU's mps2-an505 (Cortex-M33), mps2-an386 (Cortex-M4F) and
 * mps2-an385 (Cortex-M3, no FPU). Hardware is an STM32U575 (M33) and a TM4C123
 * (M4F). Everything this file touches is architectural, so each bench and its
 * board are the same target as far as the port is concerned, and the two
 * architectures differ here in one mechanism only: the stack guard (see below).
 * Whether there is an FPU is a separate question, answered by the build.
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
 * inert there.
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
 * is an implementation choice: 8 on both QEMU benches, 4 on the STM32U575, 3 on
 * the TM4C123. A hardcoded "SysTick = 0xE0, PendSV = 0xFF" collapses to the
 * SAME level when N is 3, which would let SysTick be masked by a
 * preempt-disable and silently stop the clock inside every critical section.
 * init() writes 0xFF to a priority register and reads back which bits stuck,
 * then derives the two values from that.
 *
 *
 * The stack guard: PSPLIM on ARMv8-M, an MPU region on ARMv7-M, or nothing
 * ======================================================================
 * cyros hands every thread a caller-owned buffer, so an overrun walks into
 * whatever the application put below it. The policy is a hardware guard where
 * the part has one and it is cheap, and no software stand-in where it does not.
 *
 *   ARMv8-M  PSPLIM, reloaded on every switch. The SP update itself faults
 *            (UsageFault, STKOF), before anything is written.
 *   ARMv7-M  the HIGHEST-numbered MPU region, so no application region can
 *            override it, made a 128-byte no-access window at the bottom of
 *            the running thread's buffer and moved by one RBAR write per
 *            switch (9 cycles a switch on a TM4C123, measured on one binary
 *            with the guard on and off). The store into it faults
 *            (MemManage, DACCVIOL), and an exception that tries to stack into
 *            it faults too (MSTKERR). A frame larger than the window can step
 *            over it in one `sub sp`, which PSPLIM would catch and this cannot.
 *   ARMv7-M, no MPU   no guard. Decided, not defaulted: guard_region stays 0.
 *
 * 128 bytes rather than the architectural minimum of 32: an exception taken
 * with FP state live RESERVES a 104-byte frame but, with lazy stacking, writes
 * only its bottom 32 bytes, so over a thinner guard the write can land wholly
 * below it and nothing faults (Zephyr met this, issue 14828). Without an FPU
 * every frame is 32 bytes and that reason goes, but a frame larger than the
 * window steps over it more easily the thinner it is, so both builds keep 128.
 *
 *
 * Floating point: saved exactly when the image can have it
 * ========================================================
 * The FPU enable and PendSV's save of s16-s31 exist only when __ARM_FP is
 * defined, which is when this translation unit may use FP instructions: hard
 * float and softfp, not soft. (__VFP_FP__ would be wrong, GCC defines it even
 * for a Cortex-M3. __ARM_PCS_VFP would be wrong too, it drops softfp, whose
 * code does use the registers.)
 *
 * Without __ARM_FP the port never enables the FPU, so no FP instruction can
 * execute, CONTROL.FPCA stays clear and every EXC_RETURN has FType set. The
 * save that is compiled out would never have run. On a part with no FPU the
 * architecture fixes FType at 1 regardless.
 *
 * NOT SUPPORTED: a soft-float build of cyros linked with softfp objects (the
 * calling convention matches, so the linker allows it) in an application that
 * turns the FPU on itself. Threads then have FP state this port does not save,
 * and s16-s31 leak between them. Hard-float objects cannot get in, the linker
 * refuses to mix the two ABIs. Not checked at run time, because the check would
 * sit on every switch for a configuration that needs a deliberate FPU enable.
 */

#include <cyros/port/port_core.h>

/* For cyros_port_get_core_id, and for that one function only.
 *
 * A core layer may CALL the MCU contract, exactly as an MCU layer may call the
 * core contract. The property the layering protects is that `armv7m_armv8m`
 * links UNCHANGED against any MCU layer, and depending on a declaration
 * preserves that completely. What a layer may never do is IMPLEMENT the other
 * half, and that is mechanically checked rather than left to this comment.
 *
 * The dependency exists because neither architecture has a core identity, so
 * per-core state in a core-level file cannot be indexed without asking the MCU.
 * On a single-core target the call folds away entirely: see this_core().
 */
#include <cyros/port/port_mcu.h>

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
   std::uint32_t stack_limit;  /* the guard's bound (see the header) */
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

/* A CACHE of the running thread's TLS base, one slot per core.
 *
 * TLS IS PER THREAD, not per core, and nothing here changes that. The
 * authoritative copy is `tls` in each thread's own context, and
 * cyros_port_switch saves the outgoing thread's and loads the incoming one's.
 * This array only answers "on this core, which thread is running and where is
 * its TLS base", so it is indexed by core for the same reason PSP is per core:
 * there is one running thread per core, not one in the system.
 *
 * With a single slot, core 1's switch would overwrite core 0's notion of the
 * running thread. The next save on core 0 would then write core 1's thread's
 * base into core 0's thread's context, so the two threads would end up sharing
 * a TLS base. Per core is what keeps the thread-to-TLS mapping one to one.
 *
 * No two cores ever write the same word, so no atomics are needed. There is no
 * false sharing to pad against either: an SSE-200 M33 has no data cache.
 */
void* current_tls[CYROS_PORT_CORE_COUNT] = {};

/**
 * @brief Which core is executing, for indexing per-core port state.
 *
 * On a single-core target this is the constant 0, so the MMIO read disappears
 * entirely and the arrays above collapse to one slot addressed by a literal.
 * That is what makes calling into the MCU contract from here acceptable: the
 * per-access cost lands only on a build that genuinely has more than one core.
 *
 * Measured on the bench build: supporting multicore at all costs a SINGLE-core
 * image 32 bytes of text, and all of it is init_this_core and
 * device_irq_priority becoming externally callable rather than folding away.
 * None of it is on the switch path. Compiling this same file for two cores
 * costs a further 44 bytes of text and one pointer of bss, which is the MMIO
 * read and the array indexing, and that lands only on the SMP target.
 */
inline std::uint32_t this_core() noexcept
{
   if constexpr (CYROS_PORT_CORE_COUNT == 1) {
      return 0u;
   } else {
      return cyros_port_get_core_id();
   }
}

/* Derived in cyros_port_init once the implemented priority width is known. */
std::uint32_t pendsv_priority  = 0xFFu;
std::uint32_t systick_priority = 0xFFu;

/* The AIRCR.PRIGROUP the two priorities above were derived against. AIRCR is
 * banked per core, and the derivation runs on the bootstrap core only, so every
 * other core checks its own agrees before it applies them. */
std::uint32_t derived_prigroup = 0u;

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

#if !CYROS_CORTEX_M_HAS_STACK_LIMIT
/* The MPU region the ARMv7-M guard uses, plus one, so that zero means no guard:
 * derived in cyros_port_init from MPU_TYPE, written once by the bootstrap core. */
std::uint32_t guard_region_plus_one = 0u;

constexpr std::uint32_t guard_bytes = 128u;

/* No access, execute-never, SIZE such that 2^(SIZE+1) == guard_bytes, enabled. */
constexpr std::uint32_t guard_rasr =
   cortex_m::mpu_rasr_xn | (0u << 24) | ((7u - 1u) << 1) | cortex_m::mpu_rasr_enable;
static_assert((1u << (((guard_rasr >> 1) & 0x1Fu) + 1u)) == guard_bytes);
#endif

/* Before moving PSP. On ARMv8-M an MSR to PSP is checked against the OUTGOING
 * thread's PSPLIM, so setting the new stack first would fault whenever the
 * incoming stack sits below the outgoing one. Nothing to do on ARMv7-M, where
 * the MPU polices addresses, not the stack pointer. */
inline void release_stack_guard() noexcept
{
#if CYROS_CORTEX_M_HAS_STACK_LIMIT
   cortex_m::set_psplim(0u);
#endif
}

/* After moving PSP: guard the stack that is about to run. */
inline void load_stack_guard(std::uint32_t limit) noexcept
{
#if CYROS_CORTEX_M_HAS_STACK_LIMIT
   cortex_m::set_psplim(limit);
#else
   if (guard_region_plus_one != 0u) {
      /* VALID makes the same write select the region, so RNR is left pointing
       * at the guard, which cyros_port_start_first relies on. The exception
       * return that follows a switch is context-synchronising, the DSB makes
       * sure the write has landed before it. */
      cortex_m::reg(cortex_m::mpu_rbar) =
         limit | cortex_m::mpu_rbar_valid | (guard_region_plus_one - 1u);
      cortex_m::dsb();
   }
#endif
}

/**
 * @brief Catch a thread entry point that returns.
 *
 * port.h says a thread entry MUST NOT return and leaves enforcement to the
 * port. Sitting in the initial frame's LR slot costs nothing and turns an
 * otherwise unbounded jump into a named panic.
 */
/**
 * @brief EXC_RETURN for a fresh thread: Thread mode, PSP, standard frame.
 *
 * Bit 4 (FType) set means NO floating-point state in the exception frame,
 * bit 3 (Mode) set means Thread rather than Handler, bit 2 (SPSEL) set means
 * PSP rather than MSP.
 */
constexpr std::uint32_t exc_return_thread_psp_no_fp = 0xFFFFFFFDu;

[[noreturn]] void thread_return_trap()
{
   cyros_port_system_error(0, 0, "thread entry returned", 0);
}

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
   current_tls[this_core()] = nullptr;

   /* Mask everything for the whole of bring-up. Nothing may preempt the kernel
    * between here and cyros_port_start_first, which is the moment the first
    * thread stack exists and PSP becomes meaningful. A SysTick landing in that
    * window would stack an exception frame onto MSP and then try to switch away
    * from a thread that does not exist. */
   cortex_m::disable_irq();
   cortex_m::set_basepri(0u);

#if defined(__ARM_FP)
   /* Turn the FPU on before anything can execute an FP instruction. The kernel
    * itself uses none, but this build may contain them, so a user thread may
    * touch one at any time and a disabled FPU turns that into a NOCP
    * UsageFault a long way from the cause. Without __ARM_FP it stays off, which
    * is what lets PendSV skip the FP save (see the header). */
   cortex_m::reg(cortex_m::scb_cpacr) =
      cortex_m::reg(cortex_m::scb_cpacr) | cortex_m::cpacr_fpu_full_access;
   cortex_m::dsb();
   cortex_m::isb();
#endif

   /* The spacing between PendSV and SysTick is a whole preemption GROUP only
    * under the PRIGROUP it was derived for (see cyros_port_init). A core whose
    * own AIRCR disagreed could put its SysTick in PendSV's group, and every
    * critical section there would stop its clock: the 6.0 bug, on one core. */
   CYROS_ASSERT_OP((cortex_m::reg(cortex_m::scb_aircr) >> cortex_m::aircr_prigroup_shift)
                      & cortex_m::aircr_prigroup_mask, ==, derived_prigroup);

   cortex_m::reg8(cortex_m::shpr_pendsv)  = static_cast<std::uint8_t>(pendsv_priority);
   cortex_m::reg8(cortex_m::shpr_systick) = static_cast<std::uint8_t>(systick_priority);

#if !CYROS_CORTEX_M_HAS_STACK_LIMIT
   if (guard_region_plus_one != 0u) {
      /* The region stays disabled until cyros_port_start_first has a thread to
       * guard. ENABLE and PRIVDEFENA are ORed in, not written, so an MPU the
       * application already set up keeps its configuration. PRIVDEFENA keeps
       * the default map for privileged code, which is all of cyros, so turning
       * the MPU on changes nothing but the guard. */
      cortex_m::reg(cortex_m::mpu_rbar) = cortex_m::mpu_rbar_valid | (guard_region_plus_one - 1u);
      cortex_m::reg(cortex_m::mpu_rasr) = 0u;
      cortex_m::reg(cortex_m::mpu_ctrl) =
         cortex_m::reg(cortex_m::mpu_ctrl) | cortex_m::mpu_ctrl_privdefena | cortex_m::mpu_ctrl_enable;
      cortex_m::reg(cortex_m::scb_shcsr) = cortex_m::reg(cortex_m::scb_shcsr) | cortex_m::shcsr_memfaultena;
   }
#endif

   /* Clear any reschedule left pending by a previous kernel lifecycle. The unit
    * tests initialise and finalise repeatedly, and a stale PENDSVSET would fire
    * into the next lifecycle's first thread. */
   cortex_m::reg(cortex_m::scb_icsr) = cortex_m::icsr_pendstclr;

   cortex_m::dsb();
   cortex_m::isb();
}

} // namespace cyros::port::cortex_m

void cyros_port_init(cyros_port_reschedule_t handler)
{
   CYROS_ASSERT(handler != nullptr);
   reschedule_handler = handler;

   std::uint32_t const bits = discover_priority_bits();
   CYROS_ASSERT_OP(bits, >=, 2u);   /* need at least two distinguishable levels */
   CYROS_ASSERT_OP(bits, <=, 8u);

   /* Granularity comes from TWO independent places and the coarser one wins.
    *
    * The first is how many priority bits the part implements, which is what
    * discover_priority_bits found.
    *
    * The second is AIRCR.PRIGROUP, and ignoring it is a silent bug rather than
    * a missing feature. PRIGROUP splits every priority field into a GROUP part
    * and a SUB-priority part, and preemption and BASEPRI masking consider ONLY
    * the group part. At the reset default of 0 the bottom bit is sub-priority,
    * so on a part implementing all 8 bits, PendSV at 0xFF and SysTick at 0xFE
    * land in the SAME group. Raising BASEPRI to PendSV's level then masks
    * SysTick too, and every kernel critical section stops the clock.
    *
    * No check that looks at the priority NUMBERS can see this, since the two
    * values do differ. Only an interrupt actually firing shows it, which is
    * what test_cortex_m33_systick is for.
    *
    * PRIGROUP is read rather than written, because an application may have set
    * it for its own device IRQs. The cost is that changing PRIGROUP after
    * cyros_port_init is not supported. */
   std::uint32_t const implemented_step = 1u << (8u - bits);

   std::uint32_t const prigroup =
      (cortex_m::reg(cortex_m::scb_aircr) >> cortex_m::aircr_prigroup_shift) & cortex_m::aircr_prigroup_mask;
   std::uint32_t const group_step = 1u << (prigroup + 1u);
   derived_prigroup = prigroup;

   std::uint32_t const step = implemented_step > group_step ? implemented_step : group_step;

   /* PendSV lowest, SysTick a full preemption group above it. Any device IRQ
    * must be configured at systick_priority or above (numerically lower) to
    * remain serviceable while preemption is disabled. */
   pendsv_priority  = 0xFFu & ~(implemented_step - 1u);
   systick_priority = pendsv_priority - step;
   preempt_mask_value = pendsv_priority;

#if !CYROS_CORTEX_M_HAS_STACK_LIMIT
   /* The highest-numbered region, because on overlap the higher number wins
    * and an application region must not be able to switch the guard off. No
    * MPU (DREGION 0) means no guard, by policy (see the header). */
   guard_region_plus_one = (cortex_m::reg(cortex_m::mpu_type) >> 8) & 0xFFu;
#endif

   /* Derived above, APPLIED here, and the split is what makes a second core
    * cheap. The three values are facts about the core design, so every core in
    * a homogeneous part derives them identically. Deriving once on the
    * bootstrap core and having each secondary core only apply them keeps the
    * globals written by exactly one core, so there is no race to reason about
    * even though the values would have agreed anyway. */
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
    * is 8-byte granular, so both ends round toward the middle. The ARMv7-M
    * guard is an MPU region, whose base must be aligned to its size, so there
    * the bottom rounds up to that instead and the usable stack starts above
    * the guard. That costs up to 248 bytes of the buffer. */
#if CYROS_CORTEX_M_HAS_STACK_LIMIT
   constexpr std::uintptr_t guard_align = 8u;
   constexpr std::uintptr_t guard_size  = 0u;
#else
   /* The probe in cyros_port_init decides whether there is a guard to align. */
   CYROS_ASSERT(reschedule_handler != nullptr);
   std::uintptr_t const guard_align = guard_region_plus_one != 0u ? guard_bytes : 8u;
   std::uintptr_t const guard_size  = guard_region_plus_one != 0u ? guard_bytes : 0u;
#endif
   std::uintptr_t const limit = (base + guard_align - 1u) & ~(guard_align - 1u);
   std::uintptr_t const top   = (base + stack_size) & ~static_cast<std::uintptr_t>(7u);
   CYROS_ASSERT_OP(top, >, limit + guard_size);

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

   /* EXC_RETURN, which the PendSV epilogue loads into lr and returns through.
    * Thread mode, PSP, and NO FP state: a thread that has never run cannot
    * have touched the FPU, so its first exception return uses the standard
    * frame. The hardware sets FType to 0 on its own the first time the thread
    * does use FP. */
   *--sp = exc_return_thread_psp_no_fp;

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

   /* Read once and reuse. On a multicore target this is an MMIO read, and the
    * switch is the hottest path the port has. */
   std::uint32_t const core = this_core();

   if (from != nullptr) {
      from->sp  = cortex_m::get_psp();
      from->tls = current_tls[core];
   }

   release_stack_guard();
   cortex_m::set_psp(to->sp);
   load_stack_guard(to->stack_limit);

   current_tls[core] = to->tls;
}

/**
 * @brief The PendSV handler. Every context switch in the system passes here.
 *
 * Naked because the prologue and epilogue ARE the mechanism: a compiler-emitted
 * frame would save the wrong registers to the wrong stack. The handler runs on
 * MSP while the thread it interrupted is stacked on PSP.
 *
 * EXC_RETURN IS SAVED IN THE THREAD'S OWN FRAME, not on MSP. With the FPU
 * enabled it is per-thread state: bit 4 (FType) says whether THAT thread has an
 * extended exception frame carrying FP state, so threads do not share a value
 * and it belongs with the rest of the thread's context. Pushing it to MSP
 * instead only works while every thread's EXC_RETURN happens to be identical.
 */
extern "C" [[gnu::naked]] void PendSV_Handler(void)
{
   asm volatile(
      "mrs   r0, psp                    \n"  /* the interrupted thread's stack  */
#if defined(__ARM_FP)
      /* FType (bit 4) is CLEAR when this thread has an extended exception
       * frame, i.e. when it has used the FPU. s0-s15 and FPSCR are the
       * hardware's business; s16-s31 are callee-saved and ours. Executing this
       * vstmdb is also what flushes any pending LAZY save out to FPCAR, which
       * still points at this thread's frame, before anything switches. */
      "tst   lr, #0x10                  \n"
      "it    eq                         \n"
      "vstmdbeq r0!, {s16-s31}          \n"
#endif
      "stmdb r0!, {r4-r11, lr}          \n"  /* callee-saved, plus EXC_RETURN   */
      "msr   psp, r0                    \n"
      "push  {r3, lr}                   \n"  /* keeps MSP 8-aligned across the bl */
      "bl    cyros_port_pendsv_dispatch \n"  /* may call cyros_port_switch      */
      "pop   {r3, lr}                   \n"
      "mrs   r0, psp                    \n"  /* possibly a DIFFERENT stack now  */
      "ldmia r0!, {r4-r11, lr}          \n"  /* including THAT thread's EXC_RETURN */
#if defined(__ARM_FP)
      /* Mirrors the prologue, and keys off the INCOMING thread's EXC_RETURN,
       * which the ldmia above has just restored. */
      "tst   lr, #0x10                  \n"
      "it    eq                         \n"
      "vldmiaeq r0!, {s16-s31}          \n"
#endif
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

   current_tls[this_core()] = first->tls;

   release_stack_guard();
   cortex_m::set_psp(first->sp);
   load_stack_guard(first->stack_limit);
#if !CYROS_CORTEX_M_HAS_STACK_LIMIT
   if (guard_region_plus_one != 0u) {
      /* The region was configured disabled in init_this_core. load_stack_guard
       * has just selected it, so this write is the guard's. */
      cortex_m::reg(cortex_m::mpu_rasr) = guard_rasr;
      cortex_m::dsb();
      cortex_m::isb();
   }
#endif

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
      /* Step over the saved EXC_RETURN. This path does not use it: it is
       * entering Thread mode by hand rather than by exception return, which is
       * the whole reason this function is not just a `bx lr`. */
      "add   sp, sp, #4         \n"
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
   /* YIELD is a hint with no effect on a single-threaded core, but it is the
    * architecturally correct marker for a spin body and costs one cycle. */
   asm volatile("yield" ::: "memory");
}

void cyros_port_idle(void)
{
   /* WFI wakes on any enabled interrupt even while PRIMASK masks it, so this
    * is safe inside the kernel's idle loop regardless of masking state.
    *
    * The NOP is a TM4C123 erratum (TI SPMZ849, SYSCTL#04): waking from WFI at
    * 40 MHz or more with interrupts disabled can fetch bad instructions from the
    * prefetch buffer, and TI's workaround is one instruction between the WFI
    * and the return. The kernel idles with interrupts enabled today, so it is
    * not exposed, but this keeps that from being the only thing standing
    * between an idle loop that masks and a wild fetch. One cycle, everywhere. */
   cortex_m::wfi();
   cortex_m::nop();
}


/* ============================================================================
 * Measurement
 * ========================================================================= */

namespace
{

/* The cycle counter is 32 bits and the contract is 64, so each core extends
 * its own count by noticing when a read comes back smaller than the last one.
 * That misses a wrap only if two reads on one core are a whole period apart:
 * 2^32 cycles, about 27 seconds at 160 MHz and 18 minutes at 4 MHz. A trace
 * with a gap that long shows time jumping backwards by a period, which is
 * visible rather than silent. */
struct cycle_extension
{
   std::uint32_t last;
   std::uint32_t wraps;
};

cycle_extension cycle_extensions[CYROS_PORT_CORE_COUNT] = {};

} // namespace

uint64_t cyros_port_timestamp(void)
{
   /* Masked so that reading the counter and extending it are one step: an
    * interrupt taking its own stamp in between would see the same wrap and
    * count it twice. */
   auto const token = cyros_port_irq_save();

   /* Started on first use, per core, because the DWT is per core and nothing
    * else in cyros needs it. A debugger that resets the DWT is caught here too. */
   if ((cortex_m::reg(cortex_m::dwt_ctrl) & cortex_m::dwt_ctrl_cyccntena) == 0u) {
      cortex_m::reg(cortex_m::dcb_demcr) |= cortex_m::demcr_trcena;
      cortex_m::reg(cortex_m::dwt_ctrl) |= cortex_m::dwt_ctrl_cyccntena;
   }

   auto& extension = cycle_extensions[this_core()];
   std::uint32_t const now = cortex_m::reg(cortex_m::dwt_cyccnt);
   if (now < extension.last) {
      ++extension.wraps;
   }
   extension.last = now;
   std::uint64_t const stamp = (std::uint64_t{extension.wraps} << 32) | now;

   cyros_port_irq_restore(token);
   return stamp;
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
