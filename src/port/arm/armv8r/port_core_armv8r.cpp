/**
 * @file port_core_armv8r.cpp
 * @brief ARMv8-R AArch32 CORE LAYER (the Cortex-R52): all of port_core.h.
 *
 * Bench is QEMU's mps3-an536 (two R52s). No board yet. The R-profile keeps the
 * A-profile's exception model, banked modes and a GIC, so this layer shares
 * nothing with the M-profile ones but cyros's contract. Its shape is the
 * RISC-V layer's, for the same reason: the hardware saves almost nothing on
 * exception entry. `~/cyros-claude/r52-port-notes.md` has the measurements.
 *
 *
 * The central decision: ALL SWITCHING HAPPENS IN ONE EXCEPTION PATH
 * =================================================================
 * A reschedule is SGI 0, the GIC's software-generated interrupt, at the
 * LOWEST priority this layer uses. It is taken only when CPSR.I is clear and
 * the GIC's priority mask (ICC_PMR) lets it through, and every handler runs
 * with CPSR.I set, so a reschedule pended inside an interrupt is taken when
 * that interrupt returns, never during it. That is PendSV's property, from
 * the GIC.
 *
 * `cyros_port_thread_yield` is an SVC, which enters the same path
 * synchronously. An SGI is a write to the GIC and arrives a little later, so a
 * yield built on it could return before the switch.
 *
 *
 * Modes and stacks
 * ================
 * Threads run in System mode, privileged, on the System-mode SP. Handlers run
 * in Supervisor mode, whose banked SP is the core's interrupt stack. So the
 * hardware's banked stack pointers ARE the thread/interrupt split, which MSP
 * and PSP are on Cortex-M.
 *
 * An IRQ or an SVC stores the interrupted thread's frame on the THREAD's
 * stack: SRSDB puts the return address and the CPSR there, then System mode
 * pushes r0 to r12 and lr (armv8r.hpp has the layout). A suspended thread is a
 * pointer to its frame, and cyros_port_switch is one store, as on RISC-V. The
 * entry then moves to Supervisor mode and calls C on the interrupt stack, so a
 * thread stack needs room for one frame (64 bytes, 328 with the FP state
 * below) and never for a handler.
 *
 * The interrupt stack is the stack the core was standing on when it started
 * its first thread, as on RISC-V. Before that, Supervisor mode's SP is the
 * board's, so a reschedule pended during bring-up (the port test's) is a real
 * exception through the real entry and exit.
 *
 * Nothing here uses the IRQ mode's own SP, FIQ or Group 0. A fault (Undefined,
 * Prefetch or Data Abort) is the board's: it owns the vector table, as an
 * M-profile board owns its fault vectors.
 *
 *
 * Masking: CPSR.I and the GIC's priority mask
 * ===========================================
 *
 *   Interrupt masking  (cyros_port_irq_save/restore)       -> CPSR.I
 *   Preemption control (cyros_port_preempt_disable/enable) -> ICC_PMR
 *
 * The reschedule SGI sits one priority step above the lowest the GIC keeps,
 * and a preempt-disable lowers PMR to exactly that priority, so it masks the
 * reschedule and nothing above it: the timer and every device interrupt still
 * run. That is BASEPRI on ARMv7-M, with the GIC's mask in its place. The
 * priorities are derived in cyros_port_init from what PMR actually keeps (5
 * bits on QEMU's R52), as the M-profile layer derives its own.
 *
 * Each TOKEN carries the previous state: the I bit, or the previous PMR. No
 * thread is switched out with PMR lowered or I set, since neither the SGI nor
 * a yield can then be taken, so every thread resumes with both open and a
 * token read before a switch is still true after it.
 *
 * Baseline priority is CPSR.I clear and PMR open, outside a handler.
 *
 *
 * Floating point: saved exactly when the image can have it
 * ========================================================
 * In a build with FP instructions (__ARM_FP) every exception saves the whole
 * FP state with the frame: FPSCR, d0 to d15, and d16 to d31 on a core that has
 * them, which MVFR0 says (QEMU's R52 does, with Advanced SIMD). No macro tells
 * the compiler's register count apart, so the entry asks the hardware, and the
 * frame keeps d16 to d31's slot either way so its layout is fixed. Eager and
 * whole, not lazy: a handler's compiled code may use any FP register, and a
 * switch must also carry the callee-saved d8 to d15 to the incoming thread.
 * Without __ARM_FP none of it is compiled and the FPU is never enabled. The
 * BOARD enables the FPU before any compiled code, since GCC moves 64-bit values
 * through d registers, and each core's init enables it again.
 *
 *
 * TLS is a thread ID register
 * ===========================
 * TPIDRPRW, the privileged software thread ID register, holds the running
 * thread's TLS base, and cyros_port_switch saves and loads it with the frame.
 *
 *
 * The stack guard: EL1 MPU region 15, moved on every switch
 * =========================================================
 * cyros hands every thread a caller-owned buffer, so an overrun walks into
 * whatever the application put below it. The policy is a hardware guard where
 * one is cheap and none otherwise. Here it is EL1 MPU region 15 (the R52 has
 * 16 by default), 128 bytes at the bottom of the RUNNING thread's buffer,
 * rounded up to the MPU's 64-byte granule, moved by three register writes per
 * switch. The MPU is on with its background region, so every EL1 access no
 * region matches keeps the default memory map, as PRIVDEFENA does on ARMv7-M.
 *
 * READ-ONLY, because PMSAv8-32 has no permission that denies EL1, where cyros
 * runs. An overrun is a write, so it faults (Data Abort, a permission fault)
 * before it reaches whatever lies below the buffer. A region overlapping one
 * the application defined is a fault on every access, which only strengthens
 * the guard.
 *
 * No spill zone, unlike RISC-V: a fault in the exception entry's own frame
 * save is a Data Abort like any other, taken in Abort mode on that mode's
 * stack, so it never stacks another frame lower.
 *
 * What it cannot catch, as on ARMv7-M: a frame larger than the guard whose
 * stores all miss it, the sp stepping over it in one adjustment.
 *
 * CYROS_ARMV8R_STACK_GUARD set to 0 builds it out: the MPU is left alone and a
 * switch writes nothing. It is local to this file, a switch for measuring or
 * ruling the guard out.
 */

#include <cyros/port/port_core.h>

/* For cyros_port_get_core_id, as in the other layers: a core layer may call
 * the MCU contract, it may not implement it. On a single-core target the call
 * folds away (this_core below). */
#include <cyros/port/port_mcu.h>

#include "armv8r.hpp"

#include <cstddef>
#include <cstdint>

#ifndef CYROS_ARMV8R_STACK_GUARD
#define CYROS_ARMV8R_STACK_GUARD 1
#endif

namespace armv8r = cyros::port::armv8r;

/* Whether this core has d16 to d31, which the exception path reads to decide
 * whether to save them. Set by the bootstrap core in cyros_port_init. */
extern "C" { std::uint32_t cyros_armv8r_fp_d32 = 0u; }

/* The FP half of the exception entry and exit (the file comment), as string
 * fragments of the two naked functions below. r0 and r1 are free at both
 * points: the entry has saved them, the exit restores them after. */
#if defined(__ARM_FP)
#define CYROS_ARMV8R_FP_SAVE                                   \
   ".fpu   neon-fp-armv8                 \n"                  \
   "vmrs   r0, fpscr                     \n"                  \
   "push   {r0, r1}                      \n"                  \
   "vpush  {d0-d15}                      \n"                  \
   "movw   r1, #:lower16:cyros_armv8r_fp_d32 \n"              \
   "movt   r1, #:upper16:cyros_armv8r_fp_d32 \n"              \
   "ldr    r1, [r1]                      \n"                  \
   "cbz    r1, 1f                        \n"                  \
   "vpush  {d16-d31}                     \n"                  \
   "b      2f                            \n"                  \
   "1:                                   \n"                  \
   "sub    sp, sp, #128                  \n"                  \
   "2:                                   \n"
#define CYROS_ARMV8R_FP_RESTORE                                \
   ".fpu   neon-fp-armv8                 \n"                  \
   "movw   r1, #:lower16:cyros_armv8r_fp_d32 \n"              \
   "movt   r1, #:upper16:cyros_armv8r_fp_d32 \n"              \
   "ldr    r1, [r1]                      \n"                  \
   "cbz    r1, 3f                        \n"                  \
   "vpop   {d16-d31}                     \n"                  \
   "b      4f                            \n"                  \
   "3:                                   \n"                  \
   "add    sp, sp, #128                  \n"                  \
   "4:                                   \n"                  \
   "vpop   {d0-d15}                      \n"                  \
   "pop    {r0, r1}                      \n"                  \
   "vmsr   fpscr, r0                     \n"
#else
#define CYROS_ARMV8R_FP_SAVE    ""
#define CYROS_ARMV8R_FP_RESTORE ""
#endif

/* ============================================================================
 * Context
 * ========================================================================= */

/**
 * @brief A suspended thread's port state: where its frame is, its TLS, and the
 *        bottom of its stack guard (0 for none).
 */
struct cyros_port_context
{
   std::uint32_t* frame;
   void*          tls;
   std::uintptr_t guard;
};

static_assert(sizeof(cyros_port_context) <= CYROS_PORT_CONTEXT_SIZE,
              "CYROS_PORT_CONTEXT_SIZE in port_traits.h is too small for this port");
static_assert(alignof(cyros_port_context) <= CYROS_PORT_CONTEXT_ALIGN,
              "CYROS_PORT_CONTEXT_ALIGN in port_traits.h is too weak for this port");

namespace
{

cyros_port_reschedule_t reschedule_handler = nullptr;

/* The stack guard's geometry and region (the file comment). */
constexpr std::uintptr_t guard_bytes  = 128u;
constexpr std::uintptr_t guard_granule = 64u;
constexpr std::uint32_t  guard_region = 15u;

constexpr bool guard_built = CYROS_ARMV8R_STACK_GUARD != 0;

/* Whether this R52 has region 15. Written by the bootstrap core in
 * cyros_port_init, read everywhere after. */
bool guard_available = false;

/* Whether threads are guarded: built in, and the MPU able. Built out, every
 * use folds away. */
inline bool guarded() noexcept
{
   return guard_built && guard_available;
}

std::uint32_t mpu_regions() noexcept
{
   return (armv8r::read_mpuir() >> 8) & 0xFFu;   /* DREGION */
}

/* Region 15 over [bottom, bottom + guard_bytes): disabled first, so it never
 * spans the old and new places at once, which could cover the stack the
 * handler is running on. The exception return that follows is context
 * synchronising. */
inline void load_guard(std::uintptr_t bottom) noexcept
{
   armv8r::write_prlar15(0u);
   armv8r::write_prbar15(static_cast<std::uint32_t>(bottom) | armv8r::prbar_ap_ro_el1 | armv8r::prbar_xn);
   armv8r::write_prlar15(static_cast<std::uint32_t>(bottom + guard_bytes - guard_granule) | armv8r::prlar_en);
}

/* The GIC priorities, derived by the bootstrap core in cyros_port_init and
 * read everywhere after (the file comment). */
std::uint32_t pmr_open            = 0xFFu;   /* the lowest priority PMR keeps     */
std::uint32_t reschedule_priority = 0xFFu;   /* one step above it, SGI 0          */
std::uint32_t timer_priority      = 0x80u;

/* The R52's integrated GIC (armv8r.hpp). */
constexpr std::uintptr_t redistributor_offset = 0x100000u;
constexpr std::uintptr_t redistributor_stride = 0x20000u;   /* RD_base and SGI_base */
constexpr std::uintptr_t sgi_frame            = 0x10000u;

constexpr std::uintptr_t gicd_ctlr        = 0x0000u;
constexpr std::uint32_t  gicd_ctlr_grp1   = 1u << 1;
constexpr std::uint32_t  gicd_ctlr_are    = 1u << 4;
constexpr std::uint32_t  gicd_ctlr_rwp    = 1u << 31;

constexpr std::uintptr_t gicr_typer       = 0x0008u;   /* 64 bits, affinity in the top word */
constexpr std::uintptr_t gicr_waker       = 0x0014u;
constexpr std::uint32_t  gicr_waker_sleep = 1u << 1;   /* ProcessorSleep */
constexpr std::uint32_t  gicr_waker_asleep = 1u << 2;  /* ChildrenAsleep */

constexpr std::uintptr_t gicr_igroupr0    = sgi_frame + 0x0080u;
constexpr std::uintptr_t gicr_isenabler0  = sgi_frame + 0x0100u;
constexpr std::uintptr_t gicr_icpendr0    = sgi_frame + 0x0280u;
constexpr std::uintptr_t gicr_ipriorityr  = sgi_frame + 0x0400u;

inline volatile std::uint32_t& reg(std::uintptr_t address) noexcept
{
   return *reinterpret_cast<volatile std::uint32_t*>(address);
}

inline volatile std::uint8_t& reg8(std::uintptr_t address) noexcept
{
   return *reinterpret_cast<volatile std::uint8_t*>(address);
}

/* PERIPHBASE, from IMP_CBAR, whose low bits are not the address. */
inline std::uintptr_t distributor() noexcept
{
   return armv8r::read_cbar() & 0xFFE00000u;
}

inline std::uintptr_t redistributor(std::uint32_t core) noexcept
{
   return distributor() + redistributor_offset + core * redistributor_stride;
}

/**
 * @brief What each core's exception path keeps.
 *
 * `frame` is the RUNNING thread's frame while an exception is in progress,
 * which cyros_port_switch replaces with the incoming thread's, and which the
 * exit then restores. No two cores touch each other's entry.
 */
struct core_state
{
   std::uint32_t* frame;
   std::uint32_t  depth;
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

/* The reschedule, from inside the exception path. The kernel's handler may
 * call cyros_port_switch, which replaces `frame`. */
void run_reschedule(core_state& state)
{
   CYROS_ASSERT_OP(state.depth, ==, 1u);   /* never from inside a handler */
   CYROS_ASSERT(reschedule_handler != nullptr);
   reschedule_handler();
}

/* The two ways into the exception path, told apart by the entries. */
constexpr std::uint32_t entered_by_irq = 0u;
constexpr std::uint32_t entered_by_svc = 1u;

} // namespace


/* ============================================================================
 * The exception path
 * ========================================================================= */

/**
 * @brief The IRQ vector's target. The board's vector table branches here.
 *
 * Naked because the prologue and epilogue ARE the mechanism. Stores the frame
 * on the interrupted thread's System-mode stack, calls the dispatcher on the
 * Supervisor stack, and restores whatever frame it hands back, which is a
 * DIFFERENT thread's after a switch.
 */
extern "C" [[gnu::naked]] void cyros_port_irq_entry(void)
{
   asm volatile(
      "sub    lr, lr, #4                    \n"   /* the interrupted instruction */
      "srsdb  sp!, #0x1f                    \n"   /* its pc and CPSR, on its stack */
      "cps    #0x1f                         \n"
      "push   {r0-r12, lr}                  \n"
      CYROS_ARMV8R_FP_SAVE
      "mov    r0, sp                        \n"   /* the frame */
      "movs   r1, #0                        \n"   /* entered_by_irq */
      "cps    #0x13                         \n"   /* the interrupt stack */
      "bl     cyros_port_exception_dispatch \n"
      "b      cyros_port_exception_exit     \n");
}

/**
 * @brief The SVC vector's target: a yield. The second half, from
 *        `cyros_port_exception_exit`, is every entry's exit, and how
 *        cyros_port_start_first enters a thread for the first time.
 */
extern "C" [[gnu::naked]] void cyros_port_svc_entry(void)
{
   asm volatile(
      "srsdb  sp!, #0x1f                    \n"   /* lr already follows the svc */
      "cps    #0x1f                         \n"
      "push   {r0-r12, lr}                  \n"
      CYROS_ARMV8R_FP_SAVE
      "mov    r0, sp                        \n"
      "movs   r1, #1                        \n"   /* entered_by_svc */
      "cps    #0x13                         \n"
      "bl     cyros_port_exception_dispatch \n"
      ".global cyros_port_exception_exit    \n"
      ".type  cyros_port_exception_exit, %function \n"
      ".thumb_func                          \n"
      "cyros_port_exception_exit:           \n"
      "cps    #0x1f                         \n"
      "mov    sp, r0                        \n"
      CYROS_ARMV8R_FP_RESTORE
      "pop    {r0-r12, lr}                  \n"
      "rfeia  sp!                           \n");
}

/**
 * @brief The C half of every IRQ and SVC. Returns the frame to resume.
 *
 * An SVC is a yield. An IRQ is acknowledged and dispatched by INTID: SGI 0 is
 * the reschedule, PPI 30 the timer, anything else a device's.
 */
extern "C" std::uint32_t* cyros_port_exception_dispatch(std::uint32_t* frame, std::uint32_t entered_by)
{
   core_state& state = cores[this_core()];
   ++state.depth;
   state.frame = frame;

   if (entered_by == entered_by_svc) {
      run_reschedule(state);
   }
   else {
      std::uint32_t const iar = armv8r::read_icc_iar1();
      std::uint32_t const intid = iar & 0xFFFFFFu;
      if (intid == armv8r::reschedule_intid) {
         /* Ended BEFORE the reschedule, so a request made while it runs (a wake
          * from inside the scheduler) is kept for the next one rather than
          * lost. */
         armv8r::write_icc_eoir1(iar);
         run_reschedule(state);
      }
      else if (intid == armv8r::timer_intid) {
         armv8r::timer_interrupt();
         armv8r::write_icc_eoir1(iar);
      }
      else if (intid != armv8r::spurious_intid) {
         cyros_armv8r_device_interrupt(intid);
         armv8r::write_icc_eoir1(iar);
      }
   }

   --state.depth;
   return state.frame;
}

/**
 * @brief The default for a device interrupt nobody handles: panic, naming it.
 */
extern "C" [[gnu::weak]] void cyros_armv8r_device_interrupt(std::uint32_t intid)
{
   cyros_port_system_error(intid, 0, "device interrupt with no handler", 0);
}


/* ============================================================================
 * Platform Initialisation
 * ========================================================================= */

namespace cyros::port::armv8r
{

void init_this_core()
{
   /* Masked for the whole of bring-up, as on every port: nothing may enter
    * the kernel until cyros_port_start_first has a thread to run. */
   asm volatile("cpsid i" ::: "memory");

#if defined(__ARM_FP)
   /* The FPU, on this core, whatever the board did: the exception path saves
    * FP state, and a disabled FPU would make that an Undefined Instruction. */
   write_cpacr(read_cpacr() | cpacr_fpu_full_access);
   isb();
   asm volatile("vmsr fpexc, %0" : : "r"(fpexc_en) : "memory");
   isb();
#endif

   /* The CPU interface through system registers. The R52 resets with EL2
    * allowing it (ICC_HSRE.Enable), and a board's EL2 code must keep that. */
   write_icc_sre(read_icc_sre() | 1u);
   isb();
   CYROS_ASSERT((read_icc_sre() & 1u) != 0u);

   /* This core's redistributor, checked to be this core's, then woken. */
   std::uint32_t const core = core_number();
   std::uintptr_t const rd = redistributor(core);
   CYROS_ASSERT_OP(reg(rd + gicr_typer + 4u) & 0xFFu, ==, core);
   reg(rd + gicr_waker) = reg(rd + gicr_waker) & ~gicr_waker_sleep;
   while ((reg(rd + gicr_waker) & gicr_waker_asleep) != 0u) {}

   /* The reschedule SGI and the timer PPI: Group 1, their priorities, and
    * enabled. A reschedule left pending by a previous kernel lifecycle would
    * otherwise fire into the next lifecycle's first thread. */
   std::uint32_t const ours = (1u << reschedule_intid) | (1u << timer_intid);
   reg(rd + gicr_igroupr0) = reg(rd + gicr_igroupr0) | ours;
   reg8(rd + gicr_ipriorityr + reschedule_intid) = static_cast<std::uint8_t>(reschedule_priority);
   reg8(rd + gicr_ipriorityr + timer_intid)      = static_cast<std::uint8_t>(timer_priority);
   reg(rd + gicr_icpendr0) = 1u << reschedule_intid;
   reg(rd + gicr_isenabler0) = ours;
   dsb();

   /* The CPU interface: everything we use let through, all eight priority
    * bits as group priority (nothing nests, so grouping decides nothing),
    * one write of ICC_EOIR1 both dropping and deactivating, and Group 1 on. */
   write_icc_pmr(pmr_open);
   write_icc_bpr1(0u);
   write_icc_ctlr(read_icc_ctlr() & ~(1u << 1));
   write_icc_igrpen1(1u);
   isb();

   /* The timer stays off until the time source starts it. */
   write_cntp_ctl(cntp_ctl_imask);
   write_tpidrprw(0u);

   /* The MPU on with its background region, every core able if the bootstrap
    * core was, and the guard region off until cyros_port_start_first has a
    * thread to guard: the bring-up runs on the boot stack. ORed in, so an MPU
    * the application already set up keeps its configuration. */
   if constexpr (guard_built) {
      CYROS_ASSERT((mpu_regions() > guard_region) == guard_available);
      if (guard_available) {
         write_prlar15(0u);
         write_sctlr(read_sctlr() | sctlr_m | sctlr_br);
      }
   }
   isb();
}

void raise_reschedule(std::uint32_t core) noexcept
{
   /* ICC_SGI1R: the target list is a bitmap of Aff0 within the sender's own
    * Aff1, which is the R52's cluster. The DSB makes everything this core
    * wrote visible before the target can take the interrupt. */
   std::uint64_t const aff1 = (read_mpidr() >> 8) & 0xFFu;
   std::uint64_t const value = (std::uint64_t{1} << core)
                             | (aff1 << 16)
                             | (std::uint64_t{reschedule_intid} << 24);
   dsb();
   write_icc_sgi1r(value);
   isb();
}

} // namespace cyros::port::armv8r

void cyros_port_init(cyros_port_reschedule_t handler)
{
   CYROS_ASSERT(handler != nullptr);
   asm volatile("cpsid i" ::: "memory");

   /* How many priority bits the GIC keeps: write all ones to PMR and read back
    * which stuck. The lowest of them is PMR fully open, the reschedule one step
    * above it, and the timer in the middle, well above the mask a
    * preempt-disable sets. */
   armv8r::write_icc_sre(armv8r::read_icc_sre() | 1u);
   armv8r::isb();
   armv8r::write_icc_pmr(0xFFu);
   pmr_open = armv8r::read_icc_pmr();
   std::uint32_t const step = pmr_open & (~pmr_open + 1u);
   CYROS_ASSERT_OP(step, <=, 0x40u);   /* at least two priority bits */
   reschedule_priority = pmr_open - step;
   timer_priority      = 0x80u & pmr_open;

   /* The distributor's Group 1 enable, which every SGI and PPI also needs
    * under affinity routing. ORed in, so an application's configuration
    * stands. */
   std::uintptr_t const gicd = distributor();
   reg(gicd + gicd_ctlr) = reg(gicd + gicd_ctlr) | gicd_ctlr_are | gicd_ctlr_grp1;
   while ((reg(gicd + gicd_ctlr) & gicd_ctlr_rwp) != 0u) {}

   /* The bootstrap core decides whether there is a guard, and every core's
    * init_this_core, this one's included, checks it agrees. */
   guard_available = mpu_regions() > guard_region;

#if defined(__ARM_FP)
   /* MVFR0.SIMDReg 2 is 32 double registers. Read with the FPU on, which the
    * board's reset path guarantees in an FP build. */
   std::uint32_t mvfr0;
   asm volatile(".fpu neon-fp-armv8 \n vmrs %0, mvfr0" : "=r"(mvfr0));
   cyros_armv8r_fp_d32 = (mvfr0 & 0xFu) == 2u ? 1u : 0u;
#endif

   reschedule_handler = handler;
   armv8r::init_this_core();
}


/* ============================================================================
 * Interrupt Control
 * ========================================================================= */

bool cyros_port_interrupts_enabled(void)
{
   return (armv8r::read_cpsr() & armv8r::cpsr_i) == 0u;
}

cyros_mask_token_t cyros_port_irq_save(void)
{
   std::uint32_t const cpsr = armv8r::read_cpsr();
   asm volatile("cpsid i" ::: "memory");
   return cpsr & armv8r::cpsr_i;
}

void cyros_port_irq_restore(cyros_mask_token_t token)
{
   /* Restores to MASKED when the token says the caller was already masked.
    * Unmasking takes a pending reschedule at once, the ISB making sure it is
    * taken before this returns. */
   if ((token & armv8r::cpsr_i) == 0u) {
      asm volatile("cpsie i \n isb" ::: "memory");
   } else {
      asm volatile("cpsid i" ::: "memory");
   }
}


/* ============================================================================
 * Preemption Control
 * ========================================================================= */

cyros_mask_token_t cyros_port_preempt_disable(void)
{
   /* Lowering PMR is self-synchronising: no interrupt it masks is taken once
    * the write has executed. */
   std::uint32_t const previous = armv8r::read_icc_pmr();
   armv8r::write_icc_pmr(reschedule_priority);
   return previous;
}

void cyros_port_preempt_enable(cyros_mask_token_t token)
{
   /* Raising PMR back takes a pended reschedule at once when I is clear, the
    * ISB making sure it is taken before this returns. */
   armv8r::write_icc_pmr(token);
   armv8r::isb();
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
   CYROS_ASSERT_OP(stack_size, >=, 2u * armv8r::frame_bytes);

   auto const base = reinterpret_cast<std::uintptr_t>(stack_base);
   std::uintptr_t const top = (base + stack_size) & ~static_cast<std::uintptr_t>(7u);
   CYROS_ASSERT_OP(top - armv8r::frame_bytes, >, base);

   /* The guard, when there is one: the bottom of the buffer rounded up to the
    * MPU's granule. The decision is cyros_port_init's, so it must have run. */
   CYROS_ASSERT(reschedule_handler != nullptr);
   std::uintptr_t guard = 0u;
   if (guarded()) {
      guard = (base + guard_granule - 1u) & ~(guard_granule - 1u);
      CYROS_ASSERT_OP(top - armv8r::frame_bytes, >, guard + guard_bytes);
   }

   /* The frame cyros_port_exception_exit consumes. RFE enters `entry` in
    * System mode with I clear, so the thread starts at baseline priority, and
    * its sp is the frame's top, the 8-aligned top of its stack. FIQ stays
    * masked: nothing here uses Group 0. Thumb follows the entry's address. */
   auto* frame = reinterpret_cast<std::uint32_t*>(top - armv8r::frame_bytes);
   for (std::uint32_t i = 0; i < armv8r::frame_words; ++i) { frame[i] = 0u; }

   auto const address = reinterpret_cast<std::uintptr_t>(entry);
   frame[armv8r::frame_r0]   = reinterpret_cast<std::uintptr_t>(arg);
   frame[armv8r::frame_lr]   = reinterpret_cast<std::uintptr_t>(&thread_return_trap);
   frame[armv8r::frame_pc]   = address & ~std::uintptr_t{1};
   frame[armv8r::frame_cpsr] = armv8r::cpsr_mode_sys | armv8r::cpsr_f
                             | ((address & 1u) != 0u ? armv8r::cpsr_t : 0u);

   context->frame = frame;
   context->tls   = nullptr;
   context->guard = guard;
}

void cyros_port_context_destroy(cyros_port_context* context)
{
   CYROS_ASSERT(context != nullptr);
   /* Nothing is owned. Poisoned, so a resume faults on a null frame. */
   context->frame = nullptr;
   context->tls   = nullptr;
   context->guard = 0u;
}

void cyros_port_switch(cyros_port_context* from, cyros_port_context* to)
{
   core_state& state = cores[this_core()];

   /* Reachable only from the exception path, where `frame` is the running
    * thread's complete frame. Anywhere else this would save garbage. */
   CYROS_ASSERT_OP(state.depth, ==, 1u);
   CYROS_ASSERT(to != nullptr);
   CYROS_ASSERT(to->frame != nullptr);

   if (from != nullptr) {
      from->frame = state.frame;
      from->tls   = reinterpret_cast<void*>(armv8r::read_tpidrprw());
   }
   state.frame = to->frame;
   armv8r::write_tpidrprw(reinterpret_cast<std::uintptr_t>(to->tls));

   /* The incoming thread's guard. The handler runs on the interrupt stack and
    * the exit reads the new frame, which lies above the new guard, so the
    * moment of the move is free. */
   if (guarded()) {
      load_guard(to->guard);
   }
}

/* Not marked [[noreturn]], as on the other ports: port_core.h declares it
 * plain. */
void cyros_port_start_first(cyros_port_context* first)
{
   CYROS_ASSERT(first != nullptr);
   CYROS_ASSERT(first->frame != nullptr);
   CYROS_ASSERT_OP(armv8r::read_cpsr() & armv8r::cpsr_mode_mask, ==, armv8r::cpsr_mode_sys);

   core_state& state = cores[this_core()];
   state.depth = 0u;
   state.frame = first->frame;
   armv8r::write_tpidrprw(reinterpret_cast<std::uintptr_t>(first->tls));

   /* The first thread's guard, on from here: only threads run on guarded
    * stacks, never this core's boot or interrupt stack. */
   if (guarded()) {
      load_guard(first->guard);
   }

   /* The System-mode stack this core is standing on becomes its interrupt
    * stack, Supervisor mode's SP. Nothing above it is ever returned to, and
    * everything a handler uses lies below. Then the exit enters the thread. */
   asm volatile(
      "mov   r1, sp                       \n"
      "bic   r1, r1, #7                   \n"
      "cps   #0x13                        \n"
      "mov   sp, r1                       \n"
      "mov   r0, %0                       \n"
      "b     cyros_port_exception_exit    \n"
      :
      : "r"(first->frame)
      : "r0", "r1", "memory");

   __builtin_unreachable();
}


/* ============================================================================
 * Reschedule Requests
 * ========================================================================= */

void cyros_port_thread_yield(void)
{
   /* port.h asks ports that can observe the execution priority to assert the
    * baseline precondition. This one can. */
   CYROS_ASSERT_OP(cores[this_core()].depth, ==, 0u);
   CYROS_ASSERT((armv8r::read_cpsr() & armv8r::cpsr_i) == 0u);
   CYROS_ASSERT_OP(armv8r::read_icc_pmr(), ==, pmr_open);

   /* Synchronous: the exception path runs the reschedule before the next
    * instruction, and resumes here when this thread is next switched in. */
   asm volatile("svc 0" ::: "memory");
}

void cyros_port_pend_reschedule(void)
{
   /* Callable from anywhere. Raising the SGI is the whole implementation: it is
    * taken when I is clear and PMR lets it through, which is port.h's next
    * safe point. */
   armv8r::raise_reschedule(this_core());
}


/* ============================================================================
 * Thread-Local Storage
 * ========================================================================= */

void cyros_port_set_tls_pointer(void* tls_base)
{
   armv8r::write_tpidrprw(reinterpret_cast<std::uintptr_t>(tls_base));
}

void* cyros_port_get_tls_pointer(void)
{
   return reinterpret_cast<void*>(armv8r::read_tpidrprw());
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
   /* WFI wakes on any interrupt the GIC signals to this core, whatever CPSR.I
    * says, so this is safe in the kernel's idle loop regardless of masking. */
   asm volatile("dsb sy \n wfi" ::: "memory");
}


/* ============================================================================
 * Measurement
 * ========================================================================= */

std::uint64_t cyros_port_timestamp(void)
{
   /* The generic timer's count, which every core reads, so stamps compare
    * across cores, as MTIME's do on the RP2350. 62.5 MHz on QEMU. */
   return armv8r::read_cntpct();
}


/* ============================================================================
 * Debug & Diagnostics
 * ========================================================================= */

void cyros_port_system_error(std::uintptr_t auxilary1,
                             std::uintptr_t auxilary2,
                             char const* file_optional,
                             int line_optional)
{
   asm volatile("cpsid i" ::: "memory");

   armv8r::write0("\n*** CYROS PANIC ***\n  aux1 = ");
   armv8r::write_hex(static_cast<std::uint32_t>(auxilary1));
   armv8r::write0("\n  aux2 = ");
   armv8r::write_hex(static_cast<std::uint32_t>(auxilary2));

   if (file_optional != nullptr && file_optional[0] != '\0') {
      armv8r::write0("\n  at   = ");
      armv8r::write0(file_optional);
      armv8r::write0(":");
      armv8r::write_hex(static_cast<std::uint32_t>(line_optional));
   }
   armv8r::write0("\n");

   armv8r::host_exit(1u);
}

void cyros_port_wait_for_debugger(void)
{
   static volatile int resume = 0;
   armv8r::write0("cyros: waiting for debugger, set 'resume' to 1\n");
   while (resume == 0) { asm volatile("nop"); }
}

void cyros_port_breakpoint(void)
{
   asm volatile("bkpt 0");
}

void* cyros_port_get_stack_pointer(void)
{
   void* sp = nullptr;
   asm volatile("mov %0, sp" : "=r"(sp));
   return sp;
}
