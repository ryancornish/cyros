/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file armv8r.hpp
 * @brief The armv8r core layer's internal header: system registers, the
 *        exception frame, the Cortex-R52's GIC, semihosting, and what the
 *        layer needs from its target.
 *
 * ARMv8-R AArch32 at EL1, as the Cortex-R52 runs it. Everything here is the
 * architecture or the R52 itself: the generic timer, the GICv3 CPU interface
 * (system registers), and the R52's integrated GIC, whose distributor sits at
 * PERIPHBASE (IMP_CBAR) and its redistributors 1 MB above (Cortex-R52 TRM,
 * "GIC memory map"). So unlike the M-profile layers this one finds its
 * interrupt controller without being told.
 */
#ifndef CYROS_PORT_ARMV8R_HPP
#define CYROS_PORT_ARMV8R_HPP

#include <cstdint>

#if !defined(__ARM_ARCH_PROFILE) || __ARM_ARCH_PROFILE != 'R' || __ARM_ARCH < 8
#  error "the armv8r core layer is for ARMv8-R AArch32 (-mcpu=cortex-r52)"
#endif

namespace cyros::port::armv8r
{

/* ============================================================================
 * System registers
 * ----------------------------------------------------------------------------
 * One accessor per register, because the encoding is an immediate in the
 * instruction, as on RISC-V.
 * ========================================================================= */

#define CYROS_ARMV8R_CP15(name, op1, crn, crm, op2)                                   \
   inline std::uint32_t read_##name() noexcept                                        \
   {                                                                                  \
      std::uint32_t value;                                                            \
      asm volatile("mrc p15, " #op1 ", %0, " #crn ", " #crm ", " #op2                \
                   : "=r"(value) : : "memory");                                       \
      return value;                                                                   \
   }                                                                                  \
   inline void write_##name(std::uint32_t value) noexcept                             \
   {                                                                                  \
      asm volatile("mcr p15, " #op1 ", %0, " #crn ", " #crm ", " #op2                \
                   : : "r"(value) : "memory");                                        \
   }

CYROS_ARMV8R_CP15(mpidr,     0, c0,  c0,  5)
CYROS_ARMV8R_CP15(mpuir,     0, c0,  c0,  4)
CYROS_ARMV8R_CP15(cpacr,     0, c1,  c0,  2)
CYROS_ARMV8R_CP15(sctlr,     0, c1,  c0,  0)
CYROS_ARMV8R_CP15(prbar15,   0, c6,  c15, 4)   /* EL1 MPU region 15, direct     */
CYROS_ARMV8R_CP15(prlar15,   0, c6,  c15, 5)
CYROS_ARMV8R_CP15(cbar,      1, c15, c3,  0)   /* IMP_CBAR: PERIPHBASE           */
CYROS_ARMV8R_CP15(tpidrprw,  0, c13, c0,  4)   /* privileged thread ID: the TLS  */
CYROS_ARMV8R_CP15(cntfrq,    0, c14, c0,  0)
CYROS_ARMV8R_CP15(cntp_ctl,  0, c14, c2,  1)
CYROS_ARMV8R_CP15(icc_pmr,   0, c4,  c6,  0)
CYROS_ARMV8R_CP15(icc_iar1,  0, c12, c12, 0)
CYROS_ARMV8R_CP15(icc_eoir1, 0, c12, c12, 1)
CYROS_ARMV8R_CP15(icc_bpr1,  0, c12, c12, 3)
CYROS_ARMV8R_CP15(icc_ctlr,  0, c12, c12, 4)
CYROS_ARMV8R_CP15(icc_sre,   0, c12, c12, 5)
CYROS_ARMV8R_CP15(icc_igrpen1, 0, c12, c12, 7)
CYROS_ARMV8R_CP15(dfsr,      0, c5,  c0,  0)
CYROS_ARMV8R_CP15(dfar,      0, c6,  c0,  0)
CYROS_ARMV8R_CP15(ifsr,      0, c5,  c0,  1)
CYROS_ARMV8R_CP15(ifar,      0, c6,  c0,  2)

#undef CYROS_ARMV8R_CP15

/** The physical count, shared by every core (CNTPCT, one 64-bit read). */
inline std::uint64_t read_cntpct() noexcept
{
   std::uint32_t low, high;
   asm volatile("mrrc p15, 0, %0, %1, c14" : "=r"(low), "=r"(high) : : "memory");
   return (static_cast<std::uint64_t>(high) << 32) | low;
}

/** This core's physical compare (CNTP_CVAL), in one 64-bit write. */
inline void write_cntp_cval(std::uint64_t value) noexcept
{
   asm volatile("mcrr p15, 2, %0, %1, c14"
                : : "r"(static_cast<std::uint32_t>(value)), "r"(static_cast<std::uint32_t>(value >> 32))
                : "memory");
}

/** Generate a Group 1 SGI (ICC_SGI1R, one 64-bit write). */
inline void write_icc_sgi1r(std::uint64_t value) noexcept
{
   asm volatile("mcrr p15, 0, %0, %1, c12"
                : : "r"(static_cast<std::uint32_t>(value)), "r"(static_cast<std::uint32_t>(value >> 32))
                : "memory");
}

inline std::uint32_t read_cpsr() noexcept
{
   std::uint32_t value;
   asm volatile("mrs %0, cpsr" : "=r"(value) : : "memory");
   return value;
}

inline void isb() noexcept { asm volatile("isb" ::: "memory"); }
inline void dsb() noexcept { asm volatile("dsb sy" ::: "memory"); }

/* CPSR. */
inline constexpr std::uint32_t cpsr_mode_mask = 0x1Fu;
inline constexpr std::uint32_t cpsr_mode_sys  = 0x1Fu;
inline constexpr std::uint32_t cpsr_mode_svc  = 0x13u;
inline constexpr std::uint32_t cpsr_t         = 1u << 5;
inline constexpr std::uint32_t cpsr_f         = 1u << 6;
inline constexpr std::uint32_t cpsr_i         = 1u << 7;

/* CPACR: CP10 and CP11, which are the FPU, at full access from EL1 and EL0. */
inline constexpr std::uint32_t cpacr_fpu_full_access = 0xFu << 20;
inline constexpr std::uint32_t fpexc_en              = 1u << 30;

/* SCTLR: the EL1 MPU, and its background region (the default memory map for
 * any EL1 access no region matches). */
inline constexpr std::uint32_t sctlr_m  = 1u << 0;
inline constexpr std::uint32_t sctlr_br = 1u << 17;

/* PMSAv8-32 region registers. PRBAR: BASE[31:6], AP[2:1], XN[0]. PRLAR:
 * LIMIT[31:6], the last 64-byte block included, AttrIndx[3:1], EN[0]. */
inline constexpr std::uint32_t prbar_ap_ro_el1 = 2u << 1;
inline constexpr std::uint32_t prbar_xn        = 1u << 0;
inline constexpr std::uint32_t prlar_en        = 1u << 0;

/* The generic timer's EL1 physical timer: CNTP_CTL bits and its PPI. */
inline constexpr std::uint32_t cntp_ctl_enable = 1u << 0;
inline constexpr std::uint32_t cntp_ctl_imask  = 1u << 1;
inline constexpr std::uint32_t timer_intid     = 30u;

/* The reschedule is SGI 0, local or from another core. */
inline constexpr std::uint32_t reschedule_intid = 0u;

/* ICC_IAR1 reads this when nothing is pending. */
inline constexpr std::uint32_t spurious_intid = 1023u;

/* ============================================================================
 * The exception frame
 * ----------------------------------------------------------------------------
 * Every IRQ and SVC stores the interrupted thread's registers on its OWN stack
 * (the System-mode SP): r0 to r12, its lr, then the return address and the
 * CPSR to return with, which SRSDB stores and RFEIA loads. In a build with FP
 * instructions (__ARM_FP) the FP state goes below them: FPSCR and a pad word,
 * d0 to d15, then d16 to d31, whose slot is reserved even on a core with 16
 * double registers so the layout is fixed. A suspended thread is a pointer to
 * the lowest word, as on RISC-V.
 *
 *   frame + 0                     d16-d31  (FP builds, 32 words)
 *   frame + 32                    d0-d15   (32 words)
 *   frame + 64                    FPSCR, pad
 *   frame + fp_words + 0..12      r0-r12
 *   frame + fp_words + 13         lr
 *   frame + fp_words + 14, 15     pc, CPSR
 * ========================================================================= */

#if defined(__ARM_FP)
inline constexpr std::uint32_t fp_words = 32u + 32u + 2u;
#else
inline constexpr std::uint32_t fp_words = 0u;
#endif

inline constexpr std::uint32_t frame_words = fp_words + 16u;
inline constexpr std::uint32_t frame_bytes = frame_words * 4u;
inline constexpr std::uint32_t frame_r0    = fp_words + 0u;
inline constexpr std::uint32_t frame_r1    = fp_words + 1u;
inline constexpr std::uint32_t frame_lr    = fp_words + 13u;
inline constexpr std::uint32_t frame_pc    = fp_words + 14u;
inline constexpr std::uint32_t frame_cpsr  = fp_words + 15u;

static_assert(frame_bytes % 8u == 0u, "the AAPCS keeps sp 8-aligned at calls, and the frame must keep it");

/* ============================================================================
 * Semihosting
 * ----------------------------------------------------------------------------
 * For the panic report only. On ARMv8 AArch32 the trap is HLT, 0x3C in T32,
 * which QEMU services at any exception level. Operation numbers as everywhere.
 * ========================================================================= */

[[gnu::noinline]] inline long semihost(long op, void volatile* arg) noexcept
{
   register long r0 asm("r0") = op;
   register void volatile* r1 asm("r1") = arg;
   asm volatile("hlt 0x3c" : "+r"(r0) : "r"(r1) : "memory");
   return r0;
}

inline void write0(char const* text) noexcept
{
   semihost(0x04, const_cast<char*>(text));
}

inline void write_hex(std::uint32_t value) noexcept
{
   char buffer[11] = { '0', 'x' };
   for (int i = 0; i < 8; ++i) {
      std::uint32_t const nibble = (value >> ((7 - i) * 4)) & 0xFu;
      buffer[2 + i] = static_cast<char>(nibble < 10 ? '0' + nibble : 'a' + (nibble - 10));
   }
   buffer[10] = '\0';
   write0(buffer);
}

[[noreturn]] inline void host_exit(std::uint32_t code) noexcept
{
   volatile std::uint32_t block[2] = { 0x20026u, code };   /* ADP_Stopped_ApplicationExit */
   semihost(0x20, block);                                  /* SYS_EXIT_EXTENDED           */
   for (;;) { asm volatile("wfi"); }
}

/* ============================================================================
 * The core layer's own functions that a target calls
 * ========================================================================= */

/**
 * @brief Per-core bring-up: masks, this core's redistributor and CPU
 *        interface, the reschedule SGI and the timer PPI, and the stack guard.
 *
 * cyros_port_init runs it on the bootstrap core. A multicore target runs it on
 * every other core before that core enters the kernel.
 */
void init_this_core();

/** Raise the reschedule SGI on core `core`, this one or another. */
void raise_reschedule(std::uint32_t core) noexcept;

/** This core's number within its cluster: MPIDR.Aff0. */
inline std::uint32_t core_number() noexcept
{
   return read_mpidr() & 0xFFu;
}

/* ============================================================================
 * What the core layer needs from its TARGET
 * ========================================================================= */

/** The body of the EL1 physical timer interrupt. Defined by the target, which
 * hands it to its time source (src/port/common/mtime.hpp). */
void timer_interrupt() noexcept;

} // namespace cyros::port::armv8r

/**
 * @brief Every interrupt that is neither the reschedule nor the timer: a
 *        device's, by its GIC INTID. WEAK, so the application supplies its
 *        devices' handlers, as it owns an M-profile vector table.
 *
 * Called inside the IRQ path with interrupts masked, on the core's interrupt
 * stack, after the acknowledge and before the end of interrupt. The default
 * panics, naming the INTID.
 */
extern "C" void cyros_armv8r_device_interrupt(std::uint32_t intid);

#endif /* CYROS_PORT_ARMV8R_HPP */
