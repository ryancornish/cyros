/**
 * @file cortex_m.hpp
 * @brief Minimal Cortex-M register access, barriers and semihosting.
 *
 * Deliberately NOT CMSIS. CMSIS would be a vendor header tree in the build for
 * the dozen registers this port touches, and it brings its own opinions about
 * startup and device headers. Everything needed here is architectural, defined
 * by the ARMv8-M Architecture Reference Manual, and identical on every Cortex-M
 * part including both the QEMU mps2-an505 bench and the STM32U575.
 *
 * When a second ARM port appears (cortex_m4, ARMv7-M) this file is the thing to
 * lift into src/port/arm/common/. It is kept here until there is a second
 * caller, because a shared directory with one user is just a longer path.
 */

#ifndef CYROS_PORT_CORTEX_M_HPP
#define CYROS_PORT_CORTEX_M_HPP

#include <cstdint>

namespace cyros::port::cortex_m
{

/* ============================================================================
 * Memory-mapped registers
 * ----------------------------------------------------------------------------
 * Addresses are architectural (the System Control Space at 0xE000E000).
 * ========================================================================= */

inline constexpr std::uintptr_t scs_base   = 0xE000E000u;
inline constexpr std::uintptr_t systick_base = scs_base + 0x010u;
inline constexpr std::uintptr_t nvic_base    = scs_base + 0x100u;
inline constexpr std::uintptr_t scb_base     = scs_base + 0xD00u;

inline volatile std::uint32_t& reg(std::uintptr_t address) noexcept
{
   return *reinterpret_cast<volatile std::uint32_t*>(address);
}

inline volatile std::uint8_t& reg8(std::uintptr_t address) noexcept
{
   return *reinterpret_cast<volatile std::uint8_t*>(address);
}

/* SCB */
inline constexpr std::uintptr_t scb_icsr  = scb_base + 0x04u;  /* Interrupt Control and State  */
inline constexpr std::uintptr_t scb_aircr = scb_base + 0x0Cu;  /* Application Interrupt and Reset Control */

/* AIRCR.PRIGROUP, bits [10:8]. It splits each 8-bit priority field into a
 * GROUP (preemption) part and a SUB-priority part, and only the group part
 * decides preemption and BASEPRI masking. See the priority discussion in
 * port_cortex_m33.cpp: getting this wrong makes two adjacent priority values
 * behave as one. */
inline constexpr std::uint32_t aircr_prigroup_shift = 8u;
inline constexpr std::uint32_t aircr_prigroup_mask  = 0x7u;
inline constexpr std::uintptr_t scb_shpr  = scb_base + 0x18u;  /* System Handler Priority, 12 bytes */
inline constexpr std::uintptr_t scb_shcsr = scb_base + 0x24u;  /* System Handler Control and State */
inline constexpr std::uintptr_t scb_cpacr = scb_base + 0x88u;  /* Coprocessor Access Control   */

/* CPACR grants access to CP10 and CP11, which together ARE the FPU. Both are
 * two-bit fields: 0b11 is full access from privileged and unprivileged code.
 * Touching an FP instruction with these clear is a UsageFault (NOCP), which is
 * the usual way a hard-float image dies on its first floating-point value. */
inline constexpr std::uint32_t cpacr_fpu_full_access = (0x3u << 20) | (0x3u << 22);

/* Floating-Point Context Control. The reset defaults are ASPEN=1 and LSPEN=1:
 * the hardware allocates an extended exception frame for a thread that has
 * used the FPU, and defers writing s0-s15 into it until something actually
 * needs the registers (lazy stacking).
 *
 * The port KEEPS those defaults. The conditional `vstmdb {s16-s31}` in PendSV
 * is itself an FP instruction, so it forces any pending lazy save out to
 * FPCAR, which points into the OUTGOING thread's frame, before anything
 * switches. That is the ordering the lazy scheme requires, and it is why the
 * FP save in the handler is conditional on the same bit the hardware used to
 * decide whether to allocate the frame. */
inline constexpr std::uintptr_t fpu_fpccr = 0xE000EF34u;
inline constexpr std::uintptr_t scb_ccr   = scb_base + 0x14u;  /* Configuration and Control    */

inline constexpr std::uint32_t icsr_pendsvset = 1u << 28;
inline constexpr std::uint32_t icsr_pendstclr = 1u << 25;

/* System handler priority byte offsets within SHPR. The register file is
 * indexed from handler 4 (MemManage), so PendSV (14) is byte 10 and
 * SysTick (15) is byte 11. */
inline constexpr std::uintptr_t shpr_pendsv  = scb_shpr + 10u;
inline constexpr std::uintptr_t shpr_systick = scb_shpr + 11u;

/* SysTick */
inline constexpr std::uintptr_t systick_ctrl = systick_base + 0x00u;
inline constexpr std::uintptr_t systick_load = systick_base + 0x04u;
inline constexpr std::uintptr_t systick_val  = systick_base + 0x08u;

inline constexpr std::uint32_t systick_ctrl_enable    = 1u << 0;
inline constexpr std::uint32_t systick_ctrl_tickint   = 1u << 1;
inline constexpr std::uint32_t systick_ctrl_clksource = 1u << 2;
inline constexpr std::uint32_t systick_ctrl_countflag = 1u << 16;

/* SysTick's reload field is 24 bits. Anything wider silently truncates, which
 * is a whole class of "my tick rate is wrong" bugs. */
inline constexpr std::uint32_t systick_reload_max = 0x00FFFFFFu;


/* ============================================================================
 * Core register access and barriers
 * ========================================================================= */

inline void dsb() noexcept { asm volatile("dsb 0xF" ::: "memory"); }
inline void isb() noexcept { asm volatile("isb 0xF" ::: "memory"); }
inline void wfi() noexcept { asm volatile("wfi" ::: "memory"); }
inline void nop() noexcept { asm volatile("nop" ::: "memory"); }

inline std::uint32_t get_primask() noexcept
{
   std::uint32_t value;
   asm volatile("mrs %0, primask" : "=r"(value) :: "memory");
   return value;
}

inline void set_primask(std::uint32_t value) noexcept
{
   asm volatile("msr primask, %0" :: "r"(value) : "memory");
}

inline void disable_irq() noexcept { asm volatile("cpsid i" ::: "memory"); }
inline void enable_irq()  noexcept { asm volatile("cpsie i" ::: "memory"); }

inline std::uint32_t get_basepri() noexcept
{
   std::uint32_t value;
   asm volatile("mrs %0, basepri" : "=r"(value) :: "memory");
   return value;
}

inline void set_basepri(std::uint32_t value) noexcept
{
   asm volatile("msr basepri, %0" :: "r"(value) : "memory");
}

inline std::uint32_t get_ipsr() noexcept
{
   std::uint32_t value;
   asm volatile("mrs %0, ipsr" : "=r"(value) :: "memory");
   return value;
}

inline std::uint32_t get_psp() noexcept
{
   std::uint32_t value;
   asm volatile("mrs %0, psp" : "=r"(value) :: "memory");
   return value;
}

inline void set_psp(std::uint32_t value) noexcept
{
   asm volatile("msr psp, %0" :: "r"(value) : "memory");
}

inline void set_psplim(std::uint32_t value) noexcept
{
   /* ARMv8-M only. The stack-limit registers are the reason this port targets
    * the M33 rather than the M4: cyros hands every thread a caller-owned stack
    * buffer, and without PSPLIM an overrun is silent corruption of whatever
    * sits below it. With it, the overrun is a UsageFault at the instruction
    * that caused it. */
   asm volatile("msr psplim, %0" :: "r"(value) : "memory");
}

inline std::uint32_t get_control() noexcept
{
   std::uint32_t value;
   asm volatile("mrs %0, control" : "=r"(value) :: "memory");
   return value;
}

/**
 * @brief True when executing inside an exception handler rather than a thread.
 *
 * IPSR carries the active exception number, and is zero in Thread mode. This is
 * the port's answer to "am I in an ISR", which several contract functions need
 * in order to choose between acting now and deferring.
 */
inline bool in_handler_mode() noexcept { return get_ipsr() != 0u; }


/* ============================================================================
 * Semihosting
 * ----------------------------------------------------------------------------
 * The host (QEMU, or a debugger over SWD) traps BKPT 0xAB, reads the operation
 * from r0 and an argument block from r1. This is the port's output and panic
 * channel before any UART exists, and it works identically on the bench and on
 * real silicon under a debugger.
 *
 * It is NOT free: each call traps to the host and takes microseconds. Nothing
 * on a hot path may use it.
 * ========================================================================= */

inline constexpr long sys_write0        = 0x04;
inline constexpr long sys_exit_extended = 0x20;
inline constexpr std::uint32_t adp_stopped_application_exit = 0x20026u;

inline long semihost(long op, void volatile* arg) noexcept
{
   register long r0 asm("r0") = op;
   register void volatile* r1 asm("r1") = arg;
   asm volatile("bkpt 0xAB" : "+r"(r0) : "r"(r1) : "memory");
   return r0;
}

inline void write0(char const* text) noexcept
{
   semihost(sys_write0, const_cast<char*>(text));
}

[[noreturn]] inline void host_exit(std::uint32_t code) noexcept
{
   volatile std::uint32_t block[2] = { adp_stopped_application_exit, code };
   semihost(sys_exit_extended, block);
   __builtin_unreachable();
}

/**
 * @brief Write a 32-bit value as 0x-prefixed hex.
 *
 * Panic reporting only. There is no printf on this target and pulling one in
 * would drag newlib's stdio, a heap and a reentrancy structure into a kernel
 * that otherwise needs none of them.
 */
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

} // namespace cyros::port

#endif /* CYROS_PORT_CORTEX_M_HPP */
