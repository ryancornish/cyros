/**
 * @file riscv.hpp
 * @brief The rv32 core layer's internal header: CSR access, the trap frame,
 *        semihosting, and the few things the layer needs from its target.
 *
 * Machine mode only, 32-bit, no FPU: what QEMU's `virt` and the RP2350's
 * Hazard3 both are. Everything here is architectural (the RISC-V privileged
 * specification) except the four target functions at the bottom, which exist
 * because RISC-V puts the software interrupt and the timer OUTSIDE the core:
 * raising a hart's soft IRQ is a CLINT register on `virt` and an SIO register on
 * the RP2350, where on ARM it is a bit in the SCB.
 */
#ifndef CYROS_PORT_RISCV_HPP
#define CYROS_PORT_RISCV_HPP

#include <cstdint>

#if !defined(__riscv) || __riscv_xlen != 32
#  error "the rv32 core layer is for 32-bit RISC-V"
#endif

namespace cyros::port::riscv
{

/* ============================================================================
 * CSR access
 * ----------------------------------------------------------------------------
 * One function per CSR, because the CSR is an immediate in the instruction and
 * a generic accessor would need a statement expression, which -Wpedantic
 * refuses.
 * ========================================================================= */

#define CYROS_RISCV_CSR_ACCESSORS(name)                                               \
   inline std::uint32_t read_##name() noexcept                                         \
   {                                                                                   \
      std::uint32_t value;                                                             \
      asm volatile("csrr %0, " #name : "=r"(value));                                   \
      return value;                                                                    \
   }                                                                                   \
   inline void write_##name(std::uint32_t value) noexcept                              \
   {                                                                                   \
      asm volatile("csrw " #name ", %0" : : "r"(value) : "memory");                   \
   }                                                                                   \
   inline void set_##name(std::uint32_t bits) noexcept                                 \
   {                                                                                   \
      asm volatile("csrs " #name ", %0" : : "r"(bits) : "memory");                    \
   }                                                                                   \
   inline void clear_##name(std::uint32_t bits) noexcept                               \
   {                                                                                   \
      asm volatile("csrc " #name ", %0" : : "r"(bits) : "memory");                    \
   }

CYROS_RISCV_CSR_ACCESSORS(mstatus)
CYROS_RISCV_CSR_ACCESSORS(mie)
CYROS_RISCV_CSR_ACCESSORS(mip)
CYROS_RISCV_CSR_ACCESSORS(mtvec)
CYROS_RISCV_CSR_ACCESSORS(mscratch)
CYROS_RISCV_CSR_ACCESSORS(mcountinhibit)
CYROS_RISCV_CSR_ACCESSORS(pmpcfg0)
CYROS_RISCV_CSR_ACCESSORS(pmpaddr0)

#undef CYROS_RISCV_CSR_ACCESSORS

inline std::uint32_t read_mhartid() noexcept
{
   std::uint32_t value;
   asm volatile("csrr %0, mhartid" : "=r"(value));
   return value;
}

inline std::uint32_t read_mcause() noexcept
{
   std::uint32_t value;
   asm volatile("csrr %0, mcause" : "=r"(value));
   return value;
}

inline std::uint32_t read_mepc() noexcept
{
   std::uint32_t value;
   asm volatile("csrr %0, mepc" : "=r"(value));
   return value;
}

inline std::uint32_t read_mtval() noexcept
{
   std::uint32_t value;
   asm volatile("csrr %0, mtval" : "=r"(value));
   return value;
}

/* The bits this layer uses. */
inline constexpr std::uint32_t mstatus_mie         = 1u << 3;
inline constexpr std::uint32_t mstatus_mpie        = 1u << 7;
inline constexpr std::uint32_t mstatus_mpp_machine = 3u << 11;

inline constexpr std::uint32_t mie_msie = 1u << 3;   /* software: the reschedule */
inline constexpr std::uint32_t mie_mtie = 1u << 7;   /* timer                    */
inline constexpr std::uint32_t mie_meie = 1u << 11;  /* external                 */

/* A PMP configuration byte: address matching NAPOT, and the lock bit, which
 * on a standard core is what binds machine mode. R, W and X are bits 0 to 2,
 * and a stack guard sets none of them. */
inline constexpr std::uint8_t pmpcfg_napot = 3u << 3;
inline constexpr std::uint8_t pmpcfg_lock  = 1u << 7;

inline constexpr std::uint32_t mcause_interrupt      = 1u << 31;
inline constexpr std::uint32_t cause_soft_interrupt  = 3u;
inline constexpr std::uint32_t cause_timer_interrupt = 7u;
inline constexpr std::uint32_t cause_ext_interrupt   = 11u;
inline constexpr std::uint32_t cause_ecall_machine   = 11u;
inline constexpr std::uint32_t cause_load_access     = 5u;
inline constexpr std::uint32_t cause_store_access    = 7u;

/* ============================================================================
 * The trap frame
 * ----------------------------------------------------------------------------
 * Every trap stores the whole integer register file on the interrupted
 * thread's own stack, 32 words, laid out by register number so the slot of x_n
 * is word n. x0 is always zero and x2 is the frame's own address plus its
 * size, so their slots carry mepc and mstatus instead. A suspended thread is
 * therefore nothing but a pointer to one of these, which is what lets a context
 * switch be a single store, as it is on ARM.
 * ========================================================================= */

inline constexpr std::uint32_t frame_words   = 32u;
inline constexpr std::uint32_t frame_bytes   = frame_words * 4u;
inline constexpr std::uint32_t frame_mepc    = 0u;   /* x0's slot */
inline constexpr std::uint32_t frame_mstatus = 2u;   /* x2's slot */
inline constexpr std::uint32_t frame_ra      = 1u;
inline constexpr std::uint32_t frame_gp      = 3u;
inline constexpr std::uint32_t frame_tp      = 4u;
inline constexpr std::uint32_t frame_a0      = 10u;

static_assert(frame_bytes % 16u == 0u, "the psABI keeps sp 16-aligned, and the frame must too");

/* ============================================================================
 * Semihosting
 * ----------------------------------------------------------------------------
 * For the panic report only. The RISC-V semihosting trap is an ebreak framed
 * by two marker instructions, uncompressed and in one page, which the
 * alignment guarantees. Operation numbers are ARM's.
 * ========================================================================= */

[[gnu::noinline]] inline long semihost(long op, void volatile* arg) noexcept
{
   register long a0 asm("a0") = op;
   register void volatile* a1 asm("a1") = arg;
   asm volatile(
      ".option push         \n"
      ".option norvc        \n"
      ".p2align 4           \n"
      "slli x0, x0, 0x1f    \n"
      "ebreak               \n"
      "srai x0, x0, 7       \n"
      ".option pop          \n"
      : "+r"(a0)
      : "r"(a1)
      : "memory");
   return a0;
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
 * @brief This hart's mcycle, 64 bits read in two halves: high, low, high
 *        again, retried on a carry between them. Counts only once
 *        init_this_core has cleared mcountinhibit.
 */
inline std::uint64_t read_mcycle() noexcept
{
   std::uint32_t high, low, again;
   do {
      asm volatile("csrr %0, mcycleh" : "=r"(high));
      asm volatile("csrr %0, mcycle"  : "=r"(low));
      asm volatile("csrr %0, mcycleh" : "=r"(again));
   } while (high != again);
   return (static_cast<std::uint64_t>(high) << 32) | low;
}

/**
 * @brief Per-core trap vector, masks and soft IRQ, on the calling core.
 *
 * cyros_port_init runs it on the bootstrap core. A multicore target runs it on
 * every other core before that core enters the kernel.
 */
void init_this_core();

/* ============================================================================
 * What the core layer needs from its TARGET
 * ----------------------------------------------------------------------------
 * Declared here, defined by the target. Not part of port_mcu.h, because they
 * are internal to the RISC-V tree: no other port has a soft IRQ to raise.
 * ========================================================================= */

/** Raise hart `core`'s machine software interrupt. Its own, or another's. */
void soft_irq_raise(std::uint32_t core) noexcept;

/** Lower the calling hart's machine software interrupt. */
void soft_irq_clear(std::uint32_t core) noexcept;

/** The body of the machine timer interrupt. Defined by the target, which
 * hands it to its timer (src/port/common/mtime.hpp on every target here). */
void timer_interrupt() noexcept;

/** The body of the machine external interrupt. Defined by the target. */
void external_interrupt() noexcept;

/**
 * @brief The counter behind cyros_port_timestamp.
 *
 * The target's choice, because only it knows whether a per-hart cycle count
 * is enough or stamps must compare across harts. read_mcycle() below is the
 * per-hart answer. The RP2350 answers with its shared MTIME at the core clock
 * (rp2350-notes.md 10).
 */
std::uint64_t timestamp() noexcept;

/**
 * @brief Make PMP entry 0 bind machine mode on the calling hart, and return
 *        the configuration byte entry 0 then needs to deny every access to a
 *        NAPOT region. 0 means this target cannot: no stack guard.
 *
 * A standard PMP entry binds machine mode only when it is locked, and a locked
 * entry cannot move, so it cannot follow the running thread. Hazard3's Xh3pmpm
 * applies an UNLOCKED entry to machine mode (PMPCFGM0), and Smepmp's
 * mseccfg.RLB lets a locked one be rewritten, which is what QEMU's virt has.
 * Called on every hart before its first thread.
 */
std::uint8_t stack_guard_setup() noexcept;

} // namespace cyros::port::riscv

/**
 * @brief Every exception that is not a yield: a fault. WEAK, so the
 *        application, or a test, may replace it, as an ARM application owns
 *        its fault vectors.
 *
 * Called inside the trap with interrupts masked, on the hart's interrupt
 * stack, with the faulting thread's saved frame (riscv.hpp's layout, mepc at
 * frame[frame_mepc]). Returning resumes the frame, at whatever mepc it then
 * holds. The default never returns: it reports and panics, and says when the
 * fault hit the trap entry itself saving a frame, which is a stack overflow
 * into the guard.
 */
extern "C" void cyros_riscv_fault_handler(std::uint32_t mcause, std::uint32_t* frame);

#endif /* CYROS_PORT_RISCV_HPP */
