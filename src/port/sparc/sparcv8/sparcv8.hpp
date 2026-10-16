/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file sparcv8.hpp
 * @brief The sparcv8 core layer's internal header: the processor state
 *        register, the trap frame, the register windows, and what the layer
 *        needs from its target.
 *
 * SPARC V8 in supervisor mode with LEON3's extensions: ASR17 (the core's
 * index and its window count), CASA (which GCC emits for compare-exchange) and
 * power-down (ASR19). LEON4 and LEON5 have all three. What is NOT here is the
 * system around the core: GRLIB's IRQMP, GPTIMER and APBUART are peripherals,
 * so they are the target's, as the RP2040's TIMER is. The layer cannot even
 * raise its own reschedule without one, because SPARC V8 has no
 * self-interrupt, which is RISC-V's position with its soft IRQ.
 *
 * ~/cyros-claude/sparc-port-plan.md has the measurements behind every number.
 */
#ifndef CYROS_PORT_SPARCV8_HPP
#define CYROS_PORT_SPARCV8_HPP

#include <cyros/port/port_traits.h>

#include <cstdint>

#if !defined(__sparc__) || defined(__sparc_v9__) || defined(__arch64__)
#  error "the sparcv8 core layer is for 32-bit SPARC V8 (-mcpu=leon3)"
#endif

/* The register window count the trap handlers are assembled for. LEON3 is
 * built with 2 to 32 windows, and every part and FPGA template in use has 8,
 * as QEMU's LEON3 does. cyros_port_init checks it against ASR17. */
#ifndef CYROS_SPARC_NWINDOWS
#define CYROS_SPARC_NWINDOWS 8
#endif

/* ============================================================================
 * The trap frame
 * ----------------------------------------------------------------------------
 * Every trap that can switch (an interrupt, the yield, a fault) stores the
 * interrupted thread's state on that thread's OWN stack, below its sp, where
 * the SPARC ABI keeps nothing. A suspended thread is a pointer to it, as on
 * RISC-V and the R52.
 *
 * What it holds is what the register windows do not: the globals, Y, the
 * PSR, PC and nPC, and the interrupted window's OUTS, which are the trap
 * window's ins. The interrupted window's locals and ins, and every caller's
 * window above it, are not in the frame. They live in the register file, or
 * in each window's 64-byte save area at its own sp once flushed, which is
 * where the ABI already reserves room for them. A switch flushes them there.
 *
 *   frame + 0    PSR, PC          (one std from the trap window's l0, l1)
 *   frame + 8    nPC, Y
 *   frame + 16   (unused), g1
 *   frame + 24   g2, g3, g4, g5, g6, g7 (pairs, for ldd and std)
 *   frame + 48   o0 to o7 of the interrupted window (o6 is its sp)
 *   frame + 80   FSR, pad, f0 to f31   (hard float only)
 *
 * The offsets are macros because the assembly is a string: each is checked
 * against the C++ view below.
 * ========================================================================= */

#define CYROS_SPARC_F_PSR    0
#define CYROS_SPARC_F_PC     4
#define CYROS_SPARC_F_NPC    8
#define CYROS_SPARC_F_Y      12
#define CYROS_SPARC_F_G0     16
#define CYROS_SPARC_F_G1     20
#define CYROS_SPARC_F_G2     24
#define CYROS_SPARC_F_G4     32
#define CYROS_SPARC_F_G6     40
#define CYROS_SPARC_F_O0     48
#define CYROS_SPARC_F_O2     56
#define CYROS_SPARC_F_O4     64
#define CYROS_SPARC_F_O6     72
#define CYROS_SPARC_F_INT_BYTES 80

#if !defined(_SOFT_FLOAT)
#define CYROS_SPARC_HAS_FPU  1
#define CYROS_SPARC_F_FSR    80
#define CYROS_SPARC_F_F0     88
#define CYROS_SPARC_F_BYTES  216
#else
#define CYROS_SPARC_HAS_FPU  0
#define CYROS_SPARC_F_BYTES  80
#endif

/* Per-core state the assembly reads: four words a core, so the entry indexes
 * it with one shift. */
#define CYROS_SPARC_CORE_FRAME       0
#define CYROS_SPARC_CORE_IRQ_STACK   4
#define CYROS_SPARC_CORE_DEPTH       8
#define CYROS_SPARC_CORE_STRIDE_LOG2 4

/* The software traps (tt 0x80 + n): GCC's own, and the one cyros takes for the
 * yield from the range the SPARC ABI leaves to the operating system. */
#define CYROS_SPARC_TA_HALT          0   /* ta 0 with traps off: QEMU's shutdown */
#define CYROS_SPARC_TA_BREAKPOINT    1
#define CYROS_SPARC_TA_FLUSH_WINDOWS 3   /* GCC: __builtin_flush_register_windows */
#define CYROS_SPARC_TA_TRAP          5   /* GCC: __builtin_trap                   */
#define CYROS_SPARC_TA_YIELD         9

namespace cyros::port::sparcv8
{

inline constexpr std::uint32_t nwindows = CYROS_SPARC_NWINDOWS;

inline constexpr std::uint32_t frame_bytes = CYROS_SPARC_F_BYTES;
inline constexpr std::uint32_t frame_words = frame_bytes / 4u;
inline constexpr std::uint32_t frame_psr   = CYROS_SPARC_F_PSR / 4;
inline constexpr std::uint32_t frame_pc    = CYROS_SPARC_F_PC / 4;
inline constexpr std::uint32_t frame_npc   = CYROS_SPARC_F_NPC / 4;
inline constexpr std::uint32_t frame_y     = CYROS_SPARC_F_Y / 4;
inline constexpr std::uint32_t frame_g1    = CYROS_SPARC_F_G1 / 4;   /* g1 to g7 follow */
inline constexpr std::uint32_t frame_o0    = CYROS_SPARC_F_O0 / 4;   /* o0 to o7 follow */
inline constexpr std::uint32_t frame_o6    = CYROS_SPARC_F_O6 / 4;
#if CYROS_SPARC_HAS_FPU
inline constexpr std::uint32_t frame_fsr   = CYROS_SPARC_F_FSR / 4;
inline constexpr std::uint32_t frame_f0    = CYROS_SPARC_F_F0 / 4;   /* f0 to f31 follow */
#endif

/* The window save area the ABI reserves at every sp, and the minimum frame a
 * C call needs (save area, hidden struct-return word, six argument words). */
inline constexpr std::uint32_t window_save_bytes = 64u;
inline constexpr std::uint32_t min_frame_bytes   = 96u;

static_assert(frame_bytes % 8u == 0u, "the ABI keeps sp 8-aligned, and the frame must keep it");
static_assert(CYROS_SPARC_F_G2 == CYROS_SPARC_F_G1 + 4 && CYROS_SPARC_F_G4 == CYROS_SPARC_F_G2 + 8
              && CYROS_SPARC_F_G6 == CYROS_SPARC_F_G4 + 8 && CYROS_SPARC_F_O0 == CYROS_SPARC_F_G6 + 8,
              "the frame's globals are consecutive");
static_assert(CYROS_SPARC_F_INT_BYTES == CYROS_SPARC_F_O6 + 8, "o7 is the integer frame's last word");

/* ============================================================================
 * The processor state register
 * ========================================================================= */

inline constexpr std::uint32_t psr_cwp_mask = 0x1Fu;
inline constexpr std::uint32_t psr_et       = 1u << 5;
inline constexpr std::uint32_t psr_ps       = 1u << 6;
inline constexpr std::uint32_t psr_s        = 1u << 7;
inline constexpr std::uint32_t psr_pil_shift = 8u;
inline constexpr std::uint32_t psr_pil_mask = 0xFu << psr_pil_shift;
inline constexpr std::uint32_t psr_ef       = 1u << 12;

/* The PIL values the masks use (port_core_sparcv8.cpp, "Masking"). 14 and 15
 * mask the same interrupts, every level but the non-maskable 15, so the pair
 * carries whether preemption is also off. */
inline constexpr std::uint32_t pil_open           = 0u;
inline constexpr std::uint32_t pil_preempt        = 1u;
inline constexpr std::uint32_t pil_masked         = 14u;
inline constexpr std::uint32_t pil_masked_preempt = 15u;

/* The interrupt level of the reschedule, local or from another core. The
 * lowest, so a preempt-disable (PIL 1) masks it and nothing else. */
inline constexpr std::uint32_t reschedule_level = 1u;

/* Trap types. */
inline constexpr std::uint32_t tt_interrupt_first = 0x11u;
inline constexpr std::uint32_t tt_interrupt_last  = 0x1Fu;
inline constexpr std::uint32_t tt_software        = 0x80u;
inline constexpr std::uint32_t tt_yield           = tt_software + CYROS_SPARC_TA_YIELD;

inline std::uint32_t read_psr() noexcept
{
   std::uint32_t value;
   asm volatile("rd %%psr, %0" : "=r"(value) : : "memory");
   return value;
}

/* WRPSR is delayed: the three instructions after it may still see the old
 * PSR. A raised PIL must be in force before the region it protects, and a
 * lowered one must have let a pending interrupt in before the caller goes on,
 * so the nops are part of the write. */
inline void write_psr(std::uint32_t value) noexcept
{
   asm volatile("wr %0, %%psr \n nop \n nop \n nop" : : "r"(value) : "memory", "cc");
}

inline std::uint32_t read_wim() noexcept
{
   std::uint32_t value;
   asm volatile("rd %%wim, %0" : "=r"(value) : : "memory");
   return value;
}

inline std::uint32_t read_tbr() noexcept
{
   std::uint32_t value;
   asm volatile("rd %%tbr, %0" : "=r"(value) : : "memory");
   return value;
}

/** LEON3's configuration register: the core's index in bits 31 to 28 and its
 * window count less one in bits 4 to 0. */
inline std::uint32_t read_asr17() noexcept
{
   std::uint32_t value;
   asm volatile("rd %%asr17, %0" : "=r"(value) : : "memory");
   return value;
}

inline std::uint32_t pil() noexcept
{
   return (read_psr() & psr_pil_mask) >> psr_pil_shift;
}

/** This core's index among the LEON3s on its bus: ASR17.INDEX. */
inline std::uint32_t core_number() noexcept
{
   return read_asr17() >> 28;
}

/* ============================================================================
 * The core layer's own functions that a target calls
 * ========================================================================= */

/**
 * @brief Per-core bring-up: masks, the trap table check, this core's
 *        reschedule routing, and its trap state.
 *
 * cyros_port_init runs it on the bootstrap core. A multicore target runs it on
 * every other core before that core enters the kernel.
 */
void init_this_core();

/* ============================================================================
 * What the core layer needs from its TARGET
 * ----------------------------------------------------------------------------
 * Declared here, defined by the target, because SPARC V8 puts every interrupt
 * source outside the core: there is no way to raise one's own reschedule
 * without the interrupt controller.
 * ========================================================================= */

/** Raise the reschedule level on core `core`, its own or another's. */
void reschedule_raise(std::uint32_t core) noexcept;

/**
 * @brief Route the reschedule to the calling core, and drop one a previous
 *        kernel lifecycle left pending, which would otherwise fire into the
 *        next lifecycle's first thread.
 */
void reschedule_setup(std::uint32_t core) noexcept;

/** Every interrupt level but the reschedule's, on the calling core, inside
 * the trap with interrupts masked. The target's timer, or a device's. */
void interrupt(std::uint32_t level) noexcept;

/** The counter behind cyros_port_timestamp. */
std::uint64_t timestamp() noexcept;

/** Write text where a panic report can be read, with interrupts masked and
 * the kernel possibly broken. No allocation, no locks. */
void console_write(char const* text) noexcept;

} // namespace cyros::port::sparcv8

/**
 * @brief The trap table, 256 entries of four instructions, 4 KB aligned.
 *
 * The CORE LAYER's, because the window traps it must route are the ABI's,
 * not an application's: any C code may need them from its first call. So the
 * BOARD points TBR here before it calls any compiled code, and every core's
 * init checks it did.
 */
extern "C" std::uint32_t const cyros_sparc_trap_table[];

/**
 * @brief Every interrupt level that is neither the reschedule nor the
 *        target's own: a device's. WEAK, so the application supplies its
 *        devices' handlers, as it owns an M-profile vector table.
 *
 * Called inside the trap with interrupts masked, on the core's interrupt
 * stack. The default panics, naming the level.
 */
extern "C" void cyros_sparc_device_interrupt(std::uint32_t level);

/**
 * @brief Every trap that is not an interrupt, a window trap or the yield: a
 *        fault, or a software trap nothing claims. WEAK, so the application,
 *        or a test, may replace it.
 *
 * Called inside the trap with interrupts masked, on the core's interrupt
 * stack, with the trap type and the trapped thread's frame (the layout
 * above). Returning resumes the frame at its PC and nPC, so a handler that
 * wants to skip the trapping instruction moves them on itself. The default
 * reports and panics.
 */
extern "C" void cyros_sparc_fault_handler(std::uint32_t tt, std::uint32_t* frame);

#endif /* CYROS_PORT_SPARCV8_HPP */
