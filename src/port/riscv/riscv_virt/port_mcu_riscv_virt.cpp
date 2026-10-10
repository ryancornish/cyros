/**
 * @file port_mcu_riscv_virt.cpp
 * @brief QEMU's RISC-V `virt`: identity, bring-up, the IPI, and the CLINT
 *        registers the core layer and the time source use.
 *
 * Compiled by two targets that differ only in port_traits.h: `riscv_virt`,
 * one hart, and `riscv_virt_smp`, CYROS_PORT_CORE_COUNT harts.
 *
 * The CLINT is the SiFive-style core-local interruptor QEMU puts at 0x2000000
 * (its device tree says so): a 32-bit `msip` word per hart that raises that
 * hart's software interrupt, a 64-bit MTIMECMP per hart, and the shared
 * 64-bit MTIME. It is what the core layer cannot reach on its own, because
 * RISC-V puts the software interrupt outside the core.
 *
 * External interrupts would come through the PLIC. Nothing in cyros or its
 * tests enables one, so an external interrupt here is a fault.
 *
 *
 * How the other harts start
 * =========================
 * `-bios none` starts every hart at reset, so the BOARD parks the others: each
 * on its own stack, interrupts masked, mie.MSIE set, waiting in WFI for its
 * soft IRQ. start_cores raises it, and the board hands the hart to
 * cyros_port_secondary_core_entry below. That is the SSE-200's CPUWAIT done in
 * software, and it keeps the same split: the stack and the park are board
 * knowledge, the release and the entry are the port's.
 */

#include <cyros/port/port_mcu.h>

#include "mtime.hpp"
#include "riscv.hpp"

#include <cstddef>
#include <cstdint>

namespace riscv = cyros::port::riscv;
namespace mtime = cyros::port::mtime;

namespace
{

constexpr std::uintptr_t clint_base     = 0x02000000u;
constexpr std::uintptr_t clint_msip     = clint_base + 0x0000u;  /* + 4 per hart */
constexpr std::uintptr_t clint_mtimecmp = clint_base + 0x4000u;  /* + 8 per hart */
constexpr std::uintptr_t clint_mtime    = clint_base + 0xBFF8u;

inline volatile std::uint32_t& reg(std::uintptr_t address) noexcept
{
   return *reinterpret_cast<volatile std::uint32_t*>(address);
}

/* A read whose value is not wanted. `(void)reg(a)` would not do: a discarded
 * call returning a volatile reference is never converted to a value, so the
 * load is not emitted (measured: soft_irq_raise had no read-back). */
inline std::uint32_t load(std::uintptr_t address) noexcept
{
   return reg(address);
}

/* The entry every core runs, stashed by start_cores for the other harts.
 * Written by hart 0 before any is released, read-only after. */
cyros_port_core_entry_t core_entry = nullptr;

/* Everything this hart wrote to memory is visible to another before the
 * store that interrupts it. */
inline void fence_before_ipi() noexcept
{
   asm volatile("fence rw, ow" ::: "memory");
}

} // namespace


/* ============================================================================
 * SMP & Multi-Core Support
 * ========================================================================= */

std::uint32_t cyros_port_get_core_id(void)
{
   return riscv::read_mhartid();
}

void cyros_port_start_cores(std::size_t cores_to_use, cyros_port_core_entry_t entry)
{
   CYROS_ASSERT_OP(cores_to_use, >=, 1u);
   CYROS_ASSERT_OP(cores_to_use, <=, static_cast<std::size_t>(CYROS_PORT_CORE_COUNT));
   CYROS_ASSERT(entry != nullptr);
   /* Only hart 0 runs bring-up. The others are parked by the board. */
   CYROS_ASSERT_OP(cyros_port_get_core_id(), ==, 0u);

   core_entry = entry;
   fence_before_ipi();
   for (std::uint32_t core = 1u; core < cores_to_use; ++core) {
      riscv::soft_irq_raise(core);
   }

   /* Hart 0 runs the same entry as every other. */
   entry();
   CYROS_PORT_UNREACHABLE();
}

/**
 * @brief Where a released hart joins the kernel. Called by the board's park.
 *
 * init_this_core is the part that matters: mtvec, mscratch, the masks and the
 * soft IRQ are all per hart. It also lowers the soft IRQ that released this
 * hart, so the release is not taken as the first reschedule.
 */
extern "C" [[noreturn]] void cyros_port_secondary_core_entry(void)
{
   riscv::init_this_core();
   CYROS_ASSERT(core_entry != nullptr);
   core_entry();
   CYROS_PORT_UNREACHABLE();
}

void cyros_port_send_reschedule_ipi(std::uint32_t core_id)
{
   /* On RISC-V the cross-core and the local request are ONE mechanism: raise
    * the target hart's software interrupt. The target reads scheduler state
    * as soon as it takes it, so this hart's writes go first. */
   CYROS_ASSERT_OP(core_id, <, static_cast<std::uint32_t>(CYROS_PORT_CORE_COUNT));
   fence_before_ipi();
   riscv::soft_irq_raise(core_id);
}


/* ============================================================================
 * What the rv32 core layer needs (riscv.hpp)
 * ========================================================================= */

namespace cyros::port::riscv
{

void soft_irq_raise(std::uint32_t core) noexcept
{
   reg(clint_msip + 4u * core) = 1u;
   /* Read back, so the store has reached the CLINT before this returns. */
   (void)load(clint_msip + 4u * core);
}

void soft_irq_clear(std::uint32_t core) noexcept
{
   reg(clint_msip + 4u * core) = 0u;
   (void)load(clint_msip + 4u * core);
}

void external_interrupt() noexcept
{
   cyros_port_system_error(read_mcause(), read_mepc(), "external interrupt with no handler", 0);
}

std::uint64_t timestamp() noexcept
{
   /* Per-hart cycles, as ARM's DWT: stamps from two harts do not compare.
    * QEMU's mcycle follows host time rather than instructions, so its figures
    * are only ever relative. MTIME's 10 MHz would be too coarse to stamp
    * anything. */
   return read_mcycle();
}

void timer_interrupt() noexcept
{
   mtime::interrupt();
}

std::uint8_t stack_guard_setup() noexcept
{
   /* Smepmp, which the toolchain's runner gives QEMU (-cpu rv32,smepmp=on):
    * mseccfg.RLB lets a LOCKED entry be rewritten, and a locked entry binds
    * machine mode, so the guard is a locked entry that still moves. RLB is
    * set before any entry is locked, as the extension requires. Without
    * Smepmp this CSR access traps. */
   constexpr std::uint32_t mseccfg_rlb = 1u << 2;
   asm volatile("csrs 0x747, %0" : : "r"(mseccfg_rlb) : "memory");
   return pmpcfg_lock | pmpcfg_napot;
}

} // namespace cyros::port::riscv


/* ============================================================================
 * The MTIME timer (../../common/mtime.hpp), in the CLINT
 * ========================================================================= */

namespace cyros::port::mtime
{

std::uint64_t read() noexcept
{
   std::uint32_t high, low, again;
   do {
      high  = reg(clint_mtime + 4u);
      low   = reg(clint_mtime);
      again = reg(clint_mtime + 4u);
   } while (high != again);
   return (static_cast<std::uint64_t>(high) << 32) | low;
}

void compare_write(std::uint32_t core, std::uint64_t value) noexcept
{
   std::uintptr_t const compare = clint_mtimecmp + 8u * core;
   reg(compare)      = ~0u;
   reg(compare + 4u) = static_cast<std::uint32_t>(value >> 32);
   reg(compare)      = static_cast<std::uint32_t>(value);
}

void interrupt_enable() noexcept
{
   riscv::set_mie(riscv::mie_mtie);
}

void interrupt_disable() noexcept
{
   riscv::clear_mie(riscv::mie_mtie);
}

} // namespace cyros::port::mtime
