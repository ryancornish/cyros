/**
 * @file port_mcu_cortex_r52.cpp
 * @brief The Cortex-R52 target: identity, bring-up, the IPI, and the generic
 *        timer presented as MTIME.
 *
 * Compiled by two targets that differ only in port_traits.h: `cortex_r52`, one
 * core, and `cortex_r52_smp`, CYROS_PORT_CORE_COUNT cores. Almost everything a
 * target supplies elsewhere is the core's on the R52: the GIC is inside it and
 * the timer is architectural. What is left is how the other cores start, which
 * is the board's park and this file's release.
 *
 *
 * How the other cores start
 * =========================
 * QEMU starts every R52 at the image's entry, in EL2. The BOARD drops each to
 * EL1, gives it its stacks and parks every core but 0 in WFE until
 * `cyros_armv8r_release` is set. start_cores sets it and sends an event, and
 * the board hands the core to cyros_port_secondary_core_entry below. That is
 * the RISC-V virt park with a memory word in place of the soft IRQ, because
 * waking on an SGI would need the GIC set up on a core the port has not
 * reached yet.
 *
 *
 * The generic timer as MTIME
 * ==========================
 * src/port/common/mtime.hpp wants one 64-bit count every core reads and a
 * 64-bit compare per core whose interrupt is level while the count has reached
 * it. The EL1 physical timer is exactly that: CNTPCT, and each core's own
 * CNTP_CVAL, raising PPI 30 while CNTPCT >= CVAL. Both are single 64-bit
 * accesses here, so neither the read nor the compare can tear, which is the
 * hazard the RISC-V and RP2350 targets order their halves against.
 */

#include <cyros/port/port_mcu.h>

#include "armv8r.hpp"
#include "mtime.hpp"

#include <cstddef>
#include <cstdint>

namespace armv8r = cyros::port::armv8r;
namespace mtime  = cyros::port::mtime;

/* The word the board's park polls. Zero until start_cores releases the other
 * cores. In .bss, which core 0 zeroes while the others read it, harmlessly. */
extern "C" { volatile std::uint32_t cyros_armv8r_release = 0u; }

namespace
{

/* The entry every core runs, stashed by start_cores for the other cores.
 * Written by core 0 before any is released, read-only after. */
cyros_port_core_entry_t core_entry = nullptr;

} // namespace


/* ============================================================================
 * SMP & Multi-Core Support
 * ========================================================================= */

std::uint32_t cyros_port_get_core_id(void)
{
   return armv8r::core_number();
}

void cyros_port_start_cores(std::size_t cores_to_use, cyros_port_core_entry_t entry)
{
   CYROS_ASSERT_OP(cores_to_use, >=, 1u);
   CYROS_ASSERT_OP(cores_to_use, <=, static_cast<std::size_t>(CYROS_PORT_CORE_COUNT));
   CYROS_ASSERT(entry != nullptr);
   /* Only core 0 runs bring-up. The others are parked by the board. */
   CYROS_ASSERT_OP(cyros_port_get_core_id(), ==, 0u);

   core_entry = entry;
   if (cores_to_use > 1u) {
      armv8r::dsb();
      cyros_armv8r_release = 1u;
      armv8r::dsb();
      asm volatile("sev" ::: "memory");
   }

   /* Core 0 runs the same entry as every other. */
   entry();
   CYROS_PORT_UNREACHABLE();
}

/**
 * @brief Where a released core joins the kernel. Called by the board's park,
 *        in System mode on that core's own stack.
 *
 * init_this_core is the part that matters: the redistributor, the CPU
 * interface, the masks and the timer are all per core.
 */
extern "C" [[noreturn]] void cyros_port_secondary_core_entry(void)
{
   armv8r::init_this_core();
   CYROS_ASSERT(core_entry != nullptr);
   core_entry();
   CYROS_PORT_UNREACHABLE();
}

void cyros_port_send_reschedule_ipi(std::uint32_t core_id)
{
   /* The cross-core and the local request are ONE mechanism, SGI 0, as on
    * RISC-V. raise_reschedule orders this core's writes before it. */
   CYROS_ASSERT_OP(core_id, <, static_cast<std::uint32_t>(CYROS_PORT_CORE_COUNT));
   armv8r::raise_reschedule(core_id);
}


/* ============================================================================
 * What the armv8r core layer needs (armv8r.hpp)
 * ========================================================================= */

namespace cyros::port::armv8r
{

void timer_interrupt() noexcept
{
   mtime::interrupt();
}

} // namespace cyros::port::armv8r


/* ============================================================================
 * The MTIME timer (../../common/mtime.hpp), on the generic timer
 * ========================================================================= */

namespace cyros::port::mtime
{

std::uint64_t read() noexcept
{
   return armv8r::read_cntpct();
}

void compare_write(std::uint32_t core, std::uint64_t value) noexcept
{
   /* CNTP_CVAL is a system register, so only a core's own is reachable. The
    * time source only ever writes the calling core's. */
   CYROS_ASSERT_OP(core, ==, armv8r::core_number());
   armv8r::write_cntp_cval(value);
   armv8r::isb();
}

void interrupt_enable() noexcept
{
   armv8r::write_cntp_ctl(armv8r::cntp_ctl_enable);
   armv8r::isb();
}

void interrupt_disable() noexcept
{
   armv8r::write_cntp_ctl(armv8r::cntp_ctl_imask);
   armv8r::isb();
}

} // namespace cyros::port::mtime
