/**
 * @file port_mcu_cortex_m33_smp.cpp
 * @brief Dual Cortex-M33 on an SSE-200 subsystem: identity, bring-up, the IPI.
 *
 * The SMP sibling of port_mcu_cortex_m33.cpp. Both extend `armv8m` and share
 * every line of the core contract with it, which is the claim the split of
 * port_core.h from port_mcu.h was made to support: two targets whose processor
 * core is identical and whose MCU is not.
 *
 * Bench is QEMU's mps2-an521. The SSE-200 is also what a Musca board and an
 * IoT Kit part are built from, and the three facts this file depends on are
 * subsystem-level rather than board-level, so another SSE-200 part should need
 * only different base addresses.
 *
 *
 * Which core am I
 * ===============
 * The SSE-200 gives each core a CPU identity block whose one register reads
 * that core's number. The SAME ADDRESS answers differently per core, which is
 * what makes it usable from shared code with no per-core setup at all. There
 * is no architectural equivalent on ARMv8-M, and the SCB CPUID register is the
 * trap to avoid: it reports the part (0x410FD213 for an M33 r0p4) and is
 * therefore identical on both cores.
 *
 *
 * How the second core starts
 * ==========================
 * CPU1 comes out of reset held, with bit 1 of CPUWAIT set. CPU0 gives it a
 * vector table through INITSVTOR1 and then clears CPUWAIT. CPU1 then behaves
 * exactly like a core coming out of reset: it loads its initial MSP from word
 * 0 of that table and jumps to word 1.
 *
 * The vector table is supplied by the APPLICATION, not by cyros, for the same
 * reason the primary core's is: a vector table is board knowledge. cyros
 * declares what it needs and the bench or the product provides it. See
 * cyros_port_cpu1_vector_table below.
 *
 *
 * How one core interrupts another
 * ===============================
 * The SSE-200's MHU is a doorbell: writing a set register raises an interrupt
 * on the other core, which clears it by writing a clear register. MHU0 serves
 * BOTH directions, because it has a separate register pair per target core,
 * and measurement confirms the interrupt arrives as device IRQ 6 on whichever
 * core is targeted. One doorbell, one IRQ number, the target chosen by which
 * register is written.
 */

#include <cyros/port/port_mcu.h>

#include "cortex_m.hpp"

#include <cstddef>
#include <cstdint>

namespace cortex_m = cyros::port::cortex_m;

namespace
{

/* --- SSE-200 system control block ------------------------------------- */

constexpr std::uintptr_t sysctrl_base = 0x50021000u;
constexpr std::uintptr_t initsvtor1   = sysctrl_base + 0x114u;  /* CPU1's vector table */
constexpr std::uintptr_t cpuwait      = sysctrl_base + 0x118u;  /* bit N holds core N  */

/* --- The CPU identity block ------------------------------------------- */

constexpr std::uintptr_t cpuid_block = 0x5001F000u;

/* --- MHU0, the inter-core doorbell ------------------------------------ */

constexpr std::uintptr_t mhu0_base = 0x50003000u;

/* Per-target register pairs, 0x10 apart. Targeting core N means writing N's
 * set register, so one MHU covers both directions. */
constexpr std::uintptr_t mhu_set(std::uint32_t core)
{
   return mhu0_base + 0x004u + (static_cast<std::uintptr_t>(core) * 0x10u);
}

constexpr std::uintptr_t mhu_clear(std::uint32_t core)
{
   return mhu0_base + 0x008u + (static_cast<std::uintptr_t>(core) * 0x10u);
}

/* Measured on mps2-an521: an MHU0 doorbell arrives as device IRQ 6 on the
 * targeted core, in both directions. */
constexpr std::uint32_t mhu_irq = 6u;

/* --- NVIC ------------------------------------------------------------- */

constexpr std::uintptr_t nvic_iser0 = 0xE000E100u;
constexpr std::uintptr_t nvic_ipr0  = 0xE000E400u;   /* one BYTE per IRQ */

/* The entry point every core runs, stashed by start_cores so that the
 * secondary core's entry path can reach it. Written by core 0 strictly before
 * CPU1 is released, and read-only thereafter. */
cyros_port_core_entry_t core_entry = nullptr;

/**
 * @brief Route the doorbell to this core and make it serviceable.
 *
 * Priority matters more than it looks. The handler's whole job is to pend a
 * reschedule, and a reschedule requested while the target core sits in a
 * kernel critical section must still be TAKEN, which means the IRQ may not be
 * maskable by the preempt-disable. device_irq_priority() is the core layer's
 * answer to exactly that question.
 */
void enable_doorbell_on_this_core()
{
   cortex_m::reg8(nvic_ipr0 + mhu_irq) =
      static_cast<std::uint8_t>(cortex_m::device_irq_priority());

   /* Clear anything left pending from a previous kernel lifecycle before
    * unmasking, or the first enable takes an interrupt that means nothing. */
   cortex_m::reg(mhu_clear(cyros_port_get_core_id())) = 1u;

   cortex_m::reg(nvic_iser0) = 1u << mhu_irq;

   cortex_m::dsb();
   cortex_m::isb();
}

} // namespace


/* ============================================================================
 * Board-supplied
 * ========================================================================= */

/**
 * @brief CPU1's vector table, supplied by the application.
 *
 * Word 0 is CPU1's initial MSP and word 1 is where it starts, exactly as for
 * the primary core. ARMv8-M requires the table to be aligned to its own size
 * rounded up to a power of two, minimum 128 bytes.
 *
 * That start address must reach cyros_port_secondary_core_entry below, after
 * doing whatever board-level work the part needs. cyros declares this and
 * never defines it, which is what keeps libcyros.a free of any vector table.
 */
extern "C" void const* cyros_port_cpu1_vector_table(void);


/* ============================================================================
 * SMP & Multi-Core Support
 * ========================================================================= */

std::uint32_t cyros_port_get_core_id(void)
{
   return cortex_m::reg(cpuid_block);
}

/**
 * @brief Where a released secondary core joins the kernel.
 *
 * Called by the application's CPU1 reset handler. Everything before this is
 * board work; everything after is the kernel running on a second core.
 *
 * init_this_core() is the non-negotiable part. Every register it touches is
 * private to a core, so a secondary core that skipped it would run the same
 * kernel with default system-handler priorities and a disabled FPU. The
 * failure would not be a fault at this point, it would be PendSV at the wrong
 * priority and a context switch taken somewhere it must not be.
 */
extern "C" [[noreturn]] void cyros_port_secondary_core_entry(void)
{
   cortex_m::init_this_core();
   enable_doorbell_on_this_core();

   CYROS_ASSERT(core_entry != nullptr);
   core_entry();

   /* core_entry reaches cyros_port_start_first, which never returns. */
   CYROS_PORT_UNREACHABLE();
}

void cyros_port_start_cores(std::size_t cores_to_use, cyros_port_core_entry_t entry)
{
   CYROS_ASSERT_OP(cores_to_use, >=, 1u);
   CYROS_ASSERT_OP(cores_to_use, <=, static_cast<std::size_t>(CYROS_PORT_CORE_COUNT));
   CYROS_ASSERT(entry != nullptr);

   /* Only core 0 runs bring-up, and it is the only core executing at all until
    * CPUWAIT is cleared below. */
   CYROS_ASSERT_OP(cyros_port_get_core_id(), ==, 0u);

   core_entry = entry;

   enable_doorbell_on_this_core();

   if (cores_to_use > 1u) {
      void const* const table = cyros_port_cpu1_vector_table();
      CYROS_ASSERT(table != nullptr);

      cortex_m::reg(initsvtor1) = static_cast<std::uint32_t>(
         reinterpret_cast<std::uintptr_t>(table));

      /* The release must not be observed before the vector table it depends
       * on. Without this CPU1 can come out of reset against a stale
       * INITSVTOR1 and fetch its MSP from whatever was there. */
      cortex_m::dsb();

      cortex_m::reg(cpuwait) = 0u;
   }

   /* Core 0 runs the same entry as every other core. */
   entry();

   CYROS_PORT_UNREACHABLE();
}

void cyros_port_send_reschedule_ipi(std::uint32_t core_id)
{
   CYROS_ASSERT_OP(core_id, <, static_cast<std::uint32_t>(CYROS_PORT_CORE_COUNT));

   if (core_id == cyros_port_get_core_id()) {
      /* A core asking itself to reschedule does not need the doorbell, and
       * ringing it would take a pointless interrupt to reach the same PendSV.
       * This carries the same weak guarantee as the cross-core path. */
      cyros_port_pend_reschedule();
      return;
   }

   /* The receiving core reads shared scheduler state the moment it takes the
    * interrupt, so everything this core wrote before asking must be visible
    * first. */
   cortex_m::dsb();

   cortex_m::reg(mhu_set(core_id)) = 1u;
}

/**
 * @brief The doorbell handler. Routed from the application's vector table.
 *
 * Named like PendSV_Handler and SysTick_Handler, and for the same reason: the
 * application's vector table has to be able to name it. On this subsystem one
 * IRQ serves both directions, so both cores install the same handler and each
 * clears its own doorbell.
 *
 * Pending a reschedule is the whole job. PendSV is the lowest priority
 * exception in the system, so the switch happens when this handler and any
 * other active handler have unwound, which is exactly the guarantee
 * cyros_port_send_reschedule_ipi documents.
 */
extern "C" void MHU_Handler(void)
{
   cortex_m::reg(mhu_clear(cyros_port_get_core_id())) = 1u;

   /* Clear before pending, and make the clear visible before returning, or the
    * doorbell re-fires the instant this handler exits. */
   cortex_m::dsb();

   cyros_port_pend_reschedule();
}
