/**
 * @file port_mcu_rp2350_m33.cpp
 * @brief The RP2350's Cortex-M33 cores: identity, bring-up, the IPI, the
 *        MTIME timer and the timestamp.
 *
 * Compiled by two targets that differ only in port_traits.h: `rp2350_m33`,
 * core 0 alone, and `rp2350_m33_smp`, both cores. The core layer is
 * armv7m_armv8m, unchanged. The same chip's Hazard3 cores are the
 * rp2350_hazard3 targets, and the two share the time source
 * (../../common/port_time_mtime.cpp).
 *
 * Everything here is in the SIO, the single-cycle block each core sees at
 * 0xd0000000, and in the NVIC (rp2350-notes.md 1 and 9a):
 *
 *   CPUID           which core is reading: one address, a per-core answer
 *   DOORBELL_OUT    rings the OTHER core's doorbell, SIO_IRQ_BELL (IRQ 26)
 *   MTIME           one 64-bit counter both cores share
 *   MTIMECMP        THIS core's compare, raising SIO_IRQ_MTIMECMP (IRQ 29),
 *                   level. Each core sees its own at one address
 *
 * MTIME's rate and source (MTIME_CTRL.FULLSPEED) are the board's, as SysTick's
 * clock is on the cortex_m target. The board says the rate through
 * cyros_port_mtime_clock_hz.
 *
 * Erratum E2: writing 0 to DOORBELL_OUT_SET or DOORBELL_IN_CLR releases an SIO
 * spinlock (rp2350-notes.md 5d). cyros uses no SIO spinlock, an application
 * may, so every write below sets a bit.
 *
 *
 * Exclusives across the cores
 * ===========================
 * LDREX/STREX, and the LDAEX/STLEX GCC emits for std::atomic, are NOT atomic
 * across the two cores until ACTLR.EXTEXCLALL is set on each: 2 x 1,000,000
 * contended increments lost about 200,000, with no STREX failing
 * (rp2350-notes.md 3b). The boot ROM leaves it clear. Each core sets it here
 * before it touches anything the other core shares. It belongs in the target,
 * not the core layer: a part with no global monitor fails every external
 * exclusive with it set.
 *
 *
 * How core 1 starts
 * =================
 * Core 1 waits in the boot ROM's holding pen, which reads the inter-core
 * FIFO. start_cores sends it the boot ROM's six-word launch sequence (0, 0, 1,
 * vector table, stack, entry), each word echoed back, as the SDK's
 * multicore_launch_core1_raw does, with SEV after each push (2c). The board
 * supplies the vector table and the stack, as the SSE-200's supplies CPU1's
 * table. A debugger start that skipped the boot ROM must also seed core 1's
 * RCP salt first, or the pen never reads the FIFO, and that too is the
 * board's: a product always boots through the boot ROM, which seeds it.
 */

#include <cyros/port/port_mcu.h>

#include "cortex_m.hpp"
#include "mtime.hpp"

#include <cstddef>
#include <cstdint>

namespace cortex_m = cyros::port::cortex_m;

namespace
{

/* --- SIO -------------------------------------------------------------- */

constexpr std::uintptr_t sio_base             = 0xd0000000u;
constexpr std::uintptr_t sio_cpuid            = sio_base + 0x000u;
constexpr std::uintptr_t sio_fifo_st          = sio_base + 0x050u;
constexpr std::uintptr_t sio_fifo_wr          = sio_base + 0x054u;
constexpr std::uintptr_t sio_fifo_rd          = sio_base + 0x058u;
constexpr std::uint32_t  fifo_vld             = 1u << 0;
constexpr std::uint32_t  fifo_rdy             = 1u << 1;
constexpr std::uintptr_t sio_doorbell_out_set = sio_base + 0x180u;
constexpr std::uintptr_t sio_doorbell_in_clr  = sio_base + 0x18cu;
constexpr std::uintptr_t sio_mtime            = sio_base + 0x1b0u;
constexpr std::uintptr_t sio_mtimeh           = sio_base + 0x1b4u;
constexpr std::uintptr_t sio_mtimecmp         = sio_base + 0x1b8u;
constexpr std::uintptr_t sio_mtimecmph        = sio_base + 0x1bcu;

/* cyros rings one doorbell, bit 0 of the eight. */
constexpr std::uint32_t reschedule_doorbell = 1u << 0;

/* --- NVIC and the core ------------------------------------------------ */

constexpr std::uintptr_t nvic_iser0 = 0xE000E100u;
constexpr std::uintptr_t nvic_icer0 = 0xE000E180u;
constexpr std::uintptr_t nvic_icpr0 = 0xE000E280u;
constexpr std::uintptr_t nvic_ipr0  = 0xE000E400u;   /* one BYTE per IRQ */

constexpr std::uint32_t sio_irq_bell     = 26u;
constexpr std::uint32_t sio_irq_mtimecmp = 29u;

constexpr std::uintptr_t scb_actlr        = 0xE000E008u;
constexpr std::uint32_t  actlr_extexclall = 1u << 29;

/* A read whose value is not wanted. `(void)reg(a)` would not do: a discarded
 * call returning a volatile reference is never converted to a value, so the
 * load is not emitted (CLAUDE_cyros.md 6c). */
inline std::uint32_t load(std::uintptr_t address) noexcept
{
   return cortex_m::reg(address);
}

/* The entry every core runs, stashed by start_cores for core 1. Written by
 * core 0 before the launch, read-only after. */
cyros_port_core_entry_t core_entry = nullptr;

/* Before anything this core does can race the other's exclusives. */
void exclusives_global_on_this_core() noexcept
{
   cortex_m::reg(scb_actlr) = cortex_m::reg(scb_actlr) | actlr_extexclall;
   cortex_m::dsb();
   cortex_m::isb();
}

/* At the device priority, so a preempt-disable does not mask it: the
 * doorbell's whole job is to pend a reschedule, which must be TAKEN even while
 * this core sits in a kernel critical section, as on the SSE-200. */
void irq_enable(std::uint32_t irq) noexcept
{
   cortex_m::reg8(nvic_ipr0 + irq) = static_cast<std::uint8_t>(cortex_m::device_irq_priority());
   cortex_m::reg(nvic_icpr0) = 1u << irq;
   cortex_m::reg(nvic_iser0) = 1u << irq;
   cortex_m::dsb();
   cortex_m::isb();
}

void enable_doorbell_on_this_core() noexcept
{
   /* A ring left from a previous kernel lifecycle would otherwise fire into
    * this one's first thread. */
   cortex_m::reg(sio_doorbell_in_clr) = reschedule_doorbell;
   irq_enable(sio_irq_bell);
}

void fifo_push(std::uint32_t value) noexcept
{
   while ((cortex_m::reg(sio_fifo_st) & fifo_rdy) == 0u) {}
   cortex_m::reg(sio_fifo_wr) = value;
   asm volatile("sev" ::: "memory");
}

/* Bounded: a core 1 that never answers is a fault, not a hang. */
bool fifo_pop(std::uint32_t& value) noexcept
{
   for (std::uint32_t spins = 0; spins < 10'000'000u; ++spins) {
      if ((cortex_m::reg(sio_fifo_st) & fifo_vld) != 0u) {
         value = cortex_m::reg(sio_fifo_rd);
         return true;
      }
   }
   return false;
}

void fifo_drain() noexcept
{
   while ((cortex_m::reg(sio_fifo_st) & fifo_vld) != 0u) { (void)load(sio_fifo_rd); }
}

} // namespace

/* Where a launched core joins the kernel, defined below. */
extern "C" [[noreturn]] void cyros_port_secondary_core_entry(void);


/* ============================================================================
 * Board-supplied
 * ========================================================================= */

/**
 * @brief Core 1's vector table and the top of its stack, supplied by the
 *        application.
 *
 * Core 1 runs on the stack from launch, and it stays core 1's MSP, its
 * handler stack, as the boot stack is core 0's. 8-byte aligned. The table may
 * be core 0's own, because every handler cyros routes acts on the core it runs
 * on, and must be aligned to its size rounded up to a power of two (512 bytes
 * on this part, rp2350-notes.md 3a). cyros declares both and defines neither,
 * which keeps libcyros.a free of any vector table.
 */
extern "C" void const* cyros_port_core1_vector_table(void);
extern "C" void* cyros_port_core1_stack_top(void);


/* ============================================================================
 * SMP & Multi-Core Support
 * ========================================================================= */

std::uint32_t cyros_port_get_core_id(void)
{
   return cortex_m::reg(sio_cpuid);
}

void cyros_port_start_cores(std::size_t cores_to_use, cyros_port_core_entry_t entry)
{
   CYROS_ASSERT_OP(cores_to_use, >=, 1u);
   CYROS_ASSERT_OP(cores_to_use, <=, static_cast<std::size_t>(CYROS_PORT_CORE_COUNT));
   CYROS_ASSERT(entry != nullptr);
   CYROS_ASSERT_OP(cyros_port_get_core_id(), ==, 0u);

   exclusives_global_on_this_core();
   core_entry = entry;
   enable_doorbell_on_this_core();

   if (cores_to_use > 1u) {
      void const* const table = cyros_port_core1_vector_table();
      void* const stack = cyros_port_core1_stack_top();
      CYROS_ASSERT(table != nullptr);
      CYROS_ASSERT(stack != nullptr);
      CYROS_ASSERT_OP(reinterpret_cast<std::uintptr_t>(stack) & 7u, ==, 0u);

      std::uint32_t const sequence[6] = {
         0u, 0u, 1u,
         static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(table)),
         static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(stack)),
         static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(&cyros_port_secondary_core_entry)),
      };
      /* Everything core 1 will read, the stash above included, before the
       * words that release it. */
      cortex_m::dsb();

      /* Each word is echoed. A wrong echo restarts the sequence, and a zero is
       * sent only on an empty FIFO, which is what lets the pen resynchronise
       * from any state. 6 words on every launch measured (2c). */
      std::uint32_t index = 0;
      std::uint32_t words = 0;
      while (index < 6u) {
         if (sequence[index] == 0u) {
            fifo_drain();
            asm volatile("sev" ::: "memory");
         }
         fifo_push(sequence[index]);
         std::uint32_t echo = 0;
         CYROS_ASSERT(fifo_pop(echo));      /* core 1 is in the pen and answers */
         index = echo == sequence[index] ? index + 1u : 0u;
         CYROS_ASSERT_OP(++words, <, 64u);  /* and the sequence converges */
      }
   }

   /* Core 0 runs the same entry as every other. */
   entry();
   CYROS_PORT_UNREACHABLE();
}

/**
 * @brief Where core 1 joins the kernel: the boot ROM's launch enters here, on
 *        the board's stack and vector table.
 *
 * init_this_core is the non-negotiable part, as on the SSE-200: every register
 * it touches is private to a core, so skipping it would run this core with
 * default handler priorities and the FPU off.
 */
extern "C" [[noreturn]] void cyros_port_secondary_core_entry(void)
{
   exclusives_global_on_this_core();
   cortex_m::init_this_core();
   enable_doorbell_on_this_core();

   CYROS_ASSERT(core_entry != nullptr);
   core_entry();
   CYROS_PORT_UNREACHABLE();
}

void cyros_port_send_reschedule_ipi(std::uint32_t core_id)
{
   CYROS_ASSERT_OP(core_id, <, static_cast<std::uint32_t>(CYROS_PORT_CORE_COUNT));

   if (core_id == cyros_port_get_core_id()) {
      /* Its own reschedule needs no doorbell, as on the SSE-200. */
      cyros_port_pend_reschedule();
      return;
   }

   /* The other core reads scheduler state as soon as it takes the interrupt,
    * so this core's writes go first. With two cores, OUT is the other one. */
   cortex_m::dsb();
   cortex_m::reg(sio_doorbell_out_set) = reschedule_doorbell;
}

/**
 * @brief SIO_IRQ_BELL, routed from the application's vector table. Both cores
 *        install it, and each clears its own doorbell.
 *
 * Clear before pending, and make the clear visible before returning, or the
 * doorbell fires again the moment this returns. PendSV is lowest, so the
 * switch happens once every active handler has unwound.
 */
extern "C" void SIO_BELL_Handler(void)
{
   cortex_m::reg(sio_doorbell_in_clr) = reschedule_doorbell;
   cortex_m::dsb();
   cyros_port_pend_reschedule();
}


/* ============================================================================
 * The MTIME timer (../../common/mtime.hpp)
 * ========================================================================= */

namespace cyros::port::mtime
{

std::uint64_t read() noexcept
{
   std::uint32_t high, low, again;
   do {
      high  = cortex_m::reg(sio_mtimeh);
      low   = cortex_m::reg(sio_mtime);
      again = cortex_m::reg(sio_mtimeh);
   } while (high != again);
   return (static_cast<std::uint64_t>(high) << 32) | low;
}

void compare_write(std::uint32_t core, std::uint64_t value) noexcept
{
   /* One MTIMECMP per core at one address, so a core can only write its own.
    * The time source only ever asks for that. */
   CYROS_ASSERT_OP(core, ==, cyros_port_get_core_id());
   cortex_m::reg(sio_mtimecmp)  = ~0u;
   cortex_m::reg(sio_mtimecmph) = static_cast<std::uint32_t>(value >> 32);
   cortex_m::reg(sio_mtimecmp)  = static_cast<std::uint32_t>(value);
}

void interrupt_enable() noexcept
{
   irq_enable(sio_irq_mtimecmp);
}

void interrupt_disable() noexcept
{
   cortex_m::reg(nvic_icer0) = 1u << sio_irq_mtimecmp;
   cortex_m::dsb();
   cortex_m::isb();
}

} // namespace cyros::port::mtime

/**
 * @brief SIO_IRQ_MTIMECMP, routed from the application's vector table. Both
 *        cores install it.
 *
 * Masked for the call, so the time source sees what the RISC-V trap gives it
 * (mtime.hpp says why).
 */
extern "C" void SIO_MTIMECMP_Handler(void)
{
   cyros_mask_token_t const token = cyros_port_irq_save();
   cyros::port::mtime::interrupt();
   cyros_port_irq_restore(token);
}


/* ============================================================================
 * What the core layer needs (cortex_m.hpp)
 * ========================================================================= */

namespace cyros::port::cortex_m
{

std::uint64_t timestamp() noexcept
{
   /* MTIME, Ryan's choice for this chip (rp2350-notes.md 10): at FULLSPEED it
    * counts the core clock, so it is as fine as the DWT counter, and both
    * cores read the same counter, so stamps compare across them. 10 cycles a
    * read against the DWT's 2 (rp2350-notes.md 4a). */
   return mtime::read();
}

} // namespace cyros::port::cortex_m
