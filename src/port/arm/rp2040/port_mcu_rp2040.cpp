/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file port_mcu_rp2040.cpp
 * @brief The RP2040's Cortex-M0+ cores: identity, bring-up, the IPI, the TIMER
 *        as MTIME, the timestamp, and the lock that makes atomics atomic
 *        across the cores.
 *
 * Compiled by two targets that differ only in port_traits.h: `rp2040`, core 0
 * alone, and `rp2040_smp`, both cores, as the RP2350's two Arm targets are.
 * The core layer is armv6m, unchanged, as under cortex_m0. What this chip
 * adds is in its SIO, the single-cycle block each core sees at 0xd0000000,
 * and its TIMER, at 0x40054000 (RP2040 datasheet 2.3.1, 4.6):
 *
 *   CPUID            which core is reading: one address, a per-core answer
 *   FIFO_ST/WR/RD    two one-way FIFOs of 8 words, one to each core, each
 *                    raising its reader's SIO_IRQ_PROCn while it holds a word
 *   SPINLOCK0 to 31  hardware locks: a read claims, a write releases
 *   TIMERAWH/L       one 64-bit count both cores share, at the rate of the
 *                    watchdog's tick generator, which the board sets
 *   ALARM0 to 3      four alarms, each firing when the LOW 32 bits of the
 *                    count EQUAL it, raising TIMER_IRQ_n, and disarming itself
 *   INTR, INTE, INTF raw, enable and force, a bit per alarm
 *
 * The TIMER is MTIME with a weaker compare, so the time source is
 * ../../common's (mtime.hpp), the one the RP2350's cores compile, and this
 * file supplies a 64-bit MTIMECMP per core over an alarm: alarm 3 for core 0
 * and 2 for core 1, which leaves 0 and 1 to the application, as the RP2350
 * port leaves that chip's TIMER0 alone.
 *
 *
 * MTIMECMP OVER A 32-BIT EQUALITY ALARM
 * =====================================
 * MTIMECMP raises its interrupt while MTIME >= MTIMECMP, a level that only a
 * new compare lowers. An alarm fires once, as the count passes through its
 * value, so two things differ, and compare_write covers both:
 *
 *   - a value already reached never fires. Every write reads the count after
 *     arming, and a value the count has passed with the alarm still armed
 *     (the arm landed too late) is disarmed and FORCED through INTF, which
 *     holds the line as MTIMECMP would until the next compare_write.
 *   - a value 2^32 counts or more ahead, 71 minutes at 1 MHz, fires early,
 *     when the low word first matches. The time source takes an early
 *     interrupt as spurious and re-arms from its record
 *     (port_time_mtime.cpp), and a periodic tick is never that far.
 *
 * INTE and INTF are shared by both cores, so they change through the SET and
 * CLR aliases every APB register has, which no other core's write can race.
 * ARMED and INTR are write-one-to-clear, already per bit.
 *
 *
 * How core 1 starts, and the IPI
 * ==============================
 * Core 1 waits in the boot ROM, which reads the FIFO. start_cores sends it the
 * boot ROM's six-word launch sequence (0, 0, 1, vector table, stack, entry),
 * each word echoed back, as the SDK's multicore_launch_core1_raw does and as
 * the RP2350 port does, with SEV after each push. The board supplies the
 * vector table and the stack. 6 words on every launch, from a debugger start
 * too (arm-port-notes.md 19h): unlike the RP2350's, this boot ROM has no RCP
 * salt to wait for.
 *
 * The RP2040 has no doorbell, so once core 1 runs, the FIFOs are cyros's IPI:
 * a word into the other core's FIFO raises its SIO_IRQ_PROCn, and the word
 * carries nothing. Both lines reach both NVICs (measured), so each core
 * enables its own, core 0 IRQ 15 and core 1 IRQ 16. On one core the FIFOs
 * are left to the application.
 *
 *
 * Atomics across the cores
 * ========================
 * ARMv6-M has no exclusives, and the core layer's read-modify-writes are
 * atomic only against this core's interrupts until a target lends them a lock
 * both cores share (port_atomic_armv6m.cpp, which says what that lock cannot
 * reach). This target lends SIO spinlock 14, the first of the two the SDK
 * reserves for an operating system (PICO_SPINLOCK_ID_OS1), so an application
 * built on the SDK keeps the rest. The board releases every spinlock at reset,
 * as the SDK's runtime does: a debugger's reset halt resets only the cores,
 * and a lock the last image held would otherwise stay held.
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

constexpr std::uintptr_t sio_base     = 0xd0000000u;
constexpr std::uintptr_t sio_cpuid    = sio_base + 0x000u;
constexpr std::uintptr_t sio_fifo_st  = sio_base + 0x050u;
constexpr std::uintptr_t sio_fifo_wr  = sio_base + 0x054u;
constexpr std::uintptr_t sio_fifo_rd  = sio_base + 0x058u;
constexpr std::uint32_t  fifo_vld     = 1u << 0;
constexpr std::uint32_t  fifo_rdy     = 1u << 1;

/* PICO_SPINLOCK_ID_OS1. */
constexpr std::uintptr_t sio_spinlock_atomics = sio_base + 0x100u + 4u * 14u;

/* Core n's FIFO raises SIO_IRQ_PROCn, IRQ 15 + n. */
constexpr std::uint32_t sio_irq_proc0 = 15u;

/* --- TIMER ------------------------------------------------------------ */

constexpr std::uintptr_t timer_base   = 0x40054000u;
constexpr std::uintptr_t timer_alarm0 = timer_base + 0x10u;
constexpr std::uintptr_t timer_armed  = timer_base + 0x20u;
constexpr std::uintptr_t timer_rawh   = timer_base + 0x24u;
constexpr std::uintptr_t timer_rawl   = timer_base + 0x28u;
constexpr std::uintptr_t timer_intr   = timer_base + 0x34u;
constexpr std::uintptr_t timer_inte   = timer_base + 0x38u;
constexpr std::uintptr_t timer_intf   = timer_base + 0x3cu;

/* A write to a register's alias sets, or clears, only the bits it carries. */
constexpr std::uintptr_t set_alias = 0x2000u;
constexpr std::uintptr_t clr_alias = 0x3000u;

/* --- NVIC ------------------------------------------------------------- */

constexpr std::uintptr_t nvic_iser0 = 0xE000E100u;
constexpr std::uintptr_t nvic_icer0 = 0xE000E180u;
constexpr std::uintptr_t nvic_icpr0 = 0xE000E280u;
constexpr std::uintptr_t nvic_ipr0  = 0xE000E400u;   /* word access only on ARMv6-M */

constexpr std::uint64_t never = ~std::uint64_t{0};

/* A read whose value is not wanted. `(void)reg(a)` would not do: a discarded
 * call returning a volatile reference is never converted to a value, so the
 * load is not emitted (CLAUDE_cyros-bare-metal.md 1, trap 1). */
inline std::uint32_t load(std::uintptr_t address) noexcept
{
   return cortex_m::reg(address);
}

/* A core's alarm, 3 for core 0 and 2 for core 1. Its IRQ number is the
 * alarm's own, TIMER_IRQ_3 and TIMER_IRQ_2. */
std::uint32_t alarm_of(std::uint32_t core) noexcept
{
   return 3u - core;
}

/* At the device priority, SysTick's on the other Arm targets. On ARMv6-M a
 * preempt-disable is a software deferral and masks no interrupt, so all the
 * timer and the IPI need is to sit above PendSV. The NVIC is per core, so
 * each core enables its own lines. */
void irq_enable(std::uint32_t irq) noexcept
{
   std::uintptr_t const ipr = nvic_ipr0 + 4u * (irq / 4u);
   std::uint32_t const shift = 8u * (irq % 4u);
   cortex_m::reg(ipr) = (cortex_m::reg(ipr) & ~(0xFFu << shift))
                      | (cortex_m::device_irq_priority() << shift);
   cortex_m::reg(nvic_icpr0) = 1u << irq;
   cortex_m::reg(nvic_iser0) = 1u << irq;
   cortex_m::dsb();
   cortex_m::isb();
}

/* The entry every core runs, stashed by start_cores for core 1. Written by
 * core 0 before the launch, read-only after. */
cyros_port_core_entry_t core_entry = nullptr;

void fifo_drain() noexcept
{
   while ((cortex_m::reg(sio_fifo_st) & fifo_vld) != 0u) { (void)load(sio_fifo_rd); }
}

/* Whatever the boot ROM or the launch left in this core's FIFO raises the IRQ
 * as soon as interrupts open, and the handler clears it at the cost of one
 * redundant reschedule. The boot ROM left core 1's ROE set on every launch
 * measured (arm-port-notes.md 19h), so that path runs every time. */
void enable_ipi_on_this_core(std::uint32_t core) noexcept
{
   irq_enable(sio_irq_proc0 + core);
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
 * be core 0's own, because every handler cyros routes acts on the core it
 * runs on, and must be 256-byte aligned, which VTOR requires. cyros declares
 * both and defines neither, which keeps libcyros.a free of any vector table.
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

   core_entry = entry;

   if (cores_to_use > 1u) {
      void const* const table = cyros_port_core1_vector_table();
      void* const stack = cyros_port_core1_stack_top();
      CYROS_ASSERT(table != nullptr);
      CYROS_ASSERT(stack != nullptr);
      CYROS_ASSERT_OP(reinterpret_cast<std::uintptr_t>(table) & 255u, ==, 0u);
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
       * sent only on an empty FIFO, which is what lets the boot ROM
       * resynchronise from any state. */
      std::uint32_t index = 0;
      std::uint32_t words = 0;
      while (index < 6u) {
         if (sequence[index] == 0u) {
            fifo_drain();
            asm volatile("sev" ::: "memory");
         }
         fifo_push(sequence[index]);
         std::uint32_t echo = 0;
         bool const answered = fifo_pop(echo);
         CYROS_ASSERT(answered);            /* core 1 is in the boot ROM and answers */
         index = echo == sequence[index] ? index + 1u : 0u;
         ++words;
         CYROS_ASSERT_OP(words, <, 64u);    /* and the sequence converges */
      }

      /* After the launch, whose echoes arrive in this FIFO. Interrupts are
       * still masked from cyros_port_init, so the handler cannot run before
       * the first thread does. */
      enable_ipi_on_this_core(0u);
   }

   /* Core 0 runs the same entry as every other. */
   entry();
   CYROS_PORT_UNREACHABLE();
}

/**
 * @brief Where core 1 joins the kernel: the boot ROM's launch enters here, on
 *        the board's stack and vector table.
 *
 * init_this_core is the non-negotiable part: every register it touches is
 * private to a core (PRIMASK, the system handler priorities, the MPU that
 * holds the stack guard), so skipping it would run this core with PendSV at
 * its reset priority and no guard.
 */
extern "C" [[noreturn]] void cyros_port_secondary_core_entry(void)
{
   cortex_m::init_this_core();
   enable_ipi_on_this_core(1u);

   CYROS_ASSERT(core_entry != nullptr);
   core_entry();
   CYROS_PORT_UNREACHABLE();
}

void cyros_port_send_reschedule_ipi(std::uint32_t core_id)
{
   CYROS_ASSERT_OP(core_id, <, static_cast<std::uint32_t>(CYROS_PORT_CORE_COUNT));

   if (core_id == cyros_port_get_core_id()) {
      /* Its own reschedule needs no IPI. */
      cyros_port_pend_reschedule();
      return;
   }

   /* The other core reads scheduler state as soon as it takes the interrupt,
    * so this core's writes go first. With two cores, this core's FIFO writes
    * reach the other one. */
   cortex_m::dsb();

   /* A full FIFO holds 8 words the other core has yet to read, and reading
    * them will reschedule it after this point, so this request rides on
    * theirs. Economy, not correctness: a word written into a full FIFO is
    * dropped and sets WOF, which raises THIS core's FIFO interrupt, whose
    * handler clears it with a redundant reschedule. So an interrupt on this
    * core filling the FIFO between the check and the write costs that and
    * nothing more. */
   if ((cortex_m::reg(sio_fifo_st) & fifo_rdy) != 0u) {
      cortex_m::reg(sio_fifo_wr) = 0u;
   }
}

/**
 * @brief SIO_IRQ_PROC0 and SIO_IRQ_PROC1, IRQs 15 and 16, routed from the
 *        application's vector table. Each core enables its own, and empties
 *        its own FIFO.
 *
 * Every word read, so the line drops, and the sticky flags cleared, which hold
 * it raised too: WOF, read-on-empty's ROE, which the boot ROM leaves set on
 * core 1. PendSV is lowest, so the switch happens once every active handler
 * has unwound.
 */
extern "C" CYROS_PORT_LINKED_BY_NAME void SIO_FIFO_Handler(void)
{
   fifo_drain();
   cortex_m::reg(sio_fifo_st) = 0u;
   cortex_m::dsb();
   cyros_port_pend_reschedule();
}


/* ============================================================================
 * The TIMER as MTIME (../../common/mtime.hpp)
 * ========================================================================= */

namespace cyros::port::mtime
{

std::uint64_t read() noexcept
{
   /* The RAW pair. Reading TIMELR latches TIMEHR for the next read, one latch
    * for the whole chip, which another core or an interrupt between the two
    * reads would take. */
   std::uint32_t high, low, again;
   do {
      high  = cortex_m::reg(timer_rawh);
      low   = cortex_m::reg(timer_rawl);
      again = cortex_m::reg(timer_rawh);
   } while (high != again);
   return (static_cast<std::uint64_t>(high) << 32) | low;
}

void compare_write(std::uint32_t core, std::uint64_t value) noexcept
{
   /* Each core arms only its own alarm, and the time source only ever asks
    * that, as with the RP2350's per-core MTIMECMP. */
   CYROS_ASSERT_OP(core, ==, cyros_port_get_core_id());
   std::uint32_t const alarm = alarm_of(core);
   std::uint32_t const bit = 1u << alarm;

   /* Whatever the last compare left: the alarm disarmed, and its interrupt,
    * raw or forced, lowered. */
   cortex_m::reg(timer_armed) = bit;
   cortex_m::reg(timer_intr) = bit;
   cortex_m::reg(timer_intf + clr_alias) = bit;
   if (value == never) {
      return;
   }

   if (read() < value) {
      cortex_m::reg(timer_alarm0 + 4u * alarm) = static_cast<std::uint32_t>(value);
      /* Still ahead after arming, so the alarm will see it pass. Or behind,
       * and disarmed: the alarm fired, and INTR holds the line. */
      if (read() < value || (cortex_m::reg(timer_armed) & bit) == 0u) {
         return;
      }
      /* Behind and still armed: the count passed the value before the arm
       * landed, and the alarm would wait for the low word's next pass. */
      cortex_m::reg(timer_armed) = bit;
   }

   /* Due already: held raised, as an MTIMECMP at or below MTIME would be. */
   cortex_m::reg(timer_intf + set_alias) = bit;
}

void interrupt_enable() noexcept
{
   std::uint32_t const alarm = alarm_of(cyros_port_get_core_id());
   cortex_m::reg(timer_inte + set_alias) = 1u << alarm;
   irq_enable(alarm);
}

void interrupt_disable() noexcept
{
   std::uint32_t const alarm = alarm_of(cyros_port_get_core_id());
   cortex_m::reg(nvic_icer0) = 1u << alarm;
   cortex_m::reg(timer_inte + clr_alias) = 1u << alarm;
   cortex_m::dsb();
   cortex_m::isb();
}

} // namespace cyros::port::mtime

/**
 * @brief TIMER_IRQ_3 and TIMER_IRQ_2, cores 0 and 1's alarms, routed from the
 *        application's vector table to this one handler, which acts on the
 *        alarm of the core it runs on.
 *
 * Masked for the call, so the time source sees what the RISC-V trap gives it
 * (mtime.hpp says why).
 */
extern "C" CYROS_PORT_LINKED_BY_NAME void TIMER_ALARM_Handler(void)
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
   /* The TIMER, as MTIME is on the RP2350: both cores read the one count, so
    * stamps compare across them. Its unit is the tick generator's, a
    * microsecond on the board, not a cycle: ARMv6-M has no cycle counter,
    * and SysTick, the only counter at the core clock, wraps its 24 bits every
    * 134 ms at 125 MHz and is private to a core. */
   return mtime::read();
}

/* A read claims the lock, answering nonzero, or answers zero while the other
 * core holds it. Unbounded, as the kernel's own spinlock is: the holder is
 * masked and about a dozen instructions from its release. The SDK's
 * spin_lock_unsafe_blocking and spin_unlock, fences included: the SIO is
 * reached through the core's single-cycle port, not the bus SRAM sits on, so
 * the DMBs order the lock against the memory it guards. */
void atomic_lock() noexcept
{
   while (cortex_m::reg(sio_spinlock_atomics) == 0u) {}
   cortex_m::dmb();
}

void atomic_unlock() noexcept
{
   cortex_m::dmb();
   cortex_m::reg(sio_spinlock_atomics) = 0u;   /* any value releases */
}

} // namespace cyros::port::cortex_m
