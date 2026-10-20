/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file port_mcu_leon3.cpp
 * @brief A GRLIB LEON3 system: identity, bring-up, the IPI, the console, and
 *        GPTIMER presented as MTIME.
 *
 * Compiled by two targets that differ only in port_traits.h: `leon3`, one
 * core, and `leon3_smp`, CYROS_PORT_CORE_COUNT cores. QEMU's leon3_generic is
 * the bench. `~/cyros-claude/sparc-port-plan.md` has the measurements.
 *
 *
 * Devices are found, not assumed
 * ===============================
 * Every GRLIB system lists its devices in AMBA plug-and-play records: the AHB
 * table at 0xFFFFF000 names the AHB/APB bridge, and the APB table at the top
 * of the bridge's space names each APB device, its address and its interrupt.
 * The first use of any device scans them once, for the IRQMP (interrupt
 * controller), the GPTIMER (timers) and the APBUART (console). A system
 * without one of the first two stops with a panic naming it.
 *
 *
 * The reschedule and the IPI: the IRQMP's force registers
 * ======================================================
 * Each core has a force register, and writing a level's bit raises that
 * level on that core alone. Level 1 is the reschedule (sparcv8.hpp), so the
 * local request and the IPI are one write to a different core's register,
 * as the SGI is on the R52. The IRQMP drops the forced bit when the core
 * takes the trap. Writes to it OR bits in. The register at offset 0x08 is an
 * alias for core 0's that REPLACES them, and is never used.
 *
 *
 * How the other cores start
 * =========================
 * A LEON3 other than core 0 is powered down at reset until its bit in the
 * IRQMP's multiprocessor status register is written. QEMU then starts it at
 * the image's entry. So nothing is parked: start_cores writes the bit, and
 * the board's reset path hands the core to cyros_port_secondary_core_entry.
 *
 *
 * GPTIMER as MTIME
 * ================
 * src/port/common/mtime.hpp wants a 64-bit count every core reads and a
 * compare per core whose interrupt is level while the count has reached it.
 * GPTIMER has 32-bit down-counters with a shared prescaler, two on QEMU.
 *
 * THE COUNT is timer 1, free-running from 0xFFFFFFFF with no interrupt,
 * extended to 64 bits from an EPOCH: a 64-bit base and the raw count it was
 * taken at, so now is base + (raw at the epoch - raw now) modulo 2^32. That
 * holds while the epoch is less than 2^32 counts old (71 minutes at 1 MHz),
 * and every compare write and every compare interrupt renews it, and the
 * compare is never armed further than 2^31 counts ahead, so it always is
 * while the time source runs. Readers take it lock-free under a sequence
 * count. Timer 1's control register is written once: QEMU restarts a timer
 * from its last loaded value on any control write that sets EN without LD,
 * which on a running count is a jump.
 *
 * THE COMPARE is timer 2, one-shot, armed for the earliest compare of any
 * core. Its interrupt is unmasked on every core that takes the time source's,
 * so whichever core acknowledges it first rereads every compare under the
 * timer lock: each core whose compare has been reached gets the timer's level
 * forced on it, and timer 2 is rearmed for the rest. A forced level on the
 * core whose compare it is is exactly what a per-core compare would raise,
 * and the time source rereads the count in its handler, so an early or extra
 * interrupt costs a check. With a timer per core (a part with more than two)
 * the lock and the forcing would go, which is for when one is measured.
 *
 * THE RATE is the BOARD's (cyros_port_mtime_clock_hz), which also sets the
 * prescaler, since only the board knows its system clock.
 */

#include <cyros/port/port_mcu.h>

/* For the masks around the time source: a target may call the core
 * contract, it may not implement it. */
#include <cyros/port/port_core.h>

#include "mtime.hpp"
#include "sparcv8.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace sparcv8 = cyros::port::sparcv8;
namespace mtime   = cyros::port::mtime;

namespace
{

inline volatile std::uint32_t& reg(std::uintptr_t address) noexcept
{
   return *reinterpret_cast<volatile std::uint32_t*>(address);
}

/* ============================================================================
 * AMBA plug and play
 * ========================================================================= */

constexpr std::uintptr_t ahb_pnp_masters = 0xFFFFF000u;   /* 64 records of 32 bytes */
constexpr std::uintptr_t ahb_pnp_slaves  = 0xFFFFF800u;
constexpr std::uintptr_t apb_pnp_offset  = 0x000FF000u;   /* from the bridge's base */

constexpr std::uint32_t vendor_gaisler = 0x01u;
constexpr std::uint32_t device_apbmst  = 0x006u;   /* the AHB/APB bridge */
constexpr std::uint32_t device_apbuart = 0x00Cu;
constexpr std::uint32_t device_irqmp   = 0x00Du;
constexpr std::uint32_t device_gptimer = 0x011u;

struct devices_found
{
   std::uintptr_t irqmp;
   std::uintptr_t gptimer;
   std::uintptr_t apbuart;
   std::uint32_t  compare_level; /* timer 2's interrupt level */
   bool           scanned;
};

/* Written by the first use, on the bootstrap core before any other runs, or
 * in a panic. Read-only after. */
devices_found found = {};

inline std::uint32_t pnp_vendor(std::uint32_t id) noexcept { return id >> 24; }
inline std::uint32_t pnp_device(std::uint32_t id) noexcept { return (id >> 12) & 0xFFFu; }

/* An AHB bank address register: address bits 31 to 20 with their mask. */
inline std::uintptr_t ahb_bar_base(std::uint32_t bar) noexcept
{
   return bar & 0xFFF00000u & (bar << 16);
}

/* An APB bank address register: address bits 19 to 8, within the bridge. */
inline std::uintptr_t apb_bar_offset(std::uint32_t bar) noexcept
{
   return ((bar >> 20) & (bar >> 4) & 0xFFFu) << 8;
}

devices_found const& devices() noexcept
{
   if (found.scanned) {
      return found;
   }
   found.scanned = true;

   std::uintptr_t apb = 0u;
   for (std::uint32_t i = 0; i < 64u && apb == 0u; ++i) {
      std::uintptr_t const record = ahb_pnp_slaves + i * 32u;
      std::uint32_t const id = reg(record);
      if (pnp_vendor(id) == vendor_gaisler && pnp_device(id) == device_apbmst) {
         apb = ahb_bar_base(reg(record + 16u));
      }
   }

   if (apb != 0u) {
      for (std::uint32_t i = 0; i < 512u; ++i) {
         std::uintptr_t const record = apb + apb_pnp_offset + i * 8u;
         std::uint32_t const id = reg(record);
         if (pnp_vendor(id) != vendor_gaisler) {
            continue;
         }
         std::uintptr_t const address = apb + apb_bar_offset(reg(record + 4u));
         std::uint32_t const device = pnp_device(id);
         if (device == device_irqmp && found.irqmp == 0u) {
            found.irqmp = address;
         } else if (device == device_gptimer && found.gptimer == 0u) {
            found.gptimer = address;
            /* The unit's first level, plus the timer's index when each timer
             * has its own (config bit 8, SI), else the unit's one. */
            bool const separate = (reg(address + 0x08u) & (1u << 8)) != 0u;
            found.compare_level = (id & 0x1Fu) + (separate ? 1u : 0u);
         } else if (device == device_apbuart && found.apbuart == 0u) {
            found.apbuart = address;
         }
      }
   }

   CYROS_ASSERT2(found.irqmp != 0u, apb, 0u);
   CYROS_ASSERT2(found.gptimer != 0u, apb, 1u);
   return found;
}

/* ============================================================================
 * IRQMP
 * ========================================================================= */

constexpr std::uintptr_t irqmp_mpstatus  = 0x10u;
constexpr std::uintptr_t irqmp_mask      = 0x40u;   /* + 4 per core */
constexpr std::uintptr_t irqmp_force     = 0x80u;   /* + 4 per core */

inline std::uintptr_t mask_register(std::uint32_t core) noexcept
{
   return devices().irqmp + irqmp_mask + core * 4u;
}

inline std::uintptr_t force_register(std::uint32_t core) noexcept
{
   return devices().irqmp + irqmp_force + core * 4u;
}

/* Raise `level` on `core`. Bits written to the upper half clear instead. */
inline void force(std::uint32_t core, std::uint32_t level) noexcept
{
   reg(force_register(core)) = 1u << level;
}

inline void unforce(std::uint32_t core, std::uint32_t level) noexcept
{
   reg(force_register(core)) = (1u << level) << 16;
}

/* This core's mask register is its own to change, so a read-modify-write of
 * it, masked, races nothing. */
inline void unmask(std::uint32_t core, std::uint32_t level) noexcept
{
   std::uintptr_t const address = mask_register(core);
   reg(address) = reg(address) | (1u << level);
}

inline void mask(std::uint32_t core, std::uint32_t level) noexcept
{
   std::uintptr_t const address = mask_register(core);
   reg(address) = reg(address) & ~(1u << level);
}

/* ============================================================================
 * GPTIMER
 * ========================================================================= */

constexpr std::uintptr_t gpt_counter = 0x00u;   /* per timer, at 0x10 * n */
constexpr std::uintptr_t gpt_reload  = 0x04u;
constexpr std::uintptr_t gpt_control = 0x08u;

constexpr std::uint32_t gpt_en = 1u << 0;
constexpr std::uint32_t gpt_rs = 1u << 1;
constexpr std::uint32_t gpt_ld = 1u << 2;
constexpr std::uint32_t gpt_ie = 1u << 3;

constexpr std::uint32_t count_timer   = 1u;
constexpr std::uint32_t compare_timer = 2u;

inline std::uintptr_t timer(std::uint32_t n) noexcept
{
   return devices().gptimer + 0x10u * n;
}

/* The compare timer's interrupt level, found with the unit. */
inline std::uint32_t compare_level() noexcept
{
   return devices().compare_level;
}

/* The longest a compare is armed for, so the epoch never ages past 2^32. */
constexpr std::uint32_t longest_arm = 0x80000000u;

constexpr std::uint64_t never = ~std::uint64_t{0};

/* The epoch (the file comment). The sequence count is odd while it changes.
 * Its writer holds the timer lock with interrupts masked, so on one core no
 * reader can interrupt it. */
std::atomic<std::uint32_t> epoch_sequence{0u};
std::atomic<std::uint32_t> epoch_high{0u};
std::atomic<std::uint32_t> epoch_low{0u};
std::atomic<std::uint32_t> epoch_raw{0u};

bool counting = false;   /* timer 1 started: by the first use, on core 0 */

inline std::uint32_t raw_count() noexcept
{
   return reg(timer(count_timer) + gpt_counter);
}

void ensure_counting() noexcept
{
   if (counting) {
      return;
   }
   counting = true;
   std::uintptr_t const t = timer(count_timer);
   reg(t + gpt_reload)  = 0xFFFFFFFFu;
   reg(t + gpt_control) = gpt_en | gpt_rs | gpt_ld;
   epoch_raw.store(raw_count(), std::memory_order::relaxed);
}

std::uint64_t count() noexcept
{
   ensure_counting();
   while (true) {
      std::uint32_t const before = epoch_sequence.load(std::memory_order::acquire);
      if ((before & 1u) != 0u) {
         continue;
      }
      std::uint32_t const high = epoch_high.load(std::memory_order::relaxed);
      std::uint32_t const low  = epoch_low.load(std::memory_order::relaxed);
      std::uint32_t const raw  = epoch_raw.load(std::memory_order::relaxed);
      std::atomic_thread_fence(std::memory_order::acquire);
      if (epoch_sequence.load(std::memory_order::relaxed) != before) {
         continue;
      }
      /* The count after the epoch was read, so it is at or past it. Down
       * counter: elapsed is then minus now, modulo 2^32. */
      std::uint32_t const elapsed = raw - raw_count();
      return ((static_cast<std::uint64_t>(high) << 32) | low) + elapsed;
   }
}

/* Renew the epoch to now, and return now. Its one writer at a time holds
 * the timer lock with interrupts masked. */
std::uint64_t renew_epoch() noexcept
{
   ensure_counting();
   std::uint32_t const raw = raw_count();
   std::uint64_t const base = (static_cast<std::uint64_t>(epoch_high.load(std::memory_order::relaxed)) << 32)
                            | epoch_low.load(std::memory_order::relaxed);
   std::uint64_t const now = base + (epoch_raw.load(std::memory_order::relaxed) - raw);

   std::uint32_t const sequence = epoch_sequence.load(std::memory_order::relaxed);
   epoch_sequence.store(sequence + 1u, std::memory_order::relaxed);
   std::atomic_thread_fence(std::memory_order::release);
   epoch_high.store(static_cast<std::uint32_t>(now >> 32), std::memory_order::relaxed);
   epoch_low.store(static_cast<std::uint32_t>(now), std::memory_order::relaxed);
   epoch_raw.store(raw, std::memory_order::relaxed);
   epoch_sequence.store(sequence + 2u, std::memory_order::release);
   return now;
}

/* Each core's compare, and the lock over them and timer 2. Taken masked, by
 * every caller (mtime.hpp's are). One core needs no lock. */
std::uint64_t compares[CYROS_PORT_CORE_COUNT] = {};
std::atomic_flag timer_lock = ATOMIC_FLAG_INIT;
bool compares_initialised = false;

struct timer_lock_guard
{
   timer_lock_guard() noexcept
   {
      if constexpr (CYROS_PORT_CORE_COUNT > 1) {
         while (timer_lock.test_and_set(std::memory_order::acquire)) {
            cyros_port_cpu_relax();
         }
      }
   }
   ~timer_lock_guard()
   {
      if constexpr (CYROS_PORT_CORE_COUNT > 1) {
         timer_lock.clear(std::memory_order::release);
      }
   }
   timer_lock_guard(timer_lock_guard const&) = delete;
   timer_lock_guard& operator=(timer_lock_guard const&) = delete;
};

/* Every compare reached gets its level forced on its core, and timer 2 is
 * armed for the earliest of the rest, or as far as the epoch allows. Timer
 * lock held, masked. */
void rearm(std::uint64_t now) noexcept
{
   std::uint32_t const level = compare_level();
   std::uint64_t earliest = never;
   for (std::uint32_t core = 0; core < static_cast<std::uint32_t>(CYROS_PORT_CORE_COUNT); ++core) {
      if (compares[core] <= now) {
         force(core, level);
      } else if (compares[core] < earliest) {
         earliest = compares[core];
      }
   }

   /* A one-shot of n + 1 counts underflows n + 1 counts on, so load the
    * distance less one. At least one count, at most longest_arm. */
   std::uint64_t const distance = earliest == never ? longest_arm : earliest - now;
   std::uint32_t const counts = distance > longest_arm ? longest_arm : static_cast<std::uint32_t>(distance);
   std::uintptr_t const t = timer(compare_timer);
   reg(t + gpt_reload)  = counts - 1u;
   reg(t + gpt_control) = gpt_en | gpt_ld | gpt_ie;
}

/* The entry every core runs, stashed by start_cores for the other cores.
 * Written by core 0 before any is started, read-only after. */
cyros_port_core_entry_t core_entry = nullptr;

} // namespace


/* ============================================================================
 * SMP & Multi-Core Support
 * ========================================================================= */

CYROS_PORT_HOT std::uint32_t cyros_port_get_core_id(void)
{
   return sparcv8::core_number();
}

void cyros_port_start_cores(std::size_t cores_to_use, cyros_port_core_entry_t entry)
{
   CYROS_ASSERT_OP(cores_to_use, >=, 1u);
   CYROS_ASSERT_OP(cores_to_use, <=, static_cast<std::size_t>(CYROS_PORT_CORE_COUNT));
   CYROS_ASSERT(entry != nullptr);
   /* Only core 0 runs bring-up. The others are powered down until now. */
   CYROS_ASSERT_OP(cyros_port_get_core_id(), ==, 0u);

   core_entry = entry;
   if (cores_to_use > 1u) {
      std::uint32_t start = 0u;
      for (std::uint32_t core = 1; core < cores_to_use; ++core) {
         start |= 1u << core;
      }
      /* SPARC is TSO, so every store above is visible before this one. */
      reg(devices().irqmp + irqmp_mpstatus) = start;
   }

   /* Core 0 runs the same entry as every other. */
   entry();
   CYROS_PORT_UNREACHABLE();
}

/**
 * @brief Where a started core joins the kernel. Called by the board's reset
 *        path on that core's own stack, with the trap table installed and
 *        interrupts masked.
 */
extern "C" [[noreturn]] void cyros_port_secondary_core_entry(void)
{
   sparcv8::init_this_core();
   CYROS_ASSERT(core_entry != nullptr);
   core_entry();
   CYROS_PORT_UNREACHABLE();
}

CYROS_PORT_HOT void cyros_port_send_reschedule_ipi(std::uint32_t core_id)
{
   /* The cross-core and the local request are ONE mechanism, as on RISC-V
    * and the R52. */
   CYROS_ASSERT_OP(core_id, <, static_cast<std::uint32_t>(CYROS_PORT_CORE_COUNT));
   sparcv8::reschedule_raise(core_id);
}


/* ============================================================================
 * What the sparcv8 core layer needs (sparcv8.hpp)
 * ========================================================================= */

namespace cyros::port::sparcv8
{

void reschedule_raise(std::uint32_t core) noexcept
{
   force(core, reschedule_level);
}

void reschedule_setup(std::uint32_t core) noexcept
{
   /* Only the force bit: level 1 is the reschedule's alone, so nothing else
    * should be pending there, and a device that is costs one spurious
    * reschedule, which is harmless. (QEMU's clear register also drops every
    * core's forced bits, which GRLIB's does not, so clearing here would hide
    * a missing unforce from the tests.) */
   unforce(core, reschedule_level);
   unmask(core, reschedule_level);
   ensure_counting();
}

void interrupt(std::uint32_t level) noexcept
{
   if (level == compare_level()) {
      /* Masked for the time source, which mtime.hpp asks for, as the ARM
       * targets' handlers mask: the handler runs at the timer's level, and a
       * device nesting above it could arm a deadline between the time source
       * reading a compare and writing it back, or spin on this core's timer
       * lock or epoch sequence while this core holds them. */
      cyros_mask_token_t const token = cyros_port_irq_save();
      mtime::interrupt();
      cyros_port_irq_restore(token);
   } else {
      cyros_sparc_device_interrupt(level);
   }
}

std::uint64_t timestamp() noexcept
{
   return count();
}

void console_write(char const* text) noexcept
{
   std::uintptr_t const uart = devices().apbuart;
   if (uart == 0u) {
      return;
   }
   constexpr std::uintptr_t data = 0x00u, status = 0x04u;
   constexpr std::uint32_t transmit_full = 1u << 9;
   for (; *text != '\0'; ++text) {
      while ((reg(uart + status) & transmit_full) != 0u) {}
      reg(uart + data) = static_cast<std::uint8_t>(*text);
   }
}

} // namespace cyros::port::sparcv8


/* ============================================================================
 * The MTIME timer (../../common/mtime.hpp), on GPTIMER
 * ========================================================================= */

namespace cyros::port::mtime
{

std::uint64_t read() noexcept
{
   return count();
}

void compare_write(std::uint32_t core, std::uint64_t value) noexcept
{
   CYROS_ASSERT_OP(core, <, static_cast<std::uint32_t>(CYROS_PORT_CORE_COUNT));
   timer_lock_guard const lock;
   std::uint64_t const now = renew_epoch();
   if (!compares_initialised) {
      /* Zero is "reached", so every core starts at never instead. */
      for (std::uint64_t& compare : compares) { compare = never; }
      compares_initialised = true;
   }
   compares[core] = value;
   rearm(now);
}

void interrupt_enable() noexcept
{
   unmask(sparcv8::core_number(), compare_level());
}

void interrupt_disable() noexcept
{
   mask(sparcv8::core_number(), compare_level());
}

} // namespace cyros::port::mtime
