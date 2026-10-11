/**
 * @file port_mcu_rp2040.cpp
 * @brief The RP2040's Cortex-M0+ core 0: identity, the TIMER as MTIME, and
 *        the timestamp.
 *
 * The core layer is armv6m, unchanged, as under cortex_m0. What this chip adds
 * is its TIMER, at 0x40054000 (RP2040 datasheet 4.6):
 *
 *   TIMERAWH/L       one 64-bit count both cores share, at the rate of the
 *                    watchdog's tick generator, which the board sets
 *   ALARM0 to 3      four alarms, each firing when the LOW 32 bits of the
 *                    count EQUAL it, raising TIMER_IRQ_n, and disarming itself
 *   INTR, INTE, INTF raw, enable and force, a bit per alarm
 *
 * That is MTIME with a weaker compare, so the time source is ../../common's
 * (mtime.hpp), the one the RP2350's cores compile, and this file supplies a
 * 64-bit MTIMECMP over an alarm. cyros takes alarm 3 for core 0, and alarm 2
 * is core 1's on the two-core target to come, which leaves 0 and 1 to the
 * application, as the RP2350 port leaves that chip's TIMER0 alone.
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

constexpr std::uintptr_t sio_cpuid = 0xd0000000u;

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

/* A core's alarm, 3 for core 0 and 2 for core 1. Its IRQ number is the
 * alarm's own, TIMER_IRQ_3 and TIMER_IRQ_2. */
std::uint32_t alarm_of(std::uint32_t core) noexcept
{
   return 3u - core;
}

/* At the device priority, SysTick's on the other Arm targets. On ARMv6-M a
 * preempt-disable is a software deferral and masks no interrupt, so all the
 * timer needs is to sit above PendSV. */
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

} // namespace


/* ============================================================================
 * SMP & Multi-Core Support
 * ========================================================================= */

std::uint32_t cyros_port_get_core_id(void)
{
   return cortex_m::reg(sio_cpuid);
}

void cyros_port_start_cores(std::size_t cores_to_use, cyros_port_core_entry_t entry)
{
   /* Core 0 alone. Core 1 stays in the boot ROM until the two-core target. */
   CYROS_ASSERT_OP(cores_to_use, ==, 1u);
   CYROS_ASSERT(entry != nullptr);
   CYROS_ASSERT_OP(cyros_port_get_core_id(), ==, 0u);

   entry();

   /* entry() reaches cyros_port_start_first, which never returns. */
   CYROS_PORT_UNREACHABLE();
}

void cyros_port_send_reschedule_ipi(std::uint32_t core_id)
{
   /* One core: the cross-core request is a local one. */
   CYROS_ASSERT_OP(core_id, ==, 0u);
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
 * @brief TIMER_IRQ_3, core 0's alarm, routed from the application's vector
 *        table. Core 1's will be TIMER_IRQ_2, to the same handler, which acts
 *        on the alarm of the core it runs on.
 *
 * Masked for the call, so the time source sees what the RISC-V trap gives it
 * (mtime.hpp says why).
 */
extern "C" void TIMER_ALARM_Handler(void)
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
    * stamps will compare across them. Its unit is the tick generator's,
    * a microsecond on the board, not a cycle: ARMv6-M has no cycle counter,
    * and SysTick, the only counter at the core clock, wraps its 24 bits every
    * 134 ms at 125 MHz and is private to a core. */
   return mtime::read();
}

} // namespace cyros::port::cortex_m
