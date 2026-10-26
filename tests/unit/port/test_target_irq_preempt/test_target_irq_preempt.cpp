/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_target_irq_preempt.cpp
 * @brief A wake from an interrupt preempts at once, wherever in a switch the
 *        interrupt lands.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the switch a yield makes, on the thread's road
 *          (cyros_port_yield_to), on the RP2350's two ISAs and the RP2040.
 * Trusts:  layers 0 to 5, the semaphore (test_sync_semaphore), and the timer
 *          interrupt reaching isr_irq0 (test_target_device_irq).
 * Proves:  that once an interrupt has woken a more urgent thread, no less
 *          urgent thread runs before it, with the interrupt swept across a
 *          yield from one thread to its equal and back.
 *
 * The case it exists for is Cortex-M's, ARMv6-M's as well as ARMv8-M's. The
 * kernel decides the switch in thread context and PendSV makes it once the
 * kernel's mask is released, so an interrupt that was waiting runs in between,
 * while the switch is owed. A wake it makes must still reach the kernel:
 * PendSV makes the owed switch, then reschedules (port_core.h). If that wake
 * were lost, the thread switched to would run ahead of the woken one until its
 * next yield. On the Hazard3 the switch is made inside the mask, so there the
 * test holds it to the same promise.
 *
 * How a lost preemption shows: every thread stamps a sequence number when it
 * runs, under a critical section, and the handler stamps one when it wakes the
 * urgent thread. When the urgent thread runs, no other thread may have stamped
 * since the handler did.
 *
 * The interrupt is a timer's alarm 0 on IRQ 0, set up as test_target_device_irq
 * sets it up. The alarm is set a microsecond or two ahead and the yield made
 * after a delay that grows by one step each trial, so across a sweep the
 * interrupt lands before the yield, inside it, and after it.
 */

#include <cyros/kernel/core.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <cstddef>
#include <cstdint>

using namespace cyros;

namespace
{

/* --------------------------------------------------------------------------
 * The timer, ticking every microsecond (test_target_device_irq)
 * ----------------------------------------------------------------------- */

constexpr std::uintptr_t set_alias   = 0x2000u;
constexpr std::uintptr_t clear_alias = 0x3000u;

#if defined(__ARM_ARCH_6M__)
constexpr std::uintptr_t timer          = 0x40054000u;
constexpr std::uintptr_t timer_alarm0   = timer + 0x10u;
constexpr std::uintptr_t timer_timerawl = timer + 0x28u;
constexpr std::uintptr_t timer_intr     = timer + 0x34u;
constexpr std::uintptr_t timer_inte     = timer + 0x38u;
#else
constexpr std::uintptr_t timer          = 0x400b0000u;   /* TIMER0 */
constexpr std::uintptr_t timer_alarm0   = timer + 0x10u;
constexpr std::uintptr_t timer_timerawl = timer + 0x28u;
constexpr std::uintptr_t timer_intr     = timer + 0x3cu;
constexpr std::uintptr_t timer_inte     = timer + 0x40u;
constexpr std::uintptr_t ticks_timer0_ctrl   = 0x40108018u;
constexpr std::uintptr_t ticks_timer0_cycles = 0x4010801cu;
constexpr std::uintptr_t resets_reset    = 0x40020000u;
constexpr std::uintptr_t resets_done     = 0x40020008u;
constexpr std::uint32_t  reset_timer0    = 1u << 23;
#endif

volatile std::uint32_t& reg(std::uintptr_t address)
{
   return *reinterpret_cast<volatile std::uint32_t*>(address);
}

std::uint32_t micros() { return reg(timer_timerawl); }

void timer_init()
{
#if !defined(__ARM_ARCH_6M__)
   reg(ticks_timer0_ctrl) = 0u;
   reg(ticks_timer0_cycles) = 12u;
   reg(ticks_timer0_ctrl) = 1u;
   reg(resets_reset + clear_alias) = reset_timer0;
   while ((reg(resets_done) & reset_timer0) == 0u) {}
#endif
   reg(timer_intr) = 1u;
   reg(timer_inte + set_alias) = 1u;
}

void alarm_in(std::uint32_t us) { reg(timer_alarm0) = micros() + us; }

void irq0_enable()
{
#if defined(__riscv)
   asm volatile("csrs 0xbe0, %0" : : "r"(1u << 16) : "memory");
#elif defined(__ARM_ARCH_6M__)
   std::uint32_t const priority = reg(0xE000ED20u) >> 24;
   reg(0xE000E400u) = (reg(0xE000E400u) & ~0xFFu) | priority;           /* IPR0 */
   reg(0xE000E280u) = 1u;                                               /* ICPR */
   reg(0xE000E100u) = 1u;                                               /* ISER */
#else
   std::uint8_t const priority = *reinterpret_cast<volatile std::uint8_t*>(0xE000ED23u);
   *reinterpret_cast<volatile std::uint8_t*>(0xE000E400u) = priority;   /* IPR, IRQ 0 */
   reg(0xE000E280u) = 1u;                                               /* ICPR */
   reg(0xE000E100u) = 1u;                                               /* ISER */
#endif
}

/* A delay of `steps` turns of a loop the compiler must keep. */
void delay(std::uint32_t steps)
{
   for (std::uint32_t i = 0; i < steps; ++i) {
      asm volatile("" ::: "memory");
   }
}

/* --------------------------------------------------------------------------
 * The sequence every party stamps
 * ----------------------------------------------------------------------- */

volatile std::uint32_t sequence = 0u;

/* The stamps, each the sequence number its party last took. */
volatile std::uint32_t woken_at  = 0u;   /* the handler, as it wakes `urgent` */
volatile std::uint32_t equal_ran = 0u;   /* `equal`, each time it runs        */
volatile std::uint32_t driver_ran = 0u;  /* `driver`, before each yield       */

/* Taken by a thread, under a critical section so the handler cannot split it.
 * The handler runs with every interrupt that could split its own held off. */
std::uint32_t stamp()
{
   this_core::critical_guard const guard;
   std::uint32_t const taken = sequence + 1u;
   sequence = taken;
   return taken;
}

volatile int  urgent_runs = 0;
volatile int  overtaken   = 0;   /* urgent ran after another thread had, past the wake */
volatile bool sweeping    = true;

sync::semaphore wake{0};

}  // namespace

extern "C" void isr_irq0(void)
{
   reg(timer_intr) = 1u;   /* clear the alarm, or it is offered again */
   std::uint32_t const taken = sequence + 1u;
   sequence = taken;
   woken_at = taken;
   wake.release();
}

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 1024;
alignas(CYROS_PORT_STACK_ALIGN) std::byte urgent_stack[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte equal_stack[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte driver_stack[stack_size];

/* More urgent than the other two, so its wake preempts whichever runs. */
void urgent()
{
   while (true) {
      wake.acquire();
      this_core::critical_guard const guard;
      if (equal_ran > woken_at || driver_ran > woken_at) {
         overtaken = overtaken + 1;
      }
      urgent_runs = urgent_runs + 1;
   }
}

/* The driver's equal: runs when the driver yields, and yields straight back. */
void equal()
{
   while (sweeping) {
      equal_ran = stamp();
      this_thread::yield();
   }
}

constexpr std::uint32_t sweep_steps = 256u;
constexpr std::uint32_t sweeps      = 16u;

void driver()
{
   timer_init();
   irq0_enable();

   cyros::bench::start("an interrupt's wake runs first, wherever in a yield it lands");
   int missed = 0;
   for (std::uint32_t sweep = 0; sweep < sweeps; ++sweep) {
      for (std::uint32_t step = 0; step < sweep_steps; ++step) {
         int const before = urgent_runs;
         alarm_in(2u);
         delay(step);
         driver_ran = stamp();
         this_thread::yield();
         driver_ran = stamp();
         /* Back from the equal. The alarm may still be ahead, and lands here
          * then, where the wake preempts this loop. */
         std::uint32_t const start = micros();
         while (urgent_runs == before && micros() - start < 100u) {}
         if (urgent_runs == before) {
            missed = missed + 1;
         }
      }
   }
   sweeping = false;

   CYROS_CHECK_EQ(missed, 0);
   CYROS_CHECK_EQ(urgent_runs, static_cast<int>(sweeps * sweep_steps));
   CYROS_CHECK_EQ(overtaken, 0);
   cyros::bench::print("  wakes: ");
   cyros::bench::print_dec(static_cast<std::uint64_t>(urgent_runs));
   cyros::bench::print(", overtaken: ");
   cyros::bench::print_dec(static_cast<std::uint64_t>(overtaken));
   cyros::bench::print("\n");

   cyros::bench::finish();
}

}  // namespace

extern "C" int cyros_bench_main()
{
   cyros::bench::print("an interrupt's wake, swept across a yield\n\n");

   kernel::initialise();
   thread u(urgent, urgent_stack, thread::priority(1), core0);
   thread e(equal,  equal_stack,  thread::priority(3), core0);
   thread d(driver, driver_stack, thread::priority(3), core0);
   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
