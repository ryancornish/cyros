/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_target_interrupt_wake.cpp
 * @brief A wake made in an interrupt decides its switch there, on the target.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: an interrupt's wake (scheduler::interrupt_reschedule) and each
 *          port's half of it, cyros_port_interrupted_at_baseline and
 *          cyros_port_owe_switch, where the switch is a real exception or
 *          trap.
 * Trusts:  layers 0 to 4: the port, the time source's tick, the kernel's
 *          threads, suspend and resume, and a waitable's wake.
 * Proves:  that the threads an interrupt wakes run most urgent first, one
 *          decided and then overtaken in the same interrupt keeping its place
 *          at the front of its level. That a wake made while the interrupted
 *          thread has preemption disabled waits for the enable, the kernel's
 *          running thread still that thread meanwhile. That a wake a thread
 *          makes inside a critical section likewise leaves it running until
 *          the section ends. And that an interrupt that both decides a switch
 *          and asks for a reschedule gets both, the switch first.
 *
 * The interrupt is the kernel's own tick, through a callback time::schedule_at
 * files, so the test runs on every port's timer as it is. In each case the
 * driver, the least urgent thread, spins until the callback has run, so the
 * interrupt always takes it at baseline (in the second case, with preemption
 * disabled). The others suspend, or wait, until the callback wakes them, and
 * each notes itself in one shared order when it runs.
 *
 * Every order here is also what the handler's road gives, which pends each
 * wake and decides once at the interrupt's exit. The cases exist for what a
 * decision made inside the interrupt could get wrong instead: filing the
 * thread it overtakes at the back of its level, deciding when the port cannot
 * switch, and losing the reschedule asked for beside a switch owed. Where a
 * port makes the switch late anyway (PendSV, a signal), a decision made where
 * none may be still shows: the kernel's running thread is then not the one
 * executing for the rest of the region, which this_thread::id() reads.
 */

#include <cyros/kernel/core.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/waitable.hpp>
#include <cyros/time/time.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>

using namespace cyros;

namespace
{

constexpr std::uint32_t tick_hz = 1'000;

/* Bounded, so a callback that never runs fails the case instead of hanging
 * the suite. */
constexpr std::uint32_t spin_limit = 40'000'000;

/* --------------------------------------------------------------------------
 * The order the threads ran in
 * ----------------------------------------------------------------------- */

constexpr std::uint32_t x1 = 1u;          /* priority 10, created before x2 */
constexpr std::uint32_t x2 = 2u;          /* priority 10                    */
constexpr std::uint32_t b = 3u;           /* priority 5                     */
constexpr std::uint32_t y = 4u;           /* priority 3, waits on the gate  */
constexpr std::uint32_t driver = 9u;      /* priority 20, the interrupted   */

constexpr std::uint32_t max_notes = 8u;
volatile std::uint32_t notes[max_notes] = {};
volatile std::uint32_t note_count = 0u;

/* Under a critical section, so the interrupt cannot split it. */
void note(std::uint32_t who)
{
   this_core::critical_guard const guard;
   std::uint32_t const at = note_count;
   if (at < max_notes) {
      notes[at] = who;
   }
   note_count = at + 1u;
}

template <std::size_t N>
void expect_order(std::uint32_t const (&expected)[N])
{
   cyros::bench::print("  order = [");
   for (std::uint32_t i = 0; i < note_count && i < max_notes; ++i) {
      cyros::bench::print_dec(notes[i]);
      if (i + 1u < note_count) {
         cyros::bench::print(", ");
      }
   }
   cyros::bench::print("]\n");

   CYROS_CHECK_EQ(note_count, static_cast<std::uint32_t>(N));
   for (std::uint32_t i = 0; i < N && i < max_notes; ++i) {
      CYROS_CHECK_EQ(notes[i], expected[i]);
   }
}

/* --------------------------------------------------------------------------
 * A gate: a batch wake, which pends one reschedule after readying its waiters
 * ----------------------------------------------------------------------- */

class gate final : public waitable
{
public:
   void open() noexcept
   {
      opens.fetch_add(1u, std::memory_order_acq_rel);
      wake_all();
   }

   void pass() noexcept { this_thread::wait_on(*this); }

protected:
   bool try_satisfy(waiter_record*) noexcept override
   {
      std::uint32_t current = opens.load(std::memory_order_acquire);
      while (current > 0u) {
         if (opens.compare_exchange_weak(current, current - 1u, std::memory_order_acq_rel)) {
            return true;
         }
      }
      return false;
   }

private:
   std::atomic<std::uint32_t> opens{0u};
};

gate the_gate;

/* --------------------------------------------------------------------------
 * The threads, and the callbacks the tick runs
 * ----------------------------------------------------------------------- */

thread* thread_x1 = nullptr;
thread* thread_x2 = nullptr;
thread* thread_b  = nullptr;

void x1_entry()
{
   this_thread::suspend();
   note(x1);
}

void x2_entry()
{
   this_thread::suspend();
   note(x2);
}

void b_entry()
{
   for (int round = 0; round < 4; ++round) {
      this_thread::suspend();
      note(b);
   }
}

void y_entry()
{
   the_gate.pass();
   note(y);
}

volatile bool fired = false;
thread::id driver_id = 0;

/* x1 is decided, x2 is its equal, and b overtakes x1. */
void wake_x1_x2_then_b(void*)
{
   thread_x1->resume();
   thread_x2->resume();
   thread_b->resume();
   fired = true;
}

void wake_b(void*)
{
   thread_b->resume();
   fired = true;
}

/* b is decided, then the gate's batch wake readies y, more urgent, and pends. */
void wake_b_then_open_the_gate(void*)
{
   thread_b->resume();
   the_gate.open();
   fired = true;
}

/* Has the tick run CALLBACK two ticks from now, spinning until it has. */
bool run_and_wait(time::callback callback)
{
   note_count = 0u;
   fired = false;
   time::handle const handle = time::schedule_at(time::now() + time::duration{2}, callback, nullptr);
   if (handle.id == 0) {
      return false;
   }
   for (std::uint32_t i = 0; i < spin_limit; ++i) {
      if (fired) {
         return true;
      }
   }
   return false;
}

void driver_entry()
{
   time::start();

   cyros::bench::start("an interrupt's wakes run most urgent first, the overtaken one at the front of its level");
   CYROS_CHECK(run_and_wait(&wake_x1_x2_then_b));
   note(driver);
   expect_order({b, x1, x2, driver});

   cyros::bench::start("a wake made with preemption disabled waits for the enable");
   bool still_the_driver = false;
   {
      this_core::preemption_guard const no_preemption;
      CYROS_CHECK(run_and_wait(&wake_b));
      still_the_driver = this_thread::id() == driver_id;
      note(driver);
   }
   CYROS_CHECK(still_the_driver);
   expect_order({driver, b});

   cyros::bench::start("a wake a thread makes inside a critical section leaves it running until the section ends");
   note_count = 0u;
   still_the_driver = false;
   {
      this_core::critical_guard const masked;
      thread_b->resume();
      still_the_driver = this_thread::id() == driver_id;
      note(driver);
   }
   CYROS_CHECK(still_the_driver);
   expect_order({driver, b});

   cyros::bench::start("an interrupt that owes a switch and asks for a reschedule gets both, the switch first");
   CYROS_CHECK(run_and_wait(&wake_b_then_open_the_gate));
   note(driver);
   expect_order({y, b, driver});

   cyros::bench::finish();
}

constexpr std::size_t stack_size = thread::min_stack_size + 1024;
alignas(CYROS_PORT_STACK_ALIGN) std::byte x1_stack[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte x2_stack[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte b_stack[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte y_stack[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte driver_stack[stack_size];

}  // namespace

extern "C" int cyros_bench_main()
{
   cyros::bench::print("an interrupt's wake, deciding its switch\n\n");

   kernel::initialise();
   time::initialise(tick_hz);

   /* Each runs first and parks, in this order, leaving the driver running. */
   thread ty(y_entry, y_stack, thread::priority(3), core0);
   thread tb(b_entry, b_stack, thread::priority(5), core0);
   thread tx1(x1_entry, x1_stack, thread::priority(10), core0);
   thread tx2(x2_entry, x2_stack, thread::priority(10), core0);
   thread td(driver_entry, driver_stack, thread::priority(20), core0);
   thread_x1 = &tx1;
   thread_x2 = &tx2;
   thread_b  = &tb;
   driver_id = td.get_id();

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
