/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_multicore_owed_switch.cpp
 * @brief A wake from another core preempts at once, wherever in a switch on
 *        the thread's road its doorbell lands.
 *
 * Subject: the cross-core wake (L5) meeting a switch the kernel decided on the
 *          thread's road and the port owes (cyros_port_yield_to)
 * Trusts:  layers 0 to 4: the harness, scheduling and a waitable's wake
 *
 * On linux_preempt the owed switch and the cross-core doorbell are the same
 * signal. A doorbell sent while this core owes a switch merges with the owed
 * switch's own signal, and the port, which carries the owed switch out without
 * calling the kernel unless something asked for a reschedule, must still learn
 * that another core did. If it does not, the remote wake stays in the intake
 * until this core next switches for some other reason, and a more urgent
 * thread it readied waits behind whatever runs.
 *
 * Core 0 runs two equal threads that hand the core to each other by yielding,
 * and a more urgent one, W, waiting on a bell. Each round the yielder arms the
 * round and yields to the spinner, and core 1 rings the bell after a delay that
 * grows by one step a round, so across a sweep the doorbell lands before the
 * yield, inside it and after it. The spinner does not yield: it spins until W
 * has run, which only a preemption can bring about. A round whose W never ran
 * within a second is a lost preemption.
 */

#include <cyros/kernel/core.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/waitable.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <common/guarded_stack.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>

using namespace cyros;

namespace
{

constexpr std::uint32_t rounds = 20000;
constexpr std::uint32_t delay_steps = 512;

/* One wake a ring, as a semaphore's release: the wake can then never be
 * spent on nothing. */
class bell final : public waitable
{
public:
   void ring() noexcept
   {
      rings.fetch_add(1, std::memory_order_acq_rel);
      wake_one();
   }

   void wait() noexcept { this_thread::wait_on(*this); }

protected:
   bool try_satisfy(waiter_record*) noexcept override
   {
      std::uint32_t current = rings.load(std::memory_order_acquire);
      while (current > 0) {
         if (rings.compare_exchange_weak(current, current - 1, std::memory_order_acq_rel)) {
            return true;
         }
      }
      return false;
   }

private:
   std::atomic<std::uint32_t> rings{0};
};

struct shared_state
{
   bell                       w_bell;
   std::atomic<std::uint32_t> armed{0};     // the round the yielder has started
   std::atomic<std::uint32_t> w_ran{0};     // the last round W ran for
   std::atomic<bool>          done{false};
   std::atomic<bool>          bail{false};
   std::atomic<std::uint32_t> lost{0};      // rounds W did not run in time
};

shared_state* state = nullptr;

bool spin_until_w_ran(std::uint32_t const round)
{
   using clock = std::chrono::steady_clock;
   auto const deadline = clock::now() + std::chrono::seconds(1);
   for (std::uint32_t i = 0;; ++i) {
      if (state->w_ran.load(std::memory_order_acquire) >= round) return true;
      if ((i & 1023u) == 0 && clock::now() > deadline) return false;
      this_core::cpu_relax();
   }
}

}  // namespace

TEST(MultiCoreOwedSwitch_Test, GivenASwitchOwedOnTheThreadsRoad_WhenAnotherCoreWakesAMoreUrgentThread_ThenItPreemptsAtOnce)
{
   kernel::initialise();
   shared_state s;
   state = &s;

   static test::guarded_stack yielder_stack, spinner_stack, w_stack, ringer_stack;

   // Core 0: the yielder and the spinner, equals, hand the core back and forth.
   thread yielder([] {
         for (std::uint32_t r = 1; r <= rounds && !state->bail.load(); ++r) {
            state->armed.store(r, std::memory_order_release);
            this_thread::yield();
         }
         state->done.store(true, std::memory_order_release);
      },
      yielder_stack, thread::priority(3), core0);

   thread spinner([] {
         for (std::uint32_t r = 1; r <= rounds && !state->bail.load(); ++r) {
            if (!spin_until_w_ran(r)) {
               state->lost.fetch_add(1, std::memory_order_relaxed);
               state->bail.store(true, std::memory_order_release);
            }
            this_thread::yield();
         }
      },
      spinner_stack, thread::priority(3), core0);

   // Core 0: the more urgent thread the bell wakes.
   thread w([] {
         while (true) {
            state->w_bell.wait();
            if (state->done.load(std::memory_order_acquire)) return;
            state->w_ran.store(state->armed.load(std::memory_order_acquire), std::memory_order_release);
         }
      },
      w_stack, thread::priority(1), core0);

   // Core 1: rings once a round, the delay sweeping the yield.
   thread ringer([] {
         for (std::uint32_t r = 1; r <= rounds; ++r) {
            while (state->armed.load(std::memory_order_acquire) < r) {
               if (state->bail.load(std::memory_order_acquire) || state->done.load(std::memory_order_acquire)) break;
               this_core::cpu_relax();
            }
            if (state->bail.load(std::memory_order_acquire)) break;
            for (std::uint32_t i = 0; i < r % delay_steps; ++i) this_core::cpu_relax();
            state->w_bell.ring();
         }
         // Let W return, whatever happened above.
         while (!state->done.load(std::memory_order_acquire)) this_core::cpu_relax();
         state->w_bell.ring();
      },
      ringer_stack, thread::priority(1), core1);

   kernel::start();
   kernel::finalise();

   EXPECT_EQ(s.lost.load(), 0u)
      << "a wake from another core did not preempt the spinner: its doorbell was lost in a switch owed on the thread's road";
}
