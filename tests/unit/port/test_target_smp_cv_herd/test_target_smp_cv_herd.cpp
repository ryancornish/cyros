/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_target_smp_cv_herd.cpp
 * @brief What notify_all's herd costs when the waiters are on the other core.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: cyros::sync::condition_variable's notify_all, across two cores.
 * Trusts:  layers 0 to 7: the SMP bring-up, the cross-core wake, the semaphore
 *          and the mutex.
 * Proves:  that a notify_all on core 0 releases every waiter parked on core 1,
 *          each re-locking in priority order, in every round, and MEASURES what
 *          that costs against a mutex handover chain to the same waiters.
 *
 *
 * THE MEASUREMENT
 * ===============
 * test_target_condition_variable's three shapes, with the waiters moved to the
 * other core:
 *
 *    chain     the waiters are parked in lock(), the driver unlocks
 *    locked    the waiters are parked in wait(), the driver notifies UNDER the
 *              mutex, then unlocks
 *    unlocked  the same, notifying AFTER it unlocks
 *
 *    herd - chain  =  what not requeueing costs, across the cores
 *
 * The single-core test reads its clock on the driver, which every waiter
 * outranks. Here the driver's release returns while core 1 is still working,
 * so the waiter that finishes the round reads the clock instead, after its
 * unlock, and the figure runs from the driver's release to the last waiter's
 * unlock. That needs a timestamp both cores share: MTIME on the RP2350, the
 * TIMER on the RP2040. QEMU's figures are zero (Arm) or host time (RISC-V), and
 * there only the checks mean anything.
 *
 * Core 1 runs nothing but the waiters (priorities 2 and up), so once released
 * they finish most urgent first, as on one core. Each round enlists the
 * waiters and lets core 1 park them before the clock starts. A waiter not yet
 * parked when the release comes still completes, so the order and completion
 * checks hold whatever the timing, and only that round's figure would suffer.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/sync/condition_variable.hpp>
#include <cyros/sync/mutex.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

using namespace cyros;

static_assert(config::cores == 2, "This test is dual core");

namespace
{

constexpr std::size_t  stack_size      = thread::min_stack_size + 1024;
constexpr std::size_t  max_waiters     = 8;
constexpr int          rounds          = 16;
constexpr std::uint8_t driver_priority = 20;

/* Spins of cpu_relax: the settle lets core 1 park the enlisted waiters (about
 * 1 ms at 150 MHz, where a park takes microseconds), the budget bounds a round
 * that never finishes, so a lost wake fails a check rather than hanging. */
constexpr std::uint32_t settle_spins = 200'000u;
constexpr std::uint32_t round_budget = 20'000'000u;

enum class shape : std::uint8_t { chain, locked, unlocked };

alignas(CYROS_PORT_STACK_ALIGN) std::byte waiter_stacks[max_waiters][stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte driver_stack[stack_size];

sync::mutex              m;
sync::condition_variable cv;

std::array<sync::semaphore, max_waiters> enlist{
   sync::semaphore{0}, sync::semaphore{0}, sync::semaphore{0}, sync::semaphore{0},
   sync::semaphore{0}, sync::semaphore{0}, sync::semaphore{0}, sync::semaphore{0},
};

// Written by the driver before it enlists, which orders them for the waiters.
shape       round_shape = shape::chain;
std::size_t round_size  = 0;

bool go        = false;   // guarded by m
int  completed = 0;       // guarded by m
std::array<int, max_waiters> order{};   // guarded by m

// The waiter that finishes a round stamps it, then publishes the round's number.
std::uint64_t         finished_at = 0;
std::atomic<int>      finished_round{-1};
int                   round_number = 0;  // the driver's, read by the finisher under m
std::atomic<bool>     wrong_core{false};

void waiter(std::size_t const index)
{
   while (true) {
      enlist[index].acquire();
      if (this_core::id() != 1u) {
         wrong_core.store(true, std::memory_order_relaxed);
      }

      m.lock();
      if (round_shape != shape::chain) {
         while (!go) {
            cv.wait(m);
         }
      }
      order[static_cast<std::size_t>(completed)] = static_cast<int>(index);
      ++completed;
      bool const last = completed == static_cast<int>(round_size);
      int const number = round_number;
      m.unlock();

      if (last) {
         finished_at = cyros_port_timestamp();
         finished_round.store(number, std::memory_order_release);
      }
   }
}

void waiter_0() { waiter(0); }
void waiter_1() { waiter(1); }
void waiter_2() { waiter(2); }
void waiter_3() { waiter(3); }
void waiter_4() { waiter(4); }
void waiter_5() { waiter(5); }
void waiter_6() { waiter(6); }
void waiter_7() { waiter(7); }

struct round_result
{
   bool          finished;
   bool          in_order;
   std::uint64_t ticks;
};

round_result run_round(shape const s, std::size_t const n)
{
   round_shape = s;
   round_size  = n;

   m.lock();
   go = false;
   completed = 0;
   ++round_number;
   int const number = round_number;
   if (s != shape::chain) {
      m.unlock();
   }

   for (std::size_t i = 0; i < n; ++i) {
      enlist[i].release();
   }
   for (std::uint32_t i = 0; i < settle_spins; ++i) {
      this_core::cpu_relax();
   }

   std::uint64_t const t0 = cyros_port_timestamp();
   switch (s) {
      case shape::chain:
         m.unlock();
         break;
      case shape::locked:
         m.lock();
         go = true;
         cv.notify_all();
         m.unlock();
         break;
      case shape::unlocked:
         m.lock();
         go = true;
         m.unlock();
         cv.notify_all();
         break;
   }

   round_result result{false, false, 0};
   for (std::uint32_t i = 0; i < round_budget; ++i) {
      if (finished_round.load(std::memory_order_acquire) == number) {
         result.finished = true;
         break;
      }
      this_core::cpu_relax();
   }
   if (!result.finished) {
      return result;
   }

   m.lock();
   result.in_order = completed == static_cast<int>(n);
   for (std::size_t i = 0; result.in_order && i < n; ++i) {
      result.in_order = order[i] == static_cast<int>(i);
   }
   m.unlock();
   result.ticks = finished_at - t0;
   return result;
}

void driver()
{
   constexpr std::array<std::size_t, 4> sizes{ 1, 2, 4, 8 };
   constexpr std::array<shape, 3> shapes{ shape::chain, shape::locked, shape::unlocked };
   constexpr std::array<char const*, 3> names{ "chain   ", "locked  ", "unlocked" };

   std::uint64_t best[4][3]{};
   std::uint64_t total[4][3]{};
   bool all_finished = true;
   bool all_in_order = true;

   for (std::size_t z = 0; z < sizes.size() && all_finished; ++z) {
      for (std::size_t k = 0; k < shapes.size() && all_finished; ++k) {
         best[z][k] = ~std::uint64_t{0};
         for (int r = 0; r < rounds && all_finished; ++r) {
            auto const result = run_round(shapes[k], sizes[z]);
            all_finished = result.finished;
            all_in_order = all_in_order && result.in_order;
            best[z][k] = result.ticks < best[z][k] ? result.ticks : best[z][k];
            total[z][k] += result.ticks;
         }
      }
   }

   cyros::bench::start("every round finished: the release reached every waiter on core 1");
   CYROS_CHECK(all_finished);

   cyros::bench::start("and they completed most urgent first, every one");
   CYROS_CHECK(all_in_order);

   cyros::bench::start("the waiters ran on core 1, the driver on core 0");
   CYROS_CHECK(!wrong_core.load(std::memory_order_relaxed));
   CYROS_CHECK_EQ(this_core::id(), 0u);

   if (all_finished) {
      cyros::bench::start("release on core 0 to last unlock on core 1, timestamp ticks (min / mean over 16 rounds)");
      for (std::size_t z = 0; z < sizes.size(); ++z) {
         for (std::size_t k = 0; k < shapes.size(); ++k) {
            cyros::bench::print("  N=");
            cyros::bench::print_dec(sizes[z]);
            cyros::bench::print("  ");
            cyros::bench::print(names[k]);
            cyros::bench::print("  ");
            cyros::bench::print_dec(best[z][k]);
            cyros::bench::print(" / ");
            cyros::bench::print_dec(total[z][k] / rounds);
            cyros::bench::print("\n");
         }
      }
      if (best[0][0] == 0) {
         cyros::bench::print("  every figure is zero: no shared timestamp here\n");
      }
   }

   cyros::bench::finish();
}

} // namespace

extern "C" int cyros_bench_main()
{
   cyros::bench::print("condition_variable: notify_all's herd across two cores\n\n");

   kernel::initialise();

   constexpr std::array<void (*)(), max_waiters> entries{
      waiter_0, waiter_1, waiter_2, waiter_3, waiter_4, waiter_5, waiter_6, waiter_7,
   };
   std::array<thread, max_waiters> waiters{};
   for (std::size_t i = 0; i < max_waiters; ++i) {
      waiters[i] = thread(entries[i], waiter_stacks[i], thread::priority(static_cast<std::uint8_t>(2 + i)), core1);
   }
   thread driving(driver, driver_stack, thread::priority(driver_priority), core0);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
