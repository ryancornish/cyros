/**
 * @file test_target_smp_condition_variable.cpp
 * @brief condition_variable and event_flags across two real Cortex-M33 cores (L8).
 *
 * Subject: cyros::sync::condition_variable and cyros::sync::event_flags, on
 *          hardware rather than on pthreads
 * Trusts:  SMP bring-up (L2), the doorbell (L6), the semaphore (L6) and the
 *          mutex (L7)
 * Proves:  a waiter PARKED on one core is released by a notify or a set from
 *          the other, in both directions, many times over
 *
 * What this adds over the host tests: there a "core" is a pthread and a
 * cross-core wake is a signal. Here the wake is an MHU doorbell interrupt, and
 * a waiter that blocks has nothing else to run, so its core parks in WFI and
 * only that interrupt starts it again. Every round of both ping-pongs is a
 * block on one core released by the other.
 *
 * The notifies are issued AFTER unlocking, the case the standard allows and a
 * sequence-number condition variable gets wrong.
 *
 * QEMU does not model ARM's weak memory (arm-port-notes.md section 16), so
 * this is a PORTABILITY test, not a memory-model test.
 *
 * How this fails: a lost wakeup leaves both cores parked, so the runner's
 * timeout catches it. Progress is printed from core 0 as it goes: the phase
 * and round it stops at say which primitive lost the wake.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/sync/condition_variable.hpp>
#include <cyros/sync/event_flags.hpp>
#include <cyros/sync/mutex.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>

using namespace cyros;

static_assert(config::cores == 2, "This test is dual core");
static_assert(CYROS_PORT_CORE_COUNT == 2, "Needs the cortex_m33_smp port");

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 2048;

alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_core0[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_core1[stack_size];

constexpr int rounds         = 100;
constexpr int progress_every = 25;

sync::mutex              m;
sync::condition_variable cv;
int turn = 0;                   // guarded by m: 1 means core 1's move

sync::event_flags flags{0};
constexpr std::uint32_t to_core1 = 0b01;
constexpr std::uint32_t to_core0 = 0b10;

std::atomic<int>  cv_rounds_on_core1{0};
std::atomic<int>  flag_rounds_on_core1{0};
std::atomic<bool> wrong_core{false};
std::atomic<bool> core1_done{false};

constexpr std::uint32_t exit_budget = 20'000'000u;

void thread_on_core1()
{
   for (int i = 0; i < rounds; ++i) {
      m.lock();
      while (turn != 1) {
         cv.wait(m);            // parks: core 1 has nothing else to run
      }
      turn = 0;
      m.unlock();
      cv.notify_one();
      cv_rounds_on_core1.fetch_add(1, std::memory_order_relaxed);
   }

   for (int i = 0; i < rounds; ++i) {
      (void)flags.wait(to_core1, flags_match::any, flags_exit::consume);
      flag_rounds_on_core1.fetch_add(1, std::memory_order_relaxed);
      flags.set(to_core0);
   }

   if (this_core::id() != 1u) {
      wrong_core.store(true, std::memory_order_relaxed);
   }
   core1_done.store(true, std::memory_order_release);
}

void thread_on_core0()
{
   cyros::bench::print("  condition_variable rounds: ");
   for (int i = 0; i < rounds; ++i) {
      m.lock();
      turn = 1;
      m.unlock();
      cv.notify_one();

      m.lock();
      while (turn != 0) {
         cv.wait(m);
      }
      m.unlock();
      if ((i + 1) % progress_every == 0) {
         cyros::bench::print_hex(static_cast<std::uint32_t>(i + 1));
         cyros::bench::print(" ");
      }
   }

   cyros::bench::print("\n  event_flags rounds: ");
   for (int i = 0; i < rounds; ++i) {
      flags.set(to_core1);
      (void)flags.wait(to_core0, flags_match::any, flags_exit::consume);
      if ((i + 1) % progress_every == 0) {
         cyros::bench::print_hex(static_cast<std::uint32_t>(i + 1));
         cyros::bench::print(" ");
      }
   }
   cyros::bench::print("\n");

   bool finished = false;
   for (std::uint32_t i = 0; i < exit_budget; ++i) {
      if (core1_done.load(std::memory_order_acquire)) { finished = true; break; }
      this_core::cpu_relax();
   }

   cyros::bench::start("every condition_variable round crossed to core 1 and back");
   CYROS_CHECK_EQ(static_cast<std::uint32_t>(cv_rounds_on_core1.load()), static_cast<std::uint32_t>(rounds));

   cyros::bench::start("every event_flags round crossed to core 1 and back");
   CYROS_CHECK_EQ(static_cast<std::uint32_t>(flag_rounds_on_core1.load()), static_cast<std::uint32_t>(rounds));
   CYROS_CHECK_EQ(flags.peek(), 0u);

   cyros::bench::start("the waiter ran on the OTHER core, and finished");
   CYROS_CHECK(!wrong_core.load(std::memory_order_relaxed));
   CYROS_CHECK_EQ(this_core::id(), 0u);
   CYROS_CHECK(finished);

   cyros::bench::finish();
}

} // namespace

extern "C" int cyros_bench_main()
{
   cyros::bench::print("condition_variable and event_flags across two cores, mps2-an521\n\n");

   kernel::initialise();

   thread t0(thread_on_core0, stack_core0, thread::priority(0), core0);
   thread t1(thread_on_core1, stack_core1, thread::priority(0), core1);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
