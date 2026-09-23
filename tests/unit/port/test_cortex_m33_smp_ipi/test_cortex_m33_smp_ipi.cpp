/**
 * @file test_cortex_m33_smp_ipi.cpp
 * @brief The inter-core doorbell, exercised by blocking (L6).
 *
 * Subject: cyros_port_send_reschedule_ipi across two real cores
 * Trusts:  SMP bring-up (L2) and the semaphore (L6)
 * Proves:  a thread BLOCKED on one core is woken by a release on the other,
 *          repeatedly and in both directions
 *
 *
 * Why this needs blocking, and why polling would prove nothing
 * ===========================================================
 * A cross-core semaphore ping-pong is the smallest thing that cannot work
 * without the doorbell. When core 1's thread blocks, core 1 has nothing else
 * to run and parks in WFI. Nothing it does can observe core 0's release: it is
 * not executing. Only an interrupt from outside starts it again, and on this
 * subsystem that interrupt is the MHU.
 *
 * Replacing the blocking acquire with a poll would make this test pass with
 * the doorbell removed entirely, because a spinning core needs no wakeup. So
 * the blocking call IS the subject, not an implementation detail of it.
 *
 *
 * How this fails
 * ==============
 * By HANGING, and that is inherent rather than a shortcoming of the test. A
 * lost wakeup leaves both cores parked with no thread able to run, so there is
 * no core left to notice and report. The runner's timeout is what catches it.
 *
 * What the test can do is make the timeout informative, so progress is printed
 * as it goes and the log says how far the ping-pong got. Stopping at round 0
 * means the doorbell never worked at all; stopping later means it works and
 * something is losing wakeups, which is a completely different investigation.
 * Printing only ever happens on core 0, so the two cores never contend for the
 * semihosting trap.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <common/arm/bench.hpp>

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

/* Enough rounds that a doorbell which works once but not reliably shows up,
 * and few enough that the whole test stays well inside the runner's timeout.
 * Every round is two cross-core wakeups, so this is 200 of them. */
constexpr int rounds = 100;

constexpr int progress_every = 25;

sync::semaphore to_core1{0};
sync::semaphore to_core0{0};

/* Written by the core named, read by the other after the ping-pong has ended,
 * so a plain relaxed atomic is enough: the semaphore handoffs carry all the
 * ordering that matters. */
std::atomic<int> rounds_done_core0{0};
std::atomic<int> rounds_done_core1{0};
std::atomic<std::uint32_t> core1_ran_on{0xFFFFFFFFu};

void thread_on_core1()
{
   core1_ran_on.store(this_core::id(), std::memory_order_relaxed);

   for (int i = 0; i < rounds; ++i) {
      /* Blocks. Core 1 has nothing else runnable, so it parks here and only a
       * doorbell from core 0 can start it again. */
      to_core1.acquire();
      rounds_done_core1.store(i + 1, std::memory_order_relaxed);
      to_core0.release();
   }
}

void thread_on_core0()
{
   cyros::bench::print("  ping-pong: ");

   for (int i = 0; i < rounds; ++i) {
      to_core1.release();
      to_core0.acquire();     /* blocks the same way, in the other direction */
      rounds_done_core0.store(i + 1, std::memory_order_relaxed);

      if ((i + 1) % progress_every == 0) {
         cyros::bench::print_hex(static_cast<std::uint32_t>(i + 1));
         cyros::bench::print(" ");
      }
   }
   cyros::bench::print("\n");

   cyros::bench::start("every round completed in both directions");
   CYROS_CHECK_EQ(rounds_done_core0.load(std::memory_order_relaxed), rounds);
   CYROS_CHECK_EQ(rounds_done_core1.load(std::memory_order_relaxed), rounds);

   /* The check that says the wakeups really did cross a core boundary. Without
    * it every round above could have run on one core and passed. */
   cyros::bench::start("the blocking partner really was on the other core");
   CYROS_CHECK_EQ(this_core::id(), 0u);
   CYROS_CHECK_EQ(core1_ran_on.load(std::memory_order_relaxed), 1u);

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("cortex_m33_smp inter-core doorbell, mps2-an521\n\n");

   kernel::initialise();

   thread t0(thread_on_core0, stack_core0, thread::priority(0), core0);
   thread t1(thread_on_core1, stack_core1, thread::priority(0), core1);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
