/**
 * @file test_cortex_m33_smp_channel.cpp
 * @brief A work channel across two real Cortex-M33 cores (L7).
 *
 * Subject: cyros::ch::channel, on hardware rather than on pthreads
 * Trusts:  SMP bring-up (L2), the doorbell (L6) and the semaphore (L6)
 * Proves:  a worker PARKED on core 1 is fed by a producer on core 0, a
 *          producer parked on a full channel is fed by the worker, and stop()
 *          reaches across the boundary too
 *
 *
 * What this adds over the host tests
 * ==================================
 * The host suite runs this channel on the linux ports, where a "core" is a
 * pthread and a cross-core wake is a signal. Everything it proves is therefore
 * conditional on that emulation. Here the cores are real, and the wake is an
 * MHU doorbell interrupt on an SSE-200 subsystem.
 *
 * The blocking is the subject, exactly as in test_cortex_m33_smp_ipi. When
 * core 1's worker blocks in receive() it has nothing else to run and parks in
 * WFI, so it is not executing and nothing it does can observe core 0's send.
 * Only an interrupt from outside starts it again. A polling worker would pass
 * this test with the doorbell removed entirely.
 *
 * The channel holds TWO jobs against fifty posted, so core 0 fills it and
 * parks in send_blocking as well. Both directions of the wake are therefore
 * exercised, and a channel that only ever woke the consumer would hang here.
 *
 *
 * The caveat, and it matters
 * ==========================
 * QEMU does not model ARM's weak memory (arm-port-notes.md section 16). This
 * is a PORTABILITY test, not a memory-model test: it upgrades "the channel has
 * run on x86-64 with pthreads for cores" to "it has also run on two genuine
 * Cortex-M33 cores with a real inter-core interrupt". A missing barrier would
 * pass here exactly as silently as it passes on the Linux ports.
 *
 *
 * How this fails
 * ==============
 * A lost wakeup leaves both cores parked with no thread able to run, so no
 * core is left to notice and report, and the runner's timeout is what catches
 * it. Progress is printed as it goes so the timeout is still informative:
 * stopping at job 0 means the doorbell never worked, stopping later means
 * wakeups are being lost, and reaching the stop but never finishing means the
 * shutdown path is what broke. Printing happens only on core 0, so the two
 * cores never contend for the semihosting trap.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/ch/channel.hpp>
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

/* Enough that a doorbell which works once but not reliably shows up, and few
 * enough to stay well inside the runner's timeout. Fifty jobs through two
 * slots is at least forty-eight times the channel is full and forty-eight
 * times it is empty, so both parks happen many times over. */
constexpr int jobs_total    = 50;
constexpr int progress_every = 10;

/* Two slots against fifty jobs, deliberately. A channel big enough to hold the
 * run would only ever park the consumer. */
ch::work_channel<2> work;

std::atomic<int>  jobs_run{0};
std::atomic<bool> ran_on_wrong_core{false};
std::atomic<bool> worker_returned{false};

/* Core 0 cannot join core 1 on a target port, since kernel::start() never
 * returns here, so it waits on a flag instead. Bounded, so a shutdown that
 * never completes is still reported rather than spun on forever. */
constexpr std::uint32_t worker_exit_budget = 20'000'000u;

void thread_on_core1()
{
   /* Blocks in receive(). Core 1 has nothing else runnable, so it parks and
    * only a doorbell from core 0 can start it again. */
   ch::run(work);
   worker_returned.store(true, std::memory_order_release);
}

void thread_on_core0()
{
   cyros::bench::print("  posting: ");

   for (int i = 0; i < jobs_total; ++i) {
      /* Captureless, so it fits the default 32 byte job with room to spare and
       * the no_heap policy is satisfied at compile time. */
      work.send_blocking([] {
         if (this_core::id() != 1u) {
            ran_on_wrong_core.store(true, std::memory_order_relaxed);
         }
         jobs_run.fetch_add(1, std::memory_order_relaxed);
      });

      if ((i + 1) % progress_every == 0) {
         cyros::bench::print_hex(static_cast<std::uint32_t>(i + 1));
         cyros::bench::print(" ");
      }
   }
   cyros::bench::print("\n");

   work.stop();

   bool returned = false;
   for (std::uint32_t i = 0; i < worker_exit_budget; ++i) {
      if (worker_returned.load(std::memory_order_acquire)) { returned = true; break; }
      this_core::cpu_relax();
   }

   cyros::bench::start("every posted job ran");
   CYROS_CHECK_EQ(static_cast<std::uint32_t>(jobs_run.load(std::memory_order_relaxed)),
                  static_cast<std::uint32_t>(jobs_total));

   /* The check that says the work really did cross a core boundary. Without it
    * every job above could have run on core 0 and passed. */
   cyros::bench::start("every job ran on the OTHER core");
   CYROS_CHECK(!ran_on_wrong_core.load(std::memory_order_relaxed));
   CYROS_CHECK_EQ(this_core::id(), 0u);

   cyros::bench::start("stop() reached the parked worker and run() returned");
   CYROS_CHECK(returned);

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("cortex_m33_smp work channel across two cores, mps2-an521\n\n");

   kernel::initialise();

   thread t0(thread_on_core0, stack_core0, thread::priority(0), core0);
   thread t1(thread_on_core1, stack_core1, thread::priority(0), core1);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
