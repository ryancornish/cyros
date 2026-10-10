/**
 * @file test_target_smp_atomics.cpp
 * @brief Atomic read-modify-writes are atomic BETWEEN the cores (L0, debt to L2).
 *
 * Subject: the port's promise, made by its core bring-up, that the ISA's
 *          atomic instructions exclude each other across cores. Every
 *          std::atomic the kernel uses, its spinlocks first, rests on it.
 * Trusts:  the two-core bring-up (layer 2), only to put one thread on each
 *          core. Hence the declared harness debt.
 * Proves:  that two cores incrementing one word, by fetch_add and by a
 *          compare-exchange loop, lose nothing.
 *
 *
 * WHY A TEST OF ITS OWN
 * =====================
 * On the RP2350's Cortex-M33s, LDREX/STREX (and the LDAEX/STLEX GCC emits for
 * std::atomic) are NOT atomic across the two cores until each core sets
 * ACTLR.EXTEXCLALL. Without it 2 x 1,000,000 contended increments lost about
 * 200,000, and no STREX ever failed, so nothing retries and nothing reports
 * (rp2350-notes.md 3b). The rp2350_m33_smp target sets it on each core.
 *
 * Deleting that line passed every other two-core test, on the board, which is
 * why this one exists: the kernel's spinlocks are never contended hard enough
 * there to lose an update visibly. This test contends one word as hard as both
 * cores can, so the defect shows as a count, every run.
 *
 * QEMU's exclusives are always global and Hazard3's AMOs and LR/SC need no
 * setting (rp2350-notes.md 5c), so on those it is a regression check: it has
 * one way to fail, and a port that never had the defect passes.
 *
 *
 * WHAT IS REPORTED BUT NOT CHECKED
 * ================================
 * How many compare-exchanges had to retry. On the board that is the evidence
 * the two loops really overlapped, because a correct monitor fails a store the
 * other core interleaved. It is not a check: QEMU interleaves its harts in time
 * slices, so a correct run there can retry rarely or never.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>

using namespace cyros;

static_assert(config::cores == 2, "This test is dual core");
static_assert(CYROS_PORT_CORE_COUNT == 2, "Needs a two-core port");

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 2048;

alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_core0[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_core1[stack_size];

/* Increments per core, for each of the two counters. The board's broken mode
 * lost about one in ten, so even a short overlap loses thousands, and a run
 * takes a few tens of milliseconds there. */
constexpr std::uint32_t increments = 200'000u;

/* The two words under test, on separate cache lines from each other and from
 * the handshake below, so false sharing cannot stand in for a lost update. */
alignas(CYROS_PORT_CACHE_LINE) std::atomic<std::uint32_t> added{0u};
alignas(CYROS_PORT_CACHE_LINE) std::atomic<std::uint32_t> exchanged{0u};

alignas(CYROS_PORT_CACHE_LINE) std::atomic<bool> core1_ready{false};
std::atomic<bool> go{false};
std::atomic<bool> core1_finished{false};
std::atomic<std::uint32_t> core1_retries{0u};

/* Bounded waits, so a core that never arrives REPORTS rather than hanging
 * until the runner's timeout, as in test_cortex_m33_smp_bringup. Bounded on
 * PROGRESS where there is any: a fixed budget of yields ran out on QEMU under
 * host load, with core 1 hammering away but slowly (31 of 60 runs at 4-way,
 * 2026-10-10), and that reported a slow core as a broken one. */
constexpr std::uint32_t rendezvous_limit = 100'000u;

/* Core 1 reaching its thread: nothing to watch until it does, so a budget far
 * past any load seen. */
bool wait_for_ready()
{
   for (std::uint32_t spin = 0; spin < 100u * rendezvous_limit; ++spin) {
      if (core1_ready.load(std::memory_order_acquire)) { return true; }
      this_thread::yield();
   }
   return false;
}

/* Core 1 finishing, once core 0 has: core 1 is the only one moving the counter
 * now, so the budget restarts whenever it moves, and only a core 1 that stops
 * making progress runs it out. */
bool wait_for_finish(std::atomic<std::uint32_t> const& counter)
{
   std::uint32_t last = counter.load(std::memory_order_relaxed);
   std::uint32_t still = 0u;
   while (!core1_finished.load(std::memory_order_acquire)) {
      this_thread::yield();
      std::uint32_t const now = counter.load(std::memory_order_relaxed);
      if (now != last) {
         last = now;
         still = 0u;
      } else if (++still == rendezvous_limit) {
         return false;
      }
   }
   return true;
}

/* A pause of 0 to 15 steps, from a per-core xorshift sequence. Two cores
 * running the same loop otherwise fall into step, and their exclusive windows
 * never overlap: measured on the board, 2 to 4 retries in 400,000, and the
 * broken monitor lost nothing. Sliding one core's phase against the other's is
 * what the probe that found the defect did (core 1 jittered 0 to 15 cycles a
 * step, rp2350-notes.md 3b). */
void jitter(std::uint32_t& state)
{
   state ^= state << 13;
   state ^= state >> 17;
   state ^= state << 5;
   for (std::uint32_t step = state & 15u; step != 0u; --step) {
      asm volatile("" ::: "memory");
   }
}

/* The work both cores do, with neither yielding inside it, so the two loops
 * run truly at once. Returns how many compare-exchanges had to retry. */
std::uint32_t hammer(std::uint32_t seed)
{
   std::uint32_t retries = 0u;
   for (std::uint32_t i = 0; i < increments; ++i) {
      added.fetch_add(1u, std::memory_order_relaxed);
      jitter(seed);

      std::uint32_t seen = exchanged.load(std::memory_order_relaxed);
      while (!exchanged.compare_exchange_weak(seen, seen + 1u, std::memory_order_relaxed)) {
         ++retries;
      }
      jitter(seed);
   }
   return retries;
}

void thread_on_core1()
{
   core1_ready.store(true, std::memory_order_release);
   /* Spun on rather than yielded on: the start must be as close to core 0's
    * as the hardware allows. Bounded all the same. */
   for (std::uint32_t spin = 0; !go.load(std::memory_order_acquire); ++spin) {
      if (spin == 100'000'000u) { return; }
   }
   core1_retries.store(hammer(0x9e3779b9u), std::memory_order_relaxed);
   core1_finished.store(true, std::memory_order_release);
}

void thread_on_core0()
{
   bool const ready = wait_for_ready();
   go.store(true, std::memory_order_release);
   std::uint32_t const core0_retries = ready ? hammer(0x2545f491u) : 0u;
   bool const finished = ready && wait_for_finish(added);

   cyros::bench::start("both cores reached their threads and finished");
   CYROS_CHECK(ready);
   CYROS_CHECK(finished);

   std::uint32_t const expected = 2u * increments;
   std::uint32_t const by_add = added.load(std::memory_order_relaxed);
   std::uint32_t const by_exchange = exchanged.load(std::memory_order_relaxed);
   std::uint32_t const retries = core0_retries + core1_retries.load(std::memory_order_relaxed);

   cyros::bench::print("  expected        ");
   cyros::bench::print_hex(expected);
   cyros::bench::print("\n  fetch_add       ");
   cyros::bench::print_hex(by_add);
   cyros::bench::print("\n  compare_exchange ");
   cyros::bench::print_hex(by_exchange);
   cyros::bench::print("\n  retries         ");
   cyros::bench::print_hex(retries);
   cyros::bench::print("\n");

   cyros::bench::start("fetch_add from both cores loses nothing");
   CYROS_CHECK_EQ(by_add, expected);

   cyros::bench::start("a compare-exchange loop on both cores loses nothing");
   CYROS_CHECK_EQ(by_exchange, expected);

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("atomics across two cores\n\n");

   kernel::initialise();
   thread t0(thread_on_core0, stack_core0, thread::priority(0), core0);
   thread t1(thread_on_core1, stack_core1, thread::priority(0), core1);

   /* Does not return on a target port. thread_on_core0 exits the image. */
   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
