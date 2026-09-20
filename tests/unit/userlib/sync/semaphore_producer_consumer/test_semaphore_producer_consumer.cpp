/**
 * @file test_semaphore_producer_consumer.cpp
 * @brief Many producers and many consumers across cores, under load.
 *
 * Subject:  semaphore (L6), exercised as a system rather than as a contract
 * Trusts:   everything up to and including cross-core wake (L5)
 * Proves:   over many lifecycles, no token is lost and none is invented
 *
 * Split out of test_sync_semaphore on 2026-09-20. It is an INTEGRATION test by
 * the T1 section 4 definition: it exercises several layers together under
 * realistic timing, and its value depends on load, which is exactly what a unit
 * test is not allowed to depend on. Keeping it in the unit binary made that
 * binary's kind dishonest and made a slow, load-sensitive case run every time
 * anyone wanted the fast contract checks.
 *
 * It is also the regression reproducer for the bring-up IPI drop in
 * cross-core-defects.md section 14, which is why the comments below are so
 * specific about core placement and token counts. Do not "simplify" it: a
 * cut-down version with 80 tokens does not reproduce the defect at all.
 */
#include <cyros/sync/semaphore.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <common/guarded_stack.hpp>

#include "gtest/gtest.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

using namespace cyros;

static_assert(config::cores >= 4, "Test suite is designed for (at least) quad-core configuration");

// A semaphore must stay constant-initialisable, so a static one lands in .bss
// with no startup code and no initialisation-order surface. This is the reason
// the park list is a named-but-unusable type rather than opaque storage: opaque
// storage would need placement-new in the constructor and break this.
constinit static sync::semaphore constant_initialised_semaphore{1};

static constexpr auto STACK_SIZE = thread::min_stack_size + (16 * 1024);

/* ============================================================================
 * Semaphore contract, black-box
 *
 * The semaphore is barge-permitting by design (built on plain wake, not
 * transfer), so this suite deliberately asserts only what that design
 * promises: token accounting is exact, acquire on a zero count BLOCKS until a
 * release supplies a token, releases wake enough waiters, and no token is
 * ever lost or double-granted. It deliberately does NOT assert grant order
 * between concurrent waiters, a woken waiter races fresh arrivals for the
 * token and may lose, which is the documented barging trade.
 *
 * NOTE: GivenZeroCount_WhenAcquire_ThenBlocksUntilRelease and the
 * producer/consumer accounting test are the regression tests for the
 * unconditional-decrement acquire() bug: an acquire that never blocks wraps
 * the counter and both tests fail loudly (accounting) or via the recorded
 * order flag (blocking) rather than by hanging.
 * ========================================================================= */

namespace
{

constexpr int contract_reps = 3;

// Producer/consumer accounting volume per lifecycle. All inside one
// lifecycle, so cheap to raise.
constexpr std::uint64_t tokens_total = 20'000;

class SemaphoreProducerConsumer_Test : public ::testing::Test {};

}  // namespace

/* ============================================================================
 * Counting contract, single thread
 *
 * Pure state machine, no timing: n tokens grant exactly n try_acquires, the
 * n+1th fails, release restores exactly what it adds, and acquire on a
 * positive count takes the fast path without blocking (reaching the flag is
 * the proof, a wrongly parked thread would prevent quiescence).
 * ========================================================================= */

/* ============================================================================
 * Producer/consumer accounting under real contention
 *
 * Two producers mint a known total of tokens in mixed burst sizes across two
 * cores, two consumers on two other cores acquire until the total is
 * consumed.
 *
 * THIS IS ALSO THE REGRESSION REPRODUCER for the dropped bring-up IPI of
 * cross-core-defects.md section 14, which it found: a consumer on a
 * late-spawned core was stranded forever by a wake whose IPI was dropped while
 * its core was still unaddressable. It hangs when that defect is present,
 * measured at about 1 percent per run under 6-way load on a 4-core box, and
 * needs LOAD to show at all. Three properties are what reach the window, so
 * keep them if this test is ever trimmed: consumers on the LAST-spawned cores
 * (2 and 3), at least one producer on an EARLY-spawned core (core1, since
 * core0's threads only run after every core is addressable, and a core0-only
 * producer scores 0 in 12,000 lifecycles), and a token count high enough that
 * consumers keep finding the semaphore empty and re-parking during the first
 * microseconds of kernel::start(). A cut-down version with 80 tokens does not
 * reproduce it: the producer drains its budget before the consumers exist, so
 * nothing ever parks and no wake is ever posted. Exactness is the assertion: consumed == minted, the count ends at
 * zero, and a final try_acquire fails. A lost wakeup hangs (framework timeout
 * is the detector), a lost or double-granted token breaks the arithmetic, and
 * the wrapped-counter acquire bug fails the final-count checks immediately.
 * ========================================================================= */
TEST_F(SemaphoreProducerConsumer_Test, GivenProducersAndConsumersAcrossCores_WhenTokensFlow_ThenNoneAreLostOrInvented)
{
   for (int rep = 0; rep < contract_reps; ++rep) {
      SCOPED_TRACE("accounting rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<cyros::test::guarded_stack, 4> stacks;

      struct state
      {
         semaphore sem{0};
         std::atomic<std::uint64_t> consumed{0};
         std::atomic<std::size_t>   final_peek{~std::size_t{0}};
         std::atomic<bool>          post_drain_take{true};
      } s;

      static_assert(tokens_total % 4 == 0, "split evenly across two producers and two consumers");

      auto producer_body = [](state& s, std::uint64_t budget) {
         std::uint64_t minted = 0;
         while (minted < budget) {
            // Mixed burst sizes exercise release(1) and release(n) paths.
            std::uint64_t const burst = ((minted / 3) % 3) + 1;
            std::uint64_t const n = (burst < budget - minted) ? burst : (budget - minted);
            s.sem.release(static_cast<std::size_t>(n));
            minted += n;
         }
      };

      auto consumer_body = [](state& s) {
         for (std::uint64_t i = 0; i < tokens_total / 2; ++i) {
            s.sem.acquire();
            s.consumed.fetch_add(1, std::memory_order_relaxed);
         }
      };

      thread p0([&s, &producer_body]{ producer_body(s, tokens_total / 2); },
                stacks[0], thread::priority(1), core0);
      thread p1([&s, &producer_body]{ producer_body(s, tokens_total / 2); },
                stacks[1], thread::priority(1), core1);

      thread c0(
         [&s, &consumer_body]{ consumer_body(s); },
         stacks[2], thread::priority(1), core2);
      thread c1(
         [&s, &consumer_body]{
            consumer_body(s);
            // Runs the epilogue in one consumer to avoid racing the checks:
            // by the time BOTH consumers have their full share, every token
            // is spoken for. Spin for the sibling's completion cross-core.
            while (s.consumed.load(std::memory_order_acquire) < tokens_total) {
               this_core::cpu_relax();
            }
            s.final_peek.store(s.sem.peek(), std::memory_order_release);
            s.post_drain_take.store(s.sem.try_acquire(), std::memory_order_release);
         },
         stacks[3], thread::priority(1), core3);

      kernel::start();

      EXPECT_EQ(s.consumed.load(), tokens_total) << "consumed token count diverged from minted";
      EXPECT_EQ(s.final_peek.load(), 0u)         << "count nonzero after exact consumption";
      EXPECT_FALSE(s.post_drain_take.load())     << "try_acquire granted an unminted token";

      kernel::finalise();
   }
}
