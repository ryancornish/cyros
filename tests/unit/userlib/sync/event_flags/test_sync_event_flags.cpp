#include <cyros/sync/event_flags.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/config/config.hpp>

#include <common/guarded_stack.hpp>

#include "gtest/gtest.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

using namespace cyros;

static_assert(config::cores >= 4, "Test suite is designed for (at least) quad-core configuration");

constinit static sync::event_flags constant_initialised_event_flags{0x5};

/* ============================================================================
 * event_flags contract, black-box
 *
 * What is asserted: set and clear report the flags before them, a wait for
 * ANY returns on one matching bit and a wait for ALL only on every bit, bits
 * outside a waiter's mask do not release it, consume clears exactly the bits
 * that satisfied the wait so one set satisfies one consumer, and waiters
 * wanting different bits are released independently.
 *
 * Layer 6, beside the semaphore, so no semaphore is used as scaffolding. A
 * lone waiter's park is proved by a same-core witness of lower priority, and
 * every spin gate waits on a flag set from a different core.
 * ========================================================================= */

namespace
{

constexpr int contract_reps = 3;

class SyncEventFlags_Test : public ::testing::Test {};

/* A failure by NAME rather than the runner's timeout. Every caller is alone on
 * its core, so spinning here holds up nothing. */
void spin_until(std::atomic<int> const& value, int target, char const* what)
{
   using clock = std::chrono::steady_clock;
   auto const deadline = clock::now() + std::chrono::seconds(20);
   while (value.load(std::memory_order_acquire) < target) {
      if (clock::now() > deadline) {
         std::fprintf(stderr, "watchdog: %s\n", what);
         std::fflush(stderr);
         std::abort();
      }
      this_core::cpu_relax();
   }
}

/* Long enough for a waiter wrongly released on another core to show up. It
 * can only miss a bug, never report a false one. */
void settle()
{
   for (int i = 0; i < 2'000'000; ++i) {
      this_core::cpu_relax();
   }
}

}  // namespace

/* ============================================================================
 * The state machine, one thread, no blocking
 * ========================================================================= */
TEST_F(SyncEventFlags_Test, GivenOneThread_WhenDrivingTheBits_ThenEveryResultIsExact)
{
   kernel::initialise();

   cyros::test::guarded_stack stack;

   struct state
   {
      sync::event_flags f{0b0001};
      std::uint32_t set_before{0}, clear_before{0}, after_clear{0};
      std::uint32_t any_hit{0}, any_miss{1}, all_hit{0}, all_miss{1};
      std::uint32_t consumed{0}, after_consume{0}, blocking_any{0};
   } s;

   thread driver(
      [&s]{
         s.set_before   = s.f.set(0b0110);            // 0001 -> 0111
         s.clear_before = s.f.clear(0b0001);          // 0111 -> 0110
         s.after_clear  = s.f.peek();
         s.any_hit  = s.f.try_wait(0b1010, flags_match::any);   // 0010
         s.any_miss = s.f.try_wait(0b1001, flags_match::any);   // 0
         s.all_hit  = s.f.try_wait(0b0110, flags_match::all);   // 0110
         s.all_miss = s.f.try_wait(0b1110, flags_match::all);   // 0
         s.consumed = s.f.try_wait(0b1100, flags_match::any, flags_exit::consume); // 0100
         s.after_consume = s.f.peek();                          // 0010
         s.blocking_any = s.f.wait(0b0011);                     // set, so no block: 0010
      },
      stack, thread::priority(0), core0);

   kernel::start();

   EXPECT_EQ(constant_initialised_event_flags.peek(), 0x5u);
   EXPECT_EQ(s.set_before, 0b0001u);
   EXPECT_EQ(s.clear_before, 0b0111u);
   EXPECT_EQ(s.after_clear, 0b0110u);
   EXPECT_EQ(s.any_hit, 0b0010u)  << "an ANY wait did not return exactly the matching bits";
   EXPECT_EQ(s.any_miss, 0u);
   EXPECT_EQ(s.all_hit, 0b0110u);
   EXPECT_EQ(s.all_miss, 0u)      << "an ALL wait was satisfied by some of its bits";
   EXPECT_EQ(s.consumed, 0b0100u);
   EXPECT_EQ(s.after_consume, 0b0010u) << "consume cleared bits other than the ones it matched";
   EXPECT_EQ(s.blocking_any, 0b0010u);

   kernel::finalise();
}

/* ============================================================================
 * A wait for ALL blocks until the last bit, and ignores bits outside its mask
 *
 * The waiter parks on 0b0011. A set of a bit outside the mask, then of one of
 * its two bits, must each leave it parked. Only the second of its bits
 * releases it.
 * ========================================================================= */
TEST_F(SyncEventFlags_Test, GivenAnAllWaiter_WhenItsBitsArriveOneAtATime_ThenOnlyTheLastReleasesIt)
{
   for (int rep = 0; rep < contract_reps; ++rep) {
      SCOPED_TRACE("all rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<cyros::test::guarded_stack, 3> stacks;

      struct state
      {
         sync::event_flags f{0};
         std::atomic<int>  parked{0};
         std::atomic<int>  returned{0};
         std::uint32_t     result{0};
         int returned_early{0};
      } s;

      thread waiter(
         [&s]{
            s.result = s.f.wait(0b0011, flags_match::all);
            s.returned.store(1, std::memory_order_release);
         },
         stacks[0], thread::priority(1), core0);

      thread witness([&s]{ s.parked.store(1, std::memory_order_release); },
                     stacks[1], thread::priority(2), core0);

      thread setter(
         [&s]{
            spin_until(s.parked, 1, "the waiter never parked");
            s.f.set(0b0100);
            settle();
            s.f.set(0b0001);
            settle();
            s.returned_early = s.returned.load(std::memory_order_acquire);
            s.f.set(0b0010);
            spin_until(s.returned, 1, "the last bit of an ALL mask did not release its waiter");
         },
         stacks[2], thread::priority(0), core1);

      kernel::start();

      EXPECT_EQ(s.returned_early, 0) << "an ALL waiter returned before its last bit";
      EXPECT_EQ(s.result, 0b0011u);

      kernel::finalise();
   }
}

/* ============================================================================
 * Waiters wanting different bits are released independently
 *
 * Three waiters on three cores want bits 0, 1 and 2. Setting bit 1 must
 * release only the second: the other two are woken to re-test and must park
 * again. Then bits 0 and 2 release the rest.
 * ========================================================================= */
TEST_F(SyncEventFlags_Test, GivenWaitersOnDifferentBits_WhenOneBitIsSet_ThenOnlyItsWaiterReturns)
{
   for (int rep = 0; rep < contract_reps; ++rep) {
      SCOPED_TRACE("independent rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<cyros::test::guarded_stack, 7> stacks;

      struct state
      {
         sync::event_flags f{0};
         std::atomic<int>  parked{0};
         std::atomic<int>  returned{0};
         std::array<std::atomic<bool>, 3> done{};
         bool only_bit_one{false};
      } s;

      constexpr core_affinity cores[3] = { core0, core1, core2 };
      std::array<thread, 3> waiters{};
      std::array<thread, 3> witnesses{};
      for (std::size_t i = 0; i < 3; ++i) {
         waiters[i] = thread(
            [&s, i]{
               (void)s.f.wait(std::uint32_t{1} << i);
               s.done[i].store(true, std::memory_order_release);
               s.returned.fetch_add(1, std::memory_order_acq_rel);
            },
            stacks[i], thread::priority(1), cores[i]);
         witnesses[i] = thread([&s]{ s.parked.fetch_add(1, std::memory_order_acq_rel); },
                               stacks[3 + i], thread::priority(2), cores[i]);
      }

      thread setter(
         [&s]{
            spin_until(s.parked, 3, "a waiter never parked");
            s.f.set(0b010);
            spin_until(s.returned, 1, "bit 1 did not release its waiter");
            settle();
            s.only_bit_one = s.returned.load() == 1 && s.done[1].load() && !s.done[0].load() && !s.done[2].load();
            s.f.set(0b101);
            spin_until(s.returned, 3, "bits 0 and 2 did not release their waiters");
         },
         stacks[6], thread::priority(0), core3);

      kernel::start();

      EXPECT_TRUE(s.only_bit_one) << "setting one bit released a waiter that wanted another";

      kernel::finalise();
   }
}

/* ============================================================================
 * One set satisfies one consumer
 *
 * Two waiters on two cores consume bit 0. One set must release exactly one of
 * them and leave the bit clear, the second set the other.
 * ========================================================================= */
TEST_F(SyncEventFlags_Test, GivenTwoConsumersOfOneBit_WhenSetOnce_ThenExactlyOneReturnsAndTheBitIsCleared)
{
   for (int rep = 0; rep < contract_reps; ++rep) {
      SCOPED_TRACE("consume rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<cyros::test::guarded_stack, 5> stacks;

      struct state
      {
         sync::event_flags f{0};
         std::atomic<int>  parked{0};
         std::atomic<int>  returned{0};
         int returned_after_one{-1};
         std::uint32_t bits_after_one{~0u};
      } s;

      constexpr core_affinity cores[2] = { core0, core1 };
      std::array<thread, 2> consumers{};
      std::array<thread, 2> witnesses{};
      for (std::size_t i = 0; i < 2; ++i) {
         consumers[i] = thread(
            [&s]{
               (void)s.f.wait(0b1, flags_match::any, flags_exit::consume);
               s.returned.fetch_add(1, std::memory_order_acq_rel);
            },
            stacks[i], thread::priority(1), cores[i]);
         witnesses[i] = thread([&s]{ s.parked.fetch_add(1, std::memory_order_acq_rel); },
                               stacks[2 + i], thread::priority(2), cores[i]);
      }

      thread setter(
         [&s]{
            spin_until(s.parked, 2, "a consumer never parked");
            s.f.set(0b1);
            spin_until(s.returned, 1, "a set did not release any consumer");
            settle();
            s.returned_after_one = s.returned.load();
            s.bits_after_one = s.f.peek();
            s.f.set(0b1);
            spin_until(s.returned, 2, "the second set did not release the other consumer");
         },
         stacks[4], thread::priority(0), core2);

      kernel::start();

      EXPECT_EQ(s.returned_after_one, 1) << "one set satisfied two consumers";
      EXPECT_EQ(s.bits_after_one, 0u)    << "a consumer left the bit it matched set";
      EXPECT_EQ(s.f.peek(), 0u);

      kernel::finalise();
   }
}

/* ============================================================================
 * A consume never disturbs bits it did not match
 *
 * The consume is a compare-and-swap that clears exactly the matched bits, and
 * it retries when the flags changed underneath it. Two more cores each churn a
 * bit of their own the whole time, so consumes of bit 0 keep losing that race
 * and retrying, and each churner checks after each of its own writes that its
 * bit reads back as it left it. A consume that wrote a stale snapshot instead
 * of retrying would clear a bit just set, or resurrect one just cleared. Every
 * set of bit 0 must also be consumed exactly once.
 *
 * The race cannot be forced from outside the kernel, so this is statistical.
 * The mutant that stores the stale snapshot instead of retrying was killed in
 * 20 of 20 runs (Arch box, under a 3-way soak, 2026-09-30), against 7 of 10
 * with one churner and half the sets, which is why there are two.
 * ========================================================================= */
TEST_F(SyncEventFlags_Test, GivenAnotherCoreChurningOtherBits_WhenConsuming_ThenEachSetIsTakenOnceAndOtherBitsAreUntouched)
{
   constexpr int sets = 40'000;

   kernel::initialise();

   static std::array<cyros::test::guarded_stack, 4> stacks;

   struct state
   {
      sync::event_flags f{0};
      std::atomic<int>  consumed{0};
      std::atomic<bool> done{false};
      std::atomic<int>  corrupted{0};
      std::atomic<int>  churns{0};
   } s;

   thread consumer(
      [&s]{
         for (int i = 0; i < sets; ++i) {
            (void)s.f.wait(0b01, flags_match::any, flags_exit::consume);
            s.consumed.fetch_add(1, std::memory_order_acq_rel);
         }
      },
      stacks[0], thread::priority(1), core0);

   thread setter(
      [&s]{
         for (int i = 0; i < sets; ++i) {
            s.f.set(0b01);
            spin_until(s.consumed, i + 1, "a set of bit 0 was never consumed");
         }
         s.done.store(true, std::memory_order_release);
      },
      stacks[1], thread::priority(1), core1);

   // Each churner owns its bit: nothing else may change it.
   auto churn = [&s](std::uint32_t bit) {
      while (!s.done.load(std::memory_order_acquire)) {
         s.f.set(bit);
         if ((s.f.peek() & bit) == 0) s.corrupted.fetch_add(1, std::memory_order_relaxed);
         s.f.clear(bit);
         if ((s.f.peek() & bit) != 0) s.corrupted.fetch_add(1, std::memory_order_relaxed);
         s.churns.fetch_add(1, std::memory_order_relaxed);
      }
   };
   thread churner_a([&churn]{ churn(0b010); }, stacks[2], thread::priority(1), core2);
   thread churner_b([&churn]{ churn(0b100); }, stacks[3], thread::priority(1), core3);

   kernel::start();

   EXPECT_EQ(s.consumed.load(), sets);
   EXPECT_EQ(s.corrupted.load(), 0) << "a consume wrote back a stale bit it did not match";
   EXPECT_GT(s.churns.load(), 0)    << "the churn never ran, so the race was never exercised";
   EXPECT_EQ(s.f.peek() & 0b01, 0u);

   kernel::finalise();
}
