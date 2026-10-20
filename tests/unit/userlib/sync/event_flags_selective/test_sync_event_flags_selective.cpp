/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_sync_event_flags_selective.cpp
 * @brief A set wakes only the waiters it satisfies.
 *
 * From outside, a waiter left parked and one woken only to re-test and park
 * again look the same: both are still in their wait. The kernel trace tells
 * them apart, since a wake readies its waiter and the trace records every
 * readying. So each test drains the trace, sets, and counts thread_ready
 * records per waiter. The records are written inside the set, so they are all
 * there when it returns, whichever port and whenever the woken threads run.
 *
 * One core and priorities alone order everything: every waiter outranks the
 * setter, so by the time the setter runs each one has parked.
 */

#include <cyros/sync/event_flags.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/trace.hpp>
#include <cyros/config/config.hpp>

#include <common/guarded_stack.hpp>

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

using namespace cyros;

static_assert(config::cores == 1, "priorities alone order these tests, which needs one core");

namespace
{

constexpr std::size_t waiter_count = 20;

std::array<cyros::test::guarded_stack, waiter_count + 1> stacks;

/* Throws away everything core 0 has recorded so far. No heap: the reader is a
 * kernel thread. */
void discard_trace()
{
   std::array<trace::record, 64> chunk{};
   while (trace::read(0, chunk) != 0) {}
}

/* How many times each of @p ids was readied since the last read. */
template<std::size_t N>
std::array<int, N> readied(std::array<thread::id, N> const& ids)
{
   std::array<int, N> counts{};
   std::array<trace::record, 64> chunk{};
   while (auto const n = trace::read(0, chunk)) {
      for (std::size_t r = 0; r < n; ++r) {
         if (chunk[r].what != trace::event::thread_ready) continue;
         for (std::size_t i = 0; i < N; ++i) {
            if (chunk[r].thread == ids[i]) ++counts[i];
         }
      }
   }
   return counts;
}

}  // namespace

/* ============================================================================
 * A set leaves waiters on other bits parked
 *
 * Three waiters want bits 0, 1 and 2. Setting bit 1 must ready the second
 * once and the other two not at all. Bits 0 and 2 then release them.
 * ========================================================================= */
TEST(SyncEventFlagsSelective_Test, GivenWaitersOnOtherBits_WhenABitIsSet_ThenOnlyItsWaiterIsReadied)
{
   kernel::initialise();

   struct state
   {
      sync::event_flags          f{0};
      std::array<thread::id, 3>  ids{};
      std::array<int, 3>         counts{};
      std::atomic<int>           returned{0};
   } s;

   std::array<thread, 3> waiters{};
   for (std::size_t i = 0; i < 3; ++i) {
      waiters[i] = thread(
         [&s, i]{
            (void)s.f.wait(std::uint32_t{1} << i);
            s.returned.fetch_add(1, std::memory_order_acq_rel);
         },
         stacks[i], thread::priority(static_cast<std::uint8_t>(1 + i)), core0);
      s.ids[i] = waiters[i].get_id();
   }

   thread setter(
      [&s]{
         discard_trace();
         s.f.set(0b010);
         s.counts = readied(s.ids);
         s.f.set(0b101);
      },
      stacks[waiter_count], thread::priority(10), core0);

   kernel::start();

   EXPECT_EQ(s.counts[1], 1) << "bit 1 did not ready its waiter exactly once";
   EXPECT_EQ(s.counts[0], 0) << "setting bit 1 readied the waiter on bit 0";
   EXPECT_EQ(s.counts[2], 0) << "setting bit 1 readied the waiter on bit 2";
   EXPECT_EQ(s.returned.load(), 3);
   EXPECT_EQ(trace::dropped(0), 0u);

   kernel::finalise();
}

/* ============================================================================
 * More satisfied waiters than one walk of the queue takes
 *
 * Twenty waiters alternate between bits 0 and 1, so a set of bit 0 satisfies
 * ten of them, interleaved with ten it does not, more than the wake takes in
 * one walk of the queue. Each of the ten must be readied exactly once and none
 * of the others at all.
 *
 * If a set ever leaves a satisfied waiter parked, the setter's last loop sets
 * both bits until every waiter is out, so the test fails on its counts rather
 * than hanging.
 * ========================================================================= */
TEST(SyncEventFlagsSelective_Test, GivenMoreSatisfiedWaitersThanOneWalkTakes_WhenSet_ThenEachIsReadiedOnceAndNoOther)
{
   kernel::initialise();

   struct state
   {
      sync::event_flags                     f{0};
      std::array<thread::id, waiter_count>  ids{};
      std::array<int, waiter_count>         counts{};
      std::atomic<int>                      returned{0};
   } s;

   std::array<thread, waiter_count> waiters{};
   for (std::size_t i = 0; i < waiter_count; ++i) {
      waiters[i] = thread(
         [&s, i]{
            (void)s.f.wait(std::uint32_t{1} << (i % 2));
            s.returned.fetch_add(1, std::memory_order_acq_rel);
         },
         stacks[i], thread::priority(static_cast<std::uint8_t>(1 + i)), core0);
      s.ids[i] = waiters[i].get_id();
   }

   thread setter(
      [&s]{
         discard_trace();
         s.f.set(0b01);
         s.counts = readied(s.ids);
         s.f.set(0b10);
         while (s.returned.load(std::memory_order_acquire) < static_cast<int>(waiter_count)) {
            s.f.set(0b11);
            this_thread::yield();
         }
      },
      stacks[waiter_count], thread::priority(30), core0);

   kernel::start();

   for (std::size_t i = 0; i < waiter_count; ++i) {
      SCOPED_TRACE("waiter " + std::to_string(i) + " on bit " + std::to_string(i % 2));
      EXPECT_EQ(s.counts[i], i % 2 == 0 ? 1 : 0);
   }
   EXPECT_EQ(trace::dropped(0), 0u);

   kernel::finalise();
}
