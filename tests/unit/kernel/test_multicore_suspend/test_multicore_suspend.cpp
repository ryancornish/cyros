/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_multicore_suspend.cpp
 * @brief this_thread::suspend and thread::resume across two cores.
 *
 * Subject: suspend and resume across cores (L5)
 * Trusts:  bring-up, scheduling, waitable, and the cross-core wake
 *
 * A suspend arms nothing, so a resume from another core reaches it only
 * through the intake. These tests race the two, and race a suspend against a
 * wake of the thread's previous wait that is drained only once it suspends.
 *
 * A lost resume would leave a thread parked for good. Each wait here gives up
 * after a bound no correct run comes near, records the loss and resumes again,
 * so a defect is a failed assertion rather than a hung suite.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/kernel/waitable.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <common/guarded_stack.hpp>

#include "gtest/gtest.h"

#include <atomic>
#include <chrono>
#include <cstdint>

using namespace cyros;

static_assert(config::cores >= 2, "Test suite is designed for (at least) dual-core configuration");

int main(int argc, char** argv)
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

namespace
{

constexpr int rounds = 20'000;

class MultiCoreSuspend_Test : public ::testing::Test {};

/// Spin until @p done holds. False after a bound a correct run never nears,
/// which turns a lost resume into a failure instead of a hang.
template<typename Done>
bool await(Done done)
{
   auto const give_up = std::chrono::steady_clock::now() + std::chrono::seconds(5);
   while (!done()) {
      if (std::chrono::steady_clock::now() > give_up) return false;
      this_core::cpu_relax();
   }
   return true;
}

/// Long enough for a request sent to the other core to have been drained:
/// a remote wake there takes a few microseconds on the Linux ports.
void settle()
{
   auto const until = std::chrono::steady_clock::now() + std::chrono::microseconds(30);
   while (std::chrono::steady_clock::now() < until) this_core::cpu_relax();
}

/// A few hundred spins at most, to land the next step anywhere in a window.
void scatter(std::uint32_t& seed)
{
   seed = seed * 1'664'525u + 1'013'904'223u;
   for (std::uint32_t n = seed >> 24; n != 0; --n) {
      this_core::cpu_relax();
   }
}

/// A count of tokens, one wake per token, as a semaphore's release.
class CountingWaitable final : public waitable
{
public:
   std::atomic<int> tokens{0};

   void release() noexcept
   {
      tokens.fetch_add(1, std::memory_order_acq_rel);
      wake_one();
   }

protected:
   bool try_satisfy(waiter_record*) noexcept override
   {
      int current = tokens.load(std::memory_order_acquire);
      while (current > 0) {
         if (tokens.compare_exchange_weak(current, current - 1,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed)) {
            return true;
         }
      }
      return false;
   }
};

} // namespace

/* ============================================================================
 * A resume from the other core, landing anywhere around the suspend
 *
 * The resume for each round is sent once the previous one has been taken, so
 * no two merge into one token: every resume must release exactly one suspend.
 * Scattered, it lands before the suspend, between its prepared store and its
 * look, between the look and the commit, and on the park.
 * ========================================================================= */
TEST_F(MultiCoreSuspend_Test,
       GivenAResumeFromAnotherCore_WhenItLandsAnywhereAroundTheSuspend_ThenEachReleasesOneSuspend)
{
   kernel::initialise();

   cyros::test::guarded_stack suspender_stack;
   cyros::test::guarded_stack resumer_stack;

   struct state
   {
      thread* suspender = nullptr;
      std::atomic<int> returns{0};
      std::atomic<int> resumes{0};
      int early = 0;
      int lost = 0;
   } s;

   thread suspender(
      [&s]{
         for (int i = 0; i < rounds; ++i) {
            this_thread::suspend();
            if (s.resumes.load(std::memory_order_acquire) <= i) ++s.early;
            s.returns.store(i + 1, std::memory_order_release);
         }
      },
      suspender_stack, thread::priority(0), core0
   );

   thread resumer(
      [&s]{
         std::uint32_t seed = 1;
         for (int i = 0; i < rounds; ++i) {
            scatter(seed);
            s.resumes.store(i + 1, std::memory_order_release);
            s.suspender->resume();
            if (!await([&]{ return s.returns.load(std::memory_order_acquire) >= i + 1; })) {
               ++s.lost;
               s.suspender->resume();
               (void)await([&]{ return s.returns.load(std::memory_order_acquire) >= i + 1; });
            }
         }
         // A defect can leave it parked short of the end, so free it
         while (s.returns.load(std::memory_order_acquire) < rounds) {
            s.suspender->resume();
            settle();
         }
      },
      resumer_stack, thread::priority(0), core1
   );

   s.suspender = &suspender;

   kernel::start();

   EXPECT_EQ(s.lost, 0);
   EXPECT_EQ(s.early, 0);
   EXPECT_EQ(s.returns.load(), rounds);
   kernel::finalise();
}

/* ============================================================================
 * A wake of the thread's previous wait, drained during its suspend
 *
 * A wake that takes a waiter's node after its poll was satisfied still sends
 * the waiter a ready. From another core that ready travels through the
 * intake, and can be drained after the waiter has left the wait and
 * suspended. It carries no token, so the suspend must look again and park.
 * The release for each round lands as the suspender re-arms on w, which is
 * what puts the wake there. The resume waits for any such wake to have been
 * drained first: sent sooner, it would share the wake's intake entry and
 * hide it.
 * ========================================================================= */
TEST_F(MultiCoreSuspend_Test,
       GivenAWakeOfAnEarlierWaitDrainedDuringASuspend_WhenItLands_ThenTheSuspendParksAgain)
{
   kernel::initialise();

   cyros::test::guarded_stack suspender_stack;
   cyros::test::guarded_stack driver_stack;

   CountingWaitable w;
   struct state
   {
      thread* suspender = nullptr;
      std::atomic<int> phase{0};
      std::atomic<int> resumes{0};
      int early = 0;
      int lost = 0;
   } s;

   thread suspender(
      [&s, &w]{
         for (int i = 0; i < rounds; ++i) {
            this_thread::wait_on(w);
            s.phase.store(2 * i + 1, std::memory_order_release);
            this_thread::suspend();
            if (s.resumes.load(std::memory_order_acquire) <= i) ++s.early;
            s.phase.store(2 * i + 2, std::memory_order_release);
         }
      },
      suspender_stack, thread::priority(0), core0
   );

   thread driver(
      [&s, &w]{
         std::uint32_t seed = 7;
         for (int i = 0; i < rounds; ++i) {
            w.release();
            if (!await([&]{ return s.phase.load(std::memory_order_acquire) >= 2 * i + 1; })) {
               ++s.lost;
               break;
            }
            settle();
            scatter(seed);
            s.resumes.store(i + 1, std::memory_order_release);
            s.suspender->resume();
            if (!await([&]{ return s.phase.load(std::memory_order_acquire) >= 2 * i + 2; })) {
               ++s.lost;
               s.suspender->resume();
               (void)await([&]{ return s.phase.load(std::memory_order_acquire) >= 2 * i + 2; });
            }
         }
         // A defect can leave it parked short of the end, so free it
         while (s.phase.load(std::memory_order_acquire) < 2 * rounds) {
            w.release();
            s.suspender->resume();
            settle();
         }
      },
      driver_stack, thread::priority(0), core1
   );

   s.suspender = &suspender;

   kernel::start();

   EXPECT_EQ(s.lost, 0);
   EXPECT_EQ(s.early, 0); // A suspend that returned on the wait's late wake
   EXPECT_EQ(s.phase.load(), 2 * rounds);
   kernel::finalise();
}

/* ============================================================================
 * Two threads on two cores resuming each other
 * ========================================================================= */
TEST_F(MultiCoreSuspend_Test,
       GivenTwoThreadsOnTwoCoresResumingEachOther_WhenTheyAlternate_ThenBothFinish)
{
   kernel::initialise();

   cyros::test::guarded_stack a_stack;
   cyros::test::guarded_stack b_stack;

   struct state
   {
      thread* a = nullptr;
      thread* b = nullptr;
      std::atomic<int> a_turns{0};
      std::atomic<int> b_turns{0};
   } s;

   thread a(
      [&s]{
         for (int i = 0; i < rounds; ++i) {
            s.b->resume();
            this_thread::suspend();
            s.a_turns.fetch_add(1, std::memory_order_relaxed);
         }
         s.b->resume();
      },
      a_stack, thread::priority(0), core0
   );

   thread b(
      [&s]{
         for (int i = 0; i < rounds; ++i) {
            this_thread::suspend();
            s.b_turns.fetch_add(1, std::memory_order_relaxed);
            s.a->resume();
         }
         this_thread::suspend();
      },
      b_stack, thread::priority(0), core1
   );

   s.a = &a;
   s.b = &b;

   kernel::start();

   EXPECT_EQ(s.a_turns.load(), rounds);
   EXPECT_EQ(s.b_turns.load(), rounds);
   kernel::finalise();
}
