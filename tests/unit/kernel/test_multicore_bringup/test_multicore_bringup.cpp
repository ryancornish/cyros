/**
 * @file test_multicore_bringup.cpp
 * @brief multicore bring-up (L2, the harness floor)
 *
 * Subject:  multicore bring-up (L2, the harness floor)
 * Trusts:   the port contract (L0) and core primitives (L1)
 * Proves:   every core starts, a thread pinned to each runs on it, and the run winds down when they all exit
 *
 * Split out of test_multicore_multithread on 2026-09-20. That binary proved
 * three different things at three different layers, so a failure in any of them
 * was attributed to one name. This half is the HARNESS FLOOR: the operations
 * every core-touching test in the tree trusts before it can assert anything.
 * Nothing above layer 2 belongs here.
 */
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
#include <vector>

using namespace cyros;

static_assert(config::cores >= 4, "Test suite is designed for (atleast) quad core configuration only");

static constexpr auto STACK_SIZE = thread::min_stack_size + (16 * 1024);

int main(int argc, char** argv)
{
   ::testing::InitGoogleTest(&argc, argv);

   int result = RUN_ALL_TESTS();

   return result;
}


class MultiCoreBringUp_Test : public ::testing::Test
{
   void SetUp() override
   {
      kernel::initialise();
   }

   void TearDown() override
   {
      kernel::finalise();
   }
};


TEST_F(MultiCoreBringUp_Test,
       GivenTwoCoresAndOneThreadPinnedToEach_WhenKernelStarts_ThenBothThreadsRunOnExpectedCore)
{
   cyros::test::guarded_stack s0;
   cyros::test::guarded_stack s1;

   bool ran0 = false;
   bool ran1 = false;
   uint32_t seen_core0 = std::numeric_limits<uint32_t>::max();
   uint32_t seen_core1 = std::numeric_limits<uint32_t>::max();

   // GIVEN:

   thread t0(
      [&]{
         seen_core0 = this_core::id();
         ran0 = true;
      },
      s0,
      thread::priority(0),
      core0
   );

   thread t1(
      [&]{
         seen_core1 = this_core::id();
         ran1 = true;
      },
      s1,
      thread::priority(0),
      core1
   );

   EXPECT_EQ(kernel::active_threads(), 2u);

   // WHEN:

   kernel::start();

   // THEN:

   EXPECT_EQ(kernel::active_threads(), 0u);
   EXPECT_TRUE(ran0);
   EXPECT_TRUE(ran1);

   EXPECT_EQ(seen_core0, 0u);
   EXPECT_EQ(seen_core1, 1u);
}


TEST_F(MultiCoreBringUp_Test,
       GivenUpToFourCoresWithOneThreadEach_WhenKernelStarts_ThenAllCoresMakeProgress)
{
   if (kernel::core_count() < 4)  GTEST_SKIP() << "Need at least 4 cores for this test";

   cyros::test::guarded_stack s0;
   cyros::test::guarded_stack s1;
   cyros::test::guarded_stack s2;
   cyros::test::guarded_stack s3;

   std::array<int, 4> stages{0};

   auto make_thread = [&](uint32_t core_id, cyros::test::guarded_stack& stack) -> thread
   {
      return {
         [&, core_id]{
            // stage 1: started
            stages[core_id] = 1;

            // Do some cooperative stepping so reschedule/rotation is exercised.
            for (int i = 0; i < 3; ++i) {
               this_thread::yield();
            }

            // stage 2: finished
            stages[core_id] = 2;
         },
         stack,
         thread::priority(0),
         core_affinity::from_id(core_id)
      };
   };

   // GIVEN:

   auto t1 = make_thread(0, s0);
   auto t2 = make_thread(1, s1);
   auto t3 = make_thread(2, s2);
   auto t4 = make_thread(3, s3);

   // WHEN:

   kernel::start();

   // THEN:

   for (std::uint32_t core_id = 0; core_id < kernel::core_count(); ++core_id) {
      EXPECT_EQ(stages[core_id], 2)
         << "Core " << core_id << " did not complete its thread";
   }
   EXPECT_EQ(kernel::active_threads(), 0u);
}


TEST_F(MultiCoreBringUp_Test,
       GivenEveryCoreRunningThreadsThatExitExplicitly_WhenSystemRuns_ThenAllTerminateAndTheRunWindsDown)
{
   std::array<cyros::test::guarded_stack, 4> stacks;

   std::array<std::atomic<int>, 4> stages{};

   // GIVEN:

   // The LAST of these to retire is what drives the port's quiesce detection,
   // and under this design that decision is made inside the arbiter, after the
   // retiring thread's transition has already landed.
   std::vector<thread> threads;
   threads.reserve(stacks.size());
   auto const affinities = std::to_array({core0, core1, core2, core3});

   for (std::size_t i = 0; i < stacks.size(); ++i) {
      threads.emplace_back(
         [&, i]{
            stages[i].store(1, std::memory_order_release);
            this_thread::yield();
            stages[i].store(2, std::memory_order_release);
            this_thread::thread_exit();
         },
         stacks[i],
         thread::priority(0),
         affinities[i]
      );
   }

   // WHEN:

   kernel::start();

   // THEN:

   for (std::size_t i = 0; i < stages.size(); ++i) {
      EXPECT_EQ(stages[i].load(std::memory_order_acquire), 2)
         << "core " << i << " did not complete its thread";
   }
   EXPECT_EQ(kernel::active_threads(), 0u);
}
