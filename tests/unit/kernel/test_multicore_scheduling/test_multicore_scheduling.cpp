/**
 * @file test_multicore_scheduling.cpp
 * @brief cross-core scheduling and wake (L5)
 *
 * Subject:  cross-core scheduling and wake (L5)
 * Trusts:   bring-up (L2), scheduling (L3) and waitables (L4)
 * Proves:   a thread created for another core after start runs there, an idle core is woken by a poke, and a thread exit wakes joiners on every other core
 *
 * Split out of test_multicore_multithread on 2026-09-20, the half that is
 * genuinely about CROSS-CORE work rather than about the cores existing. It
 * assumes bring-up is proved by test_multicore_bringup at layer 2 and does not
 * re-prove it.
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


class MultiCoreScheduling_Test : public ::testing::Test
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


TEST_F(MultiCoreScheduling_Test,
       GivenTwoCores_WhenCore0CreatesAThreadPinnedToCore1AfterStart_ThenCore1RunsIt)
{
   cyros::test::guarded_stack s_creator;
   cyros::test::guarded_stack s_remote;

   bool remote_ran = false;
   uint32_t remote_seen_core = std::numeric_limits<uint32_t>::max();

   thread remote_thread;

   // GIVEN:

   thread creator(
      [&]{
         EXPECT_EQ(this_core::id(), 0u);

         remote_thread = thread(
            [&]{
               remote_seen_core = this_core::id();
               remote_ran = true;
            },
            s_remote,
            thread::priority(0),
            core1
         );
         // We can happily terminate here as remote_thread will be started on core1
      },
      s_creator,
      thread::priority(0),
      core0
   );

   // Only one thread should be registered (creator) as remote_thread handle is empty
   EXPECT_EQ(kernel::active_threads(), 1u);

   // WHEN:

   kernel::start();

   // THEN:

   EXPECT_TRUE(remote_ran)
      << "Remote thread never ran (missing intake poke / IPI / idle wake)";
   EXPECT_EQ(remote_seen_core, 1u);
}


TEST_F(MultiCoreScheduling_Test,
       GivenTwoCores_WhenCore0PokesCore1WhileCore1IsIdle_ThenCore1WakesAndRunsQueuedWork)
{
   cyros::test::guarded_stack s_core0;
   cyros::test::guarded_stack s_core1_work;

   bool core1_work_ran = false;
   thread core1_work; // Empty handle

   // GIVEN:

   // Make core1 have *no* initial threads queued pre-start by creating the core1 work post-start.
   // core1 will start in idle unless/until it receives intake work + IPI.
   thread core0_thread(
      [&]{
         EXPECT_EQ(this_core::id(), 0u);

         // Give core1 a chance to enter idle first (cooperative).
         for (int i = 0; i < 3; ++i) this_thread::yield();

         core1_work = thread(
            [&]{
               core1_work_ran = true;
            },
            s_core1_work,
            thread::priority(0),
            core1
         );

         // Yield to allow IPI -> idle wake -> intake drain -> thread run.
         for (int i = 0; i < 10; ++i) this_thread::yield();
      },
      s_core0,
      thread::priority(0),
      core0
   );

   EXPECT_EQ(kernel::active_threads(), 1u);

   // WHEN:

   kernel::start();

   // THEN:

   EXPECT_TRUE(core1_work_ran)
      << "core1 did not wake from idle to run queued work (possible missing condvar poke)";
}


/*** Explicit termination via this_thread::thread_exit(), across cores ***/

TEST_F(MultiCoreScheduling_Test,
       GivenJoinersOnThreeCores_WhenTargetOnCore0CallsThreadExit_ThenEveryJoinerIsWokenAndReturns)
{
   cyros::test::guarded_stack s_target;
   std::array<cyros::test::guarded_stack, 3> s_joiners;

   std::atomic<bool> target_exited{false};
   std::array<std::atomic<bool>, 3> joiner_returned{};

   // GIVEN:

   // The retire runs on core0's arbiter, so each joiner is readied by a
   // cross-core intake post and an IPI rather than a local enqueue.
   thread target(
      [&]{
         for (int i = 0; i < 20; ++i) this_thread::yield(); // let the joiners park
         target_exited.store(true, std::memory_order_release);
         this_thread::thread_exit();
      },
      s_target,
      thread::priority(1),
      core0
   );

   std::vector<thread> joiners;
   joiners.reserve(s_joiners.size());
   auto const affinities = std::to_array({core1, core2, core3});

   for (std::size_t i = 0; i < s_joiners.size(); ++i) {
      joiners.emplace_back(
         [&, i]{
            target.join();
            ASSERT_TRUE(target_exited.load(std::memory_order_acquire));
            joiner_returned[i].store(true, std::memory_order_release);
         },
         s_joiners[i],
         thread::priority(0),
         affinities[i]
      );
   }

   ASSERT_EQ(kernel::active_threads(), 4u);

   // WHEN:

   kernel::start();

   // THEN:

   EXPECT_TRUE(target_exited.load(std::memory_order_acquire));
   for (std::size_t i = 0; i < joiner_returned.size(); ++i) {
      EXPECT_TRUE(joiner_returned[i].load(std::memory_order_acquire))
         << "joiner " << i << " was never woken by the remote thread_exit";
   }
   EXPECT_EQ(kernel::active_threads(), 0u);
}
