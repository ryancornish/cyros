#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include "gtest/gtest.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>

using namespace cyros;

static_assert(config::cores == 1, "Test suite is designed for single core configuration only");

static constexpr auto STACK_SIZE = thread::min_stack_size + (16 * 1024);

int main(int argc, char** argv)
{
   ::testing::InitGoogleTest(&argc, argv);

   int result = RUN_ALL_TESTS();

   return result;
}

TEST(SingleCoreSingleThread_Test,
     GivenOneRegisteredThread_WhenSystemStarts_ThenThreadExecutesAndSystemExitsCleanly)
{

   kernel::initialise();

   // GIVEN:

   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> stack;

   bool thread_ran = false;

   thread thread(
      [&thread_ran]{ thread_ran = true; },
      stack,
      thread::priority(0),
      core0
   );

   ASSERT_EQ(kernel::active_threads(), 1U);

   // WHEN:

   kernel::start();

   // THEN:

   ASSERT_TRUE(thread_ran);
   ASSERT_EQ(kernel::active_threads(), 0U);

   kernel::finalise();
}

TEST(SingleCoreSingleThread_Test,
     GivenOneThread_WhenSystemStarts_ThenThreadExecutesOnPinnedCore)
{

   kernel::initialise();

   // GIVEN:

   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> stack;

   uint32_t thread_executed_on_core = std::numeric_limits<uint32_t>::max();

   thread thread(
      [&thread_executed_on_core]{ thread_executed_on_core = this_core::id(); },
      stack,
      thread::priority(0),
      core0
   );

   ASSERT_EQ(kernel::active_threads(), 1U);

   // WHEN:

   kernel::start();

   // THEN:

   ASSERT_EQ(thread_executed_on_core, 0U);
   ASSERT_EQ(kernel::active_threads(), 0U);

   kernel::finalise();
}

TEST(SingleCoreSingleThread_Test,
     GivenOneThread_WhenSystemStarts_ThenThreadCanReferenceItsOwnPriority)
{

   kernel::initialise();

   // GIVEN:

   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> stack;

   thread::priority threads_priority(std::numeric_limits<uint8_t>::max());

   thread thread(
      [&threads_priority]{ threads_priority = this_thread::priority(); },
      stack,
      thread::priority(4),
      core0
   );

   ASSERT_EQ(kernel::active_threads(), 1U);

   // WHEN:

   kernel::start();

   // THEN:

   ASSERT_EQ(threads_priority, 4U);
   ASSERT_EQ(kernel::active_threads(), 0U);

   kernel::finalise();
}

TEST(SingleCoreSingleThread_Test,
     GivenOneThreadWithMultipleYieldPoints_WhenSystemStarts_ThenThreadIsProgressivelyRescheduledUntilCompletion)
{
   kernel::initialise();

   // GIVEN:

   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> stack;

   bool thread_completed = false;

   auto entry = [&thread_completed]{
      this_thread::yield();
      this_thread::yield();
      this_thread::yield();
      thread_completed = true;
   };

   thread thread(
      entry,
      stack,
      thread::priority(0),
      core0
   );

   ASSERT_EQ(kernel::active_threads(), 1U);

   // WHEN:

   kernel::start();

   // THEN:

   ASSERT_TRUE(thread_completed);
   ASSERT_EQ(kernel::active_threads(), 0U);

   kernel::finalise();
}

TEST(SingleCoreSingleThread_Test,
     GivenTwoKernelLifecycles_WhenOneThreadRegistersInEach_ThenBothGetTheFirstUserId)
{
   // Id 0 is reserved for the idle thread. kernel::initialise() steps the
   // generator past it and kernel::finalise() rewinds it, so the first thread of
   // EVERY lifecycle is id 1, whatever ran in this process before. Two
   // lifecycles inside one test, so the rewind is checked without depending on
   // the order gtest runs cases in.

   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> stack;

   for (int lifecycle = 0; lifecycle < 2; ++lifecycle) {
      kernel::initialise();

      // GIVEN:

      thread thread([]{}, stack, thread::priority(0), core0);

      // THEN:

      EXPECT_EQ(thread.get_id(), thread::id{1}) << "in lifecycle " << lifecycle;

      kernel::start();
      kernel::finalise();
   }
}
