#include <cortos/kernel/kernel.hpp>
#include <cortos/config/config.hpp>
#include <cortos/port/port_traits.h>

#include "gtest/gtest.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>

using namespace cortos;

static_assert(config::CORES == 1, "Test suite is designed for single core configuration only");

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

   alignas(CORTOS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> stack;

   bool thread_ran = false;

   Thread thread(
      [&thread_ran]{ thread_ran = true; },
      stack,
      Thread::Priority(0),
      Core0
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

   alignas(CORTOS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> stack;

   uint32_t thread_executed_on_core = std::numeric_limits<uint32_t>::max();

   Thread thread(
      [&thread_executed_on_core]{ thread_executed_on_core = this_thread::core_id(); },
      stack,
      Thread::Priority(0),
      Core0
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

   alignas(CORTOS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> stack;

   Thread::Priority threads_priority(std::numeric_limits<uint8_t>::max());

   Thread thread(
      [&threads_priority]{ threads_priority = this_thread::priority(); },
      stack,
      Thread::Priority(4),
      Core0
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

   alignas(CORTOS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> stack;

   bool thread_completed = false;

   auto entry = [&thread_completed]{
      this_thread::yield();
      this_thread::yield();
      this_thread::yield();
      thread_completed = true;
   };

   Thread thread(
      entry,
      stack,
      Thread::Priority(0),
      Core0
   );

   ASSERT_EQ(kernel::active_threads(), 1U);

   // WHEN:

   kernel::start();

   // THEN:

   ASSERT_TRUE(thread_completed);
   ASSERT_EQ(kernel::active_threads(), 0U);

   kernel::finalise();
}
