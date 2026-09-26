/**
 * @file test_kernel_death.cpp
 * @brief The kernel's caller preconditions really stop the system.
 *
 * Subject: the misuse checks on `thread` and `kernel::` (L2)
 * Trusts:  the port's panic path, and nothing else. Most of these die before a
 *          kernel ever starts.
 *
 * Every check pinned here guards a mistake that is otherwise SILENT on this
 * design, which is what earns it a death test:
 *
 *   a thread handle destroyed before its thread finished
 *      The caller owns the stack, so the stack can go out of scope while the
 *      kernel still schedules onto it. Memory corruption, not a crash.
 *   a handle assigned over a live thread
 *      The same abandonment, reached through operator=. On 2026-09-19 this
 *      check was added and could NOT be pinned, because a death test for it
 *      did not exist. It is also the clearest case for matching the location:
 *      with this check removed the process STILL aborts, later, in the
 *      destructor, so a death test that only asked "did it die" would pass.
 *   kernel lifecycle out of order, a stack too small for its TCB, a thread
 *   pinned to a core the build does not have
 *      Each would otherwise run on into state the kernel never set up.
 *
 * Every test uses CYROS_EXPECT_PANIC (`common/death.hpp`), which requires the
 * panic to come from the named file, on the line holding the named text, as
 * well as SIGABRT. Each was mutation-verified by deleting its check.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/port/port_traits.h>

#include <common/death.hpp>
#include <common/guarded_stack.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <utility>

using namespace cyros;

namespace
{

void never_runs() {}

/* Each misuse is a free function, because the death-test macro splits its
 * argument on commas. They run only in the death-test child. */

void destroy_a_handle_whose_thread_never_ran()
{
   kernel::initialise();
   static test::guarded_stack stack;
   thread t(never_runs, stack, thread::priority(0), core0);
}   // ~thread, and the thread has not even started

void assign_over_a_live_thread()
{
   kernel::initialise();
   static test::guarded_stack a_stack;
   static test::guarded_stack b_stack;
   thread a(never_runs, a_stack, thread::priority(0), core0);
   thread b(never_runs, b_stack, thread::priority(0), core0);
   a = std::move(b);
}

void initialise_twice()
{
   kernel::initialise();
   kernel::initialise();
}

void start_before_initialise()
{
   kernel::start();
}

void start_with_no_threads()
{
   kernel::initialise();
   kernel::start();
}

void give_a_thread_too_small_a_stack()
{
   kernel::initialise();
   alignas(CYROS_PORT_STACK_ALIGN) static std::byte tiny[256];
   thread t(never_runs, tiny, thread::priority(0), core0);
}

void pin_a_thread_to_a_core_that_does_not_exist()
{
   kernel::initialise();
   static test::guarded_stack stack;
   thread t(never_runs, stack, thread::priority(0), core1);
}

}  // namespace

TEST(KernelDeath_Test, GivenAThreadThatNeverRan_WhenItsHandleIsDestroyed_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(destroy_a_handle_whose_thread_never_ran(),
                      test::panicked_at("threading_subsystem.cpp", "CYROS_ASSERT(tcb->state == thread_state::terminated);"))
      << "a thread handle went out of scope while its thread could still run on the caller's stack";
}

TEST(KernelDeath_Test, GivenAHandleOwningALiveThread_WhenAssignedOver_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(assign_over_a_live_thread(),
                      test::panicked_at("threading_subsystem.cpp", "Assigned over a live thread"))
      << "move assignment abandoned a live thread, or only the destructor caught it later";
}

TEST(KernelDeath_Test, GivenAnInitialisedKernel_WhenInitialisedAgain_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(initialise_twice(),
                      test::panicked_at("kernel.cpp", "Cannot invoke kernel::initialise twice"));
}

TEST(KernelDeath_Test, GivenNoInitialise_WhenTheKernelIsStarted_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(start_before_initialise(),
                      test::panicked_at("kernel.cpp", "kernel::initialise() must be called first"));
}

TEST(KernelDeath_Test, GivenNoThreads_WhenTheKernelIsStarted_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(start_with_no_threads(),
                      test::panicked_at("kernel.cpp", "Starting the kernel with no registered threads"));
}

TEST(KernelDeath_Test, GivenAStackSmallerThanItsTcb_WhenAThreadIsCreated_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(give_a_thread_too_small_a_stack(),
                      test::panicked_at("threading_subsystem.cpp", "Buffer too small for TLS+TCB"))
      << "a 256 byte stack was accepted, and the TCB would have been carved from below it";
}

TEST(KernelDeath_Test, GivenAnAffinityNamingNoConfiguredCore_WhenAThreadIsCreated_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(pin_a_thread_to_a_core_that_does_not_exist(),
                      test::panicked_at("kernel.cpp", "thread affinity mask allows no cores"));
}
