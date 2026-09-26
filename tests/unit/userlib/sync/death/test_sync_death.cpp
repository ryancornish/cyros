/**
 * @file test_sync_death.cpp
 * @brief Misusing a mutex stops the system rather than corrupting the PI state.
 *
 * Subject: the misuse checks in base_mutex, reached through sync::mutex and
 *          sync::cemutex (L7)
 * Trusts:  the kernel harness (L2), to give the misuse a current thread
 *
 * Each of these would otherwise leave priority inheritance quietly wrong, which
 * is the worst failure this subsystem has: a boost nothing can see.
 *
 *   unlock by a thread that does not hold the lock
 *      retires a held slot that was never filed and hands ownership to a
 *      waiter that the real owner is still racing.
 *   a ceiling lock taken by a thread more urgent than its ceiling
 *      the holder then runs BELOW a thread it can block, the exact inversion a
 *      ceiling exists to prevent (POSIX answers EINVAL).
 *   a mutex destroyed while held
 *      leaves a dangling pointer in its owner's held_slots, which the urgency
 *      fold dereferences on every pick.
 *
 * Every test requires the panic's file and failing line as well as SIGABRT
 * (`common/death.hpp`), and each was mutation-verified by deleting its check.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/cemutex.hpp>
#include <cyros/sync/mutex.hpp>

#include <common/death.hpp>
#include <common/guarded_stack.hpp>

#include <gtest/gtest.h>

using namespace cyros;

namespace
{

/* Run `body` as the kernel's only thread, at `priority`. Every misuse here
 * needs a current thread, because a mutex records its owner as a TCB. */
void run_as_a_thread(void (*body)(), thread::priority priority = thread::priority(1))
{
   kernel::initialise();
   static test::guarded_stack stack;
   thread t(body, stack, priority, core0);
   kernel::start();
   kernel::finalise();
}

void unlock_without_holding()
{
   run_as_a_thread([] {
      static sync::mutex m;
      m.unlock();
   });
}

void lock_a_ceiling_mutex_from_above_its_ceiling()
{
   run_as_a_thread([] {
         static sync::cemutex c(thread::priority(5));
         c.lock();
      },
      thread::priority(1));   // more urgent than the ceiling of 5
}

void destroy_a_mutex_while_holding_it()
{
   run_as_a_thread([] {
      sync::mutex m;
      m.lock();
   });   // ~mutex with the lock still held
}

}  // namespace

TEST(SyncDeath_Test, GivenAMutexNobodyHolds_WhenUnlocked_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(unlock_without_holding(),
                      test::panicked_at("base_mutex.cpp", "Release by non-owner"));
}

TEST(SyncDeath_Test, GivenACeilingLessUrgentThanTheLocker_WhenLocked_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(lock_a_ceiling_mutex_from_above_its_ceiling(),
                      test::panicked_at("base_mutex.cpp", "ceiling below locker"))
      << "a priority-1 thread took a lock whose ceiling is 5, so it would run below threads it blocks";
}

TEST(SyncDeath_Test, GivenAHeldMutex_WhenDestroyed_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(destroy_a_mutex_while_holding_it(),
                      test::panicked_at("base_mutex.cpp", "Still owned"))
      << "a held mutex was destroyed, leaving its owner's held slot dangling";
}
