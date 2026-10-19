/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_spinlock_single_core.cpp
 * @brief cyros::spinlock built for ONE core, where its flag is a plain store.
 *
 * On one core, masking interrupts already excludes everyone who could take
 * the lock, so the flag is set and cleared by plain stores instead of the
 * test-and-set test_spinlock exercises on four. That leaves the flag two jobs,
 * both observable only here: is_locked() must report it, and a try_lock on
 * this core while it is held must fail and leave the caller as it was. The
 * masking half is observed through cyros_port_interrupts_enabled(), as in
 * test_spinlock.
 */

#include <cyros/kernel/core.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/spinlock.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/guarded_stack.hpp>

#include <gtest/gtest.h>

using namespace cyros;

static_assert(config::cores == 1, "This is the one-core build of the lock");

namespace
{

class SpinlockSingleCore_Test : public ::testing::Test
{
protected:
   void SetUp() override    { kernel::initialise(); }
   void TearDown() override { kernel::finalise(); }
};

} // namespace

TEST_F(SpinlockSingleCore_Test, GivenOneCore_WhenTheLockIsTaken_ThenItIsHeldAndMaskedUntilUnlock)
{
   struct
   {
      spinlock lock;
      bool held_before{true};
      bool masked_while_held{false};
      bool held_while_held{false};
      bool held_after{true};
      bool enabled_after{false};
   } s;

   cyros::test::guarded_stack stack;
   thread t(
      [&s]{
         s.held_before = s.lock.is_locked();
         s.lock.lock();
         s.masked_while_held = !cyros_port_interrupts_enabled();
         s.held_while_held = s.lock.is_locked();
         s.lock.unlock();
         s.held_after = s.lock.is_locked();
         s.enabled_after = cyros_port_interrupts_enabled();
      },
      stack, thread::priority(1), core0
   );

   kernel::start();

   EXPECT_FALSE(s.held_before);
   EXPECT_TRUE(s.masked_while_held) << "holding the lock did not mask interrupts";
   EXPECT_TRUE(s.held_while_held) << "is_locked() missed a held lock";
   EXPECT_FALSE(s.held_after) << "unlock left the lock reported held";
   EXPECT_TRUE(s.enabled_after) << "unlock did not restore interrupts";
}

/* The one way a single core meets its own held lock without deadlocking: a
 * try_lock nested inside the holder. It must fail, and leave the mask exactly
 * as the holder had it, masked, rather than restoring it under the holder. */
TEST_F(SpinlockSingleCore_Test, GivenTheLockHeld_WhenTryLockedOnTheSameCore_ThenItFailsAndLeavesTheMask)
{
   struct
   {
      spinlock lock;
      bool nested_acquired{true};
      bool masked_after_failure{false};
      bool held_after_failure{false};
      bool enabled_after{false};
   } s;

   cyros::test::guarded_stack stack;
   thread t(
      [&s]{
         s.lock.lock();
         s.nested_acquired = s.lock.try_lock();
         s.masked_after_failure = !cyros_port_interrupts_enabled();
         s.held_after_failure = s.lock.is_locked();
         s.lock.unlock();
         s.enabled_after = cyros_port_interrupts_enabled();
      },
      stack, thread::priority(1), core0
   );

   kernel::start();

   EXPECT_FALSE(s.nested_acquired) << "a held lock was taken again";
   EXPECT_TRUE(s.masked_after_failure) << "the failed try_lock unmasked under the holder";
   EXPECT_TRUE(s.held_after_failure);
   EXPECT_TRUE(s.enabled_after);
}

TEST_F(SpinlockSingleCore_Test, GivenAGuard_WhenItLeavesScope_ThenTheLockIsReleased)
{
   struct
   {
      spinlock lock;
      bool held_in_scope{false};
      bool released_after{false};
   } s;

   cyros::test::guarded_stack stack;
   thread t(
      [&s]{
         {
            spinlock_guard guard(s.lock);
            s.held_in_scope = s.lock.is_locked();
         }
         s.released_after = !s.lock.is_locked();
      },
      stack, thread::priority(1), core0
   );

   kernel::start();

   EXPECT_TRUE(s.held_in_scope);
   EXPECT_TRUE(s.released_after);
}
