/**
 * @file test_spinlock.cpp
 * @brief cyros::spinlock, the primitive the kernel builds everything else on.
 *
 * Added 2026-09-19. Before this, spinlock was a public header with no test of
 * its own: every wait queue, the PI state and the intake take one, so it was
 * covered constantly but only incidentally, and nothing pinned its contract.
 *
 * The contract, from spinlock.hpp: holding the lock is an INTERRUPT-MASKING
 * critical section on the holding core, and it excludes every other core. Both
 * halves are asserted here, for every way of acquiring it: lock(), try_lock()
 * and spinlock_guard. The masking half is observed through
 * cyros_port_interrupts_enabled(), which is why this test includes the internal
 * port header: the public API has no way to ask.
 *
 * The try_lock cases are the reason this file exists in its current form. They
 * were written first and FAILED against the original spinlock: try_lock took
 * the flag without entering a critical section, while unlock always exits one,
 * so a try_lock/unlock pair unbalanced the core's interrupt depth. Alone that
 * trips the port's "unbalanced restore" assert. Nested inside an outer critical
 * section it is silent and worse: the unlock re-enables interrupts while the
 * outer section still believes they are masked.
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

#include <array>
#include <atomic>
#include <cstdint>

using namespace cyros;

static_assert(config::cores == 4, "The exclusion test runs one contender per core");

namespace
{

class Spinlock_Test : public ::testing::Test
{
protected:
   void SetUp() override    { kernel::initialise(); }
   void TearDown() override { kernel::finalise(); }
};

} // namespace


/* ============================================================================
 * Exclusion across cores
 *
 * The counter is a PLAIN integer, deliberately not atomic: a read-modify-write
 * on it is only safe if the lock really excludes the other cores, so a broken
 * lock shows up as lost updates in the final count rather than as nothing.
 * ========================================================================= */
TEST_F(Spinlock_Test, GivenFourCores_WhenEachIncrementsUnderTheLock_ThenNoUpdateIsLost)
{
   constexpr std::uint64_t per_core = 20'000;

   struct state
   {
      spinlock lock;
      std::uint64_t counter{0};   // plain on purpose, see above
   } s;

   std::array<cyros::test::guarded_stack, 4> stacks;
   constexpr std::array affinity{core0, core1, core2, core3};
   std::array<thread, 4> workers{};

   for (std::size_t i = 0; i < workers.size(); ++i) {
      workers[i] = thread(
         [&s]{
            for (std::uint64_t n = 0; n < per_core; ++n) {
               spinlock_guard guard(s.lock);
               s.counter = s.counter + 1;
            }
         },
         stacks[i], thread::priority(1), affinity[i]
      );
   }

   kernel::start();

   EXPECT_EQ(s.counter, per_core * workers.size()) << "updates were lost, the lock did not exclude";
   EXPECT_FALSE(s.lock.is_locked());
}


/* ============================================================================
 * lock(): the holder's interrupts are masked, and unlock restores them exactly
 * ========================================================================= */
TEST_F(Spinlock_Test, GivenTheLockTaken_ThenInterruptsAreMaskedUntilUnlock)
{
   struct
   {
      spinlock lock;
      bool enabled_before{false};
      bool masked_while_held{false};
      bool held_reported{false};
      bool enabled_after{false};
   } s;

   cyros::test::guarded_stack stack;
   thread t(
      [&s]{
         s.enabled_before = cyros_port_interrupts_enabled();
         s.lock.lock();
         s.masked_while_held = !cyros_port_interrupts_enabled();
         s.held_reported = s.lock.is_locked();
         s.lock.unlock();
         s.enabled_after = cyros_port_interrupts_enabled();
      },
      stack, thread::priority(1), core0
   );

   kernel::start();

   EXPECT_TRUE(s.enabled_before);
   EXPECT_TRUE(s.masked_while_held) << "holding the lock did not mask interrupts";
   EXPECT_TRUE(s.held_reported);
   EXPECT_TRUE(s.enabled_after) << "unlock did not restore interrupts";
   EXPECT_FALSE(s.lock.is_locked());
}


/* ============================================================================
 * try_lock(): the same contract as lock() when it succeeds
 *
 * A successful try_lock must leave the holder in exactly the state lock()
 * does: masked, and restored by unlock. A try_lock that only takes the flag
 * lets an ISR interrupt the holder, and an ISR wake path that then wants the
 * same lock spins forever against its own interrupted thread, which is exactly
 * what lock() masks to prevent.
 * ========================================================================= */
TEST_F(Spinlock_Test, GivenTryLockSucceeds_ThenInterruptsAreMaskedAndUnlockRestoresThem)
{
   struct
   {
      spinlock lock;
      bool acquired{false};
      bool masked_while_held{false};
      bool enabled_after{false};
   } s;

   cyros::test::guarded_stack stack;
   thread t(
      [&s]{
         s.acquired = s.lock.try_lock();
         s.masked_while_held = !cyros_port_interrupts_enabled();
         if (s.acquired) s.lock.unlock();
         s.enabled_after = cyros_port_interrupts_enabled();
      },
      stack, thread::priority(1), core0
   );

   kernel::start();

   EXPECT_TRUE(s.acquired);
   EXPECT_TRUE(s.masked_while_held) << "try_lock took the lock without masking interrupts";
   EXPECT_TRUE(s.enabled_after);
   EXPECT_FALSE(s.lock.is_locked());
}

/* A try_lock that FAILS must leave the caller exactly as it found it: not
 * holding, and with interrupts unmasked. The contender is on another core and
 * holds the lock until the prober has finished probing. */
TEST_F(Spinlock_Test, GivenALockHeldByAnotherCore_WhenTryLocked_ThenItFailsAndLeavesInterruptsAlone)
{
   struct
   {
      spinlock lock;
      std::atomic<bool> held{false};
      std::atomic<bool> probed{false};
      bool acquired{true};
      bool enabled_after_failed_probe{false};
   } s;

   std::array<cyros::test::guarded_stack, 2> stacks;

   thread holder(
      [&s]{
         s.lock.lock();
         s.held.store(true, std::memory_order_release);
         while (!s.probed.load(std::memory_order_acquire)) this_core::cpu_relax();
         s.lock.unlock();
      },
      stacks[0], thread::priority(1), core1
   );

   thread prober(
      [&s]{
         while (!s.held.load(std::memory_order_acquire)) this_core::cpu_relax();
         s.acquired = s.lock.try_lock();
         s.enabled_after_failed_probe = cyros_port_interrupts_enabled();
         s.probed.store(true, std::memory_order_release);
      },
      stacks[1], thread::priority(1), core0
   );

   kernel::start();

   EXPECT_FALSE(s.acquired) << "try_lock succeeded on a lock another core holds";
   EXPECT_TRUE(s.enabled_after_failed_probe) << "a failed try_lock left interrupts masked";
   EXPECT_FALSE(s.lock.is_locked());
}


/* ============================================================================
 * Nesting inside an outer critical section
 *
 * Interrupt masking is depth counted, so a lock taken and released inside an
 * outer critical section must leave the outer one intact: still masked after
 * the unlock, unmasked only at the outer exit. Checked for both ways of
 * acquiring, because the try_lock variant of this is the SILENT form of the
 * imbalance the file header describes: nothing asserts, interrupts just come
 * back on in the middle of a section that believes they are off.
 * ========================================================================= */
TEST_F(Spinlock_Test, GivenAnOuterCriticalSection_WhenTheLockIsTakenAndReleasedInside_ThenTheOuterStaysMasked)
{
   struct
   {
      spinlock lock;
      bool masked_after_lock_unlock{false};
      bool masked_after_try_lock_unlock{false};
      bool enabled_after_outer_exit{false};
   } s;

   cyros::test::guarded_stack stack;
   thread t(
      [&s]{
         auto const outer = this_core::enter_critical();

         s.lock.lock();
         s.lock.unlock();
         s.masked_after_lock_unlock = !cyros_port_interrupts_enabled();

         if (s.lock.try_lock()) s.lock.unlock();
         s.masked_after_try_lock_unlock = !cyros_port_interrupts_enabled();

         this_core::exit_critical(outer);
         s.enabled_after_outer_exit = cyros_port_interrupts_enabled();
      },
      stack, thread::priority(1), core0
   );

   kernel::start();

   EXPECT_TRUE(s.masked_after_lock_unlock)
      << "lock/unlock inside an outer critical section unmasked interrupts";
   EXPECT_TRUE(s.masked_after_try_lock_unlock)
      << "try_lock/unlock inside an outer critical section unmasked interrupts";
   EXPECT_TRUE(s.enabled_after_outer_exit);
}


/* ============================================================================
 * spinlock_guard releases on scope exit
 * ========================================================================= */
TEST_F(Spinlock_Test, GivenAGuard_WhenItLeavesScope_ThenTheLockIsReleasedAndInterruptsRestored)
{
   struct
   {
      spinlock lock;
      bool held_in_scope{false};
      bool released_after{false};
      bool enabled_after{false};
   } s;

   cyros::test::guarded_stack stack;
   thread t(
      [&s]{
         {
            spinlock_guard guard(s.lock);
            s.held_in_scope = s.lock.is_locked();
         }
         s.released_after = !s.lock.is_locked();
         s.enabled_after = cyros_port_interrupts_enabled();
      },
      stack, thread::priority(1), core0
   );

   kernel::start();

   EXPECT_TRUE(s.held_in_scope);
   EXPECT_TRUE(s.released_after);
   EXPECT_TRUE(s.enabled_after);
}
