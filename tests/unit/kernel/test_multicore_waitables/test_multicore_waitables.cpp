#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/waitable.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include "gtest/gtest.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

using namespace cyros;

static_assert(config::cores >= 4, "Test suite is designed for (at least) quad-core configuration");

int main(int argc, char** argv)
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}

namespace
{

// Tune this knob to trade test speed for race-detection confidence.
// 200 catches most races with reasonable test runtime; bump for harder
// hunting, drop when iterating on unrelated work.
constexpr int stress_repetitions = 200;

// Each test owns its own kernel lifecycle per rep, so the fixture is empty.
// Inheriting from ::testing::Test (rather than using TEST instead of TEST_F)
// keeps the test naming consistent with the other multicore suites.
class MultiCoreWaitables_Test : public ::testing::Test {};

/**
 * @brief Test waitable with an externally-driven atomic condition.
 *
 * Mirrors the one used by single-core waitable tests. wake_one/wake_all
 * are exposed via thin wrappers so signaler threads can drive them
 * directly without needing protected access.
 */
class TestWaitable final : public waitable
{
public:
   std::atomic<bool> condition{false};

   void set_and_wake_one() noexcept
   {
      condition.store(true, std::memory_order_release);
      wake_one();
   }

   void set_and_wake_all() noexcept
   {
      condition.store(true, std::memory_order_release);
      wake_all();
   }

   void wake_one_no_set() noexcept { wake_one(); }
   void wake_all_no_set() noexcept { wake_all(); }

protected:
   bool is_satisfied(thread& /*caller*/) noexcept override
   {
      return condition.load(std::memory_order_acquire);
   }
};

}  // namespace

/* ============================================================================
 * Cross-core wake: waiter on one core, signaler on another
 *
 * The most basic SMP waitable property: a wake originating on a different
 * core than the waiter must reach the waiter via the inbox + IPI path.
 * Replicating this under stress builds confidence that the cross-core
 * routing is reliable across many timing windows.
 * ========================================================================= */
TEST_F(MultiCoreWaitables_Test,
       GivenWaiterOnCore0AndSignalerOnCore1_WhenSignalerWakes_ThenWaiterReturns)
{
   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> waiter_stack{};
      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> signaler_stack{};

      TestWaitable w;
      std::atomic<bool> waiter_completed{false};

      thread waiter(
         [&]{
            this_thread::wait_on(w);
            waiter_completed.store(true, std::memory_order_release);
         },
         waiter_stack,
         thread::priority(0),
         core0
      );

      thread signaler(
         [&]{
            w.set_and_wake_one();
         },
         signaler_stack,
         thread::priority(0),
         core1
      );

      kernel::start();

      EXPECT_TRUE(waiter_completed.load(std::memory_order_acquire));
      kernel::finalise();
   }
}

/* ============================================================================
 * Cross-core wake to a waiter parked on wait_on_any
 * ========================================================================= */
TEST_F(MultiCoreWaitables_Test,
       GivenWaiterOnCore0WithWaitOnAny_WhenCore1WakesSecondSource_ThenWinnerIs1)
{
   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> waiter_stack{};
      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> signaler_stack{};

      TestWaitable w0;
      TestWaitable w1;
      std::atomic<std::size_t> winner{static_cast<std::size_t>(-1)};

      thread waiter(
         [&]{
            winner.store(this_thread::wait_on_any(w0, w1), std::memory_order_release);
         },
         waiter_stack,
         thread::priority(0),
         core0
      );

      thread signaler(
         [&]{
            w1.set_and_wake_one();
         },
         signaler_stack,
         thread::priority(0),
         core1
      );

      kernel::start();

      EXPECT_EQ(winner.load(std::memory_order_acquire), 1U);
      kernel::finalise();
   }
}

/* ============================================================================
 * Two signalers on different cores both wake the same waiter
 *
 * This is the double-wake idempotency case: two cores nearly simultaneously
 * call wake_one on the same waitable. The first wake unlinks the waiter;
 * the second wake finds the queue empty and is a no-op. The waiter must
 * not be enqueued twice into the ready matrix.
 *
 * If double-enqueue happened, the most likely symptom would be a corrupt
 * ready list - either an assert in the scheduler, an infinite loop, or a
 * use-after-free. So "test passes" here is "kernel survives the scenario."
 * ========================================================================= */
TEST_F(MultiCoreWaitables_Test,
       GivenTwoSignalersOnDifferentCores_WhenBothWakeSameWaitable_ThenWaiterReturnsOnce)
{
   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> waiter_stack{};
      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> s1_stack{};
      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> s2_stack{};

      TestWaitable w;
      std::atomic<int> completion_count{0};

      thread waiter(
         [&]{
            this_thread::wait_on(w);
            completion_count.fetch_add(1, std::memory_order_acq_rel);
         },
         waiter_stack,
         thread::priority(0),
         core0
      );

      thread signaler1(
         [&]{
            w.set_and_wake_one();
         },
         s1_stack,
         thread::priority(0),
         core1
      );

      thread signaler2(
         [&]{
            // Don't set condition again - just wake. If the first wake
            // already woke the waiter, this is a wake against an empty
            // queue (no-op). If somehow the queue still has the waiter,
            // this re-wakes (which would also be fine, just redundant).
            w.wake_one_no_set();
         },
         s2_stack,
         thread::priority(0),
         core2
      );

      kernel::start();

      EXPECT_EQ(completion_count.load(std::memory_order_acquire), 1)
         << "waiter must complete exactly once, no matter how many wakes target it";
      kernel::finalise();
   }
}

/* ============================================================================
 * wake_all across cores: three waiters on different cores, one signaler
 *
 * Exercises the multi-core fan-out of wake_all: the signaler's wake_all
 * posts set_thread_ready into three different schedulers' inboxes (or
 * directly readies its own core's waiter, depending on placement). All
 * three waiters must return.
 * ========================================================================= */
TEST_F(MultiCoreWaitables_Test,
       GivenThreeWaitersOnThreeCores_WhenSignalerOnFourthWakesAll_ThenAllThreeReturn)
{
   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> w0_stack{};
      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> w1_stack{};
      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> w2_stack{};
      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> signaler_stack{};

      TestWaitable w;
      std::atomic<int> woke_count{0};

      thread waiter0(
         [&]{ this_thread::wait_on(w); woke_count.fetch_add(1, std::memory_order_acq_rel); },
         w0_stack, thread::priority(0), core0);

      thread waiter1(
         [&]{ this_thread::wait_on(w); woke_count.fetch_add(1, std::memory_order_acq_rel); },
         w1_stack, thread::priority(0), core1);

      thread waiter2(
         [&]{ this_thread::wait_on(w); woke_count.fetch_add(1, std::memory_order_acq_rel); },
         w2_stack, thread::priority(0), core2);

      thread signaler(
         [&]{
            // Yield a few times to maximise the chance all three waiters
            // are parked before we fire. This isn't strictly required for
            // correctness - a waiter that hasn't yet armed will see the
            // condition true on its first check - but exercising the
            // "all three are blocked" path is more interesting.
            for (int i = 0; i < 5; ++i) this_thread::yield();
            w.set_and_wake_all();
         },
         signaler_stack,
         thread::priority(0),
         core3
      );

      kernel::start();

      EXPECT_EQ(woke_count.load(std::memory_order_acquire), 3);
      kernel::finalise();
   }
}

/* ============================================================================
 * Cross-core wake racing the waiter's own arm
 *
 * This is the scenario the "case ready" scheduler tolerance was designed
 * for. The signaler on core1 calls set_and_wake_one in a loop; the waiter
 * on core0 calls wait_on in a loop. The wake may fire BEFORE the waiter
 * has finished arming and yielded - the inbox routing must serialise this
 * so the wake isn't lost.
 *
 * We can't deterministically trigger the inner-window race on a
 * cooperative port, but running many iterations gives the OS scheduler
 * many chances to interleave the two pthreads adversarially.
 * ========================================================================= */
TEST_F(MultiCoreWaitables_Test,
       GivenWakeRacingWithArm_WhenStressed_ThenNoWakeIsLost)
{
   constexpr int iterations_per_rep = 50;

   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));
      //std::printf("\nTEST ITERATION: %d\n", rep);

      kernel::initialise();

      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 32 * 1024> waiter_stack{};
      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 32 * 1024> signaler_stack{};

      TestWaitable w;
      std::atomic<int> rounds_completed{0};

      thread waiter(
         [&]{
            for (int i = 0; i < iterations_per_rep; ++i) {
               this_thread::wait_on(w);
               // Reset for next round. The signaler will set it again.
               w.condition.store(false, std::memory_order_release);
               rounds_completed.fetch_add(1, std::memory_order_acq_rel);
            }
         },
         waiter_stack,
         thread::priority(0),
         core0
      );

      thread signaler(
         [&]{
            for (int i = 0; i < iterations_per_rep; ++i) {
               // Wait until the waiter has consumed the previous wake.
               // Without this, the signaler races ahead and sets the
               // condition before the waiter has cleared it.
               while (w.condition.load(std::memory_order_acquire)) {
                  this_thread::yield();
               }
               w.set_and_wake_one();
            }
         },
         signaler_stack,
         thread::priority(0),
         core1
      );

      kernel::start();

      EXPECT_EQ(rounds_completed.load(std::memory_order_acquire), iterations_per_rep)
         << "waiter completed fewer rounds than the signaler sent wakes - lost wake";
      kernel::finalise();
   }
}

/* ============================================================================
 * wait_on_any across sources woken by different cores
 *
 * The waiter on core0 calls wait_on_any over two sources. Each source has
 * its own signaler on a different core. Exactly one source fires per rep;
 * the winner index must match.
 * ========================================================================= */
TEST_F(MultiCoreWaitables_Test,
       GivenWaitOnAnyAcrossSourcesWokenByDifferentCores_ThenWinnerIndexIsCorrect)
{
   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> waiter_stack{};
      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> s_for_w0_stack{};
      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> s_for_w1_stack{};

      TestWaitable w0;
      TestWaitable w1;
      std::atomic<std::size_t> winner{static_cast<std::size_t>(-1)};

      // Alternate which source fires per rep so both branches get exercised.
      const bool fire_w0_this_rep = (rep % 2 == 0);

      thread waiter(
         [&]{
            winner.store(this_thread::wait_on_any(w0, w1), std::memory_order_release);
         },
         waiter_stack,
         thread::priority(0),
         core0
      );

      thread signaler_w0(
         [&]{
            if (fire_w0_this_rep) {
               w0.set_and_wake_one();
            }
         },
         s_for_w0_stack,
         thread::priority(0),
         core1
      );

      thread signaler_w1(
         [&]{
            if (!fire_w0_this_rep) {
               w1.set_and_wake_one();
            }
         },
         s_for_w1_stack,
         thread::priority(0),
         core2
      );

      kernel::start();

      const std::size_t expected = fire_w0_this_rep ? 0U : 1U;
      EXPECT_EQ(winner.load(std::memory_order_acquire), expected);
      kernel::finalise();
   }
}

/* ============================================================================
 * Cross-core join: target on one core, joiner on another
 *
 * thread::join() is implemented via wait_on(termination), so this is a
 * waitable test in disguise. With the target on core0 and the joiner on
 * core1, the target's thread_termination::terminate() (running on core0)
 * must wake the joiner parked on core1.
 * ========================================================================= */
TEST_F(MultiCoreWaitables_Test,
       GivenTargetOnCore0AndJoinerOnCore1_WhenTargetTerminates_ThenJoinerReturns)
{
   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> target_stack{};
      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> joiner_stack{};

      std::atomic<bool> joiner_returned{false};

      thread target(
         [&]{
            // Just terminate cleanly. The joiner is parked on our termination.
         },
         target_stack,
         thread::priority(0),
         core0
      );

      thread joiner(
         [&]{
            target.join();
            joiner_returned.store(true, std::memory_order_release);
         },
         joiner_stack,
         thread::priority(0),
         core1
      );

      kernel::start();

      EXPECT_TRUE(joiner_returned.load(std::memory_order_acquire));
      kernel::finalise();
   }
}