/**
 * @file test_channel_isr.cpp
 * @brief The reason the feature exists: an ISR hands work to a thread.
 *
 * Subject: cyros::ch::channel (L7), under a real asynchronous timer
 * Trusts:  the time driver (L1), semaphore (L6), and the ISR-safe wake path
 *
 * The rest of the channel suite runs entirely in thread context, so it proves
 * the semantics and nothing about the claim the whole design is built around:
 * that a send is safe from an interrupt handler and that the job it queues
 * runs somewhere with interrupts on and the full kernel API available.
 *
 * Here the send happens inside a genuine timer ISR on the preempt port, one
 * that interrupted whatever was running. Single core, deliberately: the ISR
 * and the worker share a core, which is the harder arrangement and the
 * realistic one, because on a single-core MCU the deferred work has nowhere
 * else to go.
 *
 * HOW THREAD CONTEXT IS PROVED, and why the obvious check would not do it.
 * Comparing thread ids inside the job proves little, since an ISR runs on
 * whatever thread it interrupted and could report the worker's own id. So the
 * first job BLOCKS, on a semaphore that nothing releases until later. An
 * interrupt handler cannot block, so reaching the far side of that acquire is
 * the evidence, and the ordering flag records that the release really did come
 * first rather than the acquire passing straight through.
 *
 * That the worker is stuck inside that job is not a side effect to work
 * around, it is a second test: the channel fills behind it and the ISR starts
 * getting `false` back from try_send, which is the only way to observe an ISR
 * meeting a full channel and declining to block.
 *
 * Nothing here spins. Every wait is a semaphore, because on one core a
 * spin-wait in the controller would starve the worker it is waiting for.
 *
 * Assertions are counts and order only. There are no wall-clock upper bounds:
 * a job that never runs fails as a suite hang, not as a flaky margin.
 */

#include <cyros/ch/channel.hpp>

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/time/time.hpp>

#include <common/guarded_stack.hpp>

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

using namespace cyros;

namespace
{

constexpr std::uint32_t frequency = 1'000u;   // 1 kHz tick, millisecond granularity

// Enough jobs that a send which works once but not repeatedly shows up.
constexpr int jobs_wanted = 40;

// Small, so the worker stuck in the first job fills it quickly.
constexpr std::size_t channel_slots = 4;

struct shared
{
   ch::work_channel<channel_slots> ch;

   sync::semaphore gate{0};          // the first job blocks here
   sync::semaphore channel_filled{0}; // the ISR reports a full channel here
   sync::semaphore enough{0};         // a job reports the target reached here

   std::atomic<int> posted{0};
   std::atomic<int> refused{0};
   std::atomic<int> ran{0};

   std::atomic<bool> gate_released{false};
   std::atomic<bool> first_job_blocked_and_resumed{false};
   std::atomic<bool> run_returned{false};
   std::atomic<bool> reported_full{false};
};

/* The timer callback runs in ISR context. Everything it touches has to be
 * legal there: try_send takes a spinlock, moves the job, and releases a
 * semaphore, and every one of those is ISR-safe by the contract the class
 * documents. A blocking send here would be a bug the port would catch. */
void post_from_isr(void* arg) noexcept
{
   auto& s = *static_cast<shared*>(arg);

   bool const sent = s.ch.try_send([&s] {
      int const index = s.ran.fetch_add(1, std::memory_order_acq_rel);

      if (index == 0) {
         /* An ISR cannot block. Getting past this line is the whole proof. */
         s.gate.acquire();
         s.first_job_blocked_and_resumed.store(s.gate_released.load(std::memory_order_acquire),
                                               std::memory_order_release);
      }

      if (index + 1 == jobs_wanted) { s.enough.release(); }
   });

   if (sent) {
      s.posted.fetch_add(1, std::memory_order_relaxed);
   } else {
      /* A full channel, met from an interrupt handler, declined without
       * blocking. Reported once, because this fires on every tick afterwards. */
      if (s.refused.fetch_add(1, std::memory_order_relaxed) == 0) {
         s.reported_full.store(true, std::memory_order_release);
         s.channel_filled.release();
      }
   }
}

class ChannelIsr_Test : public ::testing::Test
{
protected:
   void SetUp() override
   {
      kernel::initialise();
      time::initialise(frequency);
   }

   void TearDown() override
   {
      time::finalise();
      kernel::finalise();
   }
};

}  // namespace

TEST_F(ChannelIsr_Test, GivenATimerIsr_WhenItSendsWork_ThenTheJobsRunInThreadContext)
{
   static std::array<test::guarded_stack, 2> stacks;

   /* Local, not static: `shared` holds a channel and three semaphores, none of
    * which is copyable or movable, so it cannot be reset between runs and has
    * to be built fresh. The ISR reaches it through the void* argument. */
   shared s;

   /* Priority 0, so it runs first and starts the tick before anything can
    * arm. It then blocks immediately, handing the core to the worker. */
   thread controller(
      [&] {
         time::start();
         auto const timer = time::schedule_recurring(time::from_milliseconds(1),
                                                     &post_from_isr, &s);

         // The ISR has met a full channel, which means the worker is parked
         // inside the first job. Let it go.
         s.channel_filled.acquire();
         s.gate_released.store(true, std::memory_order_release);
         s.gate.release();

         // Enough jobs have run. Stop the source, then the channel.
         s.enough.acquire();
         (void)time::cancel(timer);
         s.ch.stop();
      },
      stacks[0], thread::priority(0), core0);

   thread worker(
      [&] {
         ch::run(s.ch);
         s.run_returned.store(true, std::memory_order_release);
      },
      stacks[1], thread::priority(1), core0);

   kernel::start();

   EXPECT_TRUE(s.first_job_blocked_and_resumed.load())
      << "the first job did not block and resume, so it did not run in thread context";
   EXPECT_TRUE(s.reported_full.load())
      << "the channel never filled, so try_send was never exercised from an ISR against a full channel";
   EXPECT_GT(s.refused.load(), 0)
      << "try_send never returned false from an ISR";
   EXPECT_GE(s.ran.load(), jobs_wanted)
      << "not every job an ISR queued reached the worker";
   EXPECT_EQ(s.ran.load(), s.posted.load())
      << "the number of jobs run does not match the number the ISR queued";
   EXPECT_TRUE(s.run_returned.load())
      << "run() did not return after stop()";
}
