#include <cyros/rr/round_robin.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/time/time.hpp>
#include <cyros/port/port_traits.h>
#include <cyros/config/config.hpp>

#include "gtest/gtest.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

using namespace cyros;

static_assert(config::cores == 1, "Test suite is designed for single core configuration only");

static constexpr auto STACK_SIZE = thread::min_stack_size + (32 * 1024);

namespace
{

alignas(CYROS_PORT_STACK_ALIGN) std::array<std::byte, STACK_SIZE> bootstrap_stack{};
alignas(CYROS_PORT_STACK_ALIGN) std::array<std::byte, STACK_SIZE> a_stack{};
alignas(CYROS_PORT_STACK_ALIGN) std::array<std::byte, STACK_SIZE> b_stack{};

}  // namespace

/*
 * Round-robin proof-of-life on the real preempt timer.
 *
 * Two equal-priority threads each run a tight, non-yielding spin. On a single
 * core only one runs at a time, so the ONLY thing that can hand the core to the
 * other is an asynchronous timer interrupt rotating them. That is why this test
 * is backed by a real time driver (tickless) on linux_preempt, not simulation:
 * a spinner never pumps virtual time, so under simulation the slice timer would
 * never fire and the threads would run to completion one after the other.
 *
 * The witness is a_saw_b: thread A records whether it ever observed B making
 * progress while A itself was still running. That can only be true if the slice
 * timer rotated the core from A to B and back while A was mid-run, which is
 * exactly round-robin. The target is chosen well above one slice worth of
 * iterations so at least one rotation is forced before either thread finishes.
 */
TEST(RoundRobin_Test,
     GivenTwoEqualPriorityThreads_WhenRoundRobinTimerIsActivated_ThenBothThreadsRotate)
{
   kernel::initialise();

   // GIVEN: a spin long enough to span many 1 ms slices.
   constexpr std::uint64_t target = 10'000'000;

   std::atomic<std::uint64_t> a_count{0};
   std::atomic<std::uint64_t> b_count{0};
   std::atomic<bool>          a_saw_b{false};

   thread bootstrap(
      []()
      {
         rr::enable_round_robin(time::from_milliseconds(1));
         time::start();
      },
      bootstrap_stack,
      thread::priority(0),
      core0
   );

   auto bounded_spin = [&](std::atomic<std::uint64_t>& mine,
                           std::atomic<std::uint64_t>& peer,
                           std::atomic<bool>* saw_peer)
   {
      for (std::uint64_t i = 0; i < target; ++i) {
         mine.fetch_add(1, std::memory_order_relaxed);
         if (saw_peer && peer.load(std::memory_order_relaxed) != 0) {
            saw_peer->store(true);
         }
      }
   };

   thread a(
      [&]{ bounded_spin(a_count, b_count, &a_saw_b); },
      a_stack,
      thread::priority(1),
      core0
   );

   thread b(
      [&]{ bounded_spin(b_count, a_count, nullptr); },
      b_stack,
      thread::priority(1),
      core0
   );

   // WHEN: tickless time, a 1 ms slice enabled before release (it pends and is
   // anchored at time::start()), then time and the kernel go live.
   time::initialise(1'000'000);          // 1 MHz, matches the port clock

   kernel::start();

   // THEN: both ran to completion, and A saw B progress mid-run, which is only
   // possible if the slice timer rotated them.
   EXPECT_EQ(a_count.load(), target);
   EXPECT_EQ(b_count.load(), target);
   EXPECT_TRUE(a_saw_b.load());

   kernel::finalise();
}
