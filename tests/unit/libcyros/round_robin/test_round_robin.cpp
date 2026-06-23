#include <cyros/rr/round_robin.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/time/time.hpp>
#include <cyros/port/port_traits.h>
#include <cyros/config/config.hpp>

#include "gtest/gtest.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>

using namespace cyros;

static_assert(config::cores == 1, "Test suite is designed for single core configuration only");

namespace
{

alignas(CYROS_PORT_STACK_ALIGN) std::array<std::byte, 32 * 1024> a_stack{};
alignas(CYROS_PORT_STACK_ALIGN) std::array<std::byte, 32 * 1024> b_stack{};

}  // namespace

TEST(RoundRobin_Test,
     GivenTwoEqualPriorityThreads_WhenRoundRobinTimerIsActivated_ThenBothThreadsRotate)
{

   kernel::initialise();

   // GIVEN:

   constexpr std::uint64_t target = 5'000'000;

   std::atomic<std::uint64_t> a_count{0};
   std::atomic<std::uint64_t> b_count{0};
   std::atomic<bool>          a_saw_b{false};

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

   // GIVEN:

   thread a(
      [&]{ bounded_spin(a_count, b_count, &a_saw_b); },
      a_stack,
      thread::priority(0),
      core0
   );

   thread b(
      [&]{ bounded_spin(b_count, a_count, nullptr); },
      b_stack,
      thread::priority(0),
      core0
   );

   // WHEN:

   time::initialise(10);

   rr::setup_round_robin();

   time::start();

   kernel::start();

   // THEN:

   kernel::finalise();
}