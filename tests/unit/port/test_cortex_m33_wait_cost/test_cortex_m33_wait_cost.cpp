/**
 * @file test_cortex_m33_wait_cost.cpp
 * @brief What the ordinary wait path costs, in cycles, on the target.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the block and wake path every waitable shares (wait_on_any, the arm
 *          guard, wake_one), measured through the semaphore.
 * Trusts:  layers 0 to 5, and the semaphore's counting (test_sync_semaphore).
 * Proves:  that every hand-off completed, and REPORTS what each costs, so a
 *          change to the shared wait path can be priced for the waitables that
 *          never asked for it.
 *
 * Two figures, each the minimum over rounds of the mean over 256 operations:
 *
 *    ping-pong   two threads on one core, each blocking on its own semaphore
 *                and releasing the other's: one round trip is two blocking
 *                waits, two wakes and two context switches
 *    fast path   release then acquire on one thread, never blocking: the
 *                poll that succeeds on the first try
 *
 * Written against the public API alone and nothing newer than the semaphore,
 * so the same file builds on an older tree and the two can be compared.
 *
 * Cycles come from cyros_port_timestamp, the DWT counter on the M33. QEMU does
 * not model it, so there every figure is zero and only the checks mean
 * anything: tests/hardware/u575/run_test.sh test_cortex_m33_wait_cost.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/arm/bench.hpp>

#include <cstddef>
#include <cstdint>

using namespace cyros;

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 1024;
constexpr int         per_round  = 256;
constexpr int         rounds     = 16;

alignas(CYROS_PORT_STACK_ALIGN) std::byte ping_stack[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte pong_stack[stack_size];

sync::semaphore to_pong{0};
sync::semaphore to_ping{0};
sync::semaphore fast{0};

volatile int pongs = 0;

void pong()
{
   for (;;) {
      to_pong.acquire();
      pongs = pongs + 1;
      to_ping.release();
   }
}

void ping()
{
   std::uint64_t best_round_trip = ~std::uint64_t{0};
   std::uint64_t best_fast       = ~std::uint64_t{0};
   bool fast_ok = true;

   for (int r = 0; r < rounds; ++r) {
      // pong is more urgent and parked on to_pong, so each release switches to
      // it, it releases to_ping and parks, and our acquire then blocks only if
      // it has not. Either way one iteration is one full round trip.
      std::uint64_t const t0 = cyros_port_timestamp();
      for (int i = 0; i < per_round; ++i) {
         to_pong.release();
         to_ping.acquire();
      }
      std::uint64_t const t1 = cyros_port_timestamp();
      auto const trip = (t1 - t0) / per_round;
      best_round_trip = trip < best_round_trip ? trip : best_round_trip;

      std::uint64_t const f0 = cyros_port_timestamp();
      for (int i = 0; i < per_round; ++i) {
         fast.release();
         fast.acquire();
      }
      std::uint64_t const f1 = cyros_port_timestamp();
      auto const one = (f1 - f0) / per_round;
      best_fast = one < best_fast ? one : best_fast;
      fast_ok = fast_ok && fast.peek() == 0;
   }

   cyros::bench::start("every ping was answered, and the fast path never kept a token");
   CYROS_CHECK_EQ(pongs, rounds * per_round);
   CYROS_CHECK(fast_ok);

   cyros::bench::start("cycles, minimum over 16 rounds of the mean over 256");
   cyros::bench::print("  ping-pong round trip (2 blocking waits, 2 wakes, 2 switches)  ");
   cyros::bench::print_dec(best_round_trip);
   cyros::bench::print("\n  release then acquire, never blocking                          ");
   cyros::bench::print_dec(best_fast);
   cyros::bench::print("\n");
   if (best_round_trip == 0) {
      cyros::bench::print("  every figure is zero: no cycle counter here (QEMU does not model the DWT)\n");
   }

   cyros::bench::finish();
}

} // namespace

extern "C" int cyros_bench_main()
{
   cyros::bench::print("cortex_m33 wait cost: the shared block and wake path, through a semaphore\n\n");

   kernel::initialise();

   thread ponger(pong, pong_stack, thread::priority(1), core0);
   thread pinger(ping, ping_stack, thread::priority(2), core0);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
