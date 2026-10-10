/**
 * @file test_target_smp_wait_cost.cpp
 * @brief What a wake ACROSS cores costs, in cycles, on the target.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the cross-core block and wake path: a release on one core readying
 *          a thread pinned to the other, the reschedule IPI, and the switch it
 *          causes there, measured through the semaphore.
 * Trusts:  layers 0 to 5, the semaphore's counting (test_sync_semaphore), and
 *          the cross-core wake test_target_smp_ipi proves.
 * Proves:  that every hand-off completed, and REPORTS what each costs.
 *
 * test_target_wait_cost's ping-pong with the two threads on different cores.
 * One round trip is two cross-core wakes: each release readies a thread on the
 * other core and rings its IPI, that core leaves its idle thread (asleep in
 * WFI, since its only thread is blocked) and switches, and the woken thread
 * releases back. So it prices the IPI, the wake from sleep and the switch, and
 * none of it can be seen on one core.
 *
 * Timed on core 0 alone, so the per-core cycle counters of the QEMU machines
 * would do, if QEMU modelled them. It does not, so there every figure is zero
 * and only the checks mean anything. On the RP2350 the stamps are MTIME at the
 * core clock, on either ISA.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <cstddef>
#include <cstdint>

using namespace cyros;

static_assert(config::cores == 2, "This test is dual core");
static_assert(CYROS_PORT_CORE_COUNT == 2, "Needs a two-core port");

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 1024;
constexpr int         per_round  = 256;
constexpr int         rounds     = 16;

alignas(CYROS_PORT_STACK_ALIGN) std::byte ping_stack[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte pong_stack[stack_size];

sync::semaphore to_pong{0};
sync::semaphore to_ping{0};

volatile int pongs = 0;

void pong()
{
   while (true) {
      to_pong.acquire();
      pongs = pongs + 1;
      to_ping.release();
   }
}

void ping()
{
   std::uint64_t best = ~std::uint64_t{0};
   std::uint64_t worst = 0;

   for (int r = 0; r < rounds; ++r) {
      std::uint64_t const t0 = cyros_port_timestamp();
      for (int i = 0; i < per_round; ++i) {
         to_pong.release();
         to_ping.acquire();
      }
      std::uint64_t const t1 = cyros_port_timestamp();
      auto const trip = (t1 - t0) / per_round;
      best = trip < best ? trip : best;
      worst = trip > worst ? trip : worst;
   }

   cyros::bench::start("every ping was answered from the other core");
   CYROS_CHECK_EQ(pongs, rounds * per_round);
   CYROS_CHECK_EQ(to_pong.peek(), 0);
   CYROS_CHECK_EQ(to_ping.peek(), 0);

   cyros::bench::start("cycles, the mean over 256, best and worst of 16 rounds");
   cyros::bench::print("  cross-core round trip (2 blocking waits, 2 wakes, 2 IPIs, 2 switches)  ");
   cyros::bench::print_dec(best);
   cyros::bench::print(" to ");
   cyros::bench::print_dec(worst);
   cyros::bench::print("\n");
   if (best == 0) {
      cyros::bench::print("  every figure is zero: no cycle counter here (QEMU does not model one)\n");
   }

   cyros::bench::finish();
}

} // namespace

extern "C" int cyros_bench_main()
{
   cyros::bench::print("wait cost across cores: a semaphore ping-pong between core 0 and core 1\n\n");

   kernel::initialise();

   thread ponger(pong, pong_stack, thread::priority(1), core1);
   thread pinger(ping, ping_stack, thread::priority(2), core0);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
