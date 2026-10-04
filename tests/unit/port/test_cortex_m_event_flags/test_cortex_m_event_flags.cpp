/**
 * @file test_cortex_m_event_flags.cpp
 * @brief event_flags on the target, and what a set costs with idle waiters.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: cyros::sync::event_flags.
 * Trusts:  layers 0 to 5, and the semaphore used to enlist waiters (L6).
 * Proves:  that a set releases exactly the waiter whose bit it set when the
 *          switch is a real PendSV, and MEASURES what the others cost: set()
 *          wakes every waiter, and each one whose bit is clear re-tests its
 *          mask and parks again.
 *
 * N waiters wait on N different bits, one each, at priorities 2 upwards. The
 * driver (priority 20) sets the bit of the LEAST urgent waiter, so every other
 * waiter wakes, runs, re-tests and parks before the target runs: the worst
 * case. The figure runs from the set to the driver's return, by which time the
 * target has consumed its bit and parked again. The difference between N and
 * N=1, divided by N-1, is what one idle waiter costs a set.
 *
 * Cycles come from cyros_port_timestamp, the DWT counter on the M33 and the
 * M4. QEMU does not model it, so there every figure is zero and only the checks
 * mean anything: tests/hardware/u575/run_test.sh test_cortex_m_event_flags.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/event_flags.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/arm/bench.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

using namespace cyros;

namespace
{

constexpr std::size_t stack_size  = thread::min_stack_size + 1024;
constexpr std::size_t max_waiters = 8;
constexpr int         rounds      = 16;

alignas(CYROS_PORT_STACK_ALIGN) std::byte waiter_stacks[max_waiters][stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte driver_stack[stack_size];

sync::event_flags flags{0};

std::array<sync::semaphore, max_waiters> enlist{
   sync::semaphore{0}, sync::semaphore{0}, sync::semaphore{0}, sync::semaphore{0},
   sync::semaphore{0}, sync::semaphore{0}, sync::semaphore{0}, sync::semaphore{0},
};

// Touched by one thread at a time: one core, and every hand-off is a block or
// a wake.
bool phase_over = false;
std::array<int, max_waiters> satisfied{};

constexpr std::uint32_t bit(std::size_t i) { return std::uint32_t{1} << i; }

void waiter(std::size_t const index)
{
   while (true) {
      enlist[index].acquire();
      while (true) {
         (void)flags.wait(bit(index), flags_match::any, flags_exit::consume);
         if (phase_over) break;
         satisfied[index] = satisfied[index] + 1;
      }
   }
}

void waiter_0() { waiter(0); }
void waiter_1() { waiter(1); }
void waiter_2() { waiter(2); }
void waiter_3() { waiter(3); }
void waiter_4() { waiter(4); }
void waiter_5() { waiter(5); }
void waiter_6() { waiter(6); }
void waiter_7() { waiter(7); }

void driver()
{
   constexpr std::array<std::size_t, 4> sizes{ 1, 2, 4, 8 };
   std::uint64_t best[4]{};
   bool only_target = true;

   for (std::size_t z = 0; z < sizes.size(); ++z) {
      std::size_t const n = sizes[z];
      std::size_t const target = n - 1;

      phase_over = false;
      satisfied = {};
      // Each is more urgent, so it runs at once and parks in wait.
      for (std::size_t i = 0; i < n; ++i) {
         enlist[i].release();
      }

      best[z] = ~std::uint64_t{0};
      for (int r = 0; r < rounds; ++r) {
         std::uint64_t const t0 = cyros_port_timestamp();
         flags.set(bit(target));
         std::uint64_t const t1 = cyros_port_timestamp();
         best[z] = (t1 - t0) < best[z] ? (t1 - t0) : best[z];
      }

      only_target = only_target && satisfied[target] == rounds;
      for (std::size_t i = 0; i < target; ++i) {
         only_target = only_target && satisfied[i] == 0;
      }

      // Release the phase: every waiter consumes its bit, sees the flag, and
      // parks on its enlist semaphore again.
      phase_over = true;
      std::uint32_t all = 0;
      for (std::size_t i = 0; i < n; ++i) all |= bit(i);
      flags.set(all);
   }

   cyros::bench::start("each set released exactly the waiter whose bit it set, every round");
   CYROS_CHECK(only_target);
   CYROS_CHECK_EQ(flags.peek(), 0u);

   cyros::bench::start("set to return, target least urgent, cycles (min over 16 rounds)");
   for (std::size_t z = 0; z < sizes.size(); ++z) {
      cyros::bench::print("  N=");
      cyros::bench::print_dec(sizes[z]);
      cyros::bench::print("  ");
      cyros::bench::print_dec(best[z]);
      cyros::bench::print("\n");
   }
   if (best[0] == 0) {
      cyros::bench::print("  every figure is zero: no cycle counter here (QEMU does not model the DWT)\n");
   }

   cyros::bench::finish();
}

} // namespace

extern "C" int cyros_bench_main()
{
   cyros::bench::print("cortex_m event_flags: what a set costs with idle waiters\n\n");

   kernel::initialise();

   constexpr std::array<void (*)(), max_waiters> entries{
      waiter_0, waiter_1, waiter_2, waiter_3, waiter_4, waiter_5, waiter_6, waiter_7,
   };
   std::array<thread, max_waiters> waiters{};
   for (std::size_t i = 0; i < max_waiters; ++i) {
      waiters[i] = thread(entries[i], waiter_stacks[i], thread::priority(static_cast<std::uint8_t>(2 + i)), core0);
   }
   thread driving(driver, driver_stack, thread::priority(20), core0);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
