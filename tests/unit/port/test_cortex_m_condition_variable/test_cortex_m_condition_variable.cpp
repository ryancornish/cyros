/**
 * @file test_cortex_m_condition_variable.cpp
 * @brief condition_variable on the target, and what notify_all's herd costs.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: cyros::sync::condition_variable.
 * Trusts:  layers 0 to 7, so the port, the kernel, the semaphore and the mutex.
 * Proves:  that notify_all releases every waiter, in priority order, when the
 *          switch is a real PendSV, and MEASURES the cost of waking the waiters
 *          and letting each re-lock, against the cost requeue would have.
 *
 *
 * THE MEASUREMENT
 * ===============
 * notify_all wakes N waiters and each re-locks the mutex. The alternative,
 * requeue, moves them onto the mutex's queue without waking them, so what it
 * would cost is at best a mutex handover chain: N threads already parked in
 * lock(), released one after another by the unlocks. That chain is measured on
 * the same waiters, so
 *
 *    herd - chain  =  what not requeueing costs
 *
 * Three shapes per N, each from the moment the driver starts releasing to the
 * moment the last waiter has had the mutex and let it go:
 *
 *    chain     the waiters are parked in lock(), the driver unlocks
 *    locked    the waiters are parked in wait(), the driver notifies UNDER the
 *              mutex, then unlocks
 *    unlocked  the same, notifying AFTER it unlocks
 *
 * Single core with strict priorities, so every interleaving is determined: the
 * waiters (priorities 2 and up) are all more urgent than the driver (20), and
 * each round runs to completion before the driver's clock read.
 *
 * Cycles come from cyros_port_timestamp, the DWT counter on the M33 and the
 * M4. QEMU does not model the DWT, so there every figure is zero and only the
 * checks mean anything. On the U575 the figures are the point: run it with
 * tests/hardware/u575/run_test.sh test_cortex_m_condition_variable --mhz 160.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/condition_variable.hpp>
#include <cyros/sync/mutex.hpp>
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

constexpr std::size_t stack_size   = thread::min_stack_size + 1024;
constexpr std::size_t max_waiters  = 8;
constexpr int         rounds       = 16;
constexpr std::uint8_t driver_priority = 20;

enum class shape : std::uint8_t { chain, locked, unlocked };

alignas(CYROS_PORT_STACK_ALIGN) std::byte waiter_stacks[max_waiters][stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte driver_stack[stack_size];

sync::mutex              m;
sync::condition_variable cv;

// Everything below is touched by one thread at a time: the core is single and
// every hand-off between the driver and a waiter is a block or a wake.
std::array<sync::semaphore, max_waiters> enlist{
   sync::semaphore{0}, sync::semaphore{0}, sync::semaphore{0}, sync::semaphore{0},
   sync::semaphore{0}, sync::semaphore{0}, sync::semaphore{0}, sync::semaphore{0},
};
shape round_shape = shape::chain;
bool  go = false;            // guarded by m
int   completed = 0;         // guarded by m
std::array<int, max_waiters> order{};
bool  order_ok = true;

void waiter(std::size_t const index)
{
   while (true) {
      enlist[index].acquire();

      if (round_shape == shape::chain) {
         m.lock();             // the driver holds it, so this parks
      } else {
         m.lock();
         while (!go) {
            cv.wait(m);
         }
      }
      order[static_cast<std::size_t>(completed)] = static_cast<int>(index);
      ++completed;
      m.unlock();
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

/* One round: enlist n waiters in the shape's parked state, then time the
 * release. Returns the cycles. */
std::uint64_t run_round(shape s, std::size_t n)
{
   round_shape = s;
   go = false;
   completed = 0;

   if (s == shape::chain) {
      m.lock();
   }
   // Each waiter is more urgent, so it runs the moment it is enlisted and parks
   // (in lock() for chain, in wait() otherwise) before this returns.
   for (std::size_t i = 0; i < n; ++i) {
      enlist[i].release();
   }

   std::uint64_t const t0 = cyros_port_timestamp();
   switch (s) {
      case shape::chain:
         m.unlock();
         break;
      case shape::locked:
         m.lock();
         go = true;
         cv.notify_all();
         m.unlock();
         break;
      case shape::unlocked:
         m.lock();
         go = true;
         m.unlock();
         cv.notify_all();
         break;
   }
   std::uint64_t const t1 = cyros_port_timestamp();

   // Every waiter is more urgent than this thread, so all have finished.
   bool this_round_ok = completed == static_cast<int>(n);
   for (std::size_t i = 0; this_round_ok && i < n; ++i) {
      this_round_ok = order[i] == static_cast<int>(i);
   }
   order_ok = order_ok && this_round_ok;
   return t1 - t0;
}

void driver()
{
   constexpr std::array<std::size_t, 4> sizes{ 1, 2, 4, 8 };
   constexpr std::array<shape, 3> shapes{ shape::chain, shape::locked, shape::unlocked };
   constexpr std::array<char const*, 3> names{ "chain   ", "locked  ", "unlocked" };

   // [size][shape] minimum and total over the rounds
   std::uint64_t best[4][3]{};
   std::uint64_t total[4][3]{};

   for (std::size_t z = 0; z < sizes.size(); ++z) {
      for (std::size_t k = 0; k < shapes.size(); ++k) {
         best[z][k] = ~std::uint64_t{0};
         for (int r = 0; r < rounds; ++r) {
            auto const cycles = run_round(shapes[k], sizes[z]);
            best[z][k] = cycles < best[z][k] ? cycles : best[z][k];
            total[z][k] += cycles;
         }
      }
   }

   cyros::bench::start("every waiter completed, most urgent first, in every round");
   CYROS_CHECK(order_ok);

   cyros::bench::start("release to last unlock, cycles (min / mean over 16 rounds)");
   for (std::size_t z = 0; z < sizes.size(); ++z) {
      for (std::size_t k = 0; k < shapes.size(); ++k) {
         cyros::bench::print("  N=");
         cyros::bench::print_dec(sizes[z]);
         cyros::bench::print("  ");
         cyros::bench::print(names[k]);
         cyros::bench::print("  ");
         cyros::bench::print_dec(best[z][k]);
         cyros::bench::print(" / ");
         cyros::bench::print_dec(total[z][k] / rounds);
         cyros::bench::print("\n");
      }
   }
   if (best[0][0] == 0) {
      cyros::bench::print("  every figure is zero: no cycle counter here (QEMU does not model the DWT)\n");
   }

   cyros::bench::finish();
}

} // namespace

extern "C" int cyros_bench_main()
{
   cyros::bench::print("cortex_m condition_variable: notify_all's herd against a handover chain\n\n");

   kernel::initialise();

   constexpr std::array<void (*)(), max_waiters> entries{
      waiter_0, waiter_1, waiter_2, waiter_3, waiter_4, waiter_5, waiter_6, waiter_7,
   };
   std::array<thread, max_waiters> waiters{};
   for (std::size_t i = 0; i < max_waiters; ++i) {
      waiters[i] = thread(entries[i], waiter_stacks[i], thread::priority(static_cast<std::uint8_t>(2 + i)), core0);
   }
   thread driving(driver, driver_stack, thread::priority(driver_priority), core0);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
