/**
 * @file test_cortex_m33_sync.cpp
 * @brief Semaphore, mutex and priority inheritance, on the target.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: cyros::sync::semaphore and cyros::sync::mutex, including the
 *          priority-inheritance path through base_mutex.
 * Trusts:  layers 0 to 2, so the port's masking, its stack guard, its SysTick,
 *          and the kernel bring-up.
 * Proves:  that blocking, waking and PI work when the context switch is a real
 *          PendSV exception rather than a library call or a signal return.
 *
 *
 * WHY THIS IS WORTH HAVING
 * ========================
 * Priority inheritance is the most carefully designed and most subtle thing in
 * cyros, and `cross-core-validation.md` opens by saying that every cross-core
 * mechanism in the project has been validated "exclusively against one Linux
 * port on x86-64". This file does not close that gap, because it is single
 * core and QEMU's TCG does not model weak memory. What it DOES close is a
 * different and narrower one: until now, no PI code had ever run anywhere that
 * the context switch was a hardware exception.
 *
 * The parts that could plausibly differ and are exercised here: a wake issued
 * from inside a mutex release while the releasing thread is still boosted, a
 * block that has to unwind through PendSV rather than through a fiber jump,
 * and a priority change taking effect at a hardware-defined safe point instead
 * of a software-simulated one.
 *
 *
 * SEQUENCING ON ONE CORE WITHOUT A CLOCK
 * ======================================
 * Single core with strict priority scheduling, so "what runs next" is fully
 * determined and the test needs no timing at all. A semaphore is used as a
 * one-shot gate to get the threads into the interleaving PI requires:
 *
 *   H (priority 1) starts first, being highest, and immediately blocks on the
 *     gate. That hands the core to L.
 *   L (priority 5) takes the mutex, then releases the gate. H is now ready and
 *     higher priority, so it preempts L at once.
 *   H tries the mutex, finds L holding it, and blocks. Blocking is what raises
 *     L to H's priority. The core returns to L.
 *   L observes its own effective priority, which is the assertion, and
 *     releases.
 *
 * Lower number is higher priority throughout.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/sync/mutex.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <common/arm/bench.hpp>

#include <cstddef>
#include <cstdint>

using namespace cyros;

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 2048;

alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_high[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_low[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_counter[stack_size];

constexpr std::uint8_t priority_high = 1;
constexpr std::uint8_t priority_low  = 5;

sync::semaphore gate{0};
sync::mutex     resource;

/* Observations, all written by exactly one thread and read after both have
 * finished, so no synchronisation of their own is needed. */
volatile std::uint8_t low_priority_while_h_waited = 0xFF;
volatile std::uint8_t low_priority_after_release  = 0xFF;
volatile bool high_acquired_the_mutex = false;
volatile int  order_index = 0;
volatile int  order[8]    = {};

void mark(int value)
{
   /* Explicit read-modify-write: ++ on a volatile is deprecated since C++20
    * and -Werror rejects it. */
   int const index = order_index;
   if (index < 8) {
      order[index] = value;
      order_index = index + 1;
   }
}


/* ---------------------------------------------------------------------------
 * Priority inheritance
 * ------------------------------------------------------------------------ */

void high_thread()
{
   mark(1);

   /* Blocks at once: the gate starts empty. This is what lets the LOWER
    * priority thread run first, which the scenario needs. */
   gate.acquire();

   mark(3);

   /* L holds it, so this blocks and boosts L to this thread's priority. */
   resource.lock();
   mark(5);
   high_acquired_the_mutex = true;
   resource.unlock();
}

void low_thread()
{
   mark(2);

   resource.lock();

   /* Wakes H, which immediately preempts this thread and blocks on the mutex,
    * boosting this thread before control comes back here. */
   gate.release();

   /* THE ASSERTION. Control only returns here because H blocked on the mutex
    * this thread holds, and the boost is what the PI substrate exists to do. */
   low_priority_while_h_waited = this_thread::priority();
   mark(4);

   resource.unlock();

   /* Releasing hands the mutex and the core to H, so this line runs only after
    * H has had it. The boost must be gone by then. */
   low_priority_after_release = this_thread::priority();
   mark(6);
}


/* ---------------------------------------------------------------------------
 * Plain semaphore counting, as a control
 * ------------------------------------------------------------------------ */

sync::semaphore work_available{0};
volatile int items_consumed = 0;

void report_and_finish();

void counter_thread()
{
   for (int i = 0; i < 4; ++i) {
      work_available.acquire();
      items_consumed = items_consumed + 1;
   }

   /* Lowest priority of the three, and its semaphore is pre-loaded so it never
    * blocks. It therefore gets the core only once the PI scenario above has
    * fully played out, which makes it the right place to report from. There is
    * nowhere else: kernel::start() does not return on this port. */
   report_and_finish();
}


void report_and_finish()
{
   cyros::bench::start("both threads reached their mutex sections in order");
   cyros::bench::print("  order = [");
   for (int i = 0; i < order_index; ++i) {
      cyros::bench::print_hex(static_cast<std::uint32_t>(order[i]));
      if (i + 1 < order_index) { cyros::bench::print(", "); }
   }
   cyros::bench::print("]\n");

   /* H starts (1), blocks on the gate so L runs (2), L opens the gate and H
    * preempts (3), H blocks on the mutex so L resumes boosted (4), L releases
    * and H takes it (5), then L finishes (6). */
   bool const order_ok =
      order_index == 6 &&
      order[0] == 1 && order[1] == 2 && order[2] == 3 &&
      order[3] == 4 && order[4] == 5 && order[5] == 6;
   CYROS_CHECK(order_ok);

   cyros::bench::start("the high-priority thread got the mutex");
   CYROS_CHECK(high_acquired_the_mutex);

   cyros::bench::start("holding a contended mutex BOOSTS the holder");
   cyros::bench::print("  low thread's effective priority while high waited = ");
   cyros::bench::print_hex(low_priority_while_h_waited);
   cyros::bench::print("\n");
   CYROS_CHECK_EQ(low_priority_while_h_waited, priority_high);

   cyros::bench::start("and the boost is dropped on release");
   cyros::bench::print("  effective priority after unlock = ");
   cyros::bench::print_hex(low_priority_after_release);
   cyros::bench::print("\n");
   CYROS_CHECK_EQ(low_priority_after_release, priority_low);

   cyros::bench::start("a semaphore counts every release");
   CYROS_CHECK_EQ(items_consumed, 4);

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("cortex_m33 sync: semaphore, mutex, priority inheritance\n\n");

   kernel::initialise();

   /* The counter runs at the lowest priority of the three, so it only gets the
    * core once the PI scenario has fully played out. Its releases are posted
    * from here, before the kernel starts, so they are simply waiting for it. */
   work_available.release(4);

   thread high(high_thread, stack_high, thread::priority(priority_high), core0);
   thread low(low_thread, stack_low, thread::priority(priority_low), core0);
   thread counter(counter_thread, stack_counter, thread::priority(7), core0);

   CYROS_CHECK_EQ(kernel::active_threads(), 3u);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
