/**
 * @file test_cortex_m_trace.cpp
 * @brief The kernel trace on the target, stamped by the DWT cycle counter.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the trace points and the recorder, and cyros_port_timestamp() on
 *          the armv7m_armv8m core layer.
 * Trusts:  layers 0 to 7 as proved on the target by the other cortex_m
 *          tests, the mutex in particular.
 * Proves:  that the trace records a block, a handover and a wake in order when
 *          every switch is a PendSV exception, and that it costs nothing to
 *          read afterwards. On silicon it also shows what those steps cost in
 *          core cycles, which is the reason the stamp is a cycle count.
 *
 * QEMU does not model the DWT, so there every stamp is zero. The order checks
 * still mean something there, and the stamp checks run only where the counter
 * does, on real hardware (tests/hardware/u575/run_test.sh).
 *
 *
 * SEQUENCING ON ONE CORE WITHOUT A CLOCK
 * ======================================
 * Strict priorities, so what runs next is fully determined:
 *
 *   H (priority 1) takes the mutex, then parks on the gate.
 *   W (priority 5) is now the most urgent runnable thread. It tries the mutex
 *     and has to park on it, since H holds it and nothing else can run.
 *   G (priority 10) releases the gate. H preempts it at once, and H's unlock
 *     hands the mutex straight to W.
 *   R (priority 20) runs last, once everything above it has finished, reads
 *     the trace and reports. kernel::start() does not return on this port.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/trace.hpp>
#include <cyros/sync/mutex.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/port/port_traits.h>

#include <common/arm/bench.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

using namespace cyros;

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 2048;

alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_h[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_w[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_g[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_r[stack_size];

sync::mutex     resource;
sync::semaphore gate{0};

thread::id id_h = 0;
thread::id id_w = 0;

constexpr std::uint8_t user_tag = 42;

std::uintptr_t mutex_id()
{
   return reinterpret_cast<std::uintptr_t>(static_cast<base_mutex const*>(&resource));
}

char const* name_of(trace::event e)
{
   switch (e) {
      case trace::event::thread_created:    return "created   ";
      case trace::event::thread_ready:      return "ready     ";
      case trace::event::thread_running:    return "running   ";
      case trace::event::thread_blocked:    return "blocked   ";
      case trace::event::thread_terminated: return "terminated";
      case trace::event::remote_wake:       return "remote    ";
      case trace::event::mutex_acquired:    return "acquired  ";
      case trace::event::mutex_released:    return "released  ";
      case trace::event::user:              return "user      ";
   }
   return "?         ";
}

struct expected
{
   trace::event   what;
   thread::id     id;
   std::uintptr_t object;
   bool           check_object;
};

/* Where each expected record was found, or -1. A subsequence match: anything
 * may come between two expected records. */
template<std::size_t N>
std::array<int, N> find_in_order(trace::record const* records, std::size_t count,
                                 std::array<expected, N> const& want)
{
   std::array<int, N> at{};
   at.fill(-1);
   std::size_t next = 0;
   for (std::size_t i = 0; i < count && next < N; ++i) {
      auto const& r = records[i];
      auto const& e = want[next];
      if (r.what == e.what && r.thread == e.id && (!e.check_object || r.object == e.object)) {
         at[next++] = static_cast<int>(i);
      }
   }
   return at;
}

void thread_h()
{
   resource.lock();
   gate.acquire();   // parks, holding the mutex
   resource.unlock(); // hands it to W
}

void thread_w()
{
   resource.lock(); // parks: H holds it
   trace::user(user_tag, 7);
   trace::user(user_tag, 8); // back to back, so the gap is one record's cost
   resource.unlock();
}

void thread_g()
{
   gate.release();
}

void thread_r()
{
   std::array<trace::record, 64> records{};
   std::size_t const count = trace::read(0, records);

   cyros::bench::start("the timeline, as recorded");
   std::uint64_t const origin = count > 0 ? records[0].timestamp : 0;
   for (std::size_t i = 0; i < count; ++i) {
      auto const& r = records[i];
      cyros::bench::print("  +");
      cyros::bench::print_dec(r.timestamp - origin);
      cyros::bench::print("  ");
      cyros::bench::print(name_of(r.what));
      cyros::bench::print(" thread ");
      cyros::bench::print_dec(r.thread);
      cyros::bench::print("  priority ");
      cyros::bench::print_dec(r.priority);
      if (r.what == trace::event::thread_blocked || r.what == trace::event::mutex_acquired
          || r.what == trace::event::mutex_released) {
         cyros::bench::print(r.object == mutex_id() ? "  (the mutex)" : (r.object == 0 ? "  (not a mutex)" : "  (?)"));
      }
      cyros::bench::print("\n");
   }

   cyros::bench::start("every record fitted");
   CYROS_CHECK_EQ(trace::dropped(0), 0u);

   cyros::bench::start("a block, a handover and a wake, in order");
   std::array<expected, 8> const want{{
      {trace::event::mutex_acquired, id_h, mutex_id(), true},
      {trace::event::thread_blocked, id_h, 0,          true},  // on the gate, not a mutex
      {trace::event::thread_blocked, id_w, mutex_id(), true},  // on the mutex, named
      {trace::event::mutex_released, id_h, mutex_id(), true},
      {trace::event::mutex_acquired, id_w, mutex_id(), true},  // the handover, committed by H
      {trace::event::thread_ready,   id_w, 0,          false},
      {trace::event::thread_running, id_w, 0,          false},
      {trace::event::user,           id_w, 7,          true},
   }};
   auto const at = find_in_order(records.data(), count, want);
   for (std::size_t i = 0; i < want.size(); ++i) {
      CYROS_CHECK(at[i] >= 0);
   }

   cyros::bench::start("a user record carries its tag");
   if (at[7] >= 0) {
      CYROS_CHECK_EQ(records[static_cast<std::size_t>(at[7])].priority, user_tag);
   }

   cyros::bench::start("the buffer is in time order");
   bool ordered = true;
   for (std::size_t i = 1; i < count; ++i) {
      ordered = ordered && records[i - 1].timestamp <= records[i].timestamp;
   }
   CYROS_CHECK(ordered);

   bool const counting = count > 1 && records[count - 1].timestamp != 0;
   if (!counting) {
      cyros::bench::print("  every stamp is zero: no cycle counter here (QEMU does not model the DWT)\n");
   } else {
      cyros::bench::start("and the counter moves");
      CYROS_CHECK(records[count - 1].timestamp > records[0].timestamp);

      /* Two intervals the timeline brackets cleanly. Each includes the cost
       * of recording, since the stamps are taken by the recorder. */
      auto const since = [&](std::size_t from, trace::event what, thread::id id) -> int {
         for (std::size_t i = from; i < count; ++i) {
            if (records[i].what == what && records[i].thread == id) {
               return static_cast<int>(i);
            }
         }
         return -1;
      };
      int const woken  = at[2] >= 0 ? since(static_cast<std::size_t>(at[2]), trace::event::thread_ready, id_h) : -1;
      int const resumed = woken >= 0 ? since(static_cast<std::size_t>(woken), trace::event::thread_running, id_h) : -1;
      if (resumed >= 0) {
         cyros::bench::print("  wake to run, G's release readies H and H preempts G: ");
         cyros::bench::print_dec(records[static_cast<std::size_t>(resumed)].timestamp
                                 - records[static_cast<std::size_t>(woken)].timestamp);
         cyros::bench::print(" cycles\n");
      }
      if (at[7] >= 0 && static_cast<std::size_t>(at[7]) + 1 < count) {
         cyros::bench::print("  one record, two user() calls back to back: ");
         cyros::bench::print_dec(records[static_cast<std::size_t>(at[7]) + 1].timestamp
                                 - records[static_cast<std::size_t>(at[7])].timestamp);
         cyros::bench::print(" cycles\n");
      }
      if (at[3] >= 0 && at[4] >= 0) {
         cyros::bench::print("  H's unlock to the handover committed to W: ");
         cyros::bench::print_dec(records[static_cast<std::size_t>(at[4])].timestamp
                                 - records[static_cast<std::size_t>(at[3])].timestamp);
         cyros::bench::print(" cycles\n");
      }
   }

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("cortex_m trace: the kernel's own record of a handover\n\n");

   kernel::initialise();

   thread h(thread_h, stack_h, thread::priority(1), core0);
   thread w(thread_w, stack_w, thread::priority(5), core0);
   thread g(thread_g, stack_g, thread::priority(10), core0);
   thread r(thread_r, stack_r, thread::priority(20), core0);
   id_h = h.get_id();
   id_w = w.get_id();

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
