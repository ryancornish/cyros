/**
 * @file test_trace.cpp
 * @brief The kernel trace: what each trace point records, in what order, and
 *        that a core's buffer neither loses nor reorders what it keeps.
 *
 * Read through the public API only, the way a tool would. Order is checked as
 * a SUBSEQUENCE: the expected records must appear in the order given, with
 * anything allowed between them, because the idle thread and preemption add
 * records that a test has no business pinning.
 *
 * Each test's interleaving is driven by priorities and gates on one core, or by
 * a flag set from the OTHER core where two cores meet.
 */

#include <cyros/kernel/core.hpp>
#include <cyros/kernel/diagnostics.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/trace.hpp>
#include <cyros/sync/mutex.hpp>
#include <cyros/sync/semaphore.hpp>

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using namespace cyros;
using namespace std::chrono_literals;

namespace
{

alignas(16) std::array<std::byte, 64 * 1024> s_a;
alignas(16) std::array<std::byte, 64 * 1024> s_b;
alignas(16) std::array<std::byte, 64 * 1024> s_c;

class Trace_Test : public ::testing::Test
{
protected:
   void SetUp() override    { kernel::initialise(); }
   void TearDown() override { kernel::finalise(); }
};

/* Everything @p core has recorded and nobody has read yet, oldest first. */
std::vector<trace::record> drain(std::uint32_t core)
{
   std::vector<trace::record> all;
   std::array<trace::record, 64> chunk{};
   while (auto const n = trace::read(core, chunk)) {
      all.insert(all.end(), chunk.begin(), chunk.begin() + static_cast<std::ptrdiff_t>(n));
   }
   return all;
}

std::uintptr_t id_of(base_mutex const& m)
{
   return reinterpret_cast<std::uintptr_t>(&m);
}

char const* name_of(trace::event e)
{
   switch (e) {
      case trace::event::thread_created:    return "created";
      case trace::event::thread_ready:      return "ready";
      case trace::event::thread_running:    return "running";
      case trace::event::thread_blocked:    return "blocked";
      case trace::event::thread_terminated: return "terminated";
      case trace::event::remote_wake:       return "remote_wake";
      case trace::event::mutex_acquired:    return "acquired";
      case trace::event::mutex_released:    return "released";
      case trace::event::user:              return "user";
   }
   return "?";
}

/* One line per record, so a failed order check shows what WAS recorded. */
std::string dump(std::vector<trace::record> const& records)
{
   std::ostringstream out;
   out << '\n';
   for (auto const& r : records) {
      out << "  " << r.timestamp << ' ' << name_of(r.what) << " thread " << r.thread
          << " core " << int{r.core} << " priority " << int{r.priority}
          << " object 0x" << std::hex << r.object << std::dec << '\n';
   }
   return out.str();
}

/* An expected record. An unset field matches anything. */
struct expected
{
   trace::event                  what;
   cyros::thread::id             thread;
   std::optional<std::uint8_t>   priority{};
   std::optional<std::uintptr_t> object{};
};

bool matches(trace::record const& r, expected const& e)
{
   return r.what == e.what && r.thread == e.thread
       && (!e.priority || r.priority == *e.priority)
       && (!e.object || r.object == *e.object);
}

/* How many of @p want appear in @p records in the given order. All of them
 * did when this returns want.size(). */
std::size_t matched_in_order(std::vector<trace::record> const& records, std::vector<expected> const& want)
{
   std::size_t next = 0;
   for (auto const& r : records) {
      if (next < want.size() && matches(r, want[next])) {
         ++next;
      }
   }
   return next;
}

/* The first record matching @p e, if any. */
std::optional<trace::record> first(std::vector<trace::record> const& records, expected const& e)
{
   for (auto const& r : records) {
      if (matches(r, e)) {
         return r;
      }
   }
   return std::nullopt;
}

} // namespace

TEST_F(Trace_Test, GivenAThread_WhenItRunsToCompletion_ThenItsWholeLifeIsRecordedInOrder)
{
   thread a([]{}, s_a, thread::priority(5), core0);
   auto const a_id = a.get_id();

   kernel::start();

   auto const records = drain(0);
   EXPECT_EQ(matched_in_order(records, {
                {trace::event::thread_created,    a_id, 5},
                {trace::event::thread_ready,      a_id, 5},
                {trace::event::thread_running,    a_id, 5},
                {trace::event::thread_terminated, a_id, 5},
             }), 4u) << dump(records);

   for (auto const& r : records) {
      if (r.thread == a_id) {
         EXPECT_EQ(r.core, 0u) << dump(records);
      }
   }

   // Each stamp is taken inside its own append, so a buffer is in time order.
   ASSERT_GE(records.size(), 2u);
   for (std::size_t i = 1; i < records.size(); ++i) {
      EXPECT_LE(records[i - 1].timestamp, records[i].timestamp) << dump(records);
   }
   EXPECT_LT(records.front().timestamp, records.back().timestamp) << "the clock never moved";

   EXPECT_EQ(trace::dropped(0), 0u);
   EXPECT_EQ(trace::dropped(1), 0u);
}

TEST_F(Trace_Test, GivenAThreadThatParksOnAMutex_ThenTheBlockNamesTheMutex_AndTheHandoverIsRecorded)
{
   struct
   {
      sync::mutex     m;
      sync::semaphore gate{0};
   } t;

   /* H takes the mutex and parks on the gate. W, the only thread left that can
    * run, then has to park on the mutex. G, least urgent, runs last and
    * releases the gate, and H's unlock hands the mutex to W. */
   thread h(
      [&t]{
         t.m.lock();
         t.gate.acquire();
         t.m.unlock();
      },
      s_a, thread::priority(1), core0
   );

   thread w(
      [&t]{
         t.m.lock();
         t.m.unlock();
      },
      s_b, thread::priority(5), core0
   );

   thread g([&t]{ t.gate.release(); }, s_c, thread::priority(10), core0);

   auto const h_id = h.get_id();
   auto const w_id = w.get_id();
   auto const m    = id_of(t.m);

   kernel::start();

   auto const records = drain(0);
   EXPECT_EQ(matched_in_order(records, {
                {trace::event::mutex_acquired, h_id, 1, m},
                {trace::event::thread_blocked, h_id, 1, 0},  // on the gate, which is not a mutex
                {trace::event::thread_blocked, w_id, 5, m},  // on the mutex, named
                {trace::event::mutex_released, h_id, 1, m},
                {trace::event::mutex_acquired, w_id, 5, m},  // the handover, committed by H
                {trace::event::thread_ready,   w_id, 5},
                {trace::event::thread_running, w_id, 5},
                {trace::event::mutex_released, w_id, 5, m},
             }), 8u) << dump(records);
}

TEST_F(Trace_Test, GivenAMoreUrgentWaiter_ThenTheHolderIsRecordedRunningAtTheWaitersUrgency)
{
   struct
   {
      sync::mutex     m;
      sync::semaphore gate{0};
   } t;

   /* H parks on the gate first so that L can take the mutex. L's release of the
    * gate lets H preempt it and try the mutex, which boosts L to H's urgency.
    *
    * H need not PARK on the mutex. Its own reschedule request can land while it
    * is still on its way to parking, and then L at urgency 1 ties with H at base
    * 1, so the tie can go to L while H waits ready. Either way L runs boosted
    * until its unlock hands the mutex to H, and that is what is asserted. */
   thread h(
      [&t]{
         t.gate.acquire();
         t.m.lock();
         t.m.unlock();
      },
      s_a, thread::priority(1), core0
   );

   thread l(
      [&t]{
         t.m.lock();
         t.gate.release();
         t.m.unlock();
      },
      s_b, thread::priority(5), core0
   );

   auto const h_id = h.get_id();
   auto const l_id = l.get_id();
   auto const m    = id_of(t.m);

   kernel::start();

   auto const records = drain(0);
   EXPECT_EQ(matched_in_order(records, {
                {trace::event::mutex_acquired, l_id, 5, m},
                {trace::event::thread_running, l_id, 1},     // L at H's urgency: the boost
                {trace::event::mutex_released, l_id, 5, m},
                {trace::event::mutex_acquired, h_id, 1, m},  // the handover, committed by L
                {trace::event::thread_running, h_id, 1},
                {trace::event::mutex_released, h_id, 1, m},
                {trace::event::thread_running, l_id, 5},     // and back to its own priority
             }), 7u) << dump(records);
}

TEST_F(Trace_Test, GivenAWaiterOnAnotherCore_WhenHandedTheMutex_ThenTheWakeIsRecordedByTheWakingCoreFirst)
{
   struct
   {
      sync::mutex       m;
      std::atomic<bool> held{false};
      thread*           waiter{nullptr};
      bool              waiter_armed{false};
   } t;

   thread s(
      [&t]{
         t.m.lock();
         t.held.store(true, std::memory_order_release);

         /* Unlock only once the waiter is queued on the mutex, so that the
          * unlock is a handover to it rather than a free mutex it takes later.
          * blocked_on is set exactly while a thread is queued, and the chain
          * query reads it. Bounded by time, because the waiter's core is a host
          * thread that the host may deschedule. */
         auto const deadline = std::chrono::steady_clock::now() + 10s;
         while (std::chrono::steady_clock::now() < deadline) {
            if (diagnostics::blocking_chain_of(*t.waiter).length == 1) {
               t.waiter_armed = true;
               break;
            }
            this_core::cpu_relax();
         }
         t.m.unlock();
      },
      s_a, thread::priority(5), core0
   );

   thread w(
      [&t]{
         while (!t.held.load(std::memory_order_acquire)) { // set from core 0
            this_core::cpu_relax();
         }
         t.m.lock();
         t.m.unlock();
      },
      s_b, thread::priority(5), core1
   );
   t.waiter = &w;

   auto const s_id = s.get_id();
   auto const w_id = w.get_id();
   auto const m    = id_of(t.m);

   kernel::start();

   ASSERT_TRUE(t.waiter_armed) << "the waiter never queued on the mutex";

   auto const on0 = drain(0);
   auto const on1 = drain(1);

   EXPECT_EQ(matched_in_order(on0, {
                {trace::event::mutex_acquired, s_id, 5, m},
                {trace::event::mutex_released, s_id, 5, m},
                {trace::event::mutex_acquired, w_id, 5, m},  // committed on core 0 for core 1's thread
                {trace::event::remote_wake,    w_id, 5},
             }), 4u) << dump(on0);

   /* Not blocked first, necessarily: the handover can land while the waiter is
    * still on its way to parking, and the kernel then readies it in place. The
    * ready on core 1 is the wake arriving either way, because nothing else on
    * core 1 could ready it. */
   EXPECT_EQ(matched_in_order(on1, {
                {trace::event::thread_ready,   w_id, 5},
                {trace::event::thread_running, w_id, 5},
                {trace::event::mutex_released, w_id, 5, m},
             }), 3u) << dump(on1);

   auto const wake  = first(on0, {trace::event::remote_wake, w_id});
   auto const ready = first(on1, {trace::event::thread_ready, w_id});
   ASSERT_TRUE(wake && ready);
   EXPECT_EQ(wake->core, 1u);  // the THREAD's core, not the recording one

   // On Linux both cores stamp from one clock, so the order across cores shows.
   EXPECT_LE(wake->timestamp, ready->timestamp) << "on core 0:" << dump(on0) << "on core 1:" << dump(on1);
}

TEST_F(Trace_Test, GivenMoreRecordsThanFit_ThenTheNewestAreDroppedAndCounted)
{
   // Each yield records a ready and a running, so this is well over 256.
   thread a(
      []{
         for (int i = 0; i < 300; ++i) {
            this_thread::yield();
         }
      },
      s_a, thread::priority(5), core0
   );
   auto const a_id = a.get_id();

   kernel::start();

   std::vector<trace::record> records(1024);
   ASSERT_EQ(trace::read(0, records), 256u); // the capacity, and not one more
   EXPECT_GT(trace::dropped(0), 0u);

   // The OLDEST survive. The very first was the thread's creation, recorded
   // before any core ran.
   EXPECT_EQ(records[0].what, trace::event::thread_created);
   EXPECT_EQ(records[0].thread, a_id);

   EXPECT_EQ(trace::read(0, records), 0u); // read once, and nothing invented
}

TEST_F(Trace_Test, GivenAReaderOnAnotherCore_WhenItKeepsUp_ThenEveryRecordArrivesOnceAndInOrder)
{
   /* The writer records user values 0 to total-1 and never runs more than 64
    * ahead of the reader, so nothing may be dropped. The buffer wraps about 80
    * times, and the reader reads while the writer's core appends.
    *
    * consumed is RELAXED on purpose. It only paces the writer, so the ordering
    * that makes a slot safe to read, and safe to overwrite, has to come from
    * the buffer's own head and tail. Under ThreadSanitizer (coop) that is what
    * this test checks: with acquire and release here, the test would supply
    * the ordering itself and hide a buffer that lacked it. */
   static constexpr std::uint32_t total = 20'000;

   struct
   {
      std::atomic<std::uint32_t> consumed{0};
      std::atomic<bool>          writer_done{false};
      bool                       writer_starved{false};
      std::vector<std::uintptr_t> seen;
   } t;
   t.seen.reserve(total);

   thread writer(
      [&t]{
         for (std::uint32_t i = 0; i < total && !t.writer_starved; ++i) {
            // Bounded by time, because the reader's core is a host thread.
            auto const deadline = std::chrono::steady_clock::now() + 10s;
            while (i - t.consumed.load(std::memory_order_relaxed) >= 64) { // set from core 1
               if (std::chrono::steady_clock::now() > deadline) {
                  t.writer_starved = true;
                  break;
               }
               this_core::cpu_relax();
            }
            trace::user(9, i);
         }
         t.writer_done.store(true, std::memory_order_release);
      },
      s_a, thread::priority(5), core0
   );

   thread reader(
      [&t]{
         std::array<trace::record, 32> chunk{};
         while (true) {
            // Done is read BEFORE the last read, so a read that follows it
            // sees every record the writer made.
            bool const done = t.writer_done.load(std::memory_order_acquire);
            auto const n = trace::read(0, chunk);
            for (std::size_t i = 0; i < n; ++i) {
               if (chunk[i].what == trace::event::user) {
                  t.seen.push_back(chunk[i].object);
               }
            }
            t.consumed.store(static_cast<std::uint32_t>(t.seen.size()), std::memory_order_relaxed);
            if (done && n == 0) {
               break;
            }
         }
      },
      s_b, thread::priority(5), core1
   );

   kernel::start();

   EXPECT_FALSE(t.writer_starved);
   EXPECT_EQ(trace::dropped(0), 0u);
   ASSERT_EQ(t.seen.size(), total);
   for (std::uint32_t i = 0; i < total; ++i) {
      if (t.seen[i] != i) {
         ADD_FAILURE() << "user value " << i << " arrived as " << t.seen[i];
         break;
      }
   }
}

TEST_F(Trace_Test, GivenAUserEvent_ThenItCarriesTheTagTheValueAndTheCallingThread)
{
   thread a([]{ trace::user(7, 0xC0FFEE); }, s_a, thread::priority(5), core1);
   auto const a_id = a.get_id();

   kernel::start();

   auto const records = drain(1);
   auto const mark = first(records, {trace::event::user, a_id});
   ASSERT_TRUE(mark) << dump(records);
   EXPECT_EQ(mark->priority, 7u);
   EXPECT_EQ(mark->object, 0xC0FFEEu);
   EXPECT_EQ(mark->core, 1u);
}

TEST_F(Trace_Test, GivenAFinishedLifecycle_ThenItsTraceOutlivesFinaliseButNotTheNextInitialise)
{
   {
      thread a([]{}, s_a, thread::priority(5), core0);
      kernel::start();
   }
   kernel::finalise();

   std::array<trace::record, 1> one{};
   EXPECT_EQ(trace::read(0, one), 1u); // still readable after finalise

   kernel::initialise(); // the fixture's TearDown finalises this one
   EXPECT_EQ(trace::read(0, one), 0u); // and the rest discarded
   EXPECT_EQ(trace::dropped(0), 0u);
}
