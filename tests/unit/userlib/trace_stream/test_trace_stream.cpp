/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_trace_stream.cpp
 * @brief The trace stream: what stream_writer writes is the format stream.hpp
 *        describes, every field of a record survives it, and what the kernel
 *        dropped is reported rather than hidden.
 *
 * The stream is decoded here by a reader written from the format's description
 * in stream.hpp, not from the writer's code, so a writer and a decoder cannot
 * agree on a mistake. Values whose every byte differs are chosen wherever a
 * field is checked, so a swapped or truncated byte shows.
 *
 * The subject is the encoding. What the kernel records, and in what order, is
 * test_trace's, and is trusted here.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/trace.hpp>
#include <cyros/sync/mutex.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/trace/stream.hpp>

#include <common/trace_file.hpp>

#include <gtest/gtest.h>

#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <map>
#include <span>
#include <string>
#include <utility>
#include <vector>

using namespace cyros;

namespace
{

alignas(16) std::array<std::byte, 64 * 1024> s_a;
alignas(16) std::array<std::byte, 64 * 1024> s_b;

class Trace_Stream_Test : public ::testing::Test
{
protected:
   void SetUp() override    { kernel::initialise(); }
   void TearDown() override { kernel::finalise(); }
};

/* Keeps every byte, and the size of each sink call, so a test can check that
 * each call was one whole packet. */
struct memory_sink
{
   std::vector<std::byte>*   bytes;
   std::vector<std::size_t>* calls;

   void operator()(std::span<std::byte const> packet) const
   {
      bytes->insert(bytes->end(), packet.begin(), packet.end());
      calls->push_back(packet.size());
   }
};

std::uint64_t little_endian(std::span<std::byte const> bytes, std::size_t at, std::size_t width)
{
   std::uint64_t value = 0;
   for (std::size_t i = 0; i < width; ++i) {
      value |= std::uint64_t{std::to_integer<std::uint8_t>(bytes[at + i])} << (8 * i);
   }
   return value;
}

std::string text(std::span<std::byte const> bytes)
{
   return {reinterpret_cast<char const*>(bytes.data()), bytes.size()};
}

/* A records packet's records, with the core whose buffer they came from. */
struct recorded
{
   std::uint8_t  on;
   trace::record r;
};

struct contents
{
   bool                                      whole = true; // Every packet fit, and held whole records
   std::vector<trace::packet>                kinds;
   std::vector<std::size_t>                  packet_sizes;
   std::string                               magic;
   std::uint64_t                             version      = 0;
   std::uint64_t                             cores        = 0;
   std::uint64_t                             timestamp_hz = 0;
   std::vector<recorded>                     records;
   std::vector<std::size_t>                  records_per_packet;
   std::vector<std::pair<std::uint8_t, std::uint32_t>> dropped;
   std::map<std::uint64_t, std::string>      thread_names;
   std::map<std::uint64_t, std::string>      object_names;
   std::map<std::uint64_t, std::string>      tag_names;
};

/* The format as stream.hpp states it: kind u8, length u16, payload. */
contents decode(std::span<std::byte const> stream)
{
   contents out;
   std::size_t at = 0;
   while (at < stream.size()) {
      if (stream.size() - at < 3) {
         out.whole = false;
         break;
      }
      auto const kind   = static_cast<trace::packet>(stream[at]);
      auto const length = little_endian(stream, at + 1, 2);
      if (stream.size() - at - 3 < length) {
         out.whole = false;
         break;
      }
      auto const p = stream.subspan(at + 3, length);
      at += 3 + length;

      out.kinds.push_back(kind);
      out.packet_sizes.push_back(3 + length);

      switch (kind) {
         case trace::packet::header:
            out.magic        = text(p.first(4));
            out.version      = little_endian(p, 4, 2);
            out.cores        = little_endian(p, 6, 2);
            out.timestamp_hz = little_endian(p, 8, 8);
            out.whole        = out.whole && length == 16;
            break;

         case trace::packet::records: {
            auto const on = std::to_integer<std::uint8_t>(p[0]);
            if ((length - 1) % 24 != 0) {
               out.whole = false;
            }
            auto const count = (length - 1) / 24;
            for (std::size_t i = 0; i < count; ++i) {
               auto const o = 1 + i * 24;
               out.records.push_back({on, trace::record{
                  .timestamp = little_endian(p, o, 8),
                  .object    = static_cast<std::uintptr_t>(little_endian(p, o + 8, 8)),
                  .thread    = static_cast<cyros::thread::id>(little_endian(p, o + 16, 4)),
                  .what      = static_cast<trace::event>(p[o + 20]),
                  .core      = std::to_integer<std::uint8_t>(p[o + 21]),
                  .priority  = std::to_integer<std::uint8_t>(p[o + 22]),
                  .reserved  = std::to_integer<std::uint8_t>(p[o + 23]),
               }});
            }
            out.records_per_packet.push_back(count);
            break;
         }

         case trace::packet::dropped:
            out.dropped.emplace_back(std::to_integer<std::uint8_t>(p[0]),
                                     static_cast<std::uint32_t>(little_endian(p, 1, 4)));
            out.whole = out.whole && length == 5;
            break;

         case trace::packet::thread_name:
            out.thread_names[little_endian(p, 0, 4)] = text(p.subspan(4));
            break;

         case trace::packet::object_name:
            out.object_names[little_endian(p, 0, 8)] = text(p.subspan(8));
            break;

         case trace::packet::tag_name:
            out.tag_names[little_endian(p, 0, 1)] = text(p.subspan(1));
            break;

         default:
            break; // A reader skips what it does not know
      }
   }
   return out;
}

/* The user records in @p stream, in stream order. */
std::vector<recorded> user_records(contents const& stream)
{
   std::vector<recorded> found;
   for (auto const& rec : stream.records) {
      if (rec.r.what == trace::event::user) {
         found.push_back(rec);
      }
   }
   return found;
}

/* Each core's records, in the order they were written, must be in the order
 * they were stamped. A writer that reordered packets would break it. */
void expect_each_core_in_time_order(contents const& stream)
{
   std::map<std::uint8_t, std::uint64_t> last;
   for (auto const& rec : stream.records) {
      auto const [it, first] = last.try_emplace(rec.on, rec.r.timestamp);
      if (!first) {
         EXPECT_LE(it->second, rec.r.timestamp) << "core " << int{rec.on};
         it->second = rec.r.timestamp;
      }
   }
}

/* A path in the working directory no other run of this binary shares. */
std::string scratch_path(char const* what)
{
   return std::string("test_trace_stream.") + what + "." + std::to_string(::getpid()) + ".cytrace";
}

std::vector<std::byte> read_and_remove(std::string const& path)
{
   std::ifstream in(path, std::ios::binary);
   std::vector<char> raw((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
   std::remove(path.c_str());
   auto const bytes = std::as_bytes(std::span{raw});
   return {bytes.begin(), bytes.end()};
}

} // namespace

TEST_F(Trace_Stream_Test, GivenANewWriter_ThenTheStreamOpensWithAHeaderDescribingThisConfiguration)
{
   std::vector<std::byte>   bytes;
   std::vector<std::size_t> calls;

   trace::stream_writer writer(memory_sink{&bytes, &calls}, 0x0102030405060708);

   auto const stream = decode(bytes);
   ASSERT_TRUE(stream.whole);
   ASSERT_EQ(stream.kinds.size(), 1u);
   EXPECT_EQ(stream.kinds[0], trace::packet::header);
   EXPECT_EQ(stream.magic, "CYTR");
   EXPECT_EQ(stream.version, trace::stream_version);
   EXPECT_EQ(stream.cores, config::cores);
   EXPECT_EQ(stream.timestamp_hz, 0x0102030405060708u);
}

TEST_F(Trace_Stream_Test, GivenUserRecordsOnTwoCores_ThenEveryFieldSurvivesTheStream_UnderItsRecordingCore)
{
   static constexpr std::uintptr_t value0 = 0x0123456789ABCDEF;
   static constexpr std::uintptr_t value1 = 0xFEDCBA9876543210;

   thread a([]{ trace::user(0xA5, value0); }, s_a, thread::priority(7), core0);
   thread b([]{ trace::user(0x5A, value1); }, s_b, thread::priority(9), core1);
   auto const a_id = a.get_id();
   auto const b_id = b.get_id();

   kernel::start();

   std::vector<std::byte>   bytes;
   std::vector<std::size_t> calls;
   trace::stream_writer writer(memory_sink{&bytes, &calls}, 1);
   writer.drain();

   auto const stream = decode(bytes);
   ASSERT_TRUE(stream.whole);
   EXPECT_EQ(calls, stream.packet_sizes) << "a sink call that is not one whole packet";

   auto const users = user_records(stream);
   ASSERT_EQ(users.size(), 2u);

   auto const& from_a = users[0].r.thread == a_id ? users[0] : users[1];
   auto const& from_b = users[0].r.thread == a_id ? users[1] : users[0];

   EXPECT_EQ(from_a.on, 0u);
   EXPECT_EQ(from_a.r.thread, a_id);
   EXPECT_EQ(from_a.r.core, 0u);
   EXPECT_EQ(from_a.r.priority, 0xA5u); // A user record's priority is its tag
   EXPECT_EQ(from_a.r.object, value0);
   EXPECT_EQ(from_a.r.reserved, 0u);

   EXPECT_EQ(from_b.on, 1u);
   EXPECT_EQ(from_b.r.thread, b_id);
   EXPECT_EQ(from_b.r.core, 1u);
   EXPECT_EQ(from_b.r.priority, 0x5Au);
   EXPECT_EQ(from_b.r.object, value1);

   // The kernel's own records come through too, with their own event values
   std::size_t created = 0;
   for (auto const& rec : stream.records) {
      if (rec.r.what == trace::event::thread_created && (rec.r.thread == a_id || rec.r.thread == b_id)) {
         ++created;
         EXPECT_EQ(rec.r.priority, rec.r.thread == a_id ? 7u : 9u);
      }
   }
   EXPECT_EQ(created, 2u);

   // Stamps are the port's, nanoseconds here: a stream that kept only the low
   // bytes would have stamps that wrap within seconds of boot
   EXPECT_GT(from_a.r.timestamp, 0xFFFFFFFFu);
   expect_each_core_in_time_order(stream);

   EXPECT_TRUE(stream.dropped.empty());
}

TEST_F(Trace_Stream_Test, GivenAFullBuffer_ThenTheStreamKeepsWhatTheBufferKept_InOrder_AndReportsTheDropsOnce)
{
   static constexpr std::uintptr_t produced = config::trace_records_per_core + 50;

   thread a(
      []{
         for (std::uintptr_t i = 0; i < produced; ++i) {
            trace::user(1, i);
         }
      },
      s_a, thread::priority(5), core0
   );

   kernel::start();

   std::vector<std::byte>   bytes;
   std::vector<std::size_t> calls;
   trace::stream_writer writer(memory_sink{&bytes, &calls}, 1);
   writer.drain(0);

   auto const stream = decode(bytes);
   ASSERT_TRUE(stream.whole);

   std::size_t on0 = 0;
   for (auto const& rec : stream.records) {
      on0 += rec.on == 0 ? 1 : 0;
   }
   EXPECT_EQ(on0, config::trace_records_per_core) << "the buffer was full, so all of it was there to write";

   for (auto const n : stream.records_per_packet) {
      EXPECT_LE(n, decltype(writer)::records_per_packet);
   }

   // The buffer keeps its oldest, so the user values kept are 0, 1, 2 ... with
   // no gap: a writer that lost or reordered a packet would leave one
   auto const users = user_records(stream);
   ASSERT_FALSE(users.empty());
   for (std::size_t i = 0; i < users.size(); ++i) {
      EXPECT_EQ(users[i].r.object, i);
   }
   EXPECT_LT(users.size(), produced);

   ASSERT_EQ(stream.dropped.size(), 1u);
   EXPECT_EQ(stream.dropped[0].first, 0u);
   EXPECT_EQ(stream.dropped[0].second, trace::dropped(0));
   EXPECT_GT(stream.dropped[0].second, 0u);
   EXPECT_EQ(stream.kinds.back(), trace::packet::dropped) << "a drop is reported after the drain's records";

   // Nothing new since: a second drain writes nothing at all, the drops included
   auto const before = bytes.size();
   EXPECT_EQ(writer.drain(0), 0u);
   EXPECT_EQ(bytes.size(), before);
}

TEST_F(Trace_Stream_Test, GivenACoreRecordingFasterThanTheSinkTakes_ThenDrainStillReturns_AfterOneBuffersWorth)
{
   /* The sink records twice a packet's worth on the core being drained each
    * time it is handed a packet, so every read finds a full packet waiting and
    * the core is never caught up. A drain with no bound would never return, so
    * the sink gives up by name long after a bounded one would have. The first
    * read must find a full packet too, hence the records made beforehand. */
   struct
   {
      std::size_t written = 0;
   } t;

   thread a(
      [&t]{
         using writer_t = trace::stream_writer<void (*)(std::span<std::byte const>)>;

         static std::size_t calls = 0;
         writer_t writer(
            [](std::span<std::byte const>) {
               if (++calls > 10 * config::trace_records_per_core) {
                  std::fprintf(stderr, "drain() never returned\n");
                  std::abort();
               }
               for (std::size_t i = 0; i < 2 * writer_t::records_per_packet; ++i) {
                  trace::user(2, i);
               }
            },
            1
         );
         for (std::size_t i = 0; i < writer_t::records_per_packet; ++i) {
            trace::user(2, i);
         }
         t.written = writer.drain(0);
      },
      s_a, thread::priority(5), core0
   );

   kernel::start();

   EXPECT_EQ(t.written, config::trace_records_per_core);
}

TEST_F(Trace_Stream_Test, GivenNames_ThenEachIsWrittenAsGiven_AndALongOneIsCut)
{
   sync::mutex m;
   std::vector<std::byte>   bytes;
   std::vector<std::size_t> calls;
   trace::stream_writer writer(memory_sink{&bytes, &calls}, 1);

   using writer_t = decltype(writer);
   std::string const long_name(writer_t::max_name + 10, 'x');

   writer.name_thread(0x01020304, "producer");
   writer.name_thread(5, long_name);
   writer.name_object(&m, "bus");
   writer.name_tag(0xA5, "frame");

   auto const stream = decode(bytes);
   ASSERT_TRUE(stream.whole);
   EXPECT_EQ(calls, stream.packet_sizes);
   EXPECT_EQ(stream.thread_names.at(0x01020304), "producer");
   EXPECT_EQ(stream.thread_names.at(5), long_name.substr(0, writer_t::max_name));
   EXPECT_EQ(stream.object_names.at(reinterpret_cast<std::uintptr_t>(&m)), "bus");
   EXPECT_EQ(stream.tag_names.at(0xA5), "frame");
}

TEST_F(Trace_Stream_Test, GivenATraceFile_WhileTwoCoresRun_ThenTheFileHoldsTheWholeStream)
{
   /* B records more than a buffer holds, paced, after the mutex. Only a drain
    * running WHILE the cores run can keep all of it, so a helper that drained
    * only at the end shows as drops. The pace leaves the drain about 50 ms of
    * slack, against its 100 us period. */
   static constexpr std::uintptr_t paced = config::trace_records_per_core + 44;
   static constexpr auto           pace  = std::chrono::microseconds(200);

   struct
   {
      sync::mutex       m;
      sync::semaphore   done{0};
      std::atomic<bool> released{false};
   } t;

   thread a(
      [&t]{
         t.m.lock();
         trace::user(1, 10);
         t.m.unlock();
         t.released.store(true, std::memory_order_release);
         t.done.release();
      },
      s_a, thread::priority(5), core0
   );

   thread b(
      [&t]{
         while (!t.released.load(std::memory_order_acquire)) { // Set from core 0
            this_core::cpu_relax();
         }
         t.m.lock();
         trace::user(2, 20);
         t.m.unlock();
         t.done.acquire();

         for (std::uintptr_t i = 0; i < paced; ++i) {
            trace::user(3, i);
            auto const until = std::chrono::steady_clock::now() + pace;
            while (std::chrono::steady_clock::now() < until) {
               this_core::cpu_relax();
            }
         }
      },
      s_b, thread::priority(5), core1
   );
   auto const a_id = a.get_id();
   auto const b_id = b.get_id();

   auto const path = scratch_path("concurrent");
   {
      test::trace_file file(path.c_str(), std::chrono::microseconds(100));
      file.name_thread(a_id, "a");
      file.name_thread(b_id, "b");
      file.name_object(&t.m, "m");
      kernel::start();
   }

   auto const stream = decode(read_and_remove(path));
   ASSERT_TRUE(stream.whole);
   ASSERT_FALSE(stream.kinds.empty());
   EXPECT_EQ(stream.kinds[0], trace::packet::header);
   EXPECT_EQ(stream.timestamp_hz, 1'000'000'000u);
   EXPECT_EQ(stream.thread_names.at(a_id), "a");
   EXPECT_EQ(stream.thread_names.at(b_id), "b");
   EXPECT_EQ(stream.object_names.at(reinterpret_cast<std::uintptr_t>(&t.m)), "m");

   // Both threads' whole lives are in the file, recorded on their own cores
   auto const m = reinterpret_cast<std::uintptr_t>(&t.m);
   std::map<std::pair<cyros::thread::id, trace::event>, std::size_t> seen;
   std::uintptr_t next_paced = 0;
   for (auto const& rec : stream.records) {
      if (rec.r.thread != a_id && rec.r.thread != b_id) continue;
      if (rec.r.what == trace::event::user && rec.r.priority == 3) {
         EXPECT_EQ(rec.r.object, next_paced++);
         continue;
      }
      ++seen[{rec.r.thread, rec.r.what}];
      if (rec.r.what == trace::event::thread_terminated || rec.r.what == trace::event::user) {
         EXPECT_EQ(rec.on, rec.r.core);
      }
      if (rec.r.what == trace::event::mutex_acquired || rec.r.what == trace::event::mutex_released) {
         EXPECT_EQ(rec.r.object, m);
      }
   }
   for (auto const id : {a_id, b_id}) {
      EXPECT_EQ((seen[{id, trace::event::thread_created}]), 1u);
      EXPECT_EQ((seen[{id, trace::event::mutex_acquired}]), 1u);
      EXPECT_EQ((seen[{id, trace::event::mutex_released}]), 1u);
      EXPECT_EQ((seen[{id, trace::event::user}]), 1u);
      EXPECT_EQ((seen[{id, trace::event::thread_terminated}]), 1u);
   }

   EXPECT_EQ(next_paced, paced);

   expect_each_core_in_time_order(stream);
   EXPECT_TRUE(stream.dropped.empty()) << "the drain did not run while the cores did";
}

TEST_F(Trace_Stream_Test, GivenATraceFileThatNeverDrainsDuringTheRun_ThenItsLastDrainWritesEverything)
{
   thread a([]{ trace::user(4, 40); }, s_a, thread::priority(5), core0);
   auto const a_id = a.get_id();

   auto const path = scratch_path("final");
   {
      // A period no run reaches: the drain thread waits before its first pass
      // and the stop ends the wait, so the destructor's drain is the only one
      test::trace_file file(path.c_str(), std::chrono::hours(1));
      kernel::start();
   }

   auto const stream = decode(read_and_remove(path));
   ASSERT_TRUE(stream.whole);

   bool user = false;
   bool terminated = false;
   for (auto const& rec : stream.records) {
      if (rec.r.thread != a_id) continue;
      user       = user || (rec.r.what == trace::event::user && rec.r.object == 40);
      terminated = terminated || rec.r.what == trace::event::thread_terminated;
   }
   EXPECT_TRUE(user);
   EXPECT_TRUE(terminated);
}
