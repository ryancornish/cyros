/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file stream.hpp
 * @brief The kernel trace as a byte stream, for a tool at the other end of a file or a wire.
 *
 * trace::read() hands records over in the target's own layout, and the layout
 * differs between targets: the object field is 4 bytes on a 32-bit core, and
 * SPARC is big endian. stream_writer drains the buffers and writes the records
 * in one layout that no target decides, so one decoder reads every target's
 * stream (debug/cyros_trace.py, which also turns a stream into a Perfetto
 * trace).
 *
 * The writer is the drain's encoder, not the drain. Which thread calls drain(),
 * and how often, is the application's choice: a host program uses an ordinary
 * OS thread, a board a thread of its own writing to a UART. Keeping up is the
 * caller's job. What the kernel could not keep is reported in the stream, never
 * hidden.
 *
 * A writer is the reader of every core's buffer, and a buffer has one reader at
 * a time, so nothing else may call trace::read() while a writer drains. A
 * writer is not itself thread safe. One stream covers one kernel lifecycle,
 * because the next kernel::initialise() discards the trace and numbers threads
 * from 1 again.
 *
 * ## The format, version 1
 *
 * A stream is a sequence of packets, each
 *
 *    kind     u8
 *    length   u16, of the payload
 *    payload  length bytes
 *
 * Every integer is little endian, whatever the target's own order. A reader
 * skips a kind it does not know, so a kind can be added without a new version.
 * The sink receives each packet whole, in one call, so a transport that frames
 * (COBS over a UART, say) frames one call.
 *
 *    1 header, always the first packet
 *         magic          the 4 bytes "CYTR"
 *         version        u16
 *         cores          u16, config::cores
 *         timestamp_hz   u64, record timestamps per second, or 0 for unknown
 *
 *    2 records, from one read() of one core
 *         core           u8, the core that RECORDED them, whose buffer they were in
 *         then 24 bytes per record, oldest first:
 *            timestamp   u64
 *            object      u64
 *            thread      u32
 *            event       u8, a trace::event
 *            core        u8, the thread's core
 *            priority    u8
 *            reserved    u8
 *
 *    3 dropped, after the records of a drain() in which the count grew
 *         core           u8
 *         dropped        u32, trace::dropped(core), the lifecycle's total so far
 *
 *    4 thread name       thread u32, then the name's bytes, unterminated
 *    5 object name       object u64, then the name's bytes
 *    6 user tag name     tag u8, then the name's bytes
 *
 * A name longer than max_name bytes is cut to max_name.
 *
 * Where in a core's records the dropped ones belonged is not known exactly. A
 * record is dropped only while that core's buffer is full, so the loss lies
 * after the records already written before the previous dropped packet for that
 * core, if any, and before the end of the records of the drain that reports it.
 */

#ifndef CYROS_TRACE_STREAM_HPP
#define CYROS_TRACE_STREAM_HPP

#include <cyros/config/config.hpp>
#include <cyros/kernel/assert.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/trace.hpp>

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <utility>

namespace cyros::trace
{

inline constexpr std::uint16_t stream_version = 1;

/// A record's size in the stream, which is not its size in memory
inline constexpr std::size_t stream_record_size = 24;

enum class packet : std::uint8_t
{
   header      = 1,
   records     = 2,
   dropped     = 3,
   thread_name = 4,
   object_name = 5,
   tag_name    = 6,
};

/**
 * @brief Encodes a lifecycle's trace into a stream of packets, one sink call per packet.
 *
 * @tparam Sink Called with each packet as a `std::span<std::byte const>`. It
 *              writes the bytes somewhere, and must not call back into the writer.
 */
template <typename Sink>
   requires std::invocable<Sink&, std::span<std::byte const>>
class stream_writer
{
public:
   /// The longest name kept, in bytes
   static constexpr std::size_t max_name = 64;
   /// Records moved per trace::read(), and so the most in one records packet.
   /// drain() holds them twice on the caller's stack, as read and as encoded,
   /// about 800 bytes
   static constexpr std::size_t records_per_packet = 16;

   /**
    * @brief Write the header, so that a stream always begins with one.
    *
    * @param timestamp_hz What one second is in record timestamps: 1'000'000'000
    *                     on the Linux ports, the core clock on a Cortex-M. The
    *                     writer cannot know it, so the caller says.
    */
   stream_writer(Sink destination, std::uint64_t timestamp_hz) noexcept
      : sink(std::move(destination))
   {
      packet_buffer<16> out(packet::header);
      out.bytes(std::as_bytes(std::span{"CYTR", 4}));
      out.u16(stream_version);
      out.u16(static_cast<std::uint16_t>(config::cores));
      out.u64(timestamp_hz);
      emit(out);
   }

   /// Give a thread a name for a tool to show. Names may come at any point in the stream
   void name_thread(cyros::thread::id thread, std::string_view name) noexcept
   {
      packet_buffer<4 + max_name> out(packet::thread_name);
      out.u32(thread);
      out.name(name);
      emit(out);
   }

   /// Name what a record's object field holds, such as a mutex's address
   void name_object(void const* object, std::string_view name) noexcept
   {
      packet_buffer<8 + max_name> out(packet::object_name);
      out.u64(reinterpret_cast<std::uintptr_t>(object));
      out.name(name);
      emit(out);
   }

   /// Name a tag passed to trace::user()
   void name_tag(std::uint8_t tag, std::string_view name) noexcept
   {
      packet_buffer<1 + max_name> out(packet::tag_name);
      out.u8(tag);
      out.name(name);
      emit(out);
   }

   /**
    * @brief Write what @p core has recorded and nobody has read, and any new drops.
    * @return How many records were written
    *
    * Stops once it has written a buffer's worth, so a core recording faster than
    * the sink can take it cannot hold the caller here forever. Whatever is left
    * waits for the next call.
    */
   std::size_t drain(std::uint32_t core) noexcept
   {
      CYROS_REQUIRE_OP(core, <, config::cores); // No such core in this configuration

      // Never zero, because with tracing off `written < 0` trips -Wtype-limits.
      // The one read then returns nothing
      constexpr std::size_t budget = std::max<std::size_t>(config::trace_records_per_core, 1);

      std::size_t written = 0;
      std::array<record, records_per_packet> chunk{};

      while (written < budget) {
         auto const count = trace::read(core, chunk);
         if (count == 0) break;

         packet_buffer<1 + records_per_packet * stream_record_size> out(packet::records);
         out.u8(static_cast<std::uint8_t>(core));
         for (std::size_t i = 0; i < count; ++i) {
            auto const& r = chunk[i];
            out.u64(r.timestamp);
            out.u64(r.object);
            out.u32(r.thread);
            out.u8(static_cast<std::uint8_t>(r.what));
            out.u8(r.core);
            out.u8(r.priority);
            out.u8(r.reserved);
         }
         emit(out);

         written += count;
         if (count < chunk.size()) break; // Caught up
      }

      if (auto const dropped = trace::dropped(core); dropped != reported_dropped[core]) {
         packet_buffer<5> out(packet::dropped);
         out.u8(static_cast<std::uint8_t>(core));
         out.u32(dropped);
         emit(out);
         reported_dropped[core] = dropped;
      }
      return written;
   }

   /// drain() every core, in core order
   std::size_t drain() noexcept
   {
      std::size_t written = 0;
      for (std::uint32_t core = 0; core < config::cores; ++core) {
         written += drain(core);
      }
      return written;
   }

private:
   static_assert(config::cores <= 255, "A packet names its core in one byte");
   static_assert(records_per_packet * stream_record_size + 1 <= 0xFFFF, "A payload's length is a u16");

   /* One packet, built in place: the 3-byte head, then the payload. The length
    * is filled in when the packet is handed to the sink. */
   template <std::size_t MaxPayload>
   struct packet_buffer
   {
      std::array<std::byte, 3 + MaxPayload> storage{};
      std::size_t                           size = 3;

      explicit packet_buffer(packet kind) noexcept
      {
         storage[0] = static_cast<std::byte>(kind);
      }

      void u8(std::uint8_t value) noexcept
      {
         storage[size++] = static_cast<std::byte>(value);
      }

      void u16(std::uint16_t value) noexcept { little_endian(value, 2); }
      void u32(std::uint32_t value) noexcept { little_endian(value, 4); }
      void u64(std::uint64_t value) noexcept { little_endian(value, 8); }

      void bytes(std::span<std::byte const> value) noexcept
      {
         std::ranges::copy(value, storage.begin() + static_cast<std::ptrdiff_t>(size));
         size += value.size();
      }

      void name(std::string_view value) noexcept
      {
         bytes(std::as_bytes(std::span{value.data(), std::min(value.size(), max_name)}));
      }

      std::span<std::byte const> finish() noexcept
      {
         auto const length = size - 3;
         storage[1] = static_cast<std::byte>(length & 0xFF);
         storage[2] = static_cast<std::byte>(length >> 8);
         return std::span{storage}.first(size);
      }

   private:
      void little_endian(std::uint64_t value, std::size_t width) noexcept
      {
         for (std::size_t i = 0; i < width; ++i) {
            storage[size++] = static_cast<std::byte>((value >> (8 * i)) & 0xFF);
         }
      }
   };

   template <std::size_t MaxPayload>
   void emit(packet_buffer<MaxPayload>& out) noexcept
   {
      sink(out.finish());
   }

   Sink                                      sink;
   std::array<std::uint32_t, config::cores> reported_dropped{};
};

} // namespace cyros::trace

#endif // CYROS_TRACE_STREAM_HPP
