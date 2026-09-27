/**
 * @file trace.cpp
 * @brief Kernel trace recorder: one buffer per core, appended to by that core alone.
 *
 * Each core's buffer has exactly one producer, the core itself, and one
 * consumer, whoever calls read(). The producer appends with interrupts masked,
 * so an ISR's record cannot land in the middle of the record it interrupted,
 * and the stamp is taken inside the same masked step, so a buffer is always in
 * timestamp order.
 *
 * The two sides meet only at two free-running counters: head, the records ever
 * appended, and tail, the records ever consumed. The producer writes a slot and
 * then releases head. The consumer copies slots and then releases tail. Each
 * acquires the other's counter before touching a slot, so the consumer never
 * reads a half-written record and the producer never overwrites one still being
 * copied. Neither side waits for the other: a full buffer drops the new record,
 * and an empty one reads as nothing.
 */

#include <cyros/kernel/assert.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/kernel/trace.hpp>

#include "scheduler.hpp"
#include "trace_points.hpp"

#include <algorithm>
#include <array>
#include <atomic>

namespace cyros::trace
{

static_assert(sizeof(thread::id) == sizeof(record::thread));

namespace
{

struct alignas(CYROS_PORT_CACHE_LINE) core_log
{
   std::atomic<std::uint32_t> head{0};
   std::atomic<std::uint32_t> tail{0};
   /* Written by the producer only. Atomic so a reader on another core can ask. */
   std::atomic<std::uint32_t> dropped{0};
   std::array<record, config::trace_records_per_core> slots{};
};

/* No buffers at all when tracing is off, so the build carries no trace memory. */
constinit std::array<core_log, enabled ? config::cores : 0> logs{};

void append_on_this_core(event what, thread::id thread, std::uint8_t core,
                         std::uint8_t priority, std::uintptr_t object) noexcept
{
   this_core::critical_guard guard;

   auto const this_core = cyros_port_get_core_id();
   CYROS_ASSERT_OP(this_core, <, logs.size());
   auto& log = logs[this_core];

   auto const head = log.head.load(std::memory_order_relaxed);
   auto const tail = log.tail.load(std::memory_order_acquire);
   if (head - tail == config::trace_records_per_core) {
      log.dropped.store(log.dropped.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
      return;
   }

   log.slots[head & (config::trace_records_per_core - 1)] = record{
      .timestamp = cyros_port_timestamp(),
      .object    = object,
      .thread    = thread,
      .what      = what,
      .core      = core,
      .priority  = priority,
      .reserved  = 0,
   };
   log.head.store(head + 1, std::memory_order_release);
}

} // namespace

void append(event what, thread_control_block const& subject, std::uintptr_t object) noexcept
{
   if constexpr (enabled) {
      // Urgency for a switch, because that is the priority the thread actually
      // runs at, and a boost is otherwise invisible: nothing stores it. Asked
      // only for thread_running, which runs where pick_next itself asks.
      auto const priority = (what == event::thread_running) ? urgency(subject) : subject.base_priority;

      // Blocked on a mutex names the mutex. The thread published it before
      // parking and clears it only after it runs again, so here it is current.
      if (what == event::thread_blocked) {
         object = reinterpret_cast<std::uintptr_t>(subject.blocked_on.load(std::memory_order_relaxed));
      }

      append_on_this_core(what, subject.id, static_cast<std::uint8_t>(subject.pinned_core), priority, object);
   }
}

void reset() noexcept
{
   // Only ever between lifecycles, when no core is running to append
   for (auto& log : logs) {
      log.head.store(0, std::memory_order_relaxed);
      log.tail.store(0, std::memory_order_relaxed);
      log.dropped.store(0, std::memory_order_relaxed);
   }
}

std::size_t read(std::uint32_t core, std::span<record> const& out) noexcept
{
   if constexpr (!enabled) {
      return 0;
   } else {
      CYROS_REQUIRE_OP(core, <, logs.size()); // No such core in this configuration
      auto& log = logs[core];

      auto const tail = log.tail.load(std::memory_order_relaxed); // Ours, as the one reader
      auto const head = log.head.load(std::memory_order_acquire);
      auto const count = std::min<std::size_t>(head - tail, out.size());

      for (std::size_t i = 0; i < count; ++i) {
         out[i] = log.slots[(tail + i) & (config::trace_records_per_core - 1)];
      }
      log.tail.store(tail + static_cast<std::uint32_t>(count), std::memory_order_release);
      return count;
   }
}

std::uint32_t dropped(std::uint32_t core) noexcept
{
   if constexpr (!enabled) {
      return 0;
   } else {
      CYROS_REQUIRE_OP(core, <, logs.size()); // No such core in this configuration
      return logs[core].dropped.load(std::memory_order_relaxed);
   }
}

void user([[maybe_unused]] std::uint8_t tag, [[maybe_unused]] std::uintptr_t value) noexcept
{
   if constexpr (enabled) {
      auto const& self = scheduler_for_this_core().get_current_thread();
      append_on_this_core(event::user, self.id, static_cast<std::uint8_t>(self.pinned_core), tag, value);
   }
}

} // namespace cyros::trace
