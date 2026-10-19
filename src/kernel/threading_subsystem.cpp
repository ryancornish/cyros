/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

#include "threading_subsystem.hpp"

#include <cyros/kernel/assert.hpp>

#include "align.hpp"
#include "scheduler.hpp"

namespace cyros
{

static_assert(sizeof(thread_control_block) <= CYROS_PORT_CONTEXT_SIZE + thread::tcb_size,
              "thread_control_block outgrew thread::tcb_size, so thread::min_stack_size "
              "now under-reports. Bump tcb_size.");

thread::thread(entry_fn&& entry, std::span<std::byte> stack, priority priority, core_affinity affinity)
{
   stack_layout slayout(stack, 0);
   tcb = ::new (slayout.tcb) thread_control_block(
      priority,
      affinity,
      slayout.user_stack,
      std::move(entry)
   );
   thread_registry::register_thread(*tcb);
}


thread::~thread()
{
   if (tcb == nullptr) return; // thread handle has been moved from, or is otherwise empty

   CYROS_REQUIRE(tcb->state == thread_state::terminated); // Destroyed before its thread finished
}

thread::thread(thread&& other) noexcept : tcb(other.tcb)
{
   other.tcb = nullptr;   // moving an empty handle is legal, and a no-op
}

thread& thread::operator=(thread&& other) noexcept
{
   // Self-move is a no-op. Without this guard the two lines below emptied the
   // handle (tcb = other.tcb, then other.tcb = nullptr, with other == this),
   // losing the thread and skipping the destructor's must-be-terminated check.
   if (this == &other) return *this;

   // Overwriting a handle is destroying it, so it carries the destructor's
   // contract: the thread it owned must already have terminated. This used to
   // abandon a live thread silently.
   if (tcb != nullptr) {
      CYROS_REQUIRE(tcb->state == thread_state::terminated); // Assigned over a live thread
   }

   tcb = other.tcb;
   other.tcb = nullptr;
   return *this;
}

[[nodiscard]] thread::id thread::get_id() const noexcept
{
   CYROS_REQUIRE(tcb != nullptr); // An empty handle, moved from or never assigned

   return tcb->id;
}

[[nodiscard]] thread::priority thread::get_priority() const noexcept
{
   CYROS_REQUIRE(tcb != nullptr); // An empty handle, moved from or never assigned

   return urgency(*tcb);
}

void thread::join() noexcept
{
   CYROS_REQUIRE(tcb != nullptr); // An empty handle, moved from or never assigned

   this_thread::wait_on(tcb->termination);
}


thread_control_block::thread_control_block(thread::priority priority,
                                           core_affinity affinity,
                                           std::span<std::byte> stack,
                                           thread::entry_fn&& entry)
   : base_priority(priority),
     affinity(affinity),
     stack(stack),
     entry(std::move(entry))
{
   cyros_port_context_init(context(), stack.data(), stack.size(), thread_launcher, this);
}


stack_layout::stack_layout(std::span<std::byte> const buffer, std::size_t const tls_bytes)
{
   auto const base = reinterpret_cast<std::uintptr_t>(buffer.data());
   auto const end  = base + buffer.size();

   // TCB at very top, aligned down
   auto const tcb_start = align_down(end - sizeof(thread_control_block), alignof(thread_control_block));
   tcb = reinterpret_cast<thread_control_block*>(tcb_start);

   // TLS just below TCB
   auto const tls_size = align_up(tls_bytes, alignof(std::max_align_t));
   auto const tls_top  = tcb_start;
   auto const tls_base = align_down(tls_top - tls_size, alignof(std::max_align_t));

   CYROS_REQUIRE_OP(tls_base, >=, base); // Buffer too small for TLS+TCB

   auto const tls_offset = static_cast<std::size_t>(tls_base - base);
   auto const tls_length = static_cast<std::size_t>(tls_top  - tls_base);
   tls_region = buffer.subspan(tls_offset, tls_length); // zero-length span if tls_bytes == 0

   // User stack: everything below TLS
   auto const stack_len = static_cast<std::size_t>(tls_base - base);
   CYROS_REQUIRE_OP(stack_len, >, 64); // Buffer too small after carving TCB/TLS

   user_stack = buffer.subspan(0, stack_len);
}


bool thread_ready_queue::remove(thread_control_block& tcb) noexcept
{
   thread_control_block* prev = nullptr;
   for (auto* cur = head; cur != nullptr; prev = cur, cur = cur->next) {
      if (cur != &tcb) continue;
      if (prev) prev->next = cur->next; else head = cur->next;
      if (tail == cur) tail = prev;
      tcb.next = &tcb; // restore the not-enqueued sentinel
      return true;
   }
   return false;
}


void thread_ready_matrix::remove_thread(thread_control_block& tcb) noexcept
{
   auto const priority = tcb.base_priority;
   CYROS_ASSERT_OP(priority, <, config::max_priorities);

   bool const removed = matrix[priority].remove(tcb);
   CYROS_ASSERT(removed); // Caller vouched the thread was enqueued

   if (matrix[priority].empty()) {
      bitmap &= ~(1u << priority);
   }
}


namespace this_thread
{


[[nodiscard]] thread::id id()
{
   return scheduler_for_this_core().current_thread_id();
}

[[nodiscard]] thread::priority priority()
{
   return scheduler_for_this_core().current_thread_urgency();
}

[[noreturn]] void thread_exit()
{
   auto& tcb = scheduler_for_this_core().get_current_thread();

   // Flag the thread's intent to terminate, the arbiter will handle the state
   // transition and teardown on next reschedule (requested immediately).
   // Any preemption landing before this point is okay as eventually the scheduler
   //  will pick this thread again and continue teardown.
   tcb.disposition = thread_disposition::terminating;
   // Any preemption landing after this point will cause the arbiter to retire
   // this thread and never return. Most of the time, this window is not preempted
   // and so we manually trigger a reschedule which is guaranteed to not pick this thread
   // again.
   cyros_port_thread_yield();

   CYROS_PORT_UNREACHABLE(); // Bug: The arbiter declined to retire this thread!
}

CYROS_HOT void yield()
{
   // Strong request: an explicit yield deliberately gives up the CPU and
   // relies on the reschedule round-trip having completed on return.
   cyros_port_thread_yield();
}

} // namespace this_thread

} // namespace cyros
