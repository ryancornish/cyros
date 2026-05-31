#include "scheduler.hpp"

namespace cyros
{

void scheduler::pin_thread(thread_control_block& tcb)
{
   tcb.pinned_core = core_id;
   pinned_thread_counter.fetch_add(1, std::memory_order_relaxed);
}

void scheduler::init_idle_thread()
{
   stack_layout slayout(idle_stack, 0);
   idle_thread = ::new (slayout.tcb) thread_control_block(
      idle_thread_id,
      config::max_priorities-1,
      core_affinity::from_id(core_id),
      slayout.user_stack,
      idle_task,
      nullptr
   );
   idle_thread->pinned_core = core_id;
}

// Core-local operations (only called on owning core)
void scheduler::start() noexcept
{
   CYROS_ASSERT(idle_thread != nullptr); // init_idle_thread() must run before start()

   auto* first = ready_matrix.pop_best_thread();
   if (first == nullptr) {
      first = idle_thread;
   }
   CYROS_ASSERT(first != nullptr);
   CYROS_ASSERT_OP(first->state, ==, thread_control_block::thread_state::ready);
   first->state = thread_control_block::thread_state::running;
   current_thread = first;

   //cyros_port_set_thread_pointer(current_thread);
   cyros_port_start_first(current_thread->context());
}

void scheduler::set_thread_ready(thread_control_block& tcb) noexcept
{
   CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);

   tcb.state = thread_control_block::thread_state::ready;

   // Idle thread does not belong in the ready_matrix,
   // but DOES follow state transition semantics
   if (&tcb == idle_thread) return;

   ready_matrix.enqueue_thread(tcb);
}

void scheduler::drain_inbox() noexcept
{
   inbox_poke_pending.store(false, std::memory_order_release);

   cross_core_request request;
   while (inbox.pop(request)) {
      switch (request.type) {
         case cross_core_request::set_thread_ready:
            set_thread_ready(*request.tcb);
            break;
      }
   }
}

// Cross-core safe posting API
bool scheduler::post_to_inbox(cross_core_request request) noexcept
{
   // Many-producer safe
   if (!inbox.push(request)) return false; // Full

   bool expected = false;
   if (inbox_poke_pending.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
      cyros_port_send_reschedule_ipi(core_id);
   }
   return true;
}

/**
 * @brief Selects the next runnable thread for this core and performs a context switch.
 *
 * Invariants / contract:
 * - Called only by the owning core of this scheduler (no cross-core mutation).
 * - @c current_thread is non-null and is the thread currently executing on this core.
 * - The currently running thread is not present in the ready matrix on entry.
 * - On entry, @c current_thread->state is one of:
 *     - Running    => running normally - treated as preempted/rotated and re-enqueued
 *                     (except idle).
 *     - Ready      => "armed-then-woken before yield" - the thread armed itself on
 *                     a wait_queue but was woken by a concurrent signaller before
 *                     it reached this point. Indistinguishable from Running for
 *                     scheduling purposes: rotate and re-enqueue.
 *     - Blocked    => parking on a wait_queue - must already be removed from ready
 *                     structures. Not re-enqueued.
 *     - Terminated => must not be re-enqueued.
 * - Any cross-core readying requests must be visible via @c drain_inbox() before selection.
 */
void scheduler::reschedule() noexcept
{
   CYROS_ASSERT(current_thread);
   CYROS_ASSERT(!current_thread->is_enqueued());

   drain_inbox();

   auto* previous_thread = current_thread;

   switch (previous_thread->state) {
      case thread_control_block::thread_state::running:
      case thread_control_block::thread_state::ready:
         set_thread_ready(*previous_thread);
         break;

      case thread_control_block::thread_state::blocked:
      case thread_control_block::thread_state::terminated:
         break;
   }

   auto* next_thread = ready_matrix.pop_best_thread();
   if (!next_thread) next_thread = idle_thread;

   current_thread = next_thread;
   next_thread->state = thread_control_block::thread_state::running;
   cyros_port_switch(previous_thread->context(), next_thread->context());
}

void scheduler::reset()
{
   CYROS_ASSERT_OP(inbox.approx_size(), ==, 0); // Cannot reset whilst inbox is not empty
   CYROS_ASSERT(ready_matrix.empty()); // Cannot reset whilst threads still in the queue

   pinned_thread_counter.store(0, std::memory_order_relaxed);
   inbox_poke_pending.store(false, std::memory_order_relaxed);
   current_thread = nullptr;
}

}  // namespace cyros