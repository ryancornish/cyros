#include "scheduler.hpp"

namespace cortos
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
      idle_task
   );
   idle_thread->pinned_core = core_id;
}

// Core-local operations (only called on owning core)
void scheduler::start() noexcept
{
   CORTOS_ASSERT(idle_thread != nullptr); // init_idle_thread() must run before start()

   auto* first = ready_matrix.pop_best_thread();
   if (first == nullptr) {
      first = idle_thread;
   }
   CORTOS_ASSERT(first != nullptr);
   CORTOS_ASSERT_OP(first->state, ==, thread_control_block::thread_state::ready);
   first->state = thread_control_block::thread_state::running;
   current_thread = first;

   //cortos_port_set_thread_pointer(current_thread);
   cortos_port_start_first(current_thread->context());
}

void scheduler::set_thread_ready(thread_control_block& tcb) noexcept
{
   CORTOS_ASSERT_OP(tcb.pinned_core, ==, core_id);

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
      cortos_port_send_reschedule_ipi(core_id);
   }
   return true;
}

/**
* @brief Selects the next runnable thread for this core and performs a context switch.
*
* Invariants / contract:
* - Called only by the owning core of this scheduler (no cross-core mutation).
* - @c current_thread is non-null and is the thread currently executing on this core.
* - On entry, @c current_thread->state is NEVER Ready:
*     - Running    => treated as preempted/rotated and re-enqueued as Ready (except idle).
*     - Blocked    => must already be removed from ready structures - not re-enqueued.
*     - Terminated => must not be re-enqueued.
* - The currently running thread is not present in the ready matrix on entry.
* - Any cross-core readying requests must be visible via @c drain_inbox() before selection.
*/
void scheduler::reschedule() noexcept
{
   CORTOS_ASSERT(current_thread);
   CORTOS_ASSERT(!current_thread->is_enqueued());
   CORTOS_ASSERT(current_thread->state != thread_control_block::thread_state::ready);

   drain_inbox();

   auto* previous_thread = current_thread;

   switch (previous_thread->state) {
      case thread_control_block::thread_state::running:
         set_thread_ready(*previous_thread);
         break;

      case thread_control_block::thread_state::blocked:
      case thread_control_block::thread_state::terminated:
         break;

      case thread_control_block::thread_state::ready:
         __builtin_unreachable();
   }

   auto* next_thread = ready_matrix.pop_best_thread();
   if (!next_thread) next_thread = idle_thread;

   current_thread = next_thread;
   next_thread->state = thread_control_block::thread_state::running;
   cortos_port_switch(previous_thread->context(), next_thread->context());
}

void scheduler::prepare_block_current_thread(std::span<waitable* const> waitables)
{
   current_thread->prepare_block(waitables);
}

waitable::result scheduler::commence_block_current_thread()
{
   return current_thread->commence_block();
}

void scheduler::notify_block_current_thread(std::span<waitable* const> waitables) const
{
   current_thread->notify_block(waitables);
}

void scheduler::reset()
{
   CORTOS_ASSERT_OP(inbox.approx_size(), ==, 0); // Cannot reset whilst inbox is not empty
   CORTOS_ASSERT(ready_matrix.empty()); // Cannot reset whilst threads still in the queue

   pinned_thread_counter.store(0, std::memory_order_relaxed);
   inbox_poke_pending.store(false, std::memory_order_relaxed);
   current_thread = nullptr;
}

}  // namespace cortos