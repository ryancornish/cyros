/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

#include "port_critical.hpp"
#include "scheduler.hpp"
#include "trace_points.hpp"

#include <cyros/kernel/assert.hpp>
#include <cyros/kernel/core.hpp>

#include <cstdint>

namespace cyros
{

void scheduler::pin_thread(thread_control_block& tcb)
{
   CYROS_ASSERT(core_id < CYROS_PORT_CORE_COUNT);
   tcb.pinned_core = core_id;
   pinned_thread_counter.fetch_add(1, std::memory_order_relaxed);
}

void scheduler::initialise(std::uint32_t const core)
{
   CYROS_ASSERT_OP(core, <, config::cores);
   core_id = core;

   stack_layout slayout(idle_stack, 0);
   idle_thread = ::new (slayout.tcb) thread_control_block(
      config::max_priorities-1,
      core_affinity::from_id(core_id),
      slayout.user_stack,
      idle_task
   );
   idle_thread->id = idle_thread_id;
   idle_thread->state = thread_state::ready;
   idle_thread->pinned_core = core_id;
}

void scheduler::start()
{
   CYROS_ASSERT(idle_thread != nullptr); // initialise() must run before start()

   auto* first = ready_matrix.pop_best_thread();
   if (first == nullptr) {
      first = idle_thread;
   }
   CYROS_ASSERT(first != nullptr);
   CYROS_ASSERT_OP(first->state, ==, thread_state::ready);

   set_thread_running(*first);

   //cyros_port_set_thread_pointer(current_thread);
   cyros_port_start_first(current_thread->context());
}

CYROS_HOT schedule_hint scheduler::set_thread_ready(thread_control_block& tcb, reschedule_policy const policy)
{
   CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);

   // Interrupt-masked because this is the choke point every wake funnels
   // through, including ISR wakes: the matrix splice must be atomic against
   // both a cross-core reschedule IPI landing mid-splice (a latent race even in
   // thread context) and an ISR wake re-entering it.
   auto const mask = cyros_port_irq_save();
   auto const hint = make_ready(tcb);

   // A wake that readies a thread more urgent than the running one, made by
   // the running thread itself at baseline, preempts it here and now, on the
   // thread's road (Switching threads, below). The wake knows it is a
   // preemption, so nothing need work that out later. Anywhere else, an
   // interrupt or a masked region, the switch waits for the next safe point,
   // which the caller's pend arranges and reschedule() serves.
   if (hint == schedule_hint::warranted && policy != reschedule_policy::never &&
       cyros_port_masked_from_baseline(mask))
   {
      direct_reschedule(departure_reason::preempted, mask);
      return schedule_hint::rescheduled;
   }
   cyros_port_irq_restore(mask);
   return hint;
}

void scheduler::set_thread_terminated(thread_control_block& tcb)
{
   CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);
   CYROS_ASSERT_OP(tcb.state, ==, thread_state::running);
   CYROS_ASSERT(tcb.disposition == thread_disposition::terminating);
   CYROS_ASSERT(&tcb != idle_thread); // A core always needs somewhere to go
   CYROS_ASSERT(!tcb.is_enqueued());
   CYROS_REQUIRE(tcb.holds_nothing()); // Thread cannot own a base_mutex on termination

   tcb.state = thread_state::terminated;

   trace::thread_terminated(tcb); // Before the raise, after which the TCB may be gone

   thread_registry::unregister_thread(tcb);

   cyros_port_context_destroy(tcb.context());

   // Signal any and all joiners waiting on this thread.
   // This is the last valid action for this TCB, nothing may
   // access it afterwards. Any joiners on this can destroy the
   // outer thread handle if desired. This releases the owner
   // from the promise that the stack outlives this thread.
   //
   // Never pend: this runs inside reschedule(), before its pick, so the
   // pick that follows already sees every joiner the raise readied. A pend
   // would buy a second reschedule after the switch, which would send the
   // thread just picked to the back of its level.
   tcb.termination.raise(reschedule_policy::never);
}

std::uint8_t scheduler::current_thread_urgency() const
{
   return current_thread ? urgency(*current_thread) : 0;
}

void scheduler::service_intake(thread_control_block& tcb, std::uint8_t bits)
{
   if (bits & thread_control_block::request_bit(thread_request::make_ready)) {
      if (tcb.disposition != thread_disposition::terminating) {
         tcb.disposition = thread_disposition::none;
      }
      // Runs as a switch takes the intake in, before its pick, so there is
      // no hint to act on.
      (void)set_thread_ready(tcb);
   }

}

void scheduler::post_intake(thread_control_block& tcb, thread_request request)
{
   auto const bit = thread_control_block::request_bit(request);

   // One atomic answers both questions. A previously-zero field means this TCB
   // is not on the intake and we must put it there. Any non-zero value means it
   // already is, and since these requests carry truth rather than values, the
   // entry already queued discharges this intent too.
   auto const old = tcb.pending_requests.fetch_or(bit, std::memory_order_acq_rel);
   if (old != 0) return;

   if (push_intake(tcb)) {
      // Empty-to-non-empty, so nobody else has poked this core for this batch.
      cyros_port_send_reschedule_ipi(core_id);
   }
}

void scheduler::link_holder(thread_control_block& tcb)
{
   CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);
   if (tcb.is_listed_holder()) return; // already tracked

   // Appended, not pushed. pick_next keeps the first holder at the best
   // urgency, so head insertion would give every tie to the most recently
   // linked, which is the thread that just ran. See scheduler::holders_head.
   tcb.holder_next = nullptr;
   if (holders_tail == nullptr) {
      holders_head = &tcb;
   } else {
      holders_tail->holder_next = &tcb;
   }
   holders_tail = &tcb;
}

void scheduler::unlink_holder(thread_control_block& tcb)
{
   CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);
   if (!tcb.is_listed_holder()) return; // not tracked

   // Singly linked and removed by search. The list is holders-on-this-core,
   // measured at about one, so the search is not worth a back pointer in the TCB,
   // and TCB bytes are the scarcer resource. The predecessor is tracked here
   // rather than through a pointer-to-pointer walk only because the tail needs
   // repairing when the last element goes.
   thread_control_block* previous = nullptr;
   auto* current = holders_head;
   while (current != nullptr && current != &tcb) {
      previous = current;
      current  = current->holder_next;
   }
   CYROS_ASSERT(current == &tcb); // holder missing from its own core's list

   if (previous == nullptr) {
      holders_head = tcb.holder_next;
   } else {
      previous->holder_next = tcb.holder_next;
   }
   if (holders_tail == &tcb) {
      holders_tail = previous;
   }
   tcb.holder_next = &tcb; // restore the not-linked sentinel
}

bool scheduler::push_intake(thread_control_block& tcb)
{
   // The self-pointer says "position published, link not yet". It must be in
   // place BEFORE the exchange makes the node reachable. nullptr cannot serve as
   // this marker: the bottom of the chain legitimately ends in nullptr, and a
   // consumer reading it there would stop early and strand everything beneath.
   tcb.intake_next.store(&tcb, std::memory_order_relaxed);

   // Interrupts masked across the exchange and the link store. This is what
   // bounds the consumer's wait in await_intake_link to two instructions, and
   // it is the entire reason an exchange is acceptable here instead of a CAS
   // retry loop. Preempt-grade would NOT do: the timer ISR would still fire and
   // can itself wake a thread, so a same-core ISR could run a whole push while
   // this node is still showing the sentinel.
   //
   // enter_critical is depth-based, so calling this from an ISR that already
   // masked (a future chrono alarm doing wake_one) simply nests.
   port_critical_guard guard;

   auto* prev = intake_head.exchange(&tcb, std::memory_order_acq_rel);

   tcb.intake_next.store(prev, std::memory_order_release);

   // Empty-to-non-empty is decided by the same atomic that enqueued, so unlike a
   // separate poke flag it cannot go stale against the queue it describes.
   return prev == nullptr;
}

/**
 * @brief Wait for a producer to publish a node's link. Bounded, see push_intake.
 */
static thread_control_block* await_intake_link(thread_control_block* node)
{
   thread_control_block* next = nullptr;
   while ((next = node->intake_next.load(std::memory_order_acquire)) == node) {
      cyros_port_cpu_relax();
   }
   return next;
}

void scheduler::drain_intake()
{
   auto* node = intake_head.exchange(nullptr, std::memory_order_acq_rel);
   if (node == nullptr) return;

   // The stack is LIFO and thread_ready_queue is FIFO within a priority, which
   // round robin's fairness depends on, so restore arrival order before
   // servicing. O(taken), same as the walk that follows.
   thread_control_block* fifo = nullptr;
   while (node != nullptr) {
      auto* next = await_intake_link(node);
      node->intake_next.store(fifo, std::memory_order_relaxed);
      fifo = node;
      node = next;
   }

   while (fifo != nullptr) {
      auto* next = fifo->intake_next.load(std::memory_order_relaxed);
      fifo->intake_next.store(nullptr, std::memory_order_relaxed);

      // Release the claims BEFORE servicing, so a request raised while we are
      // servicing this one queues fresh instead of folding into work already
      // done. acq_rel because taking all bits at once is a read-modify-write.
      auto const bits = fifo->pending_requests.exchange(0, std::memory_order_acq_rel);

      service_intake(*fifo, bits);
      fifo = next;
   }
}

/* ============================================================================
 * Switching threads
 * ----------------------------------------------------------------------------
 * A core switches threads because its running thread departs, for one of
 * four reasons, each of which files it somewhere different (depart):
 *
 *   yielded     to the back of its level, behind its equals
 *   preempted   a strictly more urgent thread is ready: to the FRONT of its
 *               level, since it gave nothing up (POSIX's SCHED_FIFO, ThreadX
 *               and Zephyr have it so)
 *   parked      it committed to blocking: out of the ready structures
 *   exited      it is retired, and its joiners woken
 *
 * The next thread is then the pick (pick_next), or the idle thread, and it
 * becomes the running thread (run_next). Before any of it, wakes posted by
 * other cores are taken in (drain_intake), so the pick sees them. That is the
 * whole of a switch, and there are two roads into it. The steps, and the
 * thread's road, are inline in scheduler.hpp.
 *
 * The thread's road, direct_reschedule. The running thread yields, parks,
 * or readies a thread more urgent than itself, at baseline. It knows why it
 * departs, so it takes the steps itself, at once, inside the critical section
 * that made the change, and hands the port the result (cyros_port_direct_switch).
 * Nothing can come between the change and the switch: this core's interrupts
 * stay masked throughout, and another core's wakes reach this core only
 * through the intake, which the road takes in first.
 *
 * The handler's road, reschedule(). Every other reason to switch arises where
 * no switch can be made: an interrupt's wake, a wake inside a masked region,
 * a time slice, a remote core's doorbell. So it pends a reschedule, and the
 * port calls reschedule() from its handler at the next safe point. By then
 * other wakes may have landed, and nobody can say why the thread departs, so
 * reschedule() works it out from what the thread was left as (departure_of).
 * That reconstruction is the one step the thread's road does not need.
 *
 * Exit always takes the handler's road (this_thread::thread_exit yields to
 * it). The handler runs on the port's own stack, and an exiting thread must
 * not run on its own stack again once its joiners may free it
 * (set_thread_terminated).
 *
 * The two-phase block meets both roads. A waiter arms, polls, and only then
 * commits, under the mask: still prepared, it parks on the thread's road,
 * and if a wake landed in between, which cleared prepared, it does not park
 * (commit_to_block). A waiter preempted before its commit is filed with
 * prepared kept, so its commit still decides when it runs again. Termination
 * is the one wish a wake may not revoke, enforced where the wake lands:
 * set_thread_ready and global_ready_thread refuse a terminating thread
 * outright.
 * ========================================================================= */

CYROS_HOT void scheduler::yield()
{
   auto const mask = cyros_port_irq_save();
   CYROS_REQUIRE(cyros_port_masked_from_baseline(mask)); // yield() off baseline: in an interrupt, a critical section or with preemption disabled

   direct_reschedule(departure_reason::yielded, mask);
}

CYROS_HOT void scheduler::park(cyros_mask_token_t const mask)
{
   CYROS_REQUIRE(cyros_port_masked_from_baseline(mask)); // Blocked off baseline: in an interrupt, a critical section or with preemption disabled

   direct_reschedule(departure_reason::parked, mask);
}

/**
 * @brief The handler's road: called by the port at a safe point, after a
 *        reschedule was pended or the port's yield (Switching threads, above).
 *
 * Entry contract: called only by the owning core, current_thread non-null and
 * not enqueued. A blocked or created thread is never the running thread.
 */
CYROS_HOT void scheduler::reschedule()
{
   CYROS_ASSERT(current_thread);
   CYROS_ASSERT(!current_thread->is_enqueued());

   if constexpr (config::cores > 1) {
      if (intake_pending()) {
         drain_intake();
      }
   }

   auto& prev_thread = *current_thread;

   // Where the port saves the current context into.
   // nullptr means discard the current context (akin to terminating it)
   cyros_port_context* capture_context = prev_thread.context();
   thread_control_block* next_thread = nullptr;
   {
      // The departure and the pick must observe and mutate the matrix
      // atomically. A pick made stale by an ISR-triggered wake is repaired by
      // the ISR's own pend tail-chaining a corrective reschedule.
      port_critical_guard guard;

      switch (prev_thread.state) {
         case thread_state::running: {
            auto const reason = departure_of(prev_thread);
            if (reason == departure_reason::exited) {
               capture_context = nullptr; // Tell the port to retire the current context
            }
            depart(prev_thread, reason);
            break;
         }

         case thread_state::ready:       // Readied just now, by a remote wake taken in
         case thread_state::terminated:  // Retired by an earlier pass
            break;

         case thread_state::blocked:
         case thread_state::created:
         default:
            CYROS_ASSERT1(false, prev_thread.state); // Illegal thread state
            break;
      }

      next_thread = &run_next();
   }

   cyros_port_switch(capture_context, next_thread->context());
}

[[gnu::noinline, gnu::cold]] thread_control_block* scheduler::pick_with_holders()
{
   // The matrix is keyed on base_priority, which no boost can change, so its
   // head is the most urgent NON-holder by construction.
   auto const best_base = ready_matrix.best_priority();

   thread_control_block* best_holder = nullptr;
   std::uint8_t best_holder_urgency = 0xFF;
   for (auto* h = holders_head; h != nullptr; h = h->holder_next) {
      auto const u = urgency(*h);
      if (u < best_holder_urgency) {
         best_holder_urgency = u;
         best_holder = h;
      }
   }

   if (best_holder == nullptr) {
      return ready_matrix.pop_best_thread();
   }
   if (best_base >= 0) {
      auto const base = static_cast<std::uint8_t>(best_base);
      if (base < best_holder_urgency) {
         return ready_matrix.pop_best_thread();
      }
      if (base == best_holder_urgency) {
         /* No arrival order spans the two structures, so alternate. Giving the
          * tie to either side outright starves the other outright: the loser is
          * re-examined on the next pick and loses identically, because being
          * passed over does not change its position. See tie_rotor. */
         auto const bit = std::uint32_t{1} << base;
         tie_rotor ^= bit;
         if ((tie_rotor & bit) == 0) {
            return ready_matrix.pop_best_thread();
         }
      }
   }

   unlink_holder(*best_holder);
   return best_holder;
}

void scheduler::reset()
{
   // At shutdown the intake may still hold stale requests: a signaller can post
   // faster than the target drains, and the target may terminate with surplus
   // wakes outstanding. A request against a terminated thread is a no-op, but we
   // must not leave the chain behind.
   // The intake links live in TCBs, which are user memory that outlives the
   // kernel run, so a leftover chain would be inherited by the next lifecycle.
   // Construction is a placement new so a REUSED TCB comes back clean, but a
   // TCB that is merely still alive would not. Clear both ends here.
   auto* node = intake_head.exchange(nullptr, std::memory_order_relaxed);
   while (node != nullptr) {
      // No await_intake_link here: every core thread is joined before teardown,
      // so no push can be in flight and no link can still be unpublished.
      auto* next = node->intake_next.load(std::memory_order_relaxed);
      node->intake_next.store(nullptr, std::memory_order_relaxed);
      node->pending_requests.store(0, std::memory_order_relaxed);
      CYROS_ASSERT_OP(node->state, ==, thread_state::terminated);
      node = next;
   }
   CYROS_ASSERT(ready_matrix.empty()); // Bug: Cannot reset whilst threads still in the queue

   CYROS_ASSERT(holders_head == nullptr); // Bug: An occupant in list means a thread was readied as a holder and the core never ran it
   holders_tail = nullptr;
   // Not correctness, but a lifecycle should not inherit the previous one's
   // tie phase: these objects outlive kernel::finalise.
   tie_rotor = 0;

   pinned_thread_counter.store(0, std::memory_order_relaxed);
   current_thread = nullptr;

   if (idle_thread != nullptr) {
      cyros_port_context_destroy(idle_thread->context());
      idle_thread = nullptr;
   }
}

}  // namespace cyros