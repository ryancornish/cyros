/**
 * @file waitable.cpp
 * @brief Implementation of the waitable kernel base class.
 *
 * Lost-wakeup correctness rests on two cooperating mechanisms:
 *
 *  1. The two-phase block in this file: arm (link a wait_node into the
 *     queue under the queue lock, set the thread's state to blocked), then
 *     park (yield to the scheduler). The queue lock is released between
 *     arm and park.
 *
 *  2. The scheduler's tolerance of state==ready on entry to reschedule().
 *     A wake that arrives in the window between arm-unlock and park sees a
 *     properly-linked, blocked thread; it unlinks the node and sets
 *     state=ready. When the thread then yields, reschedule() sees
 *     state==ready and treats it as a rotation (re-enqueue, pick the next
 *     thread). No yield is "lost"; no wake is dropped.
 *
 * Block-on-any uses a SEPARATE wait_node per source, allocated on the
 * blocking thread's stack. A wait_node has exactly one 'next' pointer; it
 * cannot be in two intrusive lists at once. The TCB's embedded wait_node
 * serves single-wait blocks at zero extra cost; block_on_any pays N stack
 * nodes for N sources, with no heap and no pool.
 */

#include <cyros/kernel/waitable.hpp>
#include <cyros/port/port.h>

#include "scheduler.hpp"
#include "threading_subsystem.hpp"

namespace cyros
{

using reschedule_policy = waitable::reschedule_policy;

/**
 * @brief Applies the caller's reschedule_policy to a completed wake.
 *
 * Governs only the LOCAL pend: cross-core wakes have already emitted their IPI via the
 * ready path before this runs, so 'never' does not (and must not) suppress those.
 */
static void apply_reschedule_policy(reschedule_policy const policy, schedule_hint const hint)
{
   if (policy == reschedule_policy::never) {
      return;
   }

   if (policy == reschedule_policy::always || hint == schedule_hint::warranted) {
      cyros_port_pend_reschedule();
   }
}

/* ============================================================================
 * waitable::wait_queue
 * ========================================================================= */

bool waitable::wait_queue::empty() const noexcept
{
   // Advisory: not under lock. Fine for "should I bother waking" hints.
   return head == nullptr;
}

void waitable::wait_queue::arm(wait_node& n) noexcept
{
   spinlock_guard guard(lock);

   // Mark the thread blocked while we still hold the queue lock. A concurrent
   // wake on this queue serialises on this lock; a wake that sees the node
   // here is guaranteed to take the unlink-and-ready path correctly.
   CYROS_ASSERT(n.owner != nullptr);
   CYROS_ASSERT(n.next  == nullptr); // node must not already be on a list

   n.owner->state = thread_control_block::thread_state::blocked;

   // Priority-ordered insert (best at head).
   wait_node** slot = &head;
   while (*slot && (*slot)->owner->effective_priority <= n.owner->effective_priority) {
      slot = &(*slot)->next;
   }
   n.next = *slot;
   *slot  = &n;
}

void waitable::wait_queue::disarm(wait_node& n) noexcept
{
   spinlock_guard guard(lock);

   // Unlink n if still present. If a wake already removed it, the loop simply
   // finds nothing and falls through - that is the expected path when disarm
   // runs after a wake. Critically, block_on_any disarms ALL its nodes after
   // a wake. Only the queue that woke the thread had the node, the others
   // already let go of it. Idempotent unlink makes that safe.
   wait_node** slot = &head;
   while (*slot && *slot != &n) slot = &(*slot)->next;
   if (*slot == &n) {
      *slot = n.next;
      n.next = nullptr;
   }

   // Restore running state under the lock if this disarm is being called
   // because the caller decided NOT to park (predicate already true). If the
   // node was already removed by a wake, state is already 'ready' and we must
   // leave that alone - the scheduler will rotate via the ready path.
   if (n.owner->state == thread_control_block::thread_state::blocked) {
      n.owner->state = thread_control_block::thread_state::running;
   }
}

void waitable::wait_queue::wake_one(reschedule_policy policy) noexcept
{
   thread_control_block* woken = nullptr;
   {
      spinlock_guard guard(lock);
      if (head == nullptr) return;

      wait_node* n = head;
      head = n->next;
      n->next = nullptr;

      // Mark ready under the lock so a racing arm/disarm sees a consistent
      // state.
      n->owner->state = thread_control_block::thread_state::ready;
      woken = n->owner;
   }

   schedule_hint hint = wake_thread(*woken);
   apply_reschedule_policy(policy, hint);
}

void waitable::wait_queue::wake_all(reschedule_policy policy) noexcept
{
   wait_node* batch = nullptr;
   {
      spinlock_guard guard(lock);
      batch = head;
      head = nullptr;
      for (wait_node* n = batch; n != nullptr; n = n->next) {
         n->owner->state = thread_control_block::thread_state::ready;
      }
   }

   schedule_hint aggregate = schedule_hint::unwarranted;
   for (wait_node* n = batch; n != nullptr; ) {
      wait_node* next = n->next;
      n->next = nullptr;
      schedule_hint h = wake_thread(*n->owner);
      if (h == schedule_hint::warranted) {
         aggregate = schedule_hint::warranted;
      }
      n = next;
   }

   apply_reschedule_policy(policy, aggregate);
}

/* ============================================================================
 * waitable - public surface
 * ========================================================================= */

waitable::~waitable()
{
   // It is a programming error to destroy an waitable with parked waiters.
   CYROS_ASSERT(queue.empty());
}

bool waitable::is_satisfied(thread_control_block& /*caller*/) noexcept
{
   return false;
}

void waitable::wake_one(reschedule_policy policy) noexcept
{
   queue.wake_one(policy);
}

void waitable::wake_all(reschedule_policy policy) noexcept
{
   queue.wake_all(policy);
}

} // namespace cyros
