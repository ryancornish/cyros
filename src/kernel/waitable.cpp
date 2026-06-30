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

void waitable::wait_queue::arm(wait_node& node) noexcept
{
   spinlock_guard guard(lock);

   CYROS_ASSERT(node.owner != nullptr);
   CYROS_ASSERT(node.next  == nullptr); // node must not already be on a list

   // For now, hard code the state change. They are supposed to all be done through the
   // scheduler interface, but we dont have access to that here
   node.owner->state = thread_control_block::thread_state::running_pending;

   // Priority-ordered insert (best at head).
   wait_node** slot = &head;
   while (*slot && (*slot)->owner->effective_priority <= node.owner->effective_priority) {
      slot = &(*slot)->next;
   }
   node.next = *slot;
   *slot  = &node;
}

void waitable::wait_queue::disarm(wait_node& node) noexcept
{
   spinlock_guard guard(lock);

   // Unlink n if still present. A wake may have already removed it;
   // idempotent unlink makes that safe (no-op if not found).
   wait_node** slot = &head;
   while (*slot && *slot != &node) {
      slot = &(*slot)->next;
   }
   if (*slot == &node) {
      *slot = node.next;
      node.next = nullptr;
   }
}

void waitable::wait_queue::wake_one(reschedule_policy policy) noexcept
{
   thread_control_block* woken = nullptr;
   {
      spinlock_guard guard(lock);

      if (head == nullptr) return;
      wait_node* node = head;
      woken = node->owner;
      head = node->next;
      node->next = nullptr;
   }

   schedule_hint hint = kernel_set_thread_ready(*woken);
   apply_reschedule_policy(policy, hint);
}

void waitable::wait_queue::wake_all(reschedule_policy policy) noexcept
{
   wait_node* batch = nullptr;
   {
      spinlock_guard guard(lock);
      batch = head;
      head = nullptr;
   }

   schedule_hint aggregate_hint = schedule_hint::unwarranted;
   for (wait_node* node = batch; node != nullptr; ) {
      wait_node* next = node->next;
      node->next = nullptr;
      schedule_hint hint = kernel_set_thread_ready(*node->owner);
      if (hint == schedule_hint::warranted) {
         aggregate_hint = schedule_hint::warranted;
      }
      node = next;
   }

   apply_reschedule_policy(policy, aggregate_hint);
}

/* ============================================================================
 * waitable - public surface
 * ========================================================================= */

waitable::~waitable()
{
   // It is a programming error to destroy an waitable with parked waiters.
   CYROS_ASSERT(queue.empty());
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
