#include <cortos/kernel/waitable.hpp>
#include <cortos/port/port.h>

#include "wait_subsystem.hpp"
#include "threading_subsystem.hpp"

namespace cortos
{

[[nodiscard]] bool waitable::empty() const noexcept
{
   return head == nullptr && tail == nullptr;
}

void waitable::add(wait_node& wait_node) noexcept
{
   CORTOS_ASSERT1(wait_node.active_waitable == this || wait_node.active_waitable == nullptr, wait_node.active_waitable);
   CORTOS_ASSERT(!wait_node.is_enqueued());

   wait_node.active_waitable = this;

   wait_node.prev = tail;
   wait_node.next = nullptr;
   if (tail) {
      tail->next = &wait_node;
   } else {
      head = &wait_node;
   }
   tail = &wait_node;
}

void waitable::remove(wait_node& wait_node) noexcept
{
   if (wait_node.active_waitable != this) return;

   if (wait_node.prev) {
      wait_node.prev->next = wait_node.next;
   } else {
      head = wait_node.next;
   }
   if (wait_node.next) {
      wait_node.next->prev = wait_node.prev;
   } else {
      tail = wait_node.prev;
   }
   wait_node.prev = wait_node.next = nullptr;
   wait_node.active_waitable = nullptr;
}

wait_node* waitable::pick_best() noexcept
{
   wait_node* best = nullptr;
   uint8_t best_priority = std::numeric_limits<uint8_t>::max();

   for (auto* iter = head; iter; iter = iter->next) {
      CORTOS_ASSERT(iter->active);
      CORTOS_ASSERT(iter->tcb != nullptr);
      CORTOS_ASSERT(iter->active_waitable == this);

      // Skip nodes from already-satisfied wait_for_any groups (race window during teardown / multi-signal).
      if (iter->active_group && iter->active_group->done.load(std::memory_order_acquire)) continue;

      auto* tcb = iter->tcb;
      if (!best || tcb->is_higher_priority_than(best_priority)) {
         best = iter;
         best_priority = tcb->effective_priority;
      }
   }
   return best;
}

void waitable::signal_one(bool acquired) noexcept
{
   wait_node* wait_node = nullptr;
   {
      spinlock_guard guard(wait_lock);
      wait_node = pick_best();
   }
   if (!wait_node) return; // No current waiters

   if (wait_node->wake_thread(acquired) == ready_action::reschedule) {
      cortos_port_pend_reschedule();
   }
}

void waitable::signal_all(bool acquired) noexcept
{
   bool reschedule = false;

   while (true) {
      wait_node* wait_node = nullptr;
      {
         spinlock_guard guard(wait_lock);
         wait_node = pick_best();
      }
      if (!wait_node) break;

      if (wait_node->wake_thread(acquired) == ready_action::reschedule) {
         reschedule = true;
      }
   }
   if (reschedule) {
      cortos_port_pend_reschedule();
   }
}

void waitable::for_each_waiter(waiter_visitor visitor) const
{
   // TODO: Might need a spinlock for cross-core waiting?
   for (auto* iter = head; iter; iter = iter->next) {
      if (!iter->active) continue;
      if (iter->active_group && iter->active_group->done.load(std::memory_order_acquire)) continue;
      if (!iter->tcb) continue;

      visitor(iter->tcb->create_waiter());
   }
}

} // namespace cortos
