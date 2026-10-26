/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file waitable.cpp
 * @brief Implementation of the waitable kernel base class.
 *
 * Lost-wakeup correctness rests on the two-phase block in this file: the
 * thread says it is prepared to block, arms (links a wait_node into the queue
 * under the queue lock), polls, and only then commits (commit_to_block),
 * which parks it only if it is still prepared, looked at and parked in one
 * critical section. The queue lock is released between arm and commit. A
 * wake that lands in that window takes the node and clears the disposition,
 * so the commit finds it cleared and does not park, and a wake from another
 * core that is still in flight is taken in by the park's own switch, which
 * finds the thread readied and does not park it either. No wake is dropped.
 *
 * A wait_node has exactly one 'next' pointer, so it cannot be in two intrusive
 * lists at once. A wait on w with a cancel_signal therefore uses two, one per
 * queue. Both live on the blocking thread's own stack, so there is no heap
 * and no pool. Every wait goes through wait_engine::run, at the bottom.
 */

#include <cyros/kernel/waitable.hpp>
#include <cyros/kernel/assert.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/port/port.h>

#include "base_mutex_access.hpp"
#include "scheduler.hpp"
#include "threading_subsystem.hpp"

#include <algorithm>
#include <array>
#include <bit>

namespace cyros
{

namespace
{

/**
 * @brief min(base, best waiter of every base_mutex held).
 *
 * The definition of effective priority, folded from current truth.
 *
 * Evaluable by any core at any time and with no pi_lock: held_slots is a bounded
 * array of atomics and the queue tops are maintained atomics for exactly that.
 * The one restriction is on queue locks, see urgency_at.
 */
[[nodiscard]] std::uint8_t donated_floor(thread_control_block const& target, unsigned const depth) noexcept
{
   // Fast path: holds nothing, so urgency is base priority by definition. This
   // is the overwhelming majority of calls, measured at 98.9 percent of picks in
   // the only test in the suite that exercises mutexes at all, and 100 percent
   // everywhere else.
   auto mask = target.held_mask.load(std::memory_order_acquire);
   if (mask == 0) return target.base_priority;

   std::uint8_t floor = target.base_priority;
   while (mask != 0) {
      auto const slot = static_cast<std::size_t>(std::countr_zero(mask));
      mask &= static_cast<std::uint8_t>(mask - 1); // clear lowest set bit

      auto* held = target.held_slots[slot].load(std::memory_order_acquire);
      if (held == nullptr) continue; // bit was stale, see the ordering note on held_mask
      floor = std::min(base_mutex_access::urgency_contribution(*held, depth), floor);
   }
   return floor;
}

/**
 * @brief Applies the caller's reschedule_policy to a completed wake.
 *
 * Governs only the LOCAL pend: cross-core wakes have already emitted their IPI via the
 * ready path before this runs, so 'never' does not (and must not) suppress those.
 */
void apply_reschedule_policy(reschedule_policy const policy, schedule_hint const hint)
{
   // A wake that took its reschedule itself has nothing left to ask for,
   // whatever its policy: `always` asks for one reschedule, not two.
   if (policy == reschedule_policy::never || hint == schedule_hint::rescheduled) {
      return;
   }

   if (policy == reschedule_policy::always || hint == schedule_hint::warranted) {
      cyros_port_pend_reschedule();
   }
}

} // namespace

/* ============================================================================
 * wait_queue
 * ========================================================================= */

/**
 * @brief Priority-ordered insert (best at head), keyed on BASE priority.
 *
 * Base never changes, so a node's position is fixed for the life of the wait and
 * nothing ever has to re-slot it. That is what removed the whole reslot-and-chase
 * machinery: the old key was effective priority, which moved under the queue and
 * had to be chased.
 */
CYROS_HOT void wait_queue::link(wait_node& node) noexcept
{
   // Behind every waiter as urgent or more, so FIFO within a priority. The
   // head is atomic (see its declaration), so it is the one slot not reached
   // through a node.
   auto* const first = head.load(std::memory_order_relaxed);
   if (first == nullptr || node.owner->base_priority < first->owner->base_priority) {
      node.next = first;
      head.store(&node, std::memory_order_relaxed);
      return;
   }
   wait_node* before = first;
   while (before->next && before->next->owner->base_priority <= node.owner->base_priority) {
      before = before->next;
   }
   node.next = before->next;
   before->next = &node;
}

/**
 * @brief Unlink node from wait_queue if present.
 * Idempotent if node not present.
 */
CYROS_HOT bool wait_queue::unlink(wait_node& node) noexcept
{
   auto* const first = head.load(std::memory_order_relaxed);
   if (first == &node) {
      head.store(node.next, std::memory_order_relaxed);
      node.next = nullptr;
      return true;
   }
   for (auto* before = first; before != nullptr; before = before->next) {
      if (before->next == &node) {
         before->next = node.next;
         node.next = nullptr;
         return true;
      }
   }
   return false;
}

std::uint8_t wait_queue::top(std::atomic<thread_control_block*> const& resource_owner,
                             unsigned const depth,
                             inheritance_cache const& pi) const noexcept
{
   auto const cached = pi.top_priority.load(std::memory_order_acquire);

   // Fast path: no waiter here holds anything, so none can be more urgent than
   // its base priority and the cache is exact. This is almost every queue.
   if (!pi.may_have_bridges.load(std::memory_order_acquire)) return cached; // "acquire" pairs with store's "release"

   // Budget exhausted means a wait-for cycle, i.e. the application has
   // deadlocked. Answer conservatively instead of following it forever.
   if (depth == 0) return 0;

   // Snapshot under the lock, then release BEFORE recursing. Holding it across
   // the recursion would nest queue locks around the wait-for graph.
   static constexpr std::size_t max_snapshot = 4;
   std::array<thread_control_block*, max_snapshot> bridges{};
   std::size_t found = 0;
   bool overflow = false;
   {
      spinlock_guard guard(const_cast<spinlock&>(lock));

      // Relaxed is enough here: the transition this read must not miss is the
      // handover, which stores the owner under this same lock, so the lock
      // orders it. See the header note for why the read must be under the
      // lock at all.
      auto* const holder = resource_owner.load(std::memory_order_relaxed);

      for (auto* n = head.load(std::memory_order_relaxed); n != nullptr; n = n->next) {
         // Re-derive from the current truth (stale answers are plausible and err toward over-boost.)
         if (n->owner->holds_nothing()) continue;
         // The owner itself, still armed in its own queue between the winning
         // CAS and its disarm. Recursing into it would be a self-cycle that
         // exhausts the budget and answers 0. See the note on top()'s
         // resource_owner parameter for why that is not the harmless
         // over-boost it looks like.
         if (n->owner == holder) continue;
         if (found == max_snapshot) { overflow = true; break; }
         bridges[found++] = n->owner;
      }
   }
   if (overflow) return 0; // Over-boost, the safe direction

   std::uint8_t best = cached;
   for (std::size_t i = 0; i < found; ++i) {
      best = std::min(best, urgency_at(*bridges[i], depth - 1));
   }
   return best;
}

CYROS_HOT void wait_queue::arm(wait_node& node, inheritance_cache* pi) noexcept
{
   spinlock_guard guard(lock);

   CYROS_ASSERT(node.owner != nullptr);
   CYROS_ASSERT(node.next  == nullptr); // node must not already be on a list

   link(node);

   if (pi != nullptr && !node.owner->holds_nothing()) {
      // Adding a bridge, set the sticky hint
      pi->may_have_bridges.store(true, std::memory_order_release); // "release" pairs with load's "acquire"
   }
   refresh_top(pi);
}

/**
 * @brief Remove an armed node, idempotent against a racing wake.
 * @return true if this removed the node, false if it was already gone.
 *
 * A node leaves a queue two ways: its owner disarms it, or a wake takes it
 * (every wake unlinks the node it chooses). So when the owner disarms each
 * node exactly once per arm, false means a wake chose that waiter.
 * wait_engine relies on exactly that, see the note there.
 *
 * Leaving a queue can only LOWER a holder's urgency, and a de-boost needs no
 * prompt because urgency is folded at the point of use, so nothing here
 * reports whether the queue's top moved.
 */
CYROS_HOT bool wait_queue::disarm(wait_node& node, inheritance_cache* pi) noexcept
{
   spinlock_guard guard(lock);

   if (!unlink(node)) return false;
   refresh_top(pi);
   return true;
}

inline void wait_queue::refresh_top(inheritance_cache* pi) noexcept
{
   if (pi == nullptr) return; // Nothing reads it, so do not maintain it

   // Requires the queue lock held. Release pairs with the acquire in top() so
   // a fold prompted by a queue change (via a doorbell, or by reaching this
   // queue through a bridge) observes the value that change produced.
   auto* const first = head.load(std::memory_order_relaxed);
   pi->top_priority.store(first != nullptr ? first->owner->base_priority : inheritance_cache::no_waiter,
                          std::memory_order_release);

   // No more waiters, so we can safely clear the sticky hint
   if (first == nullptr) {
      pi->may_have_bridges.store(false, std::memory_order_release); // "release" pairs with load's "acquire"
   }
}

CYROS_HOT void wait_queue::wake_one(reschedule_policy policy, inheritance_cache* pi) noexcept
{
   thread_control_block* chosen = nullptr;
   {
      spinlock_guard guard(lock);

      wait_node* node = head.load(std::memory_order_relaxed);
      if (node == nullptr) return;
      chosen = node->owner;
      head.store(node->next, std::memory_order_relaxed);
      node->next = nullptr;
      refresh_top(pi);
   }

   schedule_hint hint = global_ready_thread(*chosen, policy);
   apply_reschedule_policy(policy, hint);
}

void wait_queue::wake_all(reschedule_policy policy, inheritance_cache* pi) noexcept
{
   // Atomic batch admit. Preemption is held off for the entire batch so
   // every waiter lands on the ready matrix before any of them can run on this
   // core.
   schedule_hint aggregate_hint = schedule_hint::unwarranted;

   auto token = cyros_port_preempt_disable();

   while (true) {
      thread_control_block* chosen = nullptr;
      {
         spinlock_guard guard(lock);

         wait_node* node = head.load(std::memory_order_relaxed);
         if (node == nullptr) break;
         chosen = node->owner;
         head.store(node->next, std::memory_order_relaxed);
         node->next = nullptr;
         refresh_top(pi);
      }
      // never: the batch's one pend follows it, at the enable
      schedule_hint hint = global_ready_thread(*chosen, reschedule_policy::never);
      if (hint == schedule_hint::warranted) {
         aggregate_hint = schedule_hint::warranted;
      }
   }

   apply_reschedule_policy(policy, aggregate_hint);

   cyros_port_preempt_enable(token);
}

void wait_queue::raise_all(std::atomic<bool>& flag, reschedule_policy policy) noexcept
{
   // ONE hold of the lock, with the readies inside it, where every other wake
   // readies outside. A waiter returns from its wait only after its disarm,
   // which takes this lock, so neither a waiter woken here nor one whose poll
   // saw the flag can return, and destroy what owns this queue, until this is
   // done with it. wake_all re-takes the lock after each wake, and a joiner
   // could free the stack a termination lives in between two of them.
   //
   // The flag goes inside for the same reason: stored before the lock, a
   // waiter arriving in between could poll it, disarm and return before the
   // lock was ever taken.
   schedule_hint aggregate_hint = schedule_hint::unwarranted;

   auto token = cyros_port_preempt_disable();
   {
      spinlock_guard guard(lock);

      flag.store(true, std::memory_order_release);

      wait_node* node = head.load(std::memory_order_relaxed);
      head.store(nullptr, std::memory_order_relaxed);
      while (node != nullptr) {
         wait_node* const next = node->next;
         node->next = nullptr;
         if (global_ready_thread(*node->owner, reschedule_policy::never) == schedule_hint::warranted) {
            aggregate_hint = schedule_hint::warranted;
         }
         node = next;
      }
   }

   apply_reschedule_policy(policy, aggregate_hint);

   cyros_port_preempt_enable(token);
}

void wait_queue::wake_all_where(pick_fn pick, reschedule_policy policy) noexcept
{
   // wake_all's batch, preemption held off throughout, except that a waiter
   // pick refuses stays queued. One walk under the lock unlinks every waiter
   // pick accepts, and they are readied after it, since readying reaches the
   // scheduler, which no queue lock may be held across. What is kept for
   // that is each one's TCB, never its node: a node lives on its waiter's
   // stack and is touched only under the lock. A walk that fills the batch
   // walks again from the head, where the waiters it took are gone.
   schedule_hint aggregate_hint = schedule_hint::unwarranted;

   auto token = cyros_port_preempt_disable();

   std::array<thread_control_block*, 8> chosen [[indeterminate]];
   std::size_t count = 0;
   do {
      count = 0;
      {
         spinlock_guard guard(lock);

         wait_node* before = nullptr;
         wait_node* node = head.load(std::memory_order_relaxed);
         while (node != nullptr && count < chosen.size()) {
            wait_node* const next = node->next;
            if (pick(*node->owner)) {
               chosen[count++] = node->owner;
               if (before == nullptr) {
                  head.store(next, std::memory_order_relaxed);
               } else {
                  before->next = next;
               }
               node->next = nullptr;
            } else {
               before = node;
            }
            node = next;
         }
      }
      for (std::size_t i = 0; i < count; ++i) {
         if (global_ready_thread(*chosen[i], reschedule_policy::never) == schedule_hint::warranted) {
            aggregate_hint = schedule_hint::warranted;
         }
      }
   } while (count == chosen.size());

   apply_reschedule_policy(policy, aggregate_hint);

   cyros_port_preempt_enable(token);
}

CYROS_HOT bool wait_queue::wake_one_and_commit(commit_fn commit, reschedule_policy policy, inheritance_cache* pi) noexcept
{
   thread_control_block* chosen = nullptr;
   {
      spinlock_guard guard(lock);

      if (wait_node* node = head.load(std::memory_order_relaxed); node != nullptr) {
         chosen = node->owner;
         head.store(node->next, std::memory_order_relaxed);
         node->next = nullptr;
         refresh_top(pi);
      }
      CYROS_ASSERT(chosen == nullptr || chosen->id != 0);

      // The commit for BOTH outcomes happens under the lock. Deciding the
      // empty case outside it would let a waiter arm, poll the still-held
      // resource, and park just before this release frees it, a lost wakeup
      // with no future wake to recover it.
      commit(chosen);
   }

   if (chosen == nullptr) {
      return false;
   }

   // The commit is already done, so readying outside the lock is pure
   // delivery. Only the TCB is touched out here. The wait_node lives on the
   // waiter's stack and is only dereferenced under the lock, matching the
   // wake_one discipline.
   schedule_hint hint = global_ready_thread(*chosen, policy);
   apply_reschedule_policy(policy, hint);
   return true;
}

bool wait_queue::wake_one_and_transfer(transfer_fn transfer, reschedule_policy policy, inheritance_cache* pi) noexcept
{
   return wake_one_and_commit(
      [&transfer](thread_control_block* chosen) {
         transfer(chosen != nullptr ? chosen->id : 0);
      },
      policy, pi);
}

/* ============================================================================
 * waitable - public surface
 * ========================================================================= */

waitable::~waitable()
{
   // It is a programming error to destroy an waitable with parked waiters.
   CYROS_REQUIRE(queue.empty()); // Destroyed with waiters parked on it
}

CYROS_HOT void waitable::wake_one(reschedule_policy policy) noexcept
{
   queue.wake_one(policy);
}

void waitable::wake_all(reschedule_policy policy) noexcept
{
   queue.wake_all(policy);
}

void waitable::wake_all_where(std::function_ref<bool(waiter_record const*)> wants, reschedule_policy policy) noexcept
{
   // The record each waiter's poll gets, bound by identity as a wait binds
   // it. A queued waiter is inside its wait, so the record it installed
   // before arming is still there: it comes off only after the last disarm,
   // and a disarm takes the lock this walk holds.
   queue.wake_all_where(
      [this, &wants](thread_control_block const& owner) noexcept {
         waiter_record const* const record = owner.wait_record;
         return wants((record != nullptr && record->source == this) ? record : nullptr);
      },
      policy);
}

bool waitable::wake_one_and_transfer(transfer_fn transfer, reschedule_policy policy) noexcept
{
   return queue.wake_one_and_transfer(transfer, policy);
}

void waitable::anchor() noexcept
{
   // The key function, see its declaration.
}

/* ============================================================================
 * cancel_signal
 * ========================================================================= */

void cancel_signal::raise(reschedule_policy const policy) noexcept
{
   // The flag and the wakes together: a waiter that arms after them polls the
   // flag, and one armed before is woken. raise_all says why together.
   queue.raise_all(raised_flag, policy);
}

bool cancel_signal::try_satisfy(waiter_record*) noexcept
{
   return raised();
}

/* ============================================================================
 * waiter_record
 * ========================================================================= */

waiter_record::waiter_record(waitable& source) noexcept
   : source(&source), installed_on(&scheduler_for_this_core().get_current_thread())
{
   CYROS_REQUIRE(installed_on->wait_record == nullptr); // One record per thread at a time
   installed_on->wait_record = this;
}

waiter_record::~waiter_record()
{
   // Uninstalls from the thread that installed it, wherever this runs, so no
   // thread can be left holding a destroyed record. Not looking up the current
   // thread here is measured: about 23 cycles a wait on the U575.
   //
   // Unreachable as a caller error: the constructor refuses a second record on
   // a thread, so while this one lives its thread's slot holds exactly it.
   CYROS_ASSERT(installed_on->wait_record == this);
   installed_on->wait_record = nullptr;
}



std::uint8_t urgency_at(thread_control_block const& tcb, unsigned const depth) noexcept
{
   return donated_floor(tcb, depth);
}

CYROS_HOT void commit_to_block(thread_control_block& tcb)
{
   // Hazard: the thread could read 'prepared', an ISR could then clear the
   // disposition as it readies the thread, and the thread would park anyway,
   // on a wake already spent. Masking interrupts makes the look and the park
   // one step, because every write to a disposition is made on its thread's
   // own core: by the thread, by an ISR there, or by its scheduler. A remote
   // wake goes through the intake, which the park's switch takes in first.
   auto const mask = cyros_port_irq_save();
   if (tcb.disposition != thread_disposition::prepared) {
      cyros_port_irq_restore(mask);
      return;
   }

   // Parked and switched away in this same critical section, which the switch
   // releases. Returns once the thread has been woken and runs again.
   scheduler_for_this_core().park(mask);
}

void request_repick(thread_control_block& tcb)
{
   /* Prompt every core that could now want to run something different, by
    * walking the wait-for chain rather than shouting at all of them.
    *
    * The donor knows only the holder it failed to CAS against. If that holder is
    * itself blocked, its urgency rose too, and so did the urgency of whatever
    * holds the resource IT is parked on, all the way down. The thread that can
    * actually act is at the far end, and its core has no other reason to pick.
    *
    * blocked_on is what makes that chain walkable. Every read in here is a fact
    * whose staleness costs a misdirected or a missing prompt and never a wrong
    * value, so no lock is taken and nothing is retried: a lost prompt is
    * repaired by the next reschedule on that core, exactly as a doorbell is
    * allowed to be.
    *
    * Every owner met is prompted, not just the last one. That costs at most one
    * hint per hop and it is what keeps a stale state read from silently dropping
    * the prompt that mattered. Bounded by the same depth budget the fold uses,
    * so a wait-for cycle in the application terminates the walk instead of
    * spinning in it. */
   auto const this_core = cyros_port_get_core_id();
   auto* target = &tcb;

   for (unsigned hop = 0; hop < max_inheritance_depth; ++hop) {
      if (target->pinned_core == this_core) {
         cyros_port_pend_reschedule();
      } else {
         cyros_port_send_reschedule_ipi(target->pinned_core);
      }

      // Deliberately no thread_state check before continuing. blocked_on is
      // published while the waiter is armed, whether or not the scheduler has
      // parked it yet, so a ready or still-running waiter genuinely has
      // somewhere to pass this on. Gating on state == blocked here once
      // reintroduced an unbounded inversion: a waiter preempted between arming
      // and parking is rotated out ready with blocked_on set, it does not
      // re-donate when it resumes, and stopping the walk at it silences the
      // far end of the chain, which is the thread the walk exists to reach.
      auto* const next_resource = target->blocked_on.load(std::memory_order_acquire);
      if (next_resource == nullptr) return;

      auto* const next = base_mutex_access::holder_of(*next_resource);
      if (next == nullptr || next == target) return;

      target = next;
   }
}

/* The caller side of the two-phase block this file opens by describing: one
 * loop for every wait, on w alone or on w until a cancel signal is raised, a
 * template over which, so a wait on w alone compiles with one node and no
 * cancel branch. Friended by waitable, wait_queue and waiter_record.
 *
 * A wake takes the head node whatever its owner is doing, so a node still
 * queued after its owner stopped needing it can have a wake spent on it that
 * was meant for a waiter queued behind. The loop loses no wake by two rules,
 * each one line below:
 *
 *  1. w is polled after its node is off the queue and before the cancel may
 *     end the wait, so a wake of w that chose us is used, or met a barger
 *     first. That is also what makes a transfer to us count: the poll
 *     recognises the ownership, so nothing has to hand it back.
 *  2. A wake of w that took our node after a poll had already satisfied us
 *     is passed on, unless a waiter_record bound to w owns its wakes.
 *
 * The cancel signal needs neither, because it is a level: a wake that lands
 * on its node is owed to no one.
 *
 * LOAD-BEARING: "the disarm found the node gone" means exactly "a wake chose
 * us", which waiter_record::chosen() and rule 2 both rest on, while two facts
 * hold:
 *
 *  a. A node leaves its queue by exactly one of two paths per pass: its owner
 *     disarms it, here, or a wake takes it (wait_queue's wake_one, wake_all,
 *     wake_all_where and wake_one_and_commit, the only unlinks besides
 *     disarm).
 *  b. Each pass disarms each node exactly once, here, after its last armed
 *     poll.
 *
 * Break either and chosen() lies, and a condition variable either loses a
 * notify (a missed mark: its waiter sleeps forever) or invents one (a false
 * mark: it returns as if notified, and a timed wait that timed out reports
 * no_timeout). Disarming a node twice is exactly the false mark. Adding any
 * other way to unlink a node is exactly the missed mark.
 */
class wait_engine
{
   using wait_node = wait_queue::wait_node;

public:
   template<bool Cancellable>
   static bool run(waitable& w, cancel_signal* cancel) noexcept;
};

template<bool Cancellable>
bool wait_engine::run(waitable& w, cancel_signal* const cancel) noexcept
{
   auto& tcb = scheduler_for_this_core().get_current_thread();

   // The caller's record, if it installed one naming w, goes to w's polls,
   // and a record naming anything else reaches none. Bound by identity.
   waiter_record* const record =
      (tcb.wait_record != nullptr && tcb.wait_record->source == &w) ? tcb.wait_record : nullptr;

   wait_node node{ .owner = &tcb, .next = nullptr };
   [[maybe_unused]] wait_node cancel_node{ .owner = &tcb, .next = nullptr };

   while (true) {
      // Armed before the first poll, with a full fence between, so a release
      // that tests for waiters without the lock (waitable::may_have_waiters)
      // either finds us armed or is found by the poll. A wake from here on
      // readies us, and we are no longer 'prepared' to block.
      tcb.disposition = thread_disposition::prepared;
      w.queue.arm(node);
      if constexpr (Cancellable) cancel->queue.arm(cancel_node);
      wait_queue::peek_fence();

      bool const satisfied = w.try_satisfy(record);
      bool cancelled = false;
      if constexpr (Cancellable) cancelled = !satisfied && cancel->raised();

      if (satisfied || cancelled) {
         tcb.disposition = thread_disposition::none;
      } else {
         // Park, unless a wake beat us to it.
         commit_to_block(tcb);
      }

      bool const chosen = !w.queue.disarm(node);
      if constexpr (Cancellable) (void)cancel->queue.disarm(cancel_node);
      if (chosen && record != nullptr) {
         record->chosen_by_wake = true;
      }

      if (satisfied) {
         if (chosen && record == nullptr) {
            // Rule 2. The wake that took our node may have been meant for a
            // waiter queued behind us, which would otherwise stay parked
            // beside what that wake was for. At worst this wakes it for
            // nothing, and its loop parks it again. A record bound to w is
            // left out because it owns the wake instead: it was marked
            // chosen above, as a condition variable learns of its notify,
            // and passing the wake on as well would deliver it twice.
            w.queue.wake_one(reschedule_policy::automatic);
         }
         return true;
      }

      // Rule 1: w first, unarmed. A wake usually leaves what it was for, and
      // taking it here saves arming again. An unarmed
      // poll loses nothing, since if it fails the next pass arms and polls
      // again before it may park. And it is safe for every waitable, because
      // past a wait's first poll a poll may only take what satisfies it
      // (try_satisfy's contract). The one poll that does more, the condition
      // variable's release of its mutex, is a first poll, and the first pass
      // always polls armed.
      if (w.try_satisfy(record)) {
         return true;
      }
      if constexpr (Cancellable) {
         if (cancel->raised()) {
            return false;
         }
      }
   }
}

namespace this_thread
{

CYROS_HOT void wait_on(waitable& w) noexcept
{
   (void)wait_engine::run<false>(w, nullptr);
}

[[nodiscard]] CYROS_HOT bool wait_on(waitable& w, cancel_signal& cancel) noexcept
{
   return wait_engine::run<true>(w, &cancel);
}

} // namespace this_thread

} // namespace cyros
