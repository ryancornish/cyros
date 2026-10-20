/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

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
 * A wait_node has exactly one 'next' pointer, so it cannot be in two intrusive
 * lists at once, which is why waiting on several sources needs one node per
 * source. The nodes live in a wait_node_vector on the blocking thread's own
 * stack, so there is no heap and no pool. Every wait goes through wait_on_any,
 * including a wait on a single source.
 */

#include <cyros/kernel/waitable.hpp>
#include <cyros/kernel/assert.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/port/port.h>

#include "base_mutex_access.hpp"
#include "scheduler.hpp"
#include "waitable_utilities.hpp"
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
   if (policy == reschedule_policy::never) {
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
 * waitable_arm_guard relies on exactly that, see the note there.
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

   schedule_hint hint = global_ready_thread(*chosen);
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
      schedule_hint hint = global_ready_thread(*chosen);
      if (hint == schedule_hint::warranted) {
         aggregate_hint = schedule_hint::warranted;
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
         if (global_ready_thread(*chosen[i]) == schedule_hint::warranted) {
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
   schedule_hint hint = global_ready_thread(*chosen);
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
   // The record each waiter's poll gets, bound by identity as wait_on_any
   // binds it. A queued waiter is inside its wait, so the record it installed
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

void waitable::renounce_if_assigned(thread::id) noexcept
{
   // Nothing to hand back: an ownerless waitable assigns nobody anything.
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
   // Hazard: the thread could read 'prepared', an ISR could clear the
   // disposition, and the thread would blindly overwrite it to 'committed'.
   // Masking interrupts makes the test and the store one step with no
   // read-modify-write, because every write to a disposition is made on its
   // thread's own core: by the thread, by an ISR there, or by its scheduler. A
   // remote wake goes through the intake and is applied here, in a reschedule.
   bool committed = false;
   {
      this_core::critical_guard guard;
      if (tcb.disposition == thread_disposition::prepared) {
         tcb.disposition = thread_disposition::committed;
         committed = true;
      }
   }

   // A wake that lands before the yield clears the disposition and finds us
   // running, and the reschedule then rotates us instead of parking us, as it
   // does for any wake that beats the park.
   if (committed) {
      cyros_port_thread_yield();
   }
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

/* The caller side of the two-phase block this file opens by describing. Lives
 * here for its access: it is friended by both waitable and wait_queue, and it
 * drives wait_node_vector and waitable_arm_guard. */
namespace this_thread
{

[[nodiscard]] CYROS_HOT std::size_t wait_on_any(std::span<waitable_ref> waitables) noexcept
{
   CYROS_REQUIRE(!waitables.empty()); // Nothing to wait on
   CYROS_REQUIRE_OP(waitables.size(), <=, config::max_wait_nodes); // More than config::max_wait_nodes

   auto& tcb = scheduler_for_this_core().get_current_thread();

   // Indeterminate, so GCC does not zero it first. Every wait would otherwise
   // pay a memset of all config::max_wait_nodes slots (68 bytes at 8) to
   // protect bytes nothing reads: the vector only ever touches the nodes it
   // constructs, which are exactly the ones in use.
   wait_node_vector nodes [[indeterminate]] (waitables.size(), tcb);

   // The caller's record, if it installed one, goes to the one waitable it
   // names and to no other source. Bound by identity, not by position.
   waiter_record* const record = tcb.wait_record;
   auto const record_for = [record](waitable const& w) noexcept -> waiter_record* {
      return (record != nullptr && record->source == &w) ? record : nullptr;
   };

   // Lowest index wins on ties.
   auto const first_satisfied = [&]() noexcept -> std::optional<std::size_t> {
      for (std::size_t i = 0; waitable& waitable : waitables) {
         if (waitable.try_satisfy(record_for(waitable))) {
            return i;
         }
         ++i;
      }
      return std::nullopt;
   };

   std::optional<std::size_t> chosen;
   while (!chosen) {
      tcb.disposition = thread_disposition::prepared;

      {
         // All waitable wakes are serialised on the arm_guard.
         // If a wake fires BEFORE the arm_guard:
         // - Thread is not readied because we are not registered.
         waitable_arm_guard arm_guard(waitables, nodes, record);
         // If a wake fires AFTER the arm_guard:
         // - Thread is readied and we are no longer 'prepared' to block.

         chosen = first_satisfied();
         if (chosen) {
            tcb.disposition = thread_disposition::none;

            // Leave the queue we just acquired from IMMEDIATELY, rather than
            // at the guard below. An owner still armed in its own resource's
            // queue is a cycle in the wait-for graph: the fold reads that
            // queue, finds the owner listed as a bridge, and recurses back
            // into the owner until the depth budget runs out and answers 0.
            // That is not the harmless over-boost it looks like. A thread
            // folding to 0 can never be beaten by set_thread_ready's
            // urgency < current comparison, so its core stops raising
            // preemption hints entirely while the window is open.
            //
            // Safe to do early: we own it, so no release can transfer it to
            // us in the meantime, which is the only thing the other nodes
            // have to stay armed for.
            arm_guard.leave(*chosen);
         } else {
            // Nothing was satisfied. Park, unless a wake beat us to it.
            commit_to_block(tcb);
         }
      } // arm_guard: disarm every node not already left, noting which a wake took

      if (!chosen) {
         // Readied, by a wake or by losing the commit to one. Poll before
         // arming again: a wake usually leaves what it was for, and taking it
         // here saves arming and disarming every queue a second time. An
         // unarmed poll loses nothing, since if every poll fails the next pass
         // arms and polls again before it may park. And it is safe for every
         // waitable, because past a wait's first poll a poll may only take what
         // satisfies it (try_satisfy's contract). The one poll that does more,
         // the condition variable's release of its mutex, is a first poll, and
         // the first pass always polls armed.
         chosen = first_satisfied();
      }
   }

   // The sweep must run here, AFTER the disarm above: while any node of ours
   // was still armed, a racing release could commit a transfer to us. With
   // every node off every queue no further transfer can choose us, so the set
   // of in-flight assignments is frozen and the ones we are not returning are
   // handed back. The contract this enforces: on return the caller owns
   // exactly waitables[chosen], plus whatever it already owned before the call.
   for (std::size_t i = 0; waitable& waitable : waitables) {
      if (i != chosen) {
         waitable.renounce_if_assigned(tcb.id);
      }
      ++i;
   }
   return *chosen;
}

} // namespace this_thread

} // namespace cyros
