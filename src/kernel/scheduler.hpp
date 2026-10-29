/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

#ifndef CYROS_SCHEDULER_HPP
#define CYROS_SCHEDULER_HPP

#include "threading_subsystem.hpp"
#include "trace_points.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace cyros
{

/**
 * @brief A return-type to communicate whether the caller
 *        ought to trigger a reschedule
 */
enum class [[nodiscard]] schedule_hint
{
   unwarranted, ///< The action did not ready any better thread that currently running
   warranted,   ///< The action may have readied a better thread that currently running
   rescheduled, ///< The action warranted a reschedule and has made it already
};

void idle_task();

[[nodiscard]] inline std::uint8_t urgency(thread_control_block const& tcb) noexcept; // Defined below the class

/**
 * @brief Per-core state controller
 *
 * Owns its own special thread: 'idle thread' (id: 0)
 */
class scheduler
{
private:
   /**
    * @brief The reasons for a thread 'departing'
    *
    * 'departing' means a thread transitioning from 'running'
    * to any other state. The reason determines where it is next filed.
    */
   enum class departure_reason : std::uint8_t
   {
      yielded,   ///< Pushed to the back of its queue
      preempted, ///< Pushed to the front of its queue (a more urgent thread is ready)
      parked,    ///< Committed to blocking: out of the ready structures until woken
      exited,    ///< Terminated
   };

   std::uint32_t core_id{0};
   std::atomic<std::uint32_t> pinned_thread_counter{0};
   thread_control_block* current_thread{nullptr};
   thread_control_block*    idle_thread{nullptr};
   alignas(CYROS_PORT_STACK_ALIGN) std::array<std::byte, thread::min_stack_size> idle_stack{};

   thread_ready_matrix ready_matrix;

   /* Head of this core's intake stack (MPSC)
    *
    * The TCB is the message: what a producer wants is recorded in
    * thread_control_block::pending_requests, and this list only says WHICH TCBs
    * to look at.*/
   std::atomic<thread_control_block*> intake_head{nullptr};

   /* Threads pinned here that hold at least one base_mutex, i.e. exactly the
    * threads whose urgency can differ from their base priority. Core-local and
    * needs no atomics: see thread_control_block::holder_next.
    *
    * FIFO, appended at the tail, for the same reason every matrix level is:
    * pick_next keeps the FIRST holder at the best urgency, so head insertion
    * would hand the tie to the most recently linked. A holder is unlinked when
    * it is picked and re-linked when it is re-readied, so under head insertion
    * the thread that just ran would go straight back to the front and win
    * again, starving every other holder tied with it. */
   thread_control_block* holders_head{nullptr};
   thread_control_block* holders_tail{nullptr};

   /* Which side won the last urgency tie, one bit per priority level.
    *
    * A boosted holder is not in the matrix, so when its urgency exactly equals
    * the matrix's best base there is no arrival order to compare and the winner
    * has to be chosen by policy. Choosing a side outright starves the other
    * outright: whichever loses is never re-examined, because the losing side's
    * position does not change by being passed over. Alternating bounds the wait
    * for both at two picks per tied group.
    *
    * PER LEVEL rather than a single bool. One shared bit lets ties at two
    * different levels consume each other's turns, so a level whose ties always
    * land on the same phase starves anyway, which is the bug this is meant to
    * fix wearing a different hat.
    *
    * This costs proportionality: the split is even between the two GROUPS, so a
    * lone holder takes half against n matrix threads rather than 1/(n+1). That
    * is deliberate. Bounded delay is the property that matters at equal
    * priority, and the surplus goes to a thread inside a critical section,
    * which shortens it and shrinks the blocking time of whoever is waiting on
    * it. Same direction as the bridge-overflow over-boost in wait_queue::top.*/
   std::uint32_t tie_rotor{0};
   static_assert(config::max_priorities <= std::numeric_limits<decltype(tie_rotor)>::digits,
                 "tie_rotor cannot represent that many priorities!");

   /* File a thread that has just become ready: the matrix at its base
    * priority, or the holder list if it holds something, because a holder's
    * urgency can beat its base and the pick folds it there. The caller holds
    * this core's critical section and the thread is in neither structure.
    *
    * The part of set_thread_ready every admission needs, and all a switch
    * needs to file the thread departing: that thread is running, so it cannot
    * be terminated, enqueued or listed, and nothing reads a hint about it.
    *
    * AT_FRONT files a matrix thread ahead of its equals: a preempted thread.
    * A holder goes on the holder list either way, where equal urgency is
    * settled by tie_rotor.
    *
    * Inline, like set_thread_running, set_thread_blocked and pick_next: every
    * switch runs them, and out of line the asserts in them are enough to
    * stop GCC at -O2 inlining them, when the call costs more than the body. */
    // TODO: Rewrite description and maybe rename
   void admit(thread_control_block& tcb, bool at_front = false)
   {
      tcb.state = thread_state::ready;

      trace::thread_ready(tcb);

      // Idle thread does not belong in the ready_matrix,
      // but DOES follow state transition semantics
      if (&tcb == idle_thread) {
         return;
      }

      // A thread holding a base_mutex can be more urgent than its base priority,
      // so it cannot be filed under a fixed key. It goes on the holder list, which
      // the pick folds urgency over. Everyone else is filed at base_priority, which
      // never changes, so nothing ever re-keys the matrix.
      if (tcb.holds_nothing()) {
         if (at_front) {
            ready_matrix.enqueue_thread_front(tcb);
         } else {
            ready_matrix.enqueue_thread(tcb);
         }
      } else {
         link_holder(tcb);
      }
   }

   /* set_thread_ready's work, under this core's critical section, which the
    * caller holds. */
   schedule_hint make_ready(thread_control_block& tcb)
   {
      // Idempotent: a remote core might have sent us a late request to ready
      // this thread, and we have already terminated it since (or are terminating). No-op.
      if (tcb.state == thread_state::terminated || tcb.disposition == thread_disposition::terminating) {
         return schedule_hint::unwarranted;
      }

      // Idempotent: if already admitted, this is a redundant wake/admit (e.g.
      // two signallers raced, or a stale wake from a prior round). The thread
      // is already going to run.
      if (tcb.is_enqueued() || tcb.is_listed_holder()) {
         CYROS_ASSERT(tcb.state == thread_state::ready);

         // Re-route before returning. Ownership can arrive AFTER admission:
         // hand_over files a handed-over resource into the new owner's slots from
         // the releasing core, and a thread that was already sitting in the ready
         // matrix at that moment is a holder filed where the urgency fold never
         // looks, which is unbounded inversion for whoever parks behind the
         // resource it now owns. Every such handover is followed by a redundant
         // admission of the new owner, which is what makes this the one place the
         // routing can be re-derived without a new message. The reverse migration
         // cannot be needed: only a running thread can release, so a READY
         // holder cannot stop being one.
         if (tcb.is_enqueued() && !tcb.holds_nothing()) {
            ready_matrix.remove_thread(tcb);
            link_holder(tcb);
            if (urgency(tcb) < current_thread_urgency()) {
               return schedule_hint::warranted;
            }
         }
         return schedule_hint::unwarranted;
      }

      admit(tcb);

      // The idle thread's urgency is the worst there is, so it never beats the
      // running thread and needs no case of its own here.
      if (urgency(tcb) < current_thread_urgency()) {
         return schedule_hint::warranted;
      }
      return schedule_hint::unwarranted;
   }

   /* ------------------------------------------------------------------------
    * Switching threads (scheduler.cpp tells it whole)
    *
    * A switch is one decision with two roads into it. The running thread
    * departs for a reason, which files it (depart), the pick becomes the
    * running thread (run_next), and the port switches. On the thread's road
    * (direct_reschedule) the thread itself yields, parks or is preempted by
    * its own wake, at baseline, and knows why. On the handler's road
    * (reschedule) the port calls in at a safe point, and the reason is worked
    * out from what the thread was left as (departure_of).
    * ---------------------------------------------------------------------- */

   /**
    * @brief Thread-initiated reschedule
    *
    * Does not need the handler's road (reschedule()) to work out what thread
    * to swap to, as we already know based on the event/s leading up to this
    * call. Must be called at baseline, with this core's interrupts masked by
    * cyros_port_irq_save, so that no wake on this core can land mid-pick.
    * Returns once the thread is picked to run again (may be immediately, if this is the best thread).
    *
    * @param reason Why the thread is departing (e.g. voluntary yield, waking a better thread, etc...)
    * @param mask Interrupt mask - released by `cyros_port_direct_switch`, or
    *        at once if the thread is picked again
    *
    * @note Forced inline, because when the assert is compiled in, LTO doesn't
    *       otherwise inline this. Inlined, `reason` is a constant in each
    *       caller, so depart's switch folds away.
    */
   [[gnu::always_inline]]
   void direct_reschedule(departure_reason const reason, cyros_mask_token_t const mask)
   {
      CYROS_ASSERT(reason != departure_reason::exited); // Exit takes the handler's road

      if constexpr (config::cores > 1) {
         if (intake_pending()) {
            drain_intake();
         }
      }

      // Unless a remote wake just taken in readied it, which settles its
      // departure as reschedule() settles it: filed already, and a park revoked.
      auto& prev_thread = *current_thread;
      if (prev_thread.state == thread_state::running) {
         depart(prev_thread, reason);
      }

      auto& next_thread = run_next();
      if (&next_thread == &prev_thread) {
         // Picked again: alone at its level after a yield, or readied by a
         // remote wake. It runs on.
         cyros_port_irq_restore(mask);
         return;
      }
      // Saves this thread, resumes NEXT and releases MASK. Returns once this
      // thread is switched back in, by either road.
      cyros_port_direct_switch(prev_thread.context(), next_thread.context(), mask);
   }

   /**
    * @brief File the running thread by why it departs (Switching threads,
    *        scheduler.cpp).
    */
   void depart(thread_control_block& tcb, departure_reason const reason)
   {
      switch (reason) {
         case departure_reason::yielded:   admit(tcb);                   break;
         case departure_reason::preempted: admit(tcb, /*at_front*/ true); break;
         case departure_reason::parked:    set_thread_blocked(tcb);      break;
         case departure_reason::exited:    set_thread_terminated(tcb);   break;
      }
   }

   /**
    * @brief Why the running thread departs, worked out from what it was left as:
    *        the handler's road, which was not there when it happened.
    *
    * Under this core's critical section. Only this core writes a disposition
    * (the thread, an ISR here, or this scheduler), so the read is stable.
    *
    * - terminating            -> exited
    * - none or prepared       -> preempted, when a strictly more urgent thread
    *                             is ready, and yielded otherwise: a time slice
    *                             or a reschedule with nothing more urgent ready
    *                             rotates equals. Prepared is kept either way.
    *
    * A park never reaches here: a commit is made and parked on the thread's road.
    */
   [[nodiscard]] departure_reason departure_of(thread_control_block const& tcb) const
   {
      if (tcb.disposition == thread_disposition::terminating) {
         return departure_reason::exited;
      }
      return preempted(tcb) ? departure_reason::preempted : departure_reason::yielded;
   }

   /**
    * @brief Whether the running thread is being preempted: a strictly more urgent
    *        thread is ready in the matrix.
    *
    * Read before the thread is filed. Only the plain case is answered: with a
    * holder anywhere, urgency is a fold, and the thread keeps the rotation it
    * always had.
    */
   [[nodiscard]] bool preempted(thread_control_block const& tcb) const
   {
      // The bitmap first: on a rotation among equals it is all that runs.
      return ready_matrix.has_more_urgent_than(tcb.base_priority) && holders_head == nullptr && tcb.holds_nothing();
   }

   /**
    * @brief The pick, made the running thread: the idle thread when nothing is
    *        ready. May be the thread that just departed.
    */
   thread_control_block& run_next()
   {
      auto* next = pick_next();
      if (next == nullptr) {
         next = idle_thread;
      }
      set_thread_running(*next);
      return *next;
   }

   /**
    * @brief Push a TCB onto this core's intake stack.
    *
    * @return true when the intake was EMPTY before this push, meaning the
    *         caller owns sending the IPI. That decision comes from the same
    *         atomic that enqueues, so there is no separate "is a drain already
    *         coming" flag to go stale against it.
    *
    * Caller must already hold the claim in tcb.pending_requests, so a TCB is
    * on the stack at most once.
    *
    * @note Runs on the posting core, never this one: global_ready_thread
    *       posts across cores only. Private because post_intake, which takes
    *       the claim, is its one caller.
    */
   [[nodiscard]] bool push_intake(thread_control_block& tcb);

   /**
    * @brief Act on every request bit set for @p tcb.
    *
    * @note Runs on this scheduler's own core only
    */
   void service_intake(thread_control_block& tcb, std::uint8_t bits);

   /**
    * @brief Take the whole intake chain and service every request on it.
    *
    * This is redundant (and should not be called) on single-core configurations.
    * To save an expensive function call + atomic exchange operation, wrap the callsite
    * in a cheaper atomic load that checks if there are actually any requests.
    * E.g.
    * @code
    * if constexpr (config::cores > 1) {
    *    if (intake_pending()) {
    *       drain_intake();
    *    }
    * }
    * @endcode
    *
    * The relaxed load is enough. If it sees the intake empty, a push still in
    * flight finds it empty too, so its poster sends this core a reschedule,
    * and that drains it.
    *
    * @note Runs on this scheduler's own core only
    */
   void drain_intake();

public:
   static constexpr thread::id idle_thread_id = 0; // Reserved

   constexpr scheduler() noexcept = default;

   ~scheduler() = default;
   scheduler(scheduler&&) = delete;
   scheduler(scheduler const&) = delete;
   scheduler& operator=(scheduler&&) = delete;
   scheduler& operator=(scheduler const&) = delete;

   [[nodiscard]] constexpr thread::id current_thread_id() const
   {
      return current_thread ? current_thread->id : 0;
   }

   /**
    * @brief Urgency of the running thread, folded from truth.
    */
   [[nodiscard]] std::uint8_t current_thread_urgency() const;

   /**
    * @brief Threads pinned here. Used by pin_thread_to_core to load balance.
    */
   [[nodiscard]] std::uint32_t pinned_thread_count() const
   {
      return pinned_thread_counter.load(std::memory_order_relaxed);
   }

   [[nodiscard]] constexpr thread_control_block& get_current_thread() const
   {
      CYROS_ASSERT(current_thread != nullptr); // Not invocable from non-thread context

      return *current_thread;
   }

   void pin_thread(thread_control_block& tcb);

   /**
    * @brief Bind this scheduler to its core and create the core's idle thread.
    *        kernel::initialise() calls it once per core, and reset() undoes it.
    */
   void initialise(std::uint32_t core);

   void start();

   /**
    * @brief Ready a thread pinned here, and say whether a reschedule is now
    *        warranted.
    *
    * A wake whose POLICY allows a reschedule, made by the running thread at
    * baseline, takes the reschedule it warrants itself, on the thread's road,
    * in the critical section that readied the thread, and returns
    * schedule_hint::rescheduled once the waker runs again. Everything that
    * must not switch (reschedule() taking in the intake, a batch of wakes
    * that pends once after) uses the default, which never does.
    */
   schedule_hint set_thread_ready(thread_control_block& tcb, reschedule_policy policy = reschedule_policy::never);

   void set_thread_running(thread_control_block& tcb)
   {
      CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);
      CYROS_ASSERT_OP(tcb.state, ==, thread_state::ready);

      tcb.state = thread_state::running;
      current_thread = &tcb;

      trace::thread_running(tcb);
   }

   void set_thread_blocked(thread_control_block& tcb)
   {
      CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);
      CYROS_ASSERT_OP(tcb.state, ==, thread_state::running);
      CYROS_ASSERT_OP(tcb.disposition, ==, thread_disposition::prepared);
      CYROS_ASSERT(!tcb.is_enqueued());

      tcb.disposition = thread_disposition::none;
      tcb.state = thread_state::blocked;

      trace::thread_blocked(tcb);
   }

   void set_thread_terminated(thread_control_block& tcb);

   /**
    * @brief Choose the most urgent runnable thread on this core.
    *
    * Best of the matrix head (most urgent non-holder, by base priority) and the
    * most urgent holder, whose urgency is folded from truth rather than read
    * from a cache. Returns nullptr when nothing is runnable.
    */
   thread_control_block* pick_next()
   {
      // Holders are the only threads whose urgency can beat their base priority,
      // and they are rare: measured at about 1 percent of picks in the one test
      // that exercises mutexes, and zero everywhere else. With none, the matrix's
      // head is the pick. The fold over them is out of line, so the switch path
      // does not carry its registers (performance-investigation.md 10).
      if (holders_head == nullptr) [[likely]] {
         return ready_matrix.pop_best_thread();
      }
      return pick_with_holders();
   }
   thread_control_block* pick_with_holders();

   /**
    * @brief Ask this core to service @p type for @p tcb. Any core may call.
    *
    * Claims, enqueues and notifies. There is no failure mode: the TCB is the
    * message and it always has room for itself, so nothing can overflow and
    * there is no full-queue case needing a correct response.
    */
   void post_intake(thread_control_block& tcb, thread_request request);

   /**
    * @brief Append a ready thread that holds something to the holder list.
    *
    * Called from set_thread_ready, not from acquisition: a thread that takes a
    * resource is running, so it joins the list at its next re-ready. Idempotent.
    */
   void link_holder(thread_control_block& tcb);

   /**
    * @brief Remove a holder from the list because the pick chose it.
    *
    * The counterpart to link_holder, and the only way out of the list. A thread
    * that releases its last resource is running, so it is already off it.
    */
   void unlink_holder(thread_control_block& tcb);

   /**
    * @brief True when this core has intake work outstanding. Advisory.
    *
    * Cheap enough for the idle loop to check before sleeping, which is what
    * keeps a lost IPI to a scheduling round instead of a hang.
    */
   [[nodiscard]] bool intake_pending() const
   {
      // Relaxed: this only decides whether to look, never what is true.
      return intake_head.load(std::memory_order_relaxed) != nullptr;
   }

   /**
    * @brief this_thread::yield: to the back of its level, and the pick runs.
    *
    * The thread's road. Called by the running thread at baseline.
    */
   void yield();

   /**
    * @brief Park the running thread, which is prepared to block, and run the
    *        pick.
    *
    * The thread's road, called by commit_to_block inside the critical section
    * in which it found the thread still prepared, whose token is MASK, so no
    * wake on this core can land between that look and the park. The switch
    * releases MASK, and this returns once the thread has been woken and runs
    * again.
    */
   void park(cyros_mask_token_t mask);

   /**
    * @brief The handler's road: called by the port at a safe point, once a
    *        reschedule was pended or the port's yield made.
    *
    * Works out why the running thread departs from what it was left as, files
    * it, and switches to the pick. See the .cpp.
    */
   void reschedule();

   void reset();
};

/**
 * @brief The scheduler for the calling core.
 *
 * DEFINED IN kernel.cpp, not scheduler.cpp. The scheduler array is a member of
 * the kernel_state singleton, which lives in kernel.cpp's anonymous namespace so
 * that no other translation unit can reach the instance. This is the declared
 * door to it, and the only one: reaching ANOTHER core's scheduler stays private
 * to kernel.cpp behind global_ready_thread, because that is where
 * the local-versus-remote decision and the cross-core intake protocol live.
 *
 * Safe by construction in a way scheduler_for_core(id) is not. A core can only
 * ever obtain its own, and every scheduler method that mutates asserts its
 * pinned-core precondition anyway.
 *
 * [[gnu::pure]] is sound only because threads never migrate cores, so a core's
 * scheduler is a fixed answer for the life of the call. If migration is ever
 * implemented, revisit this attribute before anything else.
 */
[[nodiscard, gnu::pure]] scheduler& scheduler_for_this_core();

/**
 * @brief Make a thread runnable on its pinned core (cross-core-safe).
 *
 * Transitions @p tcb to the ready state and enqueues it on the ready queue of
 * its pinned core. If the thread belongs to another core, a cross-core request
 * is posted so that the owning scheduler performs the enqueue.
 *
 * The cross-core sibling of scheduler::set_thread_ready, and the reason
 * scheduler_for_core(id) stays private: this is where the local-versus-remote
 * decision and the intake protocol live, so nothing else needs to reach a
 * remote scheduler. DEFINED IN kernel.cpp for that access.
 *
 * @param tcb thread control block of the thread to make runnable.
 * @param policy the waker's: anything but never lets a wake on this core take
 *        the reschedule it warrants itself (scheduler::set_thread_ready).
 * @return schedule_hint::warranted if the thread was queued on the current core
 *         and has higher priority than the running thread, indicating the caller
 *         should request a local reschedule, and schedule_hint::rescheduled
 *         if that reschedule has been made already.
 */
schedule_hint global_ready_thread(thread_control_block& tcb, reschedule_policy policy);

/* ---------------------------------------------------------------------------
 * A thread's relationship to scheduling: what it is worth, when it parks, and
 * when a core should think again. Free functions rather than scheduler methods
 * because none of them is about ONE core's state, and all are DEFINED IN
 * waitable.cpp alongside the queue machinery that drives them, but for
 * urgency's fast path, inline below.
 * ------------------------------------------------------------------------- */

/* How far a transitive donation is followed before the answer is taken to be
 * maximally urgent.
 *
 * Real chains are short: the deepest the suite builds is 6, and the flagship PI
 * test is 2. The budget is not a performance knob, it is what stops a wait-for
 * cycle (a deadlocked application) from being walked forever. Exhausting it
 * yields 0, which over-boosts, which is the safe direction. */
inline constexpr unsigned max_inheritance_depth = 8;

/**
 * @brief urgency() with an explicit recursion budget.
 *
 * A boosted holder can itself be blocked on something it does not own, whose
 * queue contains another holder, and so on: that chain is transitive priority
 * inheritance. Following it needs a bound, because a wait-for CYCLE is a
 * deadlocked application and the kernel must not walk one forever.
 *
 * CALLER MUST NOT HOLD A QUEUE LOCK. The recursion reaches wait_queue::top,
 * which takes one to snapshot the bridges parked on that queue. It releases it
 * before recursing, so queue locks are never NESTED and a wait-for cycle is
 * merely observed rather than deadlocked on, but that only holds while every
 * caller enters from outside. In particular nothing inside a wake_one_and_commit
 * callback may ask for a thread's urgency, and thread::get_priority() is exactly
 * that question.
 *
 * held_slots is a bounded array of atomics so the fold itself needs no lock, and
 * a torn read yields a stale-but-plausible urgency, the same tolerance the rest
 * of this design rests on.
 */
[[nodiscard]] std::uint8_t urgency_at(thread_control_block const& tcb, unsigned depth) noexcept;

/**
 * @brief A thread's urgency, computed from truth: min(base, best waiter of
 *        every base_mutex it holds).
 *
 * This is the DEFINITION of priority inheritance, evaluated rather than read.
 * There is no stored effective priority and no walk maintaining one: a cache
 * whose inputs live on other cores is what every priority-inheritance bug in
 * this project's history has been.
 *
 * Callable by ANY core with no lock held. That is not incidental: held_slots is
 * a bounded array of atomics rather than an intrusive list precisely so a
 * remote core can read what a thread holds without taking its pi_lock, and
 * queue tops are maintained atomics for the same reason. A racing release
 * yields a stale-but-plausible answer, which is the tolerance the value-free
 * doorbell already rests on.
 *
 * Cost: one load and a compare when the thread holds nothing, which is 98.9
 * percent of picks in the only test that exercises mutexes at all and 100
 * percent everywhere else, and inline for that reason: every wake asks twice.
 * Otherwise one iteration per resource actually held, measured at 1.01
 * resources per holder.
 */
[[nodiscard]] inline std::uint8_t urgency(thread_control_block const& tcb) noexcept
{
   if (tcb.held_mask.load(std::memory_order_acquire) == 0) {
      return tcb.base_priority;
   }
   return urgency_at(tcb, max_inheritance_depth);
}

/**
 * @brief Commit to parking, unless a wake has already revoked the intent.
 *
 * The subtle half of the two-phase block, and the ONLY part of it that is worth
 * sharing between the wait engine and the primitives with blocking loops of
 * their own (base_mutex::lock, this_thread::suspend). The arming loops differ
 * enough that a shared abstraction would cost more readability than it saves,
 * but this test-and-park must be identical everywhere or the lost-wakeup window
 * reopens.
 *
 * The test and the park run with this core's interrupts masked, so a wake
 * cannot land between deciding to block and parking. A wake that arrived
 * earlier has already cleared the disposition, and that is what this re-reads:
 * seeing anything other than prepared means someone readied us after we armed,
 * so we must not park. Still prepared, it parks (scheduler::park), and returns
 * once the thread runs again, never before the switch: the caller disarms
 * next, and a thread that disarmed before it parked would park armed on
 * nothing.
 *
 * Caller must be armed on every source it intends to wake from BEFORE calling,
 * and at baseline, as for any yield.
 */
void commit_to_block(thread_control_block& tcb);

/**
 * @brief Ask @p tcb's core to reconsider what it is running.
 *
 * A pure hint carrying nothing. Used when a thread's urgency has RISEN because a
 * waiter arrived on something it holds: the new urgency is already visible in
 * that resource's queue top, but the owning core will not notice until it picks
 * again. Losing this costs latency, never correctness, because the next pick for
 * any reason folds the same truth.
 *
 * Only boosts need it. A de-boost that is observed late merely means the holder
 * ran at its old urgency slightly longer, which is safe.
 */
void request_repick(thread_control_block& tcb);

} // namespace cyros

#endif // CYROS_SCHEDULER_HPP