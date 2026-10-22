/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

#ifndef CYROS_WAITABLE_HPP
#define CYROS_WAITABLE_HPP

#include <cyros/config/config.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/spinlock.hpp>
#include <cyros/kernel/attribute.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>

namespace cyros
{

/**
 * @brief Caller's instruction for whether signalling should trigger a
 *        reschedule on the *local* core.
 */
enum class reschedule_policy
{
   automatic, ///< Conditionally trigger a local reschedule if a better thread is made ready (preferred)
   never,     ///< Never trigger a local reschedule
   always,    ///< Always trigger a local reschedule
};

/**
 * @brief A waiter's per-wait record, for a waitable whose answer depends on
 *        WHICH waiter is asking.
 *
 * A primitive derives its waiter record from this (the mutex a condition
 * variable must release, the bits an event-flags waiter wants) and builds it
 * on the waiting thread's stack, naming the waitable it belongs to. Building
 * it installs it on the calling thread, and destroying it uninstalls it. While
 * installed, any ordinary wait_on / wait_on_any that includes that waitable
 * hands the record to that waitable's polls (and to no other source's), and
 * records on it whether a wake of that waitable chose this waiter.
 *
 *    waiter self(*this, m);         // a waiter_record naming this waitable
 *    this_thread::wait_on(*this);   // an ordinary wait
 *
 * One record per thread at a time, since a thread is in one wait at a time,
 * which is checked. Build it on the waiting thread, in thread context, around
 * the wait.
 */
class CYROS_PUBLIC waiter_record
{
public:
   explicit waiter_record(waitable& source) noexcept;
   ~waiter_record();

   waiter_record(waiter_record&&) = delete;
   waiter_record(waiter_record const&) = delete;
   waiter_record& operator=(waiter_record&&) = delete;
   waiter_record& operator=(waiter_record const&) = delete;

   /**
    * @brief True once a wake of the source waitable has chosen this waiter.
    *        Sticky for the life of the record.
    *
    * The waker records nothing. A wake takes the waiter's node off the queue
    * and readies it, as every wake always has, and the WAITER discovers it
    * was chosen when its own disarm finds that node already gone
    * (waitable_arm_guard, where the rule this rests on is written down).
    * Written and read only by the waiting thread, so it needs no atomic.
    *
    * Discovered at the end of the pass the wake landed in, so the poll in
    * that same pass may still read false. The waiter was readied, so it does
    * not park, and the next pass's poll reads true.
    */
   [[nodiscard]] bool chosen() const noexcept { return chosen_by_wake; }

private:
   waitable const*       source{nullptr};
   thread_control_block* installed_on{nullptr};
   bool                  chosen_by_wake{false};

   friend class waitable;
   template<std::size_t> friend class waitable_arm_guard;
   friend class wait_engine;
};

/**
 * @brief The priority-inheritance state of a base_mutex's wait_queue.
 *
 * Owned by the base_mutex and passed through the wait_queue API to be
 * consumend by wait_queue::top().
 *
 * Owns a 1-1 relationship of priority-inheritance details with a corresponding
 * wait_queue.
 *
 * @note Private implementation detail of the kernel. Not a user-facing type!
 */
class CYROS_PUBLIC inheritance_cache
{
   static constexpr std::uint8_t no_waiter = 0xFF;

   /**
    * @brief Minimum base-priority of all waiters in the wait_queue
    *
    * Base-priorities are an immutable property of the thread, so this cache
    * won't go stale.
    */
   std::atomic<std::uint8_t> top_priority{no_waiter};

   /**
    * @brief A 'sticky' hint that indicates there _may_ be a waiter in wait_queue
    *        that itself holds another base_mutex (making it a bridge)
    *
    * Sticky in the sense that it is set when a holder arms on the wait_queue,
    * and cleared only when the queue completely empties of waiters.
    *
    * If this is **false**, then there are certainly no bridges in the queue and
    * we need not look (fast path).
    * If this is **true**, then there _may_ be one or more bridges in the queue and
    * we walk the queue to find them.
    *
    * The hint is thus an optimisation that errs only toward 'looking-for', never toward 'missing-a'
    */
   std::atomic<bool> may_have_bridges{false};

   friend class wait_queue;
};

/**
 * @brief Intrusive priority-ordered list of waiting threads.
 *
 * Owns park-list mechanics and the commit-under-lock discipline that makes a
 * barge-free handoff possible.
 *
 * @note Private implementation detail of the kernel. Not a user-facing type!
 */
class CYROS_PUBLIC wait_queue
{
   /**
    * @brief Per-thread parking record (one per TCB, reused).
    *
    * Threaded into one or more queues while the thread is parked.
    */
   struct wait_node
   {
      thread_control_block* owner{nullptr};
      wait_node*            next {nullptr};
   };

   using transfer_fn = std::function_ref<void(thread::id)>;
   using commit_fn   = std::function_ref<void(thread_control_block*)>;
   using pick_fn     = std::function_ref<bool(thread_control_block const&)>;
   constexpr wait_queue() noexcept = default;

   wait_queue(wait_queue&&)                 = delete;
   wait_queue(wait_queue const&)            = delete;
   wait_queue& operator=(wait_queue&&)      = delete;
   wait_queue& operator=(wait_queue const&) = delete;

   /**
    * @brief Add/remove a waiter onto the queue
    *
    * Generic waitable variant - no inheritance cache
    */
   void arm   (wait_node& node) noexcept { arm(node, nullptr); }
   bool disarm(wait_node& node) noexcept { return disarm(node, nullptr); }

   void wake_one(reschedule_policy policy) noexcept { wake_one(policy, nullptr); }
   void wake_all(reschedule_policy policy) noexcept { wake_all(policy, nullptr); }
   bool wake_one_and_transfer(transfer_fn transfer, reschedule_policy policy) noexcept
   {
      return wake_one_and_transfer(transfer, policy, nullptr);
   }

   /**
    * @brief Add/remove a waiter onto the queue
    *
    * base_mutex variant - permits an inheritance cache
    */
   void arm   (wait_node& node, inheritance_cache* pi) noexcept;
   bool disarm(wait_node& node, inheritance_cache* pi) noexcept;

   void wake_one(reschedule_policy policy, inheritance_cache* pi) noexcept;
   void wake_all(reschedule_policy policy, inheritance_cache* pi) noexcept;
   bool wake_one_and_transfer(transfer_fn transfer, reschedule_policy policy, inheritance_cache* pi) noexcept;
   bool wake_one_and_commit(commit_fn commit, reschedule_policy policy, inheritance_cache* pi) noexcept;

   /// wake_all, but only the waiters @p pick accepts, asked under the lock
   /// about each queued waiter's thread. Waitable queues only, so no cache.
   void wake_all_where(pick_fn pick, reschedule_policy policy) noexcept;

   [[nodiscard]] bool empty() const noexcept
   {
      // Advisory: not under lock. Fine for "should I bother waking" hints, and
      // see waitable::may_have_waiters for the one order in which acting on a
      // false answer is safe.
      return head.load(std::memory_order_relaxed) == nullptr;
   }

   /**
    * @brief Best waiter urgency on this queue, which is what its holder inherits.
    *
    * Fast path is one atomic load. top_priority caches the minimum BASE priority
    * of the waiters, and base never changes, so that cache has only immutable
    * inputs and can never go stale. No re-slotting, ever.
    *
    * The slow path exists because a waiter that itself HOLDS something can be
    * more urgent than its base: that is the transitive chain. Only such a waiter
    * can be, so the fold runs over the bridges alone rather than the whole queue,
    * and may_have_bridges is false for the overwhelming majority of queues.
    *
    * @param resource_owner The owner word of the resource this queue backs.
    *        Read UNDER the queue lock and excluded from the bridge fold: an
    *        acquirer is briefly both owner and armed waiter in its own queue
    *        (the window between the winning CAS and its disarm), and recursing
    *        into it from its own queue is a self-cycle that exhausts the depth
    *        budget and answers 0. Skipping the owner makes that cycle
    *        unrepresentable at every recursion depth rather than merely brief.
    *
    *        The read must happen inside the lock, not be passed in as a value.
    *        A handover commits the owner under this same lock after unlinking
    *        the chosen waiter, so an under-lock read is exactly consistent
    *        with the queue contents: a thread observed both armed and owning
    *        is the acquire-window self-cycle and nothing else. A value read
    *        before the lock can go stale against exactly that handover, and
    *        the ex-owner can re-arm as a legitimate bridge behind it, which
    *        the stale exclusion would then skip, an UNDER-report.
    *
    *        The lock-free CAS take is the one owner transition outside this
    *        lock, and racing it is UNREACHABLE rather than merely benign. A
    *        fold arrives at a queue only through
    *        base_mutex::urgency_contribution, whose sole caller walks a thread's
    *        held_slots, and a mutex is filed there only by claim_slot, called
    *        either after the CAS that made the
    *        owner or from the handover commit under this lock. So anything able
    *        to consult this queue already observes the owner word set. Stated
    *        as unreachable on purpose: calling it "the safe over-boost
    *        direction" would repeat the mistake this parameter exists to fix,
    *        since a fabricated 0 is unsafe for a RUNNING thread and the
    *        acquirer here is running. Principle 8.
    * @param depth Remaining recursion budget. A wait-for cycle means the
    *        application has deadlocked; the budget stops the kernel following it
    *        forever and the answer is then conservative rather than wrong.
    */
   [[nodiscard]] std::uint8_t top(std::atomic<thread_control_block*> const& resource_owner,
                                  unsigned depth,
                                  inheritance_cache const& pi) const noexcept;

   void link  (wait_node&) noexcept;
   bool unlink(wait_node&) noexcept;
   inline void refresh_top(inheritance_cache* pi) noexcept; // inline: every queue operation calls it, and only waitable.cpp does

   /* The full fence each side of a lock-free emptiness test needs: between a
    * waiter arming and its first poll, and between a releaser changing what
    * the waiters poll and its test (waitable::may_have_waiters). On one core
    * a thread and the interrupts that preempt it already see each other's
    * accesses in program order, so only the compiler must be held back. */
   static void peek_fence() noexcept
   {
      if constexpr (config::cores > 1) {
#if defined(__SANITIZE_THREAD__)
         // ThreadSanitizer refuses standalone fences. Read-modify-writes of one
         // shared word order the two sides the same way, since whichever comes
         // second in that word's order synchronises with the first.
         peek_fence_word.fetch_add(0, std::memory_order_acq_rel);
#else
         std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
      } else {
         std::atomic_signal_fence(std::memory_order_seq_cst);
      }
   }
#if defined(__SANITIZE_THREAD__)
   static inline constinit std::atomic<unsigned> peek_fence_word{0};
#endif

   spinlock lock;

   /* Base-priority-ordered, best at head. Atomic only so that empty() may read
    * it without the lock. Every other access holds the lock and is relaxed,
    * which compiles to the plain loads and stores it would be anyway. */
   std::atomic<wait_node*> head{nullptr};

   // The only types that may drive a queue.
   friend class waitable;
   friend class base_mutex;
   friend class pi_wait_queue;
   template<std::size_t> friend class waitable_arm_guard;
   friend class wait_node_vector;
   friend class wait_engine;
};

/**
 * @brief A wait queue owned by a priority-inheritance resource.
 *
 * Nothing but composition: the queue that parks the waiters, plus the
 * inheritance state only base_mutex ever reads. Each forwarder supplies the
 * cache, so a resource holding one of these cannot forget its own bookkeeping,
 * and a plain waitable holding a bare wait_queue cannot accidentally pay for it.
 *
 * @note Private implementation detail of the kernel. Not a user-facing type!
 */
class CYROS_PUBLIC pi_wait_queue
{
   using wait_node = wait_queue::wait_node;
   using commit_fn = wait_queue::commit_fn;

   void arm   (wait_node& node) noexcept { queue.arm(node, &pi); }
   void disarm(wait_node& node) noexcept { (void)queue.disarm(node, &pi); }

   [[nodiscard]] bool empty() const noexcept { return queue.empty(); }

   bool wake_one_and_commit(commit_fn commit, reschedule_policy policy) noexcept
   {
      return queue.wake_one_and_commit(commit, policy, &pi);
   }

   [[nodiscard]] std::uint8_t top(std::atomic<thread_control_block*> const& resource_owner,
                                  unsigned depth) const noexcept
   {
      return queue.top(resource_owner, depth, pi);
   }

   wait_queue        queue;
   inheritance_cache pi;

   friend class base_mutex;
};


/* ============================================================================
 * waitable - kernel base class for blockable objects
 * ----------------------------------------------------------------------------
 * Inherit from waitable to make your object something threads can block ON
 * (semaphore, event, thread-termination, a future alarm). The base owns a
 * private wait_queue and exposes to the derived class:
 *
 *   try_satisfy(record)     virtual. Attempt to satisfy the caller without
 *                           blocking, consuming the resource if that is what
 *                           satisfaction means. wait_on_any polls it between
 *                           arm and park, which is the two-phase block that
 *                           closes the lost-wakeup window. Runs in the calling
 *                           thread's context, with the caller's waiter_record
 *                           for this waitable, or null.
 *   wake_one / wake_all     signal waiters. ISR-safe.
 *   wake_all_where          wake only the waiters a filter accepts. ISR-safe.
 *   wake_one_and_transfer   barge-free handoff, see the protocol note below.
 *
 * Threads wait via this_thread::wait_on(w) or wait_on_any(w0, w1, ...), never
 * through methods on the waitable itself. Use composition instead of
 * inheritance when your object merely uses blocking internally without being a
 * wait target.
 *
 * Barging, and the transfer protocol
 * ----------------------------------
 * waitable is BARGE-PERMITTING by default: a wake does not reserve anything
 * for the woken thread, so a fresh caller may take the resource first and the
 * woken thread re-parks. That is cheap and right for counting semaphores,
 * events and joins.
 *
 * Barge-free handoff, where a release commits the resource to the chosen
 * waiter under the queue lock so nothing can interpose, is available through
 * wake_one_and_transfer plus a try_satisfy that recognises ownership by id,
 * plus renounce_if_assigned for the group-wait interaction. This CANNOT be
 * built outside the kernel from wake_one: choosing the best waiter and
 * committing to it must happen under the queue lock, which nothing else can
 * reach. That is why the surface exists here.
 *
 * STATUS: no in-tree primitive currently derives a transfer-shaped waitable.
 * The kernel mutex needs barge-free handoff WITH priority inheritance and gets
 * both from base_mutex, which composes the same wait_queue directly and is
 * deliberately not a waitable. This surface is the path for a barge-free
 * primitive WITHOUT inheritance (a strict-fairness semaphore, a mailbox that
 * hands its datum to exactly one waiter). test_transfer_waitables is its
 * conformance test and is what keeps the dual-satisfaction contract honest.
 *
 * Per-waiter state
 * ----------------
 * Most waitables answer the same for every waiter. One that does not (a
 * condition variable must release the caller's mutex, event flags test the
 * caller's own mask) has its waiters build a waiter_record naming it before an
 * ordinary wait, from its own member functions. Users never see the record,
 * so such a type usually inherits waitable privately: a user's plain wait_on
 * would poll it with a null record.
 *
 * The record also answers waiter_record::chosen(): did a wake of this waitable
 * pick this waiter? A wake still carries nothing and writes nothing into the
 * waiter. The waiter works it out from its own wait node, which a wake takes
 * off the queue. That is what a primitive needs when the wake is itself the
 * event, as for a condition variable, whose notify_one is a plain wake_one.
 *
 * Spurious wakeups
 * ----------------
 * A woken thread is not guaranteed its condition holds: the resource may have
 * been consumed before it runs. wait_on_any re-checks and re-parks
 * transparently, the same mandatory loop as around pthread_cond_wait.
 * ========================================================================= */

class CYROS_PUBLIC waitable
{
public:
   waitable(waitable&&) = delete;
   waitable(waitable const&) = delete;
   waitable& operator=(waitable&&) = delete;
   waitable& operator=(waitable const&) = delete;

protected:
   using transfer_fn = wait_queue::transfer_fn;

   waitable() noexcept = default;
   ~waitable();

   /**
    * @brief Attempt to satisfy the calling thread without blocking.
    *
    * Consume the resource if that is what satisfaction means (a semaphore
    * decrements, a transfer-shaped primitive recognises ownership by
    * this_thread::id()). Runs in the calling thread's context with no queue
    * lock held.
    *
    * A wait's FIRST poll runs with the caller armed on this waitable, so it
    * may do something a wake must not miss, as the condition variable's first
    * poll releases its mutex. A later poll may run unarmed, after a wake and
    * before the caller arms again, so past the first a poll may do nothing
    * but take what satisfies it.
    *
    * @param record The caller's waiter_record if it installed one naming this
    *        waitable, otherwise null. A waitable that needs none ignores it.
    * @return true when the caller is satisfied and will not block.
    */
   virtual bool try_satisfy(waiter_record* record) noexcept = 0;

   /**
    * @brief Wake the single highest-priority waiting thread (if any).
    *
    * @param policy After waking a waiting thread, apply this reschedule policy.
    *
    * No-op if no waiters. Safe within ISR context.
    * Woken thread may be barged by another thread. This reduces latency
    * at the cost of waiter fairness.
    */
   void wake_one(reschedule_policy policy = reschedule_policy::automatic) noexcept;

   /**
    * @brief Whether anything may be waiting here. Lock-free.
    *
    * For a release that would otherwise take the queue's lock only to find it
    * empty. False is safe to act on in ONE order: change the state the waiters
    * poll FIRST, then ask. This fences before it looks, and a waiter fences
    * between arming and its first poll (wait_on_any), so one side always sees
    * the other: the release finds the waiter armed, or the waiter's poll finds
    * the release. True may be stale, and a wake then finds nobody, as it
    * always could.
    */
   [[nodiscard]] bool may_have_waiters() const noexcept
   {
      wait_queue::peek_fence();
      return !queue.empty();
   }

   /**
    * @brief Wake ALL waiting threads (if any).
    *
    * @param policy After waking a waiting thread, apply this reschedule policy.
    *
    * No-op if no waiters. Safe from ISR context.
    * Threads are woken as a batch with preemption disabled
    * before releasing all at once.
    */
   void wake_all(reschedule_policy policy = reschedule_policy::automatic) noexcept;

   /**
    * @brief Wake only the waiting threads @p wants accepts.
    *
    * @param wants Asked about each waiter, with the record its poll would get:
    *        its installed waiter_record if that names this waitable, else null.
    * @param policy After waking, apply this reschedule policy.
    *
    * For a waitable whose waiters want different things, such as event flags
    * waited on by mask: a change wakes only the waiters it can satisfy, and the
    * rest stay parked and never run, where wake_all would run each of them
    * only to park it again.
    *
    * @p wants must answer exactly as the waiter's own poll would. A false for
    * a waiter the poll would satisfy is a lost wakeup. A true for one it would
    * not is only a spurious wake, which the wait loop absorbs. Change the state
    * the waiters poll BEFORE the call: a waiter that arms after it polls the
    * change itself.
    *
    * @p wants runs under the queue lock, with interrupts masked, on each
    * queued waiter, so it must be tiny and read only: no lock, no wake, no
    * blocking. A waiter's record is alive and unchanging while it is asked,
    * since the waiter installs it before arming and removes it after its last
    * disarm. Safe from ISR context.
    */
   void wake_all_where(std::function_ref<bool(waiter_record const*)> wants,
                       reschedule_policy policy = reschedule_policy::automatic) noexcept;

   /**
    * @brief Wake-and-handover to the single highest-priority waiting thread (if any).
    *
    * @param transfer Callable that hands over resource ownership to woken thread.
    * @param policy After waking a waiting thread, apply this reschedule policy.
    * @return true when a waiter was chosen and readied, false when the
    *         queue was empty and transfer received 0. Discardable.
    *
    * No-op if no waiters. Safe from ISR context.
    * The barge-free sibling of wake_one(). Pops the best waiter and invokes
    * transfer exactly once under the queue lock, passing the waiter's thread
    * id, or 0 when no thread is parked. transfer must record the resource
    * state for BOTH outcomes, e.g. owner = next_owner_id. Committing the
    * empty case under the same lock is what closes the lost wakeup where a
    * waiter arms, polls the still-held resource, and parks just before this
    * release frees it.
    *
    * A woken thread finds the resource already assigned to it, so the
    * derived hook must recognise ownership by id in addition to
    * taking the resource when free. No fresh caller can interpose between
    * the release and the woken thread running, which is the barge-free
    * guarantee.
    *
    * transfer runs under a spinlock. It must be tiny, must not block, must
    * not wake, and must not touch this waitable's queue.
    */
   bool wake_one_and_transfer(transfer_fn transfer, reschedule_policy policy = reschedule_policy::automatic) noexcept;

   /**
    * @brief Group-wait handback hook.
    *
    * A transfer can commit ownership to a thread parked in wait_on_any at any
    * moment while its node is armed, so a group wait can wake owning MORE
    * than the waitable it returns. The contract is that it owns exactly the
    * returned index (plus anything it held before the call), so wait_on_any
    * sweeps the non-chosen waitables through this hook after the final
    * disarm, when no further transfer can choose the caller. Ownerless
    * waitables have nothing to hand back, hence the default no-op.
    *
    * Defined in waitable.cpp, not here, and that is load-bearing. As the one
    * virtual with a body it is the class's key function, so its vtable and
    * itself exist once, in libcyros. Inline, every application that derives
    * a waitable emitted its own copies, and against a release archive (its
    * merged object's hidden symbols made local) the linker kept the
    * application's and discarded the archive's, leaving the archive pointing
    * into a discarded section: no derived waitable linked at all.
    */
   virtual void renounce_if_assigned(thread::id thread_id) noexcept;

private:
   /// Composed, not inherited. See wait_queue above for why the type is
   /// nameable here and still unusable.
   wait_queue queue;

   friend class wait_node_vector;
   template<std::size_t> friend class waitable_arm_guard;
   friend class wait_engine;
};


/**
 * @brief waitable type that NEVER blocks the thread
 *
 * This allows wait_for_any({target, non_blocking_token});
 * patterns.
 */
class CYROS_PUBLIC non_blocking_token : public waitable
{
protected:
   bool try_satisfy(waiter_record*) noexcept override
   {
      return true;
   }
};


} // namespace cyros

#endif // CYROS_WAITABLE_HPP
