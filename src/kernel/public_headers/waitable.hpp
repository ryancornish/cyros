#ifndef CYROS_WAITABLE_HPP
#define CYROS_WAITABLE_HPP

#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/function.hpp>
#include <cyros/kernel/spinlock.hpp>

#include <cstddef>
#include <cstdint>
#include <span>

namespace cyros
{

/* ============================================================================
 * waitable - kernel base class for blockable objects
 *
 * Inherit from waitable to make your object parkable. The base class owns
 * a private wait_queue and exposes:
 *   - block()             : stateless park; wake on any signal (caller accepts
 *                           the possibility of missed events - see below)
 *   - block_until(pred)   : park until a caller-supplied predicate holds
 *   - wake_one/wake_all   : protected; for derived classes to signal waiters
 *   - is_satisfied(self)  : virtual; overridden by derived primitives so
 *                           block_on_any can poll them
 *
 * Block-on-any is provided as a free function (block_on_any) that is a
 * friend of waitable.
 *
 * Composition vs inheritance
 * --------------------------
 * Inherit waitable when your object IS something threads block ON
 * (mutex, semaphore, thread-termination, event, timer-source). Use plain
 * composition - an waitable member - when your object merely USES blocking
 * internally without being a wait target itself.
 *
 * The lost-wakeup problem and the two-phase block
 * -----------------------------------------------
 * A thread cannot simply "check, then park": between the check and the park,
 * another context may signal the condition, and the wakeup is lost. waitable
 * solves this with a two-phase block performed internally by block_until():
 *
 *   1. arm   : record intent to block, under the queue lock. Any signal
 *              AFTER this point is guaranteed to reach the caller.
 *   2. check : evaluate the predicate. If true, abandon the block.
 *   3. park  : commit to blocking. A signal that arrived since arming is
 *              honoured, not lost. Loops on spurious wakeup.
 *
 * The raw three-phase primitives are NOT exposed.
 *
 * Spurious wakeups
 * ----------------
 * A woken thread is NOT guaranteed that its condition holds: the resource may
 * have been consumed by a higher-priority waiter, or by a fresh caller racing
 * in, before the woken thread runs. block_until() and block_on_any() handle
 * this by re-checking and re-blocking transparently. This is intrinsic to all
 * blocking primitives (cf. the mandatory while-loop around pthread_cond_wait)
 * and is not a Cyros-specific cost.
 *
 * Barging
 * -------
 * waitable is BARGE-PERMITTING by design: a wake does not reserve the
 * resource for the woken thread, so a fresh thread may acquire it between
 * the wake and the woken thread running. This is cheap, high-throughput, and
 * sufficient for many primitives (counting semaphore, event, join). If your
 * primitive requires strict, barge-free, priority-fair handoff (typically a
 * mutex that must avoid priority inversion), build it on handoff_queue
 * instead, which packages that harder guarantee. Choose waitable for speed
 * and simplicity; choose handoff_queue when fairness is a correctness
 * requirement.
 *
 * Concurrency & ISR safety
 * ------------------------
 * wake_one() and wake_all() are safe to call from ISR context. A wake from
 * an ISR (or any context that cannot synchronously switch) defers its
 * reschedule via the weak reschedule contract - see reschedule_policy. List
 * mutation is protected by a per-queue lock; the queue is safe under SMP
 * and against concurrent block()/wake() on different cores.
 * ========================================================================= */
class waitable
{
public:
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

   virtual ~waitable();

   waitable(waitable&&) = delete;
   waitable(waitable const&) = delete;
   waitable& operator=(waitable&&) = delete;
   waitable& operator=(waitable const&) = delete;

protected:
   waitable() noexcept = default;

   /**
    * @brief Override in derived primitives so block_on_any can poll them.
    *
    * Called under the waitable's queue lock during block_on_any prepare /
    * recheck. Must not block. Side-effects ARE permitted if and only if they
    * are atomic with returning true (e.g. "take the lock if free, return
    * true; else return false unchanged"); see mutex::is_satisfied for the
    * canonical example.
    *
    * The default implementation returns false. A pure notification waitable
    * (no associated state) MAY leave the default and rely on callers using
    * block(); such an waitable cannot participate meaningfully in
    * block_on_any.
    *
    * @param caller The thread evaluating the predicate (always the currently
    *               running thread on this core).
    */
   virtual bool is_satisfied(thread& caller) noexcept = 0;

   /**
    * @brief Wake the single highest-priority parked thread, if any.
    *
    * No-op if no waiters. Safe from ISR context. Use this for primitives
    * where at most one waiter can proceed per signal (mutex unlock,
    * semaphore release of one permit).
    */
   void wake_one(reschedule_policy policy = reschedule_policy::automatic) noexcept;

   /**
    * @brief Wake ALL parked threads.
    *
    * Use only when the signalled condition can genuinely satisfy every
    * waiter (manual-reset event, barrier release, broadcast of thread
    * termination to all joiners). Using wake_all where wake_one suffices is
    * the classic thundering-herd mistake. Safe from ISR context.
    */
   void wake_all(reschedule_policy policy = reschedule_policy::automatic) noexcept;

private:
   /**
    * @brief Per-thread parking record (one per TCB, reused).
    *
    * Lives in the TCB, threaded into one or more waitable::wait_queue
    * instances when the thread is parked.
    */
   struct wait_node
   {
      thread_control_block* owner{nullptr};
      wait_node*            next {nullptr};

      // For block_on_any: which waitable does this slot in the call's source
      // array correspond to. Unused (and zero) in single-wait blocks.
      uint8_t source_index{0};
   };
   friend class wait_node_vector;

   /**
    * @brief Private intrusive priority-ordered list of parked threads.
    *
    * Nested inside waitable specifically so derived primitives cannot grab
    * a reference to one. There is no public API that returns or accepts a
    * wait_queue&. All access goes through waitable's own methods (or the
    * friend free function block_on_any).
    */
   class wait_queue
   {
   public:
      // Three-phase primitives - internal only. See class doc for protocol.
      void arm   (wait_node& n) noexcept;
      void disarm(wait_node& n) noexcept;

      void wake_one(reschedule_policy) noexcept;
      void wake_all(reschedule_policy) noexcept;

      [[nodiscard]] bool empty() const noexcept;

   private:
      spinlock   lock;
      wait_node* head{nullptr}; // priority-ordered, best at head
   };
   friend class waitable_arm_guard;

   wait_queue queue;

   friend std::size_t this_thread::wait_on_any(std::span<waitable_ref> waitables) noexcept;
};


/**
 * @brief waitable type that NEVER blocks the thread
 *
 * This allows wait_for_any({target, non_blocking_token});
 * patterns.
 */
class non_blocking_token : public waitable
{
protected:
   bool is_satisfied(thread&) noexcept override
   {
      return true;
   }
};

} // namespace cyros

#endif // CYROS_WAITABLE_HPP
