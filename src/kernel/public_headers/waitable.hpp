#ifndef CORTOS_WAITABLE_HPP
#define CORTOS_WAITABLE_HPP

#include <cortos/kernel/function.hpp>
#include <cortos/kernel/thread.hpp>
#include <cortos/kernel/spinlock.hpp>

#include <type_traits>

namespace cortos
{

/**
 * @brief Base class for objects that can block threads
 *
 * waitable is inherited by synchronization primitives (Mutex, Semaphore, etc.)
 * and time-aware objects (Timer). It provides hooks for custom behavior when
 * threads block/wake on the object.
 *
 * Threads do NOT call methods on waitable directly. Instead, use the free
 * functions kernel::wait_for() and kernel::wait_for_any().
 *
 * Example (Timer in libcortos):
 *   class Timer : public waitable
 *   {
 *      time_point wakeup_time;
 *      // TimeDriver calls wake_one() when time expires
 *   };
 *
 *   Timer timer;
 *   Mutex mutex;
 *   auto result = kernel::wait_for_any({&mutex, &timer});
 *   // Woken by whichever fired first
 */
class waitable
{
public:
   using predicate = function<bool(), 64, heap_policy::no_heap>;

   /**
   * @brief Result of a wait operation
   *
   * Returned from `wait_for()` and `wait_for_any()` to indicate which `waitable`
   * triggered the wake-up and whether the thread acquired a resource.
   */
   struct result
   {
      int  index{-1};       ///< Index of waitable that triggered (-1 if none)
      bool acquired{false}; ///< True if resource was acquired (e.g., mutex locked)
   };
   static_assert(std::is_trivially_copyable_v<waitable::result>, "waitable::result must be trivially copyable");

   /**
   * @brief Snapshot of a thread waiting on a waitable
   *
   * waiter is a lightweight, read-only snapshot of a thread at the moment it
   * blocks on or is removed from a waitable. It contains no ownership or control
   * semantics and is safe to copy and store.
   */
   struct waiter
   {
      thread::id       id;                  ///< Unique thread identifier
      thread::priority base_priority;       ///< Thread's base (static) priority
      thread::priority effective_priority;  ///< Thread's effective priority at snapshot time
      std::uint32_t    pinned_core;         ///< Core the thread is pinned to
      core_affinity     affinity;            ///< Core affinity mask

      /**
      * @brief Compare priority against another waiter
      * @return true if this waiter has higher scheduling priority than rhs
      */
      [[nodiscard]] constexpr bool higher_priority_than(waiter const& rhs) const noexcept
      {
         return effective_priority.val < rhs.effective_priority.val;
      }
   };
   static_assert(std::is_trivially_copyable_v<waitable::waiter>, "waitable::waiter must be trivially copyable");

   virtual ~waitable() = default;

   waitable(waitable const&)            = delete;
   waitable& operator=(waitable const&) = delete;
   waitable(waitable&&)            = delete;
   waitable& operator=(waitable&&) = delete;

   /**
    * @brief Check if any threads are waiting
    * @return true if wait queue is empty, false if threads are waiting
    */
   [[nodiscard]] bool empty() const noexcept;

   /**
    * @brief Signal one waiting thread (highest priority)
    * @param acquired True if signalled thread acquired the resource (e.g., mutex lock)
    *
    * Moves the highest-priority waiting thread to the ready queue.
    * If no threads are waiting, this is a no-op.
    *
    * The 'acquired' parameter is returned in waitable::result:
    * - Mutex::unlock() -> wake_one(true)  // Woken thread now owns mutex
    * - Semaphore::post() -> wake_one(false) // Woken thread is just notified
    * - Timer::expire() -> wake_one(false)   // Woken thread didn't acquire anything
    *
    * Called by the owning primitive (e.g., Mutex::unlock(), Timer expiry).
    */
   void signal_one(bool acquired = true) noexcept;

   /**
    * @brief Signal all waiting threads
    * @param acquired True if signalled threads acquired the resource
    *
    * Moves all waiting threads to the ready queue.
    * If no threads are waiting, this is a no-op.
    */
   void signal_all(bool acquired = true) noexcept;

protected:
   // Abstract Base Class
   waitable() = default;

   /**
    * @brief Called when a thread blocks on this waitable
    * @param waiter Details of the blocking thread
    *
    * Override to implement custom behavior (e.g., priority inheritance).
    * Called before thread is added to wait queue.
    * @note no-op when not overridden
    */
   virtual void on_thread_blocked(waiter waiter) { (void)waiter; }

   /**
    * @brief Called when a thread is removed from wait queue
    * @param waiter Details of the thread being removed (woken or cancelled)
    *
    * Override to implement cleanup (e.g., clear inherited priority).
    * Called after thread is removed from wait queue.
    * @note no-op when not overridden
    */
   virtual void on_thread_removed(waiter waiter) { (void)waiter; }

   using waiter_visitor = function<void(waiter const&), 64, heap_policy::no_heap>;
   /**
    * @brief Visit each thread currently waiting on this waitable
    *
    * Calls @p visitor once per waiter with a snapshot taken during traversal.
    * @warning The visitor must not block.
    *
    * Example:
    * @code
    * priority max_priority = priority{0};
    * for_each_waiter([&](waiter w) {
    *    if (w.effective_priority > max_priority) {
    *       max_priority = w.effective_priority;
    *    }
    * });
    */
   void for_each_waiter(waiter_visitor visitor) const;

private:
   friend struct thread_control_block;
   friend class  waitable_group_lock;

   struct wait_node* head{nullptr};
   struct wait_node* tail{nullptr};

   mutable spinlock wait_lock;

   void add(wait_node& wait_node) noexcept;
   void remove(wait_node& wait_node) noexcept;

   // Select best waiter but do NOT unlink it.
   // Caller must hold wait_lock.
   // FIFO among equals: scan from head, pick first with highest priority.
   wait_node* pick_best() noexcept;
};

} // namespace cortos

#endif // CORTOS_WAITABLE_HPP
