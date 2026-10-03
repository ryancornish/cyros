#ifndef CYROS_CONDITION_VARIABLE_HPP
#define CYROS_CONDITION_VARIABLE_HPP

#include <cyros/kernel/base_mutex.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/waitable.hpp>
#include <cyros/kernel/visibility.hpp>

#include <concepts>
#include <functional>

namespace cyros::time { struct time_point; struct duration; }

namespace cyros::sync
{

enum class cv_status
{
   no_timeout,
   timeout,
};

/**
 * @brief Wait until a predicate over mutex-guarded state holds.
 *
 * std::condition_variable semantics over any base_mutex (sync::mutex or
 * sync::cemutex). wait() releases the mutex and blocks as one step with
 * respect to notify, and re-acquires it through the ordinary barge-free
 * lock() before returning, so the woken thread donates to whoever holds the
 * mutex the moment it arms there. The inheritance property needs nothing from
 * this type.
 *
 * Wakeups are spurious only in the ways the standard permits, so wait in a
 * loop over the predicate, or use the predicate overloads.
 *
 * How a notify cannot be lost
 * ---------------------------
 * A notify is not a sequence number that waiters compare against. That shape
 * lets a thread that starts waiting AFTER a notify steal the wake meant for
 * one that was already parked (glibc bug 13165), whenever the notifier does
 * not hold the mutex, which the standard allows and recommends. Instead a
 * waiter returns only when a notify's wake chose IT:
 *
 *  - Each wait builds a record on its own stack (a waiter_record naming this
 *    variable), then waits in the ordinary way.
 *  - It arms on the queue BEFORE releasing the mutex. The release happens in
 *    try_satisfy's first poll, which runs with the node armed, so a notify
 *    issued after the release is guaranteed to find the waiter.
 *  - notify_one is a plain wake_one, notify_all a plain wake_all. A wake
 *    takes the chosen waiter's node off the queue, and the waiter's own
 *    disarm finds it gone and marks its record (waiter_record::chosen). So
 *    the threads woken are exactly the threads notified, and no wake can go
 *    unmarked.
 *
 * ISR-safe notify: both notifies are the ISR-safe wakes and nothing else.
 * wait is thread context only. notify_all pops until the queue is empty, so a
 * waiter that arms while it runs is woken too, like any wake_all.
 *
 * Two other ways of knowing a notify chose this waiter were built and
 * measured, the kernel marking the record from the wake and a registry in the
 * cv: ~/cyros-claude/archive/roadmap-history.md, with patches beside it.
 *
 * Not a waitable to users. It inherits privately, so it cannot join a
 * wait_on_any group, which would bypass the record and the mutex release. A
 * timed wait is provided here instead.
 */
class CYROS_PUBLIC condition_variable : private waitable
{
public:
   constexpr condition_variable() noexcept = default;

   /**
    * @brief Unblock the highest-priority thread waiting, if any.
    *
    * Safe with or without the mutex held, and from an ISR.
    */
   void notify_one() noexcept;

   /**
    * @brief Unblock every thread waiting.
    *
    * Safe with or without the mutex held, and from an ISR. The waiters then
    * contend for the mutex in priority order, one wakeup each. See the note
    * on requeue in the implementation.
    */
   void notify_all() noexcept;

   /**
    * @brief Release @p m, block until notified, re-acquire @p m.
    *
    * The caller must own @p m. Returns owning @p m, whatever woke it.
    */
   void wait(base_mutex& m) noexcept;

   /**
    * @brief wait() until @p stop_waiting returns true, checked under @p m.
    */
   template<std::predicate Predicate>
   void wait(base_mutex& m, Predicate stop_waiting)
   {
      while (!stop_waiting()) {
         wait(m);
      }
   }

   /**
    * @brief wait(), giving up at @p tp / after @p d.
    *
    * PROVIDED BY THE CHRONO FEATURE, like semaphore's timed methods: sync is
    * time-free, and calling one without chrono is a link error naming the
    * method. The mutex is re-acquired before returning in both outcomes.
    *
    * @return timeout only if the deadline passed with no notify handed to
    *         this waiter. A notify racing the deadline reports no_timeout.
    */
   [[nodiscard]] cv_status wait_until(base_mutex& m, time::time_point tp) noexcept;

   [[nodiscard]] cv_status wait_for(base_mutex& m, time::duration d) noexcept;

   /**
    * @brief Timed wait until @p stop_waiting holds.
    *
    * @return the predicate's final value, evaluated under @p m.
    *
    * The predicate is a function_ref rather than a template parameter because
    * these live in chrono, where the time types are complete.
    */
   [[nodiscard]] bool wait_until(base_mutex& m, time::time_point tp,
                                 std::function_ref<bool()> stop_waiting) noexcept;

   [[nodiscard]] bool wait_for(base_mutex& m, time::duration d,
                               std::function_ref<bool()> stop_waiting) noexcept;

private:
   /**
    * @brief One thread's wait, on its own stack for the length of the wait.
    *
    * Touched only by the waiter. Whether a notify chose it is the base's
    * chosen(), which the waiter's own disarm records.
    */
   struct waiter : waiter_record
   {
      waiter(waitable& source, base_mutex& m) noexcept : waiter_record(source), mutex(&m) {}

      base_mutex* mutex;
      bool        released{false};
   };

   /**
    * @brief Release the caller's mutex on the first poll, then report whether
    *        a notify has chosen it.
    */
   bool try_satisfy(waiter_record* record) noexcept override;
};

}  // namespace cyros::sync

namespace cyros
{
using sync::condition_variable;
using sync::cv_status;
}

#endif // CYROS_CONDITION_VARIABLE_HPP
