/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file condition_variable_timed.cpp
 * @brief The timed condition_variable methods, owned by chrono, declared by sync.
 *
 * Same split as semaphore_timed.cpp: sync is time-free, and the dependency
 * points chrono -> sync. The timeout bounds the wait for a notify, never the
 * re-lock, which is the ordinary untimed lock(). That keeps this clear of the
 * no-timed-mutex decision (mutex-first-class-plan.md D5).
 */

#include <cyros/chrono/alarm.hpp>
#include <cyros/chrono/chrono.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/condition_variable.hpp>


namespace cyros::sync
{

cv_status condition_variable::wait_until(base_mutex& m, time::time_point tp) noexcept
{
   // An expired deadline still releases and re-acquires, as a wait would, so a
   // caller looping on this gives other lockers a turn. Without the early out
   // the alarm would not fire until the driver's next evaluation, which would
   // stretch "already too late" into "up to one tick late".
   if (!(time::now() < tp)) {
      m.unlock();
      m.lock();
      return cv_status::timeout;
   }

   chrono::alarm deadline;
   deadline.arm_at(tp);

   bool notified = false;
   {
      waiter self(*this, m);

      // This variable is polled first on every pass, so its first poll, which
      // releases m, always runs. Its poll answers chosen(), and the wait
      // polls it after its last disarm before the deadline may end the wait,
      // so true here is exactly "a notify chose us". A notify that lands as
      // the deadline passes is never swallowed.
      notified = this_thread::wait_on(static_cast<waitable&>(*this), deadline);
   }
   deadline.disarm();

   m.lock();

   return notified ? cv_status::no_timeout : cv_status::timeout;
}

cv_status condition_variable::wait_for(base_mutex& m, time::duration d) noexcept
{
   return wait_until(m, time::now() + d);
}

bool condition_variable::wait_until(base_mutex& m, time::time_point tp,
                                    std::function_ref<bool()> stop_waiting) noexcept
{
   while (!stop_waiting()) {
      if (wait_until(m, tp) == cv_status::timeout) {
         return stop_waiting();
      }
   }
   return true;
}

bool condition_variable::wait_for(base_mutex& m, time::duration d,
                                  std::function_ref<bool()> stop_waiting) noexcept
{
   return wait_until(m, time::now() + d, stop_waiting);
}

} // namespace cyros::sync
