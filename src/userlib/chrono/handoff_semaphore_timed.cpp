/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file handoff_semaphore_timed.cpp
 * @brief The timed handoff_semaphore methods, owned by chrono, declared by
 *        sync, as semaphore_timed.cpp's are.
 */

#include <cyros/chrono/alarm.hpp>
#include <cyros/chrono/chrono.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/handoff_semaphore.hpp>

namespace cyros::sync
{

bool handoff_semaphore::try_acquire_until(time::time_point tp) noexcept
{
   if (!(time::now() < tp)) {
      return try_acquire();
   }
   if (try_acquire()) {
      return true;
   }

   chrono::alarm deadline;
   deadline.arm_at(tp);

   // A token handed over wins over a deadline that passed in the same pass:
   // the wait polls this after its last disarm and before the deadline may
   // end it, and that poll finds the hand-over. So false means none was.
   waiter self(*this);
   bool const taken = this_thread::wait_on(*this, deadline);
   settle(self);
   return taken;
}

bool handoff_semaphore::try_acquire_for(time::duration d) noexcept
{
   if (d.value == 0) {
      return try_acquire();
   }
   return try_acquire_until(time::now() + d);
}

} // namespace cyros::sync
