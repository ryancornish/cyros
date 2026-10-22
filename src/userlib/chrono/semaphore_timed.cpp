/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file semaphore_timed.cpp
 * @brief The timed semaphore methods, owned by chrono, declared by sync.
 *
 * sync is a time-free feature, so these definitions live here: a package that
 * imports sync alone links clean until something calls a timed method, and
 * including chrono is what satisfies the linker. The dependency points
 * chrono -> sync, never the reverse.
 */

#include <cyros/chrono/alarm.hpp>
#include <cyros/chrono/chrono.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/semaphore.hpp>

namespace cyros::sync
{

bool semaphore::try_acquire_until(time::time_point tp) noexcept
{
   // An expired deadline degrades to the non-blocking take. Without this the
   // alarm would not fire until the driver's next evaluation, which would
   // stretch "already too late" into "up to one tick late".
   if (!(time::now() < tp)) {
      return try_acquire();
   }

   chrono::alarm deadline;
   deadline.arm_at(tp);

   // The semaphore is polled before the deadline on every pass, so a token
   // wins over a deadline that passed in the same pass. True means the poll
   // took it already. The alarm disarms itself on the way out.
   return this_thread::wait_on(*this, deadline);
}

bool semaphore::try_acquire_for(time::duration d) noexcept
{
   if (d.value == 0) {
      return try_acquire();
   }
   return try_acquire_until(time::now() + d);
}

} // namespace cyros::sync
