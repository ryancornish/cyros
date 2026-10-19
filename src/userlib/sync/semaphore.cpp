/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

#include <cyros/sync/semaphore.hpp>

namespace cyros::sync
{

[[nodiscard]] std::size_t semaphore::peek() const noexcept
{
   return counter.load(std::memory_order::relaxed);
}

void semaphore::release(std::size_t n) noexcept
{
   counter.fetch_add(n, std::memory_order_release);
   // The tokens are in before the test, which is the order may_have_waiters
   // needs. Nobody waiting is the common case, and then no lock is taken.
   if (!may_have_waiters()) {
      return;
   }
   for (std::size_t i = 0; i < n; ++i) {
      wake_one();
   }
}

void semaphore::acquire() noexcept
{
   // A free token is taken here, without arming a wait node. The general wait
   // path would find the same token on its first poll, after an arm and a
   // disarm that cost most of an uncontended acquire. It barges exactly as
   // that poll does, and as try_acquire always has.
   if (try_acquire()) {
      return;
   }
   this_thread::wait_on(*this);
}

[[nodiscard]] bool semaphore::try_acquire() noexcept
{
   auto current = counter.load(std::memory_order_relaxed);
   // Another thread may change counter after we have loaded it.
   // If so, then the compare-exchange will fail and we try again.
   while (current > 0) {
      if (counter.compare_exchange_weak(current, current - 1,
                                        std::memory_order_acquire,
                                        std::memory_order_relaxed)) {
         return true;
      }
   }
   return false;
}

bool semaphore::try_satisfy(waiter_record*) noexcept
{
   return try_acquire();
}

}  // namespace cyros::sync
