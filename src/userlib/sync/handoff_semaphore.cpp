/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

#include <cyros/sync/handoff_semaphore.hpp>
#include <cyros/kernel/thread.hpp>

namespace cyros::sync
{

/* How a token reaches a waiter
 * ----------------------------
 * A release with a waiter queued hands the token over under the queue's lock
 * (waitable::wake_one_and_transfer), which pops the most urgent waiter, and
 * the token is that waiter's from then on. Nothing records it: the waiter
 * learns it from its own wait, whose disarm finds its node taken
 * (waiter_record::chosen), because every wake of this waitable is a hand-over.
 * With nobody queued the token is counted, under the same lock a waiter arms
 * under, so a waiter arming next polls it.
 *
 * Releasing without the lock
 * --------------------------
 * Nobody waiting is the common case, and then a release counts its tokens with
 * no lock, as semaphore's does, in the order may_have_waiters needs: the
 * tokens in, then the test. A waiter that armed in between either polled a
 * token or parked without one, and the release cannot tell which. So it takes
 * back each token still counted, up to the number it released, and hands it
 * over properly (reclaim). One gone was taken by the waiter's poll or another
 * arrival, and nothing is owed for it.
 *
 * Each way on from the test is a function of its own, never inlined, so the
 * path with nobody waiting makes no call: a call anywhere in release would
 * have it build a frame and save registers on every release, waiters or not.
 *
 * A waiter that takes a counted token and is handed one
 * -----------------------------------------------------
 * A waiter's first poll runs armed, so a hand-over can choose it after that
 * poll took a counted token, before its disarm. It then holds two, and gives
 * one back (settle). */

[[nodiscard]] std::size_t handoff_semaphore::peek() const noexcept
{
   return counter.load(std::memory_order::relaxed);
}

CYROS_HOT void handoff_semaphore::release(std::size_t n) noexcept
{
   if (may_have_waiters()) {
      hand_over_each(n);
      return;
   }
   counter.fetch_add(n, std::memory_order_release);
   if (may_have_waiters()) {
      reclaim(n);
   }
}

[[gnu::noinline]] CYROS_HOT void handoff_semaphore::hand_over_each(std::size_t n) noexcept
{
   for (; n > 0; --n) {
      hand_over();
   }
}

[[gnu::noinline, gnu::cold]] void handoff_semaphore::reclaim(std::size_t n) noexcept
{
   for (; n > 0 && try_acquire(); --n) {
      hand_over();
   }
}

CYROS_HOT void handoff_semaphore::hand_over() noexcept
{
   wake_one_and_transfer([this](thread::id const chosen) {
      if (chosen == 0) {
         counter.fetch_add(1, std::memory_order_release);
      }
   });
}

CYROS_HOT void handoff_semaphore::acquire() noexcept
{
   // A counted token is free to take: tokens are counted only while nobody
   // waits.
   if (try_acquire()) {
      return;
   }
   waiter self(*this);
   this_thread::wait_on(*this);
   settle(self);
}

[[nodiscard]] CYROS_HOT bool handoff_semaphore::try_acquire() noexcept
{
   auto current = counter.load(std::memory_order_relaxed);
   while (current > 0) {
      if (counter.compare_exchange_weak(current, current - 1,
                                        std::memory_order_acquire,
                                        std::memory_order_relaxed)) {
         return true;
      }
   }
   return false;
}

CYROS_HOT bool handoff_semaphore::try_satisfy(waiter_record* const record) noexcept
{
   // Never null: waitable is a private base, so only this type's own
   // members wait on it, and each installs a waiter first.
   auto* const self = static_cast<waiter*>(record);
   if (self->chosen()) {
      return true;
   }
   if (try_acquire()) {
      self->took_counted = true;
      return true;
   }
   return false;
}

CYROS_HOT void handoff_semaphore::settle(waiter const& self) noexcept
{
   if (self.took_counted && self.chosen()) {
      release(1);
   }
}

}  // namespace cyros::sync
