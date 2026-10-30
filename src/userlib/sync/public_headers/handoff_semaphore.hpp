/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

#ifndef CYROS_HANDOFF_SEMAPHORE_HPP
#define CYROS_HANDOFF_SEMAPHORE_HPP

#include <cyros/kernel/waitable.hpp>
#include <cyros/kernel/attribute.hpp>

#include <atomic>
#include <cstddef>

namespace cyros::time { struct time_point; struct duration; }

namespace cyros::sync
{

/**
 * @brief A counting semaphore whose release hands its token to a waiter.
 *
 * The same counting contract as semaphore, without barging. A release while a
 * thread waits gives the token to the most urgent waiter, which returns from
 * its acquire holding it, and nothing else can take it in between: not the
 * releaser, not a thread arriving on another core, not an interrupt. A token
 * is counted only while nobody waits. So a waiter is never woken for nothing,
 * and the most urgent waiter gets the next token, which is what a bound on
 * its wait rests on.
 *
 * The price is the reverse of semaphore's. A token released to a waiter that
 * cannot run yet (it is less urgent than the releaser, or its core is busy)
 * is unusable until it runs, where semaphore would let a running thread take
 * it. A thread that releases and acquires again in a loop, with a waiter
 * queued, waits behind it every time.
 *
 * One exception, a race of arrivals rather than an overtaking: a thread that
 * starts to wait on one core while a release runs on another may find the
 * token taken by a third that arrived as it did. A waiter queued before a
 * release began is never overtaken.
 *
 * Waited on through its own members only. It inherits waitable privately,
 * because a wait that has not installed this type's waiter record could not
 * tell a token handed to it from no token at all.
 */
class CYROS_PUBLIC handoff_semaphore : private waitable
{
public:
   constexpr explicit handoff_semaphore(std::size_t n) : counter(n) {}

   /// The tokens counted now. A token handed to a waiter is not counted.
   [[nodiscard]] std::size_t peek() const noexcept;

   /// Hands each token to the most urgent waiter, and counts it if none waits.
   /// ISR-safe.
   void release(std::size_t n = 1) noexcept;

   void acquire() noexcept;

   /// Takes a counted token. Never one handed to a waiter.
   [[nodiscard]] bool try_acquire() noexcept;

   /**
    * @brief try_acquire, blocking up to @p d / until @p tp for a token.
    *
    * PROVIDED BY THE CHRONO FEATURE, as semaphore's are. A token handed over
    * as the deadline passes is taken: the call returns true and the token is
    * not lost. A deadline at or before now degrades to plain try_acquire.
    */
   [[nodiscard]] bool try_acquire_for(time::duration d) noexcept;

   [[nodiscard]] bool try_acquire_until(time::time_point tp) noexcept;

private:
   /// The record each wait installs (waiter_record), which tells a token
   /// handed over from none, and notes a counted token taken.
   class waiter : public waiter_record
   {
   public:
      explicit waiter(handoff_semaphore& s) noexcept : waiter_record(s) {}
      bool took_counted{false};
   };

   bool try_satisfy(waiter_record* record) noexcept override;

   /// The token to the most urgent waiter, or counted if none waits.
   void hand_over() noexcept;

   /// hand_over, @p n times: a release that found a waiter queued.
   void hand_over_each(std::size_t n) noexcept;

   /// A release that counted @p n tokens and then found a waiter armed: each
   /// still counted, up to @p n, taken back and handed over.
   void reclaim(std::size_t n) noexcept;

   /// After a wait: a waiter that took a counted token and was also handed
   /// one gives one back.
   void settle(waiter const& self) noexcept;

   std::atomic<std::size_t> counter;
};

}  // namespace cyros::sync

namespace cyros
{
using sync::handoff_semaphore;
}

#endif // CYROS_HANDOFF_SEMAPHORE_HPP
