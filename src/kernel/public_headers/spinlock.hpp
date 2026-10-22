/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

#ifndef CYROS_SPINLOCK_HPP
#define CYROS_SPINLOCK_HPP

#include <cyros/config/config.hpp>
#include <cyros/kernel/core.hpp>

#include <atomic>
#include <type_traits>
#include <cyros/kernel/attribute.hpp>

namespace cyros
{

/**
 * @brief Interrupt-masking spinlock for short critical sections
 *
 * Holding the lock is an interrupt-masking critical section on the calling
 * core: neither a thread switch nor an ISR can interleave with the holder.
 *
 * The cost of the guarantee is a latency contract: the system's worst-case
 * interrupt latency includes the longest hold plus any contention spin, so
 * sections must stay tiny (i.e. a few dozen instructions). For longer critical
 * sections, use a mutex.
 *
 * Usage:
 * @code
 *   spinlock lock;
 *   lock.lock();
 *   // ... critical section ...
 *   lock.unlock();
 * @endcode
 *
 * Or with RAII:
 * @code
 *   spinlock lock;
 *   {
 *       spinlock_guard guard(lock);
 *       // ... critical section ...
 *   } // Automatically unlocked
 * @endcode
 */
class CYROS_PUBLIC spinlock
{
public:
   constexpr spinlock() = default;

   ~spinlock() = default;

   spinlock(spinlock const&)            = delete;
   spinlock& operator=(spinlock const&) = delete;
   spinlock(spinlock&&)                 = delete;
   spinlock& operator=(spinlock&&)      = delete;

   /**
    * @brief Acquire the spinlock (busy-wait)
    *
    * Blocks until the lock is acquired.
    * Uses CPU hints to reduce power consumption while spinning.
    *
    * Inline, with unlock, because every queue operation takes one and the
    * body is a few instructions: out of line, GCC was found calling it.
    */
   void lock()
   {
      auto const saved = this_core::enter_critical();
      while (!take(flag)) {
         this_core::cpu_relax();
      }
      token = saved; // only the holder writes it, see spinlock.cpp
   }

   /**
    * @brief Release the spinlock
    */
   void unlock()
   {
      drop(flag);
      this_core::exit_critical(token);
   }

   /**
    * @brief Try to acquire the spinlock without blocking
    * @return true if acquired, false if already locked
    *
    * On success the caller holds the lock in exactly the state lock() gives:
    * inside an interrupt-masking critical section that unlock() ends. On
    * failure the caller is left exactly as it was, not holding and not masked.
    */
   bool try_lock();

   /**
    * @brief Check if the spinlock is currently locked
    * @return true if locked, false if unlocked
    *
    * Note: This is racy and should only be used for debugging/assertions.
    */
   [[nodiscard]] bool is_locked() const
   {
      return held(flag);
   }

private:
   // It is more optimal is single-core configurations to just use an atomic bool
   // as the flag. (It is either held, or it is not). On multi-core configurations.
   // a proper atomic flag must be used as test-and-set semantics are necessary.
   using flag_type = std::conditional_t<config::cores == 1, std::atomic<bool>, std::atomic_flag>;

   static bool held(std::atomic<bool> const& flag) noexcept { return flag.load(std::memory_order_relaxed); }
   static bool held(std::atomic_flag  const& flag) noexcept { return flag.test(std::memory_order_relaxed); }

   static void drop(std::atomic<bool>& flag) noexcept { flag.store(false, std::memory_order_relaxed); }
   static void drop(std::atomic_flag&  flag) noexcept { flag.clear(std::memory_order_release); }

   // Single-core: interrupts are already masked,so a plain store takes it.
   static bool take(std::atomic<bool>& flag) noexcept
   {
      if (flag.load(std::memory_order_relaxed)) return false;

      flag.store(true, std::memory_order_relaxed);
      return true;
   }

   // Multi-core: test-and-set is necessary for handling contention
   static bool take(std::atomic_flag& flag) noexcept
   {
      return !flag.test_and_set(std::memory_order_acquire);
   }

   flag_type flag;
   this_core::critical_token token{};
};

/**
 * @brief RAII guard for spinlocks
 *
 * Automatically acquires lock on construction and releases on destruction.
 */
class CYROS_PUBLIC spinlock_guard
{
public:
   explicit spinlock_guard(spinlock& lock) : lock(lock)
   {
      lock.lock();
   }

   ~spinlock_guard()
   {
      lock.unlock();
   }

   spinlock_guard(spinlock_guard const&)            = delete;
   spinlock_guard& operator=(spinlock_guard const&) = delete;
   spinlock_guard(spinlock_guard&&)                 = delete;
   spinlock_guard& operator=(spinlock_guard&&)      = delete;

private:
   spinlock& lock;
};

} // namespace cyros

#endif // CYROS_SPINLOCK_HPP
