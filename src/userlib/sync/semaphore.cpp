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
   wake_all(); // Don't think this is optimal... n = 2 with 10 waiters means 8 spurious wakes
}

void semaphore::acquire() noexcept
{
   counter.fetch_sub(1, std::memory_order_acquire);
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

bool semaphore::wait_condition(thread& caller) noexcept
{
   (void)caller;
   return try_acquire();
}

}  // namespace cyros::sync
