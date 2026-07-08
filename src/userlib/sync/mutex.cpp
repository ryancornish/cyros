#include <cyros/sync/mutex.hpp>

#include <cyros/kernel/thread.hpp>

namespace cyros::sync
{

bool mutex::wait_condition(thread& caller) noexcept
{
   thread::id expected = 0;
   if (owner.compare_exchange_strong(expected, caller.get_id(), std::memory_order_acq_rel)) {
      return true; // free, uncontended take
   }
   // ownership may already have been handed to us by a racing release().
   return expected == caller.get_id();
}

bool mutex::try_lock() noexcept
{
   thread::id expected = 0;
   return owner.compare_exchange_strong(expected, this_thread::id());
}

void mutex::lock() noexcept
{
   this_thread::wait_on(*this);
}

void mutex::unlock() noexcept
{
   wake_one_and_transfer([this](thread::id next_owner_id)
   {
      owner.store(next_owner_id, std::memory_order_release);
   });
}

} // namespace cyros::sync
