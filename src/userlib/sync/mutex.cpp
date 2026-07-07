#include <cyros/sync/mutex.hpp>

#include <cyros/kernel/thread.hpp>

namespace cyros::sync
{

bool mutex::is_satisfied(thread& caller) noexcept
{
   thread::id expected = 0;
   return owner.compare_exchange_strong(expected, caller.get_id());
}


bool mutex::try_lock() noexcept
{
   return this_thread::wait_on_any(*this, nbt) == 0;
}

void mutex::lock() noexcept
{
   this_thread::wait_on(*this);
}

void mutex::unlock() noexcept
{
   owner = 0;
   wake_one();
}

} // namespace cyros::sync
