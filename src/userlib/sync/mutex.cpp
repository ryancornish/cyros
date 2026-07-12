#include <cyros/sync/mutex.hpp>

#include <cyros/kernel/thread.hpp>

namespace cyros::sync
{

bool mutex::wait_condition(thread& caller) noexcept
{
   return pi_acquire_condition(caller);
}

bool mutex::try_lock() noexcept
{
   return pi_try_acquire();
}

void mutex::lock() noexcept
{
   this_thread::wait_on(*this);
}

void mutex::unlock() noexcept
{
   pi_release();
}

} // namespace cyros::sync
