#ifndef CYROS_MUTEX_HPP
#define CYROS_MUTEX_HPP

#include <cyros/kernel/waitable.hpp>
#include <cyros/kernel/thread.hpp>

#include <atomic>

namespace cyros::time
{
struct time_point;
struct duration;
}

namespace cyros::sync
{

class mutex : public waitable
{
public:
   [[nodiscard]] bool try_lock() noexcept;

   void lock() noexcept;

   void unlock() noexcept;

   bool try_lock_for(time::time_point tp) noexcept;

   bool try_lock_until(time::duration d) noexcept;

protected:
   bool wait_condition(thread& caller) noexcept override;

private:
   std::atomic<thread::id> owner{0};
};

}  // namespace cyros::sync




#endif // CYROS_MUTEX_HPP
