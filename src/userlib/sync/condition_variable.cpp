#include <cyros/sync/condition_variable.hpp>

namespace cyros::sync
{

void condition_variable::notify_one() noexcept
{
   // A notify with nobody armed is lost by definition. A waiter that has not
   // armed yet still holds the mutex, so it has not started waiting.
   wake_one();
}

void condition_variable::notify_all() noexcept
{
   // No requeue onto the mutex: the waiters are woken and each relocks, which
   // costs a wakeup per waiter. Moving them onto the mutex's PI queue without
   // waking them would nest a pi_wait_queue lock under this one and touch the
   // inheritance cache. Measured not to be worth it on one core,
   // arm-port-notes.md 15l.
   wake_all();
}

void condition_variable::wait(base_mutex& m) noexcept
{
   {
      // The record is installed only for the wait itself, not the re-lock.
      waiter self(*this, m);

      // Arms, then polls: the poll releases m, so no notify after the release
      // can miss us. See the class note.
      this_thread::wait_on(*this);
   }

   m.lock();
}

bool condition_variable::try_satisfy(waiter_record* record) noexcept
{
   // Never null: this variable is only waited on from its own members, each
   // of which installs a record naming it, since the waitable base is private.
   auto& self = *static_cast<waiter*>(record);

   if (!self.released) {
      self.released = true;
      // An unlock can hand the mutex over and reach the scheduler. We are
      // armed and prepared, so a reschedule it pends rotates us with that
      // disposition preserved, the same as any preemption mid-wait.
      self.mutex->unlock();
   }
   return self.chosen();
}

}  // namespace cyros::sync
