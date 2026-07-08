#include <cyros/kernel/spinlock.hpp>
#include <cyros/port/port.h>

namespace cyros
{

void spinlock::lock()
{
   // A spinlock disables preemption (so the holder cannot be switched out
   // mid-section) but does NOT mask interrupts.
   token = this_core::disable_preemption();
   while (flag.test_and_set(std::memory_order_acquire)) {
      // busy-wait with cpu yield hint
      cyros_port_cpu_relax();
   }
}

void spinlock::unlock()
{
   flag.clear(std::memory_order_release);
   this_core::enable_preemption(token);
}

} // namespace cyros
