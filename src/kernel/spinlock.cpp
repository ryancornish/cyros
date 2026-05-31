#include <cyros/kernel/spinlock.hpp>
#include <cyros/port/port.h>

namespace cyros
{

void spinlock::lock()
{
   // A spinlock disables preemption (so the holder cannot be switched out
   // mid-section) but does NOT mask interrupts. Preemption control is owned
   // by the port - see port.h "Preemption Control".
   cyros_port_preempt_disable();
   while (flag.test_and_set(std::memory_order_acquire)) {
      // busy-wait with cpu yield hint
      cyros_port_cpu_relax();
   }
}

void spinlock::unlock()
{
   flag.clear(std::memory_order_release);
   // Re-enabling preemption is a contract safe point: if a reschedule was
   // pended while the lock was held, the port resolves it here.
   cyros_port_preempt_enable();
}

} // namespace cyros
