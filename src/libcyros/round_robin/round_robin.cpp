#include <cyros/rr/round_robin.hpp>

#include <cyros/kernel/kernel.hpp>
#include <cyros/time/time.hpp>

namespace cyros::rr
{

void setup_round_robin()
{
   (void)time::schedule_at(time::now() + time::duration(10),
                           +[](void*)
                           {
                              cyros::kernel::pend_reschedule();
                           }, nullptr);
}

} // namespace cyros::rr
