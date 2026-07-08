#include <cyros/rr/round_robin.hpp>

#include <cyros/kernel/core.hpp>
#include <cyros/time/time.hpp>

namespace cyros::rr
{

/**
 * @brief Timer callback: rotate the calling core's run queue.
 *
 * Fires in timer ISR context on the core that armed the slice. Pending a
 * reschedule re-enqueues the running thread behind its equal-priority peers and
 * picks the next, which is the rotation. The reschedule is delivered when the ISR
 * returns, so the switch happens off the ISR, not inside it.
 */
static void rotate(void*) noexcept
{
   this_core::pend_reschedule();
}

void enable_round_robin(time::duration slice)
{
   // The handle is intentionally discarded: round-robin runs for the life of the
   // system and is torn down with the time driver, not cancelled piecemeal.
   (void)time::schedule_recurring(slice, &rotate, nullptr);
}

} // namespace cyros::rr
