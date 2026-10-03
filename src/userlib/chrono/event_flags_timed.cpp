/**
 * @file event_flags_timed.cpp
 * @brief The timed event_flags methods, owned by chrono, declared by sync.
 *
 * Same split as semaphore_timed.cpp: sync is time-free, and the dependency
 * points chrono -> sync.
 */

#include <cyros/chrono/alarm.hpp>
#include <cyros/chrono/chrono.hpp>
#include <cyros/kernel/assert.hpp>
#include <cyros/sync/event_flags.hpp>


namespace cyros::sync
{

std::uint32_t event_flags::try_wait_until(std::uint32_t mask, time::time_point tp,
                                          flags_match match, flags_exit exit) noexcept
{
   // An expired deadline degrades to the non-blocking test. Without this the
   // alarm would not fire until the driver's next evaluation, which would
   // stretch "already too late" into "up to one tick late".
   if (!(time::now() < tp)) {
      return try_wait(mask, match, exit);
   }
   CYROS_REQUIRE(mask != 0);

   chrono::alarm deadline;
   deadline.arm_at(tp);

   // The flags are polled before the alarm on every pass, so bits that are
   // set win over a deadline that fired in the same pass.
   std::size_t index = 0;
   std::uint32_t matched = 0;
   {
      waiter self(*this, request{mask, match, exit});
      index = this_thread::wait_on_any(static_cast<waitable&>(*this), deadline);
      matched = self.req.matched;
   }
   deadline.disarm();

   return index == 0 ? matched : 0;
}

std::uint32_t event_flags::try_wait_for(std::uint32_t mask, time::duration d,
                                        flags_match match, flags_exit exit) noexcept
{
   if (d.value == 0) {
      return try_wait(mask, match, exit);
   }
   return try_wait_until(mask, time::now() + d, match, exit);
}

} // namespace cyros::sync
