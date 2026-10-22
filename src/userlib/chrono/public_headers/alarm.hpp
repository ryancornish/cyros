/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

#ifndef CYROS_CHRONO_ALARM_HPP
#define CYROS_CHRONO_ALARM_HPP

#include <cyros/kernel/waitable.hpp>
#include <cyros/kernel/attribute.hpp>
#include <cyros/time/time.hpp>


namespace cyros::chrono
{

/**
 * @brief A one-shot deadline: a cancel_signal raised by a time:: timer.
 *
 * Expiry is a level, not an event to consume: once fired, any thread that
 * waits on this alarm is satisfied until it is re-armed. That is what makes a
 * fire that lands before the waiter parks harmless, the pre-park poll sees it.
 *
 * this_thread::wait_on(thing, alarm) is a timed wait on the thing, false when
 * the deadline ended it, and this_thread::wait_on(alarm) is a sleep.
 *
 * @note Arm and destroy on the same thread. The timer callback runs on the
 *       arming core, so a same-thread disarm or destroy cannot race it. There
 *       is deliberately no support for destroying an armed alarm from another
 *       thread.
 */
class CYROS_PUBLIC alarm : public cancel_signal
{
public:
   alarm() noexcept = default;

   /**
    * @brief Destroying an armed alarm disarms it first.
    */
   ~alarm();

   /**
    * @brief Arm to fire at @p tp. The alarm must not already be pending.
    *
    * A deadline at or before now fires on the driver's next evaluation, which
    * is the schedule_at contract, not something alarm papers over.
    */
   void arm_at(time::time_point tp) noexcept;

   /**
    * @brief Arm to fire @p d from now. The alarm must not already be pending.
    */
   void arm_in(time::duration d) noexcept;

   /**
    * @brief Return to idle: cancel a pending timer, clear an expired one.
    *
    * Safe in every state. If the timer is mid-fire when the cancel misses
    * (reachable only under the simulation driver, whose pump runs callbacks
    * outside its lock), this waits out the two-statement callback so that
    * returning from disarm always means the callback is finished with this
    * object.
    */
   void disarm() noexcept;

   /**
    * @brief True once the deadline has fired and the alarm has not been
    *        re-armed or disarmed since: raised(), in the alarm's own words.
    */
   [[nodiscard]] bool expired() const noexcept { return raised(); }

private:
   static void on_timer(void* self) noexcept;

   [[nodiscard]] bool pending() const noexcept;

   time::handle timer{};
};

} // namespace cyros::chrono

#endif // CYROS_CHRONO_ALARM_HPP
