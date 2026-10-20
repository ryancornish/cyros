/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

#include <cyros/sync/event_flags.hpp>
#include <cyros/kernel/assert.hpp>
#include <cyros/kernel/thread.hpp>

namespace cyros::sync
{

std::uint32_t event_flags::set(std::uint32_t mask) noexcept
{
   // Release pairs with the acquire in attempt(), so a waiter satisfied by
   // these bits sees whatever the setter wrote before setting them.
   auto const before = bits.fetch_or(mask, std::memory_order_release);

   // Wake only the waiters the flags now satisfy, each asked exactly as its
   // poll asks, so the rest stay parked and never run. The flags are read
   // afresh for each waiter: a set racing this one can only add bits, and a
   // clear or a consume only takes away bits whose waiters' polls would now
   // fail as well.
   wake_all_where([this](waiter_record const* record) noexcept {
      // Never null, as in try_satisfy: every wait here installs a record.
      return satisfied_by(static_cast<waiter const*>(record)->req,
                          bits.load(std::memory_order_acquire));
   });
   return before;
}

std::uint32_t event_flags::clear(std::uint32_t mask) noexcept
{
   return bits.fetch_and(~mask, std::memory_order_acq_rel);
}

std::uint32_t event_flags::peek() const noexcept
{
   return bits.load(std::memory_order_acquire);
}

std::uint32_t event_flags::wait(std::uint32_t mask, flags_match match, flags_exit exit) noexcept
{
   CYROS_REQUIRE(mask != 0);

   waiter self(*this, request{mask, match, exit});
   this_thread::wait_on(*this);
   return self.req.matched;
}

std::uint32_t event_flags::try_wait(std::uint32_t mask, flags_match match, flags_exit exit) noexcept
{
   CYROS_REQUIRE(mask != 0);

   request r{mask, match, exit};
   return attempt(r) ? r.matched : 0;
}

bool event_flags::attempt(request& self) noexcept
{
   auto current = bits.load(std::memory_order_acquire);
   while (true) {
      if (!satisfied_by(self, current)) {
         return false;
      }
      auto const hit = current & self.mask;
      if (self.exit == flags_exit::keep) {
         self.matched = hit;
         return true;
      }
      // Clear exactly the bits that satisfied us, in the same step as the
      // test, so two consumers cannot both take one set.
      if (bits.compare_exchange_weak(current, current & ~hit,
                                     std::memory_order_acq_rel,
                                     std::memory_order_acquire)) {
         self.matched = hit;
         return true;
      }
   }
}

bool event_flags::try_satisfy(waiter_record* record) noexcept
{
   // Never null: this is only waited on from its own members, each of which
   // installs a record naming it, since the waitable base is private.
   return attempt(static_cast<waiter*>(record)->req);
}

}  // namespace cyros::sync
