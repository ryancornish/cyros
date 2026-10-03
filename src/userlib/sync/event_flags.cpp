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
   wake_all();
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
      auto const hit = current & self.mask;
      bool const satisfied = self.match == flags_match::any ? hit != 0 : hit == self.mask;
      if (!satisfied) {
         return false;
      }
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
