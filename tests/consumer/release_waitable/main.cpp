/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * An application's own waitable, linked against a RELEASE archive.
 *
 * waitable is the extension point for blocking on something cyros does not
 * know about. This project derives one and waits on it, against the archive a
 * release ships: LTO across cyros, then every hidden symbol of the merged
 * object made local.
 *
 * It exists because that link once failed outright. waitable's one virtual
 * with a body was defined in its header, so the archive and this application
 * each emitted a copy, the linker kept the application's and discarded the
 * archive's, and the archive's own references, made local, pointed into the
 * discarded section ("defined in discarded section"). Building, linking and
 * running at all is most of the test; the wake is the rest. A cancel_signal,
 * cyros's own derived waitable, is built and waited on here too, for the same
 * reason: its vtable must link from this side of a release archive.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/waitable.hpp>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdio>

using namespace cyros;

namespace
{

class flag final : public waitable
{
public:
   void raise() noexcept
   {
      up.store(true, std::memory_order_release);
      wake_one();
   }

protected:
   bool try_satisfy(waiter_record*) noexcept override
   {
      return up.load(std::memory_order_acquire);
   }

private:
   std::atomic<bool> up{false};
};

flag ready;
flag never;
cancel_signal stop;
alignas(16) std::array<std::byte, 64 * 1024> waiter_stack;
alignas(16) std::array<std::byte, 64 * 1024> canceller_stack;
alignas(16) std::array<std::byte, 64 * 1024> raiser_stack;
std::atomic<bool> woken{false};
std::atomic<bool> cancelled{false};

} // namespace

int main()
{
   kernel::initialise();

   // Both waiters outrank the raiser on one core, so they are parked before
   // it runs, and only its wake and its cancel can let them finish.
   thread waiter([]{ this_thread::wait_on(ready); woken.store(true); },
                 waiter_stack, thread::priority(1), core0);
   thread canceller([]{ cancelled.store(!this_thread::wait_on(never, stop)); },
                    canceller_stack, thread::priority(1), core0);
   thread raiser([]{ ready.raise(); stop.raise(); },
                 raiser_stack, thread::priority(2), core0);

   kernel::start();
   kernel::finalise();

   bool const ok = woken.load() && cancelled.load();
   std::printf("release_waitable: %s, %s\n", woken.load() ? "woken" : "NOT WOKEN",
               cancelled.load() ? "cancelled" : "NOT CANCELLED");
   return ok ? 0 : 1;
}
