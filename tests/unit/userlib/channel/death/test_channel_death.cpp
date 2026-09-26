/**
 * @file test_channel_death.cpp
 * @brief The strict send() really does stop the system on a full channel (L7).
 *
 * Subject: cyros::ch::channel::send
 *
 * The headline behaviour of `send`, and the only claim in the channel suite
 * that cannot be checked by looking at a return value: the process has to die.
 * This is the first death test in the project, and it is in a binary of its own
 * for a reason that took a while to find.
 *
 * WHY IT WAS ALONE IN ITS BINARY, and why that no longer matters. A death
 * test forks and re-executes, and the child inherits the forking thread's
 * signal mask. Until 2026-09-24 ONE completed kernel lifecycle in the parent
 * left that mask in a state the preempt port's own invariant rejected, so the
 * child panicked inside `kernel::initialise()` before it reached the channel.
 * The port now adopts the thread's mask in `cyros_port_init` instead of
 * assuming it (`preempt-masking-model.md`), and this test now matches the
 * panic's location as well as the signal, so a death for the wrong reason
 * FAILS it rather than passing.
 *
 * HOW THE LOCATION IS SEEN. gtest matches against the child's stderr, and the
 * port's panic goes to stdout. `CYROS_EXPECT_PANIC` (`common/death.hpp`)
 * points the child's stdout at stderr first, then requires both SIGABRT and a
 * panic raised on the strict-send check in `channel.hpp`.
 *
 * Test 8f in the main suite is the other half of the same distinction: a send
 * that loses a race with stop() must NOT die. Together they pin both sides of
 * "full is a sizing bug, finished is not".
 */

#include <cyros/ch/channel.hpp>

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>

#include <common/death.hpp>
#include <common/guarded_stack.hpp>

#include <gtest/gtest.h>

#include <csignal>
#include <cstdint>

using namespace cyros;

namespace
{

/* A free function rather than a block inside the macro, because the macro
 * splits its argument on commas and a lambda's capture and parameter lists are
 * full of them. */
void overfill_a_channel()
{
   kernel::initialise();

   static test::guarded_stack stack;
   static ch::channel<std::uint32_t, 1> ch;

   thread driver(
      [] {
         ch.send(1);   // fills the single slot
         ch.send(2);   // no room, and no policy said that was acceptable
      },
      stack, thread::priority(0), core0);

   kernel::start();
   kernel::finalise();
}

}  // namespace

TEST(ChannelDeath_Test, GivenAFullChannel_WhenStrictSend_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(overfill_a_channel(),
                      test::panicked_at("channel.hpp", "CYROS_REQUIRE1(result != outcome::full"))
      << "send() accepted a value into a full channel instead of stopping the system";
}
