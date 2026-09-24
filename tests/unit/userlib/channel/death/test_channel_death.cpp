/**
 * @file test_channel_death.cpp
 * @brief The strict send() really does stop the system on a full channel (L7).
 *
 * Subject: cyros::chan::channel::send
 *
 * The headline behaviour of `send`, and the only claim in the channel suite
 * that cannot be checked by looking at a return value: the process has to die.
 * This is the first death test in the project, and it is in a binary of its own
 * for a reason that took a while to find.
 *
 * WHY IT CANNOT SHARE A BINARY. A death test forks and re-executes, and the
 * child inherits the forking thread's signal mask. On the preempt port ONE
 * completed kernel lifecycle in the parent is enough to leave that mask in a
 * state the port's own invariant rejects, so the child panics inside
 * `kernel::initialise()` at `port_linux_preempt.cpp:554` before it ever reaches
 * the channel. The death then happens for the wrong reason, which is worse than
 * no test at all. Measured 2026-09-23 on the Arch box: with zero prior tests
 * the right panic fires, with one prior test the wrong one does.
 *
 * That is a finding about the PORT, not about the channel, and it is written up
 * in `channel-proposal.md` section 16i. Keeping this test alone sidesteps it
 * without hiding it.
 *
 * WHY THE REGEX IS EMPTY. gtest matches a death test's message against the
 * child's STDERR. The linux port's panic goes to stdout (`std::printf` in
 * `port_linux_common.cpp:97`), so there is nothing on stderr to match beyond
 * the abort itself. `KilledBySignal(SIGABRT)` is therefore the assertion, and
 * it is a stronger one than a text match would be: it says the process aborted,
 * not merely that it exited.
 *
 * Test 8f in the main suite is the other half of the same distinction: a send
 * that loses a race with stop() must NOT die. Together they pin both sides of
 * "full is a sizing bug, finished is not".
 */

#include <cyros/chan/channel.hpp>

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>

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
   static chan::channel<std::uint32_t, 1> ch;

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
   /* Re-executes the binary for the child. The default `fast` style forks the
    * already-threaded process, which is exactly the unsafe thing once a kernel
    * has spawned core threads. Roadmap A2 section 8 step 4 asks for this style
    * for the same reason. */
   GTEST_FLAG_SET(death_test_style, "threadsafe");

   EXPECT_EXIT(overfill_a_channel(), ::testing::KilledBySignal(SIGABRT), "")
      << "send() accepted a value into a full channel instead of stopping the system";
}
