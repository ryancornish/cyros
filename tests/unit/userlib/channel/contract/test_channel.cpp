/* ============================================================================
 * channel contract
 *
 * Subject: `cyros::chan::channel` (L7). Trusts spinlock (L1), semaphore
 * (L6) and function (L0), all proved below, plus the harness floor to run
 * cores at all.
 *
 * The suite is organised by what each send variant PROMISES, because the whole
 * design rests on overflow policy belonging to the producer. So there is one
 * group per variant, and then the accounting tests that all of them share.
 *
 * EVERY RENDEZVOUS HERE BLOCKS, it does not spin. An earlier version bounded
 * each wait with a spin budget so that a broken claim would report as itself,
 * and under a 3-way load on a 4 core box that budget was what broke: the tests
 * failed about one run in seven for reasons that had nothing to do with the
 * channel. A budget large enough to be safe under load is too large to be a
 * useful bound anyway. So waits are semaphores, and a claim that fails by not
 * happening hangs.
 *
 * The one exception is the WATCHDOG, which guards the shutdown tests. A parked
 * thread cannot be freed from outside, so a stop() that fails to wake one
 * wedges the binary and the suite reports a bare 60 second timeout naming
 * nothing. There a bounded spin earns its place, because it turns that into a
 * message saying which claim broke. It gets core3 to ITSELF, and the comment on
 * `watchdog_core` below says why that is not optional.
 * ========================================================================= */

#include <cyros/chan/channel.hpp>

#include <cyros/config/config.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/port/port_traits.h>

#include <common/guarded_stack.hpp>

#include "gtest/gtest.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>

using namespace cyros;

static_assert(config::cores >= 4, "Test suite is designed for (at least) quad-core configuration");

/* A channel must stay constant-initialisable, so a static one lands in .bss
 * with no startup code and no initialisation-order surface. Both members that
 * could have broken it are the reason: spinlock and semaphore are both
 * constexpr-constructible, and std::array<T, N> value-initialises. */
constinit static chan::channel<std::uint32_t, 4> constant_initialised_channel;

namespace
{

/* Long enough that it cannot expire while the shutdown is merely slow, short
 * enough to report in about a second when it is genuinely wedged. It costs
 * nothing when the shutdown works, because the predicate is already true the
 * first time it is read. */
constexpr std::uint32_t wedge_budget = 20'000'000;

/* THE WATCHDOG'S CORE MUST HOST NOTHING THAT WILL RUN AGAIN. A watchdog spins,
 * `cpu_relax()` is a hardware pause and never yields, and on the COOPERATIVE
 * port a spinning thread never gives its core back. Put it on a core with a
 * thread that is merely parked and the watchdog starves exactly the thread it
 * is waiting for, then reports the deadlock it caused itself. The port supports
 * four cores, so core3 is reserved for it and the shutdown tests below are
 * sized to fit in the other three. */
inline constexpr core_affinity watchdog_core = core3;

template<typename Predicate>
void fail_fast_if_wedged(char const* claim, Predicate&& done) noexcept
{
   for (std::uint32_t i = 0; i < wedge_budget; ++i) {
      if (done()) { return; }
      this_core::cpu_relax();
   }
   std::fprintf(stderr, "\n*** channel test wedged: %s ***\n", claim);
   std::fflush(stderr);
   std::abort();
}

class Channel_Test : public ::testing::Test {};

}  // namespace

/* ============================================================================
 * 1. FIFO, and the shape of the ring
 *
 * One producer, one receiver, entirely inside one thread so nothing about
 * ordering is left to the scheduler. This is the test that would catch an
 * advance() that skipped a slot or a head and tail that drifted apart, which
 * is why it sends more than Capacity values and wraps the ring several times.
 * ========================================================================= */
TEST_F(Channel_Test, GivenOneProducerAndOneReceiver_WhenValuesAreSent_ThenTheyComeBackInOrder)
{
   kernel::initialise();

   test::guarded_stack driver_stack;

   struct state
   {
      chan::channel<std::uint32_t, 4> ch;
      std::atomic<bool>        order_held{true};
      std::atomic<std::uint32_t> received{0};
      std::atomic<bool>        empty_when_drained{false};
   } s;

   thread driver(
      [&s] {
         std::uint32_t expected = 0;
         // Three full wraps of a four-slot ring, two in flight at a time, so
         // head and tail are never equal at the same point twice.
         for (std::uint32_t round = 0; round < 6; ++round) {
            EXPECT_TRUE(s.ch.try_send(round * 2));
            EXPECT_TRUE(s.ch.try_send(round * 2 + 1));

            for (int i = 0; i < 2; ++i) {
               auto const got = s.ch.try_receive();
               if (!got || *got != expected) { s.order_held.store(false); }
               ++expected;
               s.received.fetch_add(1);
            }
         }
         s.empty_when_drained.store(!s.ch.try_receive().has_value());
      },
      driver_stack, thread::priority(0), core0);

   kernel::start();

   EXPECT_TRUE(s.order_held.load())         << "values did not come back in send order";
   EXPECT_EQ(s.received.load(), 12u)        << "not every sent value was received";
   EXPECT_TRUE(s.empty_when_drained.load()) << "a drained channel still yielded a value";

   kernel::finalise();
}

/* ============================================================================
 * 2. try_send refuses EXACTLY when full
 *
 * Exactly is the claim: not one short, not one over. An off-by-one in the
 * space semaphore's initial count or in advance() shows up here as a channel
 * that holds Capacity-1 or Capacity+1.
 * ========================================================================= */
TEST_F(Channel_Test, GivenAChannel_WhenFilled_ThenTrySendRefusesExactlyAtCapacity)
{
   kernel::initialise();

   test::guarded_stack driver_stack;

   struct state
   {
      chan::channel<std::uint32_t, 3> ch;
      std::atomic<int>  accepted{0};
      std::atomic<bool> refused_when_full{false};
      std::atomic<bool> accepted_after_one_freed{false};
      std::atomic<std::size_t> size_when_full{0};
   } s;

   thread driver(
      [&s] {
         int accepted = 0;
         for (int i = 0; i < 3; ++i) {
            if (s.ch.try_send(static_cast<std::uint32_t>(i))) { ++accepted; }
         }
         s.accepted.store(accepted);
         s.size_when_full.store(s.ch.size());
         s.refused_when_full.store(!s.ch.try_send(99));

         (void)s.ch.try_receive();  // free exactly one slot
         s.accepted_after_one_freed.store(s.ch.try_send(99));
      },
      driver_stack, thread::priority(0), core0);

   kernel::start();

   EXPECT_EQ(s.accepted.load(), 3)                     << "a channel of 3 did not take 3 values";
   EXPECT_EQ(s.size_when_full.load(), 3u)              << "size() disagrees with what was accepted";
   EXPECT_TRUE(s.refused_when_full.load())             << "try_send accepted a 4th value into a channel of 3";
   EXPECT_TRUE(s.accepted_after_one_freed.load())      << "freeing a slot did not make room";

   kernel::finalise();
}

/* ============================================================================
 * 3. send_overwrite drops the OLDEST, and only the oldest
 *
 * The survivors must stay in FIFO order. A naive "write over the head and
 * leave the indices alone" passes a size check and fails this one.
 * ========================================================================= */
TEST_F(Channel_Test, GivenAFullChannel_WhenSendOverwrite_ThenTheOldestIsDroppedAndTheRestKeepOrder)
{
   kernel::initialise();

   test::guarded_stack driver_stack;

   struct state
   {
      chan::channel<std::uint32_t, 3> ch;
      std::array<std::uint32_t, 3> drained{};
      std::atomic<std::size_t> size_after_overwrite{0};
      std::atomic<int> drained_count{0};
   } s;

   thread driver(
      [&s] {
         s.ch.send(10);
         s.ch.send(11);
         s.ch.send(12);          // full: 10, 11, 12

         s.ch.send_overwrite(13); // drops 10, leaving 11, 12, 13
         s.size_after_overwrite.store(s.ch.size());

         int n = 0;
         while (auto got = s.ch.try_receive()) {
            if (n < 3) { s.drained[static_cast<std::size_t>(n)] = *got; }
            ++n;
         }
         s.drained_count.store(n);
      },
      driver_stack, thread::priority(0), core0);

   kernel::start();

   EXPECT_EQ(s.size_after_overwrite.load(), 3u) << "overwrite changed the occupancy of a full channel";
   EXPECT_EQ(s.drained_count.load(), 3)         << "a channel of 3 did not yield exactly 3 values";
   EXPECT_EQ(s.drained[0], 11u) << "the oldest survivor is wrong, so the wrong value was dropped";
   EXPECT_EQ(s.drained[1], 12u) << "survivors lost FIFO order";
   EXPECT_EQ(s.drained[2], 13u) << "the newly written value is not last";

   kernel::finalise();
}

/* ============================================================================
 * 4. send_overwrite on a channel that is NOT full is an ordinary send
 * ========================================================================= */
TEST_F(Channel_Test, GivenRoomToSpare_WhenSendOverwrite_ThenNothingIsDropped)
{
   kernel::initialise();

   test::guarded_stack driver_stack;

   struct state
   {
      chan::channel<std::uint32_t, 4> ch;
      std::atomic<std::size_t> size_after{0};
      std::atomic<std::uint32_t> first{0};
      std::atomic<int> count{0};
   } s;

   thread driver(
      [&s] {
         s.ch.send_overwrite(1);
         s.ch.send_overwrite(2);
         s.size_after.store(s.ch.size());

         int n = 0;
         std::uint32_t first = 0;
         while (auto got = s.ch.try_receive()) {
            if (n == 0) { first = *got; }
            ++n;
         }
         s.first.store(first);
         s.count.store(n);
      },
      driver_stack, thread::priority(0), core0);

   kernel::start();

   EXPECT_EQ(s.size_after.load(), 2u) << "overwrite dropped something on a channel with room";
   EXPECT_EQ(s.count.load(), 2)       << "not both values survived";
   EXPECT_EQ(s.first.load(), 1u)      << "the older value was dropped despite there being room";

   kernel::finalise();
}

/* ============================================================================
 * 5. stop() drains what is queued and refuses what comes after
 * ========================================================================= */
TEST_F(Channel_Test, GivenQueuedValues_WhenStopped_ThenTheyStillArriveAndLaterSendsAreRefused)
{
   kernel::initialise();

   test::guarded_stack driver_stack;

   /* Capacity is 2 and exactly 2 values are queued, so the channel is FULL when
    * it is stopped. That matters: with room to spare a post-stop
    * send_overwrite is turned away by the space token and never reaches the
    * code that replaces the oldest, which is the path that has to honour the
    * stop. */
   struct state
   {
      chan::channel<std::uint32_t, 2> ch;
      std::atomic<int>  drained{0};
      std::atomic<bool> order_held{true};
      std::atomic<bool> send_refused{false};
      std::atomic<bool> overwrite_refused{false};
      std::atomic<bool> empty_at_end{false};
      std::atomic<bool> reported_stopped{false};
   } s;

   thread driver(
      [&s] {
         s.ch.send(1);
         s.ch.send(2);
         s.ch.stop();
         s.reported_stopped.store(s.ch.is_stopped());

         // Refused, and NOT a hard error: shutdown racing a send is a
         // lifecycle race, not a sizing bug.
         s.send_refused.store(!s.ch.try_send(3));
         s.ch.send(4);                      // must not panic
         s.ch.send_overwrite(5);            // full and stopped: must change nothing
         s.overwrite_refused.store(s.ch.size() == 2);

         std::uint32_t expected = 1;
         while (auto got = s.ch.receive()) {
            if (*got != expected) { s.order_held.store(false); }
            ++expected;
            s.drained.fetch_add(1);
         }
         s.empty_at_end.store(s.ch.size() == 0);
      },
      driver_stack, thread::priority(0), core0);

   kernel::start();

   EXPECT_TRUE(s.reported_stopped.load())  << "is_stopped() did not report the stop";
   EXPECT_EQ(s.drained.load(), 2)          << "stop() did not deliver the values already queued";
   EXPECT_TRUE(s.order_held.load())
      << "draining after stop did not yield exactly the values queued before it, so a send "
         "after stop changed the contents";
   EXPECT_TRUE(s.send_refused.load())      << "try_send succeeded on a stopped channel";
   EXPECT_TRUE(s.overwrite_refused.load()) << "a send after stop still changed the channel";
   EXPECT_TRUE(s.empty_at_end.load())      << "the channel was not empty after draining";

   kernel::finalise();
}

/* ============================================================================
 * 6. stop() releases every BLOCKED receiver
 *
 * The accounting test. stop() does not own the worker threads, so it has to
 * count them itself, and the count has to be right for one, two and three
 * workers alike. A stop that woke only the first would hang here, so the
 * result is gathered with a bounded wait and reported by name.
 *
 * Run as a loop over worker counts rather than three tests, because what is
 * being checked is that the SAME code settles for each of them.
 * ========================================================================= */
TEST_F(Channel_Test, GivenBlockedReceivers_WhenStopped_ThenEveryOneOfThemReturns)
{
   for (int workers = 1; workers <= 2; ++workers) {
      SCOPED_TRACE("worker count " + std::to_string(workers));

      kernel::initialise();

      static std::array<test::guarded_stack, 4> stacks;

      struct state
      {
         chan::channel<std::uint32_t, 4> ch;
         sync::semaphore   parked{0};   // one release per witness
         std::atomic<int>  returned{0};
         std::atomic<bool> all_parked{false};
      } s;

      std::array<std::optional<thread>, 2> receivers;
      std::array<std::optional<thread>, 2> witnesses;
      static std::array<test::guarded_stack, 3> extra_stacks;

      for (int i = 0; i < workers; ++i) {
         receivers[static_cast<std::size_t>(i)].emplace(
            [&s] {
               // Empty and not stopped, so this must block.
               auto const got = s.ch.receive();
               if (!got) { s.returned.fetch_add(1); }
            },
            stacks[static_cast<std::size_t>(i)],
            thread::priority(1),
            core_affinity::from_id(static_cast<std::uint32_t>(i)));

         /* Lower priority on the receiver's OWN core, so it can only run once
          * the receiver has vacated that core, which happens exactly when the
          * receiver parks inside receive(). A flag set by the receiver itself
          * would only prove it entered the call, and stop() would then usually
          * win the race and be answered by the cheap pre-check instead of by
          * the wake path this test exists to exercise. */
         witnesses[static_cast<std::size_t>(i)].emplace(
            [&s] { s.parked.release(); },
            extra_stacks[static_cast<std::size_t>(i)],
            thread::priority(2),
            core_affinity::from_id(static_cast<std::uint32_t>(i)));
      }

      thread stopper(
         [&s, workers] {
            // Blocks rather than spins. A witness that never runs hangs the
            // test, which the watchdog below reports by name.
            for (int i = 0; i < workers; ++i) { s.parked.acquire(); }
            s.all_parked.store(true);
            s.ch.stop();
         },
         stacks[3], thread::priority(0), core2);

      thread watchdog(
         [&s, workers] {
            fail_fast_if_wedged("stop() left a blocked receiver parked forever",
                                [&] { return s.returned.load() == workers; });
         },
         extra_stacks[2], thread::priority(2), watchdog_core);

      kernel::start();

      EXPECT_TRUE(s.all_parked.load())        << "receivers never all parked inside receive()";
      EXPECT_EQ(s.returned.load(), workers)   << "stop() did not release every blocked receiver";

      kernel::finalise();
   }
}

/* ============================================================================
 * 7. send_blocking waits for space, and gets it when a receiver frees a slot
 *
 * Two separate claims, and each needs its own evidence.
 *
 * That it PARKED is shown by a witness thread at lower priority on the
 * sender's own core, which cannot run until the sender gives that core up.
 *
 * That it did not return UNTIL a slot was freed is shown by the receiver
 * reading `sender_done` immediately BEFORE the receive that frees one. If the
 * sender had already finished by then, send_blocking returned without waiting.
 * An earlier version of this test read the flag from the wrong side, after the
 * receive, which is a race the sender wins about one run in seven under load.
 * ========================================================================= */
TEST_F(Channel_Test, GivenAFullChannel_WhenSendBlocking_ThenItCompletesOnlyAfterASlotIsFreed)
{
   kernel::initialise();

   static std::array<test::guarded_stack, 3> stacks;

   struct state
   {
      chan::channel<std::uint32_t, 1> ch;
      sync::semaphore   sender_parked{0};
      std::atomic<bool> sender_done{false};
      std::atomic<bool> sender_done_before_receive{true};
      std::atomic<bool> receiver_saw_first{false};
      std::atomic<bool> witness_ran{false};
   } s;

   thread sender(
      [&s] {
         s.ch.send(1);            // fills the single slot
         s.ch.send_blocking(2);   // must park: no room, and the witness proves it
         s.sender_done.store(true);
      },
      stacks[0], thread::priority(0), core0);

   // Lower priority on the sender's OWN core, so reaching this at all means
   // the sender parked inside send_blocking and vacated the core.
   thread witness(
      [&s] {
         s.witness_ran.store(true);
         s.sender_parked.release();
      },
      stacks[1], thread::priority(1), core0);

   thread receiver(
      [&s] {
         s.sender_parked.acquire();

         // Read BEFORE freeing a slot. This is the ordering assertion.
         s.sender_done_before_receive.store(s.sender_done.load());

         auto const first = s.ch.receive();
         s.receiver_saw_first.store(first.has_value() && *first == 1);

         auto const second = s.ch.receive();   // the value that was waiting
         (void)second;
      },
      stacks[2], thread::priority(0), core1);

   kernel::start();

   EXPECT_TRUE(s.witness_ran.load())
      << "send_blocking never gave up the core, so it did not park";
   EXPECT_FALSE(s.sender_done_before_receive.load())
      << "send_blocking had already returned before any slot was freed";
   EXPECT_TRUE(s.receiver_saw_first.load())  << "the receiver did not get the first value";
   EXPECT_TRUE(s.sender_done.load())         << "send_blocking never completed";

   kernel::finalise();
}

/* ============================================================================
 * 8. Every value is received exactly once, many producers and many receivers
 *
 * The accounting shape of the semaphore producer/consumer test. Four cores,
 * two producers and two receivers, a channel far smaller than the traffic, so
 * the ring wraps thousands of times and both semaphores are driven to their
 * limits in both directions.
 *
 * What it would catch: a lost value (sum short), a duplicated value (sum long
 * or a repeat), a lost wake (hang, reported by the bounded wait).
 * ========================================================================= */
TEST_F(Channel_Test, GivenManyProducersAndReceivers_WhenDraining_ThenEveryValueArrivesExactlyOnce)
{
   kernel::initialise();

   static std::array<test::guarded_stack, 4> stacks;

   constexpr std::uint32_t per_producer = 4'000;
   constexpr std::uint32_t producers    = 2;

   struct state
   {
      chan::channel<std::uint32_t, 8> ch;
      std::atomic<std::uint64_t> sum_sent{0};
      std::atomic<std::uint64_t> sum_received{0};
      std::atomic<std::uint32_t> count_received{0};
      std::atomic<std::uint32_t> producers_done{0};
   } s;

   /* The LAST producer to finish closes the channel. That is the ordering every
    * shutdown wants, stop producing then stop the channel, and doing it inside
    * a producer rather than in a watcher thread avoids a spin-wait that would
    * have to live on somebody's core and starve them there. */
   std::array<std::optional<thread>, 2> senders;
   for (std::uint32_t p = 0; p < producers; ++p) {
      senders[p].emplace(
         [&s, p] {
            for (std::uint32_t i = 1; i <= per_producer; ++i) {
               std::uint32_t const value = (p * per_producer) + i;
               s.sum_sent.fetch_add(value);
               s.ch.send_blocking(value);
            }
            if (s.producers_done.fetch_add(1) + 1 == producers) { s.ch.stop(); }
         },
         stacks[p], thread::priority(1), core_affinity::from_id(p));
   }

   std::array<std::optional<thread>, 2> drainers;
   for (std::uint32_t r = 0; r < 2; ++r) {
      drainers[r].emplace(
         [&s] {
            while (auto got = s.ch.receive()) {
               s.sum_received.fetch_add(*got);
               s.count_received.fetch_add(1);
            }
         },
         stacks[2 + r], thread::priority(1), core_affinity::from_id(2 + r));
   }

   kernel::start();

   EXPECT_EQ(s.count_received.load(), producers * per_producer)
      << "the number of values received does not match the number sent";
   EXPECT_EQ(s.sum_received.load(), s.sum_sent.load())
      << "a value was lost or delivered twice";

   kernel::finalise();
}

/* ============================================================================
 * 8b. send_overwrite under a CONCURRENT receiver
 *
 * The single-threaded overwrite tests cannot reach the case that matters,
 * because there the ring and the semaphores never disagree. This one can.
 *
 * What it is really checking: when send_overwrite drops the oldest value, it
 * must take that value's `items` token with it. A token that outlives the value
 * behind it leaves a receiver waking to an empty ring on a channel that is
 * still open, and `receive()` then reports the channel finished when it is not.
 * So the assertion is not about which values arrive, since overwrite is lossy
 * by design and nothing here can predict what survives. It is that the receiver
 * NEVER stops early, and that the survivors stay in order.
 * ========================================================================= */
TEST_F(Channel_Test, GivenAConcurrentReceiver_WhenOverwritingHard_ThenNoTokenOutlivesItsValue)
{
   kernel::initialise();

   static std::array<test::guarded_stack, 2> stacks;

   constexpr std::uint32_t sends      = 20'000;
   constexpr std::uint32_t quiet_spin = 1'000'000;

   struct state
   {
      chan::channel<std::uint32_t, 4> ch;
      std::atomic<bool> producer_done{false};
      std::atomic<bool> receiver_exited_early{false};
      std::atomic<bool> order_held{true};
      std::atomic<bool> all_values_plausible{true};
      std::atomic<std::uint32_t> got{0};
      std::atomic<std::size_t> size_at_end{1};
   } s;

   thread producer(
      [&s] {
         for (std::uint32_t i = 1; i <= sends; ++i) {
            s.ch.send_overwrite(i);
         }

         /* THE QUIET PERIOD IS THE TEST. During the storm the ring is almost
          * always full, so a token with no value behind it still finds one and
          * hides. Stopping the sends lets the receiver catch up, drain to
          * empty, and block. A surplus token surfaces exactly here, as a
          * receive() that reports the channel finished while it is still open.
          * Without this pause the surplus only surfaces after the stop, where
          * an early return is indistinguishable from a correct one. */
         for (std::uint32_t i = 0; i < quiet_spin; ++i) { this_core::cpu_relax(); }

         s.producer_done.store(true);
         s.ch.stop();
      },
      stacks[0], thread::priority(1), core0);

   thread receiver(
      [&s] {
         std::uint32_t previous = 0;
         while (auto value = s.ch.receive()) {
            if (*value <= previous)  { s.order_held.store(false); }
            if (*value > sends)      { s.all_values_plausible.store(false); }
            previous = *value;
            s.got.fetch_add(1);
         }
         // The only legitimate reason to be here is that the channel was
         // stopped. Anything else is a token that outlived its value.
         if (!s.producer_done.load()) { s.receiver_exited_early.store(true); }
         s.size_at_end.store(s.ch.size());
      },
      stacks[1], thread::priority(1), core1);

   kernel::start();

   EXPECT_FALSE(s.receiver_exited_early.load())
      << "receive() reported the channel finished while the producer was still sending, "
         "so an items token outlived the value it stood for";
   EXPECT_TRUE(s.order_held.load())           << "surviving values were delivered out of order";
   EXPECT_TRUE(s.all_values_plausible.load()) << "a value came back that was never sent";
   EXPECT_GT(s.got.load(), 0u)                << "nothing at all got through";
   EXPECT_LE(s.got.load(), sends)             << "more values arrived than were ever sent";
   EXPECT_EQ(s.size_at_end.load(), 0u)        << "the channel was not empty after draining";

   kernel::finalise();
}

/* ============================================================================
 * 8c. stop() releases every BLOCKED SENDER, not just the first
 *
 * The sending half of the relay, and the mirror of test 6. Three senders are
 * blocked on a full channel of one slot when it is stopped, and all three must
 * return. A stop that signalled once would release one of them and hang the
 * other two.
 * ========================================================================= */
TEST_F(Channel_Test, GivenSeveralBlockedSenders_WhenStopped_ThenEveryOneOfThemReturns)
{
   kernel::initialise();

   static std::array<test::guarded_stack, 4> stacks;

   /* Two, not three. One relay hop is what proves the chain, and core3 is the
    * watchdog's. */
   constexpr int blocked = 2;

   struct state
   {
      chan::channel<std::uint32_t, 1> ch;
      sync::semaphore   filled{0};    // one release per sender
      sync::semaphore   parked{0};    // one release per witness
      std::atomic<int>  returned{0};
      std::atomic<bool> all_entered{false};
   } s;

   thread filler(
      [&s] {
         s.ch.send(1);                  // the single slot is now full
         s.filled.release(blocked);     // release the senders

         for (int i = 0; i < blocked; ++i) { s.parked.acquire(); }
         s.all_entered.store(true);
         s.ch.stop();
      },
      stacks[0], thread::priority(0), core0);

   static test::guarded_stack watchdog_stack;
   thread watchdog(
      [&s] {
         fail_fast_if_wedged("stop() left a blocked sender parked forever",
                             [&] { return s.returned.load() == blocked; });
      },
      watchdog_stack, thread::priority(2), watchdog_core);

   static std::array<test::guarded_stack, blocked> witness_stacks;
   std::array<std::optional<thread>, blocked> senders;
   std::array<std::optional<thread>, blocked> witnesses;

   for (int i = 0; i < blocked; ++i) {
      senders[static_cast<std::size_t>(i)].emplace(
         [&s] {
            s.filled.acquire();
            s.ch.send_blocking(7);   // no room, and none is coming
            s.returned.fetch_add(1);
         },
         stacks[static_cast<std::size_t>(i) + 1],
         thread::priority(1),
         core_affinity::from_id(static_cast<std::uint32_t>(i) + 1));

      /* Same witness trick as test 6, and for the same reason: a flag the
       * sender sets before the call only proves it got there. This can only run
       * once the sender has parked inside send_blocking and given up its core,
       * so it is the parking itself that is observed. */
      witnesses[static_cast<std::size_t>(i)].emplace(
         [&s] { s.parked.release(); },
         witness_stacks[static_cast<std::size_t>(i)],
         thread::priority(2),
         core_affinity::from_id(static_cast<std::uint32_t>(i) + 1));
   }

   kernel::start();

   EXPECT_TRUE(s.all_entered.load())        << "senders never all parked inside send_blocking";
   EXPECT_EQ(s.returned.load(), blocked)    << "stop() did not release every blocked sender";

   kernel::finalise();
}

/* ============================================================================
 * 8d. A non-blocking send must not STEAL the shutdown token
 *
 * stop() leaves one token in each semaphore, and every path that consumes one
 * and finds the channel dead puts it back. That give-back is one line, it is
 * easy to lose in a refactor, and losing it strands whoever was next in the
 * relay. The receiving side of the same rule lives in settle().
 *
 * Making the theft happen ON PURPOSE rather than by luck is the whole
 * construction. Every participant is pinned to ONE core, and the thief runs the
 * stop and the theft inside a critical section, so nothing else on that core
 * can run until it is done. The token is therefore guaranteed to be taken
 * before any blocked sender gets a chance at it, which is the interleaving a
 * racing test would hit only occasionally.
 * ========================================================================= */
TEST_F(Channel_Test, GivenBlockedSendersAtStop_WhenATrySendTakesTheToken_ThenItIsGivenBack)
{
   kernel::initialise();

   static std::array<test::guarded_stack, 6> stacks;

   constexpr int blocked = 3;
   constexpr int thefts  = 8;

   struct state
   {
      chan::channel<std::uint32_t, 1> ch;
      std::atomic<int> returned{0};
      std::atomic<int> refused{0};
   } s;

   // Priority 0 on core0, so this runs first and the single slot is full
   // before any sender gets there.
   thread filler([&s] { s.ch.send(1); },
                 stacks[0], thread::priority(0), core0);

   // Priority 1: each runs, finds no room, and parks. One core, so they park
   // one after another with nothing else interleaving.
   std::array<std::optional<thread>, blocked> senders;
   for (int i = 0; i < blocked; ++i) {
      senders[static_cast<std::size_t>(i)].emplace(
         [&s] {
            s.ch.send_blocking(7);
            s.returned.fetch_add(1);
         },
         stacks[static_cast<std::size_t>(i) + 1], thread::priority(1), core0);
   }

   // Priority 2: runs only once every sender above has parked.
   thread thief(
      [&s] {
         int refused = 0;
         /* The critical section is what makes this deterministic. stop() makes
          * a sender runnable, and without the mask that sender would preempt
          * this thread immediately and take its own token before the theft. */
         auto const token = this_core::enter_critical();
         s.ch.stop();
         for (int i = 0; i < thefts; ++i) {
            if (!s.ch.try_send(9)) { ++refused; }
         }
         this_core::exit_critical(token);
         s.refused.store(refused);
      },
      stacks[4], thread::priority(2), core0);

   thread watchdog(
      [&s] {
         fail_fast_if_wedged("a try_send after stop() swallowed the token a blocked sender needed",
                             [&] { return s.returned.load() == blocked; });
      },
      stacks[5], thread::priority(0), watchdog_core);

   kernel::start();

   EXPECT_EQ(s.returned.load(), blocked)
      << "a blocked sender was never released, so the shutdown token was consumed and not returned";
   EXPECT_EQ(s.refused.load(), thefts)
      << "try_send queued a value into a stopped channel";

   kernel::finalise();
}

/* ============================================================================
 * 8e. A STOPPED channel's contents are immutable, even under overwrite pressure
 *
 * stop() promises that values already queued are still delivered. send_overwrite
 * is the only send that can reach into a full ring and change what is there, so
 * it is the only one that can break that promise.
 *
 * Reaching the path at all takes some doing. stop() leaves a token in the space
 * semaphore, so a lone overwriter always takes that token and is turned away
 * politely, never reaching the code that replaces the oldest. It is only when
 * TWO overwriters race for that single token that the loser falls through to
 * the replace path, and that is the window this test manufactures: two threads
 * on two cores hammering a stopped, full channel until one of them loses.
 * ========================================================================= */
TEST_F(Channel_Test, GivenAStoppedFullChannel_WhenOverwritersRace_ThenTheQueuedValuesAreUntouched)
{
   kernel::initialise();

   static std::array<test::guarded_stack, 4> stacks;

   constexpr std::uint32_t attempts = 50'000;

   struct state
   {
      chan::channel<std::uint32_t, 4> ch;
      sync::semaphore   ready{0};        // one release per overwriters
      sync::semaphore   overwriters_done{0};
      std::array<std::uint32_t, 8> drained{};
      std::atomic<int>  drained_count{0};
   } s;

   thread setup(
      [&s] {
         for (std::uint32_t i = 1; i <= 4; ++i) { s.ch.send(i); }
         s.ch.stop();
         s.ready.release(2);   // release both racers
      },
      stacks[0], thread::priority(0), core0);

   std::array<std::optional<thread>, 2> overwriters;
   for (std::uint32_t w = 0; w < 2; ++w) {
      overwriters[w].emplace(
         [&s] {
            s.ready.acquire();
            for (std::uint32_t i = 0; i < attempts; ++i) { s.ch.send_overwrite(999); }
            s.overwriters_done.release();
         },
         stacks[w + 1], thread::priority(1), core_affinity::from_id(w + 1));
   }

   thread drainer(
      [&s] {
         /* Blocks rather than spins. This waits on 2 x `attempts` contended
          * operations, and a spin budget sized for that is either useless as a
          * bound or flaky under load, so it is not a bound at all. */
         s.overwriters_done.acquire();
         s.overwriters_done.acquire();

         int n = 0;
         while (auto value = s.ch.try_receive()) {
            if (n < 8) { s.drained[static_cast<std::size_t>(n)] = *value; }
            ++n;
         }
         s.drained_count.store(n);
      },
      stacks[3], thread::priority(1), core3);

   kernel::start();

   EXPECT_EQ(s.drained_count.load(), 4)
      << "a stopped channel did not hold exactly the four values queued before the stop";
   for (std::uint32_t i = 0; i < 4 && i < static_cast<std::uint32_t>(s.drained_count.load()); ++i) {
      EXPECT_EQ(s.drained[i], i + 1)
         << "value " << i << " was changed by a send_overwrite after stop()";
   }

   kernel::finalise();
}

/* ============================================================================
 * 8f. The strict send() must not panic on a SHUTDOWN race
 *
 * send() treats a full channel as a sizing bug and stops the system. That makes
 * it very important that it can tell "full" from "finished", because a send
 * that loses a race with stop() is an ordinary lifecycle event and must be
 * refused quietly.
 *
 * The two look identical from the outside: in both cases there is no space
 * token to be had. After stop() there IS one, the relay token, but a second
 * sender racing the first for it comes away empty-handed all the same. So this
 * runs two senders against a stopped, full channel until one of them loses that
 * race, which without the distinction panics and takes the process with it.
 *
 * A failure here is therefore an abort rather than an assertion, and the
 * surviving assertions only run if the race was survived at all.
 * ========================================================================= */
TEST_F(Channel_Test, GivenAStoppedFullChannel_WhenStrictSendsRace_ThenTheyAreRefusedNotFatal)
{
   kernel::initialise();

   static std::array<test::guarded_stack, 4> stacks;

   constexpr std::uint32_t attempts = 50'000;

   struct state
   {
      chan::channel<std::uint32_t, 4> ch;
      sync::semaphore   ready{0};        // one release per senders
      sync::semaphore   senders_done{0};
      std::array<std::uint32_t, 8> drained{};
      std::atomic<int>  drained_count{0};
   } s;

   thread setup(
      [&s] {
         for (std::uint32_t i = 1; i <= 4; ++i) { s.ch.send(i); }
         s.ch.stop();
         s.ready.release(2);   // release both racers
      },
      stacks[0], thread::priority(0), core0);

   std::array<std::optional<thread>, 2> senders;
   for (std::uint32_t w = 0; w < 2; ++w) {
      senders[w].emplace(
         [&s] {
            s.ready.acquire();
            // Every one of these must be refused quietly. One panic ends the run.
            for (std::uint32_t i = 0; i < attempts; ++i) { s.ch.send(999); }
            s.senders_done.release();
         },
         stacks[w + 1], thread::priority(1), core_affinity::from_id(w + 1));
   }

   thread drainer(
      [&s] {
         /* Blocks rather than spins. This waits on 2 x `attempts` contended
          * operations, and a spin budget sized for that is either useless as a
          * bound or flaky under load, so it is not a bound at all. */
         s.senders_done.acquire();
         s.senders_done.acquire();

         int n = 0;
         while (auto value = s.ch.try_receive()) {
            if (n < 8) { s.drained[static_cast<std::size_t>(n)] = *value; }
            ++n;
         }
         s.drained_count.store(n);
      },
      stacks[3], thread::priority(1), core3);

   kernel::start();

   EXPECT_EQ(s.drained_count.load(), 4)
      << "a strict send after stop() still changed the channel";
   for (std::uint32_t i = 0; i < 4 && i < static_cast<std::uint32_t>(s.drained_count.load()); ++i) {
      EXPECT_EQ(s.drained[i], i + 1) << "value " << i << " was changed by a send after stop()";
   }

   kernel::finalise();
}

/* ============================================================================
 * 9. The work channel: jobs run, in thread context, in FIFO order
 *
 * One worker, so start order and finish order are the same thing and FIFO is
 * actually observable. Also the test that `run` terminates when the channel
 * does, which is what lets the kernel quiesce.
 * ========================================================================= */
TEST_F(Channel_Test, GivenAWorkChannel_WhenRun_ThenJobsExecuteInOrderAndRunReturnsAtStop)
{
   kernel::initialise();

   static std::array<test::guarded_stack, 2> stacks;

   struct state
   {
      chan::work_channel<8> ch;
      std::atomic<std::uint32_t> order_hash{0};
      std::atomic<int>  ran{0};
      std::atomic<bool> run_returned{false};
      std::atomic<bool> ran_in_worker{false};
      std::uint32_t     worker_thread_id{0};
   } s;

   thread worker(
      [&s] {
         s.worker_thread_id = this_thread::id();
         chan::run(s.ch);
         s.run_returned.store(true);
      },
      stacks[0], thread::priority(1), core0);

   thread poster(
      [&s] {
         for (std::uint32_t i = 1; i <= 5; ++i) {
            s.ch.send_blocking([&s, i] {
               // Order-sensitive accumulator: any permutation gives a
               // different hash, so FIFO is checked rather than assumed.
               s.order_hash.store(s.order_hash.load() * 10u + i);
               s.ran.fetch_add(1);
               if (this_thread::id() == s.worker_thread_id) {
                  s.ran_in_worker.store(true);
               }
            });
         }
         s.ch.stop();
      },
      stacks[1], thread::priority(1), core1);

   kernel::start();

   EXPECT_EQ(s.ran.load(), 5)             << "not every job ran";
   EXPECT_EQ(s.order_hash.load(), 12345u) << "jobs did not run in FIFO order";
   EXPECT_TRUE(s.ran_in_worker.load())    << "jobs did not run on the worker thread";
   EXPECT_TRUE(s.run_returned.load())     << "run() did not return when the channel stopped";

   kernel::finalise();
}

/* ============================================================================
 * 10. A job larger than JobSize does not compile
 *
 * Negative compile tests are not expressible in the builder, so the claim is
 * pinned as a static_assert on the same trait the function's own static_assert
 * uses. It fails at build time here if the bound ever stops being enforced.
 * ========================================================================= */
namespace
{

struct big_capture
{
   std::array<std::byte, 64> payload;
};

// A 16-byte job holds a small lambda and not a 64-byte one. Both directions
// are asserted, so a bound that stopped rejecting anything is caught too.
static_assert(std::is_constructible_v<chan::job<32>, void (*)()>,
              "a plain function pointer must fit a 32 byte job");
static_assert(sizeof(big_capture) > 16,
              "the oversized capture must actually be oversized for the check below to mean anything");

}  // namespace
