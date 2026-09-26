/**
 * @file test_time_death.cpp
 * @brief The time driver refuses a lifecycle used out of order.
 *
 * Subject: the periodic driver's lifecycle preconditions (L1)
 * Trusts:  the port's panic path. No kernel.
 *
 * A sample of the driver's lifecycle checks, the largest single group of caller
 * preconditions in the tree (`assert-proposal.md` section 2). They are still
 * spelled CYROS_ASSERT, pending that proposal's plan step 2, so each is matched
 * on its own source text rather than on a comment.
 *
 *   initialise() twice
 *      would re-register the ISR and clear nothing, over timers still live.
 *   start() before initialise()
 *      would program the timer with a frequency of zero.
 *
 * Both are mutation-verified by deleting the check.
 */

#include <cyros/time/time.hpp>

#include <common/death.hpp>

#include <gtest/gtest.h>

using namespace cyros;

namespace
{

void initialise_twice()
{
   time::initialise(1'000);
   time::initialise(1'000);
}

void start_before_initialise()
{
   time::start();
}

}  // namespace

TEST(TimeDeath_Test, GivenAnInitialisedDriver_WhenInitialisedAgain_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(initialise_twice(),
                      test::panicked_at("time_driver_periodic.cpp", "CYROS_ASSERT(!tconfig.initialised);"));
}

TEST(TimeDeath_Test, GivenNoInitialise_WhenStarted_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(start_before_initialise(),
                      test::panicked_at("time_driver_periodic.cpp", "CYROS_ASSERT(tconfig.initialised);"))
      << "start() went on to program a timer at zero Hz";
}
