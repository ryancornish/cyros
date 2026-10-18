/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_precondition_switch.cpp
 * @brief CYROS_CHECK_PRECONDITIONS off: a CYROS_REQUIRE neither stops nor
 *        evaluates anything, and CYROS_FATAL is untouched.
 *
 * Set here the way a toolchain sets it, ahead of every cyros header. The
 * default, every check on, is what the death tests stand on, so it is proved
 * there rather than here.
 */

#define CYROS_CHECK_PRECONDITIONS 0

#include <cyros/kernel/assert.hpp>

#include <gtest/gtest.h>

namespace
{

int evaluations = 0;

bool counted(bool value)
{
   ++evaluations;
   return value;
}

} // namespace

TEST(Precondition_Switch, GivenChecksOff_ThenAFailingRequireNeitherStopsNorEvaluates)
{
   evaluations = 0;

   CYROS_REQUIRE(counted(false));
   CYROS_REQUIRE1(counted(false), counted(false));
   CYROS_REQUIRE2(counted(false), counted(false), counted(false));
   CYROS_REQUIRE_OP(counted(false), ==, counted(true));

   EXPECT_EQ(evaluations, 0);
}

TEST(Precondition_Switch, GivenChecksOff_ThenFatalStillChecks)
{
   evaluations = 0;

   CYROS_FATAL(counted(true));
   CYROS_FATAL2(counted(true), 1, 2);

   EXPECT_EQ(evaluations, 2);
}

TEST(Precondition_Switch, GivenChecksOff_ThenAValueKeptOnlyForItsCheckStillCountsAsUsed)
{
   // Under -Werror this is the test: were the check to drop its operands, the
   // variable would be unused and the file would not compile
   auto const only_checked = 5;
   CYROS_REQUIRE(only_checked > 0);
   SUCCEED();
}
