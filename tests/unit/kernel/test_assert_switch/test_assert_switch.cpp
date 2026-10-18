/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_assert_switch.cpp
 * @brief CYROS_CHECK_ASSERTS off: an internal CYROS_ASSERT neither stops nor
 *        evaluates anything, and CYROS_REQUIRE is untouched.
 *
 * Set here the way a toolchain sets it, ahead of every cyros header. The
 * default, every assert on, is what the rest of the suite runs on, so it is
 * proved there rather than here.
 */

#define CYROS_CHECK_ASSERTS 0

#include <cyros/kernel/assert.hpp>
#include <cyros/port/port_core.h>

#include <gtest/gtest.h>

namespace
{

int evaluations = 0;

bool counted(bool value)
{
   ++evaluations;
   return value;
}

int const* counted_pointer(int const* value)
{
   ++evaluations;
   return value;
}

} // namespace

TEST(Assert_Switch, GivenAssertsOff_ThenAFailingAssertNeitherStopsNorEvaluates)
{
   int const something = 0;
   evaluations = 0;

   CYROS_ASSERT(counted(false));
   CYROS_ASSERT1(counted(false), counted(false));
   CYROS_ASSERT2(counted(false), counted(false), counted(false));
   CYROS_ASSERT_OP(counted(false), ==, counted(true));
   CYROS_ASSERT_NULL(counted_pointer(&something));

   EXPECT_EQ(evaluations, 0);
}

TEST(Assert_Switch, GivenAssertsOff_ThenRequireStillChecks)
{
   evaluations = 0;

   CYROS_REQUIRE(counted(true));
   CYROS_REQUIRE2(counted(true), 1, 2);

   EXPECT_EQ(evaluations, 2);
}

TEST(Assert_Switch, GivenAssertsOff_ThenAValueKeptOnlyForItsCheckStillCountsAsUsed)
{
   // Under -Werror this is the test: were the assert to drop its operands, the
   // variables would be unused, or set but unused, and the file would not
   // compile. The second is the shape the RP2 boot loops have.
   auto const only_checked = 5;
   CYROS_ASSERT(only_checked > 0);

   unsigned words = 0;
   ++words;
   CYROS_ASSERT_OP(words, <, 64u);
   SUCCEED();
}
