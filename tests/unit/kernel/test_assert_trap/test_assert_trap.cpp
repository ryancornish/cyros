/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_assert_trap.cpp
 * @brief CYROS_CHECK_ASSERTS 2: an internal CYROS_ASSERT still checks, and a
 *        failure traps on the spot with no report, while CYROS_REQUIRE is
 *        untouched.
 *
 * Set here the way a toolchain sets it, ahead of every cyros header. On the
 * host a trap is SIGILL, where the reporting path ends in SIGABRT after
 * printing a KERNEL PANIC banner, so the signal and the silence together say
 * which path a failure took.
 */

#define CYROS_CHECK_ASSERTS 2

#include <cyros/kernel/assert.hpp>
#include <cyros/port/port_core.h>

#include <common/death.hpp>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <csignal>

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

void failing_assert()    { CYROS_ASSERT(counted(false)); }
void failing_assert2()   { CYROS_ASSERT2(counted(false), 1, 2); }
void failing_assert_op() { CYROS_ASSERT_OP(evaluations, ==, -1); }
void failing_assert_null()
{
   int const something = 0;
   CYROS_ASSERT_NULL(counted_pointer(&something));
}

/// The death test child, prepared as CYROS_EXPECT_PANIC prepares its own.
void in_child(void (*failing)())
{
   cyros::test::enter_death_test_child();
   failing();
}

auto const no_report = ::testing::Not(::testing::HasSubstr("KERNEL PANIC"));

} // namespace

TEST(Assert_Trap, GivenAssertsTrapping_WhenAnAssertPasses_ThenItEvaluatesItsConditionOnly)
{
   int const something = 0;
   evaluations = 0;

   CYROS_ASSERT(counted(true));
   CYROS_ASSERT1(counted(true), counted(false));
   CYROS_ASSERT2(counted(true), counted(false), counted(false));
   CYROS_ASSERT_OP(counted(true), ==, true);
   CYROS_ASSERT_NULL(counted_pointer(nullptr));

   // One each: the condition, never the aux values, which only a report reads.
   EXPECT_EQ(evaluations, 5);
   (void)something;
}

TEST(Assert_Trap, GivenAssertsTrapping_WhenAnAssertFails_ThenItTrapsWithNoReport)
{
   GTEST_FLAG_SET(death_test_style, "threadsafe");

   EXPECT_EXIT(in_child(failing_assert),      ::testing::KilledBySignal(SIGILL), no_report);
   EXPECT_EXIT(in_child(failing_assert2),     ::testing::KilledBySignal(SIGILL), no_report);
   EXPECT_EXIT(in_child(failing_assert_op),   ::testing::KilledBySignal(SIGILL), no_report);
   EXPECT_EXIT(in_child(failing_assert_null), ::testing::KilledBySignal(SIGILL), no_report);
}

TEST(Assert_Trap, GivenAssertsTrapping_ThenRequireStillReports)
{
   // The switch is the internal asserts' alone: a caller's misuse still panics
   // with its banner, through cyros::panic.
   CYROS_EXPECT_PANIC(CYROS_REQUIRE(counted(false)), ::testing::HasSubstr("KERNEL PANIC"));
}
