/**
 * @file assert.hpp
 * @brief The public hard-error channel: checks a consumer may write, and the
 *        panic they end in.
 *
 *   CYROS_REQUIRE   a caller misused an API. The caller's bug.
 *   CYROS_FATAL     the environment failed. Nobody's bug, and unfixable here.
 *
 * Both stop the system through cyros::panic. They are identical in expansion
 * today, and that is deliberate: the distinction they carry is which one may
 * EVER become optional. A precondition is a claim about the caller, so how much
 * checking a build pays for is the user's trade to make, and roadmap A2 plans to
 * express these as C++26 `pre()` with the level set per toolchain. An
 * environment failure is not a claim about anyone, and compiling out the check
 * that `mmap` succeeded does not make memory appear, so CYROS_FATAL is
 * unconditional for the life of the project.
 *
 * Writing the right one now is what makes that migration a rename rather than a
 * re-reading of every site.
 *
 * NOTHING HERE IS COMPILED OUT TODAY. CYROS_REQUIRE is unconditional, exactly
 * as the checks it replaces were, so adopting it changes no behaviour.
 */

#ifndef CYROS_ASSERT_HPP
#define CYROS_ASSERT_HPP

#include <cyros/kernel/visibility.hpp>
#include <cyros/port/port_traits.h>

#include <cstdint>

namespace cyros
{

/**
 * @brief Stop the system, reporting up to two values and a source location.
 *
 * Does not return and does not unwind. The port decides what stopping means:
 * on the linux ports it prints and exits, on target it is the panic handler.
 *
 * Callable directly for a failure with no natural predicate, but prefer
 * CYROS_REQUIRE or CYROS_FATAL, which say who is at fault and capture the
 * location for free.
 *
 * @param aux1 First value to report, printed as AUX1. Often the offending operand.
 * @param aux2 Second value to report, printed as AUX2.
 * @param file Source file, or "" when the port does not capture location.
 * @param line Source line, or 0 when the port does not capture location.
 */
[[noreturn]] CYROS_PUBLIC void panic(std::uintptr_t aux1 = 0,
                                     std::uintptr_t aux2 = 0,
                                     char const*    file = "",
                                     int            line = 0) noexcept;

}  // namespace cyros

/* Location capture follows the port's own setting, so a public check reports
 * exactly what an internal one does on the same build. A port that sets
 * CYROS_PORT_CAPTURE_LOCATION to 0 is trading diagnosis for the image size of
 * every __FILE__ string, and that trade has to apply here too or it buys
 * nothing. */
#if CYROS_PORT_CAPTURE_LOCATION
 #define CYROS_CHECK_FILE (__FILE__)
 #define CYROS_CHECK_LINE (__LINE__)
#else
 #define CYROS_CHECK_FILE ""
 #define CYROS_CHECK_LINE 0
#endif

#define CYROS_CHECK2(condition, aux1, aux2)                                   \
   (__builtin_expect(!!(condition), 1)                                        \
       ? (void)0                                                              \
       : ::cyros::panic((std::uintptr_t)(aux1), (std::uintptr_t)(aux2),       \
                        CYROS_CHECK_FILE, CYROS_CHECK_LINE))

/**
 * @def CYROS_REQUIRE
 * @brief A precondition of a public API. Failure means the CALLER has a bug.
 *
 * Use at the top of a public entry point, for a condition the caller controls
 * and could have checked: an unbalanced unlock, a stack buffer too small, a
 * handle used after it was moved from, an alarm re-armed while pending.
 *
 * Do not use for a condition the caller cannot control. That is CYROS_FATAL if
 * the environment failed, and an internal CYROS_ASSERT if cyros is at fault.
 */
#define CYROS_REQUIRE(condition)             CYROS_CHECK2(condition, 0, 0)
#define CYROS_REQUIRE1(condition, aux1)      CYROS_CHECK2(condition, aux1, 0)
#define CYROS_REQUIRE2(condition, aux1, aux2) CYROS_CHECK2(condition, aux1, aux2)

/**
 * @def CYROS_REQUIRE_OP
 * @brief CYROS_REQUIRE for a comparison, reporting both operands.
 *
 * `CYROS_REQUIRE_OP(size, >=, minimum)` panics with the actual size in AUX1 and
 * the minimum in AUX2, which is usually the whole diagnosis. Both operands are
 * evaluated once each on the failing path and once in the condition, so the
 * operands must be free of side effects.
 */
#define CYROS_REQUIRE_OP(lhs, op, rhs)       CYROS_CHECK2((lhs) op (rhs), lhs, rhs)

/**
 * @def CYROS_FATAL
 * @brief The environment failed. Not a bug in cyros and not a bug in the caller.
 *
 * Use where a resource the system depends on was refused and there is no
 * recovery: a mapping failed, a signal handler would not install, a fixed table
 * sized by configuration is full.
 *
 * Never compiled out, in any build, ever. A check that a resource exists is not
 * a diagnostic aid, it is the only thing standing between the failure and
 * undefined behaviour.
 */
#define CYROS_FATAL(condition)               CYROS_CHECK2(condition, 0, 0)
#define CYROS_FATAL1(condition, aux1)        CYROS_CHECK2(condition, aux1, 0)
#define CYROS_FATAL2(condition, aux1, aux2)  CYROS_CHECK2(condition, aux1, aux2)

#endif  // CYROS_ASSERT_HPP
