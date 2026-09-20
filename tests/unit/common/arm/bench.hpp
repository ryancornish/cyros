/**
 * @file bench.hpp
 * @brief Minimal assertion harness for tests that run on the target.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: nothing. This is harness, not a test.
 * Trusts:  ARM semihosting, and that the host propagates an exit status.
 * Proves:  nothing.
 *
 * WHY THIS EXISTS RATHER THAN GTEST. gtest does not run bare metal: it wants a
 * heap, exceptions, iostreams and a hosted main. The surface the cyros suite
 * actually uses is small (measured 2026-09-20 across every unit test: EXPECT_EQ
 * and friends, TEST, TEST_F, one TEST_P), so a shim is cheap. This is the
 * smaller half of that shim, carrying only what a target bring-up needs.
 *
 * It deliberately does NOT try to be gtest-compatible. The tests that run here
 * are written for here. A source that builds under both harnesses is a
 * worthwhile thing to have later, and is a different piece of work from getting
 * a port off the ground.
 *
 * Output goes out one semihosting trap at a time, which costs microseconds per
 * call. Nothing in a timing-sensitive region may check anything.
 */

#ifndef CYROS_TEST_ARM_BENCH_HPP
#define CYROS_TEST_ARM_BENCH_HPP

#include <cstdint>

namespace cyros::bench
{

/* ---------------------------------------------------------------------------
 * Semihosting
 *
 * Duplicated from the port's own cortex_m.hpp on purpose. That header is port
 * internal and a test must not reach into it: the harness has to be able to
 * report that the port is broken, which it cannot do if it is built out of the
 * thing under test.
 * ------------------------------------------------------------------------ */

inline constexpr long sys_write0        = 0x04;
inline constexpr long sys_exit_extended = 0x20;
inline constexpr std::uint32_t adp_stopped_application_exit = 0x20026u;

inline long semihost(long op, void volatile* arg) noexcept
{
   register long r0 asm("r0") = op;
   register void volatile* r1 asm("r1") = arg;
   asm volatile("bkpt 0xAB" : "+r"(r0) : "r"(r1) : "memory");
   return r0;
}

inline void print(char const* text) noexcept
{
   semihost(sys_write0, const_cast<char*>(text));
}

[[noreturn]] inline void host_exit(std::uint32_t code) noexcept
{
   volatile std::uint32_t block[2] = { adp_stopped_application_exit, code };
   semihost(sys_exit_extended, block);
   __builtin_unreachable();
}

inline void print_hex(std::uint32_t value) noexcept
{
   char buffer[11] = { '0', 'x' };
   for (int i = 0; i < 8; ++i) {
      std::uint32_t const nibble = (value >> ((7 - i) * 4)) & 0xFu;
      buffer[2 + i] = static_cast<char>(nibble < 10 ? '0' + nibble : 'a' + (nibble - 10));
   }
   buffer[10] = '\0';
   print(buffer);
}


/* ---------------------------------------------------------------------------
 * Results
 * ------------------------------------------------------------------------ */

inline int checks_run    = 0;
inline int checks_failed = 0;

inline void record(bool passed, char const* expression, char const* file, int line) noexcept
{
   ++checks_run;
   if (passed) { return; }

   ++checks_failed;
   print("  [FAIL] ");
   print(expression);
   print("\n         at ");
   print(file);
   print(":");
   print_hex(static_cast<std::uint32_t>(line));
   print("\n");
}

inline void record_eq(std::uint32_t lhs, std::uint32_t rhs,
                      char const* expression, char const* file, int line) noexcept
{
   ++checks_run;
   if (lhs == rhs) { return; }

   ++checks_failed;
   print("  [FAIL] ");
   print(expression);
   print("\n         lhs = ");
   print_hex(lhs);
   print("  rhs = ");
   print_hex(rhs);
   print("\n         at ");
   print(file);
   print(":");
   print_hex(static_cast<std::uint32_t>(line));
   print("\n");
}

inline void start(char const* name) noexcept
{
   print("=== ");
   print(name);
   print(" ===\n");
}

/**
 * @brief Report and exit with a status the host can see.
 *
 * A run that checked nothing exits nonzero. A test binary whose body was
 * optimised away, or whose kernel never reached the checks, otherwise reports
 * success by saying nothing, which is the exact false green T1 removed from the
 * host runner.
 */
[[noreturn]] inline void finish() noexcept
{
   print("checks run    = ");
   print_hex(static_cast<std::uint32_t>(checks_run));
   print("\nchecks failed = ");
   print_hex(static_cast<std::uint32_t>(checks_failed));
   print("\n");

   if (checks_run == 0) {
      print("RESULT: FAIL (no checks ran)\n");
      host_exit(2u);
   }
   if (checks_failed != 0) {
      print("RESULT: FAIL\n");
      host_exit(1u);
   }
   print("RESULT: PASS\n");
   host_exit(0u);
}

} // namespace cyros::bench


#define CYROS_CHECK(expr) \
   ::cyros::bench::record((expr), #expr, __FILE__, __LINE__)

#define CYROS_CHECK_EQ(lhs, rhs) \
   ::cyros::bench::record_eq( \
      static_cast<std::uint32_t>(lhs), static_cast<std::uint32_t>(rhs), \
      #lhs " == " #rhs, __FILE__, __LINE__)

#endif /* CYROS_TEST_ARM_BENCH_HPP */
