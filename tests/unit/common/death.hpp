/**
 * @file death.hpp
 * @brief Death tests that prove WHICH check fired, not merely that one did.
 *
 *    #include <common/death.hpp>
 *
 *    CYROS_EXPECT_PANIC(misuse_the_api(), cyros::test::panicked_at("kernel.cpp", "initialise twice"))
 *       << "what the misuse should have been stopped by";
 *
 * THE PROBLEM IT SOLVES. gtest matches a death test against the child's STDERR,
 * and the Linux ports print a panic to STDOUT (`port_linux_common.cpp`). So a
 * plain death test can only assert that the child aborted. That passes for ANY
 * panic, including one on a different line for a different reason, which is
 * precisely the failure a death test most needs to rule out: a check that has
 * stopped firing, masked by a later one that still does.
 *
 * CYROS_EXPECT_PANIC points the child's stdout at its stderr before running the
 * statement. The panic banner (`KERNEL PANIC at <file>:<line>`) and the context
 * printer's `>>` line, which is the source text of the check that failed, then
 * reach the matcher. It still requires SIGABRT, so it is strictly stronger than
 * the signal-only form, never weaker.
 *
 * The child also gets a watchdog. A check that stops firing usually lets the
 * misuse carry on, and "carry on" is often a hang rather than a different death:
 * a kernel started with no threads idles forever. Without the watchdog that
 * hangs the whole binary, and the runner's timeout cannot even clean up, since
 * the child is a separate process. With it the child is killed by SIGALRM after
 * `death_test_watchdog_s`, which fails the test by name.
 *
 * It also sets gtest's `threadsafe` death-test style itself, so a test cannot
 * forget to. That style re-executes the binary for each death test. The default
 * `fast` style forks the already-threaded parent, which is unsafe once a kernel
 * has spawned core threads.
 *
 * The statement is one expression or call, as for EXPECT_EXIT. Put anything with
 * commas in it, a lambda for instance, in a function and call that.
 */

#ifndef CYROS_TEST_DEATH_HPP
#define CYROS_TEST_DEATH_HPP

#include <gtest/gtest.h>

#include <csignal>
#include <cstdio>
#include <ostream>
#include <string>
#include <sys/resource.h>
#include <unistd.h>

namespace cyros::test
{

/** Death tests take about 100 ms here, so this only ever fires on a hang. */
inline constexpr unsigned death_test_watchdog_s = 10;

/**
 * @brief Prepare the death-test child: route the panic, arm the watchdog.
 *
 * Called inside the child only. The panic path flushes stdio before it
 * terminates, so nothing it prints is lost in a buffer. SIGALRM is not a signal
 * cyros uses, and its default action ends the process. Core dumps are switched
 * off for the child, since its death is the expected outcome.
 */
inline void enter_death_test_child() noexcept
{
   std::fflush(stdout);
   ::dup2(STDERR_FILENO, STDOUT_FILENO);
   ::alarm(death_test_watchdog_s);

   // An expected death needs no core dump. Where cores are piped to
   // systemd-coredump, every death test would otherwise store one, and they
   // bury the real crashes coredumpctl is used to find (measured 2026-09-24:
   // about 340 in one night of mutation runs).
   struct rlimit const no_core{0, 0};
   ::setrlimit(RLIMIT_CORE, &no_core);
}

/**
 * @brief Matches the output of a cyros panic raised in a given file, on a line
 *        whose source text contains a given fragment.
 *
 * The fragment is compared as plain text, not a regex, so it can be copied
 * straight from the check: `"CYROS_REQUIRE1(result != outcome::full"`. It is
 * compared against the `>>` line only, never the context around it, so it pins
 * the line that actually failed.
 */
class panic_matcher : public ::testing::MatcherInterface<std::string const&>
{
public:
   panic_matcher(std::string file, std::string fragment)
      : file(std::move(file)), fragment(std::move(fragment)) {}

   bool MatchAndExplain(std::string const& out, ::testing::MatchResultListener* listener) const override
   {
      auto const banner = out.find("KERNEL PANIC at ");
      if (banner == std::string::npos) {
         *listener << "no KERNEL PANIC banner at all";
         return false;
      }
      std::string const where = line_at(out, banner);

      // A path component match, so "channel.hpp" cannot match "test_channel.hpp".
      auto const at = where.find(file + ":");
      if (at == std::string::npos || (where[at - 1] != '/' && where[at - 1] != ' ')) {
         *listener << "the panic was raised somewhere else: " << where;
         return false;
      }

      auto const marker = out.find(">> ", banner);
      if (marker == std::string::npos) {
         *listener << "no >> line, so the source of the failing check was not printed";
         return false;
      }
      std::string const failed = line_at(out, marker);
      if (failed.find(fragment) == std::string::npos) {
         *listener << "a different check in that file failed: " << failed;
         return false;
      }
      return true;
   }

   void DescribeTo(std::ostream* os) const override
   {
      *os << "a KERNEL PANIC in " << file << " on a line containing \"" << fragment << "\"";
   }

private:
   static std::string line_at(std::string const& out, std::size_t from)
   {
      auto const end = out.find('\n', from);
      return out.substr(from, end == std::string::npos ? std::string::npos : end - from);
   }

   std::string file;
   std::string fragment;
};

inline ::testing::Matcher<std::string const&> panicked_at(std::string file, std::string fragment)
{
   return ::testing::MakeMatcher(new panic_matcher(std::move(file), std::move(fragment)));
}

}  // namespace cyros::test

/* The `if` sets the death-test style before gtest decides how to run the child,
 * and is written as a condition so the macro stays a single statement that
 * still accepts `<< "message"` exactly as EXPECT_EXIT does. */
#define CYROS_EXPECT_PANIC(statement, matcher)                                   \
   if ((GTEST_FLAG_SET(death_test_style, "threadsafe"), false)) {} else          \
      EXPECT_EXIT({ ::cyros::test::enter_death_test_child(); statement; },      \
                  ::testing::KilledBySignal(SIGABRT), matcher)

#endif // CYROS_TEST_DEATH_HPP
