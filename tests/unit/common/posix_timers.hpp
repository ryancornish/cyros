/**
 * @file posix_timers.hpp
 * @brief Count the POSIX timers alive in this process, for unit tests.
 *
 * For tests that must prove a timer was DELETED rather than merely disarmed.
 * From inside the process the two look identical, since neither delivers a
 * signal. The difference is a kernel object that outlives the code that created
 * it, and `/proc/self/timers` is the one place that shows it: one `ID:` line
 * per live timer, whichever thread created it and whether or not that thread
 * still exists.
 *
 *    #include <common/posix_timers.hpp>
 */

#ifndef CYROS_TEST_POSIX_TIMERS_HPP
#define CYROS_TEST_POSIX_TIMERS_HPP

#include <fstream>
#include <string>

namespace cyros::test
{

/**
 * @brief How many POSIX timers this process currently owns.
 * @return The count, or -1 if `/proc/self/timers` cannot be read.
 *
 * -1 means "cannot tell", which happens on a kernel built without
 * CONFIG_CHECKPOINT_RESTORE. A caller must skip on it, never read it as zero,
 * or the test passes on such a kernel without having looked.
 */
inline int live_posix_timers()
{
   std::ifstream in("/proc/self/timers");
   if (!in) return -1;

   int count = 0;
   for (std::string line; std::getline(in, line);) {
      if (line.starts_with("ID:")) ++count;
   }
   return count;
}

}  // namespace cyros::test

#endif // CYROS_TEST_POSIX_TIMERS_HPP
