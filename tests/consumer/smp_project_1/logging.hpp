#ifndef LOGGING_HPP
#define LOGGING_HPP

#include <cyros/kernel/core.hpp>

#include <cstdarg>
#include <cstdio>

namespace logging
{

inline void printf(char const* fmt, ...)
{
   // Every cyros thread on a core runs on that core's ONE OS thread, so glibc's
   // stdio lock cannot tell them apart. Preempting mid-vprintf would let another
   // cyros thread on this core re-enter stdio under a lock this OS thread
   // already holds. Holding off preemption closes that. Other cores are
   // separate OS threads, which glibc's own FILE lock already serialises.
   cyros::this_core::preemption_guard guard;

   va_list args;
   va_start(args, fmt);
   (void)std::vprintf(fmt, args);
   va_end(args);
}

}  // namespace logging

#endif
