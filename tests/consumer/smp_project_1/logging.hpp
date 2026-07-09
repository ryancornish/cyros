#ifndef LOGGING_HPP
#define LOGGING_HPP

#include <cyros/port/port.h>

#include <cstdarg>
#include <cstdio>

namespace log
{

inline void printf(char const* fmt, ...)
{
   auto token = cyros_port_preempt_disable(); // TODO: Swap with kernel-level API

   va_list args;
   va_start(args, fmt);
   (void)std::vprintf(fmt, args);
   va_end(args);

   cyros_port_preempt_enable(token); // TODO: Swap with kernel-level API
}

}  // namespace log

#endif
