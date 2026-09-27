/**
 * @file trace_points.hpp
 * @brief The kernel's trace points: placed at notable state transition points
 *
 * Trace points may be in places where interrupts are masked or within a spinlock,
 * therefore a point mustn't take a lock, nor do I/O.
 */
#ifndef CYROS_TRACE_POINTS_HPP
#define CYROS_TRACE_POINTS_HPP

#include <cyros/config/config.hpp>
#include <cyros/kernel/trace.hpp>

#include "threading_subsystem.hpp"

#include <cstdint>

namespace cyros
{

class base_mutex;

namespace trace
{

inline constexpr bool enabled = config::trace_records_per_core > 0;

void append(event what, thread_control_block const& subject, std::uintptr_t object = 0) noexcept;
void reset() noexcept;

inline void thread_created(thread_control_block const& tcb) noexcept
{
   if constexpr (enabled) append(event::thread_created, tcb);
}

inline void thread_ready(thread_control_block const& tcb) noexcept
{
   if constexpr (enabled) append(event::thread_ready, tcb);
}

inline void thread_running(thread_control_block const& tcb) noexcept
{
   if constexpr (enabled) append(event::thread_running, tcb);
}

inline void thread_blocked(thread_control_block const& tcb) noexcept
{
   if constexpr (enabled) append(event::thread_blocked, tcb);
}

inline void thread_terminated(thread_control_block const& tcb) noexcept
{
   if constexpr (enabled) append(event::thread_terminated, tcb);
}

inline void remote_wake(thread_control_block const& tcb) noexcept
{
   if constexpr (enabled) append(event::remote_wake, tcb);
}

inline void mutex_acquired(thread_control_block const& owner, base_mutex const& mutex) noexcept
{
   if constexpr (enabled) append(event::mutex_acquired, owner, reinterpret_cast<std::uintptr_t>(&mutex));
}

inline void mutex_released(thread_control_block const& owner, base_mutex const& mutex) noexcept
{
   if constexpr (enabled) append(event::mutex_released, owner, reinterpret_cast<std::uintptr_t>(&mutex));
}

inline void lifecycle_begins() noexcept
{
   if constexpr (enabled) reset();
}

} // namespace trace

} // namespace cyros

#endif // CYROS_TRACE_POINTS_HPP
