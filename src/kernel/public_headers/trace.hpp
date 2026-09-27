#ifndef CYROS_TRACE_HPP
#define CYROS_TRACE_HPP

#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/visibility.hpp>

#include <cstddef>
#include <cstdint>
#include <span>

/**
 * @file trace.hpp
 * @brief What the kernel did, in order, for a tool to read afterwards.
 *
 * Off unless the configuration asks for it, and off means ABSENT: every trace
 * point in the kernel compiles to nothing, and read() finds nothing to read.
 * Turn it on by declaring, in the application's config header,
 *
 *    inline constexpr std::size_t trace_records_per_core = 256;
 *
 * A power of two, the capacity of each core's buffer. Each core records into
 * its own buffer, so recording takes no lock and no core waits on another.
 *
 * What is recorded is the kernel's own transitions: a thread created, readied,
 * run, blocked and terminated, a wake sent to another core, a mutex acquired and
 * released. Anything else an application wants in the same timeline, it records
 * itself with user().
 *
 * Recording never blocks and never fails. A full buffer DROPS the new record
 * and counts it, because a record is written with interrupts masked, often in
 * the middle of a context switch, where waiting for a reader is not an option.
 * A reader that wants everything drains often enough to keep up.
 *
 * A trace is a record of past truths for a person or a tool. Nothing in cyros
 * reads it back, and nothing may: it is not an input to anything.
 */

namespace cyros::trace
{

enum class event : std::uint8_t
{
   thread_created,    ///< Registered. priority is the base priority
   thread_ready,      ///< Became runnable: created, woken, or preempted while running
   thread_running,    ///< Switched in. priority is its URGENCY as the pick computed it
   thread_blocked,    ///< Parked. object is the mutex, when it is a mutex it waits on
   thread_terminated, ///< Finished
   remote_wake,       ///< Woken from another core. Recorded on the waking core
   mutex_acquired,    ///< object is the mutex. Recorded on the core that committed it
   mutex_released,    ///< object is the mutex
   user,              ///< From user(). priority is the tag, object the value
};

/**
 * @brief One event.
 *
 * `core` is the core the THREAD belongs to, which is not always the core that
 * recorded the event: a remote_wake and a handover's mutex_acquired are recorded
 * by the core that caused them. The recording core is the buffer read() was
 * given.
 */
struct record
{
   /// cyros_port_timestamp() on the recording core: nanoseconds on the Linux
   /// ports, core clock cycles on Cortex-M, where each core counts its own
   std::uint64_t     timestamp;
   /// The mutex's address, or the user value, or 0
   std::uintptr_t    object;
   /// The thread the event is about. Every core's idle thread is id 0.
   /// Qualified, because the member's own name hides the class
   cyros::thread::id thread;
   event             what;
   std::uint8_t      core;
   /// Base priority, except as noted on the event. Lower is more urgent
   std::uint8_t      priority;
   std::uint8_t      reserved;
};

/**
 * @brief Move up to out.size() of @p core's oldest records into @p out.
 * @return How many were written
 *
 * One reader per core at a time. A reader and the core it reads from run
 * concurrently without locking each other: the core only ever appends, and the
 * reader only ever consumes. A trace survives kernel::finalise() and is
 * discarded by the next kernel::initialise().
 *
 * The span is taken by reference only because a span of this 8-byte-aligned
 * record, passed by value, draws GCC's PR77728 ABI note ("parameter passing
 * changed in GCC 7.1") in every 32-bit ARM caller's build.
 */
[[nodiscard]] CYROS_PUBLIC std::size_t read(std::uint32_t core, std::span<record> const& out) noexcept;

/**
 * @brief How many records @p core has dropped this lifecycle, for want of space.
 */
[[nodiscard]] CYROS_PUBLIC std::uint32_t dropped(std::uint32_t core) noexcept;

/**
 * @brief Record an application event in the calling thread's timeline.
 *
 * For whatever the kernel cannot see, such as "the producer queued item 3".
 * The meaning of @p tag and @p value is the application's. Thread context only.
 */
CYROS_PUBLIC void user(std::uint8_t tag, std::uintptr_t value = 0) noexcept;

} // namespace cyros::trace

#endif // CYROS_TRACE_HPP
