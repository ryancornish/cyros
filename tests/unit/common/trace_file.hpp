/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file trace_file.hpp
 * @brief The kernel trace streamed to a file by an ordinary OS thread, for a host test or program.
 *
 * Set the test's trace_records_per_core, then
 *
 *    kernel::initialise();
 *    thread a(...);
 *    test::trace_file trace("a.cytrace");
 *    trace.name_thread(a.get_id(), "producer");
 *    kernel::start();
 *    // the destructor drains what is left and closes the file
 *
 * and read the file with debug/cyros_trace.py, which also turns it into a
 * Perfetto trace. The format is cyros/trace/stream.hpp's.
 *
 * The drain runs on its own OS thread outside every cyros core, so it masks
 * nothing, takes no cyros lock and is never preempted by cyros. It is the one
 * reader of every core's buffer, so nothing else may call trace::read() while
 * it lives. One file holds one lifecycle: construct it after
 * kernel::initialise() and destroy it before the next one.
 *
 * Name things from outside the kernel, before start() or after it returns. A
 * name shares the writer with the drain under a std::mutex, which a cyros
 * thread must not take.
 */

#ifndef CYROS_TEST_TRACE_FILE_HPP
#define CYROS_TEST_TRACE_FILE_HPP

#include <cyros/kernel/thread.hpp>
#include <cyros/trace/stream.hpp>

#include <signal.h>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <span>
#include <stop_token>
#include <string_view>
#include <thread>

namespace cyros::test
{

class trace_file
{
public:
   /// The Linux ports stamp records with CLOCK_MONOTONIC, in nanoseconds
   static constexpr std::uint64_t timestamp_hz = 1'000'000'000;

   explicit trace_file(char const* path, std::chrono::microseconds period = std::chrono::milliseconds(1))
      : file(open(path))
      , writer(file_sink{file}, timestamp_hz)
      , drainer([this, period](std::stop_token stop) { run(stop, period); })
   {
   }

   ~trace_file()
   {
      drainer.request_stop();
      drainer.join();
      writer.drain(); // Whatever the cores recorded after the last pass
      std::fclose(file);
   }

   trace_file(trace_file const&)            = delete;
   trace_file& operator=(trace_file const&) = delete;

   void name_thread(cyros::thread::id thread, std::string_view name)
   {
      std::lock_guard guard(lock);
      writer.name_thread(thread, name);
   }

   void name_object(void const* object, std::string_view name)
   {
      std::lock_guard guard(lock);
      writer.name_object(object, name);
   }

   void name_tag(std::uint8_t tag, std::string_view name)
   {
      std::lock_guard guard(lock);
      writer.name_tag(tag, name);
   }

private:
   struct file_sink
   {
      std::FILE* to;

      void operator()(std::span<std::byte const> packet) const
      {
         if (std::fwrite(packet.data(), 1, packet.size(), to) != packet.size()) {
            std::fprintf(stderr, "trace_file: write failed\n");
            std::abort();
         }
      }
   };

   static std::FILE* open(char const* path)
   {
      auto* file = std::fopen(path, "wb");
      if (file == nullptr) {
         std::fprintf(stderr, "trace_file: cannot open %s\n", path);
         std::abort();
      }
      return file;
   }

   /* Waits a period, then drains, so a period longer than the run leaves
    * everything to the destructor's drain. A stop wakes the wait at once. */
   void run(std::stop_token const& stop, std::chrono::microseconds period)
   {
      // The port's signals are aimed at its core threads, but a thread outside
      // cyros has no business running one of its handlers if one ever strays.
      sigset_t all;
      sigfillset(&all);
      pthread_sigmask(SIG_BLOCK, &all, nullptr);

      std::unique_lock guard(lock);
      while (true) {
         (void)wake.wait_for(guard, stop, period, [] { return false; });
         if (stop.stop_requested()) break;
         writer.drain();
         std::fflush(file); // So a program that hangs or dies leaves its trace up to here
      }
   }

   std::FILE*                      file;
   std::mutex                      lock;
   std::condition_variable_any     wake; // Never notified. A stop request is what ends a wait early
   trace::stream_writer<file_sink> writer;
   std::jthread                    drainer; // Last, so it starts once the rest exists
};

} // namespace cyros::test

#endif // CYROS_TEST_TRACE_FILE_HPP
