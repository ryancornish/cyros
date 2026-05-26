#include <cortos/time/time.hpp>
#include <cortos/time/simulation.hpp>

#include <cortos/port/port.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace cortos::time::sim
{
   /**
    * @brief Scheduled callback event
    */
   struct event
   {
      uint32_t id{0};
      uint64_t when{0};
      callback cb{nullptr};
      void* arg{nullptr};
      bool cancelled{false};
   };

   /**
    * @brief Internal simulation time driver state
    */
   struct driver_state
   {
      simulation::Mode mode{simulation::Mode::Virtual};

      uint32_t tick_frequency_hz{1000};
      std::atomic<uint32_t> next_id{1};

      std::mutex mutex;
      std::vector<event> events;

      std::atomic<bool> running{false};
      std::thread realtime_thread;

      std::atomic<uint64_t> virtual_now{0};
      std::chrono::steady_clock::time_point realtime_epoch{};

      std::atomic<bool> started{false};
   };

   static driver_state* ds = nullptr;

   static void fire_due_callbacks(uint64_t now_ticks) noexcept
   {
      std::vector<event> due;

      {
         std::lock_guard lk(ds->mutex);

         for (auto& e : ds->events) {
            if (e.id != 0 && !e.cancelled && e.when <= now_ticks) {
               due.push_back(e);
               e.cancelled = true; // consume one-shot callback
            }
         }

         // Remove cancelled/consumed events
         std::erase_if(ds->events, [](event const& e) {
            return e.cancelled;
         });
      }

      for (auto& e : due) {
         if (e.cb) {
            e.cb(e.arg);
         }
      }
   }

   static void realtime_thread_main() noexcept
   {
      using namespace std::chrono_literals;

      while (ds->running.load(std::memory_order_acquire)) {
         std::this_thread::sleep_for(1ms);
         cortos::time::on_timer_isr();
      }
   }

   [[nodiscard]] static uint64_t realtime_now_ticks() noexcept
   {
      // Pre-start: treat as zero
      if (!ds->started.load(std::memory_order_acquire)) {
         return 0;
      }

      auto elapsed = std::chrono::steady_clock::now() - ds->realtime_epoch;
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
      ns = std::max<long>(ns, 0);

      return (static_cast<uint64_t>(ns) * ds->tick_frequency_hz) / 1'000'000'000ULL;
   }
} // namespace cortos::time::sim


namespace cortos::time
{

void initialise(uint32_t frequency_hz)
{
   CORTOS_ASSERT(sim::ds == nullptr);
   sim::ds = new sim::driver_state;
   sim::ds->tick_frequency_hz = frequency_hz;
}

void finalise()
{
   CORTOS_ASSERT(sim::ds != nullptr);
   delete sim::ds;
   sim::ds = nullptr;
}

[[nodiscard]] time_point now() noexcept
{
   CORTOS_ASSERT(sim::ds != nullptr);

   if (sim::ds->mode == simulation::Mode::Virtual) {
      return time_point{sim::ds->virtual_now.load(std::memory_order_relaxed)};
   }

   return time_point{sim::realtime_now_ticks()};
}

[[nodiscard]] handle schedule_at(time_point tp, callback cb, void* arg) noexcept
{
   CORTOS_ASSERT(sim::ds != nullptr);

   if (!cb) {
      return {};
   }

   uint32_t id = sim::ds->next_id.fetch_add(1, std::memory_order_relaxed);
   if (id == 0) {
      id = sim::ds->next_id.fetch_add(1, std::memory_order_relaxed);
   }

   {
      std::lock_guard lk(sim::ds->mutex);
      sim::ds->events.push_back(sim::event{
         .id = id,
         .when = tp.value,
         .cb = cb,
         .arg = arg,
         .cancelled = false
      });
   }

   return handle{id};
}

bool cancel(handle h) noexcept
{
   CORTOS_ASSERT(sim::ds != nullptr);

   if (h.id == 0) {
      return false;
   }

   std::lock_guard lk(sim::ds->mutex);

   for (auto& e : sim::ds->events) {
      if (e.id == h.id && !e.cancelled) {
         e.cancelled = true;
         return true;
      }
   }

   return false;
}

[[nodiscard]] duration from_milliseconds(uint32_t ms) noexcept
{
   CORTOS_ASSERT(sim::ds != nullptr);

   uint64_t const ticks = ((static_cast<uint64_t>(ms) * sim::ds->tick_frequency_hz) + 999) / 1000;

   return duration{ticks};
}

[[nodiscard]] duration from_microseconds(uint32_t us) noexcept
{
   CORTOS_ASSERT(sim::ds != nullptr);

   uint64_t const ticks = ((static_cast<uint64_t>(us) * sim::ds->tick_frequency_hz) + 999'999) / 1'000'000;

   return duration{ticks};
}

void start() noexcept
{
   CORTOS_ASSERT(sim::ds != nullptr);

   sim::ds->started.store(true, std::memory_order_release);

   if (sim::ds->mode == simulation::Mode::RealTime) {
      bool expected = false;
      if (!sim::ds->running.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
         return; // already running
      }

      sim::ds->realtime_epoch = std::chrono::steady_clock::now();
      sim::ds->realtime_thread = std::thread(sim::realtime_thread_main);
   }
}

void stop() noexcept
{
   CORTOS_ASSERT(sim::ds != nullptr);

   if (sim::ds->mode == simulation::Mode::RealTime) {
      bool was_running = sim::ds->running.exchange(false, std::memory_order_acq_rel);
      if (!was_running) {
         return;
      }

      if (sim::ds->realtime_thread.joinable()) {
         sim::ds->realtime_thread.join();
      }
   }
}

void on_timer_isr() noexcept
{
   CORTOS_ASSERT(sim::ds != nullptr);

   sim::fire_due_callbacks(now().value);
}

} // namespace cortos::time


namespace cortos::time::simulation
{

void set_mode(Mode mode) noexcept
{
   CORTOS_ASSERT(sim::ds != nullptr);
   CORTOS_ASSERT(!sim::ds->running.load(std::memory_order_acquire));

   sim::ds->mode = mode;
}

[[nodiscard]] Mode get_mode() noexcept
{
   CORTOS_ASSERT(sim::ds != nullptr);
   return sim::ds->mode;
}

void reset(time_point tp) noexcept
{
   CORTOS_ASSERT(sim::ds != nullptr);
   CORTOS_ASSERT(!sim::ds->running.load(std::memory_order_acquire));

   {
      std::lock_guard lock(sim::ds->mutex);
      sim::ds->events.clear();
   }

   sim::ds->next_id.store(1, std::memory_order_relaxed);
   sim::ds->virtual_now.store(tp.value, std::memory_order_relaxed);
   sim::ds->started.store(false, std::memory_order_relaxed);
   sim::ds->realtime_epoch = std::chrono::steady_clock::time_point{};
}

void advance_to(time_point tp) noexcept
{
   CORTOS_ASSERT(sim::ds != nullptr);
   CORTOS_ASSERT(sim::ds->mode == Mode::Virtual);

   uint64_t cur = sim::ds->virtual_now.load(std::memory_order_relaxed);
   uint64_t target = tp.value;

   target = std::max(target, cur); // monotonic clamp


   sim::ds->virtual_now.store(target, std::memory_order_release);
   cortos::time::on_timer_isr();
}

void advance_by(duration d) noexcept
{
   CORTOS_ASSERT(sim::ds != nullptr);
   CORTOS_ASSERT(sim::ds->mode == Mode::Virtual);

   uint64_t current = sim::ds->virtual_now.load(std::memory_order_relaxed);
   advance_to(time_point{current + d.value});
}

} // namespace cortos::time::simulation
