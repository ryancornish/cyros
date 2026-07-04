#include <cyros/time/time.hpp>
#include <cyros/time/simulation.hpp>

#include <cyros/config/config.hpp>
#include <cyros/port/port.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

/* ============================================================================
 * Local State & Helpers
 * ========================================================================= */

namespace
{

using cyros::time::callback;
using cyros::time::simulation::mode;

/**
 * @brief One scheduled callback.
 *
 * period == 0 is a one-shot consumed once at 'when'. period > 0 is recurring
 * and re-arms itself every 'period' ticks. A consumed or cancelled event is
 * erased from the vector after each fire pass.
 */
struct event
{
   uint32_t id{0};
   uint64_t when{0};
   uint64_t period{0};  // 0 one-shot, else the recurring interval
   callback cb{nullptr};
   void* arg{nullptr};
   bool cancelled{false};
};

/**
 * @brief Per-core scheduled callbacks.
 *
 * Each core schedules into its own list. A handle is scoped to the core that
 * created it, so cancel operates on the calling core's list.
 */
struct per_core_state
{
   std::mutex mutex;
   std::vector<event> events;
   std::atomic<uint32_t> next_id{1};
};

/**
 * @brief Simulation driver state.
 *
 * The clock and mode are driver-wide: simulation time is a single shared source
 * every core reads, matching a monotonic hardware counter. Only the scheduled
 * callbacks are per-core.
 */
struct driver_state
{
   mode active_mode{mode::virtual_time};

   uint32_t tick_frequency_hz{1000};

   std::atomic<bool> running{false};
   std::thread realtime_thread;

   std::atomic<uint64_t> virtual_now{0};
   std::chrono::steady_clock::time_point realtime_epoch{};

   std::atomic<bool> started{false};

   std::array<per_core_state, cyros::config::cores> per_core{};
};

driver_state* driver_instance = nullptr;

/// @brief The scheduled-callback state for the calling core.
per_core_state& this_core() noexcept
{
   return driver_instance->per_core[cyros_port_get_core_id()];
}

/**
 * @brief Claim the next non-zero handle id within a core's state.
 */
uint32_t next_handle_id(per_core_state& pc) noexcept
{
   uint32_t id = pc.next_id.fetch_add(1, std::memory_order_relaxed);
   if (id == 0) {
      id = pc.next_id.fetch_add(1, std::memory_order_relaxed);
   }
   return id;
}

/**
 * @brief Fire every due event across all cores outside their locks. One-shots
 *        are consumed and erased. Recurring events advance on a fixed grid from
 *        their scheduled deadline so cadence does not drift, skipping whole
 *        periods missed so a lag yields one fire and not a catch-up burst.
 *
 * Unlike the hardware drivers, where a core's own interrupt services only that
 * core, simulation owns global time and pumps it centrally, so one advance fires
 * every now-due timer regardless of which core scheduled it. Callbacks run in
 * the pump / caller context.
 */
void fire_due_callbacks(uint64_t now_ticks) noexcept
{
   for (auto& pc : driver_instance->per_core) {
      std::vector<event> due;

      {
         std::lock_guard lk(pc.mutex);

         for (auto& e : pc.events) {
            if (e.id != 0 && !e.cancelled && e.when <= now_ticks) {
               due.push_back(e);

               if (e.period == 0) {
                  e.cancelled = true;
               } else {
                  do {
                     e.when += e.period;
                  } while (e.when <= now_ticks);
               }
            }
         }

         std::erase_if(pc.events, [](event const& e) { return e.cancelled; });
      }

      for (auto& e : due) {
         if (e.cb) {
            e.cb(e.arg);
         }
      }
   }
}

void realtime_thread_main() noexcept
{
   using namespace std::chrono_literals;

   while (driver_instance->running.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(1ms);
      cyros::time::on_timer_isr();
   }
}

[[nodiscard]] uint64_t realtime_now_ticks() noexcept
{
   // Pre-start counts as zero.
   if (!driver_instance->started.load(std::memory_order_acquire)) {
      return 0;
   }

   auto elapsed = std::chrono::steady_clock::now() - driver_instance->realtime_epoch;
   auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
   ns = std::max<long>(ns, 0);

   return (static_cast<uint64_t>(ns) * driver_instance->tick_frequency_hz) / 1'000'000'000ULL;
}

} // namespace


/* ============================================================================
 * Time Driver Interface
 * ========================================================================= */

namespace cyros::time
{

void initialise(uint32_t frequency_hz)
{
   CYROS_ASSERT(driver_instance == nullptr);

   driver_instance = new driver_state;
   driver_instance->tick_frequency_hz = frequency_hz;
}

void finalise()
{
   CYROS_ASSERT(driver_instance != nullptr);

   delete driver_instance;
   driver_instance = nullptr;
}

[[nodiscard]] time_point now() noexcept
{
   CYROS_ASSERT(driver_instance != nullptr);

   if (driver_instance->active_mode == simulation::mode::virtual_time) {
      return time_point{driver_instance->virtual_now.load(std::memory_order_relaxed)};
   }

   return time_point{realtime_now_ticks()};
}

[[nodiscard]] handle schedule_at(time_point tp, callback cb, void* arg) noexcept
{
   CYROS_ASSERT(driver_instance != nullptr);

   if (!cb) {
      return {};
   }

   auto& pc = this_core();
   uint32_t id = next_handle_id(pc);

   {
      std::lock_guard lk(pc.mutex);
      pc.events.push_back(event{
         .id        = id,
         .when      = tp.value,
         .period    = 0,
         .cb        = cb,
         .arg       = arg,
         .cancelled = false,
      });
   }

   return handle{id};
}

[[nodiscard]] handle schedule_recurring(duration interval, callback cb, void* arg) noexcept
{
   CYROS_ASSERT(driver_instance != nullptr);

   if (!cb || interval.value == 0) {
      return {};
   }

   auto& pc = this_core();
   uint32_t id = next_handle_id(pc);

   {
      std::lock_guard lk(pc.mutex);
      pc.events.push_back(event{
         .id        = id,
         .when      = now().value + interval.value,
         .period    = interval.value,
         .cb        = cb,
         .arg       = arg,
         .cancelled = false,
      });
   }

   return handle{id};
}

bool cancel(handle h) noexcept
{
   CYROS_ASSERT(driver_instance != nullptr);

   if (h.id == 0) {
      return false;
   }

   auto& pc = this_core();
   std::lock_guard lk(pc.mutex);

   for (auto& event : pc.events) {
      if (event.id == h.id && !event.cancelled) {
         event.cancelled = true;
         return true;
      }
   }

   return false;
}

[[nodiscard]] duration from_milliseconds(uint32_t ms) noexcept
{
   CYROS_ASSERT(driver_instance != nullptr);

   uint64_t const ticks = ((static_cast<uint64_t>(ms) * driver_instance->tick_frequency_hz) + 999) / 1000;

   return duration{ticks};
}

[[nodiscard]] duration from_microseconds(uint32_t us) noexcept
{
   CYROS_ASSERT(driver_instance != nullptr);

   uint64_t const ticks = ((static_cast<uint64_t>(us) * driver_instance->tick_frequency_hz) + 999'999) / 1'000'000;

   return duration{ticks};
}

void start() noexcept
{
   CYROS_ASSERT(driver_instance != nullptr);

   driver_instance->started.store(true, std::memory_order_release);

   if (driver_instance->active_mode == simulation::mode::real_time) {
      bool expected = false;
      if (!driver_instance->running.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
         return; // already running
      }

      driver_instance->realtime_epoch = std::chrono::steady_clock::now();
      driver_instance->realtime_thread = std::thread(realtime_thread_main);
   }
}

void stop() noexcept
{
   CYROS_ASSERT(driver_instance != nullptr);

   if (driver_instance->active_mode == simulation::mode::real_time) {
      bool was_running = driver_instance->running.exchange(false, std::memory_order_acq_rel);
      if (!was_running) {
         return;
      }

      if (driver_instance->realtime_thread.joinable()) {
         driver_instance->realtime_thread.join();
      }
   }
}

void on_timer_isr() noexcept
{
   CYROS_ASSERT(driver_instance != nullptr);

   fire_due_callbacks(now().value);
}

} // namespace cyros::time


/* ============================================================================
 * Simulation Control Interface
 * ========================================================================= */

namespace cyros::time::simulation
{

void set_mode(mode m) noexcept
{
   CYROS_ASSERT(driver_instance != nullptr);
   CYROS_ASSERT(!driver_instance->running.load(std::memory_order_acquire));

   driver_instance->active_mode = m;
}

[[nodiscard]] mode get_mode() noexcept
{
   CYROS_ASSERT(driver_instance != nullptr);

   return driver_instance->active_mode;
}

void reset(time_point tp) noexcept
{
   CYROS_ASSERT(driver_instance != nullptr);
   CYROS_ASSERT(!driver_instance->running.load(std::memory_order_acquire));

   for (auto& pc : driver_instance->per_core) {
      std::lock_guard lock(pc.mutex);
      pc.events.clear();
      pc.next_id.store(1, std::memory_order_relaxed);
   }

   driver_instance->virtual_now.store(tp.value, std::memory_order_relaxed);
   driver_instance->started.store(false, std::memory_order_relaxed);
   driver_instance->realtime_epoch = std::chrono::steady_clock::time_point{};
}

void advance_to(time_point tp) noexcept
{
   CYROS_ASSERT(driver_instance != nullptr);
   CYROS_ASSERT(driver_instance->active_mode == mode::virtual_time);

   uint64_t cur = driver_instance->virtual_now.load(std::memory_order_relaxed);
   uint64_t target = std::max(tp.value, cur); // monotonic clamp

   driver_instance->virtual_now.store(target, std::memory_order_release);
   cyros::time::on_timer_isr();
}

void advance_by(duration d) noexcept
{
   CYROS_ASSERT(driver_instance != nullptr);
   CYROS_ASSERT(driver_instance->active_mode == mode::virtual_time);

   uint64_t current = driver_instance->virtual_now.load(std::memory_order_relaxed);
   advance_to(time_point{current + d.value});
}

} // namespace cyros::time::simulation
