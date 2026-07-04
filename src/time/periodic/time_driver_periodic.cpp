#include <cyros/time/time.hpp>

#include <cyros/config/config.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_time.h>

#include <array>
#include <cstdint>

/* ============================================================================
 * Local State & Helpers
 * ========================================================================= */

namespace
{

using cyros::time::callback;

/// @brief Fixed-size embedded-safe slot count, avoids heap allocation.
constexpr uint32_t MAX_SCHEDULED_CALLBACKS = 16;

/**
 * @brief One scheduled callback.
 *
 * period == 0 is a one-shot fired once at 'when'. period > 0 is recurring and
 * re-arms itself every 'period' ticks. id == 0 marks the slot free.
 */
struct slot
{
   uint32_t id{0};
   uint64_t when{0};    // absolute deadline in driver ticks
   uint64_t period{0};  // 0 one-shot, else the recurring interval
   callback cb{nullptr};
   void* arg{nullptr};
};

/// @brief RAII interrupt disable/restore guard.
struct irq_guard
{
   uint32_t state;

   irq_guard() noexcept : state(cyros_port_irq_save()) {}
   ~irq_guard() { cyros_port_irq_restore(state); }

   irq_guard(irq_guard const&) = delete;
   irq_guard& operator=(irq_guard const&) = delete;
};

/**
 * @brief Per-core scheduled callbacks.
 *
 * Each core owns an independent set of timer slots, scheduled into and serviced
 * from that core alone. A schedule call lands in the calling core's slots and
 * fires from that core's timer interrupt. See this_core_state().
 */
struct driver_state
{
   bool initialised{false};
   uint32_t tick_frequency_hz{0};
   uint32_t next_id{1};
   bool started{false};
   std::array<slot, MAX_SCHEDULED_CALLBACKS> slots{};
};
constinit std::array<driver_state, cyros::config::cores> driver_instances{};

/// @brief The scheduled-callback state for the calling core.
driver_state& this_core_state() noexcept
{
   return driver_instances[cyros_port_get_core_id()];
}

/**
 * @brief Claim the next non-zero handle id within a core's state.
 */
uint32_t next_handle_id(driver_state& ds) noexcept
{
   uint32_t id = ds.next_id++;
   if (id == 0) {
      id = ds.next_id++; // skip the invalid id
   }
   return id;
}

/**
 * @brief Fire every due slot for a core. One-shots are freed before invoking.
 *        Recurring slots advance on a fixed grid from their scheduled deadline
 *        so cadence does not drift, skipping whole periods missed during a stall
 *        so a lag yields one fire and not a catch-up burst.
 *
 * ISR context.
 */
void fire_due_isr(driver_state& ds, uint64_t now_ticks) noexcept
{
   for (auto& slot : ds.slots) {
      if (slot.id != 0 && slot.when <= now_ticks) {
         auto callback = slot.cb;
         auto* arg = slot.arg;

         if (slot.period == 0) {
            slot = {};
         } else {
            do {
               slot.when += slot.period;
            } while (slot.when <= now_ticks);
         }

         callback(arg);
      }
   }
}

void on_timer_isr(void*) noexcept
{
   fire_due_isr(this_core_state(), cyros_port_time_now());
}

uint64_t ceil_div_u64(uint64_t a, uint64_t b) noexcept
{
   return (a + b - 1) / b;
}

} // namespace


/* ============================================================================
 * Time Driver Interface
 * ========================================================================= */

namespace cyros::time
{

void initialise(uint32_t frequency_hz)
{
   auto& ds = this_core_state();
   CYROS_ASSERT(!ds.initialised);
   ds.initialised = true;
   ds.tick_frequency_hz = frequency_hz;
}

void finalise()
{
   auto& ds = this_core_state();
   CYROS_ASSERT(ds.initialised);
   ds = driver_state{};
}

[[nodiscard]] time_point now() noexcept
{
   return time_point{cyros_port_time_now()};
}

[[nodiscard]] handle schedule_at(time_point tp, callback cb, void* arg) noexcept
{
   if (!cb) {
      return {};
   }

   irq_guard guard;
   auto& ds = this_core_state();

   for (auto& slot : ds.slots) {
      if (slot.id == 0) {
         slot.id     = next_handle_id(ds);
         slot.when   = tp.value;
         slot.period = 0;
         slot.cb     = cb;
         slot.arg    = arg;

         // Periodic mode needs no one-shot re-arm. The tick ISR picks it up.
         return handle{slot.id};
      }
   }

   return {}; // out of slots
}

[[nodiscard]] handle schedule_recurring(duration interval, callback cb, void* arg) noexcept
{
   if (!cb || interval.value == 0) {
      return {};
   }

   irq_guard guard;
   auto& ds = this_core_state();

   for (auto& slot : ds.slots) {
      if (slot.id == 0) {
         slot.id     = next_handle_id(ds);
         slot.when   = cyros_port_time_now() + interval.value;
         slot.period = interval.value;
         slot.cb     = cb;
         slot.arg    = arg;

         return handle{slot.id};
      }
   }

   return {}; // out of slots
}

bool cancel(handle h) noexcept
{
   if (h.id == 0) {
      return false;
   }

   // Clears the slot by id under the guard, so a recurring slot cannot re-arm
   // in the ISR concurrently with this cancel. The handle is scoped to the core
   // that created it, so cancel operates on the calling core's slots.
   irq_guard guard;
   auto& ds = this_core_state();

   for (auto& slot : ds.slots) {
      if (slot.id == h.id) {
         slot = ::slot{};
         return true;
      }
   }

   return false;
}

[[nodiscard]] duration from_milliseconds(uint32_t ms) noexcept
{
   uint64_t const hz = cyros_port_time_freq_hz();
   uint64_t const ticks = ceil_div_u64(static_cast<uint64_t>(ms) * hz, 1000ULL);
   return duration{ticks};
}

[[nodiscard]] duration from_microseconds(uint32_t us) noexcept
{
   uint64_t const hz = cyros_port_time_freq_hz();
   uint64_t const ticks = ceil_div_u64(static_cast<uint64_t>(us) * hz, 1'000'000ULL);
   return duration{ticks};
}

void start() noexcept
{
   auto& ds = this_core_state();
   if (ds.started) {
      return;
   }

   CYROS_ASSERT(ds.tick_frequency_hz > 0);

   cyros_port_time_register_isr_handler(&on_timer_isr, nullptr);
   cyros_port_time_setup(ds.tick_frequency_hz);
   cyros_port_time_irq_enable();

   ds.started = true;
}

void stop() noexcept
{
   auto& ds = this_core_state();
   if (!ds.started) {
      return;
   }

   cyros_port_time_irq_disable();
   ds.started = false;
}

} // namespace cyros::time
