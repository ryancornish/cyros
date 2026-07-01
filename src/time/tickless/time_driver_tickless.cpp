#include <cyros/time/time.hpp>

#include <cyros/port/port.h>
#include <cyros/port/port_time.h>

#include <array>
#include <cstdint>
#include <limits>

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

struct driver_state
{
   bool initialised{false};
   uint32_t frequency_hz{0};
   uint32_t next_id{1};
   bool started{false};
   std::array<slot, MAX_SCHEDULED_CALLBACKS> slots{};
};
constinit driver_state driver_instance{};

/**
 * @brief Claim the next non-zero handle id.
 */
uint32_t next_handle_id() noexcept
{
   uint32_t id = driver_instance.next_id++;
   if (id == 0) {
      id = driver_instance.next_id++; // skip the invalid id
   }
   return id;
}

/**
 * @brief Program the port one-shot to the earliest live deadline, or disarm it
 *        when nothing is scheduled. Call with interrupts already masked.
 */
void rearm_locked() noexcept
{
   uint64_t earliest = std::numeric_limits<uint64_t>::max();

   for (auto const& slot : driver_instance.slots) {
      if (slot.id != 0 && slot.when < earliest) {
         earliest = slot.when;
      }
   }

   if (earliest == std::numeric_limits<uint64_t>::max()) {
      cyros_port_time_disarm();
   } else {
      // If already due the port delivers an immediate or next-possible
      // interrupt per platform policy.
      cyros_port_time_arm(earliest);
   }
}

/**
 * @brief Fire every due slot. One-shots are freed before invoking. Recurring
 *        slots advance on a fixed grid from their scheduled deadline so cadence
 *        does not drift, skipping whole periods missed during a stall so a lag
 *        yields one fire and not a catch-up burst. The caller re-arms the port
 *        one-shot afterward, so an advanced recurring deadline is picked up.
 *
 * ISR context.
 */
void fire_due_isr(uint64_t now_ticks) noexcept
{
   for (auto& slot : driver_instance.slots) {
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

void isr_trampoline(void*) noexcept
{
   cyros::time::on_timer_isr();
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
   CYROS_ASSERT(!driver_instance.initialised);

   driver_instance.initialised = true;
   driver_instance.frequency_hz = frequency_hz;
}

void finalise()
{
   CYROS_ASSERT(driver_instance.initialised);

   driver_instance = driver_state{};
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

   for (auto& slot : driver_instance.slots) {
      if (slot.id == 0) {
         slot.id     = next_handle_id();
         slot.when   = tp.value;
         slot.period = 0;
         slot.cb     = cb;
         slot.arg    = arg;

         // Do not invoke inline while masked even if already due. Re-arm to the
         // earliest deadline and let the ISR path consume it.
         rearm_locked();

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

   for (auto& slot : driver_instance.slots) {
      if (slot.id == 0) {
         slot.id     = next_handle_id();
         slot.when   = cyros_port_time_now() + interval.value;
         slot.period = interval.value;
         slot.cb     = cb;
         slot.arg    = arg;

         rearm_locked();

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

   irq_guard guard;

   for (auto& slot : driver_instance.slots) {
      if (slot.id == h.id) {
         slot = {};
         rearm_locked();
         return true;
      }
   }

   return false;
}

[[nodiscard]] duration from_milliseconds(uint32_t ms) noexcept
{
   uint64_t const ticks = ceil_div_u64(static_cast<uint64_t>(ms) * driver_instance.frequency_hz, 1000ULL);
   return duration{ticks};
}

[[nodiscard]] duration from_microseconds(uint32_t us) noexcept
{
   uint64_t const ticks = ceil_div_u64(static_cast<uint64_t>(us) * driver_instance.frequency_hz, 1'000'000ULL);
   return duration{ticks};
}

void start() noexcept
{
   if (driver_instance.started) {
      return;
   }

   cyros_port_time_register_isr_handler(&isr_trampoline, nullptr);

   // tick_hz == 0 selects tickless / one-shot mode.
   cyros_port_time_setup(0);

   {
      irq_guard guard;
      rearm_locked();
   }

   cyros_port_time_irq_enable();
   driver_instance.started = true;
}

void stop() noexcept
{
   if (!driver_instance.started) {
      return;
   }

   cyros_port_time_irq_disable();
   cyros_port_time_disarm();
   driver_instance.started = false;
}

void on_timer_isr() noexcept
{
   uint64_t const now_ticks = cyros_port_time_now();

   fire_due_isr(now_ticks);

   irq_guard guard;
   rearm_locked();
}

} // namespace cyros::time
