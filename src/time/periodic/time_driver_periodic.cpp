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
 *
 * A recurring slot scheduled on a core before that core has started uses when==0
 * as a sentinel meaning 'anchor at release': start() rewrites it to now+period so
 * its cadence begins the moment the core's time goes live. A one-shot keeps its
 * absolute 'when', so when==0 on a one-shot is a real deadline, not the sentinel.
 */
struct slot
{
   uint32_t id{0};
   uint64_t when{0};    // absolute deadline in driver ticks (0 = anchor at release, recurring only)
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
 * @brief Driver-wide state, set once by initialise().
 *
 * The frequency is a property of the shared time source, so it lives here and
 * not per-core. It configures the tick in start(). from_milliseconds() and
 * from_microseconds() convert against the port frequency directly.
 */
struct time_config
{
   bool initialised{false};
   uint32_t frequency_hz{0};
};
constinit time_config tconfig{};

/**
 * @brief Per-core scheduled callbacks and this core's tick enablement.
 *
 * Each core owns an independent slot table, started by that core in start() and
 * serviced from that core's timer interrupt. A schedule lands in the calling
 * core's slots. A handle is scoped to the core that created it.
 */
struct timetable
{
   bool started{false};
   uint32_t next_id{1};
   std::array<slot, MAX_SCHEDULED_CALLBACKS> slots{};
};
constinit std::array<timetable, cyros::config::cores> timetables{};

/// @brief The scheduled-callback timetable for the calling core.
timetable& timetable_for_this_core() noexcept
{
   return timetables[cyros_port_get_core_id()];
}

/**
 * @brief Claim the next non-zero handle id within a core's state.
 */
uint32_t next_handle_id(timetable& ttable) noexcept
{
   uint32_t id = ttable.next_id++;
   if (id == 0) {
      id = ttable.next_id++; // skip the invalid id
   }
   return id;
}

/**
 * @brief Anchor recurring slots pended before start to the release moment.
 *
 * Rewrites the when==0 sentinel on recurring slots to now+period, so every timer
 * set up during init begins its cadence from the single instant the core starts,
 * not from whenever each was scheduled. Call with interrupts masked.
 */
void anchor_pended_recurring(timetable& ttable) noexcept
{
   uint64_t const now = cyros_port_time_now();

   for (auto& slot : ttable.slots) {
      if (slot.id != 0 && slot.period != 0 && slot.when == 0) {
         slot.when = now + slot.period;
      }
   }
}

/**
 * @brief Fire every due slot for a core. One-shots are freed before invoking.
 *        Recurring slots advance on a fixed grid from their scheduled deadline
 *        so cadence does not drift, skipping whole periods missed during a stall
 *        so a lag yields one fire and not a catch-up burst.
 *
 * ISR context.
 */
void fire_due_isr(timetable& ttable, uint64_t now_ticks) noexcept
{
   for (auto& slot : ttable.slots) {
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
   fire_due_isr(timetable_for_this_core(), cyros_port_time_now());
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
   // Global, once. Runs after kernel::initialise() and may run before
   // kernel::start(). Registers the one shared ISR entry and records the tick
   // frequency. Per-core timers are created later by each core's start().
   CYROS_ASSERT(!tconfig.initialised);
   tconfig.initialised = true;
   tconfig.frequency_hz = frequency_hz;

   cyros_port_time_register_isr_handler(&on_timer_isr, nullptr);
}

void finalise()
{
   CYROS_ASSERT(tconfig.initialised);
   tconfig = {};
   for (auto& ttable : timetables) {
      ttable = {};
   }
}

[[nodiscard]] time_point now() noexcept
{
   return time_point{cyros_port_time_now()};
}

[[nodiscard]] handle schedule_at(time_point tp, callback cb, void* arg) noexcept
{
   // Legal after initialise(). On a core that has not started, the slot is
   // pended and the core's tick services it at release.
   CYROS_ASSERT(tconfig.initialised);
   if (!cb) {
      return {};
   }

   irq_guard guard;
   auto& ttable = timetable_for_this_core();

   for (auto& slot : ttable.slots) {
      if (slot.id == 0) {
         slot.id     = next_handle_id(ttable);
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
   CYROS_ASSERT(tconfig.initialised);
   if (!cb || interval.value == 0) {
      return {};
   }

   irq_guard guard;
   auto& ttable = timetable_for_this_core();

   for (auto& slot : ttable.slots) {
      if (slot.id == 0) {
         slot.id     = next_handle_id(ttable);
         slot.period = interval.value;
         slot.cb     = cb;
         slot.arg    = arg;

         // Started: anchor the cadence now. Pended: when==0 sentinel, anchored
         // to the release moment by start().
         slot.when = ttable.started ? (cyros_port_time_now() + interval.value) : 0;

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

   CYROS_ASSERT(tconfig.initialised);

   // Clears the slot by id under the guard, so a recurring slot cannot re-arm
   // in the ISR concurrently with this cancel. The handle is scoped to the core
   // that created it. Legal before start(), which removes a still-pended slot.
   irq_guard guard;
   auto& ttable = timetable_for_this_core();

   for (auto& slot : ttable.slots) {
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

[[nodiscard]] uint64_t to_milliseconds(duration d) noexcept
{
   uint64_t const hz = cyros_port_time_freq_hz();
   return (d.value * 1000ULL + hz / 2) / hz;
}

[[nodiscard]] uint64_t to_microseconds(duration d) noexcept
{
   uint64_t const hz = cyros_port_time_freq_hz();
   return (d.value * 1'000'000ULL + hz / 2) / hz;
}

void start() noexcept
{
   // Per-core. Runs after kernel::start() in this core's context. Creates this
   // core's tick, anchors everything pended to this instant, then goes live.
   // Idempotent per core.
   auto& ttable = timetable_for_this_core();
   if (ttable.started) {
      return;
   }

   CYROS_ASSERT(tconfig.initialised);
   CYROS_ASSERT(tconfig.frequency_hz > 0);

   cyros_port_time_setup(tconfig.frequency_hz);

   {
      irq_guard guard;
      anchor_pended_recurring(ttable);
      ttable.started = true;
   }

   cyros_port_time_irq_enable();
}

void stop() noexcept
{
   auto& ttable = timetable_for_this_core();
   if (!ttable.started) {
      return;
   }

   cyros_port_time_irq_disable();
   ttable.started = false;
}

} // namespace cyros::time
