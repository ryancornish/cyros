#include <cyros/time/time.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port.h>

#include <array>
#include <cstdint>

namespace cyros::time::periodic
{
   /**
    * @brief Maximum number of simultaneously scheduled callbacks
    *
    * This is a fixed-size embedded-safe limit to avoid heap allocation.
    */
   static constexpr uint32_t MAX_SCHEDULED_CALLBACKS = 16;

   /**
    * @brief Scheduled callback slot
    */
   struct slot
   {
      uint32_t id{0};
      uint64_t when{0};
      callback cb{nullptr};
      void* arg{nullptr};
   };

   /**
    * @brief RAII interrupt disable/restore guard
    */
   struct irq_guard
   {
      uint32_t state;

      irq_guard() noexcept : state(cyros_port_irq_save()) {}
      ~irq_guard() { cyros_port_irq_restore(state); }

      irq_guard(irq_guard const&) = delete;
      irq_guard& operator=(irq_guard const&) = delete;
   };

   /**
    * @brief Internal periodic driver state
    */
   struct driver_state
   {
      bool initialised{false};
      uint32_t tick_frequency_hz{0};
      uint32_t next_id{1};
      bool started{false};
      std::array<slot, MAX_SCHEDULED_CALLBACKS> slots{};
   };

   static constinit driver_state ds{};

   static void fire_due_isr(uint64_t now_ticks) noexcept
   {
      // No heap, ISR-safe.
      // Free slot before invoking callback to avoid reentrancy hazards.
      for (auto& s : ds.slots) {
         if (s.id != 0 && s.when <= now_ticks) {
            auto cb = s.cb;
            auto arg = s.arg;
            s = slot{};
            cb(arg);
         }
      }
   }

   static void isr_trampoline(void*) noexcept
   {
      cyros::time::on_timer_isr();
   }

   static inline uint64_t ceil_div_u64(uint64_t a, uint64_t b) noexcept
   {
      return (a + b - 1) / b;
   }
} // namespace cyros::time::periodic


namespace cyros::time
{

void initialise(uint32_t frequency_hz)
{
   CYROS_ASSERT(!periodic::ds.initialised);
   periodic::ds.initialised = true;
   periodic::ds.tick_frequency_hz = frequency_hz;
}

void finalise()
{
   CYROS_ASSERT(periodic::ds.initialised);
   periodic::ds = periodic::driver_state{};
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

   // SMP note:
   // For now this assumes schedule_at() is called on the time core.
   // Future work can enqueue requests to the time core and poke it via IPI.

   periodic::irq_guard guard;

   for (auto& slot : periodic::ds.slots) {
      if (slot.id == 0) {
         uint32_t id = periodic::ds.next_id++;
         if (id == 0) {
            id = periodic::ds.next_id++; // avoid invalid handle id 0
         }

         slot.id = id;
         slot.when = tp.value;
         slot.cb = cb;
         slot.arg = arg;

         // In periodic mode, no one-shot rearm is required.
         // The periodic ISR will pick this callback up when due.
         return handle{id};
      }
   }

   return {}; // out of slots
}

bool cancel(handle h) noexcept
{
   if (h.id == 0) {
      return false;
   }

   // Same SMP note as schedule_at().
   periodic::irq_guard guard;

   for (auto& slot : periodic::ds.slots) {
      if (slot.id == h.id) {
         slot = periodic::slot{};
         return true;
      }
   }

   return false;
}

[[nodiscard]] duration from_milliseconds(uint32_t ms) noexcept
{
   const uint64_t f = cyros_port_time_freq_hz();
   const uint64_t ticks = periodic::ceil_div_u64(static_cast<uint64_t>(ms) * f, 1000ULL);
   return duration{ticks};
}

[[nodiscard]] duration from_microseconds(uint32_t us) noexcept
{
   const uint64_t f = cyros_port_time_freq_hz();
   const uint64_t ticks = periodic::ceil_div_u64(static_cast<uint64_t>(us) * f, 1'000'000ULL);
   return duration{ticks};
}

void start() noexcept
{
   if (periodic::ds.started) {
      return;
   }

   CYROS_ASSERT(periodic::ds.tick_frequency_hz > 0);

   cyros_port_time_register_isr_handler(&periodic::isr_trampoline, nullptr);
   cyros_port_time_setup(periodic::ds.tick_frequency_hz);
   cyros_port_time_irq_enable();

   periodic::ds.started = true;
}

void stop() noexcept
{
   if (!periodic::ds.started) {
      return;
   }

   cyros_port_time_irq_disable();
   periodic::ds.started = false;
}

void on_timer_isr() noexcept
{
   const uint64_t now_ticks = cyros_port_time_now();
   periodic::fire_due_isr(now_ticks);
}

} // namespace cyros::time
