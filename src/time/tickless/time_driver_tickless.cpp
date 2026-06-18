#include <cyros/time/time.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_time.h>

#include <array>
#include <cstdint>
#include <limits>

namespace cyros::time::tickless
{
   static constexpr uint32_t MAX_SCHEDULED_CALLBACKS = 16;

   struct slot
   {
      uint32_t id{0};      // 0 = free
      uint64_t when{0};    // absolute deadline in driver ticks
      callback cb{nullptr};
      void* arg{nullptr};
   };

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

   static constinit driver_state ds{};

   static void fire_due_isr(uint64_t now_ticks) noexcept
   {
      // ISR context: free slot before invoking callback to avoid reentrancy hazards.
      for (auto& s : ds.slots) {
         if (s.id != 0 && s.when <= now_ticks) {
            auto cb = s.cb;
            auto arg = s.arg;
            s = slot{};
            cb(arg);
         }
      }
   }

   static void rearm_locked() noexcept
   {
      uint64_t earliest = std::numeric_limits<uint64_t>::max();

      for (auto const& slot : ds.slots) {
         if (slot.id != 0 && slot.when < earliest) {
            earliest = slot.when;
         }
      }

      if (earliest == std::numeric_limits<uint64_t>::max()) {
         cyros_port_time_disarm();
      } else {
         // Arm earliest absolute deadline. If already due, port should deliver
         // an immediate or next-possible interrupt according to platform policy.
         cyros_port_time_arm(earliest);
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
} // namespace cyros::time::tickless


namespace cyros::time
{

void initialise(uint32_t frequency_hz)
{
   CYROS_ASSERT(!tickless::ds.initialised);
   tickless::ds.initialised = true;
   tickless::ds.frequency_hz = frequency_hz;
}

void finalise()
{
   CYROS_ASSERT(tickless::ds.initialised);
   tickless::ds = tickless::driver_state{};
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

   tickless::irq_guard guard;

   for (auto& slot : tickless::ds.slots) {
      if (slot.id == 0) {
         uint32_t id = tickless::ds.next_id++;
         if (id == 0) {
            id = tickless::ds.next_id++;
         }

         slot.id = id;
         slot.when = tp.value;
         slot.cb = cb;
         slot.arg = arg;

         // If already due, do not invoke inline while IRQs are masked.
         // Rearm to the earliest deadline and let ISR path consume it.
         tickless::rearm_locked();

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

   tickless::irq_guard guard;

   for (auto& slot : tickless::ds.slots) {
      if (slot.id == h.id) {
         slot = tickless::slot{};
         tickless::rearm_locked();
         return true;
      }
   }

   return false;
}

[[nodiscard]] duration from_milliseconds(uint32_t ms) noexcept
{
   const uint64_t ticks =
      tickless::ceil_div_u64(static_cast<uint64_t>(ms) * tickless::ds.frequency_hz, 1000ULL);
   return duration{ticks};
}

[[nodiscard]] duration from_microseconds(uint32_t us) noexcept
{
   const uint64_t ticks =
      tickless::ceil_div_u64(static_cast<uint64_t>(us) * tickless::ds.frequency_hz, 1'000'000ULL);
   return duration{ticks};
}

void start() noexcept
{
   if (tickless::ds.started) {
      return;
   }

   cyros_port_time_register_isr_handler(&tickless::isr_trampoline, nullptr);

   // By convention, tick_hz == 0 means tickless / one-shot mode.
   cyros_port_time_setup(0);

   {
      tickless::irq_guard guard;
      tickless::rearm_locked();
   }

   cyros_port_time_irq_enable();
   tickless::ds.started = true;
}

void stop() noexcept
{
   if (!tickless::ds.started) {
      return;
   }

   cyros_port_time_irq_disable();
   cyros_port_time_disarm();
   tickless::ds.started = false;
}

void on_timer_isr() noexcept
{
   const uint64_t now_ticks = cyros_port_time_now();

   tickless::fire_due_isr(now_ticks);

   tickless::irq_guard guard;
   tickless::rearm_locked();
}

} // namespace cyros::time