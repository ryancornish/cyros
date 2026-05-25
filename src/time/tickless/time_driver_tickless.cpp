#include <cortos/time/time.hpp>
#include <cortos/port/port.h>

#include <array>
#include <cstdint>
#include <limits>

namespace cortos::time::tickless
{
   static constexpr uint32_t MAX_SCHEDULED_CALLBACKS = 16;

   struct Slot
   {
      uint32_t id{0};      // 0 = free
      uint64_t when{0};    // absolute deadline in driver ticks
      Callback cb{nullptr};
      void* arg{nullptr};
   };

   struct IrqGuard
   {
      uint32_t state;
      IrqGuard() noexcept : state(cortos_port_irq_save()) {}
      ~IrqGuard() { cortos_port_irq_restore(state); }

      IrqGuard(IrqGuard const&) = delete;
      IrqGuard& operator=(IrqGuard const&) = delete;
   };

   struct DriverState
   {
      bool initialised{false};
      uint32_t frequency_hz{0};
      uint32_t next_id{1};
      bool started{false};
      std::array<Slot, MAX_SCHEDULED_CALLBACKS> slots{};
   };

   static constinit DriverState ds{};

   static void fire_due_isr(uint64_t now_ticks) noexcept
   {
      // ISR context: free slot before invoking callback to avoid reentrancy hazards.
      for (auto& slot : ds.slots) {
         if (slot.id != 0 && slot.when <= now_ticks) {
            auto cb = slot.cb;
            auto arg = slot.arg;
            slot = Slot{};
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
         cortos_port_time_disarm();
      } else {
         // Arm earliest absolute deadline. If already due, port should deliver
         // an immediate or next-possible interrupt according to platform policy.
         cortos_port_time_arm(earliest);
      }
   }

   static void isr_trampoline(void*) noexcept
   {
      cortos::time::on_timer_isr();
   }

   static inline uint64_t ceil_div_u64(uint64_t a, uint64_t b) noexcept
   {
      return (a + b - 1) / b;
   }
} // namespace cortos::time::tickless


namespace cortos::time
{

void initialise(uint32_t frequency_hz)
{
   CORTOS_ASSERT(!tickless::ds.initialised);
   tickless::ds.initialised = true;
   tickless::ds.frequency_hz = frequency_hz;
}

void finalise()
{
   CORTOS_ASSERT(tickless::ds.initialised);
   tickless::ds = tickless::DriverState{};
}

[[nodiscard]] TimePoint now() noexcept
{
   return TimePoint{cortos_port_time_now()};
}

[[nodiscard]] Handle schedule_at(TimePoint tp, Callback cb, void* arg) noexcept
{
   if (!cb) {
      return {};
   }

   tickless::IrqGuard guard;

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

         return Handle{id};
      }
   }

   return {}; // out of slots
}

bool cancel(Handle h) noexcept
{
   if (h.id == 0) {
      return false;
   }

   tickless::IrqGuard guard;

   for (auto& slot : tickless::ds.slots) {
      if (slot.id == h.id) {
         slot = tickless::Slot{};
         tickless::rearm_locked();
         return true;
      }
   }

   return false;
}

[[nodiscard]] Duration from_milliseconds(uint32_t ms) noexcept
{
   const uint64_t ticks =
      tickless::ceil_div_u64(static_cast<uint64_t>(ms) * tickless::ds.frequency_hz, 1000ULL);
   return Duration{ticks};
}

[[nodiscard]] Duration from_microseconds(uint32_t us) noexcept
{
   const uint64_t ticks =
      tickless::ceil_div_u64(static_cast<uint64_t>(us) * tickless::ds.frequency_hz, 1'000'000ULL);
   return Duration{ticks};
}

void start() noexcept
{
   if (tickless::ds.started) {
      return;
   }

   cortos_port_time_register_isr_handler(&tickless::isr_trampoline, nullptr);

   // By convention, tick_hz == 0 means tickless / one-shot mode.
   cortos_port_time_setup(0);

   {
      tickless::IrqGuard guard;
      tickless::rearm_locked();
   }

   cortos_port_time_irq_enable();
   tickless::ds.started = true;
}

void stop() noexcept
{
   if (!tickless::ds.started) {
      return;
   }

   cortos_port_time_irq_disable();
   cortos_port_time_disarm();
   tickless::ds.started = false;
}

void on_timer_isr() noexcept
{
   const uint64_t now_ticks = cortos_port_time_now();

   tickless::fire_due_isr(now_ticks);

   tickless::IrqGuard guard;
   tickless::rearm_locked();
}

} // namespace cortos::time