#ifndef CYROS_SCHEDULER_HPP
#define CYROS_SCHEDULER_HPP

#include "mpsc_ring_buffer.hpp"
#include "threading_subsystem.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>

namespace cyros
{

/**
 * @brief TODO
 */
enum class schedule_hint
{
   unwarranted, ///< TODO
   warranted,   ///< TODO
};

// Implemented by the kernel
schedule_hint kernel_request_thread_ready(thread_control_block& tcb);

// Implemented by the kernel
void idle_task();

struct cross_core_request
{
   enum class request_type : uint8_t
   {
      set_thread_ready, // Enqueue a TCB into this core's ready queue
   };
   static constexpr auto set_thread_ready = request_type::set_thread_ready;

   request_type type{};
   thread_control_block* tcb{nullptr};
};

class scheduler
{
private:
   std::uint32_t const core_id;
   std::atomic<uint32_t> pinned_thread_counter{0};
   thread_control_block* current_thread{nullptr};
   thread_control_block*    idle_thread{nullptr};
   alignas(CYROS_PORT_STACK_ALIGN) std::array<std::byte, thread::min_stack_size> idle_stack{};

   thread_ready_matrix ready_matrix;

   std::atomic<bool> inbox_poke_pending{false};
   static constexpr uint32_t inbox_cap = 64; // tune later
   mpsc_ring_buffer<cross_core_request, inbox_cap> inbox;

public:
   static constexpr uint32_t idle_thread_id = 0; // Reserved

   constexpr explicit scheduler(std::size_t core_id) : core_id(core_id) {};

   ~scheduler() = default;
   scheduler(scheduler&&) = delete;
   scheduler(scheduler const&) = delete;
   scheduler& operator=(scheduler&&) = delete;
   scheduler& operator=(scheduler const&) = delete;

   [[nodiscard]] constexpr uint32_t current_thread_id() const noexcept
   {
      return current_thread ? current_thread->id : 0;
   }

   [[nodiscard]] constexpr uint8_t current_thread_priority() const noexcept
   {
      return current_thread ? current_thread->effective_priority : 0;
   }

   [[nodiscard]] constexpr thread_control_block* current_thread_reference() const noexcept
   {
      return current_thread;
   }

   [[nodiscard]] uint32_t pinned_thread_count() const noexcept
   {
      return pinned_thread_counter.load(std::memory_order_relaxed);
   }

   [[nodiscard]] bool inbox_pending() const noexcept
   {
      return inbox_poke_pending.load(std::memory_order_relaxed);
   }

   void pin_thread(thread_control_block& tcb);

   void init_idle_thread();

   // Core-local operations (only called on owning core)
   void start() noexcept;

   [[nodiscard]] schedule_hint set_thread_ready(thread_control_block& tcb) noexcept;

   void set_thread_running(thread_control_block& tcb) noexcept;

   void set_thread_blocked(thread_control_block& tcb) noexcept;

   void set_thread_terminated(thread_control_block& tcb) noexcept;

   void drain_inbox() noexcept;

   // Cross-core safe posting API
   [[nodiscard]] bool post_to_inbox(cross_core_request request) noexcept;

   /**
   * @brief Select the next runnable thread for this core and switch to it.
   *
   * Pick the highest-priority ready thread for this core and context-switch
   * to it, parking or re-enqueuing the outgoing thread as its state and
   * disposition dictate. Runs on the owning core in the current thread's
   * context, driven by a yield, a wake, or a preemption IPI. See the .cpp
   * for the full transition policy.
   */
   void reschedule() noexcept;

   void reset();
};

} // namespace cyros

#endif // CYROS_SCHEDULER_HPP