#ifndef CORTOS_SCHEDULER_HPP
#define CORTOS_SCHEDULER_HPP

#include "mpsc_ring_buffer.hpp"
#include "threading_subsystem.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>

namespace cortos
{

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
   alignas(CORTOS_PORT_STACK_ALIGN) std::array<std::byte, 4 * 1024> idle_stack{};

   thread_ready_matrix ready_matrix;

   uint32_t preempt_disable_depth{0};

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

   [[nodiscard]] uint32_t pinned_thread_count() const noexcept
   {
      return pinned_thread_counter.load(std::memory_order_relaxed);
   }

   void pin_thread(thread_control_block& tcb);

   void init_idle_thread();

   // Core-local operations (only called on owning core)
   void start() noexcept;

   void set_thread_ready(thread_control_block& tcb) noexcept;

   void drain_inbox() noexcept;

   // Cross-core safe posting API
   bool post_to_inbox(cross_core_request request) noexcept;

   /**
   * @brief Selects the next runnable thread for this core and performs a context switch.
   *
   * Invariants / contract:
   * - Called only by the owning core of this scheduler (no cross-core mutation).
   * - @c current_thread is non-null and is the thread currently executing on this core.
   * - On entry, @c current_thread->state is NEVER ready:
   *     - running    => treated as preempted/rotated and re-enqueued as ready (except idle).
   *     - blocked    => must already be removed from ready structures - not re-enqueued.
   *     - terminated => must not be re-enqueued.
   * - The currently running thread is not present in the ready matrix on entry.
   * - Any cross-core readying requests must be visible via @c drain_inbox() before selection.
   */
   void reschedule() noexcept;

   void prepare_block_current_thread(std::span<waitable* const> waitables);

   waitable::result commence_block_current_thread();

   void notify_block_current_thread(std::span<waitable* const> waitables) const;

   void disable_preemption();

   void enable_preemption();

   void reset();
};

} // namespace cortos

#endif // CORTOS_SCHEDULER_HPP
