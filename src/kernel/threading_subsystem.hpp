#ifndef CORTOS_THREADING_SUBSYSTEM_HPP
#define CORTOS_THREADING_SUBSYSTEM_HPP

#include <cortos/kernel/waitable.hpp>
#include <cortos/config/config.hpp>
#include <cortos/port/port.h>

#include "align.hpp"
#include "waitable_utilities.hpp"
#include "wait_subsystem.hpp"

#include <bitset>
#include <limits>

namespace cortos
{

void thread_launcher(void* tcb_ptr);

class thread_termination : public waitable
{
   std::atomic<bool> terminated{false};

public:
   [[nodiscard]] bool has_terminated()
   {
      return terminated.load(std::memory_order_acquire);
   }

   void terminate()
   {
      bool expected = false;
      CORTOS_ASSERT(terminated.compare_exchange_strong(expected, true, std::memory_order_release));

      signal_all(false);
   }
};

struct thread_control_block
{
   enum class thread_state : uint8_t { ready, running, blocked, terminated };
   thread_state state{thread_state::ready};

   // Intrusive 'linked-list' links for a thread_ready_queue
   thread_control_block* next{nullptr};
   thread_control_block* prev{nullptr};

   uint32_t id;
   uint8_t base_priority;
   uint8_t effective_priority; // Can change dynamically

   // Core pinning
   std::uint32_t pinned_core{0};
   core_affinity  affinity;

   std::span<std::byte> stack;
   thread::entry_fn entry;

   wait_group wait_operation{};
   wait_node_pool wait_nodes{this};

   // Thread-joining waitable
   thread_termination termination;

   // Opaque, in-place port context storage
   alignas(CORTOS_PORT_CONTEXT_ALIGN) std::array<std::byte, CORTOS_PORT_CONTEXT_SIZE> context_storage{};
   [[nodiscard]] constexpr auto*       context()       noexcept { return reinterpret_cast<cortos_port_context_t*      >(context_storage.data()); }
   [[nodiscard]] constexpr auto const* context() const noexcept { return reinterpret_cast<cortos_port_context_t const*>(context_storage.data()); }

   [[nodiscard]] constexpr bool is_enqueued() const noexcept
   {
      return next != nullptr || prev != nullptr;
   }

   [[nodiscard]] constexpr bool is_higher_priority_than(thread_control_block& rhs)  const noexcept
   {
      return effective_priority < rhs.effective_priority;
   }
   [[nodiscard]] constexpr bool is_higher_priority_than(uint8_t priority_level) const noexcept
   {
      return effective_priority < priority_level;
   }

   [[nodiscard]] waitable::waiter create_waiter() const noexcept
   {
      return {
         .id                 = id,
         .base_priority      = base_priority,
         .effective_priority = effective_priority,
         .pinned_core        = pinned_core,
         .affinity           = affinity,
      };
   }

   thread_control_block(uint32_t id, thread::priority priority, core_affinity affinity, std::span<std::byte> stack, thread::entry_fn&& entry);

   void prepare_block(std::span<waitable* const> waitables);

   void notify_block(std::span<waitable* const> waitables) const;

   waitable::result commence_block();

   void teardown_wait_group(wait_group& group) noexcept;
};


/**
 * Carves a user-provided buffer region into:
 * +----------------------+ <-- buffer's end (high address)
 * +   thread_control_block   + (Fixed size)
 * +----------------------+
 * + Thread-local storage + (Variable size)
 * +----------------------+
 * +     User's stack     +
 * +----------------------+ <-- buffer's base (low address)
 */
struct stack_layout
{
   thread_control_block* tcb;
   std::span<std::byte> tls_region;
   std::span<std::byte> user_stack;

   explicit stack_layout(std::span<std::byte> buffer, std::size_t tls_bytes);
};

class thread_ready_queue
{
private:
   thread_control_block* head{nullptr};
   thread_control_block* tail{nullptr};

public:
   [[nodiscard]] constexpr bool empty() const noexcept
   {
      return !head;
   }

   [[nodiscard]] constexpr bool has_peer() const noexcept
   {
      return head && head != tail;
   }

   [[nodiscard]] constexpr thread_control_block* front() const noexcept
   {
      return head;
   }

   // This walks the linked list so isn't 'free'
   [[nodiscard]] constexpr std::size_t size() const noexcept
   {
      std::size_t n = 0;
      for (auto* tcb = head; tcb; tcb = tcb->next) ++n;
      return n;
   }

   void push_back(thread_control_block& tcb) noexcept;

   thread_control_block* pop_front() noexcept;

   void remove(thread_control_block& tcb) noexcept;
};

class thread_ready_matrix
{
private:
   static constexpr std::size_t bitmap_bits = std::numeric_limits<uint32_t>::digits;
   std::array<thread_ready_queue, config::max_priorities> matrix{};
   uint32_t bitmap{0};
   static_assert(config::max_priorities <= bitmap_bits, "bitmap cannot hold that many priorities!");

public:
   constexpr thread_ready_matrix() = default;

   [[nodiscard]] constexpr int best_priority() const noexcept
   {
      return bitmap ? std::countr_zero(bitmap) : -1;
   }

   [[nodiscard]] constexpr bool empty() const noexcept
   {
      return bitmap == 0;
   }

   [[nodiscard]] constexpr bool empty_at(uint32_t priority) const noexcept
   {
      return matrix[priority].empty();
   }

   [[nodiscard]] constexpr bool has_peer(uint32_t priority) const noexcept
   {
      return matrix[priority].has_peer();
   }

   [[nodiscard]] constexpr std::size_t size_at(uint32_t priority) const noexcept
   {
      return matrix[priority].size();
   }

   [[nodiscard]] constexpr std::bitset<bitmap_bits> bitmap_view() const noexcept
   {
      return {bitmap};
   }

   [[nodiscard]] constexpr thread_control_block* peek_best_thread() const noexcept
   {
      if (bitmap == 0) return nullptr;
      auto const priority = std::countr_zero(bitmap);
      return matrix[priority].front();
   }

   void enqueue_thread(thread_control_block& tcb) noexcept;

   thread_control_block* pop_best_thread() noexcept;

   void remove_thread(thread_control_block& tcb) noexcept;
};

} // namespace cortos

#endif // CORTOS_THREADING_SUBSYSTEM_HPP
