#include "threading_subsystem.hpp"

namespace cyros
{

thread_control_block::thread_control_block(uint32_t id, thread::priority priority, core_affinity affinity, std::span<std::byte> stack, thread::entry_fn&& entry)
   : id(id), base_priority(priority), effective_priority(priority), affinity(affinity), stack(stack), entry(std::move(entry))
{
   cyros_port_context_init(context(), stack.data(), stack.size(), thread_launcher, this);
}

void thread_control_block::prepare_block(std::span<waitable* const> waitables)
{
   CYROS_ASSERT(state == thread_state::running);
   CYROS_ASSERT(!waitables.empty());
   CYROS_ASSERT(waitables.size() <= config::max_wait_nodes);

   wait_operation.begin(waitables.size());

   // Allocate and enqueue nodes
   for (std::size_t i = 0; auto* waitable : waitables) {
      CYROS_ASSERT(waitable != nullptr);

      wait_node* wait_node = wait_nodes.alloc(wait_operation, *waitable, i++);
      CYROS_ASSERT(wait_node != nullptr);

      waitable->add(*wait_node);
   }

   state = thread_state::blocked;
}

void thread_control_block::notify_block(std::span<waitable* const> waitables) const
{
   // Snapshot once: all blocks are given the same snapshot. This disregards
   // all side-effects on_thread_blocked invocations have on the effective priority.
   auto const waiter = create_waiter();

   for (auto* waitable : waitables) {
      waitable->on_thread_blocked(waiter);
   }
}

waitable::result thread_control_block::commence_block()
{
   cyros_port_thread_yield();

   // When we resume, winner info is in wait_operation
   return waitable::result{
      .index    = wait_operation.winner_index,
      .acquired = wait_operation.acquired,
   };
}

void thread_control_block::teardown_wait_group(wait_group& group) noexcept
{
   // Snapshot once: all removals in this teardown relate to the same waiter (this thread).
   auto const waiter = create_waiter();
   waitable_ref_vector<config::max_wait_nodes> waitables;

   // First pass: collect involved waitables
   wait_nodes.for_each_active([&](wait_node& node) {
      if (node.active_group != &group) return;
      if (node.active_waitable) {
         waitables.push(node.active_waitable);
      }
   }, &group);

   {
      waitable_group_lock lock_group(waitables);

      // Second pass: unlink and free under lock
      wait_nodes.for_each_active([&](wait_node& node) {
         if (node.active_group != &group) return;

         if (node.active_waitable) {
            node.active_waitable->remove(node);
         }
         wait_nodes.free(node);
      }, &group);
   }

   // Third pass: hooks after unlock
   for (auto* waitable : waitables) {
      waitable->on_thread_removed(waiter);
   }
}


stack_layout::stack_layout(std::span<std::byte> const buffer, std::size_t const tls_bytes)
{
   auto const base = reinterpret_cast<std::uintptr_t>(buffer.data());
   auto const end  = base + buffer.size();

   // TCB at very top, aligned down
   auto const tcb_start = align_down(end - sizeof(thread_control_block), alignof(thread_control_block));
   tcb = reinterpret_cast<thread_control_block*>(tcb_start);

   // TLS just below TCB
   auto const tls_size = align_up(tls_bytes, alignof(std::max_align_t));
   auto const tls_top  = tcb_start;
   auto const tls_base = align_down(tls_top - tls_size, alignof(std::max_align_t));

   CYROS_ASSERT_OP(tls_base, >=, base); // Buffer too small for TLS+TCB

   auto const tls_offset = static_cast<std::size_t>(tls_base - base);
   auto const tls_length = static_cast<std::size_t>(tls_top  - tls_base);
   tls_region = buffer.subspan(tls_offset, tls_length); // zero-length span if tls_bytes == 0

   // User stack: everything below TLS
   auto const stack_len = static_cast<std::size_t>(tls_base - base);
   CYROS_ASSERT_OP(stack_len, >, 64); // Buffer too small after carving TCB/TLS

   user_stack = buffer.subspan(0, stack_len);
}


void thread_ready_queue::push_back(thread_control_block& tcb) noexcept
{
   CYROS_ASSERT(!tcb.is_enqueued());
   CYROS_ASSERT_OP(tcb.state, ==, thread_control_block::thread_state::ready); // Thread must be ready to be enqueued
   tcb.next = nullptr;
   tcb.prev = tail;
   if (tail) tail->next = &tcb; else head = &tcb;
   tail = &tcb;
}

thread_control_block* thread_ready_queue::pop_front() noexcept
{
   if (empty()) return nullptr;
   auto* tcb = head;
   head = tcb->next;
   if (head) head->prev = nullptr; else tail = nullptr;
   tcb->next = tcb->prev = nullptr;
   return tcb;
}

void thread_ready_queue::remove(thread_control_block& tcb) noexcept
{
   CYROS_ASSERT(&tcb == head || tcb.prev != nullptr);
   CYROS_ASSERT(&tcb == tail || tcb.next != nullptr);

   if (tcb.prev) tcb.prev->next = tcb.next; else head = tcb.next;
   if (tcb.next) tcb.next->prev = tcb.prev; else tail = tcb.prev;
   tcb.next = tcb.prev = nullptr;
   // Invariant: if one end is null, both are null
   CYROS_ASSERT((head == nullptr) == (tail == nullptr));
}


void thread_ready_matrix::enqueue_thread(thread_control_block& tcb) noexcept
{
   CYROS_ASSERT_OP(tcb.effective_priority, <, config::max_priorities);
   CYROS_ASSERT_OP(tcb.state, ==, thread_control_block::thread_state::ready); // Can only enqueue ready threads!
   matrix[tcb.effective_priority].push_back(tcb);
   bitmap |= (1u << tcb.effective_priority);
}

thread_control_block* thread_ready_matrix::pop_best_thread() noexcept
{
   if (bitmap == 0) return nullptr;
   auto const priority = std::countr_zero(bitmap);
   thread_control_block* tcb = matrix[priority].pop_front();
   if (matrix[priority].empty()) bitmap &= ~(1u << priority);
   return tcb;
}

void thread_ready_matrix::remove_thread(thread_control_block& tcb) noexcept
{
   auto const priority = tcb.effective_priority;
   matrix[priority].remove(tcb);
   if (matrix[priority].empty()) bitmap &= ~(1u << priority);
}

} // namespace cyros
