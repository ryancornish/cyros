#include "threading_subsystem.hpp"
#include "align.hpp"

namespace cyros
{

static_assert(sizeof(thread_control_block) + 4096 <= thread::min_stack_size,
              "Public constant thread::min_stack_size no longer accurately reflects the true min_stack_size");

thread::~thread()
{
   if (tcb == nullptr) return; // thread handle has been moved from, or is otherwise empty

   CYROS_ASSERT(tcb->state == thread_state::terminated);
   tcb->public_thread_handle = nullptr;
}

thread::thread(thread&& other) noexcept : tcb(other.tcb)
{
   other.tcb = nullptr;
   tcb->public_thread_handle = this;
}

thread& thread::operator=(thread&& other) noexcept
{
   tcb = other.tcb;
   other.tcb = nullptr;
   tcb->public_thread_handle = this;
   return *this;
}

[[nodiscard]] thread::id thread::get_id() const noexcept
{
   CYROS_ASSERT(tcb != nullptr);

   return tcb->id;
}

[[nodiscard]] thread::priority thread::get_priority() const noexcept
{
   CYROS_ASSERT(tcb != nullptr);

   return tcb->effective_priority;
}

void thread::join() noexcept
{
   CYROS_ASSERT(tcb != nullptr);

   this_thread::wait_on(tcb->termination);
}


thread_control_block::thread_control_block(uint32_t id,
                                           thread::priority priority,
                                           core_affinity affinity,
                                           std::span<std::byte> stack,
                                           thread::entry_fn&& entry,
                                           thread* public_thread_handle)
   : id(id),
     base_priority(priority),
     effective_priority(priority),
     public_thread_handle(public_thread_handle),
     affinity(affinity),
     stack(stack),
     entry(std::move(entry))
{
   cyros_port_context_init(context(), stack.data(), stack.size(), thread_launcher, this);
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
   tcb->next = tcb->prev = tcb;
   return tcb;
}

void thread_ready_queue::remove(thread_control_block& tcb) noexcept
{
   CYROS_ASSERT(tcb.is_enqueued());  // can only remove enqueued threads

   if (tcb.prev) tcb.prev->next = tcb.next; else head = tcb.next;
   if (tcb.next) tcb.next->prev = tcb.prev; else tail = tcb.prev;
   tcb.next = tcb.prev = &tcb;
   // Invariant: if one end is null, both are null
   CYROS_ASSERT((head == nullptr) == (tail == nullptr));
}


void thread_ready_matrix::enqueue_thread(thread_control_block& tcb) noexcept
{
   CYROS_ASSERT_OP(tcb.effective_priority, <, config::max_priorities);
   CYROS_ASSERT_OP(tcb.state, ==, thread_state::ready);

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
