#ifndef CYROS_WAITABLE_UTILITIES_VECTOR_HPP
#define CYROS_WAITABLE_UTILITIES_VECTOR_HPP

#include <cyros/config/config.hpp>
#include <cyros/kernel/waitable.hpp>
#include <cyros/port/port.h>

#include "threading_subsystem.hpp"

#include <cstddef>
#include <inplace_vector>

namespace cyros
{

/**
 * @brief The wait nodes one blocking call owns, one per waitable.
 */
class wait_node_vector
{
private:
   using wait_node = wait_queue::wait_node;

   std::inplace_vector<wait_node, config::max_wait_nodes> store;

public:
   constexpr wait_node_vector() = default;

   /**
    * @brief One node per waitable, all owned by @p tcb.
    */
   wait_node_vector(std::size_t node_count, thread_control_block& tcb)
   {
      CYROS_ASSERT_OP(node_count, <=, config::max_wait_nodes);

      for (std::size_t i = 0; i < node_count; ++i) {
         auto slot = store.try_push_back(wait_node{
            .owner = &tcb,
            .next  = nullptr,
         });
         CYROS_ASSERT(slot.has_value());
      }
   }

   [[nodiscard]] std::size_t size() const noexcept
   {
      return store.size();
   }

   [[nodiscard]] bool empty() const noexcept
   {
      return store.empty();
   }

   wait_node const& operator[](std::size_t index) const noexcept
   {
      CYROS_ASSERT(index < store.size());

      return store[index];
   }

   wait_node& operator[](std::size_t index) noexcept
   {
      CYROS_ASSERT(index < store.size());

      return store[index];
   }
};

class waitable_arm_guard
{
   std::span<waitable_ref> waitables;
   wait_node_vector& nodes;
public:
   waitable_arm_guard(std::span<waitable_ref> waitables, wait_node_vector& nodes)
      : waitables(waitables), nodes(nodes)
   {
      for (std::size_t i = 0; i < waitables.size(); ++i) {
         waitables[i].get().queue.arm(nodes[i]);
      }
   }

   ~waitable_arm_guard()
   {
      for (std::size_t i = 0; i < waitables.size(); ++i) {
         // Return value deliberately ignored. Leaving a queue without acquiring
         // lowers its best-waiter priority, so the holder becomes LESS urgent.
         // Nothing to do: urgency is folded from top() at the point of use, and
         // observing the drop late only means the holder ran at its old urgency
         // slightly longer, which is safe. A boost needs a prompt, a de-boost
         // does not.
         waitables[i].get().queue.disarm(nodes[i]);
      }
   }

   waitable_arm_guard(waitable_arm_guard&&) = delete;
   waitable_arm_guard(waitable_arm_guard const&) = delete;
   waitable_arm_guard& operator=(waitable_arm_guard&&) = delete;
   waitable_arm_guard& operator=(waitable_arm_guard const&) = delete;
};

} // namespace cyros

#endif // CYROS_WAITABLE_UTILITIES_VECTOR_HPP
