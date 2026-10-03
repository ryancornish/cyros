#ifndef CYROS_WAITABLE_UTILITIES_VECTOR_HPP
#define CYROS_WAITABLE_UTILITIES_VECTOR_HPP

#include <cyros/config/config.hpp>
#include <cyros/kernel/waitable.hpp>
#include <cyros/port/port.h>

#include "threading_subsystem.hpp"

#include <cstddef>
#include <cstdint>
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

/**
 * @brief Arms every node of one pass of a block, and disarms each exactly once.
 *
 * LOAD-BEARING: waiter_record::chosen() is derived here, and it is correct only
 * while two facts hold:
 *
 *  1. A node leaves its queue by exactly one of two paths per pass: the owner
 *     disarms it (here), or a wake takes it (wait_queue's wake_one, wake_all
 *     and wake_one_and_commit, the only unlinks besides disarm).
 *  2. The owner disarms each node at most ONCE per pass. leave() records the
 *     nodes it has disarmed and the destructor skips them.
 *
 * Together they make "disarm found the node already gone" mean exactly "a
 * wake chose this waiter". Break either and chosen() lies, and a condition
 * variable either loses a notify (a missed mark: its waiter sleeps forever) or
 * invents one (a false mark: it returns as if notified, and a timed wait that
 * timed out reports no_timeout). Disarming the satisfied node twice, which
 * this guard did before chosen() existed and was harmless while nothing read
 * the result, is exactly the false mark. Adding any other way to unlink a
 * node is exactly the missed mark.
 *
 * Only the node of the waitable the record names is recorded. A wake of any
 * other source in the same wait must not mark the record.
 */
class waitable_arm_guard
{
   std::span<waitable_ref> waitables;
   wait_node_vector& nodes;
   waiter_record* record;
   std::uint32_t left{0}; // bit i: node i already disarmed this pass

   static_assert(config::max_wait_nodes <= 32, "left is a 32-bit mask");

public:
   waitable_arm_guard(std::span<waitable_ref> waitables, wait_node_vector& nodes, waiter_record* record)
      : waitables(waitables), nodes(nodes), record(record)
   {
      for (std::size_t i = 0; i < waitables.size(); ++i) {
         waitables[i].get().queue.arm(nodes[i]);
      }
   }

   /**
    * @brief Disarm node @p i now, rather than at the end of the pass.
    *
    * Leaving a queue without acquiring lowers its best-waiter priority, so a
    * holder becomes LESS urgent. Nothing to do about that: urgency is folded
    * from top() at the point of use, and observing the drop late only means
    * the holder ran at its old urgency slightly longer. A boost needs a
    * prompt, a de-boost does not.
    */
   void leave(std::size_t i) noexcept
   {
      auto const bit = std::uint32_t{1} << i;
      if ((left & bit) != 0) return;
      left |= bit;

      waitable& source = waitables[i].get();
      bool const was_still_queued = source.queue.disarm(nodes[i]);
      if (!was_still_queued && record != nullptr && record->source == &source) {
         record->chosen_by_wake = true;
      }
   }

   ~waitable_arm_guard()
   {
      for (std::size_t i = 0; i < waitables.size(); ++i) {
         leave(i);
      }
   }

   waitable_arm_guard(waitable_arm_guard&&) = delete;
   waitable_arm_guard(waitable_arm_guard const&) = delete;
   waitable_arm_guard& operator=(waitable_arm_guard&&) = delete;
   waitable_arm_guard& operator=(waitable_arm_guard const&) = delete;
};

} // namespace cyros

#endif // CYROS_WAITABLE_UTILITIES_VECTOR_HPP
