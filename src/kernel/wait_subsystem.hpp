#ifndef CORTOS_WAIT_SUBSYSTEM_HPP
#define CORTOS_WAIT_SUBSYSTEM_HPP

#include <cortos/kernel/waitable.hpp>
#include <cortos/config/config.hpp>
#include <cortos/port/port.h>

#include <limits>

namespace cortos
{

struct thread_control_block;
struct wait_group;

enum class ready_action : uint8_t
{
   none,
   reschedule,
};

/**
 * @brief Intrusive wait-queue node for a thread waiting on a waitable
 *
 * A wait_node represents a single thread's participation in a wait operation
 * on a specific waitable. Nodes are allocated from a per-thread pool and are
 * linked into a waitable's wait queue for the duration of the wait.
 *
 * Each wait_for_any() call creates one wait_node per waitable involved.
 * Nodes are removed and returned to the pool when the wait completes.
 */
struct wait_node
{
   static constexpr uint8_t invalid_index = std::numeric_limits<uint8_t>::max();
   // Pool bookkeeping
   uint8_t slot{invalid_index};
   bool  active{false};

   // Intrusive links for the waitable's waiter queue
   wait_node* next{nullptr};
   wait_node* prev{nullptr};

   waitable*             active_waitable{nullptr};
   wait_group*           active_group{nullptr};
   thread_control_block* tcb{nullptr};

   // Which index in wait_for_any({span}) this node corresponds to
   uint8_t index{invalid_index};

   constexpr void reset() noexcept
   {
      active   = false;
      next = prev = nullptr;
      active_waitable = nullptr;
      active_group    = nullptr;
      index    = invalid_index;
      // tcb and slot are explicitly not reset
   }

   [[nodiscard]] constexpr bool is_enqueued() const noexcept
   {
      return next || prev;
   }

   /**
    * @brief Attempt to wake the owning thread for this wait node
    *
    * Resolves a wait_for_any() race by attempting to "win" the associated wait_group.
    * If this node wins, all wait nodes participating in the same wait_group are
    * torn down (unlinked from their Waitables and returned to the per-thread pool),
    * and the owning thread is made runnable on its pinned core.
    *
    * If the wait_group has already been won by another node, this call is a no-op.
    *
    * @param acquired True if the woken thread acquired a resource as part of the wake
    *                 (e.g., mutex handoff). This is recorded in the wait_group result.
    */
   [[nodiscard]] ready_action wake_thread(bool acquired) const noexcept;
};

/**
 * @brief Per-thread pool of wait_node objects
 *
 * Each thread owns a fixed-capacity pool of WaitNodes used to represent
 * active wait operations. This avoids dynamic allocation in the kernel
 * and guarantees bounded resource usage.
 *
 * A single wait_for_any() operation may allocate multiple nodes from the
 * pool (one per waitable). All nodes are reclaimed when the wait completes.
 */
class wait_node_pool
{
   static constexpr std::size_t N = config::max_wait_nodes;
   static constexpr uint32_t ALL_NODES_FREE = (N == 32) ? std::numeric_limits<uint32_t>::max() : (1u << static_cast<uint32_t>(N)) - 1u;
   std::array<wait_node, N> nodes{};
   uint32_t free_mask{ALL_NODES_FREE};

public:
   static_assert(N > 0, "max_wait_nodes must be > 0");
   static_assert(N <= std::numeric_limits<uint32_t>::digits, "wait_node_pool currently supports up to 32 nodes via uint32_t mask");

   constexpr explicit wait_node_pool(thread_control_block* tcb) noexcept
   {
      for (std::size_t i = 0; auto& node : nodes) {
         node.tcb  = tcb;
         node.slot = i++;
      }
   }
   ~wait_node_pool() = default;

   wait_node_pool(wait_node_pool const&)            = delete;
   wait_node_pool& operator=(wait_node_pool const&) = delete;
   wait_node_pool(wait_node_pool&&)            = delete;
   wait_node_pool& operator=(wait_node_pool&&) = delete;

   void reset_all() noexcept
   {
      for (auto& node : nodes) {
         node.reset();
      }
      free_mask = ALL_NODES_FREE;
   }

   [[nodiscard]] constexpr std::size_t capacity()   const noexcept { return nodes.size(); }
   [[nodiscard]] constexpr std::size_t free_count() const noexcept { return std::popcount(free_mask); }
   [[nodiscard]] constexpr        bool empty()      const noexcept { return free_mask == 0; }

   /**
    * Allocate a node, initialize its identity fields, and return it.
    * Returns nullptr if pool exhausted.
    */
   wait_node* alloc(wait_group& g, waitable& w, uint8_t index) noexcept
   {
      CORTOS_ASSERT(index != wait_node::invalid_index);

      if (free_mask == 0) return nullptr;

      uint32_t bit = std::countr_zero(free_mask);
      free_mask &= ~(1u << bit);

      auto& node = nodes[bit];

      // Node should be inactive if the mask said it was free.
      // If not, we have a bug in free()/mask management.
      CORTOS_ASSERT(!node.active);

      node.reset();
      node.active   = true;
      node.active_group    = &g;
      node.active_waitable = &w;
      node.index    = index;

      return &node;
   }

   /**
    * Free a node back to the pool.
    * Caller is responsible for unlinking it from any waitable queue first.
    */
   void free(wait_node& node) noexcept
   {
      std::ptrdiff_t index = &node - nodes.data();

      CORTOS_ASSERT1(0 <= index && static_cast<std::size_t>(index) < N, index); // Node not from this pool

      auto& n = nodes[static_cast<std::size_t>(index)];
      CORTOS_ASSERT(n.active); // If error: You are freeing an inactive node.

      CORTOS_ASSERT_OP(n.slot, ==, static_cast<uint8_t>(index));
      n.reset();
      free_mask |= (1u << static_cast<uint32_t>(index));
   }

   /**
    * Iterate active nodes (optionally filtered by group).
    * Useful for teardown: remove all nodes for a completed wait.
    */
   void for_each_active(function<void(wait_node&), 32, heap_policy::no_heap>&& fn, wait_group* only_group = nullptr) noexcept
   {
      for (std::size_t i = 0; i < N; ++i) {
         auto& node = nodes[i];
         if (!node.active) continue;
         if (only_group && node.active_group != only_group) continue;
         fn(node);
      }
   }

   /**
    * Return pointer to node by slot index (even if inactive).
    * Primarily for debugging / assertions.
    */
   [[nodiscard]] wait_node* at(std::size_t slot) noexcept
   {
      if (slot >= N) return nullptr;
      return &nodes[slot];
   }
};

/**
 * @brief Coordination state for a multi-wait (wait_for_any) operation
 *
 * A wait_group tracks the outcome of a wait_for_any() call across multiple
 * Waitables. It records which waitable won the race and whether the waking
 * thread acquired a resource.
 *
 * Exactly one signal may win the group. All other signals are ignored once
 * the group is marked complete.
 */
struct wait_group
{
   std::atomic<bool> done{false};
   int       winner_index{-1};
   bool          acquired{false}; // Flag for 'resource waitables' (e.g. mutex)
   uint8_t expected_count{0};     // Purely for debug/sanity checking

   void begin(uint8_t n) noexcept
   {
      done.store(false, std::memory_order_relaxed);
      winner_index   = -1;
      acquired       = false;
      expected_count = n;
   }

   // Called by signal path. Returns true if *this* call won.
   bool try_win(int index, bool acquired_) noexcept
   {
      bool expected = false;
      if (!done.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
         return false;
      }
      winner_index = index;
      acquired     = acquired_;
      return true;
   }
};

} // namespace cortos

#endif // CORTOS_WAIT_SUBSYSTEM_HPP
