#include <cyros/kernel/kernel.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>
#include <cyros/config/config.hpp>

#include "scheduler.hpp"
#include "thread_action.hpp"
#include "threading_subsystem.hpp"
#include "waitable_utilities.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <ranges>

namespace cyros
{

namespace
{

struct kernel_state
{
   spinlock lock;
   std::array<scheduler, config::cores> schedulers;
   std::atomic<bool> initialised{false};
   std::atomic<bool> running{false};
   std::atomic<std::uint32_t> active_threads{0};
   std::atomic<std::uint32_t> thread_id_generator{1};

   // Compile-time construct the scheduler list with incrementing core id's.
   template<std::size_t... is>
   constexpr explicit kernel_state(std::index_sequence<is...>) noexcept : schedulers{ scheduler{is}... } {}
   constexpr kernel_state() noexcept : kernel_state(std::make_index_sequence<config::cores>{}) {}
} constinit k;

// Use this to examine how much memory the kernel uses.
[[maybe_unused]] constexpr auto kernel_memory = sizeof(k);

[[nodiscard, gnu::pure]]
scheduler& scheduler_for_core(std::uint32_t core_id)
{
   return k.schedulers.at(core_id);
}

[[nodiscard, gnu::pure]]
scheduler& scheduler_for_this_core()
{
   return k.schedulers[cyros_port_get_core_id()];
}

/**
 * @brief Load-balancing primer for registering threads
 */
void pin_thread_to_core(thread_control_block& tcb) noexcept
{
   std::uint32_t best_core = std::numeric_limits<std::uint32_t>::max();
   std::uint32_t best_load = std::numeric_limits<std::uint32_t>::max();

   bool found = false;
   for (std::uint32_t core = 0; core < config::cores; ++core) {
      if (!tcb.affinity.allows(core)) continue;

      auto const load = scheduler_for_core(core).pinned_thread_count();
      if (load < best_load) {
         best_load = load;
         best_core = core;
         found = true;
      }
   }
   CYROS_ASSERT(found); // thread affinity mask allows no cores

   scheduler_for_core(best_core).pin_thread(tcb);
}

/**
 * @brief Post a priority-recompute doorbell to a thread's owning core.
 *
 * Value-free by design: the request tells the owning core to re-derive the
 * thread's effective priority from current truth, it never carries a
 * priority. Posting to the calling core itself is legal and used as the
 * overflow path of a deep local boost chain.
 */
void post_priority_recompute(thread_control_block& tcb, thread::id const expected_id)
{
   bool const posted = scheduler_for_core(tcb.pinned_core).post_to_inbox({
      .type = cross_core_request::recompute_priority,
      .tcb  = &tcb,
      .expected_thread_id = expected_id,
   });
   CYROS_ASSERT(posted); // Inbox full
}

// Registered as isr handler for preemptive scheduling
void reschedule_this_core()
{
   auto& scheduler = scheduler_for_this_core();

   scheduler.reschedule();
}

void core_entry()
{
   auto& scheduler = scheduler_for_this_core();

   scheduler.start();
}

} // namespace

namespace thread_action
{

thread_control_block& get_current_thread_on_this_core()
{
   auto& scheduler = scheduler_for_this_core();

   return scheduler.get_current_thread();
}

void register_thread(thread_control_block& tcb)
{
   CYROS_ASSERT_OP(tcb.state, ==, thread_state::created);
   CYROS_ASSERT(!tcb.is_enqueued());

   tcb.id = k.thread_id_generator.fetch_add(1, std::memory_order_relaxed);
   k.active_threads.fetch_add(1, std::memory_order_seq_cst);

   {
      spinlock_guard guard(k.lock);
      pin_thread_to_core(tcb);
   }
   auto& scheduler = scheduler_for_core(tcb.pinned_core);

   // If cores are not running yet, enqueue directly (even for remote cores)
   if (!k.running.load(std::memory_order_acquire)) {
      (void)scheduler.set_thread_ready(tcb);
      return;
   }

   if (ready_thread(tcb) == schedule_hint::warranted) {
      // Weak request: Registering the thread is not itself blocking, it is
      // only flagging that a higher-priority thread became ready.
      cyros_port_pend_reschedule();
   }
}

schedule_hint ready_thread(thread_control_block& tcb)
{
   // Fast path: if the thread is already terminated. Can happen with stale remote-ready-requests
   // A thread that terminates AFTER this check is handled by the scheduler-level guard when the request is drained.
   if (tcb.state == thread_state::terminated) {
      return schedule_hint::unwarranted;
   }

   auto& scheduler = scheduler_for_core(tcb.pinned_core);

   auto const this_core = cyros_port_get_core_id();
   if (this_core != tcb.pinned_core) {
      bool const posted = scheduler.post_to_inbox({
         .type = cross_core_request::set_thread_ready,
         .tcb = &tcb,
      });
      CYROS_ASSERT(posted);
      return schedule_hint::unwarranted;
   }

   return scheduler.set_thread_ready(tcb);
}

// TODO: Reduce cognitive complexity
void recompute_thread_priority(thread_control_block& tcb, thread::id const expected_id)
{
   struct priority_chain_target
   {
      thread_control_block* tcb;
      std::uint32_t expected_id;
   };
   static constexpr std::size_t priority_chain_depth = 8;
   std::array<priority_chain_target, priority_chain_depth> priority_chain{};
   std::size_t pending = 0;
   priority_chain[pending++] = {
      .tcb = &tcb,
      .expected_id = expected_id,
   };

   auto const this_core_id = cyros_port_get_core_id();
   bool reschedule_warranted = false;

   while (pending > 0) {
      auto const target = priority_chain[--pending];

      if (target.tcb->id != target.expected_id) continue;          // TCB recycled, drop
      if (target.tcb->state == thread_state::terminated) continue; // Stale, drop

      if (target.tcb->pinned_core != this_core_id) {
         post_priority_recompute(*target.tcb, target.expected_id);
         continue;
      }

      // pi_lock holds the held list and active_waits stable, and (as a
      // spinlock) holds off preemption for the matrix surgery inside
      // reprioritize_thread.
      spinlock_guard pi_guard(target.tcb->pi_lock);

      std::uint8_t new_effective = target.tcb->base_priority;
      for (pi_waitable* held = target.tcb->held_head; held != nullptr; held = held->next_held) {
         std::uint8_t const top = held->queue.top();
         new_effective = std::min(top, new_effective);
      }

      if (new_effective == target.tcb->priority()) continue;

      auto const hint = scheduler_for_this_core().reprioritize_thread(*target.tcb, new_effective);
      if (hint == schedule_hint::warranted) {
         reschedule_warranted = true;
      }

      // A blocked thread's new priority must be reflected in every queue it
      // is parked on, the ordered position arm() gave it is now stale. Where
      // that changes a queue's BEST waiter, the holder of that queue's
      // resource inherits differently, so it is queued for its own recompute,
      // processed after this target's pi_lock is released.
      if (target.tcb->state == thread_state::blocked && target.tcb->active_waits != nullptr) {
         for (auto& node : *target.tcb->active_waits) {
            if (node.source == nullptr) continue;
            if (!node.source->queue.reslot(node)) continue;

            thread::id chase_id = 0;
            auto* next = node.source->donation_target(chase_id);
            if (next == nullptr) continue;

            if (pending < priority_chain.size()) {
               priority_chain[pending++] = {
                  .tcb = next,
                  .expected_id=chase_id,
               };
            } else {
               // Deep local chain: continue by doorbell instead of growing
               // the walk. Self-posting is legal and drains promptly.
               post_priority_recompute(*next, chase_id);
            }
         }
      }
   }

   if (reschedule_warranted) {
      cyros_port_pend_reschedule();
   }
}

} // namespace thread_action

void thread_launcher(void* tcb_ptr)
{
   auto* tcb = static_cast<thread_control_block*>(tcb_ptr);

   cyros_port_set_tls_pointer(tcb); // for now point tls to the tcb, but in future, tls sits just after tcb

   tcb->entry();

   auto& scheduler = scheduler_for_this_core();

   // Idle threads are outside the teardown bookkeeping and return normally
   if (tcb->id == scheduler::idle_thread_id) {
      scheduler.set_thread_terminated(*tcb);
      return;
   }

   // Teardown of the user thread must not be interrupted.
   // Bookkeeping and port exit mechanics must be made
   // atomically. It is up to the port exit routine to
   // return us to the reschedule routine.
   auto token = cyros_port_preempt_disable();

   scheduler.set_thread_terminated(*tcb);
   CYROS_ASSERT(k.active_threads.fetch_sub(1, std::memory_order_seq_cst) != 0);

   cyros_port_thread_exit(token);
}

void idle_task()
{
   // We may have received a message whilst bootstrapping (if idle_thread was first picked)
   if (scheduler_for_this_core().inbox_pending()) {
      this_thread::yield();
   }

   while (k.running.load(std::memory_order::relaxed)) {
      cyros_port_idle();

      this_thread::yield();
   }
}

namespace kernel
{

void initialise() noexcept
{
   CYROS_ASSERT(!k.initialised); // Cannot invoke kernel::initialise twice (without finalising down in between)

   cyros_port_init(reschedule_this_core);

   for (auto& scheduler : k.schedulers) {
      scheduler.init_idle_thread();
   }
   k.initialised.store(true, std::memory_order_relaxed);
}

void start() noexcept
{
   CYROS_ASSERT(k.initialised); // kernel::initialise() must be called first
   CYROS_ASSERT(k.active_threads > 0); // Starting the kernel with no registered threads is not allowed

   k.running.store(true, std::memory_order_release);

   cyros_port_start_cores(config::cores, core_entry);
}

void finalise() noexcept
{
   CYROS_ASSERT(k.initialised);

   k.running.store(false, std::memory_order_relaxed);
   k.thread_id_generator.store(1, std::memory_order_relaxed);
   k.active_threads.store(0, std::memory_order_relaxed);;

   for (auto& scheduler : k.schedulers) {
      scheduler.reset();
   }
   k.initialised.store(false, std::memory_order_relaxed);
}

std::uint32_t core_count() noexcept
{
   return config::cores;
}

std::uint32_t active_threads() noexcept
{
   return k.active_threads.load(std::memory_order_relaxed);
}

} // namespace kernel

namespace this_core
{

[[nodiscard]] std::uint32_t id() noexcept
{
   return cyros_port_get_core_id();
}

void pend_reschedule() noexcept
{
   cyros_port_pend_reschedule();
}

preemption_token disable_preemption() noexcept
{
   return { .v = cyros_port_preempt_disable() };
}

void enable_preemption(preemption_token token) noexcept
{
   cyros_port_preempt_enable(token.v);
}

} // namespace this_core

namespace this_thread
{

[[nodiscard]] thread::id id()
{
   return scheduler_for_this_core().current_thread_id();
}

[[nodiscard]] thread::priority priority()
{
   return scheduler_for_this_core().current_thread_priority();
}

[[noreturn]] void thread_exit()
{
   // todo
   __builtin_unreachable();
}

void yield()
{
   // Strong request: an explicit yield deliberately gives up the CPU and
   // relies on the reschedule round-trip having completed on return.
   cyros_port_thread_yield();
}

[[nodiscard]] std::size_t wait_on_any(std::span<waitable_ref> waitables) noexcept
{
   CYROS_ASSERT(!waitables.empty());
   CYROS_ASSERT_OP(waitables.size(), <=, config::max_wait_nodes);

   auto& tcb = thread_action::get_current_thread_on_this_core();
   wait_node_vector nodes(waitables.size(), tcb);

   active_wait_registration const registration(tcb, &nodes);

   while (true) {
      tcb.disposition = thread_disposition::prepared;
      std::optional<std::size_t> chosen;

      {
         // All waitable wakes are serialised on the arm_guard.
         // If a wake fires BEFORE the arm_guard:
         // - Thread is not readied because we are not registered.
         waitable_arm_guard arm_guard(waitables, nodes);
         // If a wake fires AFTER the arm_guard:
         // - Thread is readied and we are no longer 'prepared' to block.

         // Lowest-index wins on ties
         for (std::size_t i = 0; waitable& waitable : waitables) {
            if (waitable.wait_condition(*tcb.public_thread_handle)) {
               tcb.disposition = thread_disposition::none;
               chosen = i;
               break;
            }
            ++i;
         }

         if (!chosen) {
            auto token = cyros_port_preempt_disable();
            if (tcb.disposition == thread_disposition::prepared) {
               // Condition unsatisfied AND no wake came after arming, block until woken
               tcb.disposition = thread_disposition::committed;
               cyros_port_pend_reschedule(); // Delayed until preempt_enable()
            }
            cyros_port_preempt_enable(token);
         }
      } // arm_guard: disarm all (and de-boost any holder whose top we lowered)

      if (chosen) {
         // The sweep must run here, AFTER the disarm above: while any node of
         // ours was still armed, a racing release could commit a transfer to
         // us. With every node off every queue no further transfer can choose
         // us, so the set of in-flight assignments is frozen and the ones we
         // are not returning are handed back. The contract this enforces: on
         // return the caller owns exactly waitables[chosen], plus whatever it
         // already owned before the call.
         for (std::size_t i = 0; waitable& waitable : waitables) {
            if (i != chosen) {
               waitable.renounce_if_assigned(tcb.id);
            }
            ++i;
         }
         return *chosen;
      }
   }
}

}  // namespace this_thread

}  // namespace cyros