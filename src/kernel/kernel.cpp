#include <cyros/kernel/kernel.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>
#include <cyros/config/config.hpp>

#include "scheduler.hpp"
#include "threading_subsystem.hpp"
#include "waitable_utilities.hpp"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace cyros
{

namespace
{

void reschedule_this_core();
void core_entry();

class kernel_state
{
private:
   spinlock lock;
   std::array<scheduler, config::cores> schedulers;
   std::atomic<bool>    initialised{false};
   std::atomic<bool>        running{false};
   std::atomic<uint32_t> active_threads{0};
   std::atomic<uint32_t> thread_id_generator{1};

   template<std::size_t... is>
   constexpr explicit kernel_state(std::index_sequence<is...>) noexcept : schedulers{ scheduler{is}... } {}

public:
   // compile-time construct the scheduler list with incrementing core id's.
   constexpr kernel_state() noexcept : kernel_state(std::make_index_sequence<config::cores>{}) {}

   ~kernel_state() = default;
   kernel_state(kernel_state&&) = delete;
   kernel_state(kernel_state const&) = delete;
   kernel_state& operator=(kernel_state&&) = delete;
   kernel_state& operator=(kernel_state const&) = delete;

   [[nodiscard]] bool is_running() const noexcept
   {
      return running.load(std::memory_order_acquire);
   }

   [[nodiscard]] std::uint32_t get_active_threads() const noexcept
   {
      return active_threads.load(std::memory_order_seq_cst);
   }

   [[nodiscard]] scheduler& scheduler_for_core(std::uint32_t core_id) noexcept
   {
      return schedulers.at(core_id);
   }

   [[nodiscard]] scheduler& scheduler_for_this_core() noexcept
   {
      return scheduler_for_core(cyros_port_get_core_id());
   }

   [[nodiscard]] std::uint32_t generate_thread_id()
   {
      return thread_id_generator.fetch_add(1, std::memory_order_relaxed);
   }

   void initialise() noexcept
   {
      CYROS_ASSERT(!initialised); // cannot invoke kernel::initialise twice (without finalising down in between)

      cyros_port_init(reschedule_this_core);

      for (auto& scheduler : schedulers) {
         scheduler.init_idle_thread();
      }
      initialised = true;
   }

   void start() noexcept
   {
      CYROS_ASSERT(initialised); // kernel::initialise() must be called first
      CYROS_ASSERT(active_threads > 0); // starting the kernel with no registered threads is not allowed

      running.store(true, std::memory_order_release);

      cyros_port_start_cores(config::cores, core_entry);
   }

   void finalise() noexcept
   {
      CYROS_ASSERT(initialised.load(std::memory_order_relaxed)); // in theory there shouldn't be anything to reset yet... so why was this invoked? smells buggy

      running.store(false, std::memory_order_relaxed);
      thread_id_generator.store(1, std::memory_order_relaxed);
      active_threads.store(0, std::memory_order_relaxed);;
      initialised.store(false, std::memory_order_relaxed);

      for (auto& scheduler : schedulers) {
         scheduler.reset();
      }
   }

   // load-balancing pinner
   void pin_thread_to_core(thread_control_block& tcb) noexcept
   {
      uint32_t best_core = std::numeric_limits<uint32_t>::max();
      uint32_t best_load = std::numeric_limits<uint32_t>::max();

      bool found = false;
      for (uint32_t core = 0; core < schedulers.size(); ++core) {
         if (!tcb.affinity.allows(core)) continue;

         uint32_t load = schedulers[core].pinned_thread_count();
         if (load < best_load) {
            best_load = load;
            best_core = core;
            found = true;
         }
      }
      CYROS_ASSERT(found); // thread affinity mask allows no cores

      schedulers.at(best_core).pin_thread(tcb);
   }

   void register_thread(thread_control_block& tcb) noexcept
   {
      CYROS_ASSERT_OP(tcb.state, ==, thread_state::created);
      CYROS_ASSERT(!tcb.is_enqueued());

      active_threads.fetch_add(1, std::memory_order_seq_cst);

      {
         spinlock_guard guard(lock);
         pin_thread_to_core(tcb);
      }
      auto& scheduler = scheduler_for_core(tcb.pinned_core);

      // If cores are not running yet, enqueue directly (even for remote cores)
      if (!running.load(std::memory_order_acquire)) {
         (void)scheduler.set_thread_ready(tcb);
         return;
      }

      if (request_thread_ready(tcb) == schedule_hint::warranted) {
         // Weak request: Registering the thread is not itself blocking, it is
         // only flagging that a higher-priority thread became ready.
         cyros_port_pend_reschedule();
      }
   }

   void unregister_thread() noexcept
   {
      CYROS_ASSERT(active_threads.load(std::memory_order_relaxed) != 0);

      active_threads.fetch_sub(1, std::memory_order_seq_cst);
   }

   /**
   * @brief make a thread runnable on its pinned core (smp-safe).
   *
   * transitions @p tcb to the ready state and enqueues it on the ready queue
   * of its pinned core. if the thread belongs to another core, a cross-core
   * request is posted so that the owning scheduler performs the enqueue.
   *
   * @param tcb thread control block of the thread to make runnable.
   * @return ready_action::reschedule if the thread was queued on the current
   *         core and has higher priority than the running thread, indicating
   *         the caller should request a local reschedule.
   */
   schedule_hint request_thread_ready(thread_control_block& tcb) noexcept
   {
      // Fast path: if the thread is already terminated. Can happen with stale remote-ready-requests
      // A thread that terminates AFTER this check is handled by the scheduler-level guard when the request is drained.
      if (tcb.state == thread_state::terminated) {
         return schedule_hint::unwarranted;
      }

      auto& scheduler = scheduler_for_core(tcb.pinned_core);

      auto this_core = cyros_port_get_core_id();
      if (this_core != tcb.pinned_core) {
         bool posted = scheduler.post_to_inbox({
            .type = cross_core_request::set_thread_ready,
            .tcb = &tcb,
         });
         CYROS_ASSERT(posted);
         return schedule_hint::unwarranted;
      }

      return scheduler.set_thread_ready(tcb);
   }
};
constinit kernel_state kernel_instance;

// use this to examine how much memory the co_rtos kernel uses.
[[maybe_unused]] constexpr auto static_sizeof_kernel = sizeof(kernel_instance);

/**** kernel-global dependants ****/

// registered as isr handler for preemptive scheduling
void reschedule_this_core()
{
   auto& scheduler = kernel_instance.scheduler_for_this_core();

   scheduler.reschedule();
}

void core_entry()
{
   auto& scheduler = kernel_instance.scheduler_for_this_core();

   scheduler.start();
}

} // namespace

void thread_launcher(void* tcb_ptr)
{
   auto* tcb = static_cast<thread_control_block*>(tcb_ptr);

   cyros_port_set_tls_pointer(tcb); // for now point tls to the tcb, but in future, tls sits just after tcb

   tcb->entry();

   auto& scheduler = kernel_instance.scheduler_for_this_core();

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
   kernel_instance.unregister_thread();

   cyros_port_thread_exit(token);
}

void idle_task()
{
   // We may have received a message whilst bootstrapping (if idle_thread was first picked)
   if (kernel_instance.scheduler_for_this_core().inbox_pending()) {
      this_thread::yield();
   }

   while (kernel_instance.is_running()) {
      cyros_port_idle();
      // Weak request: idle never blocks, it only asks the scheduler to
      // re-check whether any thread has become ready.
      this_core::pend_reschedule();
   }
}

schedule_hint kernel_request_thread_ready(thread_control_block& tcb)
{
   return kernel_instance.request_thread_ready(tcb);
}

/**** public ****/

thread::thread(entry_fn&& entry, std::span<std::byte> stack, priority priority, core_affinity affinity)
{
   stack_layout slayout(stack, 0);
   tcb = ::new (slayout.tcb) thread_control_block(
      kernel_instance.generate_thread_id(),
      priority,
      affinity,
      slayout.user_stack,
      std::move(entry),
      this
   );

   kernel_instance.register_thread(*tcb);
}


namespace kernel
{

void initialise()
{
   kernel_instance.initialise();
}

void start()
{
   kernel_instance.start();
}

void finalise()
{
   kernel_instance.finalise();
}

std::uint32_t core_count() noexcept
{
   return config::cores;
}

std::uint32_t active_threads() noexcept
{
   return kernel_instance.get_active_threads();
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
   return kernel_instance.scheduler_for_this_core().current_thread_id();
}

[[nodiscard]] thread::priority priority()
{
   return kernel_instance.scheduler_for_this_core().current_thread_priority();
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

   auto& scheduler = kernel_instance.scheduler_for_this_core();
   auto* tcb = scheduler.current_thread_reference();
   wait_node_vector nodes(waitables.size(), tcb);

   while (true) {
      tcb->disposition = thread_disposition::prepared;

      // All waitable wakes are serialised on the arm_guard.
      // If a wake fires BEFORE the arm_guard:
      // - Thread is not readied because we are not registered.
      waitable_arm_guard arm_guard(waitables, nodes);
      // If a wake fires AFTER the arm_guard:
      // - Thread is readied and we are no longer 'prepared' to block.

      // Lowest-index wins on ties
      for (std::size_t i = 0; waitable& waitable : waitables) {
         if (waitable.wait_condition(*tcb->public_thread_handle)) {
            tcb->disposition = thread_disposition::none;
            return i;
         }
         ++i;
      }

      auto token = cyros_port_preempt_disable();
      if (tcb->disposition == thread_disposition::prepared) {
         // Condition unsatisfied AND no wake came after arming, block until woken
         tcb->disposition = thread_disposition::committed;
         cyros_port_pend_reschedule(); // Delayed until preempt_enable()
      }
      cyros_port_preempt_enable(token);
   }
}

}  // namespace this_thread

}  // namespace cyros