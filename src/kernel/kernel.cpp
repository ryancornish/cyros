// todo split up the implementation!

#include <cortos/kernel/kernel.hpp>
#include <cortos/port/port.h>
#include <cortos/port/port_traits.h>
#include <cortos/config/config.hpp>

#include "scheduler.hpp"
#include "threading_subsystem.hpp"
#include "wait_subsystem.hpp"
#include "waitable_utilities.hpp"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <ranges>

// invariants list:
// only core x mutates scheduler[x].ready_matrix, current_thread, blocked lists, etc.
// cross-core ops must go through scheduler::post_to_inbox() + cortos_port_send_reschedule_ipi(core).
// spinlocks disable preemption for the duration of the lock (via the port's
// preemption control); they do not mask interrupts.


namespace cortos
{

static void reschedule_this_core();
static void core_entry();

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

   [[nodiscard]] bool  is_running() const noexcept
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
      return scheduler_for_core(cortos_port_get_core_id());
   }

   [[nodiscard]] scheduler& scheduler_for_time_core() noexcept
   {
      return scheduler_for_core(config::time_core_id);
   }

   [[nodiscard]] std::uint32_t generate_thread_id()
   {
      return thread_id_generator.fetch_add(1, std::memory_order_relaxed);
   }

   void initialise() noexcept
   {
      CORTOS_ASSERT(!initialised); // cannot invoke kernel::initialise twice (without finalising down in between)

      cortos_port_init(reschedule_this_core);

      for (auto& scheduler : schedulers) {
         scheduler.init_idle_thread();
      }
      initialised = true;
   }

   void start() noexcept
   {
      CORTOS_ASSERT(initialised); // kernel::initialise() must be called first
      CORTOS_ASSERT(active_threads > 0); // starting the kernel with no registered threads is not allowed

      running.store(true, std::memory_order_release);

      cortos_port_start_cores(config::cores, core_entry);
   }

   void finalise() noexcept
   {
      CORTOS_ASSERT(initialised.load(std::memory_order_relaxed)); // in theory there shouldn't be anything to reset yet... so why was this invoked? smells buggy

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
      CORTOS_ASSERT(found); // thread affinity mask allows no cores

      schedulers.at(best_core).pin_thread(tcb);
   }

   void register_thread(thread_control_block& tcb) noexcept
   {
      active_threads.fetch_add(1, std::memory_order_seq_cst);

      {
         spinlock_guard guard(lock);
         pin_thread_to_core(tcb);
      }

      if (set_thread_ready(tcb) == schedule_hint::warranted) {
         // Weak request: the registering thread is not itself blocking, it is
         // only flagging that a higher-priority thread became ready.
         cortos_port_pend_reschedule();
      }
   }

   void unregister_thread() noexcept
   {
      CORTOS_ASSERT(active_threads.load(std::memory_order_relaxed) != 0);
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
   schedule_hint set_thread_ready(thread_control_block& tcb) noexcept
   {
      CORTOS_ASSERT_OP(tcb.state, !=, thread_control_block::thread_state::terminated);

      auto& scheduler = scheduler_for_core(tcb.pinned_core);

      // if cores are not running yet, enqueue directly (even for remote cores)
      if (!running.load(std::memory_order_acquire)) {
         scheduler.set_thread_ready(tcb);
         return schedule_hint::unwarranted;
      }

      auto this_core = cortos_port_get_core_id();
      if (this_core == tcb.pinned_core) {
         scheduler.set_thread_ready(tcb);

         if (tcb.is_higher_priority_than(scheduler.current_thread_priority())) {
            return schedule_hint::warranted;
         }
      } else {
         bool posted = scheduler.post_to_inbox({
            .type = cross_core_request::set_thread_ready,
            .tcb = &tcb
         });
         CORTOS_ASSERT(posted);
      }
      return schedule_hint::unwarranted;
   }
};
static constinit kernel_state kernel_instance;

// use this to examine how much memory the co_rtos kernel uses.
[[maybe_unused]] constexpr auto static_sizeof_kernel = sizeof(kernel_instance);

/**** kernel-global dependants ****/

// registered as isr handler for preemptive scheduling
static void reschedule_this_core()
{
   auto& scheduler = kernel_instance.scheduler_for_this_core();

   scheduler.reschedule();
}

static void core_entry()
{
   auto& scheduler = kernel_instance.scheduler_for_this_core();

   scheduler.start();
}

void thread_launcher(void* tcb_ptr)
{
   auto* tcb = static_cast<thread_control_block*>(tcb_ptr);

   cortos_port_set_tls_pointer(tcb); // for now point tls to the tcb, but in future, tls sits just after tcb

   tcb->entry();

   tcb->state = thread_control_block::thread_state::terminated;

   // signal joiners
   tcb->termination.terminate();

   if (tcb->id == scheduler::idle_thread_id) return; // idle threads are not apart of the same bookkeeping

   kernel_instance.unregister_thread();
   cortos_port_thread_exit();
}

void idle_task()
{
   while (kernel_instance.is_running()) {
      cortos_port_idle();
      // Weak request: idle never blocks, it only asks the scheduler to
      // re-check whether any thread has become ready.
      cortos_port_pend_reschedule();
   }
}

schedule_hint wait_node::wake_thread(bool acquired) const noexcept
{
   CORTOS_ASSERT(tcb != nullptr);
   CORTOS_ASSERT(active_group != nullptr);
   CORTOS_ASSERT(active_waitable != nullptr);
   CORTOS_ASSERT(index != invalid_index);
   CORTOS_ASSERT_OP(tcb->state, ==, thread_control_block::thread_state::blocked);

   if (!active_group->try_win(static_cast<int>(index), acquired)) {
      return schedule_hint::unwarranted; // lost
   }

   // winner: remove all nodes in this group (including this one)
   tcb->teardown_wait_group(*active_group);

   // thread is ready to be enqueued on its pinned core
   return kernel_instance.set_thread_ready(*tcb);
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
      std::move(entry)
   );

   kernel_instance.register_thread(*tcb);
}

thread::~thread()
{
   if (tcb == nullptr) return; // thread handle has been moved from, or is otherwise empty
   CORTOS_ASSERT(tcb->state == thread_control_block::thread_state::terminated);
}

thread::thread(thread&& other) noexcept : tcb(other.tcb)
{
   other.tcb = nullptr;
}

thread& thread::operator=(thread&& other) noexcept
{
   tcb = other.tcb;
   other.tcb = nullptr;
   return *this;
}

void thread::join() noexcept
{
   CORTOS_ASSERT(tcb != nullptr);

   kernel::wait_until([&]{
      return tcb->termination.has_terminated();
   }, tcb->termination);
}


void spinlock::lock()
{
   // A spinlock disables preemption (so the holder cannot be switched out
   // mid-section) but does NOT mask interrupts. Preemption control is owned
   // by the port - see port.h "Preemption Control".
   cortos_port_preempt_disable();
   while (flag.test_and_set(std::memory_order_acquire)) {
      // busy-wait with cpu yield hint
      cortos_port_cpu_relax();
   }
}

void spinlock::unlock()
{
   flag.clear(std::memory_order_release);
   // Re-enabling preemption is a contract safe point: if a reschedule was
   // pended while the lock was held, the port resolves it here.
   cortos_port_preempt_enable();
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

waitable::result wait_for_any(std::span<waitable* const> waitables)
{
   CORTOS_ASSERT(!waitables.empty());
   CORTOS_ASSERT_OP(waitables.size(), <=, config::max_wait_nodes);

   auto& scheduler = kernel_instance.scheduler_for_this_core();
   {
      waitable_group_lock lock_group(waitables);

      scheduler.prepare_block_current_thread(waitables);
   }
   scheduler.notify_block_current_thread(waitables);

   auto result = scheduler.commence_block_current_thread();

   return result;
}

waitable::result wait_until(waitable::predicate predicate, std::span<waitable* const> waitables)
{
   CORTOS_ASSERT(!waitables.empty());
   CORTOS_ASSERT_OP(waitables.size(), <=, config::max_wait_nodes);
   for (auto* w : waitables) CORTOS_ASSERT(w != nullptr);

   waitable::result result{.index = -1, .acquired = false};

   // fast-path: avoid locks
   if (predicate()) {
      return result;
   }

   auto& scheduler = kernel_instance.scheduler_for_this_core();

   while (true) {
      {
         waitable_group_lock lock_group(waitables);

         if (predicate()) return result;

         scheduler.prepare_block_current_thread(waitables);
      }
      scheduler.notify_block_current_thread(waitables);

      result = scheduler.commence_block_current_thread();
   }
}

} // namespace kernel

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

[[nodiscard]] std::uint32_t core_id() noexcept
{
   return cortos_port_get_core_id();
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
   cortos_port_thread_yield();
}

}  // namespace this_thread

}  // namespace cortos