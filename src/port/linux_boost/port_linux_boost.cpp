/**
 * @file port_linux_boost.cpp
 * @brief Linux simulation port using Boost.Context
 *
 * This port uses Boost.Context for fast cooperative context switching.
 * It simulates embedded behavior (stack-based context switching) while
 * running on Linux for development and testing.
 *
 * SMP support: Each pthread represents a "core". Use cyros_port_get_core_id()
 * to determine which simulated core is running.
 *
 * Reschedule model
 * ----------------
 * This port implements the two-operation reschedule contract from port.h:
 *
 *   cyros_port_thread_yield()    - strong guarantee, synchronous. Resumes the
 *                                   scheduler fiber immediately. Caller must be
 *                                   in thread context at baseline priority.
 *
 *   cyros_port_pend_reschedule() - weak guarantee, deferred-safe. If invoked
 *                                   at baseline priority it resolves now (same
 *                                   fiber resume). If invoked while the core is
 *                                   kernel-masked it sets a per-core
 *                                   "reschedule pending" flag instead.
 *
 * Baseline priority and the two depth counters
 * --------------------------------------------
 * "Kernel-masked" on this port means EITHER counter is non-zero:
 *   - interrupt_disable_depth : interrupt masking (Critical Sections).
 *   - preempt_disable_depth   : preemption disabling (Preemption Control).
 * A context switch may occur only at "baseline priority": inside a thread
 * fiber with BOTH counters at zero.
 *
 * Resolving the pending flag
 * --------------------------
 * The flag is drained at whichever safe point is reached last - that is,
 * whenever a counter returns to zero and the OTHER counter is already zero:
 *   - cyros_port_irq_restore()  reaching interrupt depth 0, and
 *   - cyros_port_preempt_enable() reaching preempt depth 0.
 * Both paths funnel through resolve_pending_reschedule_if_baseline(), which
 * only acts when the full baseline condition holds.
 */

#include <cyros/port/port.h>

#include <boost/context/fiber.hpp>
#include <boost/context/preallocated.hpp>
#include <boost/context/stack_context.hpp>

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <pthread.h>
#include <string>

/* ============================================================================
 * Port Context Structure
 * ========================================================================= */

struct cyros_port_context
{
   boost::context::fiber thread;  // Thread fiber (owned by scheduler when idle)
   void*                 stack_top;
   size_t                stack_size;
   cyros_port_entry_t   entry;
   void*                 arg;
};

// Verify that port_traits.h constants are correct
static_assert(sizeof(cyros_port_context) == CYROS_PORT_CONTEXT_SIZE,
              "CYROS_PORT_CONTEXT_SIZE mismatch - adjust in port_traits.h");
static_assert(alignof(cyros_port_context) == CYROS_PORT_CONTEXT_ALIGN,
              "CYROS_PORT_CONTEXT_ALIGN mismatch - adjust in port_traits.h");
static_assert((CYROS_PORT_STACK_ALIGN & (CYROS_PORT_STACK_ALIGN - 1)) == 0,
              "CYROS_PORT_STACK_ALIGN must be a power of two");
static_assert(CYROS_PORT_SCHEDULING_TYPE == 2);
static_assert(CYROS_PORT_ENVIRONMENT == 2);

/* ============================================================================
 * Port MultiCore Structure
 * ========================================================================= */

/**
 * For simulating the SMP schedulers on top of linux, we
 * spawn a new pthread (AKA OS-thread) for each configured core - (Except core0 because that is the initial thread).
 * Each OS-thread encapsulates a scheduler fiber that is ther "outer-context" for each user-thread pinned to a core.
 * When the user-thread "pends reschedule", the context is switched to this outer fiber so that scheduling can happen
 * on a separate stack and context to the threads.
 */
struct cpu_core
{
   pthread_t pthread{}; // @note This is null/unused for core0
   uint32_t  core_id{}; // Index from 0... num cores
   cyros_port_core_entry_t entry{};
   boost::context::fiber scheduler_fiber;

   /**
    * @brief Primitives to signal/communicate with other cores
    */
   struct core_poke
   {
      pthread_mutex_t mutex{};
      pthread_cond_t  cond_var{};
      std::atomic<bool> pending{false}; // Can be set by any core
      core_poke()  { pthread_mutex_init(&mutex, nullptr); pthread_cond_init(&cond_var, nullptr); }
      ~core_poke() { pthread_cond_destroy(&cond_var); pthread_mutex_destroy(&mutex); }
   } core_poke;

   void start_scheduler();
};

/**
 * @brief Dynamically-sized array/container wrapper for cpu_core's
 *
 * Using a std::vector _would_ be preferable, but impossible as a cpu_core is non-copyable
 */
class cpu_core_array
{
public:
   using iterator = cpu_core*;
   using const_iterator = const cpu_core*;

   constexpr cpu_core_array() = default;
   explicit cpu_core_array(size_t count, cyros_port_core_entry_t core_entry)
      : cores(std::make_unique<cpu_core[]>(count)), count(count)
   {
      for (uint32_t i = 0; i < count; ++i) {
         cores[i].core_id = i;
         cores[i].entry = core_entry;
      }
   }
   cpu_core_array(cpu_core_array&&) noexcept            = default;
   cpu_core_array& operator=(cpu_core_array&&) noexcept = default;
   cpu_core_array(cpu_core_array const&)            = delete;
   cpu_core_array& operator=(cpu_core_array const&) = delete;

   cpu_core&       operator[](size_t index)       { return cores[index]; }
   cpu_core const& operator[](size_t index) const { return cores[index]; }

   [[nodiscard]] size_t size() const { return count; }

   iterator begin() { return cores.get(); }
   iterator end()   { return cores.get() + count; }

   [[nodiscard]] const_iterator begin() const { return cores.get(); }
   [[nodiscard]] const_iterator end()   const { return cores.get() + count; }

private:
   std::unique_ptr<cpu_core[]> cores;
   size_t count{0};
};

/* ============================================================================
 * Global & Thread-Local State
 *
 * For SMP simulation, each OS-thread has its own state tracking using
 * thread_local. This is NOT stored in cyros_port_context to prevent
 * migration issues.
 * ========================================================================= */


struct global_state
{
   std::atomic<bool>        shutdown_requested{false};
   std::atomic<uint32_t>    active_contexts{0};
   cyros_port_reschedule_t reschedule_handler{nullptr};
   cpu_core_array cores;

   /// @brief Have any cores been started?
   [[nodiscard]] bool cores_launched() const { return cores.size() > 0; }

   void reset()
   {
      shutdown_requested.store(false);
      active_contexts.store(0);
      reschedule_handler = nullptr;
      cores = {};
   }
};
static constinit global_state global;

struct current_core_state
{
   cpu_core* core{nullptr};

   // Non-null when we are currently executing inside a thread fiber.
   cyros_port_context* current_context{nullptr};

   // The "caller" fiber for the currently running thread on *this OS-thread*.
   boost::context::fiber thread_caller;
   // The outermost fiber that returns us back out of the scheduler fibers when they are finished.
   boost::context::fiber os_caller;
   // Simulates pointing to fibers dedicated TLS block.
   void* tls_pointer{nullptr};

   // Interrupt-masking nesting depth (Critical Sections). Blocks the hardware.
   uint32_t interrupt_disable_depth{0};

   // Preemption-disable nesting depth (Preemption Control). Blocks the
   // scheduler from switching while non-zero; interrupts still flow.
   uint32_t preempt_disable_depth{0};

   // Set by cyros_port_pend_reschedule() when it cannot resolve immediately
   // (i.e. the core is kernel-masked). Drained at the next safe point: either
   // irq_restore() reaching interrupt depth 0, or preempt_enable() reaching
   // preempt depth 0 - whichever leaves BOTH counters at zero.
   bool reschedule_pending{false};
};
static thread_local constinit current_core_state current_core;

/* ============================================================================
 * Platform Initialization
 * ========================================================================= */

extern "C" void cyros_port_init(cyros_port_reschedule_t reschedule_handler)
{
   global.reschedule_handler = reschedule_handler;
}

/* ============================================================================
 * Reschedule resolution helper
 *
 * Shared by the two safe points (irq_restore depth 0, preempt_enable depth 0).
 * Declared early so the critical-section and preemption code below can use it.
 * ========================================================================= */

static void switch_to_scheduler_fiber();

/**
 * @brief Drain a deferred reschedule if the core is now at baseline priority.
 *
 * Baseline = inside a thread fiber, interrupt depth 0, preempt depth 0.
 * Called whenever either depth counter returns to zero.
 */
static void resolve_pending_reschedule_if_baseline()
{
   if (!current_core.reschedule_pending) return;

   // Baseline requires BOTH counters at zero...
   if (current_core.interrupt_disable_depth != 0) return;
   if (current_core.preempt_disable_depth   != 0) return;
   // ...and that we are inside a thread fiber to switch away from.
   if (!current_core.current_context) return;

   current_core.reschedule_pending = false;
   switch_to_scheduler_fiber();
}

/* ============================================================================
 * Critical Sections (Interrupt Control) - Simulated
 *
 * Tracks an interrupt-masking nesting depth. Blocks the hardware: while
 * non-zero, (simulated) interrupts cannot be delivered.
 * ========================================================================= */

extern "C" void cyros_port_disable_interrupts(void)
{
   current_core.interrupt_disable_depth++;
}

extern "C" void cyros_port_enable_interrupts(void)
{
   if (current_core.interrupt_disable_depth > 0) {
      current_core.interrupt_disable_depth--;
   }
}

extern "C" bool cyros_port_interrupts_enabled(void)
{
   return current_core.interrupt_disable_depth == 0;
}

extern "C" uint32_t cyros_port_irq_save(void)
{
   // Return previous enabled-state as 1/0 (simple)
   uint32_t prev_enabled = (current_core.interrupt_disable_depth == 0) ? 1u : 0u;
   current_core.interrupt_disable_depth++;
   return prev_enabled;
}

extern "C" void cyros_port_irq_restore(uint32_t state)
{
   (void)state;
   // Unwind one nesting level
   if (current_core.interrupt_disable_depth > 0) {
      current_core.interrupt_disable_depth--;
   }

   // Interrupt depth reaching 0 is one of the contract's safe points: if a
   // reschedule was pended while masked, resolve it (only fires if preemption
   // is also enabled).
   resolve_pending_reschedule_if_baseline();
}

/* ============================================================================
 * Preemption Control - Simulated
 *
 * Tracks a preemption-disable nesting depth. Blocks the scheduler: while
 * non-zero, no context switch is performed on this core. Interrupts are
 * unaffected.
 * ========================================================================= */

extern "C" void cyros_port_preempt_disable(void)
{
   current_core.preempt_disable_depth++;
}

extern "C" void cyros_port_preempt_enable(void)
{
   CYROS_ASSERT(current_core.preempt_disable_depth > 0); // unbalanced enable
   current_core.preempt_disable_depth--;

   // Preempt depth reaching 0 is one of the contract's safe points: if a
   // reschedule was pended while preemption was disabled, resolve it (only
   // fires if interrupts are also unmasked).
   resolve_pending_reschedule_if_baseline();
}

/* ============================================================================
 * Context Management & Switching
 * ========================================================================= */

// No-op stack allocator for preallocated memory
struct preallocated_stack_noop
{
   using traits_type = boost::context::stack_traits;
   boost::context::stack_context allocate(size_t) { std::abort(); }
   void deallocate(boost::context::stack_context&) noexcept {}
};

extern "C" void cyros_port_context_init(cyros_port_context_t* context,
                                         void* stack_base,
                                         size_t stack_size,
                                         cyros_port_entry_t entry,
                                         void* arg)
{
   global.active_contexts.fetch_add(1, std::memory_order_seq_cst);

   // Construct cyros_port_context_t in place
   ::new (context) cyros_port_context{
      .thread     = {},
      .stack_top  = static_cast<uint8_t*>(stack_base) + stack_size,
      .stack_size = stack_size,
      .entry      = entry,
      .arg        = arg,
   };

   // Build a fiber bound to the user-provided stack
   boost::context::stack_context boost_stack_context{
      .size = context->stack_size,
      .sp   = context->stack_top,
   };

   boost::context::preallocated boost_prealloc(
      boost_stack_context.sp,
      boost_stack_context.size,
      boost_stack_context
   );

   preallocated_stack_noop stack_allocator;

   context->thread = boost::context::fiber(
      std::allocator_arg,
      boost_prealloc,
      stack_allocator,
      [context](boost::context::fiber&& caller) mutable -> boost::context::fiber
      {
         current_core.thread_caller     = std::move(caller);
         current_core.current_context = context;

         try {
            context->entry(context->arg); // Enter user code
         } catch (boost::context::detail::forced_unwind const&) {
            current_core.current_context = nullptr;
            throw;
         }
         current_core.current_context = nullptr;

         return std::move(current_core.thread_caller);
      }
   );
}

extern "C" void cyros_port_context_destroy(cyros_port_context_t* context)
{
   // Verify fiber has completed
   CYROS_ASSERT(!context->thread); // Bug: destroying a live thread

   context->~cyros_port_context();
}

extern "C" void cyros_port_switch(cyros_port_context_t* /*from*/, cyros_port_context_t* to)
{
   CYROS_ASSERT(to->thread); // No context to switch to

   current_core.current_context = to;
   to->thread = std::move(to->thread).resume();
   current_core.current_context = nullptr;
}

extern "C" void cyros_port_start_first(cyros_port_context_t* first)
{
   // Nothing special to be done on the first switch
   cyros_port_switch(nullptr, first);
}

/* ============================================================================
 * Reschedule Requests
 *
 * See port.h "Reschedule Requests" for the full contract. On this cooperative
 * simulation port the synchronous switch is a resume of the scheduler fiber
 * (current_core.thread_caller).
 * ========================================================================= */

/**
 * @brief Perform the synchronous switch into the scheduler fiber.
 *
 * Precondition: we are inside a thread fiber (current_context != nullptr).
 * The scheduler fiber ('thread_caller') runs the kernel reschedule and, when
 * this thread is later selected again, control returns here.
 */
static void switch_to_scheduler_fiber()
{
   CYROS_ASSERT(current_core.current_context); // not inside a thread fiber
   CYROS_ASSERT(current_core.thread_caller);   // no scheduler fiber to resume
   current_core.thread_caller = std::move(current_core.thread_caller).resume();
}

extern "C" void cyros_port_thread_yield(void)
{
   // Strong-guarantee, synchronous. Contract precondition: thread context at
   // baseline priority. Assert it - this port can observe all conditions.
   CYROS_ASSERT(current_core.current_context);            // must be a thread
   CYROS_ASSERT(current_core.interrupt_disable_depth == 0); // interrupts unmasked
   CYROS_ASSERT(current_core.preempt_disable_depth   == 0); // preemption enabled

   switch_to_scheduler_fiber();
}

extern "C" void cyros_port_pend_reschedule(void)
{
   // Weak-guarantee, deferred-safe. Callable from any context.

   // Not inside a thread fiber (e.g. called from scheduler/idle context with
   // no current thread): nothing to switch away from. The cooperative pump in
   // start_scheduler() drives progress in that case.
   if (!current_core.current_context) return;

   const bool baseline = (current_core.interrupt_disable_depth == 0) &&
                         (current_core.preempt_disable_depth   == 0);

   if (baseline) {
      // The next safe point is now - resolve immediately. Clear any stale
      // pending flag; we are servicing it here.
      current_core.reschedule_pending = false;
      switch_to_scheduler_fiber();
   } else {
      // Kernel-masked: defer. The flag is drained at the next safe point -
      // whichever of irq_restore() / preempt_enable() leaves both depths at 0.
      current_core.reschedule_pending = true;
   }
}

extern "C" void cyros_port_thread_exit(void)
{
   CYROS_ASSERT(global.active_contexts.load(std::memory_order_relaxed) != 0);
   global.active_contexts.fetch_sub(1, std::memory_order_seq_cst);
}

/* ============================================================================
 * SMP & Multi-Core Support
 *
 * Each pthread represents a simulated "core". Core 0 runs on the calling
 * thread, additional cores spawn as pthreads.
 * ========================================================================= */


void cpu_core::start_scheduler()
{
   scheduler_fiber = boost::context::fiber(
      [this](boost::context::fiber&& caller) mutable -> boost::context::fiber
      {
         current_core.os_caller = std::move(caller);

         // Run kernel entry for this simulated core (will start first thread etc.)
         entry();

         // Cooperative pump until only idle threads remain (one idle thread per core).
         while (global.active_contexts.load(std::memory_order_acquire) > global.cores.size()) {
            global.reschedule_handler();
         }

         // First core to observe quiescence initiates shutdown.
         bool expected = false;
         if (global.shutdown_requested.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
            for (auto& c : global.cores) {
               cyros_port_send_reschedule_ipi(c.core_id);
            }
         }

         return std::move(current_core.os_caller);
      }
   );

   // Enter the scheduler fiber. When it returns, this OS-thread is done.
   scheduler_fiber = std::move(scheduler_fiber).resume();
}

void cyros_port_start_cores(size_t cores_to_use, cyros_port_core_entry_t entry)
{
   CYROS_ASSERT(cores_to_use > 0); // Invoking with 0 cores_to_use is invalid
   CYROS_ASSERT(cores_to_use <= CYROS_PORT_CORE_COUNT);

   global.cores = cpu_core_array(cores_to_use, entry);

   for (auto& core : global.cores) {
      // No need to spawn the first core/thread as that is assigned to this current calling core/thread
      if (core.core_id == 0) continue;

      pthread_create(
         &core.pthread,
         nullptr,
         +[](void* arg)-> void*
         {
            auto* init = static_cast<cpu_core*>(arg);
            current_core.core = init;

            // Enter the scheduler-fiber
            init->start_scheduler();

            // On exit, we finish this OS-thread instance and core0's OS-thread can join with us
            return nullptr;
         },
         &core
      );
   }

   // core0 runs on calling thread
   auto& core0 = global.cores[0];
   current_core.core = &core0;

   // Enter the scheduler-fiber for core0
   core0.start_scheduler();

   // When core0's scheduler-fiber returns, join to any other active Core OS-thread
   for (auto& core : global.cores) {
      if (core.core_id == 0) continue;
      pthread_join(core.pthread, nullptr);
   }
   global.reset();
}

extern "C" uint32_t cyros_port_get_core_id(void)
{
   // If no cores have been explicitly launched yet, then we must be on core0
   if (!global.cores_launched()) return 0;

   CYROS_ASSERT(current_core.core != nullptr);
   return current_core.core->core_id;
}

extern "C" void cyros_port_send_reschedule_ipi(uint32_t core_id)
{
   CYROS_ASSERT_OP(core_id, <, global.cores.size());

   auto& core_poke = global.cores[core_id].core_poke;

   // Set the pending bit first (release) so the woken core sees it.
   core_poke.pending.store(true, std::memory_order_release);

   // Wake the core if it is blocked in idle().
   pthread_mutex_lock(&core_poke.mutex);
   pthread_cond_signal(&core_poke.cond_var);
   pthread_mutex_unlock(&core_poke.mutex);

   if (core_id == current_core.core->core_id && current_core.current_context) {
      // Targeting our own core from within a thread fiber: an IPI is a weak,
      // deferred-safe request - route through pend_reschedule(), not a forced
      // synchronous yield.
      cyros_port_pend_reschedule();
   }
}

/* ============================================================================
 * Thread-Local Storage
 * ========================================================================= */

extern "C" void cyros_port_set_tls_pointer(void* tls_base)
{
   current_core.tls_pointer = tls_base;
}

extern "C" void* cyros_port_get_tls_pointer(void)
{
   return current_core.tls_pointer;
}

/* ============================================================================
 * Time Driver Port
 *
 * Provides monotonic time for real drivers (periodic / tickless) in unit
 * tests, plus tickless one-shot arming and ISR delivery when pumped.
 *
 * Note:
 * - The simulation time driver owns time and does NOT use this.
 * - Periodic driver unit tests call on_timer_isr() directly.
 * ========================================================================= */

struct time_state
{
   std::atomic<bool>        irq_enabled{false};
   std::atomic<uint64_t>            now{0};
   std::atomic<uint64_t> armed_deadline{UINT64_MAX};

   std::atomic<cyros_port_isr_handler_t> isr{nullptr};
   std::atomic<void*>                 isr_arg{nullptr};
};
static constinit time_state time_instance;

extern "C" void cyros_port_time_setup(uint32_t tick_hz)
{
   (void)tick_hz;
}

extern "C" uint64_t cyros_port_time_now(void)
{
   return time_instance.now.load(std::memory_order_relaxed);
}

extern "C" uint64_t cyros_port_time_freq_hz(void)
{
   return 1'000'000ull; // 1 tick = 1 us (recommend)
}

extern "C" void cyros_port_time_reset(uint64_t t)
{
   time_instance.now.store(t, std::memory_order_release);
   time_instance.armed_deadline.store(UINT64_MAX, std::memory_order_release);
}

extern "C" void cyros_port_time_register_isr_handler(cyros_port_isr_handler_t h, void* arg)
{
   time_instance.isr_arg.store(arg, std::memory_order_relaxed);
   time_instance.isr.store(h, std::memory_order_release);
}

extern "C" void cyros_port_time_irq_enable(void)  { time_instance.irq_enabled.store(true,  std::memory_order_release); }
extern "C" void cyros_port_time_irq_disable(void) { time_instance.irq_enabled.store(false, std::memory_order_release); }

extern "C" void cyros_port_time_arm(uint64_t deadline)
{
   // Keep earliest
   uint64_t cur = time_instance.armed_deadline.load(std::memory_order_relaxed);
   while (deadline < cur &&
            !time_instance.armed_deadline.compare_exchange_weak(cur, deadline,
                                                   std::memory_order_release,
                                                   std::memory_order_relaxed))
   {}
}

extern "C" void cyros_port_time_disarm(void)
{
   time_instance.armed_deadline.store(UINT64_MAX, std::memory_order_release);
}

// Linux-only helper for tests
extern "C" void cyros_port_time_advance(uint64_t delta)
{
   time_instance.now.fetch_add(delta, std::memory_order_release);
}

extern "C" void cyros_port_send_time_ipi(uint32_t /*core_id*/)
{
   // SMP simulation TODO: poke target core thread.
}

/* ============================================================================
 * CPU Hints & Idle
 * ========================================================================= */

extern "C" void cyros_port_cpu_relax(void)
{
   // CPU yield hint for busy-wait loops
#if defined(__x86_64__) || defined(__i386__)
   __builtin_ia32_pause();
#elif defined(__aarch64__) || defined(__arm__)
   __asm__ __volatile__("yield");
#endif
}

extern "C" void cyros_port_idle(void)
{
   // std::printf("(CORE %d) cyros_port_idle()\n", current_core.core->core_id);
   auto& core_poke = global.cores[current_core.core->core_id].core_poke;

   // Fast path: don't sleep if already pending.
   if (core_poke.pending.exchange(false, std::memory_order_acq_rel)) {
      return;
   }

   pthread_mutex_lock(&core_poke.mutex);

   // Re-check under lock (avoids missed wake if signal happens between fast-path and lock)
   while (!core_poke.pending.exchange(false, std::memory_order_acq_rel)) {
      pthread_cond_wait(&core_poke.cond_var, &core_poke.mutex);
   }

   pthread_mutex_unlock(&core_poke.mutex);
}

/* ============================================================================
 * Debug & Diagnostics
 * ========================================================================= */

static void print_formatted_context(char const* file, int target_line, int range = 2)
{
   // Colour Constants
   static constexpr auto CLR_RESET  = "\033[0m";
   static constexpr auto CLR_RED    = "\033[1;31m";
   static constexpr auto CLR_ORANGE = "\033[38;5;208m";

   std::ifstream fs(file);
   if (!fs.is_open()) return;

   std::string text;
   int current = 0;
   int start = (target_line - range > 0) ? target_line - range : 1;
   int end = target_line + range;

   while (std::getline(fs, text)) {
      current++;
      if (current >= start && current <= end) {
         std::printf("├ ");
         std::printf("%s%4d%s  ", CLR_ORANGE, current, CLR_RESET);
         if (current == target_line) {
            std::printf("%s>> %s%s\n", CLR_RED, text.c_str(), CLR_RESET);
         } else {
            std::printf("   %s\n", text.c_str());
         }
      }
      if (current > end) break;
   }
}

extern "C" void cyros_port_system_error(uintptr_t auxilary1, uintptr_t auxilary2, char const* file_optional, int line_optional)
{
   std::printf("KERNEL PANIC at %s:%d\n", file_optional, line_optional);
   print_formatted_context(file_optional, line_optional);
   std::printf("└ AUX1: 0x%lX, AUX2: 0x%lX\n", auxilary1, auxilary2);
   std::terminate();
}

extern "C" void cyros_port_breakpoint(void)
{
#if defined(__x86_64__) || defined(__i386__)
   __asm__ __volatile__("int3");
#elif defined(__aarch64__) || defined(__arm__)
   __builtin_trap();
#else
   raise(SIGTRAP);
#endif
}

extern "C" void* cyros_port_get_stack_pointer(void)
{
   void* sp;
#if defined(__x86_64__)
   __asm__ __volatile__("mov %%rsp, %0" : "=r"(sp));
#elif defined(__i386__)
   __asm__ __volatile__("mov %%esp, %0" : "=r"(sp));
#elif defined(__aarch64__)
   __asm__ __volatile__("mov %0, sp" : "=r"(sp));
#elif defined(__arm__)
   __asm__ __volatile__("mov %0, sp" : "=r"(sp));
#else
   int dummy;
   sp = &dummy;
#endif
   return sp;
}
