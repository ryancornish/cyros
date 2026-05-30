/**
 * @file port.h
 * @brief Cyros Port Layer API (C ABI)
 *
 * This is the hardware abstraction layer between the Cyros kernel and
 * platform-specific code. All functions use C linkage for easy implementation
 * in assembly or C.
 *
 * Port implementations must provide all functions declared here.
 */

#ifndef CYROS_PORT_H
#define CYROS_PORT_H

#include <cyros/port/port_traits.h>

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Port Configuration Validation
 * ========================================================================= */

#ifndef CYROS_PORT_CONTEXT_SIZE
# error "Port must define CYROS_PORT_CONTEXT_SIZE"
#endif

#ifndef CYROS_PORT_CONTEXT_ALIGN
# error "Port must define CYROS_PORT_CONTEXT_ALIGN"
#endif

#ifndef CYROS_PORT_STACK_ALIGN
# error "Port must define CYROS_PORT_STACK_ALIGN"
#endif

#ifndef CYROS_PORT_CACHE_LINE
# error "Port must define CYROS_PORT_CACHE_LINE"
#endif

#ifndef CYROS_PORT_CORE_COUNT
# error "Port must define CYROS_PORT_CORE_COUNT"
#endif

#ifndef CYROS_PORT_SCHEDULING_TYPE
# error "Port must define CYROS_PORT_SCHEDULING_TYPE (CYROS_PORT_SCHED_PREEMPTIVE or CYROS_PORT_SCHED_COOPERATIVE)"
#endif

#ifndef CYROS_PORT_ENVIRONMENT
# error "Port must define CYROS_PORT_ENVIRONMENT (CYROS_PORT_ENV_BARE_METAL or CYROS_PORT_ENV_SIMULATION)"
#endif

#if (CYROS_PORT_CONTEXT_SIZE) <= 0
# error "CYROS_PORT_CONTEXT_SIZE must be > 0"
#endif

#if (CYROS_PORT_CONTEXT_ALIGN) <= 0
# error "CYROS_PORT_CONTEXT_ALIGN must be > 0"
#endif

#if (CYROS_PORT_STACK_ALIGN) <= 0
# error "CYROS_PORT_STACK_ALIGN must be > 0"
#endif

#if (CYROS_PORT_CACHE_LINE) <= 0
# error "CYROS_PORT_CACHE_LINE must be > 0"
#endif

#if (CYROS_PORT_CORE_COUNT) <= 0
# error "CYROS_PORT_CORE_COUNT must be > 0"
#endif

#if ((CYROS_PORT_CONTEXT_ALIGN & (CYROS_PORT_CONTEXT_ALIGN - 1)) != 0)
# error "CYROS_PORT_CONTEXT_ALIGN must be a power of two"
#endif

#if ((CYROS_PORT_STACK_ALIGN & (CYROS_PORT_STACK_ALIGN - 1)) != 0)
# error "CYROS_PORT_STACK_ALIGN must be a power of two"
#endif

#if ((CYROS_PORT_CACHE_LINE & (CYROS_PORT_CACHE_LINE - 1)) != 0)
# error "CYROS_PORT_CACHE_LINE must be a power of two"
#endif

#if (CYROS_PORT_SCHEDULING_TYPE != 1) && (CYROS_PORT_SCHEDULING_TYPE != 2)
# error "Invalid CYROS_PORT_SCHEDULING_TYPE. Use CYROS_SCHED_PREEMPTIVE (1) or CYROS_SCHED_COOPERATIVE (2)."
#endif

#if (CYROS_PORT_ENVIRONMENT != 1) && (CYROS_PORT_ENVIRONMENT != 2)
# error "Invalid CYROS_PORT_ENVIRONMENT. Use CYROS_ENV_BARE_METAL (1) or CYROS_ENV_SIMULATION (2)."
#endif

/* ============================================================================
 * Type Definitions
 * ========================================================================= */

/**
 * @brief Opaque context structure (platform-specific size/alignment)
 *
 * Each port defines the actual structure. The kernel treats this as opaque.
 */
typedef struct cyros_port_context cyros_port_context_t;

/**
 * @brief Port->Kernel reschedule hook
 *
 * Installed by the kernel via cyros_port_init(). The port invokes this hook
 * when a reschedule must be serviced on the calling core - i.e. as the
 * back-end of both cyros_port_thread_yield() and cyros_port_pend_reschedule()
 * once the port has determined a reschedule may safely run. The hook runs the
 * core-local scheduler, which may perform a context switch.
 */
typedef void (*cyros_port_reschedule_t)(void);

/**
 * @brief Thread entry point signature
 */
typedef void (*cyros_port_entry_t)(void* arg);

/**
 * @brief Core entry point signature
 */
typedef void (*cyros_port_core_entry_t)(void);

/**
 * @brief ISR signature
 */
typedef void (*cyros_port_isr_handler_t)(void* arg);

/* ============================================================================
 * Platform Initialization
 * ========================================================================= */

/**
 * @brief Initialize the port layer
 * @param reschedule_handler Port->kernel hook invoked to service a reschedule
 *                           on the calling core (see cyros_port_reschedule_t).
 */
void cyros_port_init(cyros_port_reschedule_t reschedule_handler);

/* ============================================================================
 * Critical Sections (Interrupt Control)
 *
 * Interrupt masking blocks the *hardware*: while raised, asynchronous
 * interrupts cannot be delivered to the calling core. This is distinct from
 * preemption control (below) - see the note in the Preemption Control section.
 * ========================================================================= */

/**
 * @brief Disable interrupts
 *
 * In simulation, this may be a no-op or track nesting depth.
 */
void cyros_port_disable_interrupts(void);

/**
 * @brief Enable interrupts
 */
void cyros_port_enable_interrupts(void);

/**
 * @brief Check if interrupts are currently enabled
 * @return true if interrupts are enabled, false otherwise
 */
bool cyros_port_interrupts_enabled(void);

/**
 * @brief Save interrupt state and disable interrupts
 * @return Previous interrupt state
 */
uint32_t cyros_port_irq_save(void);

/**
 * @brief Restore interrupt state
 * @param state Previous state returned by cyros_port_irq_save()
 *
 * When this restores interrupts to fully unmasked AND preemption is also
 * enabled, the calling core is at baseline priority; if a reschedule was
 * pended while masked it is resolved at that point (see Reschedule Requests).
 */
void cyros_port_irq_restore(uint32_t state);

/* ============================================================================
 * Preemption Control
 *
 * Preemption disabling blocks the *scheduler*: while raised, no context switch
 * may occur on the calling core. Interrupts are NOT affected - ISRs still fire
 * and run. They simply cannot cause a thread switch until preemption is
 * re-enabled.
 *
 * Preemption vs interrupt masking
 * -------------------------------
 * These are independent, separately-nesting facilities:
 *  - Interrupt masking defends against a device ISR running mid-mutation.
 *  - Preemption disabling defends against a thread switch while the caller
 *    holds something (e.g. a spinlock) that makes switching unsafe, while
 *    still permitting ISRs to run.
 * A context switch on a core can occur only when BOTH are at zero depth:
 * interrupts unmasked and preemption enabled. That joint condition is what
 * port.h refers to as "baseline priority".
 *
 * The port owns the preemption mechanism because it is platform-specific
 * (a real priority/mask action on bare metal, a counter on a cooperative
 * simulation port). The kernel owns the policy: it decides when to call
 * these, and with what nesting.
 *
 * Both functions nest. Calls must be balanced.
 * ========================================================================= */

/**
 * @brief Disable preemption on the calling core (nestable).
 *
 * Increments the preemption-disable nesting depth. While the depth is non-zero
 * no context switch will be performed on this core. ISRs continue to run. A
 * reschedule they request via cyros_port_pend_reschedule() is recorded and
 * deferred.
 */
void cyros_port_preempt_disable(void);

/**
 * @brief Enable preemption on the calling core (nestable).
 *
 * Decrements the preemption-disable nesting depth. When the depth reaches zero
 * AND interrupts are not masked, the calling core is at baseline priority:
 * this is a contract "safe point", and if a reschedule was pended while
 * preemption was disabled it is resolved here before this call returns.
 *
 * Must be balanced against cyros_port_preempt_disable().
 */
void cyros_port_preempt_enable(void);

/* ============================================================================
 * Context Management & Switching
 * ========================================================================= */

/**
 * @brief Initialize a thread context
 * @param context Pointer to context structure (pre-allocated by kernel)
 * @param stack_base Pointer to the base (lowest address) of the stack
 * @param stack_size Size of the stack in bytes
 * @param entry Thread entry point function
 * @param arg Argument to pass to entry function
 *
 * This function sets up the context so that when port_switch() is called
 * with this context, the thread starts executing at entry(arg).
 */
void cyros_port_context_init(cyros_port_context_t* context,
                              void* stack_base,
                              size_t stack_size,
                              cyros_port_entry_t entry,
                              void* arg);

/**
 * @brief Destroy a thread context
 * @param context Pointer to context to destroy
 *
 * Called when a thread exits. Allows the port to clean up any resources.
 */
void cyros_port_context_destroy(cyros_port_context_t* context);

/**
 * @brief Switch from one context to another
 * @param from Context to save (can be NULL for first switch)
 * @param to Context to restore and resume
 *
 * Saves the current CPU state into 'from' and loads the state from 'to'.
 * Execution resumes in 'to' context.
 */
void cyros_port_switch(cyros_port_context_t* from, cyros_port_context_t* to);

/**
 * @brief Start executing the first thread
 * @param first First thread context to run
 *
 * This is called once at scheduler startup to begin execution.
 * Unlike port_switch(), there's no "from" context to save.
 */
void cyros_port_start_first(cyros_port_context_t* first);

/* ============================================================================
 * Reschedule Requests
 *
 * Cyros distinguishes TWO reschedule operations. They differ in their
 * preconditions and in the strength of their guarantee. The kernel selects the
 * correct one per call site; a port MUST implement both to this contract.
 *
 * Terminology
 * -----------
 *  - "thread context": executing inside a thread's context, not inside an ISR.
 *  - "kernel-masked": the kernel currently holds a critical section on this
 *    core - interrupts masked via cyros_port_irq_save() AND/OR preemption
 *    disabled via cyros_port_preempt_disable().
 *  - "baseline priority": thread context, with interrupts NOT masked AND
 *    preemption NOT disabled - i.e. both nesting depths at zero. On Cortex-M
 *    this is Thread mode with PRIMASK clear, BASEPRI not raised above the
 *    PendSV priority, and PendSV not otherwise gated.
 *  - "next safe point": the next moment, on this core, at which execution
 *    returns to baseline priority - typically the outermost
 *    cyros_port_irq_restore() or cyros_port_preempt_enable(), or ISR return.
 *
 * Why two operations
 * ------------------
 * A reschedule cannot always be performed at the instant it becomes
 * desirable: the requester may be inside an ISR, or inside a kernel critical
 * section, and in either case switching context immediately would be unsound.
 * Splitting the operation makes the caller's intent - and, importantly, the
 * caller's obligations - explicit at every call site.
 *
 * Summary
 * -------
 *  cyros_port_thread_yield()      strong guarantee, thread context only,
 *                                  baseline priority only, synchronous.
 *  cyros_port_pend_reschedule()   weak guarantee, callable from any context,
 *                                  may be deferred.
 * ========================================================================= */

/**
 * @brief Yield the calling thread to the scheduler (synchronous reschedule).
 *
 * Strong-guarantee reschedule. The scheduler runs immediately on the calling
 * core; if another thread is selected, the caller is switched out and this
 * call does NOT return until the caller is later switched back in. When it
 * does return, a full reschedule round-trip has provably completed.
 *
 * Precondition - the caller MUST satisfy ALL of:
 *  - Called from thread context (NEVER from an ISR).
 *  - Called at baseline priority: interrupts not masked AND preemption not
 *    disabled.
 *
 * Ports that can observe the execution priority SHOULD assert the baseline
 * precondition inside this function.
 *
 * Use when the caller is deliberately giving up the CPU and depends on the
 * round-trip having completed on return - e.g. blocking in a wait operation,
 * an explicit yield, or a join. Callers MAY rely on post-switch state being
 * valid immediately after this function returns.
 *
 * Port implementation notes:
 *  - Cortex-M: pend PendSV, then DSB/ISB. From baseline priority PendSV is
 *    taken before the next instruction, making the call synchronous.
 *  - RISC-V: trigger the software interrupt (or switch directly); from
 *    baseline priority it is serviced immediately.
 *  - Linux/boost.context: resume the scheduler fiber.
 */
void cyros_port_thread_yield(void);

/**
 * @brief Request a reschedule on the calling core (deferred-safe).
 *
 * Weak-guarantee reschedule. Guarantees ONLY that a reschedule will occur on
 * this core at or before the next safe point. This call MAY return before any
 * context switch has happened.
 *
 * Callable from ANY context: thread context or ISR, with or without a kernel
 * critical section held.
 *
 * Behaviour by context:
 *  - Baseline priority: the next safe point is "now", so the reschedule
 *    resolves before this call returns. This is observationally identical to
 *    cyros_port_thread_yield() - but see the caller obligation below.
 *  - Kernel-masked, or ISR context: the request is recorded and the actual
 *    reschedule is deferred to the next safe point - the outermost
 *    cyros_port_irq_restore() or cyros_port_preempt_enable(), or ISR return.
 *
 * Caller obligation: callers MUST NOT assume that a context switch has
 * occurred, or that any post-switch state is valid, after this call returns.
 * This holds even in the baseline-priority case: the weak guarantee IS the
 * contract, independent of how a particular context happens to satisfy it. A
 * caller that needs the round-trip must use cyros_port_thread_yield().
 *
 * Use when the caller wants to flag that a reschedule may now be warranted but
 * is not itself blocking - e.g. after making a higher-priority thread ready,
 * from a signal/wake path, or from an ISR handing work to a thread.
 *
 * Port implementation notes:
 *  - Cortex-M: set the PendSV pending bit (ICSR.PENDSVSET). The hardware
 *    defers delivery until execution priority drops to the PendSV level.
 *  - RISC-V: set the machine software interrupt pending bit (CLINT MSIP). The
 *    hardware defers delivery until software interrupts are unmasked.
 *  - Linux/boost.context: if at baseline priority, resume the scheduler fiber
 *    now; otherwise set a per-core "reschedule pending" flag that the
 *    irq_restore / preempt_enable safe points (and simulated-ISR return)
 *    check and resolve.
 */
void cyros_port_pend_reschedule(void);

/**
 * @brief Thread exit handler
 *
 * Called when a thread's entry function returns.
 * Should never return.
 */
void cyros_port_thread_exit(void);// __attribute__((noreturn));

/* ============================================================================
 * SMP & Multi-Core Support
 *
 * Each pthread represents a simulated "core". Core 0 runs on the calling
 * thread, additional cores spawn as pthreads.
 * ========================================================================= */

/**
 * @brief Get the ID of the current CPU core
 * @return Core ID (0-indexed)
 *
 * For single-core systems, always returns 0.
 * For SMP systems, returns which core is executing this code.
 */
uint32_t cyros_port_get_core_id(void);

/**
 * @brief Start (or release) all secondary cores and run entry on every core.
 * @param cores_to_use Number of cores to start
 * @param entry Entry point to run on each core
 *
 * After this call returns on the bootstrap core:
 *  - On embedded: typically never returns because entry will start the first thread.
 *  - On simulation: may return if port_start_first returns (cooperative).
 */
void cyros_port_start_cores(size_t cores_to_use, cyros_port_core_entry_t entry);

/**
 * @brief Send an IPI to another core to trigger a reschedule
 * @param core_id Target core ID
 *
 * Causes the target core to perform a reschedule at its next safe point. This
 * is the cross-core analogue of cyros_port_pend_reschedule(): it carries the
 * same weak guarantee and the receiving core resolves it exactly as a locally
 * pended reschedule would be.
 */
void cyros_port_send_reschedule_ipi(uint32_t core_id);

/* ============================================================================
 * Thread-Local Storage
 * ========================================================================= */

/**
 * @brief Set the TLS pointer for the current thread
 * @param tls_base Pointer to the thread's TLS block
 */
void cyros_port_set_tls_pointer(void* tls_base);

/**
 * @brief Get the current TLS pointer
 * @return Current thread's TLS base pointer
 */
void* cyros_port_get_tls_pointer(void);



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

/**
 * @brief Configure the underlying timer peripheral(s) used for OS time.
 * @param tick_hz Tick frequency (0 for tickless mode)
 *
 * If tick_hz > 0:
 *   Configure a periodic timer interrupt at tick_hz.
 *   The port must deliver the registered ISR handler once per tick IRQ.
 *
 * If tick_hz == 0:
 *   Configure tickless one-shot mode.
 *   The driver will call cyros_port_time_arm()/disarm() to schedule deadlines.
 *   The port must deliver the registered ISR handler when time_now() >= armed deadline.
 *
 * Called by the selected time driver during start().
 */
void cyros_port_time_setup(uint32_t tick_hz);

/**
 * @brief Monotonic time source
 * @return Current time in port ticks
 *
 * Must be monotonic 64-bit in "port ticks" (opaque unit for whole system).
 */
uint64_t cyros_port_time_now(void);

/**
 * @brief Free-running counter frequency in Hz (ticks per second).
 * @return Frequency in Hz
 *
 * For example:
 *  - DWT_CYCCNT at CPU clock: 168'000'000
 *  - Timer running at 1 MHz: 1'000'000
 *  - Linux steady_clock ns ticks: 1'000'000'000
 */
uint64_t cyros_port_time_freq_hz(void);

/**
 * @brief Reset any internal global time tracking state.
 * @param time Initial time value
 *
 * On embedded targets this is typically meaningless or implemented
 * implicitly by a system reset.
 *
 * Intended primarily for simulation and unit testing to provide
 * deterministic startup conditions.
 */
void cyros_port_time_reset(uint64_t time);

/**
 * @brief Register an ISR handler for timer interrupts
 * @param handler ISR callback function
 * @param arg Argument to pass to handler
 */
void cyros_port_time_register_isr_handler(cyros_port_isr_handler_t handler, void* arg);

/**
 * @brief Enable timer interrupts
 */
void cyros_port_time_irq_enable(void);

/**
 * @brief Disable timer interrupts
 */
void cyros_port_time_irq_disable(void);

/**
 * @brief Arm a one-shot interrupt for the given absolute deadline.
 * @param deadline Absolute time in port ticks
 *
 * If called multiple times before the interrupt fires, the port must ensure
 * the earliest deadline is honored (i.e., effectively min(current, deadline)).
 *
 * Must be safe to call with interrupts disabled.
 */
void cyros_port_time_arm(uint64_t deadline);

/**
 * @brief Disable any pending one-shot.
 */
void cyros_port_time_disarm(void);

/**
 * @brief Notify the time core that there is pending time work.
 * @param core_id Target core ID
 *
 * If unimplemented on a platform, it may be an empty function.
 * Used for SMP policy where non-time cores enqueue requests for the time core.
 */
void cyros_port_send_time_ipi(uint32_t core_id);

/* ============================================================================
 * CPU Hints & Idle
 * ========================================================================= */

/**
 * @brief CPU yield hint for busy-wait loops
 */
void cyros_port_cpu_relax(void);

/**
 * @brief Platform-specific idle behavior
 *
 * Called by the kernel's idle thread when no other threads are ready.
 * Can implement power-saving features or cooperative yielding.
 */
void cyros_port_idle(void);

/* ============================================================================
 * Debug & Diagnostics
 * ========================================================================= */

/**
 * @brief Internal Kernel asserts
 * The Kernel has been setup incorrectly, or has hit an internal system error
 */
void cyros_port_system_error(uintptr_t auxilary1, uintptr_t auxilary2, char const* file_optional, int line_optional) __attribute__((noreturn));

#if CYROS_PORT_ENVIRONMENT == 2
 #define CYROS_PORT_CAPTURE_FILE (__FILE__)
 #define CYROS_PORT_CAPTURE_LINE (__LINE__)
#else
 #define CYROS_PORT_CAPTURE_FILE ""
 #define CYROS_PORT_CAPTURE_LINE 0
#endif

#define CYROS_ASSERT2(condition, aux1, aux2) __builtin_expect(!!(condition), 1) ? (void)0 : cyros_port_system_error((uintptr_t)(aux1), (uintptr_t)(aux2), CYROS_PORT_CAPTURE_FILE, CYROS_PORT_CAPTURE_LINE)
#define CYROS_ASSERT1(condition, aux1)       CYROS_ASSERT2(condition, aux1, 0)
#define CYROS_ASSERT(condition)              CYROS_ASSERT2(condition, 0, 0)
#define CYROS_ASSERT_OP(lhs, op, rhs)        CYROS_ASSERT2((lhs) op (rhs), lhs, rhs)
#define CYROS_ASSERT_NULL(pointer)           CYROS_ASSERT2(!(pointer), pointer, 0);

/**
 * @brief Trigger a breakpoint (for debugging)
 */
void cyros_port_breakpoint(void);

/**
 * @brief Get the current stack pointer value
 * @return Current stack pointer
 */
void* cyros_port_get_stack_pointer(void);

#ifdef __cplusplus
}
#endif

#endif /* CYROS_PORT_H */
