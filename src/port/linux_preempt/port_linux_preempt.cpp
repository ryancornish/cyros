
#include <cyros/port/port.h>

#include <sigctx/sigctx.h>


/* ============================================================================
 * Port Context Structure
 * ========================================================================= */

struct cyros_port_context
{
   sigctx_inl_t         context;
   void*                stack_top;
   size_t               stack_size;
   cyros_port_entry_t   entry;
   void*                arg;
};

// Verify that port_traits.h constants are correct
static_assert(sizeof(cyros_port_context) == CYROS_PORT_CONTEXT_SIZE,
              "CYROS_PORT_CONTEXT_SIZE mismatch - adjust in port_traits.h");
static_assert(alignof(cyros_port_context) == CYROS_PORT_CONTEXT_ALIGN,
              "CYROS_PORT_CONTEXT_ALIGN mismatch - adjust in port_traits.h");
static_assert((CYROS_PORT_STACK_ALIGN & (CYROS_PORT_STACK_ALIGN - 1)) == 0,
              "CYROS_PORT_STACK_ALIGN must be a power of two");

/* ============================================================================
 * Port MultiCore Structure
 * ========================================================================= */

struct cpu_core
{
   pthread_t pthread{}; // @note This is null/unused for core0
   uint32_t  core_id{}; // Index from 0... num cores
   cyros_port_core_entry_t entry{};


   void start_scheduler();
};

/* ============================================================================
 * Platform Initialization
 * ========================================================================= */

void cyros_port_init(cyros_port_reschedule_t reschedule_handler);

/* ============================================================================
 * Critical Sections (Interrupt Control)
 * ========================================================================= */

void cyros_port_disable_interrupts(void);

void cyros_port_enable_interrupts(void);

bool cyros_port_interrupts_enabled(void);

uint32_t cyros_port_irq_save(void);

void cyros_port_irq_restore(uint32_t state);

/* ============================================================================
 * Preemption Control
 * ========================================================================= */

void cyros_port_preempt_disable(void);

void cyros_port_preempt_enable(void);

/* ============================================================================
 * Context Management & Switching
 * ========================================================================= */

void cyros_port_context_init(cyros_port_context_t* context,
                              void* stack_base,
                              size_t stack_size,
                              cyros_port_entry_t entry,
                              void* arg);

void cyros_port_context_destroy(cyros_port_context_t* context);

void cyros_port_switch(cyros_port_context_t* from, cyros_port_context_t* to);

void cyros_port_start_first(cyros_port_context_t* first);

/* ============================================================================
 * Reschedule Requests
 * ========================================================================= */

void cyros_port_thread_yield(void);

void cyros_port_pend_reschedule(void);

void cyros_port_thread_exit(void);// __attribute__((noreturn));

/* ============================================================================
 * SMP & Multi-Core Support
 * ========================================================================= */

uint32_t cyros_port_get_core_id(void);

void cyros_port_start_cores(size_t cores_to_use, cyros_port_core_entry_t entry);

void cyros_port_send_reschedule_ipi(uint32_t core_id);

/* ============================================================================
 * Thread-Local Storage
 * ========================================================================= */

void cyros_port_set_tls_pointer(void* tls_base);

void* cyros_port_get_tls_pointer(void);



/* ============================================================================
 * Time Driver Port
 * ========================================================================= */

void cyros_port_time_setup(uint32_t tick_hz);

uint64_t cyros_port_time_now(void);

uint64_t cyros_port_time_freq_hz(void);

void cyros_port_time_reset(uint64_t time);

void cyros_port_time_register_isr_handler(cyros_port_isr_handler_t handler, void* arg);

void cyros_port_time_irq_enable(void);

void cyros_port_time_irq_disable(void);

void cyros_port_time_arm(uint64_t deadline);

void cyros_port_time_disarm(void);

void cyros_port_send_time_ipi(uint32_t core_id);

/* ============================================================================
 * CPU Hints & Idle
 * ========================================================================= */

void cyros_port_cpu_relax(void);

void cyros_port_idle(void);

/* ============================================================================
 * Debug & Diagnostics
 * ========================================================================= */

void cyros_port_system_error(uintptr_t auxilary1, uintptr_t auxilary2, char const* file_optional, int line_optional) __attribute__((noreturn));

void cyros_port_breakpoint(void);

void* cyros_port_get_stack_pointer(void);