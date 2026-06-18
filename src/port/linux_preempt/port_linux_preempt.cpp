/**
 * @file port_linux_boost.cpp
 * @brief Linux simulation port using sigctx
 *
 * @TODO: Description
 */

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

/* ============================================================================
 * Verify Port Traits
 * ========================================================================= */
static_assert(sizeof(cyros_port_context) == CYROS_PORT_CONTEXT_SIZE,
              "CYROS_PORT_CONTEXT_SIZE mismatch - adjust in port_traits.h");
static_assert(alignof(cyros_port_context) == CYROS_PORT_CONTEXT_ALIGN,
              "CYROS_PORT_CONTEXT_ALIGN mismatch - adjust in port_traits.h");
static_assert((CYROS_PORT_STACK_ALIGN & (CYROS_PORT_STACK_ALIGN - 1)) == 0,
              "CYROS_PORT_STACK_ALIGN must be a power of two");


struct cpu_core
{
   pthread_t pthread{}; // @note This is null/unused for core0
   uint32_t  core_id{}; // Index from 0... num cores
   cyros_port_core_entry_t entry{};
};



/* ============================================================================
 * Port Contract API
 * ----------------------------------------------------------------------------
 * Complete implementation of the contract:
 * ========================================================================= */

/* ----------------------------------------------------------------------------
 * Platform Initialisation
 * ------------------------------------------------------------------------- */

void cyros_port_init(cyros_port_reschedule_t reschedule_handler);


/* ----------------------------------------------------------------------------
 * SMP & Multi-Core Support
 *
 * Each pthread represents a simulated "core". Core 0 runs on the calling
 * thread, additional cores spawn as pthreads.
 * ------------------------------------------------------------------------- */

uint32_t cyros_port_get_core_id(void);

void cyros_port_start_cores(size_t cores_to_use, cyros_port_core_entry_t entry);

void cyros_port_send_reschedule_ipi(uint32_t core_id);


/* ----------------------------------------------------------------------------
 * Interrupt Control
 * ------------------------------------------------------------------------- */

void cyros_port_disable_interrupts(void);

void cyros_port_enable_interrupts(void);

bool cyros_port_interrupts_enabled(void);

uint32_t cyros_port_irq_save(void);

void cyros_port_irq_restore(uint32_t state);


/* ----------------------------------------------------------------------------
 * Preemption Control
 * ------------------------------------------------------------------------- */

void cyros_port_preempt_disable(void);

void cyros_port_preempt_enable(void);


/* ----------------------------------------------------------------------------
 * Context Management & Switching
 * ------------------------------------------------------------------------- */

void cyros_port_context_init(cyros_port_context_t* context,
                              void* stack_base,
                              size_t stack_size,
                              cyros_port_entry_t entry,
                              void* arg);

void cyros_port_context_destroy(cyros_port_context_t* context);

void cyros_port_switch(cyros_port_context_t* from, cyros_port_context_t* to);

void cyros_port_start_first(cyros_port_context_t* first);


/* ----------------------------------------------------------------------------
 * Reschedule Requests
 * ------------------------------------------------------------------------- */

void cyros_port_thread_yield(void);

void cyros_port_pend_reschedule(void);

void cyros_port_thread_exit(void);


/* ----------------------------------------------------------------------------
 * Thread-Local Storage
 * ------------------------------------------------------------------------- */

void cyros_port_set_tls_pointer(void* tls_base);

void* cyros_port_get_tls_pointer(void);


/* ----------------------------------------------------------------------------
 * CPU Hints & Idle
 * ------------------------------------------------------------------------- */

void cyros_port_cpu_relax(void);

void cyros_port_idle(void);


/* ----------------------------------------------------------------------------
 * Debug & Diagnostics
 * ------------------------------------------------------------------------- */

void cyros_port_system_error(uintptr_t auxilary1, uintptr_t auxilary2, char const* file_optional, int line_optional) __attribute__((noreturn));

void cyros_port_breakpoint(void);

void* cyros_port_get_stack_pointer(void);
