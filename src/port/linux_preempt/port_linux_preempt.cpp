
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
static_assert(CYROS_PORT_SCHEDULING_TYPE == 2);
static_assert(CYROS_PORT_ENVIRONMENT == 2);

void f() {}