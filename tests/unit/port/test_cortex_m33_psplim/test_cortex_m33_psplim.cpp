/**
 * @file test_cortex_m33_psplim.cpp
 * @brief The ARMv8-M stack-limit guard, which is why this port targets an M33.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the cortex_m33 port's use of PSPLIM, programmed in
 *          cyros_port_context_init and reloaded on every cyros_port_switch.
 * Trusts:  layer 0 and the kernel bring-up layer 2 proves.
 * Proves:  that PSPLIM is set to the running thread's own stack base, that
 *          ordinary stack use does not trip it, and that overrunning the
 *          buffer raises a UsageFault with STKOF rather than corrupting
 *          whatever lies below.
 *
 *
 * WHY THIS MATTERS MORE HERE THAN ON MOST RTOSes
 * ==============================================
 * In cyros THE CALLER OWNS THE STACK. A thread's stack is a buffer the user
 * declared, and the TCB is carved out of that same buffer. So an overrun does
 * not hit a guard page or a canary, it walks into whatever the application put
 * next to it in .bss, silently. The project's existing defence is the
 * ~thread() assert that catches a stack going out of scope while its thread
 * runs, and that catches a DIFFERENT failure: lifetime, not depth.
 *
 * ARMv8-M's PSPLIM closes the depth half in hardware, at zero cost, and that
 * capability is the concrete reason this port targets an M33 rather than the
 * M4 that a cheaper board would have given. A guard nothing tests is not a
 * guard, which is what this file is for.
 *
 *
 * HOW IT ASSERTS A FAULT
 * ======================
 * The bench's startup declares each fault vector as its own weak symbol
 * aliased to a reporting handler. This test replaces UsageFault_Handler only.
 * Everything else still reports normally, so a BusFault raised by a mistake in
 * this test cannot be mistaken for the success case.
 *
 * The handler is where the test ENDS: it records the result and exits the
 * image, because there is nothing sensible to return to once a thread has run
 * off the bottom of its stack.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/arm/bench.hpp>

#include <cstddef>
#include <cstdint>

using namespace cyros;

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 2048;
alignas(CYROS_PORT_STACK_ALIGN) std::byte worker_stack[stack_size];

/* Configurable Fault Status Register. UFSR occupies its top half, and STKOF
 * (stack overflow, ARMv8-M) is bit 4 of UFSR, so bit 20 of CFSR. */
constexpr std::uintptr_t scb_cfsr = 0xE000ED28u;
constexpr std::uint32_t  cfsr_stkof = 1u << 20;

/* Set immediately before the deliberate overrun. The handler refuses to treat
 * a fault as success unless the test asked for one, so an accidental fault
 * anywhere else in this file still reports as a failure. */
volatile bool expecting_overflow = false;

std::uint32_t read_psplim()
{
   std::uint32_t value;
   asm volatile("mrs %0, psplim" : "=r"(value) :: "memory");
   return value;
}

std::uint32_t read_sp()
{
   std::uint32_t value;
   asm volatile("mov %0, sp" : "=r"(value));
   return value;
}


void test_psplim_bounds_this_threads_stack()
{
   cyros::bench::start("PSPLIM is programmed to this thread's own buffer");

   std::uint32_t const limit = read_psplim();
   std::uint32_t const sp    = read_sp();
   auto const base = reinterpret_cast<std::uint32_t>(worker_stack);
   std::uint32_t const top = base + stack_size;

   CYROS_CHECK(limit != 0u);
   CYROS_CHECK(limit >= base);
   CYROS_CHECK(limit < top);

   /* The running stack pointer must be inside the buffer and above the limit,
    * or the guard is protecting something other than this thread. */
   CYROS_CHECK(sp > limit);
   CYROS_CHECK(sp <= top);

   cyros::bench::print("  stack base = ");
   cyros::bench::print_hex(base);
   cyros::bench::print("\n  psplim     = ");
   cyros::bench::print_hex(limit);
   cyros::bench::print("\n  sp         = ");
   cyros::bench::print_hex(sp);
   cyros::bench::print("\n");
}

void test_ordinary_stack_use_does_not_trip_it()
{
   cyros::bench::start("ordinary stack use does not trip the guard");

   /* Comfortably inside the buffer. If the limit were set to the wrong end, or
    * to the top of the stack, this would fault and the test would never get to
    * the interesting part. */
   volatile std::uint8_t scratch[512];
   for (std::size_t i = 0; i < sizeof(scratch); ++i) {
      scratch[i] = static_cast<std::uint8_t>(i);
   }

   std::uint32_t sum = 0;
   for (std::size_t i = 0; i < sizeof(scratch); ++i) { sum += scratch[i]; }

   CYROS_CHECK(sum != 0u);
   CYROS_CHECK(read_sp() > read_psplim());
}

[[noreturn]] void overrun_the_stack()
{
   cyros::bench::start("running off the bottom of the stack raises STKOF");
   expecting_overflow = true;

   /* Move SP below the limit in one step. A recursive function would also work
    * but its depth depends on the optimiser, and this has to be deterministic:
    * the test's success condition is that the NEXT instruction faults.
    *
    * The distance is COMPUTED rather than a constant. The first version of
    * this test subtracted a fixed 4096, which on a 6.4 KB stack left SP a
    * comfortable 2 KB ABOVE the limit, so nothing faulted and the test
    * reported the guard as broken when it was working perfectly.
    *
    * On ARMv8-M the limit is checked on the SP update itself, so the SUB is
    * what faults, and it does so before anything is written through the bad
    * pointer. That is the property worth having: the overrun is stopped, not
    * merely detected afterwards. */
   std::uint32_t const drop = (read_sp() - read_psplim()) + 256u;

   asm volatile(
      "sub sp, sp, %0  \n"
      "str r0, [sp]    \n"
      :: "r"(drop) : "memory");

   /* Unreachable if the guard works, which is the assertion. */
   cyros::bench::record(false, "stack overrun did not fault", __FILE__, __LINE__);
   cyros::bench::print(
      "  SP went below PSPLIM and nothing happened. Either PSPLIM is not being\n"
      "  programmed, or UsageFault is not enabled in SHCSR.\n");
   cyros::bench::finish();
}

void worker()
{
   test_psplim_bounds_this_threads_stack();
   test_ordinary_stack_use_does_not_trip_it();
   overrun_the_stack();
}

} // namespace


/* Replaces the bench's weak default. Runs on MSP, so it has a working stack
 * even though the thread's is exhausted, which is exactly why the split
 * MSP/PSP model is worth having. */
extern "C" [[noreturn]] void UsageFault_Handler(void)
{
   std::uint32_t const cfsr = *reinterpret_cast<volatile std::uint32_t*>(scb_cfsr);

   if (!expecting_overflow) {
      cyros::bench::print("\n*** unexpected UsageFault, CFSR = ");
      cyros::bench::print_hex(cfsr);
      cyros::bench::print(" ***\n");
      cyros::bench::host_exit(3u);
   }

   cyros::bench::record((cfsr & cfsr_stkof) != 0u,
                        "UsageFault reports CFSR.STKOF", __FILE__, __LINE__);

   if ((cfsr & cfsr_stkof) == 0u) {
      cyros::bench::print("  faulted, but not with STKOF. CFSR = ");
      cyros::bench::print_hex(cfsr);
      cyros::bench::print("\n");
   }

   cyros::bench::finish();
}


extern "C" int cyros_bench_main()
{
   cyros::bench::print("cortex_m33 PSPLIM stack guard\n\n");

   kernel::initialise();
   thread worker_thread(worker, worker_stack, thread::priority(0), core0);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
