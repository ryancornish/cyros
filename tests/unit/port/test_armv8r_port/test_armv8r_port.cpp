/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_armv8r_port.cpp
 * @brief Port-contract test for the armv8r core layer. Runs on QEMU's
 *        mps3-an536.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: port.h's interrupt and preemption control and its reschedule
 *          delivery, as the armv8r layer implements them on CPSR.I, the GIC's
 *          priority mask and SGI 0, and the exception path's frame.
 * Trusts:  the bench harness and semihosting. Nothing in cyros above the port.
 * Proves:  that a mask token restores the PREVIOUS state, that the two
 *          facilities are independent, that a preempt-disable masks the
 *          reschedule and not the timer, that a reschedule pended while
 *          either is masked waits for the unmask and then runs exactly once,
 *          on the interrupt stack with interrupts masked, that a yield runs
 *          it synchronously, and that every register and the condition flags
 *          survive an exception.
 *
 *
 * WHY A LAYER-0 TEST CAN TAKE A RESCHEDULE HERE
 * =============================================
 * As on RISC-V: the exception path stores the frame on whatever System-mode
 * stack it arrives on, and before cyros_port_start_first the Supervisor stack
 * is the board's. So a reschedule pended from this test's main is a real
 * exception through the real entry and exit, and the handler it reaches is
 * this test's counter. That makes the register check below possible at layer
 * 0, and it is the check that matters most: the entry and exit are hand
 * written, and one wrong register list still runs, still switches, and quietly
 * replaces a register.
 */

#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <cstdint>

namespace
{

/* Architectural state, read here rather than through the port's private
 * header. A contract test that reads the port's own view of the hardware
 * proves only that the port agrees with itself. */
constexpr std::uint32_t cpsr_i        = 1u << 7;
constexpr std::uint32_t cpsr_mode     = 0x1Fu;
constexpr std::uint32_t mode_svc      = 0x13u;
constexpr std::uint32_t mode_sys      = 0x1Fu;
constexpr std::uint32_t timer_intid   = 30u;

std::uint32_t read_cpsr() { std::uint32_t v; asm volatile("mrs %0, cpsr" : "=r"(v) :: "memory"); return v; }
std::uint32_t read_pmr()  { std::uint32_t v; asm volatile("mrc p15, 0, %0, c4, c6, 0" : "=r"(v) :: "memory"); return v; }
std::uint32_t read_cbar() { std::uint32_t v; asm volatile("mrc p15, 1, %0, c15, c3, 0" : "=r"(v)); return v; }
std::uint32_t read_tpidrprw() { std::uint32_t v; asm volatile("mrc p15, 0, %0, c13, c0, 4" : "=r"(v) :: "memory"); return v; }
void enable_irq() { asm volatile("cpsie i \n isb" ::: "memory"); }

/* Core 0's redistributor, SGI frame: the R52 puts it 1 MB above PERIPHBASE. */
std::uintptr_t sgi_frame() { return (read_cbar() & 0xFFE00000u) + 0x100000u + 0x10000u; }
std::uint32_t reg(std::uintptr_t offset) { return *reinterpret_cast<volatile std::uint32_t*>(sgi_frame() + offset); }
std::uint32_t priority_of(std::uint32_t intid) { return *reinterpret_cast<volatile std::uint8_t*>(sgi_frame() + 0x400u + intid); }

bool irq_masked()    { return (read_cpsr() & cpsr_i) != 0u; }
/* An interrupt is signalled only when its priority is numerically BELOW PMR. */
bool lets_through(std::uint32_t intid) { return priority_of(intid) < read_pmr(); }
bool preempt_open()  { return lets_through(0u); }
bool sgi0_pending()  { return (reg(0x200u) & 1u) != 0u; }   /* GICR_ISPENDR0 */

volatile int reschedule_calls = 0;
volatile std::uint32_t handler_cpsr = 0;
volatile std::uintptr_t handler_sp = 0;

void counting_reschedule()
{
   reschedule_calls = reschedule_calls + 1;
   handler_cpsr = read_cpsr();
   handler_sp = reinterpret_cast<std::uintptr_t>(cyros_port_get_stack_pointer());
}

/* An SGI is a write to the GIC and arrives a little after it. Long enough for
 * that, and nothing more is promised. */
void settle()
{
   for (int i = 0; i < 64; ++i) { asm volatile("nop"); }
}


/* ---------------------------------------------------------------------------
 * Initialisation
 * ------------------------------------------------------------------------ */

void test_init_leaves_the_core_masked()
{
   cyros::bench::start("init leaves the core masked, preemption enabled");

   cyros_port_init(counting_reschedule);

   /* Bring-up must not be interruptible: until start_first there is no thread
    * to switch. And it runs in System mode, as threads do. */
   CYROS_CHECK(!cyros_port_interrupts_enabled());
   CYROS_CHECK(irq_masked());
   CYROS_CHECK_EQ(read_cpsr() & cpsr_mode, mode_sys);

   /* Preemption starts ENABLED, independently. */
   CYROS_CHECK(preempt_open());

   /* The reschedule is SGI 0 and the timer PPI 30, both Group 1 and enabled,
    * and the timer more urgent than the reschedule. */
   CYROS_CHECK_EQ(reg(0x080u) & 1u, 1u);                                 /* IGROUPR0 */
   CYROS_CHECK_EQ(reg(0x100u) & 1u, 1u);                                 /* ISENABLER0 */
   CYROS_CHECK_EQ((reg(0x080u) >> timer_intid) & 1u, 1u);
   CYROS_CHECK_EQ((reg(0x100u) >> timer_intid) & 1u, 1u);
   CYROS_CHECK(priority_of(timer_intid) < priority_of(0u));

   /* And no reschedule is left pending. */
   CYROS_CHECK(!sgi0_pending());
}


/* ---------------------------------------------------------------------------
 * Interrupt masking
 * ------------------------------------------------------------------------ */

void test_irq_token_round_trips_from_enabled()
{
   cyros::bench::start("irq_save/restore from enabled");

   enable_irq();
   CYROS_CHECK(cyros_port_interrupts_enabled());

   cyros_mask_token_t const token = cyros_port_irq_save();
   CYROS_CHECK(!cyros_port_interrupts_enabled());

   cyros_port_irq_restore(token);
   CYROS_CHECK(cyros_port_interrupts_enabled());
}

void test_irq_token_restores_to_masked_when_entered_masked()
{
   cyros::bench::start("irq_restore honours an ALREADY-MASKED caller");

   enable_irq();
   cyros_mask_token_t const outer = cyros_port_irq_save();
   cyros_mask_token_t const inner = cyros_port_irq_save();
   CYROS_CHECK(!cyros_port_interrupts_enabled());

   cyros_port_irq_restore(inner);
   CYROS_CHECK(!cyros_port_interrupts_enabled());   /* still masked by `outer` */

   cyros_port_irq_restore(outer);
   CYROS_CHECK(cyros_port_interrupts_enabled());
}

void test_irq_nesting_three_deep()
{
   cyros::bench::start("irq nesting, three deep");

   enable_irq();
   cyros_mask_token_t const t1 = cyros_port_irq_save();
   cyros_mask_token_t const t2 = cyros_port_irq_save();
   cyros_mask_token_t const t3 = cyros_port_irq_save();

   cyros_port_irq_restore(t3);
   CYROS_CHECK(!cyros_port_interrupts_enabled());
   cyros_port_irq_restore(t2);
   CYROS_CHECK(!cyros_port_interrupts_enabled());
   cyros_port_irq_restore(t1);
   CYROS_CHECK(cyros_port_interrupts_enabled());
}


/* ---------------------------------------------------------------------------
 * Preemption control
 * ------------------------------------------------------------------------ */

void test_preempt_token_round_trips()
{
   cyros::bench::start("preempt_disable/enable round trip");

   CYROS_CHECK(preempt_open());
   cyros_mask_token_t const token = cyros_port_preempt_disable();
   CYROS_CHECK(!preempt_open());
   cyros_port_preempt_enable(token);
   CYROS_CHECK(preempt_open());
}

void test_preempt_restores_to_disabled_when_entered_disabled()
{
   cyros::bench::start("preempt_enable honours an ALREADY-DISABLED caller");

   cyros_mask_token_t const outer = cyros_port_preempt_disable();
   cyros_mask_token_t const inner = cyros_port_preempt_disable();
   CYROS_CHECK(!preempt_open());

   cyros_port_preempt_enable(inner);
   CYROS_CHECK(!preempt_open());   /* still disabled by `outer` */

   cyros_port_preempt_enable(outer);
   CYROS_CHECK(preempt_open());
}

void test_preempt_disable_masks_the_reschedule_and_not_the_timer()
{
   cyros::bench::start("preempt-disable masks the reschedule, never the timer");

   cyros_mask_token_t const token = cyros_port_preempt_disable();
   CYROS_CHECK(!lets_through(0u));
   CYROS_CHECK(lets_through(timer_intid));
   cyros_port_preempt_enable(token);
   CYROS_CHECK(lets_through(0u));
   CYROS_CHECK(lets_through(timer_intid));
}


/* ---------------------------------------------------------------------------
 * The two facilities are independent
 * ------------------------------------------------------------------------ */

void test_preempt_disable_does_not_mask_interrupts()
{
   cyros::bench::start("preempt-disable leaves interrupts enabled");

   enable_irq();
   cyros_mask_token_t const token = cyros_port_preempt_disable();
   CYROS_CHECK(cyros_port_interrupts_enabled());
   CYROS_CHECK(!irq_masked());
   cyros_port_preempt_enable(token);
}

void test_irq_save_does_not_disturb_preemption_state()
{
   cyros::bench::start("irq masking leaves preemption state alone");

   enable_irq();
   cyros_mask_token_t const token = cyros_port_irq_save();
   CYROS_CHECK(preempt_open());
   cyros_port_irq_restore(token);
   CYROS_CHECK(preempt_open());
}

void test_the_two_compose_in_either_order()
{
   cyros::bench::start("interleaved irq and preempt sections unwind cleanly");

   enable_irq();
   cyros_mask_token_t const irq_outer     = cyros_port_irq_save();
   cyros_mask_token_t const preempt_inner = cyros_port_preempt_disable();
   CYROS_CHECK(irq_masked());
   CYROS_CHECK(!preempt_open());

   cyros_port_preempt_enable(preempt_inner);
   CYROS_CHECK(irq_masked());
   CYROS_CHECK(preempt_open());

   cyros_port_irq_restore(irq_outer);
   CYROS_CHECK(!irq_masked());
   CYROS_CHECK(preempt_open());
}


/* ---------------------------------------------------------------------------
 * Reschedule delivery, through the real exception path
 * ------------------------------------------------------------------------ */

void test_pend_while_interrupts_masked_waits_for_the_restore()
{
   cyros::bench::start("a reschedule pended under irq_save runs at the restore, once");

   enable_irq();
   int const before = reschedule_calls;
   cyros_mask_token_t const token = cyros_port_irq_save();

   cyros_port_pend_reschedule();
   settle();
   CYROS_CHECK_EQ(reschedule_calls, before);   /* held */
   CYROS_CHECK(sgi0_pending());                /* and visibly pending */

   cyros_port_irq_restore(token);
   settle();
   CYROS_CHECK_EQ(reschedule_calls, before + 1);
   CYROS_CHECK(!sgi0_pending());               /* consumed, not left pending */
}

void test_pend_while_preemption_disabled_waits_for_the_enable()
{
   cyros::bench::start("a reschedule pended under preempt_disable runs at the enable, once");

   enable_irq();
   int const before = reschedule_calls;
   cyros_mask_token_t const token = cyros_port_preempt_disable();

   cyros_port_pend_reschedule();
   settle();
   CYROS_CHECK_EQ(reschedule_calls, before);
   CYROS_CHECK(sgi0_pending());

   cyros_port_preempt_enable(token);
   settle();
   CYROS_CHECK_EQ(reschedule_calls, before + 1);
   CYROS_CHECK(!sgi0_pending());
}

void test_pend_at_baseline_runs_promptly()
{
   cyros::bench::start("a reschedule pended at baseline runs promptly, masked, on the interrupt stack");

   enable_irq();
   int const before = reschedule_calls;
   std::uintptr_t const here = reinterpret_cast<std::uintptr_t>(cyros_port_get_stack_pointer());
   cyros_port_pend_reschedule();
   settle();
   CYROS_CHECK_EQ(reschedule_calls, before + 1);

   /* Handlers run in Supervisor mode with I set, never on the thread's
    * stack: the board gives Supervisor mode a stack of its own. */
   CYROS_CHECK_EQ(handler_cpsr & cpsr_mode, mode_svc);
   CYROS_CHECK((handler_cpsr & cpsr_i) != 0u);
   CYROS_CHECK(handler_sp != 0u);
   CYROS_CHECK(handler_sp > here || handler_sp + 4096u < here);
}

void test_yield_is_synchronous()
{
   cyros::bench::start("thread_yield runs the reschedule before it returns");

   enable_irq();
   int const before = reschedule_calls;
   cyros_port_thread_yield();
   /* No settle: the strong guarantee is that it already happened. */
   CYROS_CHECK_EQ(reschedule_calls, before + 1);
   CYROS_CHECK(!irq_masked());                      /* the exit put I back */
   CYROS_CHECK_EQ(read_cpsr() & cpsr_mode, mode_sys);  /* and the mode */
}


/* ---------------------------------------------------------------------------
 * Every register and the flags survive an exception
 *
 * Loads a pattern into r0 to r12 and lr and sets all four condition flags,
 * takes a reschedule that is already pending, then stores what came back,
 * the flags included, for C to compare. Naked and self-contained, so the
 * compiler holds nothing of its own in them. The callee-saved registers and
 * the return address are kept on the stack around it.
 * ------------------------------------------------------------------------ */

extern "C" [[gnu::naked]] void registers_after_an_exception([[maybe_unused]] std::uint32_t* out)
{
   asm volatile(
      "push   {r4-r11, lr}               \n"
      "push   {r0}                       \n"   /* out */
      "mov    r0, #0xF0000000            \n"
      "msr    apsr_nzcvq, r0             \n"   /* N, Z, C, V: movw and movt keep them */
      ".irp n, 0,1,2,3,4,5,6,7,8,9,10,11,12 \n"
      "movw   r\\n, #\\n                  \n"
      "movt   r\\n, #0xA500              \n"
      ".endr                             \n"
      "movw   lr, #13                    \n"
      "movt   lr, #0xA500                \n"
      "cpsie  i                          \n"   /* the pending reschedule is taken here */
      "isb                               \n"
      "nop                               \n"
      "cpsid  i                          \n"
      "push   {r0-r12, lr}               \n"   /* what came back, 14 words */
      "mrs    r0, apsr                   \n"
      "ldr    r1, [sp, #56]              \n"   /* out */
      "str    r0, [r1, #56]              \n"   /* the flags, word 14 */
      "movs   r2, #0                     \n"
      "1:                                \n"
      "ldr    r0, [sp, r2, lsl #2]       \n"
      "str    r0, [r1, r2, lsl #2]       \n"
      "adds   r2, r2, #1                 \n"
      "cmp    r2, #14                    \n"
      "bne    1b                         \n"
      "add    sp, sp, #60                \n"
      "pop    {r4-r11, pc}               \n");
}

void test_every_register_survives_an_exception()
{
   cyros::bench::start("every register and the flags survive an exception");

   int const before = reschedule_calls;
   cyros_mask_token_t const token = cyros_port_irq_save();
   cyros_port_pend_reschedule();
   settle();
   CYROS_CHECK(sgi0_pending());

   std::uint32_t after[15] = {};
   registers_after_an_exception(after);

   /* The exception must actually have been taken inside the routine, or the
    * check proves nothing. */
   CYROS_CHECK_EQ(reschedule_calls, before + 1);

   std::uint32_t wrong = 0u;
   for (std::uint32_t n = 0; n < 14u; ++n) {
      if (after[n] != (0xA5000000u | n)) {
         ++wrong;
         cyros::bench::print("  wrong: r");
         cyros::bench::print_hex(n);
         cyros::bench::print(" = ");
         cyros::bench::print_hex(after[n]);
         cyros::bench::print("\n");
      }
   }
   CYROS_CHECK_EQ(wrong, 0u);
   CYROS_CHECK_EQ(after[14] & 0xF0000000u, 0xF0000000u);   /* N, Z, C and V */

   cyros_port_irq_restore(token);
}


/* ---------------------------------------------------------------------------
 * Miscellaneous contract surface
 * ------------------------------------------------------------------------ */

void test_core_identity_and_hints()
{
   cyros::bench::start("core identity and CPU hints");

   CYROS_CHECK_EQ(cyros_port_get_core_id(), 0u);
   CYROS_CHECK_EQ(CYROS_PORT_CORE_COUNT, 1u);
   cyros_port_cpu_relax();
   CYROS_CHECK(cyros_port_get_stack_pointer() != nullptr);
}

void test_tls_pointer_is_the_thread_id_register()
{
   cyros::bench::start("TLS pointer round trip, and it is TPIDRPRW");

   int marker = 0;
   cyros_port_set_tls_pointer(&marker);
   CYROS_CHECK(cyros_port_get_tls_pointer() == &marker);
   CYROS_CHECK(read_tpidrprw() == reinterpret_cast<std::uintptr_t>(&marker));

   cyros_port_set_tls_pointer(nullptr);
   CYROS_CHECK(cyros_port_get_tls_pointer() == nullptr);
}

void test_timestamp_moves()
{
   cyros::bench::start("timestamp moves forward");

   std::uint64_t const first = cyros_port_timestamp();
   for (int i = 0; i < 10000; ++i) { asm volatile("nop"); }
   std::uint64_t const second = cyros_port_timestamp();
   CYROS_CHECK(second > first);
}

void test_init_drops_a_reschedule_left_pending()
{
   cyros::bench::start("a reschedule left pending is dropped by the next init, never run");

   /* The kernel's lifecycles end and begin again, and a request pended by the
    * last must not fire into the next one's first thread. */
   int const before = reschedule_calls;
   cyros_mask_token_t const token = cyros_port_irq_save();
   cyros_port_pend_reschedule();
   settle();
   CYROS_CHECK(sgi0_pending());

   cyros_port_init(counting_reschedule);
   CYROS_CHECK(!sgi0_pending());

   cyros_port_irq_restore(token);
   settle();
   CYROS_CHECK_EQ(reschedule_calls, before);
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("armv8r port contract\n\n");

   test_init_leaves_the_core_masked();

   test_irq_token_round_trips_from_enabled();
   test_irq_token_restores_to_masked_when_entered_masked();
   test_irq_nesting_three_deep();

   test_preempt_token_round_trips();
   test_preempt_restores_to_disabled_when_entered_disabled();
   test_preempt_disable_masks_the_reschedule_and_not_the_timer();

   test_preempt_disable_does_not_mask_interrupts();
   test_irq_save_does_not_disturb_preemption_state();
   test_the_two_compose_in_either_order();

   test_pend_while_interrupts_masked_waits_for_the_restore();
   test_pend_while_preemption_disabled_waits_for_the_enable();
   test_pend_at_baseline_runs_promptly();
   test_yield_is_synchronous();
   test_every_register_survives_an_exception();

   test_core_identity_and_hints();
   test_tls_pointer_is_the_thread_id_register();
   test_timestamp_moves();
   test_init_drops_a_reschedule_left_pending();

   /* Exactly the five reschedules the tests above asked for. */
   cyros::bench::start("no reschedule beyond the ones asked for");
   CYROS_CHECK_EQ(reschedule_calls, 5);

   cyros::bench::finish();
}
