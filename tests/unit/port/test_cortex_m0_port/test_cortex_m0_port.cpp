/**
 * @file test_cortex_m0_port.cpp
 * @brief Port-contract test for the ARMv6-M port (`cortex_m0`). Runs on the
 *        target.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: port.h's interrupt and preemption control as the armv6m core layer
 *          implements them: PRIMASK for interrupts, and for preemption a
 *          software deferral, since ARMv6-M has no BASEPRI.
 * Trusts:  the bench harness and semihosting. Nothing in cyros above the port.
 * Proves:  that a mask token restores the PREVIOUS state, that a reschedule
 *          pended inside a preempt-disabled region is taken by PendSV,
 *          deferred, and delivered exactly once by the outermost enable, that
 *          the deferral never masks interrupts, and that PendSV sits strictly
 *          below SysTick.
 *
 * test_cortex_m_port is the Mainline sibling, and its header says why the
 * token cases matter. What is new here is the deferral, which no hardware
 * register implements. Its reschedule handler only counts, so PendSV returns
 * to this same thread, which runs on MSP while PendSV stores and reloads r4 to
 * r11 through PSP: PSP is pointed at a scratch buffer for that.
 */

#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <cstdint>

namespace
{

/* Architectural addresses, spelled out rather than taken from the port's
 * header, so the test does not only prove the port agrees with itself. */
constexpr std::uintptr_t scb_icsr  = 0xE000ED04u;
constexpr std::uintptr_t scb_shpr3 = 0xE000ED20u;   /* word access only on ARMv6-M */
constexpr std::uint32_t icsr_pendsvset = 1u << 28;

std::uint32_t read32(std::uintptr_t address)
{
   return *reinterpret_cast<volatile std::uint32_t*>(address);
}

std::uint32_t read_primask()
{
   std::uint32_t value;
   asm volatile("mrs %0, primask" : "=r"(value) :: "memory");
   return value;
}

void enable_irq() { asm volatile("cpsie i" ::: "memory"); }

volatile int reschedule_calls = 0;
void counting_reschedule() { reschedule_calls = reschedule_calls + 1; }

/* PendSV stores r4 to r11 below PSP, and this test runs on MSP with no thread
 * stack, so PSP gets a buffer of its own. */
alignas(8) std::uint32_t psp_scratch[64];

void point_psp_at_scratch()
{
   std::uint32_t* const top = psp_scratch + 64;
   asm volatile("msr psp, %0" :: "r"(top) : "memory");
}


/* ---------------------------------------------------------------------------
 * Initialisation
 * ------------------------------------------------------------------------ */

void test_init_leaves_interrupts_masked()
{
   cyros::bench::start("init leaves the core masked, preemption enabled");

   cyros_port_init(counting_reschedule);
   CYROS_CHECK(!cyros_port_interrupts_enabled());
   CYROS_CHECK_EQ(read_primask(), 1u);

   /* Preemption starts enabled: a first disable returns depth zero. */
   cyros_mask_token_t const token = cyros_port_preempt_disable();
   CYROS_CHECK_EQ(token, 0u);
   cyros_port_preempt_enable(token);
}

void test_pendsv_is_strictly_below_systick()
{
   cyros::bench::start("PendSV is the lowest priority, SysTick above it");

   std::uint32_t const shpr3   = read32(scb_shpr3);
   std::uint32_t const pendsv  = (shpr3 >> 16) & 0xFFu;
   std::uint32_t const systick = (shpr3 >> 24) & 0xFFu;

   /* ARMv6-M implements the top two bits: 0xC0 is the lowest level. */
   CYROS_CHECK_EQ(pendsv, 0xC0u);
   CYROS_CHECK(systick < pendsv);

   cyros::bench::print("  pendsv priority  = ");
   cyros::bench::print_hex(pendsv);
   cyros::bench::print("\n  systick priority = ");
   cyros::bench::print_hex(systick);
   cyros::bench::print("\n");
}


/* ---------------------------------------------------------------------------
 * Interrupt masking
 * ------------------------------------------------------------------------ */

void test_irq_token_restores_to_masked_when_entered_masked()
{
   cyros::bench::start("irq_restore honours an ALREADY-MASKED caller");

   enable_irq();
   cyros_mask_token_t const outer = cyros_port_irq_save();
   CYROS_CHECK(!cyros_port_interrupts_enabled());

   cyros_mask_token_t const inner = cyros_port_irq_save();
   cyros_port_irq_restore(inner);
   CYROS_CHECK(!cyros_port_interrupts_enabled());   /* still masked by `outer` */

   cyros_port_irq_restore(outer);
   CYROS_CHECK(cyros_port_interrupts_enabled());
}


/* ---------------------------------------------------------------------------
 * Preemption control: the deferral
 * ------------------------------------------------------------------------ */

void test_preempt_tokens_are_the_depth_before()
{
   cyros::bench::start("preempt tokens restore the depth before, nested");

   cyros_mask_token_t const outer = cyros_port_preempt_disable();
   cyros_mask_token_t const inner = cyros_port_preempt_disable();
   CYROS_CHECK_EQ(outer, 0u);
   CYROS_CHECK_EQ(inner, 1u);

   cyros_port_preempt_enable(inner);
   /* Still disabled by `outer`: a third disable sees depth one. */
   cyros_mask_token_t const probe = cyros_port_preempt_disable();
   CYROS_CHECK_EQ(probe, 1u);
   cyros_port_preempt_enable(probe);

   cyros_port_preempt_enable(outer);
   cyros_mask_token_t const after = cyros_port_preempt_disable();
   CYROS_CHECK_EQ(after, 0u);
   cyros_port_preempt_enable(after);
}

void test_preempt_disable_leaves_interrupts_running()
{
   cyros::bench::start("preempt-disable leaves interrupts unmasked");

   /* The reason the grade is a deferral and not PRIMASK: port.h's ISRs keep
    * running inside a preempt-disable. */
   enable_irq();
   cyros_mask_token_t const token = cyros_port_preempt_disable();
   CYROS_CHECK(cyros_port_interrupts_enabled());
   CYROS_CHECK_EQ(read_primask(), 0u);
   cyros_port_preempt_enable(token);
}

void test_a_reschedule_outside_a_region_is_delivered_at_once()
{
   cyros::bench::start("a reschedule at baseline is delivered at once");

   point_psp_at_scratch();
   enable_irq();
   int const before = reschedule_calls;
   cyros_port_pend_reschedule();
   CYROS_CHECK_EQ(reschedule_calls, before + 1);

   cyros_port_thread_yield();
   CYROS_CHECK_EQ(reschedule_calls, before + 2);
}

void test_a_reschedule_inside_a_region_is_owed_then_delivered()
{
   cyros::bench::start("a reschedule inside preempt-disable waits for the enable");

   point_psp_at_scratch();
   enable_irq();
   int const before = reschedule_calls;

   cyros_mask_token_t const token = cyros_port_preempt_disable();
   cyros_port_pend_reschedule();

   /* Not delivered. And PendSV RAN and deferred: it is no longer pending,
    * which a hardware mask would have left it. */
   CYROS_CHECK_EQ(reschedule_calls, before);
   CYROS_CHECK_EQ(read32(scb_icsr) & icsr_pendsvset, 0u);

   cyros_port_preempt_enable(token);
   CYROS_CHECK_EQ(reschedule_calls, before + 1);

   /* Delivered once: a second enable owes nothing. */
   cyros_mask_token_t const again = cyros_port_preempt_disable();
   cyros_port_preempt_enable(again);
   CYROS_CHECK_EQ(reschedule_calls, before + 1);
}

void test_only_the_outermost_enable_delivers()
{
   cyros::bench::start("only the outermost enable delivers an owed reschedule");

   point_psp_at_scratch();
   enable_irq();
   int const before = reschedule_calls;

   cyros_mask_token_t const outer = cyros_port_preempt_disable();
   cyros_mask_token_t const inner = cyros_port_preempt_disable();
   cyros_port_pend_reschedule();

   cyros_port_preempt_enable(inner);
   CYROS_CHECK_EQ(reschedule_calls, before);

   cyros_port_preempt_enable(outer);
   CYROS_CHECK_EQ(reschedule_calls, before + 1);
}

void test_an_owed_reschedule_waits_out_an_interrupt_mask()
{
   cyros::bench::start("an owed reschedule enabled under a mask runs at the unmask");

   point_psp_at_scratch();
   enable_irq();
   int const before = reschedule_calls;

   cyros_mask_token_t const preempt = cyros_port_preempt_disable();
   cyros_port_pend_reschedule();
   cyros_mask_token_t const irq = cyros_port_irq_save();

   cyros_port_preempt_enable(preempt);
   CYROS_CHECK_EQ(reschedule_calls, before);           /* pended, held by PRIMASK */
   CYROS_CHECK((read32(scb_icsr) & icsr_pendsvset) != 0u);

   cyros_port_irq_restore(irq);
   CYROS_CHECK_EQ(reschedule_calls, before + 1);
}


/* ---------------------------------------------------------------------------
 * Miscellaneous contract surface
 * ------------------------------------------------------------------------ */

void test_core_identity_and_tls()
{
   cyros::bench::start("core identity, CPU hint and TLS");

   CYROS_CHECK_EQ(cyros_port_get_core_id(), 0u);
   CYROS_CHECK_EQ(CYROS_PORT_CORE_COUNT, 1u);
   cyros_port_cpu_relax();
   CYROS_CHECK(cyros_port_get_stack_pointer() != nullptr);

   int marker = 0;
   cyros_port_set_tls_pointer(&marker);
   CYROS_CHECK(cyros_port_get_tls_pointer() == &marker);
   cyros_port_set_tls_pointer(nullptr);
   CYROS_CHECK(cyros_port_get_tls_pointer() == nullptr);
}

} // namespace


/* Entry point, called by the board startup. See bench.hpp. */
extern "C" int cyros_bench_main()
{
   cyros::bench::print("armv6m port contract\n\n");

   test_init_leaves_interrupts_masked();
   test_pendsv_is_strictly_below_systick();

   test_irq_token_restores_to_masked_when_entered_masked();

   test_preempt_tokens_are_the_depth_before();
   test_preempt_disable_leaves_interrupts_running();
   test_a_reschedule_outside_a_region_is_delivered_at_once();
   test_a_reschedule_inside_a_region_is_owed_then_delivered();
   test_only_the_outermost_enable_delivers();
   test_an_owed_reschedule_waits_out_an_interrupt_mask();

   test_core_identity_and_tls();

   /* Five deliveries above, and no stray one. */
   cyros::bench::start("every reschedule accounted for");
   CYROS_CHECK_EQ(reschedule_calls, 5);

   cyros::bench::finish();
}
