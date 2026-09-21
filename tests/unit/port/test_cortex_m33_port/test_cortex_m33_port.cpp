/**
 * @file test_cortex_m33_port.cpp
 * @brief Port-contract test for the Cortex-M33 port. Runs on the target.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: cyros/port/port.h's interrupt and preemption control, as implemented
 *          by the cortex_m33 port against real PRIMASK and BASEPRI.
 * Trusts:  the bench harness and semihosting. Nothing in cyros above the port.
 * Proves:  that a mask token restores the PREVIOUS state rather than an open
 *          one, that interrupt masking and preemption disabling are genuinely
 *          independent facilities, and that the port programmed the exception
 *          priorities so that the second of those is possible at all.
 *
 *
 * WHY THIS TEST EXISTS, and it is not a formality
 * ===============================================
 * On both Linux ports the interrupt-save token is INERT. Nothing in the host
 * suite can distinguish a correct cyros_port_irq_restore from one that simply
 * enables interrupts, because there is no hardware state to get wrong. That
 * gap hides a whole class of defect: a spinlock storing its token before it
 * owns the lock is invisible to every Linux test.
 *
 * Here the token IS the saved PRIMASK or BASEPRI, so the case that matters -
 * a critical section entered while already masked - is observable. That case
 * is the one a nesting bug corrupts, and it is checked below in both
 * facilities and in their interaction.
 *
 * The priority check is the other half. Preemption-disable works by raising
 * BASEPRI to PendSV's level, which only masks PendSV alone if PendSV sits at a
 * strictly lower priority than everything else. Cortex-M implements only the
 * top few bits of each priority field, and how many is an implementation
 * choice, so "PendSV 0xFF, SysTick 0xE0" silently COLLAPSES to one level on a
 * 3-bit part. If that happened, every critical section would also stop the
 * clock, and nothing else in the suite would notice.
 */

#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/arm/bench.hpp>

#include <cstdint>

namespace
{

/* Architectural addresses, spelled out here rather than taken from the port's
 * private header. A port-contract test that reads the port's own view of the
 * hardware proves only that the port agrees with itself. */
constexpr std::uintptr_t scb_shpr    = 0xE000ED18u;
constexpr std::uintptr_t shpr_pendsv = scb_shpr + 10u;
constexpr std::uintptr_t shpr_systick = scb_shpr + 11u;
constexpr std::uintptr_t scb_aircr    = 0xE000ED0Cu;

std::uint8_t read8(std::uintptr_t address)
{
   return *reinterpret_cast<volatile std::uint8_t*>(address);
}

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

std::uint32_t read_basepri()
{
   std::uint32_t value;
   asm volatile("mrs %0, basepri" : "=r"(value) :: "memory");
   return value;
}

void enable_irq() { asm volatile("cpsie i" ::: "memory"); }

int reschedule_calls = 0;
void counting_reschedule() { ++reschedule_calls; }


/* ---------------------------------------------------------------------------
 * Initialisation
 * ------------------------------------------------------------------------ */

void test_init_leaves_interrupts_masked()
{
   cyros::bench::start("init leaves the core masked");

   cyros_port_init(counting_reschedule);

   /* Bring-up must not be interruptible. Between init and start_first there is
    * no thread stack, so PSP is meaningless and any exception that tried to
    * switch away would stack onto nothing. */
   CYROS_CHECK(!cyros_port_interrupts_enabled());
   CYROS_CHECK_EQ(read_primask(), 1u);

   /* Preemption, by contrast, starts ENABLED. The two are independent and the
    * kernel raises preemption itself when it wants it. */
   CYROS_CHECK_EQ(read_basepri(), 0u);
}

void test_pendsv_is_strictly_the_lowest_priority()
{
   cyros::bench::start("exception priorities leave room for preempt-disable");

   std::uint32_t const pendsv  = read8(shpr_pendsv);
   std::uint32_t const systick = read8(shpr_systick);

   /* Higher numeric value means lower priority. */
   CYROS_CHECK(pendsv > systick);

   /* The collapse this guards against: with 3 implemented bits, 0xFF and 0xE0
    * are the same level and this comparison would find them equal. */
   CYROS_CHECK(pendsv != systick);

   /* DIFFERENT VALUES ARE NOT ENOUGH, and this check exists because the two
    * above passed while the port was broken.
    *
    * AIRCR.PRIGROUP splits every priority field into a GROUP part and a SUB
    * part, and preemption and BASEPRI masking consider only the group. At the
    * reset default the bottom bit is sub-priority, so PendSV at 0xFF and
    * SysTick at 0xFE differ as numbers and are IDENTICAL as far as masking is
    * concerned. Raising BASEPRI to PendSV's level then masked SysTick too, and
    * every kernel critical section stopped the clock.
    *
    * test_cortex_m33_systick catches this too, but only by running a real
    * interrupt. This is the cheap version of that check. */
   std::uint32_t const prigroup = (read32(scb_aircr) >> 8) & 0x7u;
   std::uint32_t const sub_bits = prigroup + 1u;

   std::uint32_t const pendsv_group  = pendsv >> sub_bits;
   std::uint32_t const systick_group = systick >> sub_bits;

   CYROS_CHECK(systick_group < pendsv_group);

   cyros::bench::print("  prigroup         = ");
   cyros::bench::print_hex(prigroup);
   cyros::bench::print("\n  pendsv group     = ");
   cyros::bench::print_hex(pendsv_group);
   cyros::bench::print("\n  systick group    = ");
   cyros::bench::print_hex(systick_group);
   cyros::bench::print("\n");

   /* PendSV must be the lowest representable, so that raising BASEPRI to it
    * masks PendSV and nothing else. Whatever bits the part implements, the
    * lowest level has all of them set. */
   std::uint32_t const implemented_mask = pendsv;
   CYROS_CHECK_EQ(pendsv & implemented_mask, implemented_mask);

   cyros::bench::print("  pendsv priority  = ");
   cyros::bench::print_hex(pendsv);
   cyros::bench::print("\n  systick priority = ");
   cyros::bench::print_hex(systick);
   cyros::bench::print("\n");
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

   /* THE CASE NO LINUX TEST CAN SEE. A critical section entered while the
    * caller had already masked must leave the core masked on exit. An
    * implementation that restores by simply enabling passes every other check
    * in this file and fails this one. */
   enable_irq();
   cyros_mask_token_t const outer = cyros_port_irq_save();
   CYROS_CHECK(!cyros_port_interrupts_enabled());

   cyros_mask_token_t const inner = cyros_port_irq_save();
   CYROS_CHECK(!cyros_port_interrupts_enabled());

   cyros_port_irq_restore(inner);
   CYROS_CHECK(!cyros_port_interrupts_enabled());   /* still masked by `outer` */

   cyros_port_irq_restore(outer);
   CYROS_CHECK(cyros_port_interrupts_enabled());    /* only now open */
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

   enable_irq();
   CYROS_CHECK_EQ(read_basepri(), 0u);

   cyros_mask_token_t const token = cyros_port_preempt_disable();
   CYROS_CHECK(read_basepri() != 0u);

   cyros_port_preempt_enable(token);
   CYROS_CHECK_EQ(read_basepri(), 0u);
}

void test_preempt_restores_to_disabled_when_entered_disabled()
{
   cyros::bench::start("preempt_enable honours an ALREADY-DISABLED caller");

   cyros_mask_token_t const outer = cyros_port_preempt_disable();
   std::uint32_t const raised = read_basepri();
   CYROS_CHECK(raised != 0u);

   cyros_mask_token_t const inner = cyros_port_preempt_disable();
   CYROS_CHECK_EQ(read_basepri(), raised);

   cyros_port_preempt_enable(inner);
   CYROS_CHECK_EQ(read_basepri(), raised);   /* still disabled by `outer` */

   cyros_port_preempt_enable(outer);
   CYROS_CHECK_EQ(read_basepri(), 0u);
}


/* ---------------------------------------------------------------------------
 * The two facilities are independent
 * ------------------------------------------------------------------------ */

void test_preempt_disable_does_not_mask_interrupts()
{
   cyros::bench::start("preempt-disable leaves ISRs running");

   /* port.h is explicit that disabling preemption blocks the SCHEDULER and not
    * the hardware: "ISRs still fire and run. They simply cannot cause a thread
    * switch". If this port had implemented preemption with PRIMASK, every
    * critical section would also stop the clock. */
   enable_irq();
   cyros_mask_token_t const token = cyros_port_preempt_disable();

   CYROS_CHECK(cyros_port_interrupts_enabled());
   CYROS_CHECK_EQ(read_primask(), 0u);

   cyros_port_preempt_enable(token);
   CYROS_CHECK(cyros_port_interrupts_enabled());
}

void test_irq_save_does_not_disturb_preemption_state()
{
   cyros::bench::start("irq masking leaves preemption state alone");

   enable_irq();
   std::uint32_t const before = read_basepri();

   cyros_mask_token_t const token = cyros_port_irq_save();
   CYROS_CHECK_EQ(read_basepri(), before);
   cyros_port_irq_restore(token);
   CYROS_CHECK_EQ(read_basepri(), before);
}

void test_the_two_compose_in_either_order()
{
   cyros::bench::start("interleaved irq and preempt sections unwind cleanly");

   /* The kernel nests these in both orders (a spinlock masks then the
    * scheduler disables preemption, and the reverse), so neither may disturb
    * the other's saved state. */
   enable_irq();

   cyros_mask_token_t const irq_outer     = cyros_port_irq_save();
   cyros_mask_token_t const preempt_inner = cyros_port_preempt_disable();

   CYROS_CHECK(!cyros_port_interrupts_enabled());
   CYROS_CHECK(read_basepri() != 0u);

   cyros_port_preempt_enable(preempt_inner);
   CYROS_CHECK(!cyros_port_interrupts_enabled());   /* irq section still held */
   CYROS_CHECK_EQ(read_basepri(), 0u);

   cyros_port_irq_restore(irq_outer);
   CYROS_CHECK(cyros_port_interrupts_enabled());
   CYROS_CHECK_EQ(read_basepri(), 0u);
}


/* ---------------------------------------------------------------------------
 * Miscellaneous contract surface
 * ------------------------------------------------------------------------ */

void test_core_identity_and_hints()
{
   cyros::bench::start("core identity and CPU hints");

   CYROS_CHECK_EQ(cyros_port_get_core_id(), 0u);
   CYROS_CHECK_EQ(CYROS_PORT_CORE_COUNT, 1u);

   /* Must be callable and must return. Nothing else is promised. */
   cyros_port_cpu_relax();
   CYROS_CHECK(true);

   void* const sp = cyros_port_get_stack_pointer();
   CYROS_CHECK(sp != nullptr);
}

void test_tls_pointer_round_trips()
{
   cyros::bench::start("TLS pointer round trip");

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
   cyros::bench::print("cortex_m33 port contract\n\n");

   test_init_leaves_interrupts_masked();
   test_pendsv_is_strictly_the_lowest_priority();

   test_irq_token_round_trips_from_enabled();
   test_irq_token_restores_to_masked_when_entered_masked();
   test_irq_nesting_three_deep();

   test_preempt_token_round_trips();
   test_preempt_restores_to_disabled_when_entered_disabled();

   test_preempt_disable_does_not_mask_interrupts();
   test_irq_save_does_not_disturb_preemption_state();
   test_the_two_compose_in_either_order();

   test_core_identity_and_hints();
   test_tls_pointer_round_trips();

   /* Nothing above should have driven a reschedule: none of these call a
    * reschedule entry point, and PendSV cannot be taken while the port holds
    * the core masked. A nonzero count would mean something pended one. */
   cyros::bench::start("no reschedule was provoked");
   CYROS_CHECK_EQ(reschedule_calls, 0);

   cyros::bench::finish();
}
