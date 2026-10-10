/**
 * @file test_riscv_port.cpp
 * @brief Port-contract test for the rv32 core layer. Runs on QEMU's virt.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: port.h's interrupt and preemption control and its reschedule
 *          delivery, as the rv32 layer implements them on mstatus.MIE, mie.MSIE
 *          and the machine software interrupt, and the trap path's frame.
 * Trusts:  the bench harness and semihosting. Nothing in cyros above the port.
 * Proves:  that a mask token restores the PREVIOUS state, that the two
 *          facilities are independent, that a reschedule pended while either
 *          is masked waits for the unmask and then runs exactly once, that a
 *          yield runs it synchronously, and that every register survives a
 *          trap.
 *
 *
 * WHY A LAYER-0 TEST CAN TAKE A RESCHEDULE HERE
 * =============================================
 * On ARM a reschedule cannot be observed below layer 2, because PendSV
 * switches stacks. Here the trap path stores the frame on whatever stack it
 * arrives on, and before cyros_port_start_first names an interrupt stack it
 * stays there. So a reschedule pended from this test's main thread is a real
 * trap, through the real entry and exit, and the handler it reaches is this
 * test's counter. That makes the register check below possible at layer 0,
 * and it is the check that matters most: the entry and exit are 64 hand-written
 * loads and stores, and one wrong offset still runs, still switches, and
 * quietly replaces a register.
 */

#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <cstdint>

extern "C" void cyros_port_trap_entry(void);

namespace
{

/* Architectural bits, spelled out here rather than taken from the port's
 * private header. A contract test that reads the port's own view of the
 * hardware proves only that the port agrees with itself. */
constexpr std::uint32_t mstatus_mie = 1u << 3;
constexpr std::uint32_t mie_msie    = 1u << 3;
constexpr std::uint32_t mip_msip    = 1u << 3;

std::uint32_t read_mstatus() { std::uint32_t v; asm volatile("csrr %0, mstatus" : "=r"(v) :: "memory"); return v; }
std::uint32_t read_mie()     { std::uint32_t v; asm volatile("csrr %0, mie" : "=r"(v) :: "memory"); return v; }
std::uint32_t read_mip()     { std::uint32_t v; asm volatile("csrr %0, mip" : "=r"(v) :: "memory"); return v; }
std::uint32_t read_mtvec()   { std::uint32_t v; asm volatile("csrr %0, mtvec" : "=r"(v) :: "memory"); return v; }
void enable_irq() { asm volatile("csrsi mstatus, 8" ::: "memory"); }

bool mie_set()  { return (read_mstatus() & mstatus_mie) != 0u; }
bool msie_set() { return (read_mie() & mie_msie) != 0u; }
bool msip_raised() { return (read_mip() & mip_msip) != 0u; }

volatile int reschedule_calls = 0;
void counting_reschedule() { reschedule_calls = reschedule_calls + 1; }

/* A soft IRQ is a store to the CLINT and lands a few cycles after it. Long
 * enough for that, and nothing more is promised. */
void settle()
{
   for (int i = 0; i < 64; ++i) { asm volatile("nop"); }
}


/* ---------------------------------------------------------------------------
 * Initialisation
 * ------------------------------------------------------------------------ */

void test_init_leaves_the_hart_masked()
{
   cyros::bench::start("init leaves the hart masked, preemption enabled");

   cyros_port_init(counting_reschedule);

   /* Bring-up must not be interruptible: until start_first there is no thread
    * to switch. */
   CYROS_CHECK(!cyros_port_interrupts_enabled());
   CYROS_CHECK(!mie_set());

   /* Preemption starts ENABLED, independently. */
   CYROS_CHECK(msie_set());

   /* Every trap goes to the port, in direct mode. */
   CYROS_CHECK_EQ(read_mtvec(), reinterpret_cast<std::uint32_t>(&cyros_port_trap_entry));
   CYROS_CHECK_EQ(read_mtvec() & 3u, 0u);

   /* And no reschedule is left raised. */
   CYROS_CHECK(!msip_raised());
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

   CYROS_CHECK(msie_set());
   cyros_mask_token_t const token = cyros_port_preempt_disable();
   CYROS_CHECK(!msie_set());
   cyros_port_preempt_enable(token);
   CYROS_CHECK(msie_set());
}

void test_preempt_restores_to_disabled_when_entered_disabled()
{
   cyros::bench::start("preempt_enable honours an ALREADY-DISABLED caller");

   cyros_mask_token_t const outer = cyros_port_preempt_disable();
   cyros_mask_token_t const inner = cyros_port_preempt_disable();
   CYROS_CHECK(!msie_set());

   cyros_port_preempt_enable(inner);
   CYROS_CHECK(!msie_set());   /* still disabled by `outer` */

   cyros_port_preempt_enable(outer);
   CYROS_CHECK(msie_set());
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
   CYROS_CHECK(mie_set());
   cyros_port_preempt_enable(token);
}

void test_irq_save_does_not_disturb_preemption_state()
{
   cyros::bench::start("irq masking leaves preemption state alone");

   enable_irq();
   cyros_mask_token_t const token = cyros_port_irq_save();
   CYROS_CHECK(msie_set());
   cyros_port_irq_restore(token);
   CYROS_CHECK(msie_set());
}

void test_the_two_compose_in_either_order()
{
   cyros::bench::start("interleaved irq and preempt sections unwind cleanly");

   enable_irq();
   cyros_mask_token_t const irq_outer     = cyros_port_irq_save();
   cyros_mask_token_t const preempt_inner = cyros_port_preempt_disable();
   CYROS_CHECK(!mie_set());
   CYROS_CHECK(!msie_set());

   cyros_port_preempt_enable(preempt_inner);
   CYROS_CHECK(!mie_set());
   CYROS_CHECK(msie_set());

   cyros_port_irq_restore(irq_outer);
   CYROS_CHECK(mie_set());
   CYROS_CHECK(msie_set());
}


/* ---------------------------------------------------------------------------
 * Reschedule delivery, through the real trap path
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
   CYROS_CHECK(msip_raised());                 /* and visibly pending */

   cyros_port_irq_restore(token);
   settle();
   CYROS_CHECK_EQ(reschedule_calls, before + 1);
   CYROS_CHECK(!msip_raised());                /* consumed, not left raised */
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
   CYROS_CHECK(msip_raised());

   cyros_port_preempt_enable(token);
   settle();
   CYROS_CHECK_EQ(reschedule_calls, before + 1);
   CYROS_CHECK(!msip_raised());
}

void test_pend_at_baseline_runs_promptly()
{
   cyros::bench::start("a reschedule pended at baseline runs promptly");

   enable_irq();
   int const before = reschedule_calls;
   cyros_port_pend_reschedule();
   settle();
   CYROS_CHECK_EQ(reschedule_calls, before + 1);
}

void test_yield_is_synchronous()
{
   cyros::bench::start("thread_yield runs the reschedule before it returns");

   enable_irq();
   int const before = reschedule_calls;
   cyros_port_thread_yield();
   /* No settle: the strong guarantee is that it already happened. */
   CYROS_CHECK_EQ(reschedule_calls, before + 1);
   CYROS_CHECK(mie_set());   /* and the trap's exit put MIE back */
}


/* ---------------------------------------------------------------------------
 * Every register survives a trap
 *
 * Loads a pattern into all 29 general registers the trap path saves and that
 * this routine may own (everything but zero, sp and gp), takes a trap that is
 * already pending, and counts the registers that came back wrong. Naked and
 * self-contained, so the compiler holds nothing of its own in them. ra and gp
 * are kept on the stack and reused as the scratch and the count, because
 * every other register is carrying a pattern.
 * ------------------------------------------------------------------------ */

extern "C" [[gnu::naked]] std::uint32_t registers_wrong_after_a_trap()
{
   asm volatile(
      "addi  sp, sp, -64            \n"
      "sw    ra,  0(sp)             \n"
      "sw    gp,  4(sp)             \n"
      "sw    tp,  8(sp)             \n"
      "sw    s0, 12(sp)             \n"
      "sw    s1, 16(sp)             \n"
      "sw    s2, 20(sp)             \n"
      "sw    s3, 24(sp)             \n"
      "sw    s4, 28(sp)             \n"
      "sw    s5, 32(sp)             \n"
      "sw    s6, 36(sp)             \n"
      "sw    s7, 40(sp)             \n"
      "sw    s8, 44(sp)             \n"
      "sw    s9, 48(sp)             \n"
      "sw    s10, 52(sp)            \n"
      "sw    s11, 56(sp)            \n"
      ".irp n, 4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31 \n"
      "li    x\\n, 0xA5000000 + \\n  \n"
      ".endr                        \n"
      "csrsi mstatus, 8             \n"   /* the pending reschedule is taken here */
      "nop                          \n"
      "nop                          \n"
      "csrci mstatus, 8             \n"
      "li    gp, 0                  \n"   /* the count of wrong registers */
      ".irp n, 4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31 \n"
      "li    ra, 0xA5000000 + \\n    \n"
      "beq   x\\n, ra, 1f            \n"
      "addi  gp, gp, 1              \n"
      "1:                           \n"
      ".endr                        \n"
      "mv    a0, gp                 \n"
      "lw    ra,  0(sp)             \n"
      "lw    gp,  4(sp)             \n"
      "lw    tp,  8(sp)             \n"
      "lw    s0, 12(sp)             \n"
      "lw    s1, 16(sp)             \n"
      "lw    s2, 20(sp)             \n"
      "lw    s3, 24(sp)             \n"
      "lw    s4, 28(sp)             \n"
      "lw    s5, 32(sp)             \n"
      "lw    s6, 36(sp)             \n"
      "lw    s7, 40(sp)             \n"
      "lw    s8, 44(sp)             \n"
      "lw    s9, 48(sp)             \n"
      "lw    s10, 52(sp)            \n"
      "lw    s11, 56(sp)            \n"
      "addi  sp, sp, 64             \n"
      "ret                          \n");
}

void test_every_register_survives_a_trap()
{
   cyros::bench::start("every register survives a trap");

   int const before = reschedule_calls;
   cyros_mask_token_t const token = cyros_port_irq_save();
   cyros_port_pend_reschedule();
   settle();
   CYROS_CHECK(msip_raised());

   std::uint32_t const wrong = registers_wrong_after_a_trap();

   /* The trap must actually have been taken inside the routine, or the check
    * proves nothing. */
   CYROS_CHECK_EQ(reschedule_calls, before + 1);
   CYROS_CHECK_EQ(wrong, 0u);

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

void test_tls_pointer_is_the_thread_pointer()
{
   cyros::bench::start("TLS pointer round trip, and it is tp");

   int marker = 0;
   cyros_port_set_tls_pointer(&marker);
   CYROS_CHECK(cyros_port_get_tls_pointer() == &marker);

   void* tp;
   asm volatile("mv %0, tp" : "=r"(tp));
   CYROS_CHECK(tp == &marker);

   cyros_port_set_tls_pointer(nullptr);
   CYROS_CHECK(cyros_port_get_tls_pointer() == nullptr);
}

void test_timestamp_moves()
{
   cyros::bench::start("timestamp moves forward");

   std::uint64_t const first = cyros_port_timestamp();
   settle();
   std::uint64_t const second = cyros_port_timestamp();
   CYROS_CHECK(second > first);
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("rv32 port contract\n\n");

   test_init_leaves_the_hart_masked();

   test_irq_token_round_trips_from_enabled();
   test_irq_token_restores_to_masked_when_entered_masked();
   test_irq_nesting_three_deep();

   test_preempt_token_round_trips();
   test_preempt_restores_to_disabled_when_entered_disabled();

   test_preempt_disable_does_not_mask_interrupts();
   test_irq_save_does_not_disturb_preemption_state();
   test_the_two_compose_in_either_order();

   test_pend_while_interrupts_masked_waits_for_the_restore();
   test_pend_while_preemption_disabled_waits_for_the_enable();
   test_pend_at_baseline_runs_promptly();
   test_yield_is_synchronous();
   test_every_register_survives_a_trap();

   test_core_identity_and_hints();
   test_tls_pointer_is_the_thread_pointer();
   test_timestamp_moves();

   /* Exactly the five reschedules the tests above asked for. */
   cyros::bench::start("no reschedule beyond the ones asked for");
   CYROS_CHECK_EQ(reschedule_calls, 5);

   cyros::bench::finish();
}
