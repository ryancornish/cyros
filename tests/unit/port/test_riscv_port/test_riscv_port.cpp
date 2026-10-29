/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

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
 *          trap by each of its three ways out: an interrupt that asks for no
 *          reschedule, one that runs a reschedule, and one whose reschedule
 *          switches to another context and back.
 *
 *
 * WHY A LAYER-0 TEST CAN TAKE A RESCHEDULE HERE
 * =============================================
 * On ARM a reschedule cannot be observed below layer 2, because PendSV
 * switches stacks. Here the trap path stores the frame on whatever stack it
 * arrives on, and before cyros_port_start_first names an interrupt stack it
 * stays there. So a reschedule pended from this test's main thread is a real
 * trap, through the real entry and exit, and the handler it reaches is this
 * test's counter, which can switch to a context of the test's own as the
 * kernel's reschedule would. That makes the register checks below possible at
 * layer 0, and they are the checks that matter most: the entry and its ways
 * out are about a hundred hand-written loads and stores, and one wrong offset
 * still runs, still switches, and quietly replaces a register.
 */

#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <cstddef>
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
constexpr std::uint32_t mip_mtip    = 1u << 7;

std::uint32_t read_mstatus() { std::uint32_t v; asm volatile("csrr %0, mstatus" : "=r"(v) :: "memory"); return v; }
std::uint32_t read_mie()     { std::uint32_t v; asm volatile("csrr %0, mie" : "=r"(v) :: "memory"); return v; }
std::uint32_t read_mip()     { std::uint32_t v; asm volatile("csrr %0, mip" : "=r"(v) :: "memory"); return v; }
std::uint32_t read_mtvec()   { std::uint32_t v; asm volatile("csrr %0, mtvec" : "=r"(v) :: "memory"); return v; }
void enable_irq() { asm volatile("csrsi mstatus, 8" ::: "memory"); }

bool mie_set()  { return (read_mstatus() & mstatus_mie) != 0u; }
bool msie_set() { return (read_mie() & mie_msie) != 0u; }
bool msip_raised() { return (read_mip() & mip_msip) != 0u; }
bool mtip_raised() { return (read_mip() & mip_mtip) != 0u; }

/* A switch the hook makes itself, as the kernel's reschedule would: set
 * before the reschedule is pended, taken by the next one. */
cyros_port_context* volatile switch_from = nullptr;
cyros_port_context* volatile switch_to   = nullptr;

volatile int reschedule_calls = 0;
void counting_reschedule()
{
   reschedule_calls = reschedule_calls + 1;
   if (switch_to != nullptr) {
      cyros_port_context* const to = switch_to;
      switch_to = nullptr;
      cyros_port_switch(switch_from, to);
   }
}

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
 * this routine may own (everything but zero, sp and gp), takes a trap, and
 * counts the registers that came back wrong, gp among them, which must come
 * back as it was. The trap is the interrupt already pending, or with `fault`
 * set an illegal instruction in its place, which the image's fault handler
 * resumes past. Naked and self-contained, so the compiler holds nothing of
 * its own in them. ra and gp are kept on the stack and reused as the scratch
 * and the count, ra once its own value is on the stack too, because every
 * other register is carrying a pattern.
 *
 * The 128 bytes below the routine's frame, where the trap stores its own,
 * are poisoned first, so a slot the trap fails to store comes back as the
 * poison, never as whatever an earlier frame happened to leave there.
 * ------------------------------------------------------------------------ */

extern "C" [[gnu::naked]] std::uint32_t registers_wrong_after_a_trap(std::uint32_t /*fault*/)
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
      "li    t0, 0x0BADF00D         \n"   /* the poison */
      "addi  t1, sp, -128           \n"
      "1:                           \n"
      "sw    t0, 0(t1)              \n"
      "addi  t1, t1, 4              \n"
      "bne   t1, sp, 1b             \n"
      "bnez  a0, 2f                 \n"
      ".irp n, 1,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31 \n"
      "li    x\\n, 0xA5000000 + \\n  \n"
      ".endr                        \n"
      "csrsi mstatus, 8             \n"   /* the pending interrupt is taken here */
      "nop                          \n"
      "nop                          \n"
      "csrci mstatus, 8             \n"
      "j     3f                     \n"
      "2:                           \n"
      ".irp n, 1,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31 \n"
      "li    x\\n, 0xA5000000 + \\n  \n"
      ".endr                        \n"
      ".option push                 \n"
      ".option norvc                \n"
      "csrw  mhartid, zero          \n"   /* read-only: the fault is taken here */
      ".option pop                  \n"
      "3:                           \n"
      "sw    ra, 60(sp)             \n"   /* x1 as it came back, checked last */
      "lw    ra,  4(sp)             \n"   /* gp as the routine found it */
      "sub   ra, gp, ra             \n"
      "snez  gp, ra                 \n"   /* the count of wrong registers, gp's first */
      ".irp n, 4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31 \n"
      "li    ra, 0xA5000000 + \\n    \n"
      "beq   x\\n, ra, 1f            \n"
      "addi  gp, gp, 1              \n"
      "1:                           \n"
      ".endr                        \n"
      "lw    t0, 60(sp)             \n"
      "li    t1, 0xA5000000 + 1     \n"
      "beq   t0, t1, 1f             \n"
      "addi  gp, gp, 1              \n"
      "1:                           \n"
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

void test_every_register_survives_a_trap_that_reschedules()
{
   cyros::bench::start("every register survives a trap that runs a reschedule");

   int const before = reschedule_calls;
   cyros_mask_token_t const token = cyros_port_irq_save();
   cyros_port_pend_reschedule();
   settle();
   CYROS_CHECK(msip_raised());

   std::uint32_t const wrong = registers_wrong_after_a_trap(0u);

   /* The trap must actually have been taken inside the routine, or the check
    * proves nothing. */
   CYROS_CHECK_EQ(reschedule_calls, before + 1);
   CYROS_CHECK_EQ(wrong, 0u);

   cyros_port_irq_restore(token);
}

volatile int timer_interrupts = 0;
void counting_timer(void*) { timer_interrupts = timer_interrupts + 1; }

void test_every_register_survives_an_interrupt_that_asks_for_no_reschedule()
{
   cyros::bench::start("every register survives an interrupt that asks for no reschedule");

   /* One timer interrupt, its deadline reached already, whose handler pends
    * nothing: the way out that restores the clobbered half alone. */
   int const before = reschedule_calls;
   cyros_mask_token_t const token = cyros_port_irq_save();
   cyros_port_time_register_isr_handler(counting_timer, nullptr);
   cyros_port_time_setup(0u);
   cyros_port_time_irq_enable();
   cyros_port_time_arm(cyros_port_time_now());
   settle();
   CYROS_CHECK(mtip_raised());

   std::uint32_t const wrong = registers_wrong_after_a_trap(0u);

   /* Taken inside the routine, and no reschedule run, which would have
    * counted. */
   CYROS_CHECK_EQ(timer_interrupts, 1);
   CYROS_CHECK_EQ(reschedule_calls, before);
   CYROS_CHECK_EQ(wrong, 0u);

   cyros_port_time_teardown();
   cyros_port_irq_restore(token);
}


/* ---------------------------------------------------------------------------
 * Every register survives a trap that switches away and back
 *
 * Here the test is the kernel: the hook switches the boot context out of a
 * trap to a second context, which fills every register with its own pattern
 * and switches back by a trap of its own. The boot context's frame is only
 * whole once the trap has stored its kept half, and the boot context is
 * resumed from that frame, so a register either step misses comes back as
 * the poison or as the second context's pattern.
 * ------------------------------------------------------------------------ */

alignas(CYROS_PORT_CONTEXT_ALIGN) std::byte boot_storage[CYROS_PORT_CONTEXT_SIZE];
alignas(CYROS_PORT_CONTEXT_ALIGN) std::byte other_storage[CYROS_PORT_CONTEXT_SIZE];
alignas(CYROS_PORT_STACK_ALIGN) std::byte other_stack[8192];

cyros_port_context* boot_context()  { return reinterpret_cast<cyros_port_context*>(boot_storage); }
cyros_port_context* other_context() { return reinterpret_cast<cyros_port_context*>(other_storage); }

volatile int other_ran = 0;

/* The second context's pattern in every register it may own, then the trap
 * already pending, which switches back to the boot context. Resumed, should
 * it ever be, after that trap. */
extern "C" [[gnu::naked]] void other_registers_then_a_trap()
{
   asm volatile(
      "addi  sp, sp, -64            \n"
      "sw    ra,  0(sp)             \n"
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
      ".irp n, 1,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31 \n"
      "li    x\\n, 0x5A000000 + \\n  \n"
      ".endr                        \n"
      "csrsi mstatus, 8             \n"   /* the pending reschedule: back to the boot context */
      "nop                          \n"
      "csrci mstatus, 8             \n"
      "lw    ra,  0(sp)             \n"
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

/* The second context, each time it is switched in. It starts at baseline, as
 * a new thread does. */
void other_entry(void*)
{
   while (true) {
      other_ran = other_ran + 1;
      (void)cyros_port_irq_save();
      switch_from = other_context();
      switch_to   = boot_context();
      cyros_port_pend_reschedule();
      settle();
      other_registers_then_a_trap();
   }
}

void test_every_register_survives_a_trap_that_switches()
{
   cyros::bench::start("every register survives a trap that switches away and back");

   cyros_port_context_init(other_context(), other_stack, sizeof(other_stack), other_entry, nullptr);

   int const before_calls = reschedule_calls;
   int const before_ran = other_ran;
   cyros_mask_token_t const token = cyros_port_irq_save();
   switch_from = boot_context();
   switch_to   = other_context();
   cyros_port_pend_reschedule();
   settle();
   CYROS_CHECK(msip_raised());

   std::uint32_t const wrong = registers_wrong_after_a_trap(0u);

   /* There and back: two reschedules, the other context run once between. */
   CYROS_CHECK_EQ(reschedule_calls, before_calls + 2);
   CYROS_CHECK_EQ(other_ran, before_ran + 1);
   CYROS_CHECK_EQ(wrong, 0u);

   cyros_port_irq_restore(token);
}


/* ---------------------------------------------------------------------------
 * Every register survives a fault its handler resumes
 *
 * A fault stores the whole frame at entry and is resumed from it, whatever
 * its handler changed there, so this is the one check of the kept half on a
 * fault's way in and out. The handler below resumes past the instruction.
 * ------------------------------------------------------------------------ */

volatile int faults_expected = 0;
volatile int faults_taken = 0;

void test_every_register_survives_a_fault_its_handler_resumes()
{
   cyros::bench::start("every register survives a fault its handler resumes");

   cyros_mask_token_t const token = cyros_port_irq_save();
   faults_expected = 1;

   std::uint32_t const wrong = registers_wrong_after_a_trap(1u);

   CYROS_CHECK_EQ(faults_taken, 1);
   CYROS_CHECK_EQ(faults_expected, 0);
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


/* This image's fault handler, replacing the port's weak default for the whole
 * image. It resumes past the illegal instruction the register check takes,
 * only when a test expects one, and fails the run on any other fault. */
extern "C" void cyros_riscv_fault_handler(std::uint32_t mcause, std::uint32_t* frame)
{
   if (faults_expected == 0 || mcause != 2u) {
      cyros::bench::print("\n*** unexpected fault, mcause ");
      cyros::bench::print_hex(mcause);
      cyros::bench::print(" mepc ");
      cyros::bench::print_hex(frame[0]);
      cyros::bench::print("\n");
      cyros::bench::host_exit(3u);
   }
   faults_expected = faults_expected - 1;
   faults_taken = faults_taken + 1;
   frame[0] = frame[0] + 4u;
}


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
   test_every_register_survives_a_trap_that_reschedules();
   test_every_register_survives_an_interrupt_that_asks_for_no_reschedule();
   test_every_register_survives_a_trap_that_switches();
   test_every_register_survives_a_fault_its_handler_resumes();

   test_core_identity_and_hints();
   test_tls_pointer_is_the_thread_pointer();
   test_timestamp_moves();

   /* Exactly the seven reschedules the tests above asked for. */
   cyros::bench::start("no reschedule beyond the ones asked for");
   CYROS_CHECK_EQ(reschedule_calls, 7);

   cyros::bench::finish();
}
