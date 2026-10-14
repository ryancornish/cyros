/**
 * @file test_sparc_port.cpp
 * @brief Port-contract test for the sparcv8 core layer. Runs on QEMU's
 *        leon3_generic.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: port.h's interrupt and preemption control and its reschedule
 *          delivery, as the sparcv8 layer implements them on PSR.PIL, the
 *          IRQMP's force register and the trap path's frame and windows.
 * Trusts:  the bench harness and the board's console. Nothing in cyros above
 *          the port.
 * Proves:  that a mask token restores the PREVIOUS state of its own grade
 *          only, so the two compose in any order, including overlapping
 *          ones; that a preempt-disable masks the reschedule and not a
 *          device; that a reschedule pended under either waits for the unmask
 *          and then runs once, masked; that a yield runs it synchronously;
 *          that every register, Y and the condition codes survive a trap, a
 *          yield, a trap taken in the invalid window and a handler deep enough
 *          to spill the interrupted window; that `ta 3` flushes and every
 *          window comes back; and that a fault and a device interrupt reach
 *          their hooks.
 *
 *
 * WHY A LAYER-0 TEST CAN TAKE A RESCHEDULE HERE
 * =============================================
 * As on RISC-V and the R52: before cyros_port_start_first there is no
 * interrupt stack, and the trap path stays on the stack it arrives on. So a
 * reschedule pended from this test's main is a real trap through the real
 * entry and exit, and the handler it reaches is this test's counter. The
 * register checks are the ones that matter most: the entry and exit are hand
 * written, and one wrong offset still runs, still returns, and quietly
 * replaces a register, or loads a window from the wrong place only when the
 * register file happens to be full.
 */

#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <cstddef>
#include <cstdint>

/* The port's trap table (sparcv8.hpp), which the board points TBR at. */
extern "C" std::uint32_t const cyros_sparc_trap_table[];

namespace
{

/* Architectural state, read here rather than through the port's private
 * header. A contract test that reads the port's own view of the hardware
 * proves only that the port agrees with itself. QEMU's leon3_generic puts
 * the IRQMP at 0x80000200. */
constexpr std::uintptr_t irqmp_force0 = 0x80000200u + 0x80u;
constexpr std::uintptr_t irqmp_mask0  = 0x80000200u + 0x40u;
constexpr std::uint32_t  device_level = 5u;   /* nothing on QEMU uses it */

std::uint32_t read_psr() { std::uint32_t v; asm volatile("rd %%psr, %0" : "=r"(v) :: "memory"); return v; }
std::uint32_t read_wim() { std::uint32_t v; asm volatile("rd %%wim, %0" : "=r"(v) :: "memory"); return v; }
std::uint32_t read_tbr() { std::uint32_t v; asm volatile("rd %%tbr, %0" : "=r"(v) :: "memory"); return v; }
std::uint32_t read_g7()  { std::uint32_t v; asm volatile("mov %%g7, %0" : "=r"(v) :: "memory"); return v; }
std::uint32_t pil()      { return (read_psr() >> 8) & 0xFu; }

/* PIL 0, as a thread runs: the bench's "enable_irq". */
void open_pil()
{
   asm volatile("rd %%psr, %%g1 \n andn %%g1, 0xf00, %%g1 \n wr %%g1, %%psr \n nop \n nop \n nop"
                ::: "g1", "memory", "cc");
}

volatile std::uint32_t& reg(std::uintptr_t address) { return *reinterpret_cast<volatile std::uint32_t*>(address); }

bool irq_masked()   { return pil() >= 14u; }
/* A level is taken when it is above PIL. */
bool lets_through(std::uint32_t level) { return level > pil(); }
bool preempt_open() { return lets_through(1u); }
bool reschedule_pending() { return (reg(irqmp_force0) & (1u << 1)) != 0u; }

volatile int reschedule_calls = 0;
volatile std::uint32_t handler_pil = 0;
volatile std::uintptr_t handler_sp = 0;

/* How deep the handler recurses before it returns, to spill the interrupted
 * window when asked to (the exit's hand-made underflow). */
volatile int handler_depth = 0;

[[gnu::noinline]] int recurse(int n)
{
   volatile int local = n;
   return n == 0 ? 0 : recurse(n - 1) + local;
}

/* A switch the hook makes itself, as the kernel's reschedule would: set
 * before the reschedule is pended, taken by the next one. */
cyros_port_context* volatile switch_from = nullptr;
cyros_port_context* volatile switch_to   = nullptr;

void counting_reschedule()
{
   reschedule_calls = reschedule_calls + 1;
   handler_pil = pil();
   handler_sp = reinterpret_cast<std::uintptr_t>(cyros_port_get_stack_pointer());
   if (handler_depth > 0) {
      (void)recurse(handler_depth);
   }
   if (switch_to != nullptr) {
      cyros_port_context* const to = switch_to;
      switch_to = nullptr;
      cyros_port_switch(switch_from, to);
   }
}

/* A forced level reaches the core in the next instructions on QEMU. Long
 * enough for that on a real bus, and nothing more is promised. */
void settle()
{
   for (int i = 0; i < 64; ++i) { asm volatile("nop"); }
}


/* ---------------------------------------------------------------------------
 * Initialisation
 * ------------------------------------------------------------------------ */

void test_init_leaves_the_core_masked()
{
   cyros::bench::start("init leaves the core masked, the trap table and windows checked");

   cyros_port_init(counting_reschedule);

   /* Bring-up must not be interruptible: until start_first there is no thread
    * to switch. Masked is both fields, PIL 15. */
   CYROS_CHECK(!cyros_port_interrupts_enabled());
   CYROS_CHECK_EQ(pil(), 15u);

   /* Supervisor, traps on, the board's trap table, one window invalid, and
    * the FPU on in a hard-float image. */
   std::uint32_t const psr = read_psr();
   CYROS_CHECK((psr & (1u << 7)) != 0u);   /* S  */
   CYROS_CHECK((psr & (1u << 5)) != 0u);   /* ET */
   CYROS_CHECK_EQ(read_tbr() & ~0xFFFu, reinterpret_cast<std::uintptr_t>(cyros_sparc_trap_table));
   std::uint32_t const wim = read_wim();
   CYROS_CHECK(wim != 0u && (wim & (wim - 1u)) == 0u);
#if !defined(_SOFT_FLOAT)
   CYROS_CHECK((psr & (1u << 12)) != 0u);  /* EF */
#endif

   /* The reschedule, level 1, routed to this core, and none left pending. */
   CYROS_CHECK((reg(irqmp_mask0) & (1u << 1)) != 0u);
   CYROS_CHECK(!reschedule_pending());
}


/* ---------------------------------------------------------------------------
 * Interrupt masking
 * ------------------------------------------------------------------------ */

void test_irq_token_round_trips_from_enabled()
{
   cyros::bench::start("irq_save/restore from enabled");

   open_pil();
   CYROS_CHECK(cyros_port_interrupts_enabled());

   cyros_mask_token_t const token = cyros_port_irq_save();
   CYROS_CHECK(!cyros_port_interrupts_enabled());
   CYROS_CHECK_EQ(pil(), 14u);

   cyros_port_irq_restore(token);
   CYROS_CHECK(cyros_port_interrupts_enabled());
   CYROS_CHECK_EQ(pil(), 0u);
}

void test_irq_token_restores_to_masked_when_entered_masked()
{
   cyros::bench::start("irq_restore honours an ALREADY-MASKED caller");

   open_pil();
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

   open_pil();
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

   open_pil();
   CYROS_CHECK(preempt_open());
   cyros_mask_token_t const token = cyros_port_preempt_disable();
   CYROS_CHECK(!preempt_open());
   CYROS_CHECK_EQ(pil(), 1u);
   cyros_port_preempt_enable(token);
   CYROS_CHECK(preempt_open());
   CYROS_CHECK_EQ(pil(), 0u);
}

void test_preempt_restores_to_disabled_when_entered_disabled()
{
   cyros::bench::start("preempt_enable honours an ALREADY-DISABLED caller");

   open_pil();
   cyros_mask_token_t const outer = cyros_port_preempt_disable();
   cyros_mask_token_t const inner = cyros_port_preempt_disable();
   CYROS_CHECK(!preempt_open());

   cyros_port_preempt_enable(inner);
   CYROS_CHECK(!preempt_open());   /* still disabled by `outer` */

   cyros_port_preempt_enable(outer);
   CYROS_CHECK(preempt_open());
}

void test_preempt_disable_masks_the_reschedule_and_not_a_device()
{
   cyros::bench::start("preempt-disable masks the reschedule, never a device or the timer");

   open_pil();
   cyros_mask_token_t const token = cyros_port_preempt_disable();
   CYROS_CHECK(!lets_through(1u));
   CYROS_CHECK(lets_through(2u));
   CYROS_CHECK(lets_through(device_level));
   CYROS_CHECK(lets_through(7u));   /* QEMU's GPTIMER compare */
   cyros_port_preempt_enable(token);
   CYROS_CHECK(lets_through(1u));
}


/* ---------------------------------------------------------------------------
 * The two facilities are independent, though they share PIL
 * ------------------------------------------------------------------------ */

void test_preempt_disable_does_not_mask_interrupts()
{
   cyros::bench::start("preempt-disable leaves interrupts enabled");

   open_pil();
   cyros_mask_token_t const token = cyros_port_preempt_disable();
   CYROS_CHECK(cyros_port_interrupts_enabled());
   CYROS_CHECK(!irq_masked());
   cyros_port_preempt_enable(token);
}

void test_irq_save_does_not_disturb_preemption_state()
{
   cyros::bench::start("irq masking leaves preemption state alone");

   open_pil();
   cyros_mask_token_t const token = cyros_port_irq_save();
   cyros_port_irq_restore(token);
   CYROS_CHECK(preempt_open());

   cyros_mask_token_t const preempt = cyros_port_preempt_disable();
   cyros_mask_token_t const irq = cyros_port_irq_save();
   CYROS_CHECK_EQ(pil(), 15u);
   cyros_port_irq_restore(irq);
   CYROS_CHECK(!preempt_open());   /* the restore put back ITS field only */
   CYROS_CHECK(!irq_masked());
   cyros_port_preempt_enable(preempt);
   CYROS_CHECK(preempt_open());
}

void test_the_two_compose_nested_in_either_order()
{
   cyros::bench::start("nested irq and preempt sections unwind cleanly, either way round");

   open_pil();
   cyros_mask_token_t const irq_outer     = cyros_port_irq_save();
   cyros_mask_token_t const preempt_inner = cyros_port_preempt_disable();
   CYROS_CHECK(irq_masked());
   CYROS_CHECK(!preempt_open());
   cyros_port_preempt_enable(preempt_inner);
   CYROS_CHECK(irq_masked());
   CYROS_CHECK_EQ(pil(), 14u);
   cyros_port_irq_restore(irq_outer);
   CYROS_CHECK_EQ(pil(), 0u);

   cyros_mask_token_t const preempt_outer = cyros_port_preempt_disable();
   cyros_mask_token_t const irq_inner     = cyros_port_irq_save();
   CYROS_CHECK_EQ(pil(), 15u);
   cyros_port_irq_restore(irq_inner);
   CYROS_CHECK_EQ(pil(), 1u);
   cyros_port_preempt_enable(preempt_outer);
   CYROS_CHECK_EQ(pil(), 0u);
}

void test_the_two_compose_overlapping()
{
   cyros::bench::start("OVERLAPPING irq and preempt sections, the case one PIL value cannot hold");

   /* port.h allows regions that overlap rather than nest. A token that was
    * simply the previous PIL would reopen interrupts here. */
   open_pil();
   cyros_mask_token_t const preempt = cyros_port_preempt_disable();
   cyros_mask_token_t const irq     = cyros_port_irq_save();
   cyros_port_preempt_enable(preempt);          /* not the innermost */
   CYROS_CHECK(irq_masked());
   CYROS_CHECK_EQ(pil(), 14u);
   cyros_port_irq_restore(irq);
   CYROS_CHECK_EQ(pil(), 0u);

   cyros_mask_token_t const irq2     = cyros_port_irq_save();
   cyros_mask_token_t const preempt2 = cyros_port_preempt_disable();
   cyros_port_irq_restore(irq2);                /* not the innermost */
   CYROS_CHECK(!irq_masked());
   CYROS_CHECK_EQ(pil(), 1u);
   cyros_port_preempt_enable(preempt2);
   CYROS_CHECK_EQ(pil(), 0u);
}


/* ---------------------------------------------------------------------------
 * Reschedule delivery, through the real trap path
 * ------------------------------------------------------------------------ */

void test_pend_while_interrupts_masked_waits_for_the_restore()
{
   cyros::bench::start("a reschedule pended under irq_save runs at the restore, once");

   open_pil();
   int const before = reschedule_calls;
   cyros_mask_token_t const token = cyros_port_irq_save();

   cyros_port_pend_reschedule();
   settle();
   CYROS_CHECK_EQ(reschedule_calls, before);   /* held */
   CYROS_CHECK(reschedule_pending());          /* and visibly pending */

   cyros_port_irq_restore(token);
   CYROS_CHECK_EQ(reschedule_calls, before + 1);   /* within the restore's nops */
   CYROS_CHECK(!reschedule_pending());             /* consumed, not left pending */
}

void test_pend_while_preemption_disabled_waits_for_the_enable()
{
   cyros::bench::start("a reschedule pended under preempt_disable runs at the enable, once");

   open_pil();
   int const before = reschedule_calls;
   cyros_mask_token_t const token = cyros_port_preempt_disable();

   cyros_port_pend_reschedule();
   settle();
   CYROS_CHECK_EQ(reschedule_calls, before);
   CYROS_CHECK(reschedule_pending());

   cyros_port_preempt_enable(token);
   CYROS_CHECK_EQ(reschedule_calls, before + 1);
   CYROS_CHECK(!reschedule_pending());
}

void test_pend_at_baseline_runs_promptly()
{
   cyros::bench::start("a reschedule pended at baseline runs promptly, masked, below this frame");

   open_pil();
   int const before = reschedule_calls;
   std::uintptr_t const here = reinterpret_cast<std::uintptr_t>(cyros_port_get_stack_pointer());
   cyros_port_pend_reschedule();
   settle();
   CYROS_CHECK_EQ(reschedule_calls, before + 1);

   /* Handlers run at PIL 15. Before the first thread there is no interrupt
    * stack, so this one ran on the stack it arrived on, below the frame the
    * entry stored under this function's sp. */
   CYROS_CHECK_EQ(handler_pil, 15u);
   CYROS_CHECK(handler_sp != 0u);
   CYROS_CHECK(handler_sp + 80u < here);
   CYROS_CHECK_EQ(pil(), 0u);   /* and the exit put PIL back */
}

void test_yield_is_synchronous()
{
   cyros::bench::start("thread_yield runs the reschedule before it returns");

   open_pil();
   int const before = reschedule_calls;
   cyros_port_thread_yield();
   /* No settle: the strong guarantee is that it already happened. */
   CYROS_CHECK_EQ(reschedule_calls, before + 1);
   CYROS_CHECK_EQ(pil(), 0u);
}

} // namespace


/* ---------------------------------------------------------------------------
 * Every register, Y and the condition codes survive a trap
 *
 * Loads a pattern into the globals, the window's outs (but o6, the sp),
 * locals (but l7) and ins (but i6 and i7, its frame pointer and return
 * address), and Y, and opens PIL with all four condition codes set: either
 * with a reschedule pending, which is taken there, or followed by `ta 9`, a
 * yield. Then it stores what came back for C to compare. In assembly, so the
 * compiler holds nothing of its own in them. The PSR it opens with is built
 * from its own `rd %psr`, because a PSR built elsewhere carries another
 * window's CWP. %l7 holds it. Called through call_at_depth to put the trap
 * where the register file is full.
 *
 * out[n]: 1-7 g1-g7, 8-15 o0-o7, 16-23 l0-l7, 24-31 i0-i7, 32 Y, 33 PSR.
 * ------------------------------------------------------------------------ */

extern "C" void sparc_registers_after_a_trap(std::uint32_t* out);
extern "C" void sparc_registers_after_a_yield(std::uint32_t* out);

#define SPARC_REGISTERS_LOAD                                                    \
   "   save  %sp, -256, %sp                                     \n"             \
   "   st    %i0, [%sp + 96]                                    \n"             \
   "   st    %g5, [%sp + 108]                                   \n"             \
   "   st    %g6, [%sp + 112]                                   \n"             \
   "   st    %g7, [%sp + 116]                                   \n"             \
   "   rd    %psr, %l7                                          \n"             \
   "   andn  %l7, 0xf00, %l7                                    \n"             \
   "   set   0x00f00000, %g1                                    \n"             \
   "   or    %l7, %g1, %l7                                      \n"             \
   "   set   0xA500003F, %g1                                    \n"             \
   "   wr    %g1, %y                                            \n"             \
   ".irp n, 1,2,3,4,5,6,7                                       \n"             \
   "   set   0xA5000000 + \\n, %g\\n                            \n"             \
   ".endr                                                       \n"             \
   ".irp n, 0,1,2,3,4,5,7                                       \n"             \
   "   set   0xA5000008 + \\n, %o\\n                            \n"             \
   ".endr                                                       \n"             \
   ".irp n, 0,1,2,3,4,5,6                                       \n"             \
   "   set   0xA5000010 + \\n, %l\\n                            \n"             \
   ".endr                                                       \n"             \
   ".irp n, 0,1,2,3,4,5                                         \n"             \
   "   set   0xA5000018 + \\n, %i\\n                            \n"             \
   ".endr                                                       \n"             \
   "   wr    %l7, %psr                                          \n"             \
   "   nop                                                      \n"             \
   "   nop                                                      \n"             \
   "   nop                                                      \n"

#define SPARC_REGISTERS_STORE                                                   \
   ".irp n, 1,2,3,4,5,6,7                                       \n"             \
   "   st    %g\\n, [%sp + 120 + 4 * \\n]                       \n"             \
   ".endr                                                       \n"             \
   ".irp n, 0,1,2,3,4,5,6,7                                     \n"             \
   "   st    %o\\n, [%sp + 152 + 4 * \\n]                       \n"             \
   "   st    %l\\n, [%sp + 184 + 4 * \\n]                       \n"             \
   "   st    %i\\n, [%sp + 216 + 4 * \\n]                       \n"             \
   ".endr                                                       \n"             \
   "   rd    %y, %l0                                            \n"             \
   "   st    %l0, [%sp + 248]                                   \n"             \
   "   rd    %psr, %l0                                          \n"             \
   "   st    %l0, [%sp + 252]                                   \n"             \
   "   or    %l0, 0xe00, %l0                                    \n"             \
   "   wr    %l0, %psr                                          \n"             \
   "   nop                                                      \n"             \
   "   nop                                                      \n"             \
   "   nop                                                      \n"             \
   "   ld    [%sp + 108], %g5                                   \n"             \
   "   ld    [%sp + 112], %g6                                   \n"             \
   "   ld    [%sp + 116], %g7                                   \n"             \
   "   ld    [%sp + 96], %l1                                    \n"             \
   "   add   %sp, 120, %l2                                      \n"             \
   "   mov   0, %l3                                             \n"             \
   "1: ld    [%l2 + %l3], %l4                                   \n"             \
   "   st    %l4, [%l1 + %l3]                                   \n"             \
   "   add   %l3, 4, %l3                                        \n"             \
   "   cmp   %l3, 136                                           \n"             \
   "   bne   1b                                                 \n"             \
   "    nop                                                     \n"             \
   "   ret                                                      \n"             \
   "   restore                                                  \n"

asm(
   ".pushsection .text.sparc_registers, \"ax\"                  \n"
   ".global sparc_registers_after_a_trap                        \n"
   ".type   sparc_registers_after_a_trap, #function             \n"
   "sparc_registers_after_a_trap:                               \n"
   SPARC_REGISTERS_LOAD
   SPARC_REGISTERS_STORE
   ".size sparc_registers_after_a_trap, . - sparc_registers_after_a_trap \n"
   ".global sparc_registers_after_a_yield                       \n"
   ".type   sparc_registers_after_a_yield, #function            \n"
   "sparc_registers_after_a_yield:                              \n"
   SPARC_REGISTERS_LOAD
   "   ta    9                                                  \n"
   SPARC_REGISTERS_STORE
   ".size sparc_registers_after_a_yield, . - sparc_registers_after_a_yield \n"
   ".popsection                                                 \n");

namespace
{

using register_routine = void (*)(std::uint32_t*);

/* Calls `routine` `depth` calls down, each with a live local, so with depth
 * 8 or more every window is in use and the trap's window is the invalid one.
 * Returns the sum of the locals on the way back, which only comes out right
 * if every window came back. */
[[gnu::noinline]] std::uint32_t call_at_depth(int depth, register_routine routine, std::uint32_t* out)
{
   volatile std::uint32_t local = 0x1000u + static_cast<std::uint32_t>(depth);
   if (depth == 0) {
      routine(out);
      return local;
   }
   return call_at_depth(depth - 1, routine, out) + local;
}

std::uint32_t expected_sum(int depth)
{
   std::uint32_t sum = 0;
   for (int d = 0; d <= depth; ++d) { sum += 0x1000u + static_cast<std::uint32_t>(d); }
   return sum;
}

std::uint32_t registers_wrong(std::uint32_t const (&out)[34])
{
   std::uint32_t wrong = 0;
   auto expect = [&](std::uint32_t index, std::uint32_t value, char const* name) {
      if (out[index] != value) {
         ++wrong;
         cyros::bench::print("  wrong: ");
         cyros::bench::print(name);
         cyros::bench::print_hex(index);
         cyros::bench::print(" = ");
         cyros::bench::print_hex(out[index]);
         cyros::bench::print("\n");
      }
   };
   for (std::uint32_t n = 1; n <= 7; ++n)  { expect(n, 0xA5000000u + n, "g/"); }
   for (std::uint32_t n = 0; n <= 7; ++n)  { if (n != 6) { expect(8 + n, 0xA5000008u + n, "o/"); } }
   for (std::uint32_t n = 0; n <= 6; ++n)  { expect(16 + n, 0xA5000010u + n, "l/"); }
   for (std::uint32_t n = 0; n <= 5; ++n)  { expect(24 + n, 0xA5000018u + n, "i/"); }
   expect(32, 0xA500003Fu, "y/");
   /* l7 held the PSR the routine opened with: the condition codes set. */
   if ((out[23] & 0x00F00000u) != 0x00F00000u) { ++wrong; cyros::bench::print("  wrong: l7\n"); }
   if ((out[33] & 0x00F00000u) != 0x00F00000u) { ++wrong; cyros::bench::print("  wrong: the condition codes\n"); }
   return wrong;
}

void check_registers(char const* what, register_routine routine, int depth, int handler_recursion, bool pend)
{
   cyros::bench::start(what);

   int const before = reschedule_calls;
   cyros_mask_token_t const token = cyros_port_irq_save();
   if (pend) {
      cyros_port_pend_reschedule();
      settle();
      CYROS_CHECK(reschedule_pending());
   }
   handler_depth = handler_recursion;

   std::uint32_t out[34] = {};
   std::uint32_t const sum = call_at_depth(depth, routine, out);
   handler_depth = 0;

   /* The trap must actually have been taken inside the routine, or the check
    * proves nothing. */
   CYROS_CHECK_EQ(reschedule_calls, before + 1);
   CYROS_CHECK_EQ(registers_wrong(out), 0u);
   CYROS_CHECK_EQ(sum, expected_sum(depth));

   cyros_port_irq_restore(token);
}


/* ---------------------------------------------------------------------------
 * A switch from a SHALLOW handler
 *
 * The kernel's reschedule runs deep enough below the trap window that the
 * overflow traps alone spill every window of the outgoing thread before the
 * exit looks, so the exit's own flush of the outgoing thread is idle in every
 * test built on the kernel (its mutant survived them all). Here the test is
 * the kernel: this hook switches at once, a few windows below the trap, so
 * the outgoing thread's windows are still in the register file at the exit,
 * and only the flush puts them where they come back from.
 *
 * The boot context switches to a second context, which churns every window
 * and switches back. The register check and the window sums run across the
 * round trip, from a shallow call and from three calls down.
 * ------------------------------------------------------------------------ */

alignas(CYROS_PORT_CONTEXT_ALIGN) std::byte boot_storage[CYROS_PORT_CONTEXT_SIZE];
alignas(CYROS_PORT_CONTEXT_ALIGN) std::byte other_storage[CYROS_PORT_CONTEXT_SIZE];
alignas(CYROS_PORT_STACK_ALIGN) std::byte other_stack[8192];

cyros_port_context* boot_context()  { return reinterpret_cast<cyros_port_context*>(boot_storage); }
cyros_port_context* other_context() { return reinterpret_cast<cyros_port_context*>(other_storage); }

volatile int other_ran = 0;

[[gnu::noinline]] std::uint32_t churn(int depth)
{
   volatile std::uint32_t local = static_cast<std::uint32_t>(depth) * 0x01010101u;
   return depth == 0 ? local : churn(depth - 1) ^ local;
}

/* The second context: every window reused, then back to the boot context,
 * each time it is switched in. It runs at PIL 0, as a new thread starts. */
void other_entry(void*)
{
   while (true) {
      other_ran = other_ran + 1;
      (void)churn(20);
      cyros_mask_token_t const token = cyros_port_irq_save();
      switch_from = other_context();
      switch_to   = boot_context();
      cyros_port_pend_reschedule();
      cyros_port_irq_restore(token);   /* switched out here, resumed here */
   }
}

void check_shallow_switch(char const* what, int depth)
{
   cyros::bench::start(what);

   int const before_calls = reschedule_calls;
   int const before_ran = other_ran;
   cyros_mask_token_t const token = cyros_port_irq_save();
   switch_from = boot_context();
   switch_to   = other_context();
   cyros_port_pend_reschedule();
   settle();

   std::uint32_t out[34] = {};
   std::uint32_t const sum = call_at_depth(depth, sparc_registers_after_a_trap, out);

   /* There and back: two reschedules, the other context run once between. */
   CYROS_CHECK_EQ(reschedule_calls, before_calls + 2);
   CYROS_CHECK_EQ(other_ran, before_ran + 1);
   CYROS_CHECK_EQ(registers_wrong(out), 0u);
   CYROS_CHECK_EQ(sum, expected_sum(depth));

   cyros_port_irq_restore(token);
}

void test_switch_from_a_shallow_handler()
{
   cyros_port_context_init(other_context(), other_stack, sizeof(other_stack), other_entry, nullptr);
   check_shallow_switch("a switch from a shallow handler: the outgoing windows flushed, all back", 0);
   check_shallow_switch("... from three calls down", 3);
}


/* ---------------------------------------------------------------------------
 * The windows across `ta 3`, GCC's flush
 * ------------------------------------------------------------------------ */

[[gnu::noinline]] std::uint32_t flush_at_depth(int depth)
{
   volatile std::uint32_t local = 0x2000u + static_cast<std::uint32_t>(depth);
   if (depth == 0) {
      std::uint32_t const wim_before = read_wim();
      asm volatile("ta 3" ::: "memory");
      /* Every window but this one is on its stack now, so the one above
       * this is the invalid one. */
      std::uint32_t const cwp = read_psr() & 0x1Fu;
      std::uint32_t const above = (cwp + 1u) % 8u;
      CYROS_CHECK_EQ(read_wim(), 1u << above);
      CYROS_CHECK(wim_before != 0u);
      return local;
   }
   return flush_at_depth(depth - 1) + local;
}

void test_flush_windows()
{
   cyros::bench::start("ta 3 flushes every caller's window, and each comes back");

   std::uint32_t expected = 0;
   for (int d = 0; d <= 5; ++d) { expected += 0x2000u + static_cast<std::uint32_t>(d); }
   CYROS_CHECK_EQ(flush_at_depth(5), expected);
}


/* ---------------------------------------------------------------------------
 * The fault and device hooks
 * ------------------------------------------------------------------------ */

volatile std::uint32_t fault_tt = 0;
volatile std::uint32_t fault_pc = 0;
volatile bool fault_expected = false;
volatile std::uint32_t device_levels = 0;

} // namespace

/* This image's fault handler, replacing the port's weak default for the whole
 * image. It skips the trapping instruction only when the test expects a
 * fault, and fails the run on any other. */
extern "C" void cyros_sparc_fault_handler(std::uint32_t tt, std::uint32_t* frame)
{
   if (!fault_expected) {
      cyros::bench::print("\n*** unexpected trap ");
      cyros::bench::print_hex(tt);
      cyros::bench::print(" at ");
      cyros::bench::print_hex(frame[1]);
      cyros::bench::print("\n");
      cyros::bench::host_exit(3u);
   }
   fault_tt = tt;
   fault_pc = frame[1];
   frame[1] = frame[2];          /* PC  := nPC     */
   frame[2] = frame[2] + 4u;     /* nPC := nPC + 4 */
}

extern "C" void cyros_sparc_device_interrupt(std::uint32_t level)
{
   device_levels = device_levels | (1u << level);
}

namespace
{

void test_a_fault_reaches_the_hook_and_resumes()
{
   cyros::bench::start("a software trap and an illegal instruction reach the fault hook, which resumes");

   open_pil();
   fault_expected = true;
   std::uint32_t pc = 0;
   asm volatile("set 1f, %0 \n 1: ta 0x20" : "=r"(pc) :: "memory");
   CYROS_CHECK_EQ(fault_tt, 0x80u + 0x20u);
   CYROS_CHECK_EQ(fault_pc, pc);

   asm volatile("set 1f, %0 \n 1: unimp 0" : "=r"(pc) :: "memory");
   CYROS_CHECK_EQ(fault_tt, 0x02u);   /* illegal_instruction */
   CYROS_CHECK_EQ(fault_pc, pc);
   fault_expected = false;
   CYROS_CHECK_EQ(pil(), 0u);
}

void test_a_device_level_reaches_its_hook()
{
   cyros::bench::start("a device's level reaches the hook, past a preempt-disable, held by an irq mask");

   reg(irqmp_mask0) = reg(irqmp_mask0) | (1u << device_level);
   open_pil();

   /* Under a preempt-disable a device still runs. */
   device_levels = 0u;
   cyros_mask_token_t const preempt = cyros_port_preempt_disable();
   reg(irqmp_force0) = 1u << device_level;
   settle();
   CYROS_CHECK_EQ(device_levels, 1u << device_level);
   cyros_port_preempt_enable(preempt);

   /* Under an irq mask it waits for the restore. */
   device_levels = 0u;
   cyros_mask_token_t const irq = cyros_port_irq_save();
   reg(irqmp_force0) = 1u << device_level;
   settle();
   CYROS_CHECK_EQ(device_levels, 0u);
   cyros_port_irq_restore(irq);
   CYROS_CHECK_EQ(device_levels, 1u << device_level);

   reg(irqmp_mask0) = reg(irqmp_mask0) & ~(1u << device_level);
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

void test_tls_pointer_is_g7()
{
   cyros::bench::start("TLS pointer round trip, and it is %g7");

   int marker = 0;
   cyros_port_set_tls_pointer(&marker);
   CYROS_CHECK(cyros_port_get_tls_pointer() == &marker);
   CYROS_CHECK_EQ(read_g7(), reinterpret_cast<std::uintptr_t>(&marker));

   cyros_port_set_tls_pointer(nullptr);
   CYROS_CHECK(cyros_port_get_tls_pointer() == nullptr);
}

void test_timestamp_moves()
{
   cyros::bench::start("timestamp moves forward");

   std::uint64_t const first = cyros_port_timestamp();
   for (int i = 0; i < 20000; ++i) { asm volatile("nop"); }
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
   CYROS_CHECK(reschedule_pending());

   cyros_port_init(counting_reschedule);
   CYROS_CHECK(!reschedule_pending());

   open_pil();
   settle();
   CYROS_CHECK_EQ(reschedule_calls, before);
   (void)token;
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("sparcv8 port contract\n\n");

   test_init_leaves_the_core_masked();

   test_irq_token_round_trips_from_enabled();
   test_irq_token_restores_to_masked_when_entered_masked();
   test_irq_nesting_three_deep();

   test_preempt_token_round_trips();
   test_preempt_restores_to_disabled_when_entered_disabled();
   test_preempt_disable_masks_the_reschedule_and_not_a_device();

   test_preempt_disable_does_not_mask_interrupts();
   test_irq_save_does_not_disturb_preemption_state();
   test_the_two_compose_nested_in_either_order();
   test_the_two_compose_overlapping();

   test_pend_while_interrupts_masked_waits_for_the_restore();
   test_pend_while_preemption_disabled_waits_for_the_enable();
   test_pend_at_baseline_runs_promptly();
   test_yield_is_synchronous();

   check_registers("every register, Y and the codes survive an interrupt",
                   sparc_registers_after_a_trap, 0, 0, true);
   check_registers("... taken in the invalid window, every window in use",
                   sparc_registers_after_a_trap, 10, 0, true);
   check_registers("... with a handler deep enough to spill the interrupted window",
                   sparc_registers_after_a_trap, 0, 12, true);
   check_registers("... both at once",
                   sparc_registers_after_a_trap, 10, 12, true);
   check_registers("every register, Y and the codes survive a yield",
                   sparc_registers_after_a_yield, 0, 0, false);
   check_registers("... a yield with every window in use",
                   sparc_registers_after_a_yield, 10, 12, false);

   test_switch_from_a_shallow_handler();
   test_flush_windows();
   test_a_fault_reaches_the_hook_and_resumes();
   test_a_device_level_reaches_its_hook();

   test_core_identity_and_hints();
   test_tls_pointer_is_g7();
   test_timestamp_moves();
   test_init_drops_a_reschedule_left_pending();

   /* Exactly the reschedules the tests above asked for: four from the
    * delivery tests, six from the register checks, four from the switches. */
   cyros::bench::start("no reschedule beyond the ones asked for");
   CYROS_CHECK_EQ(reschedule_calls, 14);

   cyros::bench::finish();
}
