/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_sparc_nesting.cpp
 * @brief Nested interrupts on the sparcv8 core layer. Runs on QEMU's
 *        leon3_generic.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the trap path's nesting: each handler at its own level, the core's
 *          count of traps in progress, and where a nested trap's frame and
 *          handler go.
 * Trusts:  test_sparc_port (the masks, delivery, and the frame and windows of
 *          a trap that does not nest), and for the timer the MTIME time
 *          source it drives through port_mcu.h. Nothing above the port.
 * Proves:  that a handler runs at its own level, so a higher level nests
 *          inside it while its own and a lower one wait; that a nested
 *          handler runs on the interrupt stack below the handler it
 *          interrupted, and nothing is written to the thread's stack below
 *          its frame; that every register and window of an interrupted
 *          handler survives a nested interrupt, one taken in the invalid
 *          window included; that a reschedule pended in a nested handler
 *          waits for the outermost to return, then runs once; that the
 *          reschedule and the yield run at level 1, so a device nests into
 *          the kernel's reschedule; that a device nested after a switch
 *          leaves the switch in place; that the target's timer body runs
 *          masked, so a device above the timer's level waits for it; and
 *          that a level 15 pending as a trap is taken, which nests in that
 *          trap's entry before its handler, leaves it counted as the
 *          outermost.
 *
 *
 * WHY LAYER 0
 * ===========
 * The interrupt stack exists once cyros_port_start_first has run a context,
 * and that is a port call. So this test starts one itself, with a counting
 * hook for the reschedule as test_sparc_port's, and every check runs in it:
 * the boot stack is then the interrupt stack, as under the kernel.
 *
 * The levels are ones nothing on QEMU's leon3_generic uses (the APBUART is 3,
 * GPTIMER 6 and 7), raised through core 0's force register.
 */

#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <cstddef>
#include <cstdint>

namespace
{

constexpr std::uintptr_t irqmp_mask0  = 0x80000200u + 0x40u;
constexpr std::uintptr_t irqmp_force0 = 0x80000200u + 0x80u;

constexpr std::uint32_t reschedule_level = 1u;
constexpr std::uint32_t lower_level      = 4u;
constexpr std::uint32_t outer_level      = 5u;
constexpr std::uint32_t upper_level      = 9u;
constexpr std::uint32_t nmi_level        = 15u;

std::uint32_t read_psr() { std::uint32_t v; asm volatile("rd %%psr, %0" : "=r"(v) :: "memory"); return v; }
std::uint32_t pil()      { return (read_psr() >> 8) & 0xFu; }

volatile std::uint32_t& reg(std::uintptr_t address) { return *reinterpret_cast<volatile std::uint32_t*>(address); }

void force(std::uint32_t level)   { reg(irqmp_force0) = 1u << level; }
bool pending(std::uint32_t level) { return (reg(irqmp_force0) & (1u << level)) != 0u; }

std::uintptr_t sp_now() { return reinterpret_cast<std::uintptr_t>(cyros_port_get_stack_pointer()); }

/* A forced level reaches the core in the next instructions on QEMU. */
void settle()
{
   for (int i = 0; i < 64; ++i) { asm volatile("nop"); }
}

/* The context this test runs in, and a second one to switch to. */
alignas(CYROS_PORT_CONTEXT_ALIGN) std::byte main_storage[CYROS_PORT_CONTEXT_SIZE];
alignas(CYROS_PORT_CONTEXT_ALIGN) std::byte other_storage[CYROS_PORT_CONTEXT_SIZE];
alignas(CYROS_PORT_STACK_ALIGN) std::byte main_stack[16384];
alignas(CYROS_PORT_STACK_ALIGN) std::byte other_stack[8192];

cyros_port_context* main_context()  { return reinterpret_cast<cyros_port_context*>(main_storage); }
cyros_port_context* other_context() { return reinterpret_cast<cyros_port_context*>(other_storage); }

/* The boot stack's sp just before start_first, which makes the stack below it
 * the interrupt stack. These handlers are shallow, so 16 KB below it bounds
 * where they can be. */
std::uintptr_t boot_sp = 0;

bool on_interrupt_stack(std::uintptr_t sp)
{
   return sp < boot_sp && sp > boot_sp - 16384u;
}

bool on_main_stack(std::uintptr_t sp)
{
   auto const base = reinterpret_cast<std::uintptr_t>(main_stack);
   return sp >= base && sp < base + sizeof(main_stack);
}

/* What each scenario makes the hooks do. */
enum class scenario
{
   none,
   order,
   stacks,
   registers,
   pend_in_nested,
   in_reschedule,
   after_switch,
   nmi_at_entry,
};

volatile scenario active = scenario::none;

/* The hooks' record. An entry is 0xE000 | level << 8 | the PIL it ran at, a
 * return 0xF000 | level << 8, and a handler's own mark 0xA000 | level << 8. */
constexpr std::uint32_t entered(std::uint32_t level, std::uint32_t at) { return 0xE000u | (level << 8) | at; }
constexpr std::uint32_t left(std::uint32_t level)                      { return 0xF000u | (level << 8); }
constexpr std::uint32_t marked(std::uint32_t level)                    { return 0xA000u | (level << 8); }

constexpr std::uint32_t max_events = 16u;
volatile std::uint32_t events[max_events] = {};
volatile std::uint32_t event_count = 0;

void note(std::uint32_t event)
{
   if (event_count < max_events) {
      events[event_count] = event;
   }
   event_count = event_count + 1u;
}

void clear_events()
{
   event_count = 0u;
   for (std::uint32_t i = 0; i < max_events; ++i) { events[i] = 0u; }
}

volatile std::uint32_t taken[16] = {};
volatile std::uintptr_t handler_sp[16] = {};

volatile int reschedule_calls = 0;
volatile std::uint32_t reschedule_pil = 0;

/* A switch the hook makes, as the kernel's reschedule would. */
cyros_port_context* volatile switch_from = nullptr;
cyros_port_context* volatile switch_to   = nullptr;

[[gnu::noinline]] int recurse(int n)
{
   volatile int local = n;
   return n == 0 ? 0 : recurse(n - 1) + local;
}

} // namespace


/* ---------------------------------------------------------------------------
 * Every register of a HANDLER survives a nested interrupt
 *
 * test_sparc_port's register check, opening PIL to the outer handler's level
 * rather than to 0, so a higher level already pending nests there. Loads a
 * pattern into the globals, the window's outs (but o6), locals (but l7) and
 * ins (but i6 and i7), and Y, opens PIL with all four condition codes set,
 * and stores what came back. %l7 holds the PSR it opened with, built from its
 * own `rd %psr`. The store leaves PIL's interrupt field set. The thread's
 * variant opens to 0, for a reschedule pending there.
 *
 * out[n]: 1-7 g1-g7, 8-15 o0-o7, 16-23 l0-l7, 24-31 i0-i7, 32 Y, 33 PSR.
 * ------------------------------------------------------------------------ */

extern "C" void sparc_registers_at_pil0(std::uint32_t* out);
extern "C" void sparc_registers_at_outer_level(std::uint32_t* out);

#define SPARC_REGISTERS_ROUTINE(name, pil_bits)                                 \
   ".global " name "                                            \n"             \
   ".type   " name ", #function                                 \n"             \
   name ":                                                      \n"             \
   "   save  %sp, -256, %sp                                     \n"             \
   "   st    %i0, [%sp + 96]                                    \n"             \
   "   st    %g5, [%sp + 108]                                   \n"             \
   "   st    %g6, [%sp + 112]                                   \n"             \
   "   st    %g7, [%sp + 116]                                   \n"             \
   "   rd    %psr, %l7                                          \n"             \
   "   andn  %l7, 0xf00, %l7                                    \n"             \
   "   or    %l7, " pil_bits ", %l7                             \n"             \
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
   "   nop                                                      \n"             \
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
   "   restore                                                  \n"             \
   ".size " name ", . - " name "                                \n"

asm(
   ".pushsection .text.sparc_registers, \"ax\"                  \n"
   SPARC_REGISTERS_ROUTINE("sparc_registers_at_pil0", "0")
   SPARC_REGISTERS_ROUTINE("sparc_registers_at_outer_level", "0x500")
   ".popsection                                                 \n");

/* ---------------------------------------------------------------------------
 * A trap entered with a level 15 already pending
 *
 * The bench cannot make an interrupt arrive at a chosen instruction, and on
 * QEMU a forced level is taken at the next one, before any trap instruction
 * after the store. So this plays the hardware's half of a trap itself: traps
 * off with PS := S, the level raised (held: nothing is taken while ET is 0),
 * down one window without the WIM check, the trapped PC and nPC in %l1 and
 * %l2, and a jump to the port's trap table entry for `ta 0x21`. From there it
 * is the real entry, and the level 15 is taken the moment it turns traps on,
 * before the dispatch. The fault hook skips the "trapping" nop. Eight windows,
 * as everywhere in these tests.
 * ------------------------------------------------------------------------ */

extern "C" void sparc_trap_with_level_15_pending(std::uintptr_t force_register);

asm(
   ".pushsection .text.sparc_trap_with_level_15_pending, \"ax\"  \n"
   ".global sparc_trap_with_level_15_pending                    \n"
   ".type   sparc_trap_with_level_15_pending, #function         \n"
   "sparc_trap_with_level_15_pending:                           \n"
   "   save  %sp, -96, %sp                                      \n"
   "   rd    %psr, %l0                                          \n"
   "   andn  %l0, 0x20, %l1                                     \n"   /* ET 0 */
   "   or    %l1, 0x40, %l1                                     \n"   /* PS := S (1) */
   "   wr    %l1, %psr                                          \n"
   "   nop                                                      \n"
   "   nop                                                      \n"
   "   nop                                                      \n"
   "   set   0x8000, %l2                                        \n"
   "   st    %l2, [%i0]                                         \n"   /* level 15, held */
   "   and   %l1, 0x1f, %l3                                     \n"
   "   cmp   %l3, 0                                             \n"
   "   bne,a 1f                                                 \n"
   "    sub  %l1, 1, %l1                                        \n"   /* CWP - 1 */
   "   or    %l1, 7, %l1                                        \n"   /* or 0 - 1, mod 8 */
   "1: wr    %l1, %psr                                          \n"
   "   nop                                                      \n"
   "   nop                                                      \n"
   "   nop                                                      \n"
   "   set   2f, %l1                                            \n"   /* the trap window's PC */
   "   add   %l1, 4, %l2                                        \n"   /* and nPC */
   "   set   cyros_sparc_trap_table + 0xA1 * 16, %l4            \n"
   "   jmp   %l4                                                \n"
   "    nop                                                     \n"
   "2: nop                                                      \n"   /* skipped by the hook */
   "   ret                                                      \n"
   "   restore                                                  \n"
   ".size sparc_trap_with_level_15_pending, . - sparc_trap_with_level_15_pending \n"
   ".popsection                                                 \n");

namespace
{

using register_routine = void (*)(std::uint32_t*);

/* Calls `routine` `depth` calls down, each with a live local, so with depth
 * 8 or more every window is in use and the trap's window is the invalid one.
 * The sum of the locals comes out right only if every window came back. */
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
   auto expect = [&](std::uint32_t index, std::uint32_t value) {
      if (out[index] != value) {
         ++wrong;
         cyros::bench::print("  wrong: out[");
         cyros::bench::print_hex(index);
         cyros::bench::print("] = ");
         cyros::bench::print_hex(out[index]);
         cyros::bench::print("\n");
      }
   };
   for (std::uint32_t n = 1; n <= 7; ++n) { expect(n, 0xA5000000u + n); }
   for (std::uint32_t n = 0; n <= 7; ++n) { if (n != 6) { expect(8 + n, 0xA5000008u + n); } }
   for (std::uint32_t n = 0; n <= 6; ++n) { expect(16 + n, 0xA5000010u + n); }
   for (std::uint32_t n = 0; n <= 5; ++n) { expect(24 + n, 0xA5000018u + n); }
   expect(32, 0xA500003Fu);
   if ((out[23] & 0x00F00000u) != 0x00F00000u) { ++wrong; cyros::bench::print("  wrong: l7\n"); }
   if ((out[33] & 0x00F00000u) != 0x00F00000u) { ++wrong; cyros::bench::print("  wrong: the condition codes\n"); }
   return wrong;
}

/* The register check inside the outer handler: what it is asked, what it
 * found. */
volatile int handler_call_depth = 0;
volatile int nested_recursion = 0;
volatile std::uint32_t handler_registers_wrong = 0xFFFFFFFFu;
volatile std::uint32_t handler_sum = 0;
volatile std::uint32_t upper_taken_in_routine = 0;

void registers_in_handler()
{
   /* Masked, so the upper level waits, pending, for the routine to open PIL
    * to this handler's level inside its window. */
   cyros_mask_token_t const token = cyros_port_irq_save();
   force(upper_level);
   settle();
   std::uint32_t const before = taken[upper_level];

   std::uint32_t out[34] = {};
   handler_sum = call_at_depth(handler_call_depth, sparc_registers_at_outer_level, out);
   upper_taken_in_routine = taken[upper_level] - before;
   handler_registers_wrong = registers_wrong(out);

   cyros_port_irq_restore(token);
}

/* The reschedule pended by a nested handler, as the outer one saw it. */
volatile int calls_seen_by_outer = -1;
volatile bool pending_seen_by_outer = false;

/* Devices run inside the reschedule's handler. */
volatile std::uint32_t nested_in_reschedule = 0;
volatile std::uint32_t nested_after_switch = 0;

/* The fault and level-15 record. */
volatile std::uint32_t nmi_count = 0;
volatile std::uintptr_t nmi_sp = 0;
volatile std::uint32_t nmi_count_at_fault = 0xFFFFFFFFu;
volatile std::uintptr_t fault_sp = 0;
volatile std::uintptr_t nested_fault_sp = 0;
volatile bool fault_sentinel_ok = false;

} // namespace


/* The device hook: what the active scenario asks of each level. */
extern "C" void cyros_sparc_device_interrupt(std::uint32_t level)
{
   note(entered(level, pil()));
   taken[level] = taken[level] + 1u;
   handler_sp[level] = sp_now();

   switch (active) {
      case scenario::order:
         /* The first time only: its own level and a lower one wait, the
          * upper one nests at once. */
         if (level == outer_level && taken[outer_level] == 1u) {
            force(lower_level);
            force(outer_level);
            force(upper_level);
            settle();
            note(marked(level));
         }
         break;

      case scenario::stacks:
         if (level == outer_level) {
            force(upper_level);
            settle();
         }
         break;

      case scenario::registers:
         if (level == outer_level) {
            registers_in_handler();
         } else if (level == upper_level) {
            (void)recurse(nested_recursion);
         }
         break;

      case scenario::pend_in_nested:
         if (level == outer_level) {
            force(upper_level);
            settle();
            calls_seen_by_outer = reschedule_calls;
            pending_seen_by_outer = pending(reschedule_level);
         } else if (level == upper_level) {
            cyros_port_pend_reschedule();
         }
         break;

      case scenario::nmi_at_entry:
         if (level == nmi_level) {
            nmi_count = nmi_count + 1u;
            nmi_sp = sp_now();
         }
         break;

      case scenario::none:
      case scenario::in_reschedule:
      case scenario::after_switch:
         break;
   }

   note(left(level));
}

/* The fault hook: `ta 0x21` from the thread, and `ta 0x22` from inside its
 * handler. Skips each ta. Any other fault fails the run. */
extern "C" void cyros_sparc_fault_handler(std::uint32_t tt, std::uint32_t* frame)
{
   if (active != scenario::nmi_at_entry || (tt != 0xA1u && tt != 0xA2u)) {
      cyros::bench::print("\n*** unexpected trap ");
      cyros::bench::print_hex(tt);
      cyros::bench::print(" at ");
      cyros::bench::print_hex(frame[1]);
      cyros::bench::print("\n");
      cyros::bench::host_exit(3u);
   }
   if (tt == 0xA1u) {
      volatile std::uint32_t sentinel = 0x5E17E1u;
      nmi_count_at_fault = nmi_count;
      fault_sp = sp_now();
      asm volatile("ta 0x22" ::: "memory");
      fault_sentinel_ok = sentinel == 0x5E17E1u;
   } else {
      nested_fault_sp = sp_now();
   }
   frame[1] = frame[2];          /* PC  := nPC     */
   frame[2] = frame[2] + 4u;     /* nPC := nPC + 4 */
}

namespace
{

void counting_reschedule()
{
   reschedule_calls = reschedule_calls + 1;
   reschedule_pil = pil();

   if (active == scenario::in_reschedule) {
      std::uint32_t const before = taken[upper_level];
      force(upper_level);
      settle();
      nested_in_reschedule = nested_in_reschedule + (taken[upper_level] - before);
   }

   if (switch_to != nullptr) {
      cyros_port_context* const to = switch_to;
      switch_to = nullptr;
      cyros_port_switch(switch_from, to);
      /* Once: the switch back must not nest again. */
      if (active == scenario::after_switch) {
         active = scenario::none;
         std::uint32_t const before = taken[upper_level];
         force(upper_level);
         settle();
         nested_after_switch = nested_after_switch + (taken[upper_level] - before);
      }
   }
}


/* ---------------------------------------------------------------------------
 * The checks, from the started context
 * ------------------------------------------------------------------------ */

void test_started_with_an_interrupt_stack()
{
   cyros::bench::start("the started context runs at PIL 0, on its own stack, below the boot stack");

   CYROS_CHECK_EQ(pil(), 0u);
   CYROS_CHECK(on_main_stack(sp_now()));
   CYROS_CHECK(boot_sp != 0u);
}

void test_a_higher_level_nests_and_its_own_and_lower_wait()
{
   cyros::bench::start("a handler runs at its own level: a higher one nests, its own and a lower one wait");

   clear_events();
   for (std::uint32_t i = 0; i < 16u; ++i) { taken[i] = 0u; }
   active = scenario::order;
   force(outer_level);
   settle();
   settle();
   active = scenario::none;

   std::uint32_t const expected[] = {
      entered(outer_level, outer_level),
      entered(upper_level, upper_level),   /* nested at once */
      left(upper_level),
      marked(outer_level),                 /* the outer handler goes on */
      left(outer_level),
      entered(outer_level, outer_level),   /* its own level, after it */
      left(outer_level),
      entered(lower_level, lower_level),   /* then the lower one */
      left(lower_level),
   };
   constexpr std::uint32_t count = sizeof(expected) / sizeof(expected[0]);
   CYROS_CHECK_EQ(event_count, count);
   std::uint32_t wrong = 0;
   for (std::uint32_t i = 0; i < count; ++i) {
      if (events[i] != expected[i]) { ++wrong; }
   }
   if (wrong != 0u) {
      cyros::bench::print("  events:");
      for (std::uint32_t i = 0; i < event_count && i < max_events; ++i) {
         cyros::bench::print(" ");
         cyros::bench::print_hex(events[i]);
      }
      cyros::bench::print("\n");
   }
   CYROS_CHECK_EQ(wrong, 0u);
   CYROS_CHECK_EQ(pil(), 0u);
}

/* Words below this frame, past room for the outermost trap's frame and the
 * calls this test makes, painted and checked. */
constexpr std::uint32_t paint = 0xDEC0DE5Au;
constexpr std::uintptr_t paint_margin = 1024u;

[[gnu::noinline]] std::uint32_t* paint_below(std::uintptr_t here)
{
   auto* const word = reinterpret_cast<std::uint32_t*>(main_stack);
   std::uintptr_t const end = (here - paint_margin) & ~static_cast<std::uintptr_t>(3u);
   auto* const last = reinterpret_cast<std::uint32_t*>(end);
   for (std::uint32_t* p = word; p < last; ++p) { *p = paint; }
   return last;
}

[[gnu::noinline]] std::uint32_t paint_disturbed(std::uint32_t const* last)
{
   std::uint32_t disturbed = 0;
   for (auto const* p = reinterpret_cast<std::uint32_t const*>(main_stack); p < last; ++p) {
      if (*p != paint) { ++disturbed; }
   }
   return disturbed;
}

void test_a_nested_handler_runs_below_the_one_it_interrupted()
{
   cyros::bench::start("a nested handler runs on the interrupt stack, below the one it interrupted, never on the thread's");

   std::uintptr_t const here = sp_now();
   std::uint32_t const* const last = paint_below(here);

   handler_sp[outer_level] = 0u;
   handler_sp[upper_level] = 0u;
   active = scenario::stacks;
   force(outer_level);
   settle();
   active = scenario::none;

   std::uintptr_t const outer = handler_sp[outer_level];
   std::uintptr_t const upper = handler_sp[upper_level];
   cyros::bench::print("  boot sp ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(boot_sp));
   cyros::bench::print(", outer handler sp ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(outer));
   cyros::bench::print(", nested handler sp ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(upper));
   cyros::bench::print("\n");

   CYROS_CHECK(on_interrupt_stack(outer));
   CYROS_CHECK(on_interrupt_stack(upper));
   CYROS_CHECK(upper < outer);
   CYROS_CHECK(outer - upper < 2048u);
   CYROS_CHECK(!on_main_stack(upper));
   /* Nothing below the thread's frame was written: the nested frame and its
    * handler are not on the thread's stack. */
   CYROS_CHECK(reinterpret_cast<std::uintptr_t>(last) > reinterpret_cast<std::uintptr_t>(main_stack) + 4096u);
   CYROS_CHECK_EQ(paint_disturbed(last), 0u);
}

void check_handler_registers(char const* what, int call_depth, int recursion)
{
   cyros::bench::start(what);

   handler_call_depth = call_depth;
   nested_recursion = recursion;
   handler_registers_wrong = 0xFFFFFFFFu;
   upper_taken_in_routine = 0u;
   clear_events();

   active = scenario::registers;
   force(outer_level);
   settle();
   active = scenario::none;

   /* The upper level must have nested inside the routine's window, or the
    * check proves nothing. */
   CYROS_CHECK_EQ(upper_taken_in_routine, 1u);
   CYROS_CHECK_EQ(events[1], entered(upper_level, upper_level));
   CYROS_CHECK_EQ(handler_registers_wrong, 0u);
   CYROS_CHECK_EQ(handler_sum, expected_sum(call_depth));
   CYROS_CHECK_EQ(pil(), 0u);
}

void test_a_reschedule_pended_in_a_nested_handler_waits_for_the_outermost()
{
   cyros::bench::start("a reschedule pended in a nested handler waits for the outermost to return, then runs once");

   int const before = reschedule_calls;
   calls_seen_by_outer = -1;
   pending_seen_by_outer = false;

   active = scenario::pend_in_nested;
   force(outer_level);
   settle();
   active = scenario::none;

   CYROS_CHECK_EQ(calls_seen_by_outer, before);   /* not run inside the outer handler */
   CYROS_CHECK(pending_seen_by_outer);            /* but pending there */
   CYROS_CHECK_EQ(reschedule_calls, before + 1);  /* run once the thread was back */
   CYROS_CHECK(!pending(reschedule_level));
}

void test_a_device_nests_into_the_reschedule()
{
   cyros::bench::start("the reschedule runs at level 1, and a device nests into it");

   int const before = reschedule_calls;
   nested_in_reschedule = 0u;
   active = scenario::in_reschedule;
   cyros_port_pend_reschedule();
   settle();
   active = scenario::none;

   CYROS_CHECK_EQ(reschedule_calls, before + 1);
   CYROS_CHECK_EQ(reschedule_pil, reschedule_level);
   CYROS_CHECK_EQ(nested_in_reschedule, 1u);
}

void test_a_device_nests_into_the_yield()
{
   cyros::bench::start("the yield's reschedule runs at level 1 too, and a device nests into it");

   int const before = reschedule_calls;
   nested_in_reschedule = 0u;
   active = scenario::in_reschedule;
   cyros_port_thread_yield();
   active = scenario::none;

   CYROS_CHECK_EQ(reschedule_calls, before + 1);
   CYROS_CHECK_EQ(reschedule_pil, reschedule_level);
   CYROS_CHECK_EQ(nested_in_reschedule, 1u);
   CYROS_CHECK_EQ(pil(), 0u);
}

/* The second context: every window reused, then back to the main one, each
 * time it is switched in. */
volatile int other_ran = 0;

[[gnu::noinline]] std::uint32_t churn(int depth)
{
   volatile std::uint32_t local = static_cast<std::uint32_t>(depth) * 0x01010101u;
   return depth == 0 ? local : churn(depth - 1) ^ local;
}

void other_entry(void*)
{
   while (true) {
      other_ran = other_ran + 1;
      (void)churn(20);
      cyros_mask_token_t const token = cyros_port_irq_save();
      switch_from = other_context();
      switch_to   = main_context();
      cyros_port_pend_reschedule();
      cyros_port_irq_restore(token);   /* switched out here, resumed here */
   }
}

void check_nested_after_a_switch(char const* what, int depth)
{
   cyros::bench::start(what);

   int const before_calls = reschedule_calls;
   int const before_ran = other_ran;
   nested_after_switch = 0u;

   cyros_mask_token_t const token = cyros_port_irq_save();
   switch_from = main_context();
   switch_to   = other_context();
   active = scenario::after_switch;
   cyros_port_pend_reschedule();
   settle();

   std::uint32_t out[34] = {};
   std::uint32_t const sum = call_at_depth(depth, sparc_registers_at_pil0, out);
   active = scenario::none;

   /* There and back, the device nested between the switch and the exit. */
   CYROS_CHECK_EQ(nested_after_switch, 1u);
   CYROS_CHECK_EQ(reschedule_calls, before_calls + 2);
   CYROS_CHECK_EQ(other_ran, before_ran + 1);
   CYROS_CHECK_EQ(registers_wrong(out), 0u);
   CYROS_CHECK_EQ(sum, expected_sum(depth));

   cyros_port_irq_restore(token);
}

/* The time source's callback, inside the timer's body. */
volatile std::uint32_t timer_callbacks = 0;
volatile std::uint32_t timer_callback_pil = 0;
volatile std::uint32_t upper_inside_timer = 0xFFFFFFFFu;

void timer_callback(void*)
{
   timer_callbacks = timer_callbacks + 1u;
   timer_callback_pil = pil();
   std::uint32_t const before = taken[upper_level];
   force(upper_level);
   settle();
   upper_inside_timer = taken[upper_level] - before;
}

void test_the_timer_body_runs_masked()
{
   cyros::bench::start("the timer's body runs masked: a device above the timer's level waits for it");

   timer_callbacks = 0u;
   std::uint32_t const before = taken[upper_level];
   cyros_port_time_register_isr_handler(timer_callback, nullptr);
   cyros_port_time_setup(0u);
   cyros_port_time_irq_enable();

   /* A millisecond out, at the board's 1 MHz, and a tenth of a second to
    * arrive in. */
   std::uint64_t const deadline = cyros_port_time_now() + 1000u;
   cyros_port_time_arm(deadline);
   while (timer_callbacks == 0u && cyros_port_time_now() < deadline + 100000u) {}
   cyros_port_time_teardown();

   CYROS_CHECK_EQ(timer_callbacks, 1u);
   /* Both of PIL's interrupt bits set, not the timer's own level. */
   CYROS_CHECK_EQ(timer_callback_pil & 0xEu, 0xEu);
   CYROS_CHECK_EQ(upper_inside_timer, 0u);
   CYROS_CHECK_EQ(taken[upper_level] - before, 1u);   /* it ran after */
   CYROS_CHECK_EQ(pil(), 0u);
}

void test_a_level_15_in_a_trap_entry()
{
   cyros::bench::start("a level 15 pending as a trap is taken nests in its entry, and the trap stays the outermost");

   /* Where a level 15 runs when it is the outermost trap, for comparison. */
   nmi_count = 0u;
   nmi_sp = 0u;
   active = scenario::nmi_at_entry;
   force(nmi_level);
   settle();
   std::uintptr_t const outermost_nmi_sp = nmi_sp;
   CYROS_CHECK_EQ(nmi_count, 1u);

   nmi_count = 0u;
   nmi_sp = 0u;
   nmi_count_at_fault = 0xFFFFFFFFu;
   fault_sp = 0u;
   nested_fault_sp = 0u;
   fault_sentinel_ok = false;
   sparc_trap_with_level_15_pending(irqmp_force0);
   active = scenario::none;

   cyros::bench::print("  level 15 alone at sp ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(outermost_nmi_sp));
   cyros::bench::print(", in the trap's entry ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(nmi_sp));
   cyros::bench::print(", the trap's handler ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(fault_sp));
   cyros::bench::print(", a trap inside it ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(nested_fault_sp));
   cyros::bench::print("\n");

   /* It ran inside the trap's entry: before the trap's handler, and below
    * where it runs as the outermost trap, nested under the entry's frame. */
   CYROS_CHECK_EQ(nmi_count, 1u);
   CYROS_CHECK_EQ(nmi_count_at_fault, 1u);
   CYROS_CHECK(nmi_sp < outermost_nmi_sp);
   CYROS_CHECK(on_interrupt_stack(nmi_sp));
   /* The trap it nested into was still the outermost, so a trap inside that
    * one's handler stayed below it rather than restarting the stack. */
   CYROS_CHECK(on_interrupt_stack(fault_sp));
   CYROS_CHECK(nested_fault_sp < fault_sp);
   CYROS_CHECK(fault_sp - nested_fault_sp < 2048u);
   CYROS_CHECK(fault_sentinel_ok);
   CYROS_CHECK_EQ(pil(), 0u);
}

void main_entry(void*)
{
   cyros::bench::print("sparcv8 nested interrupts\n\n");

   reg(irqmp_mask0) = reg(irqmp_mask0)
                    | (1u << lower_level) | (1u << outer_level) | (1u << upper_level) | (1u << nmi_level);

   test_started_with_an_interrupt_stack();
   test_a_higher_level_nests_and_its_own_and_lower_wait();
   test_a_nested_handler_runs_below_the_one_it_interrupted();

   check_handler_registers("every register of a handler survives a nested interrupt", 0, 0);
   check_handler_registers("... taken in the invalid window, every window in use", 10, 0);
   check_handler_registers("... with a nested handler deep enough to spill the interrupted window", 0, 12);
   check_handler_registers("... both at once", 10, 12);

   test_a_reschedule_pended_in_a_nested_handler_waits_for_the_outermost();
   test_a_device_nests_into_the_reschedule();
   test_a_device_nests_into_the_yield();

   cyros_port_context_init(other_context(), other_stack, sizeof(other_stack), other_entry, nullptr);
   check_nested_after_a_switch("a device nested after a switch: the switch lands, every register comes back", 0);
   check_nested_after_a_switch("... from ten calls down", 10);

   test_the_timer_body_runs_masked();
   test_a_level_15_in_a_trap_entry();

   reg(irqmp_mask0) = reg(irqmp_mask0)
                    & ~((1u << lower_level) | (1u << outer_level) | (1u << upper_level) | (1u << nmi_level));

   /* Exactly the reschedules asked for: one pended in a nested handler, one
    * pended and one yield with a device nested, four from the switches. */
   cyros::bench::start("no reschedule beyond the ones asked for");
   CYROS_CHECK_EQ(reschedule_calls, 7);

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros_port_init(counting_reschedule);
   cyros_port_context_init(main_context(), main_stack, sizeof(main_stack), main_entry, nullptr);

   boot_sp = sp_now();
   cyros_port_start_first(main_context());

   cyros::bench::print("FAIL: cyros_port_start_first returned\n");
   cyros::bench::host_exit(5u);
}
