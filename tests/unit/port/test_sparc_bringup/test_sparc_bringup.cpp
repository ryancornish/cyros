/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_sparc_bringup.cpp
 * @brief First kernel bring-up on the sparcv8 core layer. Runs on QEMU's
 *        leon3_generic.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the sparcv8 layer's context switch, as exercised through the
 *          kernel's own scheduler.
 * Trusts:  layer 0, the port's masking and trap path (test_sparc_port).
 * Proves:  that cyros_port_context_init builds a frame the trap exit can
 *          enter, that cyros_port_start_first reaches it, and that a yield
 *          carries a thread out and back with its register windows, its own
 *          TLS pointer and its stack intact, from a depth where every window
 *          is in use, while the other thread churns every window in between.
 *
 * The SPARC sibling of test_armv8r_bringup. On SPARC the callee-saved state is
 * the WINDOW: a function's locals and ins survive its calls because they are
 * in a window of their own. A switch must flush the outgoing thread's windows
 * to their stacks and bring the incoming thread's back, so the check holds a
 * pattern in every local and in-register of one window across a yield, and
 * the other thread recurses deep enough to reuse every window in between. TLS
 * is %g7, which the frame holds.
 *
 * kernel::start() does not return on this port, so the last thread to run
 * reports and exits the image.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>
#include <cyros/port/port_core.h>

#include <common/bench.hpp>

#include <cstddef>
#include <cstdint>
#include <initializer_list>

using namespace cyros;

static_assert(config::cores == 1, "This bring-up is single core");

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 4096;

alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_a[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_b[stack_size];

constexpr std::size_t trace_capacity = 16;
int trace[trace_capacity];
std::size_t trace_length = 0;

void mark(int value)
{
   if (trace_length < trace_capacity) { trace[trace_length++] = value; }
}

bool trace_is(std::initializer_list<int> expected)
{
   if (trace_length != expected.size()) { return false; }
   std::size_t index = 0;
   for (int value : expected) {
      if (trace[index++] != value) { return false; }
   }
   return true;
}

void print_trace()
{
   cyros::bench::print("  trace = [");
   for (std::size_t i = 0; i < trace_length; ++i) {
      cyros::bench::print_hex(static_cast<std::uint32_t>(trace[i]));
      if (i + 1 < trace_length) { cyros::bench::print(", "); }
   }
   cyros::bench::print("]\n");
}

} // namespace


/* Called from the assembly below, so unmangled. */
extern "C" void cyros_bench_yield()
{
   this_thread::yield();
}

/* ---------------------------------------------------------------------------
 * The window check
 *
 * Its own window, a pattern in every local and in i0 to i5, then a call that
 * yields. i6 and i7 are the frame pointer and the return address, which the
 * return proves. The count is built in o0 only after the call, and returned
 * through the window as i0 by `restore`.
 * ------------------------------------------------------------------------ */

extern "C" std::uint32_t window_wrong_after_a_yield();

asm(
   ".pushsection .text.window_wrong_after_a_yield, \"ax\"       \n"
   ".global window_wrong_after_a_yield                          \n"
   ".type   window_wrong_after_a_yield, #function               \n"
   "window_wrong_after_a_yield:                                 \n"
   "   save  %sp, -96, %sp                                      \n"
   ".irp n, 0,1,2,3,4,5,6,7                                     \n"
   "   set   0x5A000010 + \\n, %l\\n                            \n"
   ".endr                                                       \n"
   ".irp n, 0,1,2,3,4,5                                         \n"
   "   set   0x5A000018 + \\n, %i\\n                            \n"
   ".endr                                                       \n"
   "   call  cyros_bench_yield                                  \n"
   "    nop                                                     \n"
   "   mov   0, %o0                                             \n"
   ".irp n, 0,1,2,3,4,5,6,7                                     \n"
   "   set   0x5A000010 + \\n, %o1                              \n"
   "   cmp   %l\\n, %o1                                         \n"
   "   be    1f                                                 \n"
   "    nop                                                     \n"
   "   add   %o0, 1, %o0                                        \n"
   "1:                                                          \n"
   ".endr                                                       \n"
   ".irp n, 0,1,2,3,4,5                                         \n"
   "   set   0x5A000018 + \\n, %o1                              \n"
   "   cmp   %i\\n, %o1                                         \n"
   "   be    1f                                                 \n"
   "    nop                                                     \n"
   "   add   %o0, 1, %o0                                        \n"
   "1:                                                          \n"
   ".endr                                                       \n"
   "   ret                                                      \n"
   "   restore %o0, 0, %o0                                      \n"
   ".size window_wrong_after_a_yield, . - window_wrong_after_a_yield \n"
   ".popsection                                                 \n");

namespace
{

/* The check `depth` calls down, each with a live local, so that from depth 8
 * on every window is in use when it yields. The sum comes out right only if
 * every caller's window came back. */
[[gnu::noinline]] std::uint32_t windows_at_depth(int depth, std::uint32_t& wrong)
{
   volatile std::uint32_t local = 0x3000u + static_cast<std::uint32_t>(depth);
   if (depth == 0) {
      wrong = window_wrong_after_a_yield();
      return local;
   }
   return windows_at_depth(depth - 1, wrong) + local;
}

std::uint32_t expected_sum(int depth)
{
   std::uint32_t sum = 0;
   for (int d = 0; d <= depth; ++d) { sum += 0x3000u + static_cast<std::uint32_t>(d); }
   return sum;
}

/* Every window, and more, between the other thread's yields. */
[[gnu::noinline]] std::uint32_t churn(int depth)
{
   volatile std::uint32_t local = static_cast<std::uint32_t>(depth) * 0x01010101u;
   return depth == 0 ? local : churn(depth - 1) ^ local;
}

std::uint32_t shallow_wrong = 0xFFFFFFFFu;
std::uint32_t deep_wrong    = 0xFFFFFFFFu;
std::uint32_t deep_sum      = 0u;
bool b_churned = false;

/* Each thread's TLS pointer as its launcher set it, and how many times it was
 * found changed after a switch back in. */
void* tls_a = nullptr;
void* tls_b = nullptr;
std::uint32_t tls_changed = 0;

void check_tls(void* own)
{
   if (cyros_port_get_tls_pointer() != own) { ++tls_changed; }
}

void thread_a()
{
   tls_a = cyros_port_get_tls_pointer();
   mark(1);
   this_thread::yield();
   check_tls(tls_a);
   mark(3);
   /* A window held across a yield to B, which churns and yields back. */
   shallow_wrong = window_wrong_after_a_yield();
   check_tls(tls_a);
   mark(5);
   /* The same from twelve calls down, every window in use. */
   deep_sum = windows_at_depth(12, deep_wrong);
   check_tls(tls_a);
   mark(7);
   this_thread::yield();
}

void thread_b()
{
   tls_b = cyros_port_get_tls_pointer();
   mark(2);
   this_thread::yield();
   check_tls(tls_b);
   mark(4);
   b_churned = churn(20) != 0xFFFFFFFFu;
   this_thread::yield();
   check_tls(tls_b);
   mark(6);
   (void)churn(20);
   this_thread::yield();
   check_tls(tls_b);
   mark(8);

   cyros::bench::start("both threads ran, in registration order, across yields");
   print_trace();
   CYROS_CHECK(trace_is({1, 2, 3, 4, 5, 6, 7, 8}));

   cyros::bench::start("a window's locals and ins survive a context switch");
   CYROS_CHECK(b_churned);
   CYROS_CHECK_EQ(shallow_wrong, 0u);

   cyros::bench::start("... from a depth where every window is in use, and every caller's comes back");
   CYROS_CHECK_EQ(deep_wrong, 0u);
   CYROS_CHECK_EQ(deep_sum, expected_sum(12));

   cyros::bench::start("each thread's TLS pointer (%g7) follows it across switches");
   CYROS_CHECK(tls_a != nullptr);
   CYROS_CHECK(tls_b != nullptr);
   CYROS_CHECK(tls_a != tls_b);
   CYROS_CHECK_EQ(tls_changed, 0u);

   cyros::bench::start("kernel still reports its threads");
   CYROS_CHECK(kernel::core_count() == 1u);

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("sparcv8 kernel bring-up\n\n");

   kernel::initialise();

   cyros::bench::start("threads register before the kernel starts");
   thread a(thread_a, stack_a, thread::priority(0), core0);
   thread b(thread_b, stack_b, thread::priority(0), core0);
   CYROS_CHECK_EQ(kernel::active_threads(), 2u);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
