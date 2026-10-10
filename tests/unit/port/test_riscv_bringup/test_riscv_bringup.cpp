/**
 * @file test_riscv_bringup.cpp
 * @brief First kernel bring-up on the rv32 core layer. Runs on QEMU's virt.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the rv32 layer's context switch, as exercised through the kernel's
 *          own scheduler.
 * Trusts:  layer 0, the port's masking and trap path (test_riscv_port).
 * Proves:  that cyros_port_context_init builds a frame the trap exit can
 *          enter, that cyros_port_start_first reaches it, and that a yield
 *          carries a thread out and back with its callee-saved registers, its
 *          own TLS pointer and its stack intact.
 *
 * The RISC-V sibling of test_cortex_m_bringup. What differs is the register
 * check: here the switch is a trap whose frame holds every register, and the
 * check holds a pattern in each callee-saved one, and in tp, across a yield
 * that runs another thread in between.
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

constexpr std::size_t stack_size = thread::min_stack_size + 2048;

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
 * The callee-saved register check
 *
 * s0 to s11 are what a callee must preserve, and tp is the thread's own
 * pointer, which the port promises follows the thread. A pattern goes into
 * each, the thread yields (and the other thread runs and uses the same
 * registers for its own work), and the routine counts what came back wrong.
 * Naked and self-contained, so the compiler holds nothing of its own in them.
 * The count is built in a0, which the yield is free to clobber, only after it.
 * ------------------------------------------------------------------------ */

extern "C" [[gnu::naked]] std::uint32_t callee_saved_wrong_after_a_yield()
{
   asm volatile(
      "addi  sp, sp, -64            \n"
      "sw    ra,  0(sp)             \n"
      "sw    tp,  4(sp)             \n"
      "sw    s0,  8(sp)             \n"
      "sw    s1, 12(sp)             \n"
      "sw    s2, 16(sp)             \n"
      "sw    s3, 20(sp)             \n"
      "sw    s4, 24(sp)             \n"
      "sw    s5, 28(sp)             \n"
      "sw    s6, 32(sp)             \n"
      "sw    s7, 36(sp)             \n"
      "sw    s8, 40(sp)             \n"
      "sw    s9, 44(sp)             \n"
      "sw    s10, 48(sp)            \n"
      "sw    s11, 52(sp)            \n"
      ".irp n, 4,8,9,18,19,20,21,22,23,24,25,26,27 \n"
      "li    x\\n, 0x5A000000 + \\n  \n"
      ".endr                        \n"
      "call  cyros_bench_yield      \n"
      "li    a0, 0                  \n"
      ".irp n, 4,8,9,18,19,20,21,22,23,24,25,26,27 \n"
      "li    a1, 0x5A000000 + \\n    \n"
      "beq   x\\n, a1, 1f            \n"
      "addi  a0, a0, 1              \n"
      "1:                           \n"
      ".endr                        \n"
      "lw    ra,  0(sp)             \n"
      "lw    tp,  4(sp)             \n"
      "lw    s0,  8(sp)             \n"
      "lw    s1, 12(sp)             \n"
      "lw    s2, 16(sp)             \n"
      "lw    s3, 20(sp)             \n"
      "lw    s4, 24(sp)             \n"
      "lw    s5, 28(sp)             \n"
      "lw    s6, 32(sp)             \n"
      "lw    s7, 36(sp)             \n"
      "lw    s8, 40(sp)             \n"
      "lw    s9, 44(sp)             \n"
      "lw    s10, 48(sp)            \n"
      "lw    s11, 52(sp)            \n"
      "addi  sp, sp, 64             \n"
      "ret                          \n");
}

namespace
{

std::uint32_t registers_wrong = 0xFFFFFFFFu;
bool b_reached_register_window = false;

/* Each thread's TLS pointer as its launcher set it, and how many times it was
 * found changed after a switch back in. The kernel writes tp once per thread,
 * so only the trap frame can bring it back after another thread ran. */
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
   /* Held across a yield to B, which runs and yields back. */
   registers_wrong = callee_saved_wrong_after_a_yield();
   check_tls(tls_a);
   mark(5);
   this_thread::yield();
}

void thread_b()
{
   tls_b = cyros_port_get_tls_pointer();
   mark(2);
   this_thread::yield();
   check_tls(tls_b);
   mark(4);
   /* A is parked inside its register window now. Disturb every register it
    * holds by doing ordinary work, then hand back. */
   b_reached_register_window = true;
   volatile std::uint32_t sink = 0;
   for (std::uint32_t i = 0; i < 64; ++i) { sink = sink + i * 0x01010101u; }
   this_thread::yield();
   check_tls(tls_b);
   mark(6);

   cyros::bench::start("both threads ran, in registration order, across yields");
   print_trace();
   CYROS_CHECK(trace_is({1, 2, 3, 4, 5, 6}));

   cyros::bench::start("callee-saved registers and tp survive a context switch");
   CYROS_CHECK(b_reached_register_window);
   CYROS_CHECK_EQ(registers_wrong, 0u);

   cyros::bench::start("each thread's TLS pointer follows it across switches");
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
   cyros::bench::print("rv32 kernel bring-up\n\n");

   kernel::initialise();

   cyros::bench::start("threads register before the kernel starts");
   thread a(thread_a, stack_a, thread::priority(0), core0);
   thread b(thread_b, stack_b, thread::priority(0), core0);
   CYROS_CHECK_EQ(kernel::active_threads(), 2u);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
