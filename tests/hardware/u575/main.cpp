/**
 * @file main.cpp
 * @brief First cyros image for real silicon: NUCLEO-U575ZI-Q.
 *
 * Deliberately small and deliberately SLOW, because the point of this one is
 * to be stepped through in a debugger rather than to assert anything. It boots
 * the kernel, starts two threads and hands the CPU back and forth between them
 * forever.
 *
 * Every function below is a reasonable breakpoint. The interesting ones:
 *
 *   ping / pong          the two threads. Break here and watch `baton` change.
 *   PendSV_Handler       the context switch itself, in the port. Stepping the
 *                        naked prologue is the whole show: `stmdb r0!, {r4-r11}`
 *                        stacking onto the outgoing thread's PSP, then the
 *                        epilogue popping from a DIFFERENT stack.
 *   cyros_port_switch    where PSP is swapped, in the port.
 *   cyros_port_start_first  the one-way trip into the first thread.
 *
 * Useful gdb once stopped:
 *   p/x $psp   p/x $msp   p/x $control   p/x $primask   p/x $basepri
 *   p/x $psplim                  the ARMv8-M stack guard, per thread
 *   p baton                      which thread ran last
 *   p switch_count               how many context switches have happened
 *
 * Output goes out over SEMIHOSTING, the same channel the QEMU bench uses, so
 * it needs `arm semihosting enable` in OpenOCD (debug.sh does it). Each call
 * halts the core briefly, so nothing timing-sensitive should print.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <common/arm/bench.hpp>

#include <cstddef>
#include <cstdint>

using namespace cyros;

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 2048;

alignas(CYROS_PORT_STACK_ALIGN) std::byte ping_stack[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte pong_stack[stack_size];

/* Volatile so a debugger always sees the real value rather than a register the
 * optimiser kept. These exist to be WATCHED. */
volatile std::uint32_t baton = 0;          /* 1 = ping ran, 2 = pong ran */
volatile std::uint32_t switch_count = 0;
volatile std::uint32_t ping_loops = 0;
volatile std::uint32_t pong_loops = 0;

/* A recognisable value in a callee-saved register across a switch. If the port
 * ever loses r4-r11, this is what shows it on real hardware. */
volatile std::uint32_t ping_marker_seen = 0;

void ping()
{
   for (;;) {
      baton = 1;
      ping_loops = ping_loops + 1;
      switch_count = switch_count + 1;

      /* Put a known pattern in a callee-saved register, give up the CPU, and
       * read it back. Survives only if PendSV stacked r4-r11 correctly. */
      std::uint32_t seen = 0;
      asm volatile(
         "mov  r7, %[pattern]  \n"
         "bl   cyros_hw_yield  \n"
         "mov  %[out], r7      \n"
         : [out] "=r"(seen)
         : [pattern] "r"(0xC0FFEEu)
         : "r0", "r1", "r2", "r3", "r7", "r12", "lr", "cc", "memory");
      ping_marker_seen = seen;
   }
}

void pong()
{
   for (;;) {
      baton = 2;
      pong_loops = pong_loops + 1;
      switch_count = switch_count + 1;
      this_thread::yield();
   }
}

} // namespace


/* Called from ping's inline assembly. Not static and extern "C" so the `bl`
 * finds an unmangled symbol. */
extern "C" void cyros_hw_yield()
{
   this_thread::yield();
}


extern "C" int cyros_bench_main()
{
   cyros::bench::print("\ncyros on STM32U575, NUCLEO-U575ZI-Q\n");
   cyros::bench::print("kernel::initialise()\n");

   kernel::initialise();

   thread ping_thread(ping, ping_stack, thread::priority(0), core0);
   thread pong_thread(pong, pong_stack, thread::priority(0), core0);

   cyros::bench::print("two threads registered, starting the scheduler\n");
   cyros::bench::print("(this does not return. break on ping/pong/PendSV_Handler)\n");

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned\n");
   cyros::bench::host_exit(5u);
}
