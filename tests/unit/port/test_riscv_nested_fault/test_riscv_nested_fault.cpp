/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_riscv_nested_fault.cpp
 * @brief A fault taken and resumed inside a handler leaves the handler's own
 *        trap resuming the right frame.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the rv32 layer's trap dispatch when a trap is taken inside a trap.
 * Trusts:  the bench harness and the port's time source.
 * Proves:  that the timer handler's trap, after a fault inside the handler
 *          was resumed by cyros_riscv_fault_handler, returns to the code it
 *          interrupted, not into the fault's frame.
 *
 * The dispatcher once recorded every trap's frame as the running thread's,
 * so the outer trap resumed the inner one's frame, on the stack below the
 * handler, and the image hung. Found on the SPARC port (sparc-port-plan.md
 * 5a), whose dispatcher had the same shape. The timer ISR takes an ebreak,
 * the fault handler skips it, and main counts on.
 */
#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>
#include <common/bench.hpp>
#include <cstdint>

namespace
{
volatile int isr_count = 0;
volatile int fault_count = 0;
volatile int isr_tails = 0;
void dummy_reschedule() {}
void isr(void*)
{
   isr_count = isr_count + 1;
   asm volatile(".option push \n .option norvc \n ebreak \n .option pop" ::: "memory");
   isr_tails = isr_tails + 1;
}
} // namespace

extern "C" void cyros_riscv_fault_handler(std::uint32_t mcause, std::uint32_t* frame)
{
   if (mcause != 3u) {
      cyros::bench::print("\n*** unexpected fault, mcause ");
      cyros::bench::print_hex(mcause);
      cyros::bench::print(" mepc ");
      cyros::bench::print_hex(frame[0]);
      cyros::bench::print("\n");
      cyros::bench::host_exit(3u);
   }
   fault_count = fault_count + 1;
   frame[0] = frame[0] + 4u;
}

extern "C" int cyros_bench_main()
{
   cyros::bench::start("a fault resumed inside a handler, three times");
   cyros_port_init(dummy_reschedule);
   cyros_port_time_register_isr_handler(isr, nullptr);
   cyros_port_time_setup(1000u);
   cyros_port_time_irq_enable();
   asm volatile("csrsi mstatus, 8" ::: "memory");
   volatile std::uint32_t main_counter = 0;
   for (std::uint32_t i = 0; i < 50'000'000u && isr_count < 3; ++i) { main_counter = main_counter + 1; }
   asm volatile("csrci mstatus, 8" ::: "memory");
   cyros_port_time_teardown();
   CYROS_CHECK(isr_count >= 3);
   CYROS_CHECK_EQ(fault_count, isr_count);
   CYROS_CHECK_EQ(isr_tails, isr_count);
   cyros::bench::finish();
}
