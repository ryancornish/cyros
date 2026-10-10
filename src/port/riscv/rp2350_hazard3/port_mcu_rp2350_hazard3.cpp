/**
 * @file port_mcu_rp2350_hazard3.cpp
 * @brief The RP2350's Hazard3 cores: identity, bring-up, the IPI, and the SIO
 *        registers the core layer and the time source use.
 *
 * Compiled by two targets that differ only in port_traits.h:
 * `rp2350_hazard3`, core 0 alone, and `rp2350_hazard3_smp`, both cores.
 *
 * Everything the core layer cannot reach on its own is in the SIO, the
 * single-cycle block each core sees at 0xd0000000 (`rp2350-notes.md` 5b, 4b):
 *
 *   RISCV_SOFTIRQ  bit n raises hart n's soft IRQ, bit 8+n lowers it
 *   MTIME          one 64-bit counter both cores share
 *   MTIMECMP       THIS core's compare. Each core sees its own at one address
 *
 * Device interrupts come through Xh3irq, Hazard3's interrupt controller, as
 * the machine external interrupt. This target lets each hart take it
 * (mie.MEIE) and dispatches every pending IRQ, in Xh3irq's priority order,
 * through a table the BOARD supplies, as an ARM board supplies its vector
 * table. Enabling and prioritising an IRQ is the application's (the MEIEA and
 * MEIPRA arrays), as it is on the NVIC. Handlers run inside the trap with
 * interrupts masked, so they do not nest, and a preempt-disable (mie.MSIE)
 * does not hold them off, which is port.h's line between the two grades.
 *
 * MTIME's rate and source (MTIME_CTRL.FULLSPEED) are the board's, as SysTick's
 * clock is on ARM. The board says the rate through cyros_port_mtime_clock_hz.
 *
 * Erratum E2: a write of 0 to RISCV_SOFTIRQ releases SIO spinlock 8, among
 * others (rp2350-notes.md 5d). Every write below sets exactly one bit.
 *
 *
 * How core 1 starts
 * =================
 * Core 1 waits in the boot ROM's holding pen, which reads the inter-core FIFO.
 * start_cores sends it the boot ROM's six-word launch sequence (0, 0, 1, trap
 * vector, stack, entry), each word echoed back, as the SDK's
 * multicore_launch_core1_raw does (rp2350-notes.md 2c). Two things differ from
 * Arm: the pen sleeps on Hazard3's h3.block, so each push is followed by
 * h3.unblock or it is never seen, and there is no RCP salt to seed. The board
 * supplies core 1's stack, as the SSE-200's supplies CPU1's vector table.
 */

#include <cyros/port/port_mcu.h>

#include "mtime.hpp"
#include "riscv.hpp"

#include <cstddef>
#include <cstdint>

namespace riscv = cyros::port::riscv;
namespace mtime = cyros::port::mtime;

namespace
{

constexpr std::uintptr_t sio_base          = 0xd0000000u;
constexpr std::uintptr_t sio_fifo_st       = sio_base + 0x050u;
constexpr std::uintptr_t sio_fifo_wr       = sio_base + 0x054u;
constexpr std::uintptr_t sio_fifo_rd       = sio_base + 0x058u;
constexpr std::uint32_t  fifo_vld          = 1u << 0;
constexpr std::uint32_t  fifo_rdy          = 1u << 1;
constexpr std::uintptr_t sio_riscv_softirq = sio_base + 0x1a0u;
constexpr std::uintptr_t sio_mtime         = sio_base + 0x1b0u;
constexpr std::uintptr_t sio_mtimeh        = sio_base + 0x1b4u;
constexpr std::uintptr_t sio_mtimecmp      = sio_base + 0x1b8u;
constexpr std::uintptr_t sio_mtimecmph     = sio_base + 0x1bcu;

inline volatile std::uint32_t& reg(std::uintptr_t address) noexcept
{
   return *reinterpret_cast<volatile std::uint32_t*>(address);
}

/* A read whose value is not wanted. `(void)reg(a)` would not do: a discarded
 * call returning a volatile reference is never converted to a value, so the
 * load is not emitted (measured: soft_irq_raise had no read-back). */
inline std::uint32_t load(std::uintptr_t address) noexcept
{
   return reg(address);
}

/* The entry every core runs, stashed by start_cores for core 1. Written by
 * core 0 before the launch, read-only after. */
cyros_port_core_entry_t core_entry = nullptr;

/* Everything this core wrote to memory is visible to the other before the
 * store that interrupts or launches it. */
inline void fence_before_signal() noexcept
{
   asm volatile("fence rw, ow" ::: "memory");
}

/* Hazard3's wake for the other hart's h3.block, the Arm SEV. */
inline void h3_unblock() noexcept
{
   asm volatile("slt x0, x0, x1" ::: "memory");
}

void fifo_push(std::uint32_t value) noexcept
{
   while ((reg(sio_fifo_st) & fifo_rdy) == 0u) {}
   reg(sio_fifo_wr) = value;
   h3_unblock();
}

/* Bounded: a core 1 that never answers is a fault, not a hang. */
bool fifo_pop(std::uint32_t& value) noexcept
{
   for (std::uint32_t spins = 0; spins < 10'000'000u; ++spins) {
      if ((reg(sio_fifo_st) & fifo_vld) != 0u) {
         value = reg(sio_fifo_rd);
         return true;
      }
   }
   return false;
}

void fifo_drain() noexcept
{
   while ((reg(sio_fifo_st) & fifo_vld) != 0u) { (void)load(sio_fifo_rd); }
}

/* Every device interrupt this hart may take is then gated by its own MEIEA
 * bit, which the application sets. */
inline void enable_device_interrupts() noexcept
{
   riscv::set_mie(riscv::mie_meie);
}

} // namespace

/* The core layer's trap vector, which core 1 is launched with. */
extern "C" void cyros_port_trap_entry(void);

/* Where a launched core joins the kernel, defined below. */
extern "C" [[noreturn]] void cyros_port_secondary_core_entry(void);


/* ============================================================================
 * Board-supplied
 * ========================================================================= */

/**
 * @brief The top of core 1's stack, supplied by the application.
 *
 * Core 1 runs on it from launch, and it becomes core 1's interrupt stack when
 * its first thread starts, as the boot stack does for core 0. 16-byte aligned.
 * cyros declares it and never defines it, as cyros_port_cpu1_vector_table on
 * the SSE-200.
 */
extern "C" void* cyros_port_core1_stack_top(void);

/**
 * @brief The device interrupt handlers, one per Xh3irq line (52 on the
 *        RP2350), supplied by the application. The table and every entry in
 *        it must be valid: an entry for an IRQ nobody enables can report and
 *        stop, as an ARM board's default handler does.
 */
using cyros_irq_handler = void (*)(void);
extern "C" cyros_irq_handler const* cyros_port_irq_table(void);


/* ============================================================================
 * SMP & Multi-Core Support
 * ========================================================================= */

std::uint32_t cyros_port_get_core_id(void)
{
   return riscv::read_mhartid();
}

void cyros_port_start_cores(std::size_t cores_to_use, cyros_port_core_entry_t entry)
{
   CYROS_ASSERT_OP(cores_to_use, >=, 1u);
   CYROS_ASSERT_OP(cores_to_use, <=, static_cast<std::size_t>(CYROS_PORT_CORE_COUNT));
   CYROS_ASSERT(entry != nullptr);
   CYROS_ASSERT_OP(cyros_port_get_core_id(), ==, 0u);

   core_entry = entry;
   enable_device_interrupts();

   if (cores_to_use > 1u) {
      void* const stack = cyros_port_core1_stack_top();
      CYROS_ASSERT(stack != nullptr);
      CYROS_ASSERT_OP(reinterpret_cast<std::uintptr_t>(stack) & 15u, ==, 0u);

      std::uint32_t const sequence[6] = {
         0u, 0u, 1u,
         static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(&cyros_port_trap_entry)),
         static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(stack)),
         static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(&cyros_port_secondary_core_entry)),
      };
      fence_before_signal();

      /* Each word is echoed. A wrong echo restarts the sequence, and a zero is
       * sent only on an empty FIFO, which is what lets the pen resynchronise
       * from any state. 6 words on every launch measured (2c). */
      std::uint32_t index = 0;
      std::uint32_t words = 0;
      while (index < 6u) {
         if (sequence[index] == 0u) {
            fifo_drain();
            h3_unblock();
         }
         fifo_push(sequence[index]);
         std::uint32_t echo = 0;
         CYROS_ASSERT(fifo_pop(echo));      /* core 1 is in the pen and answers */
         index = echo == sequence[index] ? index + 1u : 0u;
         CYROS_ASSERT_OP(++words, <, 64u);  /* and the sequence converges */
      }
   }

   /* Core 0 runs the same entry as every other. */
   entry();
   CYROS_PORT_UNREACHABLE();
}

/**
 * @brief Where core 1 joins the kernel: the boot ROM's launch enters here, on
 *        the board's stack and with the core layer's trap vector.
 *
 * init_this_core makes the per-hart state the port's, as on riscv_virt.
 */
extern "C" [[noreturn]] void cyros_port_secondary_core_entry(void)
{
   riscv::init_this_core();
   enable_device_interrupts();
   CYROS_ASSERT(core_entry != nullptr);
   core_entry();
   CYROS_PORT_UNREACHABLE();
}

void cyros_port_send_reschedule_ipi(std::uint32_t core_id)
{
   /* As on riscv_virt: the cross-core and the local request are one
    * mechanism. The target reads scheduler state as soon as it takes the
    * interrupt, so this core's writes go first. */
   CYROS_ASSERT_OP(core_id, <, static_cast<std::uint32_t>(CYROS_PORT_CORE_COUNT));
   fence_before_signal();
   riscv::soft_irq_raise(core_id);
}


/* ============================================================================
 * What the rv32 core layer needs (riscv.hpp)
 * ========================================================================= */

namespace cyros::port::riscv
{

void soft_irq_raise(std::uint32_t core) noexcept
{
   reg(sio_riscv_softirq) = 1u << core;
   /* Read back, so the store has reached the SIO before this returns. */
   (void)load(sio_riscv_softirq);
}

void soft_irq_clear(std::uint32_t core) noexcept
{
   reg(sio_riscv_softirq) = 1u << (8u + core);
   (void)load(sio_riscv_softirq);
}

void external_interrupt() noexcept
{
   /* MEINEXT names the highest-priority pending, enabled IRQ, in bits 10:2,
    * and reads negative when there is none, so one trap services every IRQ
    * pending when it was taken and any raised meanwhile (rp2350-notes.md 6b).
    * A handler must clear its source, or it is offered again at once. */
   cyros_irq_handler const* const table = cyros_port_irq_table();
   while (true) {
      std::int32_t next;
      asm volatile("csrr %0, 0xbe4" : "=r"(next));
      if (next < 0) {
         return;
      }
      table[static_cast<std::uint32_t>(next) >> 2]();
   }
}

std::uint64_t timestamp() noexcept
{
   /* MTIME, Ryan's choice for this chip (rp2350-notes.md 10): at FULLSPEED it
    * counts the core clock, so it is as fine as mcycle, and both cores read
    * the same counter, so stamps compare across them. 10 cycles a read
    * against mcycle's 3 (rp2350-notes.md 4a). */
   return mtime::read();
}

void timer_interrupt() noexcept
{
   mtime::interrupt();
}

std::uint8_t stack_guard_setup() noexcept
{
   /* Xh3pmpm: PMPCFGM0 bit n applies PMP entry n to machine mode WITHOUT
    * locking it, so the guard moves with one pmpaddr0 write (rp2350-notes.md
    * 6c). Erratum E6 swaps the R and X bits, which a guard granting nothing
    * does not care about. */
   asm volatile("csrs 0xbd0, %0" : : "r"(1u) : "memory");
   return pmpcfg_napot;
}

} // namespace cyros::port::riscv


/* ============================================================================
 * The MTIME timer (../../common/mtime.hpp), in the SIO
 * ========================================================================= */

namespace cyros::port::mtime
{

std::uint64_t read() noexcept
{
   std::uint32_t high, low, again;
   do {
      high  = reg(sio_mtimeh);
      low   = reg(sio_mtime);
      again = reg(sio_mtimeh);
   } while (high != again);
   return (static_cast<std::uint64_t>(high) << 32) | low;
}

void compare_write(std::uint32_t core, std::uint64_t value) noexcept
{
   /* The SIO has one MTIMECMP per core at one address, so a core can only
    * write its own. The time source only ever asks for that. */
   CYROS_ASSERT_OP(core, ==, riscv::read_mhartid());
   reg(sio_mtimecmp)  = ~0u;
   reg(sio_mtimecmph) = static_cast<std::uint32_t>(value >> 32);
   reg(sio_mtimecmp)  = static_cast<std::uint32_t>(value);
}

void interrupt_enable() noexcept
{
   riscv::set_mie(riscv::mie_mtie);
}

void interrupt_disable() noexcept
{
   riscv::clear_mie(riscv::mie_mtie);
}

} // namespace cyros::port::mtime
