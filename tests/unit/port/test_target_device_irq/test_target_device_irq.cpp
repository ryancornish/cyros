/**
 * @file test_target_device_irq.cpp
 * @brief A device interrupt, on either of the RP2350's ISAs: delivered, graded
 *        against the two masks, and waking a thread.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: how a peripheral's interrupt reaches an application handler through
 *          the port: the NVIC on the Cortex-M33s, Xh3irq dispatched by the
 *          rp2350_hazard3 target on the Hazard3s, each through the board's
 *          weak `isr_irqN` names.
 * Trusts:  layers 0 to 5, and the semaphore's counting (test_sync_semaphore).
 * Proves:  that TIMER0's alarm interrupt reaches `isr_irq0`, that it runs
 *          inside a preempt-disable and waits out an interrupt mask, which is
 *          port.h's line between the two grades, and that a release from it
 *          wakes a blocked thread, with what that costs.
 *
 * The interrupt is TIMER0's alarm 0 (IRQ 0), which needs no wiring. The test
 * enables it the way an application would on each ISA: the NVIC's enable and
 * priority on Arm, at the priority the port gives SysTick (the most urgent
 * value a device may have and still be masked only by an interrupt mask),
 * and Xh3irq's MEIEA on RISC-V.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <cstddef>
#include <cstdint>

using namespace cyros;

namespace
{

/* --------------------------------------------------------------------------
 * TIMER0, ticking every microsecond from the 12 MHz crystal
 * ----------------------------------------------------------------------- */

constexpr std::uintptr_t timer0          = 0x400b0000u;
constexpr std::uintptr_t timer0_alarm0   = timer0 + 0x10u;
constexpr std::uintptr_t timer0_timerawl = timer0 + 0x28u;
constexpr std::uintptr_t timer0_intr     = timer0 + 0x3cu;
constexpr std::uintptr_t timer0_inte     = timer0 + 0x40u;
constexpr std::uintptr_t ticks_timer0_ctrl   = 0x40108018u;
constexpr std::uintptr_t ticks_timer0_cycles = 0x4010801cu;
constexpr std::uintptr_t resets_reset    = 0x40020000u;
constexpr std::uintptr_t resets_done     = 0x40020008u;
constexpr std::uint32_t  reset_timer0    = 1u << 23;
constexpr std::uintptr_t clear_alias     = 0x3000u;

volatile std::uint32_t& reg(std::uintptr_t address)
{
   return *reinterpret_cast<volatile std::uint32_t*>(address);
}

std::uint32_t micros() { return reg(timer0_timerawl); }

void timer0_init()
{
   reg(ticks_timer0_ctrl) = 0u;
   reg(ticks_timer0_cycles) = 12u;
   reg(ticks_timer0_ctrl) = 1u;
   reg(resets_reset + clear_alias) = reset_timer0;
   while ((reg(resets_done) & reset_timer0) == 0u) {}
   reg(timer0_intr) = 1u;     /* nothing stale */
   reg(timer0_inte) = 1u;     /* alarm 0 */
}

void alarm_in(std::uint32_t us) { reg(timer0_alarm0) = micros() + us; }

void spin_for(std::uint32_t us)
{
   std::uint32_t const start = micros();
   while (micros() - start < us) {}
}

/* IRQ 0, enabled as an application would, per ISA. */
void irq0_enable()
{
#if defined(__riscv)
   /* MEIEA: window 0 in the low bits, its 16 enables in the high half. */
   asm volatile("csrs 0xbe0, %0" : : "r"(1u << 16) : "memory");
#else
   /* At SysTick's priority, which the port derived and wrote into SHPR3. */
   std::uint8_t const priority = *reinterpret_cast<volatile std::uint8_t*>(0xE000ED23u);
   *reinterpret_cast<volatile std::uint8_t*>(0xE000E400u) = priority;   /* IPR, IRQ 0 */
   reg(0xE000E280u) = 1u;                                               /* ICPR */
   reg(0xE000E100u) = 1u;                                               /* ISER */
#endif
}

/* --------------------------------------------------------------------------
 * The handler
 * ----------------------------------------------------------------------- */

volatile std::uint32_t fired = 0u;
volatile std::uint64_t fired_at = 0u;
volatile bool release_from_isr = false;
sync::semaphore wake{0};

}  // namespace

extern "C" void isr_irq0(void)
{
   fired_at = cyros_port_timestamp();
   reg(timer0_intr) = 1u;            /* clear the alarm, or it is offered again */
   fired = fired + 1u;
   if (release_from_isr) {
      wake.release();
   }
}

namespace
{

/* --------------------------------------------------------------------------
 * The threads
 * ----------------------------------------------------------------------- */

constexpr std::size_t stack_size = thread::min_stack_size + 1024;
alignas(CYROS_PORT_STACK_ALIGN) std::byte driver_stack[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte waiter_stack[stack_size];

constexpr int rounds = 16;
volatile int woken = 0;
volatile std::uint64_t woke_at = 0u;

/* More urgent than the driver, so the wake preempts it the moment the
 * interrupt returns. */
void waiter()
{
   while (true) {
      wake.acquire();
      woke_at = cyros_port_timestamp();
      woken = woken + 1;
   }
}

void driver()
{
   timer0_init();
   irq0_enable();

   cyros::bench::start("TIMER0's alarm reaches isr_irq0");
   alarm_in(100u);
   spin_for(2000u);
   CYROS_CHECK_EQ(fired, 1u);

   cyros::bench::start("it runs inside a preempt-disable");
   {
      cyros_mask_token_t const token = cyros_port_preempt_disable();
      alarm_in(100u);
      spin_for(2000u);
      std::uint32_t const during = fired;
      cyros_port_preempt_enable(token);
      CYROS_CHECK_EQ(during, 2u);
   }

   cyros::bench::start("it waits out an interrupt mask, then runs");
   {
      cyros_mask_token_t const token = cyros_port_irq_save();
      alarm_in(100u);
      spin_for(2000u);
      std::uint32_t const during = fired;
      cyros_port_irq_restore(token);
      spin_for(100u);
      CYROS_CHECK_EQ(during, 2u);
      CYROS_CHECK_EQ(fired, 3u);
   }

   cyros::bench::start("a release from it wakes a blocked thread");
   release_from_isr = true;
   std::uint64_t best = ~std::uint64_t{0};
   std::uint64_t worst = 0u;
   for (int r = 0; r < rounds; ++r) {
      int const before = woken;
      alarm_in(200u);
      std::uint32_t const start = micros();
      while (woken == before && micros() - start < 5000u) {}
      if (woken != before) {
         std::uint64_t const cost = woke_at - fired_at;
         best = cost < best ? cost : best;
         worst = cost > worst ? cost : worst;
      }
   }
   CYROS_CHECK_EQ(woken, rounds);
   CYROS_CHECK_EQ(wake.peek(), 0);
   cyros::bench::print("  cycles from the handler's stamp to the woken thread's, best and worst of 16: ");
   cyros::bench::print_dec(best);
   cyros::bench::print(" to ");
   cyros::bench::print_dec(worst);
   cyros::bench::print("\n");

   cyros::bench::finish();
}

}  // namespace

extern "C" int cyros_bench_main()
{
   cyros::bench::print("a device interrupt: TIMER0's alarm on IRQ 0\n\n");

   kernel::initialise();
   thread w(waiter, waiter_stack, thread::priority(1), core0);
   thread d(driver, driver_stack, thread::priority(2), core0);
   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
