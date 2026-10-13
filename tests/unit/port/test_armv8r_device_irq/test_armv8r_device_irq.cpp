/**
 * @file test_armv8r_device_irq.cpp
 * @brief A device interrupt on the Cortex-R52: a GIC SPI reaching the
 *        application through the armv8r layer, graded against the two masks,
 *        and waking a thread.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: how a peripheral's interrupt reaches an application handler through
 *          the armv8r layer: acknowledged by the IRQ path, handed by INTID to
 *          the weak cyros_armv8r_device_interrupt, and ended.
 * Trusts:  layers 0 to 5, and the semaphore's counting (test_sync_semaphore).
 * Proves:  that an SPI reaches the application's handler with its INTID, that
 *          it runs inside a preempt-disable and waits out an interrupt mask,
 *          which is port.h's line between the two grades, that it is ended,
 *          so it is offered once per pend, and that a release from it wakes a
 *          blocked thread.
 *
 * The R-profile sibling of test_target_device_irq, which uses the RP boards'
 * timers. QEMU's mps3-an536 has peripherals, but the GIC can pend any SPI from
 * software (GICD_ISPENDR), which is the interrupt a device would raise as far
 * as everything after the distributor is concerned, and needs no device. The
 * test configures the SPI the way an application would: Group 1, routed to
 * core 0, at the priority the port gives the timer (the most urgent a device
 * may have and still be masked only by an interrupt mask), and enabled.
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
 * The GIC, from PERIPHBASE (IMP_CBAR)
 * ----------------------------------------------------------------------- */

constexpr std::uint32_t spi = 100u;            /* no AN536 device drives it */
constexpr std::uint32_t timer_intid = 30u;

std::uint32_t read_cbar() { std::uint32_t v; asm volatile("mrc p15, 1, %0, c15, c3, 0" : "=r"(v)); return v; }
std::uintptr_t distributor() { return read_cbar() & 0xFFE00000u; }
std::uintptr_t sgi_frame0()  { return distributor() + 0x100000u + 0x10000u; }

volatile std::uint32_t& reg(std::uintptr_t address) { return *reinterpret_cast<volatile std::uint32_t*>(address); }
volatile std::uint8_t& reg8(std::uintptr_t address) { return *reinterpret_cast<volatile std::uint8_t*>(address); }

std::uint32_t const word = spi / 32u;
std::uint32_t const bit  = 1u << (spi % 32u);

void spi_enable()
{
   std::uintptr_t const d = distributor();
   reg(d + 0x080u + 4u * word) = reg(d + 0x080u + 4u * word) | bit;     /* IGROUPR: Group 1  */
   reg8(d + 0x400u + spi) = reg8(sgi_frame0() + 0x400u + timer_intid);   /* IPRIORITYR         */
   reg(d + 0x6000u + 8u * spi) = 0u;                                      /* IROUTER: Aff 0.0.0 */
   reg(d + 0x6000u + 8u * spi + 4u) = 0u;
   reg(d + 0x280u + 4u * word) = bit;                                     /* ICPENDR            */
   reg(d + 0x100u + 4u * word) = bit;                                     /* ISENABLER          */
   asm volatile("dsb sy" ::: "memory");
}

void spi_pend()
{
   reg(distributor() + 0x200u + 4u * word) = bit;                         /* ISPENDR */
   asm volatile("dsb sy \n isb" ::: "memory");
}

bool spi_pending()
{
   return (reg(distributor() + 0x200u + 4u * word) & bit) != 0u;
}

void spin_for(std::uint32_t loops)
{
   for (std::uint32_t i = 0; i < loops; ++i) { asm volatile("nop"); }
}

/* --------------------------------------------------------------------------
 * The handler
 * ----------------------------------------------------------------------- */

volatile std::uint32_t fired = 0u;
volatile std::uint32_t last_intid = 0u;
volatile std::uint64_t fired_at = 0u;
volatile bool release_from_isr = false;
sync::semaphore wake{0};

}  // namespace

extern "C" void cyros_armv8r_device_interrupt(std::uint32_t intid)
{
   fired_at = cyros_port_timestamp();
   last_intid = intid;
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
   spi_enable();

   cyros::bench::start("a pended SPI reaches cyros_armv8r_device_interrupt, with its INTID");
   spi_pend();
   spin_for(2000u);
   CYROS_CHECK_EQ(fired, 1u);
   CYROS_CHECK_EQ(last_intid, spi);
   CYROS_CHECK(!spi_pending());

   cyros::bench::start("it runs inside a preempt-disable");
   {
      cyros_mask_token_t const token = cyros_port_preempt_disable();
      spi_pend();
      spin_for(2000u);
      std::uint32_t const during = fired;
      cyros_port_preempt_enable(token);
      CYROS_CHECK_EQ(during, 2u);
   }

   cyros::bench::start("it waits out an interrupt mask, then runs, once");
   {
      cyros_mask_token_t const token = cyros_port_irq_save();
      spi_pend();
      spin_for(2000u);
      std::uint32_t const during = fired;
      bool const pending = spi_pending();
      cyros_port_irq_restore(token);
      spin_for(2000u);
      CYROS_CHECK_EQ(during, 2u);
      CYROS_CHECK(pending);
      CYROS_CHECK_EQ(fired, 3u);
   }

   cyros::bench::start("a release from it wakes a blocked thread");
   release_from_isr = true;
   std::uint64_t best = ~std::uint64_t{0};
   std::uint64_t worst = 0u;
   for (int r = 0; r < rounds; ++r) {
      int const before = woken;
      spi_pend();
      for (std::uint32_t spin = 0; woken == before && spin < 1'000'000u; ++spin) {}
      if (woken != before) {
         std::uint64_t const cost = woke_at - fired_at;
         best = cost < best ? cost : best;
         worst = cost > worst ? cost : worst;
      }
   }
   CYROS_CHECK_EQ(woken, rounds);
   CYROS_CHECK_EQ(wake.peek(), 0);
   CYROS_CHECK_EQ(fired, 3u + rounds);
   cyros::bench::print("  timestamp counts from the handler's stamp to the woken thread's, best and worst of 16: ");
   cyros::bench::print_dec(best);
   cyros::bench::print(" to ");
   cyros::bench::print_dec(worst);
   cyros::bench::print("\n");

   cyros::bench::finish();
}

}  // namespace

extern "C" int cyros_bench_main()
{
   cyros::bench::print("armv8r device interrupt\n\n");

   kernel::initialise();
   thread w(waiter, waiter_stack, thread::priority(1), core0);
   thread d(driver, driver_stack, thread::priority(2), core0);
   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
