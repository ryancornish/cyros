/**
 * @file test_cortex_m33_smp_bringup.cpp
 * @brief Two Cortex-M33 cores, one kernel (L2).
 *
 * Subject: multicore bring-up on a bare-metal target
 * Trusts:  the single-core port (every test in this suite below layer 2)
 * Proves:  the second core is released, joins the same kernel, runs a thread
 *          pinned to it, and reports its own identity from inside that thread
 *
 * The single-core bench cannot reach any of this. `cyros_port_get_core_id`,
 * `cyros_port_start_cores` and `cyros_port_send_reschedule_ipi` are the three
 * functions that are trivially correct on one core and are the whole of the
 * MCU contract's multicore half, so until a second core exists they are
 * untested by construction.
 *
 *
 * How a two-core test ENDS, which is the part that needs care
 * ==========================================================
 * `kernel::start()` never returns on this port, so a test reports from inside
 * its last thread. With two cores "last" is not a property either core can
 * observe on its own, so one core is nominated to report and it waits for the
 * other to say it is done.
 *
 * That wait is the documented trap on this bench: a cross-core rendezvous must
 * only ever wait on a value the other core leaves STABLE, never on an
 * intermediate state the other core races past. `core1_finished` is set once
 * and never cleared, so no amount of speed on either side can make the waiter
 * miss it.
 *
 * The wait is bounded. A rendezvous that never completes would otherwise hang
 * until the runner's timeout kills it, and a timeout says only that something
 * is wrong, not what.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <common/arm/bench.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>

using namespace cyros;

static_assert(config::cores == 2, "This bring-up is dual core");
static_assert(CYROS_PORT_CORE_COUNT == 2, "Needs the cortex_m33_smp port");

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 2048;

alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_core0[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_core1[stack_size];

/* Written by one core and read by the other, so atomics rather than the plain
 * ints the single-core bring-up uses. Relaxed would be enough on this bench,
 * because QEMU does not model ARM's weak memory and would never reorder
 * anything, and that is exactly the reason to write the ordering the hardware
 * needs instead: the bench cannot catch a missing barrier, so nothing here
 * should be tuned to what the bench can see. */
std::atomic<std::uint32_t> core0_saw_id{0xFFFFFFFFu};
std::atomic<std::uint32_t> core1_saw_id{0xFFFFFFFFu};
std::atomic<bool>          core1_finished{false};

/* --------------------------------------------------------------------------
 * Core-private registers, read directly rather than through the port
 *
 * Every register the port's per-core init touches lives in the System Control
 * Space, which each core has its own copy of AT THE SAME ADDRESS. A secondary
 * core that never ran that init therefore comes up holding reset defaults
 * while core 0 holds derived values, and the two cores disagree about what
 * PendSV's priority is.
 *
 * Nothing else in a bring-up test notices. The thread on core 1 does too
 * little to care what PendSV's priority is, so the image runs and passes with
 * a second core that was never initialised. That was measured, by deleting the
 * init call and watching every other check here still pass, which is why these
 * two exist.
 *
 * The consequence of not noticing is not cosmetic. PendSV's reset priority is
 * 0, the most urgent there is, and the port's preempt-disable works by raising
 * BASEPRI to PendSV's DERIVED priority. A core whose PendSV sits at 0 is not
 * masked by that, so a context switch can be taken in the middle of a kernel
 * critical section on that core alone.
 * ----------------------------------------------------------------------- */

constexpr std::uintptr_t shpr_pendsv_addr = 0xE000ED22u;
constexpr std::uintptr_t cpacr_addr       = 0xE000ED88u;
constexpr std::uint32_t  cpacr_fpu_bits   = (0x3u << 20) | (0x3u << 22);

std::uint8_t read_byte(std::uintptr_t address)
{
   return *reinterpret_cast<volatile std::uint8_t*>(address);
}

std::uint32_t read_word(std::uintptr_t address)
{
   return *reinterpret_cast<volatile std::uint32_t*>(address);
}

std::atomic<std::uint32_t> core0_pendsv_priority{0xFFFFFFFFu};
std::atomic<std::uint32_t> core1_pendsv_priority{0xFFFFFFFFu};
std::atomic<std::uint32_t> core1_cpacr{0u};

/* Bounded, so a rendezvous that never completes REPORTS rather than hanging
 * until the runner's timeout. A timeout says only that something is wrong; a
 * failed check names which thing.
 *
 * The bound counts yields, not spins, and a yield here costs a full trip
 * through the scheduler. That is what sets the size: it has to be small enough
 * to exhaust well inside the runner's timeout, which rules out the very large
 * round numbers a raw spin loop could afford. Verified by deleting the release
 * of the second core, which makes this loop run to its end and report in about
 * a second. Releasing the core needs a handful of iterations at most, so the
 * margin is five orders of magnitude.
 */
constexpr std::uint32_t rendezvous_limit = 100'000u;

bool wait_for_core1()
{
   for (std::uint32_t spin = 0; spin < rendezvous_limit; ++spin) {
      if (core1_finished.load(std::memory_order_acquire)) { return true; }
      this_thread::yield();
   }
   return false;
}

/* --------------------------------------------------------------------------
 * The thread pinned to core 1
 * ----------------------------------------------------------------------- */

void thread_on_core1()
{
   core1_saw_id.store(this_core::id(), std::memory_order_relaxed);

   /* Read from THIS core, which is the whole point: the same addresses read
    * from core 0 would answer for core 0. */
   core1_pendsv_priority.store(read_byte(shpr_pendsv_addr), std::memory_order_relaxed);
   core1_cpacr.store(read_word(cpacr_addr), std::memory_order_relaxed);

   /* Release, and the store above is the reason. The reporting core reads
    * core1_saw_id only after seeing this flag, so this is what makes that read
    * well defined rather than a race that happens to work. */
   core1_finished.store(true, std::memory_order_release);
}

/* --------------------------------------------------------------------------
 * The thread pinned to core 0, which reports for the whole image
 * ----------------------------------------------------------------------- */

void thread_on_core0()
{
   core0_saw_id.store(this_core::id(), std::memory_order_relaxed);
   core0_pendsv_priority.store(read_byte(shpr_pendsv_addr), std::memory_order_relaxed);

   bool const rendezvous = wait_for_core1();

   cyros::bench::start("the second core reached its thread");
   CYROS_CHECK(rendezvous);

   cyros::bench::start("the kernel reports two cores");
   CYROS_CHECK_EQ(kernel::core_count(), 2u);

   /* The claim this whole target exists to support. Two threads, two cores,
    * one kernel image, and each thread reports the core it is actually
    * executing on. A single-core port passes every other check in this file
    * and fails this one. */
   cyros::bench::start("each thread runs on the core it was pinned to");
   cyros::bench::print("  core 0 thread saw id ");
   cyros::bench::print_hex(core0_saw_id.load(std::memory_order_relaxed));
   cyros::bench::print("\n  core 1 thread saw id ");
   cyros::bench::print_hex(core1_saw_id.load(std::memory_order_relaxed));
   cyros::bench::print("\n");
   CYROS_CHECK_EQ(core0_saw_id.load(std::memory_order_relaxed), 0u);
   CYROS_CHECK_EQ(core1_saw_id.load(std::memory_order_relaxed), 1u);

   /* The secondary core ran the port's per-core init, not just the kernel.
    * Both cores must have derived the SAME PendSV priority, and it must not be
    * the reset default, or the preempt-disable does not mask PendSV there. */
   std::uint32_t const p0 = core0_pendsv_priority.load(std::memory_order_relaxed);
   std::uint32_t const p1 = core1_pendsv_priority.load(std::memory_order_relaxed);

   cyros::bench::start("the second core initialised its own system handler priorities");
   cyros::bench::print("  core 0 PendSV priority ");
   cyros::bench::print_hex(p0);
   cyros::bench::print("\n  core 1 PendSV priority ");
   cyros::bench::print_hex(p1);
   cyros::bench::print("\n");
   CYROS_CHECK(p0 != 0u);        /* the reset default, which means never set */
   CYROS_CHECK_EQ(p1, p0);

   cyros::bench::start("the second core enabled its own FPU");
   CYROS_CHECK_EQ(core1_cpacr.load(std::memory_order_relaxed) & cpacr_fpu_bits,
                  cpacr_fpu_bits);

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("cortex_m33_smp kernel bring-up, mps2-an521\n\n");

   /* Before the kernel starts, only core 0 is running: CPU1 is held at reset
    * until cyros_port_start_cores releases it. So this reads 0 for the same
    * reason it would on a single-core part, and proving the identity register
    * works has to happen from inside a thread on the other core. */
   cyros::bench::start("the bootstrap core identifies itself as core 0");
   CYROS_CHECK_EQ(this_core::id(), 0u);

   kernel::initialise();

   cyros::bench::start("one thread pinned to each core registers");
   thread t0(thread_on_core0, stack_core0, thread::priority(0), core0);
   thread t1(thread_on_core1, stack_core1, thread::priority(0), core1);
   CYROS_CHECK_EQ(kernel::active_threads(), 2u);

   /* Does not return on this port. thread_on_core0 exits the image. */
   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
