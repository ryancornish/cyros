/**
 * @file test_cortex_m_stack_guard.cpp
 * @brief The core layer's stack guard, on whichever architecture this is built for.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the armv7m_armv8m core layer's stack guard, programmed in
 *          cyros_port_context_init and reloaded on every cyros_port_switch.
 *          PSPLIM on ARMv8-M, the highest-numbered MPU region on ARMv7-M.
 * Trusts:  layer 0 and the kernel bring-up layer 2 proves.
 * Proves:  that the guard covers the bottom of the running thread's own
 *          buffer, that ordinary stack use does not trip it, and that
 *          overrunning the buffer faults rather than corrupting whatever lies
 *          below: UsageFault with STKOF on ARMv8-M, MemManage with DACCVIOL and
 *          the faulting address inside the guard on ARMv7-M.
 *
 *
 * WHY THIS MATTERS MORE HERE THAN ON MOST RTOSes
 * ==============================================
 * In cyros THE CALLER OWNS THE STACK. A thread's stack is a buffer the user
 * declared, and the TCB is carved out of the top of that same buffer. So an
 * overrun does not hit a guard page, it walks into whatever the application
 * put below it in .bss, silently. The ~thread() assert catches a stack going
 * out of scope while its thread runs, which is lifetime, not depth.
 *
 * The test exists in this form because the guard can vanish without a build
 * error. gas assembles `msr psplim` for an ARMv7-M -mcpu without a word and
 * the silicon ignores it: an M4 build of the ARMv8-M-only port ran with no
 * guard, and on a TM4C123 a deliberate overrun produced no fault at all. This
 * test is what notices.
 *
 *
 * THE TWO MECHANISMS, AND WHAT THE TEST CAN SEE OF EACH
 * =====================================================
 * PSPLIM is checked on the SP update itself, so the `sub sp` faults before
 * anything is written through the bad pointer. The MPU polices addresses, not
 * the stack pointer, so on ARMv7-M the `sub` is legal and the STORE into the
 * guard faults. MSTKERR is usually set as well: the fault's own exception entry
 * tries to stack onto the overrun stack, and the guard stops that too.
 *
 * A Mainline part without an MPU runs unguarded, by policy (no software
 * stand-in). There the test proves only that the thread runs on its own
 * buffer, and says so.
 *
 *
 * HOW IT ASSERTS A FAULT
 * ======================
 * The bench's startup declares each fault vector as its own weak symbol
 * aliased to a reporting handler. This test replaces UsageFault_Handler and
 * MemManage_Handler and nothing else, so a BusFault raised by a mistake here
 * still reports as one and cannot be mistaken for success. Each handler
 * accepts only the fault this architecture's guard raises.
 *
 * The handler is where the test ENDS: it records the result and exits the
 * image, because there is nothing sensible to return to once a thread has run
 * off the bottom of its stack.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <cstddef>
#include <cstdint>

using namespace cyros;

/* The same selection the core layer makes (cortex_m.hpp), repeated rather than
 * included because that header is private to the port. */
#if defined(__ARM_ARCH_8M_MAIN__)
#  define GUARD_IS_PSPLIM 1
#elif defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_7M__)
#  define GUARD_IS_PSPLIM 0
#else
#  error "test_cortex_m_stack_guard knows ARMv7-M and ARMv8-M Mainline only"
#endif

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 2048;
alignas(CYROS_PORT_STACK_ALIGN) std::byte worker_stack[stack_size];

constexpr std::uintptr_t scb_cfsr  = 0xE000ED28u;
constexpr std::uintptr_t scb_mmfar = 0xE000ED34u;

/* CFSR: MMFSR in bits [7:0], UFSR in [31:16]. STKOF is ARMv8-M only. */
constexpr std::uint32_t cfsr_daccviol  = 1u << 1;
constexpr std::uint32_t cfsr_mmarvalid = 1u << 7;
constexpr std::uint32_t cfsr_stkof     = 1u << 20;

/* PMSAv7, for reading the ARMv7-M guard back. */
constexpr std::uintptr_t mpu_type = 0xE000ED90u;
constexpr std::uintptr_t mpu_rnr  = 0xE000ED98u;
constexpr std::uintptr_t mpu_rbar = 0xE000ED9Cu;
constexpr std::uintptr_t mpu_rasr = 0xE000EDA0u;
constexpr std::uint32_t  guard_bytes = 128u;

/* Set immediately before the deliberate overrun. A handler refuses to treat a
 * fault as success unless the test asked for one, so an accidental fault
 * anywhere else in this file still reports as a failure. */
volatile bool expecting_overflow = false;

/* The guard's bottom, as read back from the hardware: PSPLIM, or the MPU
 * region's base. The overrun aims at it. */
volatile std::uint32_t guard_bottom = 0u;

std::uint32_t reg(std::uintptr_t address)
{
   return *reinterpret_cast<volatile std::uint32_t*>(address);
}

std::uint32_t read_sp()
{
   std::uint32_t value;
   asm volatile("mov %0, sp" : "=r"(value));
   return value;
}

#if GUARD_IS_PSPLIM
std::uint32_t read_psplim()
{
   std::uint32_t value;
   asm volatile("mrs %0, psplim" : "=r"(value) :: "memory");
   return value;
}
#else
std::uint32_t mpu_regions() { return (reg(mpu_type) >> 8) & 0xFFu; }
#endif

/* The lowest address the running thread may use, or 0 when there is no guard. */
std::uint32_t guard_top()
{
#if GUARD_IS_PSPLIM
   return guard_bottom;
#else
   return guard_bottom == 0u ? 0u : guard_bottom + guard_bytes;
#endif
}


void test_the_guard_covers_this_threads_buffer()
{
   cyros::bench::start("the guard covers the bottom of this thread's own buffer");

   auto const base = reinterpret_cast<std::uint32_t>(worker_stack);
   std::uint32_t const top = base + stack_size;
   std::uint32_t const sp  = read_sp();

   /* Whatever the guard, the thread must be running on its own buffer. */
   CYROS_CHECK(sp > base);
   CYROS_CHECK(sp <= top);

#if GUARD_IS_PSPLIM
   guard_bottom = read_psplim();
   CYROS_CHECK(guard_bottom != 0u);
#else
   if (mpu_regions() == 0u) {
      cyros::bench::print("  no MPU: this port runs unguarded here, by policy\n");
      return;
   }

   /* The highest-numbered region, so that no application region can override
    * it. Selecting it through RNR is a write, but to a register only the port's
    * next switch uses, and that switch writes it again. */
   *reinterpret_cast<volatile std::uint32_t*>(mpu_rnr) = mpu_regions() - 1u;
   std::uint32_t const rbar = reg(mpu_rbar);
   std::uint32_t const rasr = reg(mpu_rasr);
   guard_bottom = rbar & ~(guard_bytes - 1u);

   CYROS_CHECK((rasr & 1u) != 0u);                 /* enabled                   */
   CYROS_CHECK_EQ((rasr >> 1) & 0x1Fu, 6u);        /* 2^(6+1) = 128 bytes       */
   CYROS_CHECK_EQ((rasr >> 24) & 0x7u, 0u);        /* AP: no access at all      */
   CYROS_CHECK((rasr & (1u << 28)) != 0u);         /* XN                        */
#endif

   CYROS_CHECK(guard_bottom >= base);
   CYROS_CHECK(guard_top() < top);
   CYROS_CHECK(sp > guard_top());

   cyros::bench::print("  stack base = ");
   cyros::bench::print_hex(base);
   cyros::bench::print(GUARD_IS_PSPLIM ? "\n  psplim     = " : "\n  guard      = ");
   cyros::bench::print_hex(guard_bottom);
   cyros::bench::print("\n  sp         = ");
   cyros::bench::print_hex(sp);
   cyros::bench::print("\n");
}

void test_ordinary_stack_use_does_not_trip_it()
{
   cyros::bench::start("ordinary stack use does not trip the guard");

   /* Comfortably inside the buffer. If the guard were set to the wrong end, or
    * to the top of the stack, this would fault and the test would never get to
    * the interesting part. */
   volatile std::uint8_t scratch[512];
   for (std::size_t i = 0; i < sizeof(scratch); ++i) {
      scratch[i] = static_cast<std::uint8_t>(i);
   }

   std::uint32_t sum = 0;
   for (std::size_t i = 0; i < sizeof(scratch); ++i) { sum += scratch[i]; }

   CYROS_CHECK(sum != 0u);
   CYROS_CHECK(read_sp() > guard_top());
}

[[noreturn]] void overrun_the_stack()
{
   cyros::bench::start("running off the bottom of the stack faults");

   if (guard_bottom == 0u) {
      cyros::bench::print("  skipped: there is no guard to run into\n");
      cyros::bench::finish();
   }

   expecting_overflow = true;

   /* Move SP into the guard in one step and store there. A recursive function
    * would also work but its depth depends on the optimiser, and this has to
    * be deterministic: the success condition is that the overrun faults.
    *
    * The distance is COMPUTED rather than a constant, because a fixed one has
    * to be larger than the stack to be sure of crossing, and too small a one
    * leaves SP above the guard so nothing faults and a working guard looks
    * broken. PSPLIM faults on the SUB, 256 bytes past its limit. The MPU
    * faults on the STR, 16 bytes inside the guard. */
#if GUARD_IS_PSPLIM
   std::uint32_t const drop = (read_sp() - guard_bottom) + 256u;
#else
   std::uint32_t const drop = read_sp() - (guard_top() - 16u);
#endif

   asm volatile(
      "sub sp, sp, %0  \n"
      "str r0, [sp]    \n"
      :: "r"(drop) : "memory");

   /* Unreachable if the guard works, which is the assertion. */
   cyros::bench::record(false, "stack overrun did not fault", __FILE__, __LINE__);
   cyros::bench::print(
      "  SP went past the guard and nothing happened. Either the port is not\n"
      "  programming it, or the fault is not enabled in SHCSR.\n");
   cyros::bench::finish();
}

void worker()
{
   test_the_guard_covers_this_threads_buffer();
   test_ordinary_stack_use_does_not_trip_it();
   overrun_the_stack();
}

[[noreturn]] void report_fault(char const* which, bool is_this_architectures_guard)
{
   std::uint32_t const cfsr = reg(scb_cfsr);

   if (!expecting_overflow || !is_this_architectures_guard) {
      cyros::bench::print("\n*** unexpected ");
      cyros::bench::print(which);
      cyros::bench::print(", CFSR = ");
      cyros::bench::print_hex(cfsr);
      cyros::bench::print(" ***\n");
      cyros::bench::host_exit(3u);
   }

#if GUARD_IS_PSPLIM
   cyros::bench::record((cfsr & cfsr_stkof) != 0u, "UsageFault reports CFSR.STKOF", __FILE__, __LINE__);
#else
   std::uint32_t const mmfar = reg(scb_mmfar);
   cyros::bench::record((cfsr & cfsr_daccviol) != 0u, "MemManage reports CFSR.DACCVIOL", __FILE__, __LINE__);
   cyros::bench::record((cfsr & cfsr_mmarvalid) != 0u, "MemManage reports CFSR.MMARVALID", __FILE__, __LINE__);
   cyros::bench::record(mmfar >= guard_bottom && mmfar < guard_top(),
                        "the faulting address is inside the guard", __FILE__, __LINE__);
   cyros::bench::print("  MMFAR = ");
   cyros::bench::print_hex(mmfar);
   cyros::bench::print("\n");
#endif
   cyros::bench::print("  CFSR  = ");
   cyros::bench::print_hex(cfsr);
   cyros::bench::print("\n");

   cyros::bench::finish();
}

} // namespace


/* Both replace the bench's weak defaults and run on MSP, so they have a working
 * stack even though the thread's is exhausted, which is exactly why the split
 * MSP/PSP model is worth having. */
extern "C" [[noreturn]] void UsageFault_Handler(void)
{
   report_fault("UsageFault", GUARD_IS_PSPLIM == 1);
}

extern "C" [[noreturn]] void MemManage_Handler(void)
{
   report_fault("MemManage", GUARD_IS_PSPLIM == 0);
}


extern "C" int cyros_bench_main()
{
   cyros::bench::print("cortex_m stack guard\n\n");

   kernel::initialise();
   thread worker_thread(worker, worker_stack, thread::priority(0), core0);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
