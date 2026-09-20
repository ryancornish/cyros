/**
 * @file test_core_primitives.cpp
 * @brief cyros/kernel/core.hpp, the public core primitives (L1)
 *
 * Subject:  cyros/kernel/core.hpp, the public core primitives (L1)
 * Trusts:   the port contract (L0), plus the harness floor to run cores at all, which is a declared debt
 * Proves:   this_core::cpu_relax() is a usable spin hint against a store from another core
 *
 * Created 2026-09-20. core.hpp was graded layer 1 by the T1 policy and had no
 * test of its own, the same gap spinlock had until 2026-09-19 and the same
 * argument for filling it: a primitive covered only from above is not covered
 * where it matters. It starts with the cpu_relax case moved out of
 * test_multicore_multithread, and enter_critical/exit_critical belong here too
 * when someone writes them.
 */
#include <cyros/kernel/core.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <common/guarded_stack.hpp>

#include "gtest/gtest.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

using namespace cyros;

static_assert(config::cores >= 4, "Test suite is designed for (atleast) quad core configuration only");

static constexpr auto STACK_SIZE = thread::min_stack_size + (16 * 1024);

int main(int argc, char** argv)
{
   ::testing::InitGoogleTest(&argc, argv);

   int result = RUN_ALL_TESTS();

   return result;
}


class CorePrimitives_Test : public ::testing::Test
{
   void SetUp() override
   {
      kernel::initialise();
   }

   void TearDown() override
   {
      kernel::finalise();
   }
};


/* ============================================================================
 * this_core::cpu_relax is the public spin-wait hint
 *
 * Added 2026-09-19 with the port header going project-internal (roadmap A1):
 * user code that spins used to reach for cyros_port_cpu_relax through port.h,
 * which consumers can no longer see. This pins the documented usage, a spin on
 * a condition ANOTHER core satisfies, through the public surface only. The
 * spinner and the setter are on different cores on purpose: cpu_relax is a
 * hardware hint and does not yield, so a same-core setter would be starved.
 * ========================================================================= */
TEST_F(CorePrimitives_Test,
       GivenAFlagSetOnAnotherCore_WhenSpinningWithCpuRelax_ThenTheSpinSeesIt)
{
   std::array<cyros::test::guarded_stack, 2> stacks;

   std::atomic<bool> flag{false};
   std::atomic<std::uint64_t> spins{0};
   std::atomic<bool> released{false};

   thread spinner(
      [&]{
         while (!flag.load(std::memory_order_acquire)) {
            this_core::cpu_relax();
            spins.fetch_add(1, std::memory_order_relaxed);
         }
         released.store(true, std::memory_order_release);
      },
      stacks[0], thread::priority(0), core1
   );

   thread setter(
      [&]{ flag.store(true, std::memory_order_release); },
      stacks[1], thread::priority(0), core0
   );

   kernel::start();

   EXPECT_TRUE(released.load()) << "the spin never observed the cross-core flag";
   EXPECT_EQ(kernel::active_threads(), 0u);
}
