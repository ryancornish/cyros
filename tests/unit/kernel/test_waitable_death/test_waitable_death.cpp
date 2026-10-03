/**
 * @file test_waitable_death.cpp
 * @brief Misusing a waiter_record stops the system rather than misrouting a wait.
 *
 * Subject: the misuse checks in waiter_record (L4)
 * Trusts:  the kernel harness (L2), to give the misuse a current thread
 *
 * A thread holds one installed record, in its TCB. A second record on a thread
 * that already has one would silently replace the first, and the first's
 * waitable would get the wrong record for the rest of the wait.
 *
 * (Destroying a record uninstalls it from the thread that built it, wherever
 * the destructor runs, so there is no second misuse to catch there.)
 *
 * Every test requires the panic's file and failing line as well as SIGABRT
 * (`common/death.hpp`).
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/waitable.hpp>

#include <common/death.hpp>
#include <common/guarded_stack.hpp>

#include <gtest/gtest.h>


using namespace cyros;

namespace
{

class plain_waitable final : public waitable
{
protected:
   bool try_satisfy(waiter_record*) noexcept override { return true; }
};

struct record : waiter_record
{
   explicit record(waitable& w) noexcept : waiter_record(w) {}
};

void install_two_records_on_one_thread()
{
   kernel::initialise();
   static test::guarded_stack stack;
   thread t([] {
         static plain_waitable w;
         record first(w);
         record second(w);
      },
      stack, thread::priority(1), core0);
   kernel::start();
   kernel::finalise();
}

}  // namespace

TEST(WaitableDeath_Test, GivenAThreadWithARecord_WhenASecondIsBuilt_ThenItStopsTheSystem)
{
   CYROS_EXPECT_PANIC(install_two_records_on_one_thread(),
                      test::panicked_at("waitable.cpp", "One record per thread"));
}
