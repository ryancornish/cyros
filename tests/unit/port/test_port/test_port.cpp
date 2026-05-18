/**
 * @file test_port.cpp
 * @brief Unit tests for port layer (boost.context backend)
 */

#include <cortos/port/port.h>

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>


static_assert(CORTOS_PORT_CORE_COUNT > 0);

// ----------------------------------------------------------------------------
// Test harness: per-core scheduler loop driven by port->kernel hook.
// ----------------------------------------------------------------------------


// Global pointer used by the C reschedule hook installed via cortos_port_init().
static constinit struct PortHarness* g_harness = nullptr;

struct PortHarness
{
   static constexpr std::size_t MAX_READY = 32;

   // Simple single-core ready queue (FIFO). Enough for port-level tests.
   std::array<cortos_port_context_t*, MAX_READY> ready{};
   std::size_t r_head{0};
   std::size_t r_tail{0};

   // Track what the harness believes is currently running (optional / informational).
   cortos_port_context_t* current{nullptr};

   void reset() noexcept
   {
      r_head = r_tail = 0;
      current = nullptr;
      ready.fill(nullptr);
   }

   bool empty() const noexcept { return r_head == r_tail; }

   std::size_t size() const noexcept { return r_tail - r_head; }

   void enqueue(cortos_port_context_t* ctx)
   {
      ASSERT_NE(ctx, nullptr);
      ASSERT_LT(size(), MAX_READY) << "Ready queue overflow in test harness";
      ready[r_tail % MAX_READY] = ctx;
      ++r_tail;
   }

   cortos_port_context_t* dequeue() noexcept
   {
      if (empty()) return nullptr;
      auto* ctx = ready[r_head % MAX_READY];
      ready[r_head % MAX_READY] = nullptr;
      ++r_head;
      return ctx;
   }

   // Called by the port via cortos_port_pend_reschedule_handler.
   void reschedule_once()
   {
      auto* next = dequeue();
      if (!next) return;

      // In these unit tests we always resume threads from the test's OS fiber.
      // That means the "caller" captured by Boost for each thread is the test thread,
      // and cortos_port_pend_reschedule() returns control back here.
      if (!current) {
         current = next;
         cortos_port_start_first(next);
         // When the thread yields/exits, we return here.
         // `current` is only "best effort" bookkeeping; the thread may have exited.
         current = nullptr;
         return;
      }

      auto* prev = current;
      current = next;
      cortos_port_switch(prev, next);
      current = nullptr;
   }

   // Drive until no ready threads remain (or step limit hit).
   void run_until_quiescent(std::size_t step_limit = 10'000)
   {
      for (std::size_t i = 0; i < step_limit; ++i) {
         if (empty()) return;
         reschedule_once();
      }
      FAIL() << "Harness did not quiesce within step limit (possible missing yield/enqueue)";
   }

   // Hook trampoline.
   static void reschedule_hook()
   {
      ASSERT_NE(g_harness, nullptr);
      g_harness->reschedule_once();
   }
};

struct ThreadArg
{
   PortHarness* harness{};
   cortos_port_context_t* ctx{};
   std::atomic<int>* stage_counter{};
   int stages{};
   int thread_id{};
   std::array<int, 64>* trace{};
   std::atomic<int>* trace_len{};
};

// A thread that records its "thread_id" into trace, yields `stages` times, then returns.
static void thread_yield_n(void* varg)
{
   auto* a = static_cast<ThreadArg*>(varg);
   ASSERT_NE(a, nullptr);
   ASSERT_NE(a->harness, nullptr);
   ASSERT_NE(a->ctx, nullptr);

   for (int i = 0; i < a->stages; ++i) {
      // Record progress
      int idx = a->trace_len->fetch_add(1, std::memory_order_acq_rel);
      ASSERT_LT(idx, static_cast<int>(a->trace->size()));
      (*a->trace)[static_cast<std::size_t>(idx)] = a->thread_id;

      a->stage_counter->fetch_add(1, std::memory_order_acq_rel);

      // Cooperative yield: put ourselves back on the ready queue, then yield to caller.
      a->harness->enqueue(a->ctx);
      cortos_port_pend_reschedule();
   }

   // Return = thread exits; port test harness regains control.
}

// A thread that runs once and returns (no yield).
static void thread_run_once(void* varg)
{
   auto* ran = static_cast<std::atomic<bool>*>(varg);
   ASSERT_NE(ran, nullptr);
   ran->store(true, std::memory_order_release);
}

class PortTest : public ::testing::Test
{
protected:
   PortHarness port_harness{};

   void SetUp() override
   {
      port_harness.reset();
      g_harness = &port_harness;
      cortos_port_init(&PortHarness::reschedule_hook);
   }

   void TearDown() override
   {
      g_harness = nullptr;
   }
};





TEST_F(PortTest, GivenInterrupts_WhenDisableEnableNested_ThenInterruptsEnabledTracksDepth)
{
   EXPECT_TRUE(cortos_port_interrupts_enabled());

   cortos_port_disable_interrupts();
   EXPECT_FALSE(cortos_port_interrupts_enabled());

   cortos_port_disable_interrupts();
   EXPECT_FALSE(cortos_port_interrupts_enabled());

   cortos_port_enable_interrupts();
   EXPECT_FALSE(cortos_port_interrupts_enabled());

   cortos_port_enable_interrupts();
   EXPECT_TRUE(cortos_port_interrupts_enabled());
}

TEST_F(PortTest, GivenIrqSaveRestore_WhenNested_ThenRestoreReturnsToPriorState)
{
   EXPECT_TRUE(cortos_port_interrupts_enabled());

   uint32_t s0 = cortos_port_irq_save();
   EXPECT_EQ(s0, 1u);
   EXPECT_FALSE(cortos_port_interrupts_enabled());

   uint32_t s1 = cortos_port_irq_save();
   EXPECT_EQ(s1, 0u);
   EXPECT_FALSE(cortos_port_interrupts_enabled());

   // Restore one level (still disabled)
   cortos_port_irq_restore(s1);
   EXPECT_FALSE(cortos_port_interrupts_enabled());

   // Restore to original enabled state
   cortos_port_irq_restore(s0);
   EXPECT_TRUE(cortos_port_interrupts_enabled());
}

TEST_F(PortTest, GivenTlsPointer_WhenSetThenGet_ThenValueRoundTrips)
{
   int x = 123;
   cortos_port_set_tls_pointer(&x);
   EXPECT_EQ(cortos_port_get_tls_pointer(), &x);

   cortos_port_set_tls_pointer(nullptr);
   EXPECT_EQ(cortos_port_get_tls_pointer(), nullptr);
}

TEST_F(PortTest, GivenGetCoreId_WhenNotStartedCores_ThenIsZero)
{
   // In the Linux boost port, core_id defaults to 0 on the calling thread unless start_cores sets it.
   EXPECT_EQ(cortos_port_get_core_id(), 0u);
}

TEST_F(PortTest, GivenSingleThread_WhenStartFirst_ThenThreadRunsAndReturnsToCaller)
{
   alignas(CORTOS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> stack{};
   alignas(CORTOS_PORT_CONTEXT_ALIGN) static std::array<std::byte, CORTOS_PORT_CONTEXT_SIZE> ctx_storage{};

   auto* ctx = reinterpret_cast<cortos_port_context_t*>(ctx_storage.data());

   std::atomic<bool> ran{false};

   cortos_port_context_init(
      ctx,
      stack.data(),
      stack.size(),
      &thread_run_once,
      &ran
   );

   // Schedule it manually via the harness
   port_harness.enqueue(ctx);
   port_harness.run_until_quiescent();

   EXPECT_TRUE(ran.load(std::memory_order_acquire));

   // Thread should have completed and returned control to us.
   // (The port's context_destroy is owned by the kernel normally, but port-level test can still destroy.)
   cortos_port_context_destroy(ctx);
}

TEST_F(PortTest, GivenTwoThreads_WhenTheyYieldAndReenqueue_ThenTheyInterleaveInFIFOOrder)
{
   alignas(CORTOS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> s0{};
   alignas(CORTOS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> s1{};

   alignas(CORTOS_PORT_CONTEXT_ALIGN) static std::array<std::byte, CORTOS_PORT_CONTEXT_SIZE> c0_storage{};
   alignas(CORTOS_PORT_CONTEXT_ALIGN) static std::array<std::byte, CORTOS_PORT_CONTEXT_SIZE> c1_storage{};

   auto* c0 = reinterpret_cast<cortos_port_context_t*>(c0_storage.data());
   auto* c1 = reinterpret_cast<cortos_port_context_t*>(c1_storage.data());

   std::atomic<int> stages_done{0};
   std::atomic<int> trace_len{0};
   std::array<int, 64> trace{};
   trace.fill(-1);

   ThreadArg a0{
      .harness = &port_harness,
      .ctx = c0,
      .stage_counter = &stages_done,
      .stages = 3,
      .thread_id = 0,
      .trace = &trace,
      .trace_len = &trace_len,
   };

   ThreadArg a1{
      .harness = &port_harness,
      .ctx = c1,
      .stage_counter = &stages_done,
      .stages = 3,
      .thread_id = 1,
      .trace = &trace,
      .trace_len = &trace_len,
   };

   cortos_port_context_init(c0, s0.data(), s0.size(), &thread_yield_n, &a0);
   cortos_port_context_init(c1, s1.data(), s1.size(), &thread_yield_n, &a1);

   // Enqueue in a known order
   port_harness.enqueue(c0);
   port_harness.enqueue(c1);

   // Drive until both threads finish all stages.
   // Each stage yields and reenqueues itself, so the harness should alternate FIFO.
   port_harness.run_until_quiescent();

   EXPECT_EQ(stages_done.load(std::memory_order_acquire), 6);
   EXPECT_EQ(trace_len.load(std::memory_order_acquire), 6);

   // Expected: 0,1,0,1,0,1
   EXPECT_EQ(trace[0], 0);
   EXPECT_EQ(trace[1], 1);
   EXPECT_EQ(trace[2], 0);
   EXPECT_EQ(trace[3], 1);
   EXPECT_EQ(trace[4], 0);
   EXPECT_EQ(trace[5], 1);

   cortos_port_context_destroy(c0);
   cortos_port_context_destroy(c1);
}

TEST_F(PortTest, GivenCpuRelax_WhenCalled_ThenDoesNotCrash)
{
   // Not asserting timing/behaviour; just that it's callable.
   cortos_port_cpu_relax();
   cortos_port_cpu_relax();
}

TEST_F(PortTest, GivenGetStackPointer_WhenCalled_ThenReturnsNonNull)
{
   void* sp = cortos_port_get_stack_pointer();
   EXPECT_NE(sp, nullptr);
}

TEST_F(PortTest, Asserts)
{
   CORTOS_ASSERT(true);
   CORTOS_ASSERT1(true == true, 30);
   CORTOS_ASSERT2(!false, 30, 20);
   CORTOS_ASSERT_OP(1, <, 2);
   int* i_am_null = nullptr;
   CORTOS_ASSERT_NULL(i_am_null);
}

// Uncomment each line to test kernel panics
TEST_F(PortTest, MakesError)
{
   // CORTOS_ASSERT(false); // ...Some handy error diagnosis comment...
   // CORTOS_ASSERT1(true == false, 30);
   // CORTOS_ASSERT2(!true, 30, 20); // ...Some handy error diagnosis comment...
   // CORTOS_ASSERT_OP(1, >, 2);  // ...Some handy error diagnosis comment...
   // int x; CORTOS_ASSERT_NULL(&x);  // ...Some handy error diagnosis comment...
}

