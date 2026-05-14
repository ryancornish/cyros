#include <cortos/kernel/kernel.hpp>
#include <gtest/gtest.h>

using namespace cortos;


/**
 * @brief Convenience aliases for common Function configurations
 */
namespace function_aliases
{
   /// General-purpose callback (no heap, 32-byte inline storage)
   using Callback = Function<void(), 32, HeapPolicy::NoHeap>;

   /// Thread entry point (no heap, 32-byte inline storage)
   using ThreadEntry = Function<void(), 32, HeapPolicy::NoHeap>;

   /// Deferred work handler (no heap, 64-byte inline storage for more captures)
   using DeferredWork = Function<void(), 64, HeapPolicy::NoHeap>;

   /// Flexible callback (can use heap if needed)
   using FlexibleCallback = Function<void(), 32, HeapPolicy::CanUseHeap>;
}

/* ============================================================================
 * Test Fixtures
 * ========================================================================= */

class FunctionTest : public ::testing::Test
{
protected:
   void SetUp() override {}
};

/* ============================================================================
 * Basic Function Tests (void())
 * ========================================================================= */

TEST_F(FunctionTest, DefaultConstructor)
{
   Function<void()> f;

   EXPECT_FALSE(f);
}

TEST_F(FunctionTest, FunctionPointer)
{
   int call_count = 0;
   auto callback = [&call_count]() { call_count++; };

   Function<void()> f(callback);

   EXPECT_TRUE(f);
   f();
   EXPECT_EQ(call_count, 1);

   f();
   EXPECT_EQ(call_count, 2);
}

TEST_F(FunctionTest, LambdaNoCapture)
{
   int value = 0;

   Function<void()> f([&value]() { value = 42; });

   EXPECT_TRUE(f);
   f();
   EXPECT_EQ(value, 42);
}

TEST_F(FunctionTest, LambdaWithCapture)
{
   int x = 10;
   int result = 0;

   Function<void(), 32> f([x, &result]() { result = x * 2; });

   EXPECT_TRUE(f);
   f();
   EXPECT_EQ(result, 20);
}

TEST_F(FunctionTest, MoveConstruction)
{
   int call_count = 0;

   Function<void()> f1([&call_count]() { call_count++; });
   Function<void()> f2(std::move(f1));

   EXPECT_FALSE(f1);  // f1 is now empty
   EXPECT_TRUE(f2);   // f2 has the callable

   f2();
   EXPECT_EQ(call_count, 1);
}

TEST_F(FunctionTest, MoveAssignment)
{
   int call_count = 0;

   Function<void()> f1([&call_count]() { call_count++; });
   Function<void()> f2;

   f2 = std::move(f1);

   EXPECT_FALSE(f1);
   EXPECT_TRUE(f2);

   f2();
   EXPECT_EQ(call_count, 1);
}

TEST_F(FunctionTest, Reset)
{
   int call_count = 0;

   Function<void()> f([&call_count]() { call_count++; });

   EXPECT_TRUE(f);
   f.reset();
   EXPECT_FALSE(f);
}

TEST_F(FunctionTest, Emplace)
{
   int value1 = 0;
   int value2 = 0;

   Function<void()> f([&value1]() { value1 = 1; });

   f();
   EXPECT_EQ(value1, 1);
   EXPECT_EQ(value2, 0);

   // Replace with new callable
   f.emplace([&value2]() { value2 = 2; });

   f();
   EXPECT_EQ(value1, 1);  // Unchanged
   EXPECT_EQ(value2, 2);  // New callable invoked
}

/* ============================================================================
 * Function with Return Values
 * ========================================================================= */

TEST_F(FunctionTest, ReturnInt)
{
   Function<int()> f([]() { return 42; });

   EXPECT_TRUE(f);
   EXPECT_EQ(f(), 42);
}

TEST_F(FunctionTest, ReturnDouble)
{
   Function<double()> f([]() { return 3.14; });

   EXPECT_TRUE(f);
   EXPECT_DOUBLE_EQ(f(), 3.14);
}

/* ============================================================================
 * Function with Arguments
 * ========================================================================= */

TEST_F(FunctionTest, SingleArgument)
{
   Function<int(int)> f([](int x) { return x * 2; });

   EXPECT_TRUE(f);
   EXPECT_EQ(f(5), 10);
   EXPECT_EQ(f(100), 200);
}

TEST_F(FunctionTest, MultipleArguments)
{
   Function<int(int, int)> f([](int a, int b) { return a + b; });

   EXPECT_TRUE(f);
   EXPECT_EQ(f(3, 4), 7);
   EXPECT_EQ(f(10, 20), 30);
}

TEST_F(FunctionTest, MixedArguments)
{
   Function<double(int, double, float)> f([](int a, double b, float c) {
      return a + b + c;
   });

   EXPECT_TRUE(f);
   EXPECT_DOUBLE_EQ(f(1, 2.5, 3.5f), 7.0);
}

/* ============================================================================
 * Heap Policy Tests
 * ========================================================================= */

TEST_F(FunctionTest, NoHeapPolicy_SmallCallable)
{
   // Should compile - small lambda fits in inline storage
   Function<void(), 32, HeapPolicy::NoHeap> f([]() {});

   EXPECT_TRUE(f);
   f();
}

TEST_F(FunctionTest, CanUseHeapPolicy)
{
   // Should compile - can use heap if needed
   int a = 1, b = 2, c = 3, d = 4, e = 5;

   Function<void(), 16, HeapPolicy::CanUseHeap> f([a, b, c, d, e]() {
      // Large capture, will use heap
      (void)a; (void)b; (void)c; (void)d; (void)e;
   });

   EXPECT_TRUE(f);
   f();
}

// This test would fail to compile (as intended):
// TEST_F(FunctionTest, NoHeapPolicy_LargeCallable)
// {
//    int a = 1, b = 2, c = 3, d = 4, e = 5, f = 6, g = 7, h = 8;
//
//    // Compile error: callable too large for 8-byte inline storage
//    Function<void(), 8, HeapPolicy::NoHeap> func([a, b, c, d, e, f, g, h]() {});
// }

/* ============================================================================
 * Inline Storage Size Tests
 * ========================================================================= */

TEST_F(FunctionTest, InlineStorageSize8)
{
   // Very small inline storage
   Function<void(), 8, HeapPolicy::NoHeap> f([]() {});

   EXPECT_TRUE(f);
   f();
}

TEST_F(FunctionTest, InlineStorageSize64)
{
   // Larger inline storage
   int a = 1, b = 2, c = 3;

   Function<void(), 64, HeapPolicy::NoHeap> f([a, b, c]() {
      (void)a; (void)b; (void)c;
   });

   EXPECT_TRUE(f);
   f();
}

/* ============================================================================
 * Functor Tests
 * ========================================================================= */

struct SimpleFunctor
{
   int& counter;

   void operator()()
   {
      counter++;
   }
};

TEST_F(FunctionTest, Functor)
{
   int count = 0;
   SimpleFunctor functor{count};

   Function<void()> f(functor);

   EXPECT_TRUE(f);
   f();
   EXPECT_EQ(count, 1);

   f();
   EXPECT_EQ(count, 2);
}

struct FunctorWithReturn
{
   int value;

   int operator()() const
   {
      return value * 2;
   }
};

TEST_F(FunctionTest, FunctorWithReturn)
{
   FunctorWithReturn functor{21};

   Function<int()> f(functor);

   EXPECT_TRUE(f);
   EXPECT_EQ(f(), 42);
}

/* ============================================================================
 * Convenience Alias Tests
 * ========================================================================= */

TEST_F(FunctionTest, CallbackAlias)
{
   using namespace function_aliases;

   int count = 0;
   Callback cb([&count]() { count++; });

   EXPECT_TRUE(cb);
   cb();
   EXPECT_EQ(count, 1);
}

TEST_F(FunctionTest, ThreadEntryAlias)
{
   using namespace function_aliases;

   bool executed = false;
   ThreadEntry entry([&executed]() { executed = true; });

   EXPECT_TRUE(entry);
   entry();
   EXPECT_TRUE(executed);
}

/* ============================================================================
 * Edge Cases
 * ========================================================================= */

TEST_F(FunctionTest, NullptrConstruction)
{
   Function<void()> f(nullptr);

   EXPECT_FALSE(f);
}

TEST_F(FunctionTest, MultipleResets)
{
   Function<void()> f([]() {});

   EXPECT_TRUE(f);
   f.reset();
   EXPECT_FALSE(f);
   f.reset();  // Should be safe to reset again
   EXPECT_FALSE(f);
}

TEST_F(FunctionTest, SelfMoveAssignment)
{
   int count = 0;
   Function<void()> f([&count]() { count++; });

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wself-move"
   f = std::move(f);  // Self-assignment
#pragma GCC diagnostic pop

   // Should still be valid
   EXPECT_TRUE(f);
   f();
   EXPECT_EQ(count, 1);
}

TEST_F(FunctionTest, DestructorCleansUp)
{
   static int destructor_count = 0;

   struct CountingCallable
   {
      ~CountingCallable() { destructor_count++; }
      void operator()() const {}
   };

   destructor_count = 0;

   {
      Function<void()> f(CountingCallable{});
      EXPECT_TRUE(f);
   }  // f destroyed here

   EXPECT_EQ(destructor_count, 2);  // Once for temporary, once for stored copy
}
