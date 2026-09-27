#include <cyros/kernel/function.hpp>
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>
#include <type_traits>

using namespace cyros;


/**
 * @brief Convenience aliases for common function configurations
 */
namespace function_aliases
{
   /// General-purpose callback (no heap, 32-byte inline storage)
   using callback = function<void(), 32, heap_policy::no_heap>;

   /// Thread entry point (no heap, 32-byte inline storage)
   using ThreadEntry = function<void(), 32, heap_policy::no_heap>;

   /// Deferred work handler (no heap, 64-byte inline storage for more captures)
   using DeferredWork = function<void(), 64, heap_policy::no_heap>;

   /// Flexible callback (can use heap if needed)
   using FlexibleCallback = function<void(), 32, heap_policy::can_use_heap>;
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
 * Basic function Tests (void())
 * ========================================================================= */

TEST_F(FunctionTest, DefaultConstructor)
{
   function<void()> f;

   EXPECT_FALSE(f);
}

TEST_F(FunctionTest, FunctionPointer)
{
   int call_count = 0;
   auto callback = [&call_count]() { call_count++; };

   function<void()> f(callback);

   EXPECT_TRUE(f);
   f();
   EXPECT_EQ(call_count, 1);

   f();
   EXPECT_EQ(call_count, 2);
}

TEST_F(FunctionTest, LambdaNoCapture)
{
   int value = 0;

   function<void()> f([&value]() { value = 42; });

   EXPECT_TRUE(f);
   f();
   EXPECT_EQ(value, 42);
}

TEST_F(FunctionTest, LambdaWithCapture)
{
   int x = 10;
   int result = 0;

   function<void(), 32> f([x, &result]() { result = x * 2; });

   EXPECT_TRUE(f);
   f();
   EXPECT_EQ(result, 20);
}

TEST_F(FunctionTest, MoveConstruction)
{
   int call_count = 0;

   function<void()> f1([&call_count]() { call_count++; });
   function<void()> f2(std::move(f1));

   EXPECT_FALSE(f1);  // f1 is now empty
   EXPECT_TRUE(f2);   // f2 has the callable

   f2();
   EXPECT_EQ(call_count, 1);
}

TEST_F(FunctionTest, MoveAssignment)
{
   int call_count = 0;

   function<void()> f1([&call_count]() { call_count++; });
   function<void()> f2;

   f2 = std::move(f1);

   EXPECT_FALSE(f1);
   EXPECT_TRUE(f2);

   f2();
   EXPECT_EQ(call_count, 1);
}

TEST_F(FunctionTest, Reset)
{
   int call_count = 0;

   function<void()> f([&call_count]() { call_count++; });

   EXPECT_TRUE(f);
   f.reset();
   EXPECT_FALSE(f);
}

TEST_F(FunctionTest, Emplace)
{
   int value1 = 0;
   int value2 = 0;

   function<void()> f([&value1]() { value1 = 1; });

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
 * function with Return Values
 * ========================================================================= */

TEST_F(FunctionTest, ReturnInt)
{
   function<int()> f([]() { return 42; });

   EXPECT_TRUE(f);
   EXPECT_EQ(f(), 42);
}

TEST_F(FunctionTest, ReturnDouble)
{
   function<double()> f([]() { return 3.14; });

   EXPECT_TRUE(f);
   EXPECT_DOUBLE_EQ(f(), 3.14);
}

/* ============================================================================
 * function with Arguments
 * ========================================================================= */

TEST_F(FunctionTest, SingleArgument)
{
   function<int(int)> f([](int x) { return x * 2; });

   EXPECT_TRUE(f);
   EXPECT_EQ(f(5), 10);
   EXPECT_EQ(f(100), 200);
}

TEST_F(FunctionTest, MultipleArguments)
{
   function<int(int, int)> f([](int a, int b) { return a + b; });

   EXPECT_TRUE(f);
   EXPECT_EQ(f(3, 4), 7);
   EXPECT_EQ(f(10, 20), 30);
}

TEST_F(FunctionTest, MixedArguments)
{
   function<double(int, double, float)> f([](int a, double b, float c) {
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
   function<void(), 32, heap_policy::no_heap> f([]() {});

   EXPECT_TRUE(f);
   f();
}

TEST_F(FunctionTest, CanUseHeapPolicy)
{
   // Should compile - can use heap if needed
   int a = 1, b = 2, c = 3, d = 4, e = 5;

   function<void(), 16, heap_policy::can_use_heap> f([a, b, c, d, e]() {
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
//    function<void(), 8, heap_policy::no_heap> func([a, b, c, d, e, f, g, h]() {});
// }

/* ============================================================================
 * Inline Storage Size Tests
 * ========================================================================= */

TEST_F(FunctionTest, InlineStorageSize8)
{
   // Very small inline storage
   function<void(), 8, heap_policy::no_heap> f([]() {});

   EXPECT_TRUE(f);
   f();
}

TEST_F(FunctionTest, InlineStorageSize64)
{
   // Larger inline storage
   int a = 1, b = 2, c = 3;

   function<void(), 64, heap_policy::no_heap> f([a, b, c]() {
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

   function<void()> f(functor);

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

   function<int()> f(functor);

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
   callback cb([&count]() { count++; });

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
   function<void()> f(nullptr);

   EXPECT_FALSE(f);
}

TEST_F(FunctionTest, MultipleResets)
{
   function<void()> f([]() {});

   EXPECT_TRUE(f);
   f.reset();
   EXPECT_FALSE(f);
   f.reset();  // Should be safe to reset again
   EXPECT_FALSE(f);
}

TEST_F(FunctionTest, SelfMoveAssignment)
{
   int count = 0;
   function<void()> f([&count]() { count++; });

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
      function<void()> f(CountingCallable{});
      EXPECT_TRUE(f);
   }  // f destroyed here

   EXPECT_EQ(destructor_count, 2);  // Once for temporary, once for stored copy
}

/* ============================================================================
 * Storage and Lifetime
 *
 * A trivially copyable callable is moved by copying bytes and has nothing to
 * destroy. Anything else goes through a manager. These pin that split: a
 * callable with a real move constructor or destructor must see every one of
 * them, exactly once each.
 * ========================================================================= */

namespace
{

/// Not trivially copyable, so it takes the manager path.
struct LifetimeCounter
{
   static inline int moves        = 0;
   static inline int destructions = 0;

   int* calls;

   explicit LifetimeCounter(int* calls) : calls(calls) {}
   LifetimeCounter(LifetimeCounter&& other) noexcept : calls(other.calls) { ++moves; }
   ~LifetimeCounter() { ++destructions; }

   void operator()() const { ++*calls; }

   static void zero() { moves = 0; destructions = 0; }
};

static_assert(!std::is_trivially_copyable_v<LifetimeCounter>);

}  // namespace

// The two pointers take the padding a single pointer would leave before the
// max_align_t buffer, so this holds on both a 32-bit and a 64-bit target.
static_assert(sizeof(function<void(), 32>) == 32 + 2 * sizeof(void*));
static_assert(sizeof(function<void(), 48>) == 48 + 2 * sizeof(void*));

// A default-constructed function must stay constant-initialisable.
constinit function<void()> constant_initialised;

TEST_F(FunctionTest, DefaultConstructedIsConstantInitialised)
{
   EXPECT_FALSE(constant_initialised);
}

TEST_F(FunctionTest, NonTrivialCallable_MoveRunsItsMoveAndDestructor)
{
   LifetimeCounter::zero();
   int calls = 0;
   {
      function<void()> f1(LifetimeCounter{&calls});
      EXPECT_EQ(LifetimeCounter::moves, 1);         // temporary into the buffer
      EXPECT_EQ(LifetimeCounter::destructions, 1);  // the temporary

      function<void()> f2(std::move(f1));
      EXPECT_EQ(LifetimeCounter::moves, 2);
      EXPECT_EQ(LifetimeCounter::destructions, 2);  // f1's, at the move
      EXPECT_FALSE(f1);

      f2();
      EXPECT_EQ(calls, 1);
   }
   // f2's copy, once. f1 had nothing left to destroy.
   EXPECT_EQ(LifetimeCounter::destructions, 3);
}

TEST_F(FunctionTest, NonTrivialCallable_MoveAssignmentDestroysTheReplacedOne)
{
   int first = 0;
   int second = 0;
   function<void()> a(LifetimeCounter{&first});
   function<void()> b(LifetimeCounter{&second});

   LifetimeCounter::zero();
   a = std::move(b);
   EXPECT_EQ(LifetimeCounter::moves, 1);
   EXPECT_EQ(LifetimeCounter::destructions, 2);  // a's old callable, and b's after the move
   EXPECT_FALSE(b);

   a();
   EXPECT_EQ(first, 0);
   EXPECT_EQ(second, 1);
}

TEST_F(FunctionTest, AssigningACallableConstructsItInPlace)
{
   int first = 0;
   int second = 0;
   function<void()> f(LifetimeCounter{&first});

   LifetimeCounter::zero();
   f = LifetimeCounter{&second};
   // One move, straight into the buffer. Going through a temporary function
   // would move twice.
   EXPECT_EQ(LifetimeCounter::moves, 1);
   EXPECT_EQ(LifetimeCounter::destructions, 2);  // the replaced callable, and the temporary

   f();
   EXPECT_EQ(first, 0);
   EXPECT_EQ(second, 1);
}

TEST_F(FunctionTest, AssigningNullptrDestroysTheCallable)
{
   int calls = 0;
   function<void()> f(LifetimeCounter{&calls});

   LifetimeCounter::zero();
   f = nullptr;
   EXPECT_FALSE(f);
   EXPECT_EQ(LifetimeCounter::destructions, 1);
}

TEST_F(FunctionTest, TriviallyCopyableCallableFillingTheBufferSurvivesMoves)
{
   std::array<std::uint64_t, 4> const values{1, 20, 300, 4000};
   auto sum = [values]() { return values[0] + values[1] + values[2] + values[3]; };
   static_assert(std::is_trivially_copyable_v<decltype(sum)>);
   static_assert(sizeof(sum) == 32);

   function<std::uint64_t(), 32> f1(sum);
   function<std::uint64_t(), 32> f2(std::move(f1));
   function<std::uint64_t(), 32> f3;
   f3 = std::move(f2);

   EXPECT_FALSE(f1);
   EXPECT_FALSE(f2);
   ASSERT_TRUE(f3);
   EXPECT_EQ(f3(), 4321u);
}

/* ============================================================================
 * Heap Policies, Observed
 *
 * A callable with its own operator new shows whether it was placed on the heap
 * without replacing the global one.
 * ========================================================================= */

namespace
{

struct HeapCounted
{
   static inline int allocations   = 0;
   static inline int deallocations = 0;

   static void* operator new(std::size_t size) { ++allocations; return ::operator new(size); }
   static void operator delete(void* p) noexcept { ++deallocations; ::operator delete(p); }

   static void zero() { allocations = 0; deallocations = 0; }
};

struct SmallHeapCallable : HeapCounted
{
   int* calls;
   void operator()() const { ++*calls; }
};

struct LargeHeapCallable : HeapCounted
{
   int* calls;
   std::array<std::byte, 64> payload{};
   void operator()() const { ++*calls; }
};

/// Small, but aligned more strictly than the inline buffer can promise.
struct alignas(2 * alignof(std::max_align_t)) OverAlignedCallable
{
   static inline int allocations = 0;

   static void* operator new(std::size_t size, std::align_val_t al) { ++allocations; return ::operator new(size, al); }
   static void operator delete(void* p, std::align_val_t al) noexcept { ::operator delete(p, al); }

   bool* aligned;
   void operator()() const
   {
      *aligned = reinterpret_cast<std::uintptr_t>(this) % alignof(OverAlignedCallable) == 0;
   }
};

}  // namespace

TEST_F(FunctionTest, MustUseHeap_AllocatesOnceAndMovesWithoutAllocating)
{
   HeapCounted::zero();
   int calls = 0;
   {
      function<void(), 32, heap_policy::must_use_heap> f1(SmallHeapCallable{{}, &calls});
      EXPECT_EQ(HeapCounted::allocations, 1);

      function<void(), 32, heap_policy::must_use_heap> f2(std::move(f1));
      EXPECT_EQ(HeapCounted::allocations, 1);
      EXPECT_EQ(HeapCounted::deallocations, 0);

      f2();
      EXPECT_EQ(calls, 1);
   }
   EXPECT_EQ(HeapCounted::deallocations, 1);
}

TEST_F(FunctionTest, CanUseHeap_SmallCallableStaysInline)
{
   HeapCounted::zero();
   int calls = 0;
   {
      function<void(), 32, heap_policy::can_use_heap> f(SmallHeapCallable{{}, &calls});
      f();
   }
   EXPECT_EQ(calls, 1);
   EXPECT_EQ(HeapCounted::allocations, 0);
}

TEST_F(FunctionTest, CanUseHeap_LargeCallableGoesToHeap)
{
   HeapCounted::zero();
   int calls = 0;
   {
      function<void(), 32, heap_policy::can_use_heap> f(LargeHeapCallable{{}, &calls, {}});
      EXPECT_EQ(HeapCounted::allocations, 1);
      f();
   }
   EXPECT_EQ(calls, 1);
   EXPECT_EQ(HeapCounted::deallocations, 1);
}

TEST_F(FunctionTest, CanUseHeap_OverAlignedCallableGoesToHeapAndStaysAligned)
{
   static_assert(sizeof(OverAlignedCallable) <= 32);

   OverAlignedCallable::allocations = 0;
   bool aligned = false;

   function<void(), 32, heap_policy::can_use_heap> f(OverAlignedCallable{&aligned});
   EXPECT_EQ(OverAlignedCallable::allocations, 1);

   f();
   EXPECT_TRUE(aligned);
}

/* ============================================================================
 * Argument Passing and Return
 * ========================================================================= */

TEST_F(FunctionTest, ReferenceArgumentReachesTheCaller)
{
   function<void(int&)> f([](int& x) { x = 7; });

   int value = 0;
   f(value);
   EXPECT_EQ(value, 7);
}

TEST_F(FunctionTest, MoveOnlyArgument)
{
   function<int(std::unique_ptr<int>)> f([](std::unique_ptr<int> p) { return *p; });

   EXPECT_EQ(f(std::make_unique<int>(5)), 5);
}

TEST_F(FunctionTest, LargeTriviallyCopyableArgument)
{
   struct large { std::array<std::uint32_t, 16> v; };
   static_assert(std::is_trivially_copyable_v<large> && sizeof(large) > 2 * sizeof(void*));

   function<std::uint32_t(large)> f([](large l) { return l.v[0] + l.v[15]; });

   large l{};
   l.v[0]  = 3;
   l.v[15] = 4;
   EXPECT_EQ(f(l), 7u);
}

TEST_F(FunctionTest, VoidSignatureDiscardsTheResult)
{
   int calls = 0;
   function<void()> f([&calls]() { return ++calls; });

   f();
   EXPECT_EQ(calls, 1);
}
