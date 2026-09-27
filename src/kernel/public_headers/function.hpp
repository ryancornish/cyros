#ifndef CYROS_FUNCTION_HPP
#define CYROS_FUNCTION_HPP

#include <array>
#include <cstddef>
#include <functional>
#include <new>
#include <type_traits>
#include <utility>

namespace cyros
{

/* ============================================================================
 * function - Type-Erased Callable with Configurable Storage
 * ========================================================================= */

/**
 * @brief Heap allocation policy for function
 */
enum class heap_policy
{
   no_heap,       // Compile error if callable doesn't fit inline storage
   can_use_heap,  // Use inline storage if possible, heap otherwise
   must_use_heap, // Always allocate on heap
};

/**
 * @brief Type-erased callable with deterministic storage semantics
 *
 * Similar to std::function but with explicit control over memory allocation:
 * - Configurable inline storage size
 * - Compile-time heap policy enforcement
 * - Move-only semantics (no accidental copies)
 * So unlike std::function, you have full control over when/if heap allocation occurs.
 *
 * @tparam Signature function signature (e.g., void(), int(float))
 * @tparam InlineSize Size of inline storage buffer in bytes
 * @tparam Policy Heap allocation policy
 *
 * Example: @code
 *   function<void(), 32, heap_policy::no_heap> callback;
 *   callback = []() { do_work(); };  // Compiles if lambda fits in 32 bytes
 *
 *   int x = 42;
 *   callback = [x]() { use(x); };  // May not fit - compile error with no_heap
 * @endcode
 */
template<typename Signature, std::size_t InlineSize = 32, heap_policy Policy = heap_policy::no_heap>
class function;

/**
 * @brief function specialization for function signatures
 */
template<typename Ret, typename... Args, std::size_t InlineSize, heap_policy Policy>
class function<Ret(Args...), InlineSize, Policy>
{
   static constexpr bool allow_heap = (Policy != heap_policy::no_heap);
   static constexpr bool force_heap = (Policy == heap_policy::must_use_heap);

   // Small, trivially copyable argument can be stored by value, in a register.
   // Otherwise arguments pass by reference, like std::function
   template<typename T>
   using param_t = std::conditional_t<std::is_trivially_copyable_v<T> && sizeof(T) <= 2 * sizeof(void*), T, T&&>;

   union storage
   {
      struct empty {} none{}; // Uninitialised optimisation. Less useful in C++26
      alignas(std::max_align_t) std::array<std::byte, InlineSize> inline_storage;
      void* heap_ptr;
   };

   enum class operation
   {
      move,
      destroy,
   };

   using invoke_function = Ret(*)(storage&, param_t<Args>...);
   using manage_function = void(*)(operation, function& self, function& other) noexcept;

   // The buffer comes first so its address is the object's.
   mutable storage buffer;
   invoke_function invoker{nullptr};
   manage_function manager{nullptr}; // nullptr: empty, or trivially copyable and inline

public:
   constexpr function() noexcept = default;
   constexpr function(std::nullptr_t) noexcept {}

   /**
    * @brief Construct from callable
    * @tparam F Callable type (lambda, function pointer, functor)
    */
   template<typename F> requires (!std::is_same_v<std::remove_cvref_t<F>, function>)
   function(F&& f)
   {
      emplace(std::forward<F>(f));
   }

   ~function()
   {
      destroy();
   }

   function(function&& other) noexcept
   {
      take(other);
   }

   function& operator=(function&& other) noexcept
   {
      if (this != &other) {
         destroy();
         take(other);
      }
      return *this;
   }

   function& operator=(std::nullptr_t) noexcept
   {
      reset();
      return *this;
   }

   /**
    * @brief Replace the current callable, constructing the new one in place
    */
   template<typename F> requires (!std::is_same_v<std::remove_cvref_t<F>, function>)
   function& operator=(F&& f)
   {
      emplace(std::forward<F>(f));
      return *this;
   }

   function(function const&) = delete("function is move-only: a copy would duplicate the captured state");
   function& operator=(function const&) = delete("function is move-only: a copy would duplicate the captured state");

   /**
    * @brief Replace current callable with a new one
    * @tparam F Callable type
    */
   template<typename F>
   void emplace(F&& f)
   {
      using decayed = std::decay_t<F>;

      // Verify callable signature matches
      static_assert(std::is_invocable_r_v<Ret, decayed&, Args...>,
                    "Callable signature does not match function signature");

      // An over-aligned callable cannot go inline: the buffer is only aligned to max_align_t
      constexpr bool fits_inline = sizeof(decayed) <= InlineSize && alignof(decayed) <= alignof(storage);
      constexpr bool use_heap    = force_heap || !fits_inline;

      // Enforce heap policy
      static_assert(!use_heap || allow_heap,
                    "Callable too large (or too aligned) for inline storage. "
                    "Increase InlineSize or allow heap allocation.");

      destroy(); // Destroy old callable if any

      if constexpr (use_heap) {
         buffer.heap_ptr = new decayed(std::forward<F>(f));
         invoker = &invoke_heap<decayed>;
         manager = &manage_heap<decayed>;
      } else {
         ::new (static_cast<void*>(buffer.inline_storage.data())) decayed(std::forward<F>(f));
         invoker = &invoke_inline<decayed>;
         manager = std::is_trivially_copyable_v<decayed> ? nullptr : &manage_inline<decayed>;
      }
   }

   /**
    * @brief Invoke the stored callable
    *
    * Note: operator() is const because it doesn't change which callable is stored,
    * but the callable itself may mutate its internal state (e.g., captured variables).
    * Invoking an empty function is a call through a null pointer.
    */
   Ret operator()(Args... args) const
   {
      return invoker(buffer, std::forward<Args>(args)...);
   }

   /**
    * @brief Check if function contains a callable
    * @return true if callable is stored, false if empty
    */
   explicit operator bool() const noexcept
   {
      return invoker != nullptr;
   }

   /**
    * @brief Clear the stored callable
    */
   void reset() noexcept
   {
      destroy();
      invoker = nullptr;
      manager = nullptr;
   }

private:
   template<typename F>
   static F& object(storage& s) noexcept
   {
      return *std::launder(reinterpret_cast<F*>(s.inline_storage.data()));
   }

   template<typename F>
   static Ret invoke_inline(storage& s, param_t<Args>... args)
   {
      return std::invoke_r<Ret>(object<F>(s), std::forward<param_t<Args>>(args)...);
   }

   template<typename F>
   static Ret invoke_heap(storage& s, param_t<Args>... args)
   {
      return std::invoke_r<Ret>(*static_cast<F*>(s.heap_ptr), std::forward<param_t<Args>>(args)...);
   }

   // A manager's move does the whole transfer, pointers included, so that
   // take() can end in a tail call rather than keep state alive across it.
   template<typename F>
   static void manage_inline(operation op, function& self, function& other) noexcept
   {
      switch (op) {
         case operation::move:
            ::new (static_cast<void*>(self.buffer.inline_storage.data())) F(std::move(object<F>(other.buffer)));
            object<F>(other.buffer).~F();
            self.hand_over_pointers(other);
            break;
         case operation::destroy:
            object<F>(self.buffer).~F();
            break;
      }
   }

   template<typename F>
   static void manage_heap(operation op, function& self, function& other) noexcept
   {
      switch (op) {
         case operation::move:
            self.buffer.heap_ptr = other.buffer.heap_ptr;
            self.hand_over_pointers(other);
            break;
         case operation::destroy:
            delete static_cast<F*>(self.buffer.heap_ptr);
            break;
      }
   }

   void hand_over_pointers(function& other) noexcept
   {
      invoker = std::exchange(other.invoker, nullptr);
      manager = std::exchange(other.manager, nullptr);
   }

   void destroy() noexcept
   {
      if (manager) {
         manager(operation::destroy, *this, *this);
      }
   }

   // Leaves other empty. Its callable, if it had one, is now this one's.
   void take(function& other) noexcept
   {
      if (other.manager) {
         other.manager(operation::move, *this, other);
      } else {
         buffer = other.buffer; // Trivially copyable, or nothing there
         hand_over_pointers(other);
      }
   }
};

} // namespace cyros

#endif // CYROS_FUNCTION_HPP
