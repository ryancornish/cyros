#ifndef CYROS_FUNCTION_HPP
#define CYROS_FUNCTION_HPP

#include <array>
#include <cstddef>
#include <new>

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
   no_heap,      // Compile error if callable doesn't fit inline storage
   can_use_heap, // Use inline storage if possible, heap otherwise
   must_use_heap // Always allocate on heap
};

/**
 * @brief Type-erased callable with deterministic storage semantics
 *
 * Similar to std::function but with explicit control over memory allocation:
 * - Configurable inline storage size
 * - Compile-time heap policy enforcement
 * - Move-only semantics (no accidental copies)
 *
 * Unlike std::function, you have full control over when/if heap allocation occurs.
 *
 * @tparam Signature function signature (e.g., void(), int(float))
 * @tparam InlineSize Size of inline storage buffer in bytes
 * @tparam Policy Heap allocation policy
 *
 * Example:
 *   function<void(), 32, heap_policy::no_heap> callback;
 *   callback = []() { do_work(); };  // Compiles if lambda fits in 32 bytes
 *
 *   int x = 42;
 *   callback = [x]() { use(x); };  // May not fit - compile error with no_heap
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

   using invoke_function  = Ret(*)(void*, Args&&...);
   using move_function    = void(*)(void*, void*);
   using destroy_function = void(*)(void*);

   struct virtual_table
   {
      invoke_function  invoke;
      move_function    move;
      destroy_function destroy;
   };
   virtual_table const* vtable{nullptr};

   // Storage for either inline object or heap pointer
   union storage
   {
      alignas(std::max_align_t) std::array<std::byte, InlineSize> inline_storage;
      void* heap_ptr;
   } storage{};

public:
   constexpr function() = default;
   constexpr function(std::nullptr_t) noexcept {}

   /**
    * @brief Construct from callable
    * @tparam F Callable type (lambda, function pointer, functor)
    */
   template<typename F>
   function(F&& f)
   {
      emplace(std::forward<F>(f));
   }

   ~function()
   {
      reset();
   }

   function(function&& other) noexcept
   {
      move_from(std::move(other));
   }

   function& operator=(function&& other) noexcept
   {
      if (this != &other) {
         reset();
         move_from(std::move(other));
      }
      return *this;
   }

   function(function const&) = delete;
   function& operator=(function const&) = delete;

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

      constexpr std::size_t func_size     = sizeof(decayed);
      constexpr bool fits_inline          = (func_size <= InlineSize);
      constexpr bool needs_heap_for_size  = !fits_inline;
      constexpr bool use_heap             = force_heap || needs_heap_for_size;

      // Enforce heap policy
      static_assert(!needs_heap_for_size || allow_heap,
                    "Callable too large for inline storage. "
                    "Increase InlineSize or allow heap allocation.");

      reset(); // Destroy old callable if any

      if constexpr (use_heap) {
         static_assert(allow_heap, "Heap usage is disabled for this function");
         auto* ptr = new decayed(std::forward<F>(f));
         storage.heap_ptr = ptr;
         vtable = &virtual_table_impl<decayed, true>::table;
      } else {
         void* buf = &storage.inline_storage;
         new (buf) decayed(std::forward<F>(f));
         vtable = &virtual_table_impl<decayed, false>::table;
      }
   }

   /**
    * @brief Invoke the stored callable
    *
    * Note: operator() is const because it doesn't change which callable is stored,
    * but the callable itself may mutate its internal state (e.g., captured variables).
    */
   Ret operator()(Args... args) const
   {
      // const_cast is safe: we're delegating to the stored callable
      return vtable->invoke(const_cast<function*>(this), std::forward<Args>(args)...);
   }

   /**
    * @brief Check if function contains a callable
    * @return true if callable is stored, false if empty
    */
   explicit operator bool() const noexcept
   {
      return vtable != nullptr;
   }

   /**
    * @brief Clear the stored callable
    */
   void reset() noexcept
   {
      if (vtable) {
         vtable->destroy(this);
         vtable = nullptr;
      }
   }

private:
   template<typename F, bool Heap>
   struct virtual_table_impl
   {
      static Ret invoke(void* self_void, Args&&... args)
      {
         auto* self = static_cast<function*>(self_void);
         F* obj = get(self);
         return (*obj)(std::forward<Args>(args)...);
      }

      static void move(void* dst_void, void* src_void)
      {
         auto* dst = static_cast<function*>(dst_void);
         auto* src = static_cast<function*>(src_void);

         if constexpr (Heap) {
            dst->storage.heap_ptr = src->storage.heap_ptr;
            src->storage.heap_ptr = nullptr;
         } else {
            F* src_obj = get(src);
            void* dst_buf = &dst->storage.inline_storage;
            new (dst_buf) F(std::move(*src_obj));
            src_obj->~F();
         }

         dst->vtable = src->vtable;
         src->vtable = nullptr;
      }

      static void destroy(void* self_void)
      {
         auto* self = static_cast<function*>(self_void);

         if constexpr (Heap) {
            if (self->storage.heap_ptr) {
               delete static_cast<F*>(self->storage.heap_ptr);
               self->storage.heap_ptr = nullptr;
            }
         } else {
            F* obj = get(self);
            obj->~F();
         }
      }

      static F* get(function* self)
      {
         if constexpr (Heap) {
            return static_cast<F*>(self->storage.heap_ptr);
         } else {
            return std::launder(reinterpret_cast<F*>(&self->storage.inline_storage));
         }
      }

      static const virtual_table table;
   };

   template<typename F, bool Heap>
   friend struct function::virtual_table_impl;

   void move_from(function&& other) noexcept
   {
      if (!other.vtable) {
         vtable = nullptr;
         return;
      }
      other.vtable->move(this, &other);
   }
};

// Out-of-line virtual_table definition
template<typename Ret, typename... Args, std::size_t InlineSize, heap_policy Policy>
template<typename F, bool Heap>
const typename function<Ret(Args...), InlineSize, Policy>::virtual_table
function<Ret(Args...), InlineSize, Policy>::virtual_table_impl<F, Heap>::table{
   .invoke  = &virtual_table_impl::invoke,
   .move    = &virtual_table_impl::move,
   .destroy = &virtual_table_impl::destroy
};

} // namespace cyros

#endif // CYROS_FUNCTION_HPP
