#ifndef CORTOS_THREAD_HPP
#define CORTOS_THREAD_HPP

#include <cortos/kernel/function.hpp>

#include <cstdint>
#include <span>

namespace cortos
{

/**
 * @brief Core affinity mask
 *
 * Bit flags indicating which cores a thread can run on.
 * Use bitwise OR to combine cores: core0 | core1
 */
struct core_affinity
{
   std::uint32_t mask;
   constexpr explicit core_affinity(std::uint32_t m) : mask(m) {}
   constexpr explicit operator std::uint32_t() const { return mask; }
   constexpr core_affinity operator|(core_affinity rhs) const { return core_affinity{mask | rhs.mask}; }
   constexpr core_affinity operator&(core_affinity rhs) const { return core_affinity{mask & rhs.mask}; }
   [[nodiscard]] constexpr bool allows(uint32_t core_id) const noexcept { return (mask & (1u << core_id)) != 0; }
   [[nodiscard]] constexpr static core_affinity from_id(std::uint32_t core_id) { return core_affinity{1u << core_id}; }
};
// Predefined core masks
inline constexpr core_affinity core0 = core_affinity{0x01};
inline constexpr core_affinity core1 = core_affinity{0x02};
inline constexpr core_affinity core2 = core_affinity{0x04};
inline constexpr core_affinity core3 = core_affinity{0x08};
inline constexpr core_affinity any_core = core_affinity{0xFFFFFFFF};


/**
 * @brief Joinable CoRTOS thread handle.
 *
 * Owns a running kernel thread. The thread's TCB is constructed inside the user-provided
 * stack buffer, so both the @c thread object and the stack buffer must outlive the thread.
 *
 * The destructor asserts the thread is terminated (i.e. no implicit detach).
 */
class thread
{
public:
   using id = std::uint32_t;
   using entry_fn = function<void(), 48, heap_policy::no_heap>;

   struct priority
   {
      std::uint8_t val;
      constexpr priority(std::uint8_t v) : val(v) {}     // Intentionally implicit
      constexpr operator uint8_t() const { return val; } // Intentionally implicit
   };


   /**
    * @brief Create empty thread handle.
    * A registered thread can be moved into this.
    */
   constexpr thread() = default;
   /**
    * @brief Create and register a new thread.
    * @param entry thread entry function.
    * @param stack User-owned stack buffer (must remain valid until termination).
    * @param priority Initial priority.
    * @param affinity Core affinity (defaults to any_core).
    */
   thread(entry_fn&& entry, std::span<std::byte> stack, priority priority, core_affinity affinity = any_core);
   ~thread();
   thread(thread&&) noexcept;
   thread& operator=(thread&&) noexcept;
   thread(thread const&)            = delete;
   thread& operator=(thread const&) = delete;

   /**
    * @brief Get thread ID
    * @return Unique thread identifier
    */
   [[nodiscard]] id get_id() const noexcept;

   /**
    * @brief Get thread priority
    * @return Current effective priority (base + inherited)
    */
   [[nodiscard]] priority get_priority() const noexcept;

   /**
    * @brief Wait for thread to exit
    *
    * Blocks until this thread terminates.
    * Can only be called once per thread.
    */
   void join() noexcept;


   static std::size_t reserved_stack_size();

private:
   struct thread_control_block* tcb{nullptr};
};

namespace this_thread
{

/**
   * @brief Get current thread ID
   */
[[nodiscard]] thread::id id();

/**
   * @brief Get current thread (effective) priority
   */
[[nodiscard]] thread::priority priority();

/**
   * @brief Get current CPU core ID (0-based)
   */
[[nodiscard]] std::uint32_t core_id() noexcept;

/**
   * @brief Exit current thread
   *
   * Marks current thread as terminated. thread never runs again.
   * scheduler switches to next ready thread.
   *
   * Note: If thread entry function returns, this is called automatically.
   */
[[noreturn]] void thread_exit();

void yield();

}  // namespace this_thread

} // namespace cortos

#endif // CORTOS_THREAD_HPP
