/**
 * @file mpsc_ring_buffer.hpp
 * @brief Lock-free Multi-Producer Single-Consumer (MPSC) ring buffer
 *
 * A high-performance, wait-free ring buffer that allows multiple producers
 * to concurrently push elements while a single consumer pops them.
 *
 * Key properties:
 * - Lock-free for producers (multiple threads can push concurrently)
 * - Wait-free for consumer (single thread, no blocking)
 * - Cache-line aligned to minimize false sharing
 * - Zero memory allocation after construction
 * - Compile-time constructable
 * - Compile-time fixed capacity (power of 2)
 */

/**
 * @brief Lock-free Multi-Producer Single-Consumer ring buffer
 *
 * @tparam T Element type (must be nothrow destructible)
 * @tparam N Buffer capacity (must be power of 2, > 0)
 *
 * Thread safety:
 * - push(): Safe to call from multiple threads concurrently
 * - pop(): Must ONLY be called from a single thread (the consumer)
 * - approx_size(): Safe but may be stale under contention
 *
 * Memory ordering:
 * - Uses acquire/release semantics for synchronization
 * - Producers synchronize via atomic head counter
 * - Consumer has exclusive access to tail counter
 * - Cell sequences enforce happens-before relationships
 *
 * Performance characteristics:
 * - push(): Wait-free in uncontended case, lock-free under contention
 * - pop(): Wait-free (never blocks or spins)
 * - Both operations are O(1)
 * - Cache-line aligned to avoid false sharing (64-byte alignment)
 *
 * Example usage:
 * @code
 *   mpsc_ring_buffer<event, 256> queue;
 *
 *   // Producer threads (multiple)
 *   void producer_thread() {
 *     event evt = create_event();
 *     if (!queue.push(std::move(evt))) {
 *       // Buffer full, handle overflow
 *     }
 *   }
 *
 *   // Consumer thread (single)
 *   void consumer_thread() {
 *     event evt;
 *     while (queue.pop(evt)) {
 *       process(evt);
 *     }
 *   }
 * @endcode
 *
 * Algorithm:
 * Based on Dmitry Vyukov's MPMC queue adapted for single consumer.
 * Each cell has a sequence number that tracks its state:
 * - sequence == position: cell is ready for producer to write
 * - sequence == position + 1: cell contains data for consumer to read
 * - sequence == position + N: cell is empty, waiting for wraparound
 *
 * Producers use CAS on head to claim slots, then write data and publish
 * by updating the cell sequence. Consumer reads tail exclusively.
 */
#ifndef CYROS_MPSC_RING_BUFFER_HPP
#define CYROS_MPSC_RING_BUFFER_HPP

#include <cyros/port/port.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace cyros
{

template<typename T, std::size_t N>
class mpsc_ring_buffer
{
   static_assert(N > 0, "Buffer capacity must be greater than zero");
   static_assert((N & (N - 1)) == 0, "Buffer capacity must be a power of two (for fast modulo via masking)");
   static_assert(std::is_nothrow_destructible_v<T>, "T must be nothrow destructible (for safe cleanup in destructor)");

public:
   /**
    * @brief Default constructor - initializes buffer with sequence numbers
    *
    * Each cell is initialized with its index as the sequence number,
    * establishing the initial ready-for-producer state.
    */
   constexpr mpsc_ring_buffer() noexcept : mpsc_ring_buffer(std::make_index_sequence<N>{}) {}

   mpsc_ring_buffer(mpsc_ring_buffer&&)            = delete;
   mpsc_ring_buffer& operator=(mpsc_ring_buffer&&) = delete;
   mpsc_ring_buffer(mpsc_ring_buffer const&)            = delete;
   mpsc_ring_buffer& operator=(mpsc_ring_buffer const&) = delete;

   /**
    * @brief Destructor - drains remaining elements
    *
    * Pops and discards all remaining elements to ensure proper destruction.
    * This is safe because destruction should only happen after all producers
    * have stopped and the consumer has finished processing.
    *
    * Warning: Do not destroy the buffer while it's still being accessed.
    */
   ~mpsc_ring_buffer() noexcept
   {
      // Single-consumer destruction: drain remaining items
      T tmp;
      while (pop(tmp)) { /* discard */ }
   }

   /**
    * @brief Get approximate number of elements in buffer
    * @return Approximate element count (may be stale under contention)
    *
    * This is a best-effort estimate that may be slightly inaccurate when
    * producers and consumer are actively running. Safe to call from any thread.
    *
    * Use cases:
    * - Statistics/monitoring
    * - Debug/logging
    * - Approximate fullness checks
    *
    * Do NOT use for:
    * - Precise synchronization
    * - Capacity decisions (use push() return value instead)
    *
    * Thread safety: Safe from any thread, but value may be stale.
    */
   [[nodiscard]] std::size_t approx_size() const noexcept
   {
      auto h = head.load(std::memory_order_acquire);
      auto t = tail.load(std::memory_order_acquire);
      return (h >= t) ? (h - t) : 0;
   }

   /**
    * @brief Push an element onto the buffer (multi-producer safe)
    * @tparam U Forwarding type (deduced)
    * @param value Element to push (forwarded to T's constructor)
    * @return true if successfully pushed, false if buffer is full
    *
    * Thread safety: Safe to call concurrently from multiple threads.
    *
    * Algorithm:
    * 1. Load current head position (relaxed - we'll CAS it anyway)
    * 2. Check if cell at position is available (sequence == position)
    * 3. If available, try to claim it via CAS on head
    * 4. If claimed, construct element in-place and publish (sequence = position + 1)
    * 5. If not available or CAS failed, retry with updated head
    *
    * Returns false only when buffer is genuinely full (consumer hasn't
    * freed enough cells). Retries automatically on CAS failures due to
    * contention with other producers.
    *
    * Exception safety: noexcept if T's constructor is noexcept.
    *
    * Performance: Wait-free in uncontended case, lock-free under contention.
    */
   template<typename U>
   bool push(U&& value) noexcept(std::is_nothrow_constructible_v<T, U&&>)
   {
      auto position = head.load(std::memory_order_relaxed);

      while (true) {
         auto& cell = cells[position & mask];
         auto sequence = cell.sequence.load(std::memory_order_acquire);
         auto diff = static_cast<intptr_t>(sequence) - static_cast<intptr_t>(position);

         if (diff == 0) {
            // Cell is available for this position - try to claim position
            if (head.compare_exchange_weak(position, position + 1,
                                           std::memory_order_relaxed,
                                           std::memory_order_relaxed)) {
               ::new (cell.storage.data()) T(std::forward<U>(value));

               // Publish: mark cell ready for consumer by setting sequence to position+1
               // Release ensures element construction happens-before consumer read
               cell.sequence.store(position + 1, std::memory_order_release);
               return true;
            }
            // CAS failed: another producer claimed it, position updated by CAS, retry
         } else if (diff < 0) {
            // sequence < position => consumer hasn't freed this cell yet - buffer full
            return false;
         } else {
            // diff > 0: another producer is ahead - reload head and retry
            position = head.load(std::memory_order_relaxed);
         }
      }
   }

   /**
    * @brief Pop an element from the buffer (single-consumer ONLY)
    * @param out Output parameter - receives the popped element
    * @return true if element was popped, false if buffer is empty
    *
    * Thread safety: Must ONLY be called from the single consumer thread.
    * Calling from multiple threads is undefined behavior.
    *
    * Algorithm:
    * 1. Load current tail position (relaxed - we own it)
    * 2. Check if cell contains data (sequence == position + 1)
    * 3. If yes, move element to output, destroy original, mark cell free
    * 4. If no, buffer is empty (or producer mid-flight)
    *
    * The cell is marked free by setting sequence to position + N, which
    * makes it available for producers when head wraps around.
    *
    * Exception safety: noexcept if T's move operations are noexcept.
    *
    * Performance: Wait-free - never blocks or spins.
    *
    * Note: Uses acquire on cell sequence to ensure producer's writes
    * to the element are visible before we read it.
    */
   bool pop(T& out) noexcept(std::is_nothrow_move_assignable_v<T> || std::is_nothrow_move_constructible_v<T>)
   {
      auto position = tail.load(std::memory_order_relaxed);
      auto& cell = cells[position & mask];

      auto sequence = cell.sequence.load(std::memory_order_acquire);
      auto diff = static_cast<intptr_t>(sequence) - static_cast<intptr_t>(position + 1);

      if (diff == 0) {
         // Cell contains an item for this position
         tail.store(position + 1, std::memory_order_relaxed);

         auto& item = cell.item();

         // Move element to output (choose best move operation)
         if constexpr (std::is_nothrow_move_assignable_v<T>) {
            out = std::move(item);
         } else {
            out = T(std::move(item));
         }

         item.~T();

         // Mark cell free for producer at position + N (wraparound)
         // Release ensures destruction happens-before producer writes
         cell.sequence.store(position + N, std::memory_order_release);
         return true;
      }

      // Buffer is empty (or producer is mid-flight writing)
      return false;
   }

private:
   /**
    * @brief Bit mask for fast modulo (N - 1, since N is power of 2)
    *
    * Used to map position to cell index via bitwise AND instead of
    * expensive modulo operation: position & mask == position % N
    */
   static constexpr std::size_t mask = N - 1;

   /**
    * @brief Ring buffer cell containing sequence and storage
    *
    * Each cell has:
    * - Atomic sequence number for synchronization
    * - Uninitialized storage for one T element
    *
    * The sequence number tracks the cell's state:
    * - Even positions: cell is empty, ready for producer
    * - Odd positions: cell contains data, ready for consumer
    * - Large positions (+ N): cell freed by consumer, waiting for wraparound
    */
   struct cell
   {
      std::atomic<std::size_t> sequence;  ///< Synchronization sequence number
      alignas(T) std::array<std::byte, sizeof(T)> storage{};  ///< Uninitialized storage for T

      constexpr cell() noexcept = default;
      /**
       * @brief Construct cell with initial sequence number
       * @param seq Initial sequence (typically the cell's index)
       */
      constexpr explicit cell(std::size_t seq) noexcept : sequence(seq) {}

      T&       item()       noexcept { return *std::launder(reinterpret_cast<T*>(storage.data())); }
      T const& item() const noexcept { return *std::launder(reinterpret_cast<T const*>(storage.data())); }
   };

   /**
    * @brief Index sequence constructor - initializes cells with sequential numbers
    * @tparam Is Index pack (0, 1, 2, ..., N-1)
    *
    * This expands to: cells{ cell{0}, cell{1}, cell{2}, ..., cell{N-1} }
    * Each cell's sequence starts at its index, establishing the initial state.
    */
   template<std::size_t... Is>
   constexpr explicit mpsc_ring_buffer(std::index_sequence<Is...>) noexcept : cells{ cell{Is}... } {}

   // Memory layout optimized to reduce false sharing:
   // Each atomic is on its own cache line (CYROS_PORT_CACHE_LINE bytes)
   // to prevent producers from invalidating consumer's cache and vice versa

   alignas(CYROS_PORT_CACHE_LINE) std::atomic<std::size_t> head{0};  ///< Producer claim counter (many writers)
   alignas(CYROS_PORT_CACHE_LINE) std::atomic<std::size_t> tail{0};  ///< Consumer pop counter (single writer)
   alignas(CYROS_PORT_CACHE_LINE) std::array<cell, N> cells{};       ///< Ring buffer cells
};

}  // namespace cyros

#endif // CYROS_MPSC_RING_BUFFER_HPP
