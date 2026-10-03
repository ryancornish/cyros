#ifndef CYROS_EVENT_FLAGS_HPP
#define CYROS_EVENT_FLAGS_HPP

#include <cyros/kernel/waitable.hpp>
#include <cyros/kernel/visibility.hpp>

#include <atomic>
#include <cstdint>

namespace cyros::time { struct time_point; struct duration; }

namespace cyros::sync
{

/// Whether a wait needs ANY of its mask's bits set, or ALL of them.
enum class flags_match : bool
{
   any,
   all,
};

/// What a satisfied wait does to the bits it matched.
enum class flags_exit : bool
{
   keep,    ///< leave them set, for every other waiter to see too
   consume, ///< clear them, atomically with the test that satisfied the wait
};

/**
 * @brief 32 flag bits that threads wait on by mask.
 *
 * "Any of these bits" or "all of these bits", which a group wait cannot
 * express: wait_on_any gives "any of several objects", not "all of several
 * conditions". The bits are a level, so a set that lands before a waiter
 * parks is seen by its poll, and nothing is lost.
 *
 * set() wakes EVERY waiter to re-test its own mask, since different waiters
 * want different bits, and those still unsatisfied park again. So a set costs
 * a wakeup per waiter, not per waiter it satisfies. That is the price of
 * testing each waiter's mask in its own poll rather than in the wake.
 *
 * Consuming waiters barge, as a semaphore's do: a set that satisfies two
 * consumers of the same bit goes to whichever polls first, which is normally
 * the more urgent. The other parks again.
 *
 * set() and clear() are ISR-safe. Waiting is thread context only.
 *
 * Not a waitable to users. It inherits privately, because a plain wait_on
 * would poll it with no mask, and a timed wait is provided here instead.
 */
class CYROS_PUBLIC event_flags : private waitable
{
public:
   constexpr explicit event_flags(std::uint32_t initial = 0) noexcept : bits(initial) {}

   /**
    * @brief Set @p mask's bits and wake every waiter to re-test.
    * @return the flags before the set.
    */
   std::uint32_t set(std::uint32_t mask) noexcept;

   /**
    * @brief Clear @p mask's bits. Wakes nobody, since clearing satisfies no one.
    * @return the flags before the clear.
    */
   std::uint32_t clear(std::uint32_t mask) noexcept;

   [[nodiscard]] std::uint32_t peek() const noexcept;

   /**
    * @brief Block until @p mask is matched.
    *
    * @param mask Non-zero. An empty mask is a caller error, which keeps 0
    *        free to mean "not satisfied" in try_wait and the timed waits.
    * @return the bits of @p mask that were set when the wait was satisfied,
    *         so all of @p mask for flags_match::all. With flags_exit::consume
    *         these are exactly the bits this wait cleared.
    */
   std::uint32_t wait(std::uint32_t mask,
                      flags_match match = flags_match::any,
                      flags_exit  exit  = flags_exit::keep) noexcept;

   /**
    * @brief wait() without blocking. @return 0 when not satisfied.
    */
   [[nodiscard]] std::uint32_t try_wait(std::uint32_t mask,
                                        flags_match match = flags_match::any,
                                        flags_exit  exit  = flags_exit::keep) noexcept;

   /**
    * @brief wait(), giving up at @p tp / after @p d. @return 0 on timeout.
    *
    * PROVIDED BY THE CHRONO FEATURE, like semaphore's timed methods: sync is
    * time-free, and calling one without chrono is a link error naming the
    * method. A deadline at or before now degrades to try_wait.
    */
   [[nodiscard]] std::uint32_t try_wait_until(std::uint32_t mask, time::time_point tp,
                                              flags_match match = flags_match::any,
                                              flags_exit  exit  = flags_exit::keep) noexcept;

   [[nodiscard]] std::uint32_t try_wait_for(std::uint32_t mask, time::duration d,
                                            flags_match match = flags_match::any,
                                            flags_exit  exit  = flags_exit::keep) noexcept;

private:
   /// What one wait asks for, and what satisfied it.
   struct request
   {
      std::uint32_t mask;
      flags_match   match;
      flags_exit    exit;
      std::uint32_t matched{0};
   };

   /// A blocking wait's record, on the waiter's stack. Touched only by the
   /// waiter. try_wait needs no record, so it installs nothing on the thread.
   struct waiter : waiter_record
   {
      waiter(waitable& source, request r) noexcept : waiter_record(source), req(r) {}

      request req;
   };

   /// Test @p r's mask against the flags, consuming if asked. Shared by the
   /// poll and try_wait, so a blocking and a non-blocking wait can never
   /// disagree about what satisfies them.
   bool attempt(request& r) noexcept;

   bool try_satisfy(waiter_record* record) noexcept override;

   std::atomic<std::uint32_t> bits;
};

}  // namespace cyros::sync

namespace cyros
{
using sync::event_flags;
using sync::flags_exit;
using sync::flags_match;
}

#endif // CYROS_EVENT_FLAGS_HPP
