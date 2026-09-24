/**
 * @file channel.hpp
 * @brief A bounded FIFO for inter-thread communication.
 *
 * A channel is a unidirectional pipe where one thread can send information
 * through it, and another thread can receive that information from it.
 * Though there are no restrictions on how many threads might want to produce data
 * (producers) and how many threads want to consume data (consumers) on a single
 * channel... the convention is that each thread instantiates its own channel,
 * and it parks on the channel to wait for any thread to give it something to do.
 *
 * Produces may put plain old data into the channel (generic), or may put a callable
 * into the channel (specialisation). A callable can then be executed by the consumer
 * in order to do request work.
 *
 * The channel itself is not tied to a thread by construction, nor does it have any
 * concept of priorities, core affinities, producer counts etc. These details must be
 * coordinated by the threads who use the channel. The channel does provide a plethora
 * of ways to send/receive that results in desired threading behaviour.
 * e.g. the ability to send and block until the data is consumed.
 *
 * Caution is advised in sending T where sizeof(T) is large. The process of moving T
 * into a channel's slot happens with interrupts mask on the producing threads core.
 * Not a good idea to keep interrupts mask because you are moving 256 bytes of data!
 * Send a pointer or a pool index instead.
 *
 * A channel transports values. A work queue is what you get when the value
 * happens to be callable, so `work_channel` is an alias rather than a second
 * type and `run` is a free function. Transporting data rather than behaviour is
 * what lets an ISR hand over a sample without its translation unit having to
 * see how the sample is processed.
 *
 * Workers are threads the caller creates, each calling `run`. The channel owns
 * no threads and no stacks, so worker count, priority and core affinity are
 * entirely the application's. Priority is the worker's, not the job's: a
 * channel is FIFO, and an application wanting two job priorities uses two
 * channels with workers at two thread priorities. That keeps the channel out of
 * any interaction with the scheduler beyond ordinary blocking, and in
 * particular out of priority inheritance, since jobs are not resources and
 * nothing holds them.
 */

#ifndef CYROS_CHANNEL_HPP
#define CYROS_CHANNEL_HPP

#include <cyros/kernel/assert.hpp>
#include <cyros/kernel/function.hpp>
#include <cyros/kernel/spinlock.hpp>
#include <cyros/sync/semaphore.hpp>

#include <array>
#include <concepts>
#include <cstddef>
#include <optional>
#include <type_traits>
#include <utility>

namespace cyros::ch
{

/**
 * @brief A bounded FIFO of `Capacity` values of `T`.
 *
 * OVERFLOW POLICY BELONGS TO THE PRODUCER, NOT TO THE CHANNEL. The same channel
 * legitimately has a thread producer that should throttle and an ISR producer
 * that must drop, so there is no channel-wide policy to disagree with. Which
 * send you call is a statement about your own overflow policy:
 *
 *   send             full is a sizing bug      any context   hard error
 *   try_send         returns false             any context   caller owns the drop
 *   send_overwrite   drops the oldest          any context   deliberately lossy
 *   send_blocking    waits for space           THREAD ONLY   see the warning below
 *
 * The unmarked verb is the strict one. Anything that tolerates a full channel
 * has to say so at the call site, which makes it answerable in review.
 *
 * The two sides are asymmetric, and that is the statement rather than an
 * oversight: a FULL channel is an error, an EMPTY one is normal. A receiver
 * waiting is the design working. A producer waiting means the system is behind.
 *
 * WARNING, send_blocking is an unbounded priority inversion. A high-priority
 * producer blocked on a full channel is waiting for a worker to drain it, and a
 * medium-priority thread that preempts that worker delays the producer with no
 * bound. `mutex` does priority inheritance and a semaphore cannot, because there
 * is no single owner to boost. So: a thread that calls send_blocking must not
 * be higher priority than the workers draining that channel. A high-priority
 * producer uses send or try_send and owns its own overflow policy.
 *
 * ISR SAFETY. Everything except send_blocking and receive is callable from an
 * ISR. The value is moved under a spinlock, which masks interrupts on the
 * sending core, so an ISR producer and a thread producer on the SAME core need
 * no further reasoning. A received value is moved OUT before the lock is
 * released and returned after, so a job never runs under the lock or with
 * interrupts masked. That ordering is the entire point of the exercise.
 */
template<typename T, std::size_t Capacity>
class channel
{
   static_assert(Capacity > 0, "a channel needs at least one slot");
   static_assert(std::is_default_constructible_v<T>,
                 "slot storage is a std::array<T, Capacity>, so T must be default constructible. "
                 "Wrap a type that is not in std::optional, or send a pointer to it.");
   static_assert(std::is_nothrow_move_assignable_v<T>,
                 "a throwing move under the channel's spinlock would be unrecoverable");
   static_assert(std::is_nothrow_move_constructible_v<T>,
                 "a throwing move under the channel's spinlock would be unrecoverable");

public:
   using value_type = T;
   static constexpr std::size_t capacity = Capacity;

   constexpr channel() = default;
   ~channel() = default;

   channel(channel const&)            = delete;
   channel& operator=(channel const&) = delete;
   channel(channel&&)                 = delete;
   channel& operator=(channel&&)      = delete;

   /**
    * @brief Send, treating a full channel as a sizing bug. Any context.
    *
    * Panics when the channel is full, because capacity is supposed to come from
    * producers times rate times worst-case drain latency, and exceeding it
    * means that arithmetic was wrong. Continuing quietly is worse than stopping.
    *
    * A STOPPED channel is refused quietly, not panicked on. Shutdown racing a
    * send is an ordinary lifecycle race, not a sizing defect.
    */
   void send(T v) noexcept
   {
      outcome const result = try_place(v);
      CYROS_REQUIRE1(result != outcome::full, Capacity);
   }

   /**
    * @brief Send if there is room. Any context. Never blocks.
    * @return true when the value was queued, false when full or stopped.
    */
   [[nodiscard]] bool try_send(T v) noexcept
   {
      return try_place(v) == outcome::queued;
   }

   /**
    * @brief Send, dropping the OLDEST queued value when there is no room.
    *        Any context. Never blocks and never fails.
    *
    * For a stream where the newest value is the one that matters and an old one
    * is worth less than a stall, a sensor sample being the usual case.
    * Surviving values stay in FIFO order, it is only the front that is lost.
    *
    * Drops the NEW value instead when there is nothing to overwrite: an empty
    * ring whose every slot is already reserved by a send in flight, or a
    * channel that has been stopped.
    */
   void send_overwrite(T v) noexcept
   {
      if (space.try_acquire()) {
         if (place_owning_space(v) != outcome::queued) {
            space.release();
         }
         return;
      }

      /* Replacing the oldest touches NEITHER semaphore, and that is not an
       * oversight. A token stands for one occupied slot, not for one particular
       * value, and this leaves the occupancy exactly as it found it: one value
       * out at the head, one in at the tail. So every token still has a slot
       * behind it and no wake is owed to anyone.
       *
       * An earlier draft took the dropped value's `items` token and gave it
       * straight back, on the theory that a token must not outlive the value it
       * stood for. Mutation testing killed that: removing the whole round trip
       * left every reachable state identical, because the invariant is
       * tokens <= occupancy and occupancy never moves here. All it did was buy
       * a semaphore call under the spinlock, which is now the one thing this
       * class never does. */
      spinlock_guard guard(lock);
      if (!stopped && occupancy > 0) {
         head = advance(head);        // drop the oldest
         ring[tail] = std::move(v);   // append the newest
         tail = advance(tail);
      }
   }

   /**
    * @brief Send, waiting while the channel is full. THREAD CONTEXT ONLY.
    *
    * Read the priority inversion warning on the class before using this.
    * Returns without queueing if the channel is stopped, either on arrival or
    * while waiting.
    */
   void send_blocking(T v) noexcept
   {
      {
         spinlock_guard guard(lock);
         if (stopped) return;
      }

      space.acquire();  // a freed slot, or the token stop() left behind

      outcome result = outcome::stopped;
      {
         spinlock_guard guard(lock);
         if (!stopped) {
            ring[tail] = std::move(v);
            tail = advance(tail);
            ++occupancy;
            result = outcome::queued;
         }
      }
      if (result == outcome::queued) { items.release(); }
      else                           { space.release(); }  // relay, see stop()
   }

   /**
    * @brief Take the oldest value, waiting for one. THREAD CONTEXT ONLY.
    * @return the value, or empty once the channel is stopped AND drained.
    *
    * Empty is the loop's terminator, which is why the return is an optional
    * rather than a blocking `T`: `while (auto v = ch.receive())` ends exactly
    * when the channel is finished, with no separate flag to consult.
    */
   [[nodiscard]] std::optional<T> receive() noexcept
   {
      {
         spinlock_guard guard(lock);
         if (occupancy == 0 && stopped) return std::nullopt;
      }

      items.acquire();

      std::optional<T> out;
      {
         spinlock_guard guard(lock);
         out = take_locked();
      }
      return settle(std::move(out));
   }

   /**
    * @brief Take the oldest value if there is one. Any context. Never blocks.
    * @return the value, or empty when the channel is empty.
    */
   [[nodiscard]] std::optional<T> try_receive() noexcept
   {
      if (!items.try_acquire()) return std::nullopt;

      std::optional<T> out;
      {
         spinlock_guard guard(lock);
         out = take_locked();
      }
      return settle(std::move(out));
   }

   /**
    * @brief Finish the channel: drain what is queued, refuse what comes after.
    *
    * Queued values are still delivered. Later sends are refused. Every blocked
    * receiver and blocked sender is released, so `run` returns and the kernel
    * can quiesce.
    *
    * Terminal. There is no restart, because restarting would mean deciding what
    * happens to values sent during the dead window, and that question has no
    * good answer.
    *
    * ONE TOKEN EACH IS ENOUGH, AND THAT IS THE WHOLE TRICK. stop() does not own
    * the threads, so it cannot know how many are blocked, and an earlier draft
    * of this class counted them under the lock in order to release one token
    * per participant. It did not need to. Every path that consumes a token and
    * finds the channel dead RELEASES IT AGAIN before returning, so a single
    * token walks from one blocked participant to the next until none are left.
    * That turns "this channel is finished" from a one-shot signal, which a
    * try_receive could steal and strand a blocked receiver forever, into a
    * durable state that anyone who asks will observe.
    *
    * The relay is therefore load-bearing, not a tidy-up. It has three homes:
    * settle() on the receiving side, and the tails of send_blocking and
    * try_place on the sending side. Removing any one of them strands whoever
    * was next in line on that side.
    */
   void stop() noexcept
   {
      {
         spinlock_guard guard(lock);
         if (stopped) return;
         stopped = true;
      }
      items.release();
      space.release();
   }

   /** @brief Queued values right now. Racy the instant it returns, for diagnostics only. */
   [[nodiscard]] std::size_t size() const noexcept
   {
      spinlock_guard guard(lock);
      return occupancy;
   }

   /** @brief Whether stop() has been called. */
   [[nodiscard]] bool is_stopped() const noexcept
   {
      spinlock_guard guard(lock);
      return stopped;
   }

private:
   enum class outcome
   {
      queued,
      full,
      stopped,
   };

   static constexpr std::size_t advance(std::size_t index) noexcept
   {
      /* A compare and select rather than a modulo, so any capacity costs the
       * same and a non-power-of-two one does not pull in a division. */
      return (index + 1u == Capacity) ? 0u : index + 1u;
   }

   /* Place a value whose space token the caller already holds. */
   outcome place_owning_space(T& v) noexcept
   {
      {
         spinlock_guard guard(lock);
         if (stopped) { return outcome::stopped; }
         ring[tail] = std::move(v);
         tail = advance(tail);
         ++occupancy;
      }
      items.release();
      return outcome::queued;
   }

   outcome try_place(T& v) noexcept
   {
      if (!space.try_acquire()) {
         /* No token does NOT mean full. After stop() the space semaphore holds
          * the relay token, and a second sender racing the first for it comes
          * away empty-handed on a channel that is merely finished. Reporting
          * that as `full` would make send() panic on an ordinary shutdown race,
          * which is the opposite of what the strict send is for. */
         spinlock_guard guard(lock);
         return stopped ? outcome::stopped : outcome::full;
      }
      outcome const result = place_owning_space(v);
      if (result != outcome::queued) { space.release(); }  // relay, see stop()
      return result;
   }

   /* Assumes `lock` is held. */
   std::optional<T> take_locked() noexcept
   {
      if (occupancy == 0) { return std::nullopt; }
      std::optional<T> out{std::move(ring[head])};
      head = advance(head);
      --occupancy;
      return out;
   }

   /* The tail shared by both receives, run with the lock RELEASED because both
    * arms wake somebody.
    *
    * An empty result means the consumed `items` token had no value behind it,
    * which only happens for the token stop() leaves behind. Passing it back on
    * is the receiving half of the relay that stop() describes, and it is why
    * stop() can release one token rather than counting blocked receivers. */
   std::optional<T> settle(std::optional<T> out) noexcept
   {
      if (out) { space.release(); }
      else     { items.release(); }
      return out;
   }

   /* THE ONE RULE THIS CLASS CANNOT BREAK: no semaphore call of any kind while
    * holding `lock`. acquire and release both reach the scheduler, and `lock`
    * masks interrupts on this core. The rule has no exceptions, which is worth
    * more than the one call it used to cost. */
   mutable spinlock lock;

   std::array<T, Capacity> ring{};

   /* OCCUPANCY IS READ FROM HERE, NEVER INFERRED FROM items.peek(). A receiver
    * that holds an items token but has not yet reached the lock leaves the
    * semaphore reading zero while the ring is still full, so anything that
    * trusted the semaphore would conclude the ring was empty and be wrong.
    *
    * It is `stop()` that makes this field necessary, not send_overwrite. The
    * token stop() leaves behind has no value under it, so take_locked has to be
    * able to tell a token that stands for a queued value from that one, and
    * head == tail cannot: it means full and empty alike. */
   std::size_t head      = 0;
   std::size_t tail      = 0;
   std::size_t occupancy = 0;

   bool stopped = false;

   sync::semaphore items{0};
   sync::semaphore space{Capacity};
};

/**
 * @brief A bounded callable, sized at the use site.
 *
 * `no_heap` means a lambda whose captures exceed `JobSize` is a COMPILE error,
 * not a runtime surprise.
 */
template<std::size_t JobSize = 32>
using job = function<void(), JobSize, heap_policy::no_heap>;

/**
 * @brief The work queue: a channel whose value happens to be callable.
 *
 * An alias, not a type. There is nothing to keep in sync with `channel`, and a
 * channel of anything else callable gets `run` just the same.
 */
template<std::size_t Capacity, std::size_t JobSize = 32>
using work_channel = channel<job<JobSize>, Capacity>;

/**
 * @brief Worker thread body: run jobs until the channel is stopped and drained.
 *
 * The whole of a worker thread. Jobs run in thread context with interrupts
 * enabled and the full kernel API available, which is the point of deferring
 * them out of an ISR in the first place.
 *
 * With several workers, jobs START in FIFO order and finish in whatever order
 * they finish.
 */
template<typename T, std::size_t Capacity> requires std::invocable<T&>
void run(channel<T, Capacity>& ch)
{
   while (auto work = ch.receive()) {
      (*work)();
   }
}

/**
 * @brief Wait for one job and run it.
 * @return false once the channel is stopped and drained, so `while (run_one(ch))`
 *         is `run(ch)`.
 */
template<typename T, std::size_t Capacity> requires std::invocable<T&>
[[nodiscard]] bool run_one(channel<T, Capacity>& ch)
{
   auto work = ch.receive();
   if (!work) { return false; }
   (*work)();
   return true;
}

}  // namespace cyros::ch

namespace cyros
{
using ch::channel;
using ch::job;
using ch::work_channel;
}

#endif  // CYROS_CHANNEL_HPP
