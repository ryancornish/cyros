/**
 * @file test_riscv_tickless.cpp
 * @brief Tickless MTIME: a 64-bit clock read and armed in 32-bit halves.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: the RISC-V MTIME time source (port/riscv/common/port_time_mtime.cpp)
 *          in tickless mode, driven through the `tickless` time driver.
 * Trusts:  layers 0 to 2, and the periodic path that test_cortex_m_systick
 *          proves on this port too.
 * Proves:  that now() is MTIME and stays monotonic across the carry out of its
 *          low word, that a deadline armed across that carry fires on time,
 *          that the counter rate is the real one, that deadlines already
 *          due fire once and leave the clock running, and that a later
 *          deadline pending behind an earlier one fires after it.
 *
 *
 * THE CARRY IS THE POINT
 * ======================
 * MTIME never wraps in practice, which is why this driver is a fraction of the
 * SysTick one. What an rv32 hart CAN get wrong is that it reaches the 64-bit
 * counter and compare in two 32-bit halves. A read that takes the low word on
 * one side of a carry and the high word on the other is out by 2^32 counts,
 * and a compare written in the wrong order holds, for a moment, a value far in
 * the past or the future.
 *
 * The carry comes every 2^32 counts, 429 s at QEMU's 10 MHz. So the image moves
 * MTIME with cyros_bench_mtime_set. Before the kernel starts it winds MTIME
 * back and forth across the carry through the port contract alone, hundreds of
 * times. Then it moves MTIME to a little before the next carry and starts the
 * kernel, so the kernel and the time driver see that carry happen under them.
 * A jump made before anything has read the clock is just a different reset
 * value.
 *
 *
 * WHAT IS NOT HERE
 * ================
 * The Cortex-M test's "keeps pace with the core clock" check, which compares
 * now() against the DWT. MTIME is never restarted, so it cannot lose counts
 * across a wake the way a reloaded SysTick did, and QEMU's mcycle follows host
 * time rather than instructions, so the comparison would mean nothing here.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/time/time.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_mcu.h>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <cstddef>
#include <cstdint>

using namespace cyros;

/* The board's MTIME rate, which in tickless mode is also the tick rate. */
extern "C" std::uint32_t cyros_port_mtime_clock_hz(void);
/* The bench's, for this test alone. */
extern "C" void cyros_bench_mtime_set(std::uint64_t value);

namespace
{

constexpr std::size_t stack_size = thread::min_stack_size + 2048;
alignas(CYROS_PORT_STACK_ALIGN) std::byte worker_stack[stack_size];

constexpr std::uint64_t carry = 0x1'0000'0000ull;

/* Where MTIME starts: this long before the carry. Long enough for bring-up and
 * the arm below to finish first, at any plausible host speed. */
std::uint64_t lead_in_counts() { return cyros_port_mtime_clock_hz() / 5u; }   /* 200 ms */

volatile int           fired_count = 0;
volatile std::uint64_t fired_at    = 0;

void on_deadline(void*)
{
   fired_count = fired_count + 1;
   fired_at = time::now().value;
}

void print_count(char const* label, std::uint64_t value)
{
   cyros::bench::print(label);
   cyros::bench::print_hex(static_cast<std::uint32_t>(value >> 32));
   cyros::bench::print("_");
   cyros::bench::print_hex(static_cast<std::uint32_t>(value));
}


void test_tickless_reports_the_counter_rate()
{
   cyros::bench::start("tickless freq_hz is MTIME's rate");
   CYROS_CHECK_EQ(cyros_port_time_freq_hz(), std::uint64_t{cyros_port_mtime_clock_hz()});
}

/**
 * @brief Across the low word's carry: now() never goes back or leaps, and a
 *        deadline on the far side fires on time.
 *
 * The deadline is armed BEFORE the carry, so the compare written holds a high
 * word one above MTIME's. A compare written low half first, or without its
 * high half, holds a value already passed, and the interrupt then fires
 * continuously and starves this thread. A read that pairs halves from either
 * side of the carry shows as a step back or a leap of 2^32.
 */
void test_the_low_word_carry()
{
   cyros::bench::start("now() is monotonic across MTIME's low-word carry, and a deadline beyond it fires on time");

   std::uint64_t const start = time::now().value;
   print_count("  started at ", start);
   cyros::bench::print("\n");
   CYROS_CHECK(start < 5u * carry);
   if (start >= 5u * carry) {
      cyros::bench::print("  bring-up took the whole lead-in, the carry is already behind\n");
      return;
   }

   fired_count = 0;
   std::uint64_t const deadline = 5u * carry + cyros_port_mtime_clock_hz() / 20u;   /* 50 ms past */
   std::uint64_t const requested = deadline - start;
   auto const handle = time::schedule_at(time::time_point{deadline}, on_deadline, nullptr);
   (void)handle;

   /* Spin on now() until the deadline fires, bounded by a count of reads
    * rather than by now(), because a stalled or leaping clock is a failure
    * this has to come back from. */
   constexpr std::uint32_t read_limit = 200'000'000;
   std::uint64_t previous = start;
   bool monotonic = true;
   bool leapt = false;
   std::uint64_t bad_before = 0;
   std::uint64_t bad_after = 0;
   std::uint32_t reads = 0;
   while (fired_count == 0 && reads < read_limit) {
      ++reads;
      std::uint64_t const current = time::now().value;
      bool const back = current < previous;
      bool const leap = !back && current - previous > carry / 2u;
      if ((back || leap) && monotonic && !leapt) {
         bad_before = previous;
         bad_after = current;
      }
      monotonic = monotonic && !back;
      leapt = leapt || leap;
      previous = current;
   }

   print_count("  ended at   ", previous);
   cyros::bench::print("\n");
   if (!monotonic || leapt) {
      print_count("  first bad step ", bad_before);
      print_count(" to ", bad_after);
      cyros::bench::print("\n");
   }
   CYROS_CHECK(monotonic);
   CYROS_CHECK(!leapt);
   CYROS_CHECK(previous >= 5u * carry);    /* the carry was crossed, not hoped for */

   CYROS_CHECK_EQ(fired_count, 1);
   if (fired_count == 1) {
      std::uint64_t const actual = fired_at - start;
      cyros::bench::print("  requested = ");
      cyros::bench::print_hex(static_cast<std::uint32_t>(requested));
      cyros::bench::print("  actual = ");
      cyros::bench::print_hex(static_cast<std::uint32_t>(actual));
      cyros::bench::print("\n");
      CYROS_CHECK(fired_at >= deadline);                  /* never early */
      CYROS_CHECK(actual < requested + requested / 4u);   /* and not lost */
   }
}

void test_the_counter_rate_is_real()
{
   cyros::bench::start("the tickless counter rate is the real one");

   if (!cyros::bench::elapsed_ns_is_available()) {
      cyros::bench::print("  no SYS_ELAPSED reference, skipping\n");
      return;
   }

   std::uint64_t const t0 = time::now().value;
   std::uint64_t const ns0 = cyros::bench::elapsed_ns();
   while ((cyros::bench::elapsed_ns() - ns0) < 200'000'000ull) { }
   std::uint64_t const ns = cyros::bench::elapsed_ns() - ns0;
   std::uint64_t const counts = time::now().value - t0;

   std::uint64_t const measured_hz = (counts * 1'000'000'000ull) / ns;
   std::uint64_t const claimed_hz  = cyros_port_time_freq_hz();

   cyros::bench::print("  claimed = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(claimed_hz));
   cyros::bench::print("  measured = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(measured_hz));
   cyros::bench::print("\n");

   CYROS_CHECK(measured_hz >= (claimed_hz * 90u) / 100u);
   CYROS_CHECK(measured_hz <= (claimed_hz * 110u) / 100u);
}

void test_a_one_shot_fires_at_the_requested_time()
{
   cyros::bench::start("a one-shot deadline is delivered, and on time");

   fired_count = 0;
   fired_at = 0;

   std::uint64_t const delay = cyros_port_mtime_clock_hz() / 50u;   /* 20 ms */
   std::uint64_t const armed_at = time::now().value;
   auto const handle = time::schedule_at(time::time_point{armed_at + delay}, on_deadline, nullptr);
   (void)handle;

   std::uint64_t const give_up = armed_at + 8u * delay;
   while (fired_count == 0 && time::now().value < give_up) { }

   CYROS_CHECK_EQ(fired_count, 1);
   if (fired_count == 0) { return; }

   std::uint64_t const actual = fired_at - armed_at;
   cyros::bench::print("  requested = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(delay));
   cyros::bench::print("  actual = ");
   cyros::bench::print_hex(static_cast<std::uint32_t>(actual));
   cyros::bench::print("\n");
   CYROS_CHECK(actual >= delay);
   CYROS_CHECK(actual < delay + delay / 4u);
}

/**
 * @brief A deadline already due, or a count or two off, fires once and leaves
 *        the clock running.
 *
 * On SysTick this was two real defects (test_cortex_m_tickless). Here the
 * compare is simply written with a value already passed, which raises the
 * interrupt at once, and the check is that it is delivered exactly once rather
 * than lost or repeated. Bounded by iterations, as there.
 */
void test_a_deadline_already_due_still_fires()
{
   cyros::bench::start("a deadline already due fires once, and the clock keeps running");

   constexpr std::uint32_t spin_limit = 20'000'000;
   bool all_fired_once = true;
   bool all_running = true;

   for (std::uint32_t i = 0; i < 64u; ++i) {
      fired_count = 0;
      std::uint64_t const armed_at = time::now().value;
      auto const handle = time::schedule_at(time::time_point{armed_at + (i % 4u)}, on_deadline, nullptr);
      (void)handle;

      std::uint32_t spins = 0;
      while (fired_count == 0 && spins < spin_limit) {
         ++spins;
         asm volatile("" ::: "memory");
      }
      /* A little longer, so a second delivery would have landed. */
      for (std::uint32_t settle = 0; settle < 2000u; ++settle) { asm volatile("" ::: "memory"); }
      bool const fired_once = fired_count == 1;

      std::uint64_t const after = time::now().value;
      spins = 0;
      while (time::now().value == after && spins < spin_limit) { ++spins; }
      bool const running = time::now().value != after;

      if ((!fired_once || !running) && all_fired_once && all_running) {
         cyros::bench::print("  arm ");
         cyros::bench::print_hex(i);
         cyros::bench::print(": fired ");
         cyros::bench::print_hex(static_cast<std::uint32_t>(fired_count));
         cyros::bench::print(running ? ", clock running\n" : ", clock STOPPED\n");
      }
      all_fired_once = all_fired_once && fired_once;
      all_running = all_running && running;
      if (!running) { break; }
   }

   CYROS_CHECK(all_fired_once);
   CYROS_CHECK(all_running);
}

/**
 * @brief Two deadlines pending at once: both fire, in order, each on time.
 *
 * The port keeps the earliest armed and ignores a later arm, so after the
 * first fires the driver's re-arm for the second must land. That needs the
 * port to retire a deadline when it is reached. Without that, the second arm
 * is ignored as later than one already passed, and the compare is rewritten
 * with the passed value until the second deadline comes round.
 */
volatile int           second_count = 0;
volatile std::uint64_t second_at    = 0;

void on_second(void*)
{
   second_count = second_count + 1;
   second_at = time::now().value;
}

void test_two_pending_deadlines_both_fire_in_order()
{
   cyros::bench::start("two pending deadlines both fire, in order, each on time");

   fired_count = 0;
   second_count = 0;
   std::uint64_t const step = cyros_port_mtime_clock_hz() / 50u;   /* 20 ms */
   std::uint64_t const armed_at = time::now().value;
   std::uint64_t const first = armed_at + step;
   std::uint64_t const second = armed_at + 2u * step;
   /* The later one first, so the earlier arm has to lower the compare. */
   auto const h2 = time::schedule_at(time::time_point{second}, on_second, nullptr);
   auto const h1 = time::schedule_at(time::time_point{first}, on_deadline, nullptr);
   (void)h1; (void)h2;

   /* Whether this thread ran at all between the two. A compare left at the
    * passed deadline is an interrupt that never stops, and it still delivers
    * both callbacks on time, because each pass re-reads the clock. What it
    * cannot do is let this thread run in between. */
   bool ran_between = false;
   std::uint64_t const give_up = armed_at + 8u * step;
   while (second_count == 0 && time::now().value < give_up) {
      if (fired_count == 1 && second_count == 0) { ran_between = true; }
   }

   CYROS_CHECK_EQ(fired_count, 1);
   CYROS_CHECK_EQ(second_count, 1);
   CYROS_CHECK(ran_between);
   if (fired_count == 1 && second_count == 1) {
      CYROS_CHECK(fired_at >= first);
      CYROS_CHECK(second_at >= second);
      CYROS_CHECK(fired_at < second_at);
      CYROS_CHECK(second_at - second < step / 4u);
   }
}

/**
 * @brief Re-arming and cancelling many times leaves only the live deadline.
 *
 * Each schedule_at may lower the compare, and each cancel leaves the driver's
 * record where it was, so the ISR sees early interrupts it must re-arm from.
 * After 200 of them, one deadline is armed for real and must fire once, on
 * time, while none of the cancelled ones fires.
 */
void test_cancelled_deadlines_never_fire()
{
   cyros::bench::start("after 200 armed and cancelled deadlines, only the live one fires");

   fired_count = 0;
   std::uint64_t const short_delay = cyros_port_mtime_clock_hz() / 1000u;   /* 1 ms */
   for (int i = 0; i < 200; ++i) {
      auto const h = time::schedule_at(
         time::time_point{time::now().value + short_delay}, on_deadline, nullptr);
      (void)time::cancel(h);
   }

   std::uint64_t const delay = cyros_port_mtime_clock_hz() / 50u;   /* 20 ms */
   std::uint64_t const armed_at = time::now().value;
   auto const live = time::schedule_at(time::time_point{armed_at + delay}, on_deadline, nullptr);
   (void)live;

   std::uint64_t const give_up = armed_at + 8u * delay;
   while (time::now().value < give_up && fired_count == 0) { }
   /* Then as long again, so a stray delivery would show. */
   std::uint64_t const settle = time::now().value + delay;
   while (time::now().value < settle) { }

   CYROS_CHECK_EQ(fired_count, 1);
   if (fired_count >= 1) {
      CYROS_CHECK(fired_at - armed_at >= delay);
   }
}

/**
 * @brief The port's tickless now() never pairs halves from either side of the
 *        carry, over many crossings.
 *
 * The kernel test above crosses the carry once, which catches a read that
 * pairs the halves wrongly only when the carry happens to land between them.
 * Before the kernel starts, nothing depends on MTIME, so it can be wound back
 * freely and the carry crossed hundreds of times. Driven through the port
 * contract directly, with no kernel and no time driver.
 */
void test_the_halves_over_many_crossings()
{
   cyros::bench::start("tickless now() reads MTIME's halves consistently over 256 carries");

   cyros_port_time_setup(0);

   constexpr std::uint32_t crossings = 256;
   constexpr std::uint32_t read_limit = 100'000;
   std::uint32_t crossed = 0;
   std::uint32_t torn = 0;
   for (std::uint32_t i = 0; i < crossings; ++i) {
      /* A few counts short, so the carry lands within the next several reads,
       * at a different point in the read each time. */
      cyros_bench_mtime_set(carry * (1u + i % 4u) - (2u + i % 16u));
      std::uint64_t previous = cyros_port_time_now();
      for (std::uint32_t reads = 0; reads < read_limit; ++reads) {
         std::uint64_t const current = cyros_port_time_now();
         if (current < previous || current - previous > carry / 2u) { ++torn; }
         previous = current;
         if ((current & 0xFFFF'FFFFull) >= 64u && (current & 0xFFFF'FFFFull) < carry / 2u) {
            break;
         }
      }
      if ((previous & 0xFFFF'FFFFull) < carry / 2u) { ++crossed; }
   }

   cyros_port_time_teardown();

   cyros::bench::print("  crossed ");
   cyros::bench::print_hex(crossed);
   cyros::bench::print(", torn reads ");
   cyros::bench::print_hex(torn);
   cyros::bench::print("\n");
   CYROS_CHECK_EQ(crossed, crossings);
   CYROS_CHECK_EQ(torn, 0u);
}

void worker()
{
   time::initialise(cyros_port_mtime_clock_hz());
   time::start();

   test_tickless_reports_the_counter_rate();
   test_the_low_word_carry();
   test_the_counter_rate_is_real();
   test_a_one_shot_fires_at_the_requested_time();
   test_a_deadline_already_due_still_fires();
   test_two_pending_deadlines_both_fire_in_order();
   test_cancelled_deadlines_never_fire();

   cyros::bench::finish();
}

} // namespace


extern "C" int cyros_bench_main()
{
   cyros::bench::print("rv32 tickless MTIME\n\n");

   test_the_halves_over_many_crossings();

   /* Forward from wherever that left it, to a little before the next carry. */
   cyros_bench_mtime_set(5u * carry - lead_in_counts());

   kernel::initialise();
   thread worker_thread(worker, worker_stack, thread::priority(0), core0);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
