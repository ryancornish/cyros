/**
 * @file test_rp2040_smp_atomic_store.cpp
 * @brief An atomic store on one RP2040 core waits for the lock the other
 *        holds (L2). Runs on the RP2040 Pico, both cores.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: port_atomic_armv6m.cpp's __atomic_store_1 and __atomic_store_4 on
 *          two cores, under rp2040_smp's lock, SIO spinlock 14.
 * Trusts:  test_armv6m_atomics_out_of_line, that the toolchain makes every
 *          atomic store a call, and the two-core bring-up
 *          (test_cortex_m_smp_bringup) to put a thread on each core.
 * Proves:  that a std::atomic store of a word and of a bool, from core 1,
 *          does not land, and does not return, while core 0 holds the lock,
 *          and lands once core 0 releases it. So a store cannot fall inside
 *          another core's read-modify-write.
 *
 * test_armv6m_atomics_out_of_line proves the store is a call, which the
 * board's semantics then rest on. That a call takes the lock only a second
 * core can show, so this is the board half. Each phase: core 1 masks its
 * interrupts and waits. Core 0 masks its own, takes spinlock 14 and says so.
 * Core 1 says it is storing and stores. Core 0 holds the lock 2 ms more,
 * watching the object with a plain load, then releases and waits for core
 * 1's store to return.
 *
 * Every hand-off is a plain volatile word, because an atomic one would be a
 * call into the lock core 0 is holding. Both cores mask interrupts for the
 * phase, so neither takes a tick that does a read-modify-write: on core 0
 * that would deadlock on its own lock, and on core 1 it would block before
 * the store, which the test could not tell from the store blocking.
 *
 * Mutants: the toolchain without -fno-inline-atomics, and a store without
 * the lock, each land the store inside the hold and fail.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/core.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <common/bench.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>

using namespace cyros;

static_assert(config::cores == 2, "Both cores take part");
static_assert(CYROS_PORT_CORE_COUNT == 2, "Needs the rp2040_smp port");

namespace
{

/* RP2040 datasheet 2.3.1 and 4.6, spelled out rather than taken from the
 * port, so the test does not only prove the port agrees with itself. */
constexpr std::uintptr_t sio_spinlock14 = 0xd0000000u + 0x100u + 4u * 14u;
constexpr std::uintptr_t timer_rawl     = 0x40054000u + 0x28u;

volatile std::uint32_t& reg(std::uintptr_t address)
{
   return *reinterpret_cast<volatile std::uint32_t*>(address);
}

std::uint32_t now_us() { return reg(timer_rawl); }

void dmb() { asm volatile("dmb" ::: "memory"); }

std::uint32_t mask()
{
   std::uint32_t primask;
   asm volatile("mrs %0, primask\n\tcpsid i" : "=r"(primask) :: "memory");
   return primask;
}

void unmask(std::uint32_t primask)
{
   asm volatile("msr primask, %0" :: "r"(primask) : "memory");
}

constexpr std::uint32_t hold_us    = 2'000u;
constexpr std::uint32_t timeout_us = 1'000'000u;

/* The objects stored to, one per width the kernel uses. */
std::atomic<std::uint32_t> word{0};
std::atomic<bool> flag{false};

constexpr std::uint32_t word_value = 0xC0FFEE42u;

/* Plain loads of the objects, as a debugger would see them: an atomic load
 * would be the very call under test. */
bool word_landed() { return *reinterpret_cast<volatile std::uint32_t const*>(&word) == word_value; }
bool flag_landed() { return *reinterpret_cast<volatile unsigned char const*>(&flag) != 0u; }

/* The hand-offs, each carrying the phase it belongs to. */
volatile std::uint32_t core1_ready   = 0;
volatile std::uint32_t core0_holds   = 0;
volatile std::uint32_t core1_storing = 0;
volatile std::uint32_t core1_stored  = 0;
volatile std::uint32_t core1_stored_at = 0;

constexpr std::uint32_t phase_word = 1u;
constexpr std::uint32_t phase_flag = 2u;

void thread_on_core1()
{
   for (std::uint32_t phase = phase_word; phase <= phase_flag; ++phase) {
      std::uint32_t const primask = mask();
      dmb();
      core1_ready = phase;
      dmb();
      while (core0_holds != phase) {}
      core1_storing = phase;
      dmb();
      if (phase == phase_word) {
         word.store(word_value);
      }
      else {
         flag.store(true);
      }
      core1_stored_at = now_us();
      dmb();
      core1_stored = phase;
      unmask(primask);
   }
}

struct phase_result
{
   bool ready;
   bool storing;
   bool landed_while_held;
   bool returned_while_held;
   bool returned;
   bool landed;
   std::uint32_t released_at;
   std::uint32_t stored_at;
};

/* Bounded in TIMER microseconds, yielding while interrupts are still on. */
template <typename Done>
bool wait_for(Done done, bool may_yield)
{
   std::uint32_t const start = now_us();
   while (!done()) {
      if (now_us() - start >= timeout_us) { return false; }
      if (may_yield) { this_thread::yield(); }
   }
   return true;
}

template <typename Landed>
phase_result run_phase(std::uint32_t phase, Landed landed)
{
   phase_result r{};
   r.ready = wait_for([&] { return core1_ready == phase; }, true);
   if (!r.ready) { return r; }

   std::uint32_t const primask = mask();
   while (reg(sio_spinlock14) == 0u) {}
   dmb();
   core0_holds = phase;
   dmb();

   r.storing = wait_for([&] { return core1_storing == phase; }, false);
   std::uint32_t const hold_from = now_us();
   while (now_us() - hold_from < hold_us) {
      if (landed()) { r.landed_while_held = true; }
      if (core1_stored == phase) { r.returned_while_held = true; }
   }

   r.released_at = now_us();
   dmb();
   reg(sio_spinlock14) = 0u;   /* any value releases */
   unmask(primask);

   r.returned = wait_for([&] { return core1_stored == phase; }, true);
   dmb();
   r.landed = landed();
   r.stored_at = core1_stored_at;
   return r;
}

void report(char const* what, phase_result const& r)
{
   cyros::bench::start(what);
   CYROS_CHECK(r.ready);
   CYROS_CHECK(r.storing);
   CYROS_CHECK(!r.landed_while_held);
   CYROS_CHECK(!r.returned_while_held);
   CYROS_CHECK(r.returned);
   CYROS_CHECK(r.landed);
   /* The store returned no earlier than the release, in TIMER microseconds. */
   auto const after = static_cast<std::int32_t>(r.stored_at - r.released_at);
   CYROS_CHECK(after >= 0);
   cyros::bench::print(after >= 0 ? "  returned " : "  returned BEFORE the release, by ");
   cyros::bench::print_dec(static_cast<std::uint32_t>(after >= 0 ? after : -after));
   cyros::bench::print(after >= 0 ? " us after the release\n" : " us\n");
}

void thread_on_core0()
{
   phase_result const w = run_phase(phase_word, word_landed);
   phase_result const f = w.ready ? run_phase(phase_flag, flag_landed) : phase_result{};

   report("a word stored on core 1 waits for the lock core 0 holds", w);
   report("a bool stored on core 1 waits for the lock core 0 holds", f);

   cyros::bench::finish();
}

constexpr std::size_t stack_size = thread::min_stack_size + 1024;

alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_core0[stack_size];
alignas(CYROS_PORT_STACK_ALIGN) std::byte stack_core1[stack_size];

} // namespace


/* Entry point, called by the board startup. See bench.hpp. */
extern "C" int cyros_bench_main()
{
   cyros::bench::print("rp2040 two-core atomic store under the lock\n\n");

   kernel::initialise();

   thread t0(thread_on_core0, stack_core0, thread::priority(0), core0);
   thread t1(thread_on_core1, stack_core1, thread::priority(0), core1);

   kernel::start();

   cyros::bench::print("FAIL: kernel::start() returned on a target port\n");
   cyros::bench::host_exit(5u);
}
