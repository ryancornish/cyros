/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file test_armv6m_atomics_out_of_line.cpp
 * @brief Every atomic load and store on ARMv6-M is a call (L2). Runs on
 *        QEMU's microbit and on the RP2040, one core or two.
 *
 * Subject / Trusts / Proves
 * -------------------------
 * Subject: -fno-inline-atomics, which the ARMv6-M root toolchain
 *          (arm-none-eabi-debug-m0.toml) puts on every ARMv6-M build.
 * Trusts:  the bench harness, and the kernel's bring-up to run, not to be
 *          right: nothing here checks what the kernel stored.
 * Proves:  that atomic loads and stores of one and four bytes call
 *          __atomic_load_N and __atomic_store_N, both in a translation unit
 *          compiled as a user's is and in libcyros.a, and that the calls
 *          reach the port's own and store what they were given.
 *
 * Why it matters: on two cores GCC's inline stores take no lock, so one can
 * land inside another core's read-modify-write and be lost. With the flag
 * every store is a call, and port_atomic_armv6m.cpp takes the same lock for
 * it as for a read-modify-write. A build without the flag compiles, links and
 * runs, so this test is how a dropped flag is noticed. No compiler macro shows
 * it (GCC 16 predefines the same with and without), so it has to be the link
 * or the run.
 *
 * How: the link wraps the four calls (test.toml), and each wrapper counts,
 * then forwards to the real one. --wrap redirects every undefined reference,
 * the archive members' included. A build without the flag never calls the
 * wrappers and fails every check below. Whether the store takes the lock is
 * test_rp2040_smp_atomic_store's subject, on the board.
 *
 * Layer 2 because the kernel's own accesses are only reached by bringing it
 * up. The kernel is never started.
 */

#include <cyros/kernel/kernel.hpp>

#include <common/bench.hpp>

#include <atomic>
#include <cstdint>

using namespace cyros;

/* GCC's own signatures for the calls (its sync-builtins.def), with unsigned
 * int as the 4-byte type, as port_atomic_armv6m.cpp says. */
extern "C" {
unsigned char __real___atomic_load_1(volatile void const* p, int order);
void          __real___atomic_store_1(volatile void* p, unsigned char v, int order);
unsigned int  __real___atomic_load_4(volatile void const* p, int order);
void          __real___atomic_store_4(volatile void* p, unsigned int v, int order);
}

namespace
{

/* Plain volatiles: an atomic here would count itself. */
volatile std::uint32_t loads_1  = 0;
volatile std::uint32_t stores_1 = 0;
volatile std::uint32_t loads_4  = 0;
volatile std::uint32_t stores_4 = 0;

struct counts
{
   std::uint32_t loads_1, stores_1, loads_4, stores_4;
};

counts now() { return { loads_1, stores_1, loads_4, stores_4 }; }

/* Globals, so no access can be folded away. */
std::atomic<std::uint32_t> user_word{0};
std::atomic<bool> user_flag{false};

void test_a_users_word_is_loaded_and_stored_by_calls()
{
   cyros::bench::start("a user's 4-byte store and load are calls");
   counts const before = now();
   user_word.store(0x5A5A1234u);
   std::uint32_t const read = user_word.load();
   counts const after = now();
   CYROS_CHECK_EQ(after.stores_4 - before.stores_4, 1u);
   CYROS_CHECK_EQ(after.loads_4 - before.loads_4, 1u);
   CYROS_CHECK_EQ(read, 0x5A5A1234u);
}

void test_a_users_flag_is_loaded_and_stored_by_calls()
{
   cyros::bench::start("a user's 1-byte store and load are calls");
   counts const before = now();
   user_flag.store(true, std::memory_order_release);
   bool const read = user_flag.load(std::memory_order_acquire);
   counts const after = now();
   CYROS_CHECK_EQ(after.stores_1 - before.stores_1, 1u);
   CYROS_CHECK_EQ(after.loads_1 - before.loads_1, 1u);
   CYROS_CHECK(read);
}

/* kernel::initialise() loads and stores its `initialised` flag, and
 * active_threads() is one 4-byte load. Exact for the latter, at least one for
 * the former, so the test does not pin the kernel's internals. */
void test_the_kernels_own_accesses_are_calls()
{
   cyros::bench::start("the kernel's own loads and stores are calls");
   counts const before = now();
   kernel::initialise();
   counts const initialised = now();
   std::uint32_t const threads = kernel::active_threads();
   counts const after = now();
   CYROS_CHECK(initialised.loads_1 - before.loads_1 >= 1u);
   CYROS_CHECK(initialised.stores_1 - before.stores_1 >= 1u);
   CYROS_CHECK_EQ(after.loads_4 - initialised.loads_4, 1u);
   CYROS_CHECK_EQ(threads, 0u);
}

} // namespace

extern "C" {

unsigned char __wrap___atomic_load_1(volatile void const* p, int order)
{
   loads_1 = loads_1 + 1u;
   return __real___atomic_load_1(p, order);
}

void __wrap___atomic_store_1(volatile void* p, unsigned char v, int order)
{
   stores_1 = stores_1 + 1u;
   __real___atomic_store_1(p, v, order);
}

unsigned int __wrap___atomic_load_4(volatile void const* p, int order)
{
   loads_4 = loads_4 + 1u;
   return __real___atomic_load_4(p, order);
}

void __wrap___atomic_store_4(volatile void* p, unsigned int v, int order)
{
   stores_4 = stores_4 + 1u;
   __real___atomic_store_4(p, v, order);
}

}

/* Entry point, called by the board startup. See bench.hpp. */
extern "C" int cyros_bench_main()
{
   cyros::bench::print("armv6m atomic loads and stores out of line\n\n");

   test_a_users_word_is_loaded_and_stored_by_calls();
   test_a_users_flag_is_loaded_and_stored_by_calls();
   test_the_kernels_own_accesses_are_calls();

   cyros::bench::finish();
}
