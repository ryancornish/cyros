/**
 * @file port_atomic_armv6m.cpp
 * @brief The atomic operations ARMv6-M has no instructions for.
 *
 * ARMv6-M has no LDREX/STREX. GCC still compiles every std::atomic, keeping
 * aligned loads and stores up to a word inline, since those are single-copy
 * atomic, and turning each read-modify-write into a call: __atomic_fetch_add_4,
 * __atomic_compare_exchange_1 and so on. Nothing on the target supplies them,
 * so a link with the kernel fails until something does, and that is this
 * file.
 *
 * Each is a plain load and store with PRIMASK set, which is atomic against
 * this core's interrupts. That is the whole of atomicity on one core. A second
 * core is not stopped by PRIMASK (cross-core-validation.md 1b): on the
 * RP2040, 2 x 200,000 increments under PRIMASK alone lost about 27,000
 * (arm-port-notes.md 19h). So a multicore target also supplies a lock the
 * cores share, cortex_m::atomic_lock() and atomic_unlock(), taken inside the
 * masked region, and on one core the calls are compiled out.
 *
 *
 * Loads and stores, and -fno-inline-atomics
 * ==========================================
 * By default GCC keeps aligned loads and stores up to a word inline, and an
 * inline store takes no lock. One from another core can land between a
 * read-modify-write's load and its store and be overwritten, lost. So every
 * ARMv6-M build compiles with -fno-inline-atomics (the architecture's root
 * toolchain, arm-none-eabi-debug-m0.toml), and GCC calls __atomic_load_N and
 * __atomic_store_N as well, which this file supplies: a store under the lock,
 * a load bare. One core needs none of it and compiles the same way, so the
 * architecture has one code generation and QEMU's M0 runs the calls.
 * test_armv6m_atomics_out_of_line fails a build without the flag. A
 * downstream ARMv6-M toolchain must carry it, and a prebuilt object compiled
 * without it keeps its inline stores.
 *
 * One store stays inline even with the flag: std::atomic_flag::clear (GCC
 * 16). So a read-modify-write stores only when it changes the value. Then one
 * that changes nothing can always be ordered before the store it raced. The
 * case that needs it is cyros's own spinlock, released by clear while the
 * other core retries its test-and-set: one that always stored 1 put the lock
 * back, held by nobody, 1,820 times in 40,000 rounds on the RP2040, and one
 * that stores only over a 0 never did (arm-port-notes.md 19h).
 *
 *
 * Ordering
 * ========
 * On one core: a single in-order core with no write buffer that reorders
 * normal memory needs no fence for these to be sequentially consistent with
 * each other and with atomic_flag's inline clear. What it needs is that the
 * COMPILER keeps memory accesses on the right side of each, which the
 * "memory" clobbers on the PRIMASK accessors and the call itself provide.
 * On more than one, the lock's acquire and release are fences, which makes
 * each read-modify-write a full one, as GCC's inline seq_cst accesses expect.
 *
 * The names and signatures are GCC's (its sync-builtins.def). A mismatch is a
 * -Wbuiltin-declaration-mismatch, so -Werror holds them to it.
 */

#include <cyros/port/port_core.h>
#include <cyros/port/port_traits.h>

#include "cortex_m.hpp"

#include <cstdint>

namespace cortex_m = cyros::port::cortex_m;

namespace
{

constexpr bool multicore = CYROS_PORT_CORE_COUNT > 1;

/* Masked, and on more than one core under the target's lock as well. A pair
 * of calls and a plain token rather than a guard object: GCC 16 zero-fills a
 * C++26 stack object before its constructor runs, which put the token in a
 * stack slot and cost the single core 1 us a ping-pong round trip
 * (arm-port-notes.md 19h). */
inline std::uint32_t enter() noexcept
{
   std::uint32_t const primask = cortex_m::get_primask();
   cortex_m::disable_irq();
   if constexpr (multicore) {
      cortex_m::atomic_lock();
   }
   return primask;
}

inline void leave(std::uint32_t primask) noexcept
{
   if constexpr (multicore) {
      cortex_m::atomic_unlock();
   }
   cortex_m::set_primask(primask);
}

template <typename T, typename Op>
T read_modify_write(volatile void* address, Op op) noexcept
{
   auto* const object = static_cast<volatile T*>(address);
   std::uint32_t const primask = enter();
   T const old = *object;
   T const updated = op(old);
   /* No store when nothing changes, for atomic_flag::clear (the header). */
   if (updated != old) {
      *object = updated;
   }
   leave(primask);
   return old;
}

template <typename T>
bool compare_exchange(volatile void* address, void* expected, T desired) noexcept
{
   auto* const object = static_cast<volatile T*>(address);
   auto* const want = static_cast<T*>(expected);
   std::uint32_t const primask = enter();
   T const current = *object;
   bool const equal = current == *want;
   /* Likewise. */
   if (equal && desired != current) {
      *object = desired;
   }
   leave(primask);
   if (!equal) {
      *want = current;
   }
   return equal;
}

/* Fenced on more than one core, as GCC's inline seq_cst load is. Fencing only
 * what the memory order asks for changed no figure measured
 * (arm-port-notes.md 19h). */
template <typename T>
T load(volatile void const* address) noexcept
{
   if constexpr (multicore) {
      cortex_m::dmb();
   }
   T const value = *static_cast<volatile T const*>(address);
   if constexpr (multicore) {
      cortex_m::dmb();
   }
   return value;
}

template <typename T>
void store(volatile void* address, T value) noexcept
{
   std::uint32_t const primask = enter();
   *static_cast<volatile T*>(address) = value;
   leave(primask);
}

} // namespace

/* One set per width GCC calls out for: 1, 2 and 4 bytes. The 8-byte forms are
 * deliberately absent, so a 64-bit atomic stays a link error here as on every
 * other target (CLAUDE_cyros-bare-metal.md 1, trap 3). The memory-order
 * arguments are ignored: every one of these is sequentially consistent. */
#define CYROS_ARMV6M_ATOMICS(N, T)                                                         \
   extern "C" T __atomic_load_##N(volatile void const* p, int)                             \
   {                                                                                       \
      return load<T>(p);                                                                   \
   }                                                                                       \
   extern "C" void __atomic_store_##N(volatile void* p, T v, int)                          \
   {                                                                                       \
      store<T>(p, v);                                                                      \
   }                                                                                       \
   extern "C" T __atomic_exchange_##N(volatile void* p, T v, int)                          \
   {                                                                                       \
      return read_modify_write<T>(p, [v](T) { return v; });                                \
   }                                                                                       \
   extern "C" bool __atomic_compare_exchange_##N(volatile void* p, void* expected,         \
                                                 T desired, bool, int, int)                \
   {                                                                                       \
      return compare_exchange<T>(p, expected, desired);                                    \
   }                                                                                       \
   extern "C" T __atomic_fetch_add_##N(volatile void* p, T v, int)                         \
   {                                                                                       \
      return read_modify_write<T>(p, [v](T old) { return static_cast<T>(old + v); });      \
   }                                                                                       \
   extern "C" T __atomic_fetch_sub_##N(volatile void* p, T v, int)                         \
   {                                                                                       \
      return read_modify_write<T>(p, [v](T old) { return static_cast<T>(old - v); });      \
   }                                                                                       \
   extern "C" T __atomic_fetch_and_##N(volatile void* p, T v, int)                         \
   {                                                                                       \
      return read_modify_write<T>(p, [v](T old) { return static_cast<T>(old & v); });      \
   }                                                                                       \
   extern "C" T __atomic_fetch_or_##N(volatile void* p, T v, int)                          \
   {                                                                                       \
      return read_modify_write<T>(p, [v](T old) { return static_cast<T>(old | v); });      \
   }                                                                                       \
   extern "C" T __atomic_fetch_xor_##N(volatile void* p, T v, int)                         \
   {                                                                                       \
      return read_modify_write<T>(p, [v](T old) { return static_cast<T>(old ^ v); });      \
   }                                                                                       \
   extern "C" T __atomic_fetch_nand_##N(volatile void* p, T v, int)                        \
   {                                                                                       \
      return read_modify_write<T>(p, [v](T old) { return static_cast<T>(~(old & v)); });   \
   }

/* GCC's own types for the three widths, which are not the <cstdint> names
 * here: arm-none-eabi's std::uint32_t is unsigned long, and the builtins take
 * unsigned int. */
static_assert(sizeof(unsigned char) == 1 && sizeof(unsigned short) == 2
              && sizeof(unsigned int) == 4);

CYROS_ARMV6M_ATOMICS(1, unsigned char)
CYROS_ARMV6M_ATOMICS(2, unsigned short)
CYROS_ARMV6M_ATOMICS(4, unsigned int)

#undef CYROS_ARMV6M_ATOMICS

/* std::atomic_flag's test_and_set. Byte-wide, as GCC lays the flag out, and
 * set to 1 only over a 0, which keeps a spinlock's release, an inline clear,
 * from being overwritten (the header). */
extern "C" bool __atomic_test_and_set(volatile void* p, int)
{
   auto const set = [](unsigned char old) { return old != 0u ? old : static_cast<unsigned char>(1); };
   return read_modify_write<unsigned char>(p, set) != 0u;
}
