/**
 * @file port_atomic_armv6m.cpp
 * @brief The atomic read-modify-writes ARMv6-M has no instructions for.
 *
 * ARMv6-M has no LDREX/STREX. GCC still compiles every std::atomic, keeping
 * aligned loads and stores up to a word inline, since those are single-copy
 * atomic, and turning each read-modify-write into a call: __atomic_fetch_add_4,
 * __atomic_compare_exchange_1 and so on. Nothing on the target supplies them,
 * so a link with the kernel fails until something does, and that is this
 * file. One core, so masking interrupts around the plain read and write is
 * the whole of atomicity (cross-core-validation.md 1b says why a second core
 * changes that: PRIMASK on one core does nothing about the other).
 *
 * The static_assert below holds a multicore ARMv6-M target, the RP2040, to
 * supplying its own over a hardware lock instead.
 *
 * Ordering. A single in-order core with no write buffer that reorders normal
 * memory needs no fence for these to be sequentially consistent with each
 * other and with the inline loads and stores. What it needs is that the
 * COMPILER keeps memory accesses on the right side of each, which the
 * "memory" clobbers on the PRIMASK accessors and the call itself provide.
 *
 * The names and signatures are GCC's (its sync-builtins.def). A mismatch is a
 * -Wbuiltin-declaration-mismatch, so -Werror holds them to it.
 */

#include <cyros/port/port_core.h>
#include <cyros/port/port_traits.h>

#include "cortex_m.hpp"

#include <cstdint>

static_assert(CYROS_PORT_CORE_COUNT == 1,
              "PRIMASK makes these atomic on one core only: a multicore ARMv6-M target "
              "supplies its own over a hardware lock");

namespace cortex_m = cyros::port::cortex_m;

namespace
{

template <typename T, typename Op>
T read_modify_write(volatile void* address, Op op) noexcept
{
   auto* const object = static_cast<volatile T*>(address);
   std::uint32_t const primask = cortex_m::get_primask();
   cortex_m::disable_irq();
   T const old = *object;
   *object = op(old);
   cortex_m::set_primask(primask);
   return old;
}

template <typename T>
bool compare_exchange(volatile void* address, void* expected, T desired) noexcept
{
   auto* const object = static_cast<volatile T*>(address);
   auto* const want = static_cast<T*>(expected);
   std::uint32_t const primask = cortex_m::get_primask();
   cortex_m::disable_irq();
   T const current = *object;
   bool const equal = current == *want;
   if (equal) {
      *object = desired;
   }
   cortex_m::set_primask(primask);
   if (!equal) {
      *want = current;
   }
   return equal;
}

} // namespace

/* One set per width GCC calls out for: 1, 2 and 4 bytes. The 8-byte forms are
 * deliberately absent, so a 64-bit atomic stays a link error here as on every
 * other target (CLAUDE_cyros.md 6a, trap 3). The memory-order arguments are
 * ignored: every one of these is sequentially consistent. */
#define CYROS_ARMV6M_ATOMICS(N, T)                                                         \
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

/* std::atomic_flag's test_and_set. Byte-wide, as GCC lays the flag out. */
extern "C" bool __atomic_test_and_set(volatile void* p, int)
{
   auto const set = [](unsigned char) { return static_cast<unsigned char>(1); };
   return read_modify_write<unsigned char>(p, set) != 0u;
}
