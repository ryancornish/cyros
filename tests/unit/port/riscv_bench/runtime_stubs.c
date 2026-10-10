/**
 * @file runtime_stubs.c
 * @brief The C library and C++ runtime functions cyros and its tests need that
 *        the bridge toolchain (riscv-port-plan.md 2) does not have, for every
 *        RISC-V board: QEMU's virt and the RP2350's Hazard3.
 *
 * newlib and libsupc++ supply all of these on ARM. When a real RISC-V
 * toolchain arrives, the C library half goes, and the static destructor half
 * stays as ../arm_bench/syscall_stubs.c keeps it.
 *
 * The board supplies cyros_bench_write and cyros_bench_exit.
 */

#include <stddef.h>
#include <stdint.h>

void cyros_bench_write(char const* text);
__attribute__((noreturn)) void cyros_bench_exit(uint32_t code);

/* ---------------------------------------------------------------------------
 * The C library functions the bridge toolchain does not have. Byte loops
 * through a volatile pointer, so the compiler cannot turn them back into calls
 * to themselves.
 * ------------------------------------------------------------------------ */

void* memset(void* dest, int value, size_t count)
{
   unsigned char volatile* d = (unsigned char volatile*)dest;
   while (count--) { *d++ = (unsigned char)value; }
   return dest;
}

void* memcpy(void* restrict dest, void const* restrict src, size_t count)
{
   unsigned char volatile* d = (unsigned char volatile*)dest;
   unsigned char const* s = (unsigned char const*)src;
   while (count--) { *d++ = *s++; }
   return dest;
}

void* memmove(void* dest, void const* src, size_t count)
{
   unsigned char volatile* d = (unsigned char volatile*)dest;
   unsigned char const* s = (unsigned char const*)src;
   if (d < s) {
      while (count--) { *d++ = *s++; }
   } else {
      d += count;
      s += count;
      while (count--) { *--d = *--s; }
   }
   return dest;
}

int memcmp(void const* lhs, void const* rhs, size_t count)
{
   unsigned char const* a = (unsigned char const*)lhs;
   unsigned char const* b = (unsigned char const*)rhs;
   for (; count != 0u; --count, ++a, ++b) {
      if (*a != *b) { return *a < *b ? -1 : 1; }
   }
   return 0;
}

/* ---------------------------------------------------------------------------
 * The C++ runtime symbols the bridge toolchain does not have
 *
 * Static destructors are registered and dropped, exactly as
 * ../arm_bench/syscall_stubs.c does and for its reason: an image never exits,
 * so a destructor that "runs at exit" never should. __cxa_pure_virtual is
 * libsupc++'s on ARM. A real RISC-V toolchain brings both, and these
 * definitions then simply win over its archive members.
 * ------------------------------------------------------------------------ */

void* __dso_handle = 0;

int __cxa_atexit(void (*destructor)(void*), void* argument, void* dso)
{
   (void)destructor; (void)argument; (void)dso;
   return 0;   /* accepted and discarded */
}

void __cxa_finalize(void* dso) { (void)dso; }

__attribute__((noreturn)) void __cxa_pure_virtual(void)
{
   cyros_bench_write("\n*** pure virtual call ***\n");
   cyros_bench_exit(4u);
}
