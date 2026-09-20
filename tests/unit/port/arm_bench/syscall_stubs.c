/**
 * @file syscall_stubs.c
 * @brief newlib's OS interface, for a target that has no OS.
 *
 * WHY THESE EXIST AT ALL. cyros never calls malloc, printf or exit. But the
 * arm-none-eabi libstdc++ is built against newlib and is compiled HOSTED (see
 * build/toolchains/arm-none-eabi-base.toml for why -ffreestanding is wrong
 * here), so a handful of its error paths reach abort(). abort() pulls in
 * _exit, _kill and _getpid, and once the linker is walking libc it wants the
 * file and heap stubs too. Measured 2026-09-20: the kernel's only route into
 * this is std::__throw_out_of_range_fmt from kernel.cpp.
 *
 * WHY THEY PANIC RATHER THAN RETURN -1. The conventional bare-metal stub
 * returns a failure code and lets the program carry on. That is the wrong
 * choice for this bench. Nothing in cyros should ever reach a heap, a file
 * descriptor or abort(), so arriving here is not a condition to be handled,
 * it is a defect to be reported. A silent -1 from _sbrk would turn "the kernel
 * allocated" into "an allocation failed somewhere", which is a much harder
 * thing to find.
 *
 * _exit is the one exception, because it has a correct meaning: end the image.
 */

#include <sys/stat.h>
#include <sys/types.h>

#define SYS_WRITE0        0x04
#define SYS_EXIT_EXTENDED 0x20
#define ADP_STOPPED_APPLICATION_EXIT 0x20026u

static void stub_write0(char const* text)
{
   register long r0 __asm__("r0") = SYS_WRITE0;
   register void const* r1 __asm__("r1") = text;
   __asm__ volatile("bkpt 0xAB" : "+r"(r0) : "r"(r1) : "memory");
}

__attribute__((noreturn)) static void stub_exit(unsigned code)
{
   volatile unsigned block[2] = { ADP_STOPPED_APPLICATION_EXIT, code };
   register long r0 __asm__("r0") = SYS_EXIT_EXTENDED;
   register volatile unsigned* r1 __asm__("r1") = block;
   __asm__ volatile("bkpt 0xAB" : "+r"(r0) : "r"(r1) : "memory");
   __builtin_unreachable();
}

__attribute__((noreturn)) static void unsupported(char const* name)
{
   stub_write0("\n*** cyros reached a libc facility it must not use: ");
   stub_write0(name);
   stub_write0(" ***\n");
   stub_exit(6u);
}

/* The one with a real meaning. newlib's abort() arrives here, and so does any
 * exit path, so this is also how a libstdc++ error path terminates the image
 * with a status the runner can see. */
__attribute__((noreturn)) void _exit(int status)
{
   stub_exit((unsigned)status);
}

/* abort() calls raise(), which needs these two. */
int _kill(int pid, int sig)
{
   (void)pid; (void)sig;
   unsupported("_kill (abort called)");
}

int _getpid(void)
{
   return 1;
}

/* The heap. cyros is heap-free by design: every allocation in the kernel is a
 * placement new into caller-owned storage. If this is ever called, something
 * introduced a real allocation. */
void* _sbrk(ptrdiff_t increment)
{
   (void)increment;
   unsupported("_sbrk (something tried to allocate)");
}

/* Entropy. newlib 4.6 routes some of its hardening through this. Nothing in
 * cyros wants randomness, so reaching it is as much a defect as the rest. */
int _getentropy(void* buffer, size_t length)
{
   (void)buffer; (void)length;
   unsupported("_getentropy");
}

/* Files. Output goes through semihosting, never through a descriptor. */
int _close(int file)                          { (void)file; unsupported("_close"); }
int _fstat(int file, struct stat* st)         { (void)file; (void)st; unsupported("_fstat"); }
int _isatty(int file)                         { (void)file; unsupported("_isatty"); }
off_t _lseek(int file, off_t ptr, int dir)    { (void)file; (void)ptr; (void)dir; unsupported("_lseek"); }
ssize_t _read(int file, void* ptr, size_t len)  { (void)file; (void)ptr; (void)len; unsupported("_read"); }
ssize_t _write(int file, void const* ptr, size_t len) { (void)file; (void)ptr; (void)len; unsupported("_write"); }

/* ---------------------------------------------------------------------------
 * Static destructors, which on a target that never exits must not run
 * ---------------------------------------------------------------------------
 *
 * A file-scope C++ object with a non-trivial destructor registers that
 * destructor with __cxa_atexit at construction time, which pulls in
 * __dso_handle and newlib's fini machinery. Any cyros application hits this
 * the moment it declares a semaphore or a mutex at file scope, because both
 * derive from waitable and so have a virtual destructor.
 *
 * The registration is made a no-op rather than supported. An embedded image
 * has no exit: main never returns, the scheduler runs forever, and a
 * destructor that "runs at exit" would either never run or run at a moment
 * when the kernel has already stopped. Dropping the registration is both
 * correct and what every embedded C++ runtime does.
 *
 * The cost is that static destructors never run. That is the right trade here
 * and it should be stated rather than discovered.
 */

void* __dso_handle = 0;

int __cxa_atexit(void (*destructor)(void*), void* argument, void* dso)
{
   (void)destructor; (void)argument; (void)dso;
   return 0;   /* accepted and discarded */
}

void __cxa_finalize(void* dso) { (void)dso; }

/* Referenced by newlib's __libc_fini_array. Nothing to finalise. */
void _init(void) {}
void _fini(void) {}
