#include <cyros/kernel/spinlock.hpp>

namespace cyros
{

/* Every acquire path follows the same two rules, which is why try_lock lives
 * here next to lock() rather than inline in the header. It used to be inline,
 * took the flag WITHOUT entering a critical section, and still paired with this
 * unlock(), which always exits one, so a try_lock/unlock pair unbalanced the
 * core's interrupt depth (tests/unit/kernel/test_spinlock).
 *
 * 1. Interrupt-masking grade, deliberately. A preemption-only grade would let an
 *    ISR interrupt a holder on its own core, and an ISR wake path that then took
 *    this lock would spin against its own interrupted thread forever. Masking
 *    first also means contention spins with interrupts masked, which is the
 *    conventional trade (holders release in bounded tiny time by contract) and
 *    what keeps the acquire race ISR-free.
 * 2. Only the HOLDER writes `token`, and only once the flag is its own. A
 *    contender writing it while spinning would overwrite the holder's saved
 *    interrupt state, and the holder's unlock would then restore the
 *    contender's. The linux ports ignore the token, so nothing here can observe
 *    that. On a port whose token IS the saved state (PRIMASK or BASEPRI on ARM)
 *    it would restore the wrong mask. */
void spinlock::lock()
{
   auto const saved = this_core::enter_critical();
   while (flag.test_and_set(std::memory_order_acquire)) {
      this_core::cpu_relax();
   }
   token = saved;
}

bool spinlock::try_lock()
{
   auto const saved = this_core::enter_critical();
   if (flag.test_and_set(std::memory_order_acquire)) {
      this_core::exit_critical(saved); // leave the caller exactly as it was
      return false;
   }
   token = saved;
   return true;
}

void spinlock::unlock()
{
   flag.clear(std::memory_order_release);
   this_core::exit_critical(token);
}

} // namespace cyros
