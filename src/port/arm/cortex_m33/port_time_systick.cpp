/**
 * @file port_time_systick.cpp
 * @brief SysTick time source for the Cortex-M33 port. Periodic and tickless.
 *
 *
 * THE CLOCK, AND IT IS MEASURED NOT GUESSED
 * =========================================
 * `cyros_port_systick_clock_hz()` is supplied by the BOARD and has no default,
 * for the reasons at its declaration below. The QEMU bench answers 20 MHz,
 * which is what mps2-an505 drives SysTick at, measured against the semihosting
 * SYS_ELAPSED reference (a 1 GHz nanosecond counter, per SYS_TICKFREQ):
 * 20,000,051 Hz with CLKSOURCE=1 and 19,999,929 Hz with CLKSOURCE=0, both
 * within 50 ppm of exactly 20 MHz.
 *
 * A guessed clock is silently wrong by whatever factor it is off, and scales
 * every duration in the system with nothing to catch it. `test_cortex_m33_systick`
 * asserts the absolute rate against that same semihosting reference.
 *
 * EVERY board states its own. There is no default to be wrong, which is the
 * point: the STM32U575 boots at 4 MHz and any real application raises it, so
 * no single compiled-in number could be right for both.
 *
 *
 * TWO MODES, AND HOW THEY DIFFER
 * ==============================
 * `cyros_port_time_setup(tick_hz)` with tick_hz > 0 is PERIODIC: SysTick
 * reloads at a fixed interval, every interrupt is a tick, and `now()` is a
 * count of those ticks. Port ticks are tick_hz.
 *
 * `setup(0)` is TICKLESS: SysTick free-runs, `now()` is a count of CLOCK
 * cycles, and an interrupt is scheduled only when a deadline needs one. Port
 * ticks are `cyros_port_systick_clock_hz`. This is the mode that saves power
 * on a real part, because an idle system takes one interrupt per counter wrap
 * rather than one per tick.
 *
 *
 * WHAT MAKES TICKLESS AWKWARD ON THIS HARDWARE
 * ============================================
 * SysTick is a 24-bit DOWN counter with auto-reload, no capture register and
 * no compare register. Three consequences shape everything below.
 *
 * 1. The counter cannot span a long deadline. At 20 MHz a full 24-bit period
 *    is only 839 ms, so a longer deadline is reached across several wraps and
 *    the high bits have to live in software (`base`).
 *
 * 2. `now()` must combine a software word with a hardware one, and they move
 *    independently. The read is therefore masked, and it re-reads VAL when
 *    COUNTFLAG says a wrap landed between the two, which is the boundary this
 *    file's earlier comment warned would be subtly wrong if done casually.
 *
 * 3. Re-arming mid-interval means changing LOAD while the counter is running.
 *    Writing VAL forces the reload, so the elapsed part of the current interval
 *    must be folded into `base` FIRST or it is lost. And if a wrap is already
 *    pending at that moment, its interrupt must be CANCELLED (ICSR.PENDSTCLR),
 *    because the fold already accounted for it and letting the ISR also add a
 *    period would double-count.
 *
 *
 * WHY THE WRAP INTERRUPT STAYS ON IN TICKLESS
 * ===========================================
 * `cyros_port_time_irq_disable()` does NOT stop the counter or its interrupt
 * here. It stops DELIVERY of the driver's callback. The wrap interrupt is
 * structural: it is what carries `base`, so switching it off would silently
 * freeze `now()` one period later. The tickless driver calls `arm()` before
 * `irq_enable()` and `irq_disable()` before `disarm()`, and both orders work
 * because the deadline is recorded independently of whether delivery is armed.
 *
 *
 * THE 64-BIT READ PROBLEM, WHICH IS REAL ON THIS TARGET
 * ====================================================
 * std::atomic<std::uint64_t>::is_always_lock_free is FALSE on ARMv8-M, because
 * the profile has no LDREXD. The Linux ports' time sources hold
 * `std::atomic<uint64_t> now` and that is fine on x86-64; copying it here would
 * silently emit a libatomic call taking an address-hashed lock, inside an ISR,
 * on the hottest path in the time layer.
 *
 * So every 64-bit value below is plain and every access is inside a masked
 * region or inside the ISR itself.
 */

#include <cyros/port/port_mcu.h>

#include "cortex_m.hpp"

#include <cstdint>

namespace cortex_m = cyros::port::cortex_m;

/**
 * @brief What is feeding SysTick RIGHT NOW, in Hz. **The board must define
 *        this. There is deliberately no default.**
 *
 * Signature, which a board must match exactly:
 *
 *     extern "C" uint32_t cyros_port_systick_clock_hz(void);
 *
 * WHY A FUNCTION RATHER THAN A CONSTANT. It describes a RUNTIME fact. The
 * STM32U575 boots from MSIS at 4 MHz and essentially every real application
 * raises that to 80 MHz or more before doing anything else, so a value fixed at
 * link time is wrong for almost everyone. A function is read when the timer is
 * programmed, which makes the natural code order ("configure the clock tree,
 * then start the kernel") correct by construction.
 *
 * WHY NO WEAK DEFAULT. A default is a value that is silently wrong whenever the
 * board forgot to say otherwise, and this port has now been bitten twice by
 * exactly that: a guessed 25 MHz against QEMU's real 20, and a 4 MHz reset
 * value that any real application invalidates within microseconds of boot.
 * Neither failed to build or run. Both silently scaled every duration in the
 * system. Without a default, a board that does not state its clock fails to
 * LINK, naming this symbol, which is the one failure mode that cannot be
 * mistaken for working software.
 *
 * WHAT IT MUST RETURN. The frequency actually reaching SysTick at the moment of
 * the call. A board may return a literal if its clock is fixed after startup
 * (see tests/unit/port/arm_bench, and tests/hardware/u575), or read its own
 * clock tree if it is not.
 *
 * WHEN IT IS READ, and therefore the contract on the caller. Only inside
 * cyros_port_time_setup, which `time::start()` calls. So:
 *
 *   - Configure the clock tree BEFORE `time::start()`.
 *   - Do not change it while the time driver is running. Nothing can detect
 *     that, and every duration silently rescales from the moment it happens.
 *   - To change it deliberately, bracket the change: `time::stop()`, retune,
 *     `time::start()`. The new rate is picked up by the second start.
 *
 * That third rule is a real limitation rather than a preference. Lifting it
 * takes a time source that does not run off the CPU clock at all.
 */
extern "C" std::uint32_t cyros_port_systick_clock_hz(void);

namespace
{

enum class timer_mode : std::uint8_t { none, periodic, tickless };

timer_mode active_mode = timer_mode::none;

/* Periodic state. `now()` counts interrupts. */
std::uint64_t tick_count = 0;
std::uint64_t configured_tick_hz = 0;

/* Tickless state. `now()` counts clock cycles, split across a software high
 * part and the hardware counter. */
constexpr std::uint64_t never = ~std::uint64_t{0};
constexpr std::uint64_t max_period = std::uint64_t{cortex_m::systick_reload_max} + 1u;

std::uint64_t base = 0;             /* cycles completed before this interval */
std::uint32_t current_reload = 0;   /* what LOAD holds right now             */
std::uint64_t armed_deadline = never;
bool delivery_enabled = false;

cyros_port_isr_handler_t isr_handler  = nullptr;
void*                    isr_argument = nullptr;


/* ---------------------------------------------------------------------------
 * Tickless helpers. Every one of these requires interrupts to be MASKED, or to
 * be running inside the SysTick ISR, which amounts to the same guarantee.
 * ------------------------------------------------------------------------ */

/**
 * @brief The current cycle count, combining `base` with the hardware counter.
 *
 * The re-read is the interesting part. COUNTFLAG is set when the counter has
 * reached zero since CTRL was last read, so if it is set here the counter has
 * ALREADY reloaded and VAL belongs to the next interval. Reading VAL first and
 * then testing COUNTFLAG means a wrap landing between the two is caught, and
 * the re-read picks up a VAL that is consistent with the period being added.
 */
std::uint64_t now_tickless_locked() noexcept
{
   std::uint32_t val = cortex_m::reg(cortex_m::systick_val);

   if ((cortex_m::reg(cortex_m::systick_ctrl) & cortex_m::systick_ctrl_countflag) != 0u) {
      /* Wrapped. VAL above may predate the reload, so take it again. */
      val = cortex_m::reg(cortex_m::systick_val);
      return base + (std::uint64_t{current_reload} + 1u) + (current_reload - val);
   }

   if (val == 0u) {
      /* ZERO WITH COUNTFLAG CLEAR MEANS "NOT RELOADED YET", NOT "FULLY
       * ELAPSED", and the difference is a whole interval.
       *
       * The counter reloads from LOAD one timer clock AFTER a write to
       * SYST_CVR, and reads back as zero inside that window. Feeding that into
       * the elapsed formula below gives `current_reload - 0`, i.e. a complete
       * interval that has not happened.
       *
       * The two cases are distinguishable because a counter that genuinely
       * reaches zero sets COUNTFLAG and reloads, and the branch above has
       * already taken that path. So zero here can only be the post-write
       * window, where no time has elapsed since `base` was set.
       *
       * Getting this wrong reports a deadline one full 24-bit period late: a
       * callback reading VAL==0 with CTRL==0x7 immediately after the ISR
       * resized the interval makes now() jump 16.7 million cycles. */
      return base;
   }

   return base + (current_reload - val);
}

/**
 * @brief The reload value wanted for an interval starting at @p from.
 *
 * Capped at a full 24-bit period, so a distant deadline is reached across
 * several wraps. A deadline already in the past gets the shortest possible
 * interval rather than being delivered from here, which keeps callback
 * delivery in exactly one place: the ISR.
 */
std::uint32_t desired_reload(std::uint64_t from) noexcept
{
   std::uint64_t interval = max_period;

   if (armed_deadline != never) {
      interval = (armed_deadline > from) ? (armed_deadline - from) : 1u;
      if (interval > max_period) {
         interval = max_period;
      }
   }

   return static_cast<std::uint32_t>(interval - 1u);
}

/**
 * @brief Restart the counter at @p from with a freshly sized interval.
 *
 * THE INVARIANT this whole file rests on: `base` is the absolute cycle count
 * at the moment the counter last (re)loaded, so `now()` is always
 * `base + (current_reload - VAL)`.
 *
 * Writing VAL reloads the counter and DISCARDS whatever it had counted, so the
 * caller must have folded that into `base` first. Every caller here passes a
 * `from` obtained from `now_tickless_locked()` for exactly that reason.
 */
void restart_interval_locked(std::uint64_t from) noexcept
{
   current_reload = desired_reload(from);
   cortex_m::reg(cortex_m::systick_load) = current_reload;
   /* Any write to VAL reloads the counter from LOAD and clears COUNTFLAG. */
   cortex_m::reg(cortex_m::systick_val) = 0u;
}

/**
 * @brief Fold the running interval into `base` and start a new one from now.
 *
 * The PENDSTCLR is load-bearing and is the subtlety this whole file is built
 * around. `now_tickless_locked()` accounts for a wrap that has already
 * happened, and the SysTick exception for that wrap is still pending because
 * interrupts are masked. Letting it run afterwards would add the period a
 * SECOND time. Cancelling it is correct precisely because its effect has
 * already been applied.
 */
void retime_locked() noexcept
{
   base = now_tickless_locked();
   cortex_m::reg(cortex_m::scb_icsr) = cortex_m::icsr_pendstclr;
   restart_interval_locked(base);
}

} // namespace


/* ============================================================================
 * Time Driver Port Contract
 * ========================================================================= */

void cyros_port_time_setup(std::uint32_t tick_hz)
{
   cyros_mask_token_t const token = cyros_port_irq_save();

   cortex_m::reg(cortex_m::systick_ctrl) = 0u;   /* stop before reprogramming */
   cortex_m::reg(cortex_m::scb_icsr) = cortex_m::icsr_pendstclr;

   tick_count     = 0;
   base           = 0;
   armed_deadline = never;
   delivery_enabled = false;

   std::uint32_t const clock_hz = cyros_port_systick_clock_hz();

   /* A board that returns nonsense is caught here rather than producing a tick
    * rate that is merely wrong. Zero would divide by zero below; the upper
    * bound is a sanity rail, no Cortex-M part runs SysTick anywhere near it. */
   CYROS_ASSERT_OP(clock_hz, >, 0u);
   CYROS_ASSERT_OP(clock_hz, <=, 1'000'000'000u);

   if (tick_hz == 0u) {
      active_mode = timer_mode::tickless;
      /* In tickless a port tick IS a counter cycle, so the tick rate is the
       * clock rate. Captured here so freq_hz() reports what the timer was
       * actually programmed against rather than re-reading a board function
       * whose answer may since have changed. */
      configured_tick_hz = clock_hz;

      /* The counter and its wrap interrupt run from here, independently of
       * delivery. See the file comment: the wrap interrupt carries `base`. */
      restart_interval_locked(0);
      cortex_m::reg(cortex_m::systick_ctrl) =
         cortex_m::systick_ctrl_clksource
         | cortex_m::systick_ctrl_tickint
         | cortex_m::systick_ctrl_enable;
   }
   else {
      active_mode = timer_mode::periodic;

      std::uint32_t const reload = (clock_hz / tick_hz) - 1u;

      /* SysTick's reload field is 24 bits. Without this check a tick_hz too low
       * for the clock truncates and produces a tick rate that is wrong by a
       * factor of anything, reported as if it were correct. */
      CYROS_ASSERT_OP(reload, <=, cortex_m::systick_reload_max);
      CYROS_ASSERT_OP(reload, >, 0u);

      configured_tick_hz = tick_hz;
      current_reload = reload;

      cortex_m::reg(cortex_m::systick_load) = reload;
      cortex_m::reg(cortex_m::systick_val)  = 0u;

      /* Counting, but the interrupt stays off until irq_enable. The driver
       * owns that decision, and in periodic mode nothing depends on the
       * interrupt for correctness of now(). */
      cortex_m::reg(cortex_m::systick_ctrl) =
         cortex_m::systick_ctrl_clksource | cortex_m::systick_ctrl_enable;
   }

   cyros_port_irq_restore(token);
}

std::uint64_t cyros_port_time_now(void)
{
   /* Masked because the value is two words in periodic mode, and a software
    * word plus a hardware one in tickless. */
   cyros_mask_token_t const token = cyros_port_irq_save();

   std::uint64_t const value = (active_mode == timer_mode::tickless)
                             ? now_tickless_locked()
                             : tick_count;

   cyros_port_irq_restore(token);
   return value;
}

std::uint64_t cyros_port_time_freq_hz(void)
{
   /* Periodic: a port tick IS a SysTick interrupt, so the rate is the
    * configured tick rate. Tickless: a port tick is a counter CYCLE, and
    * setup() captured the clock rate into the same field. Either way this
    * reports what the hardware was actually programmed against. */
   return configured_tick_hz;
}

void cyros_port_time_reset(std::uint64_t time)
{
   cyros_mask_token_t const token = cyros_port_irq_save();

   if (active_mode == timer_mode::tickless) {
      base = time;
      cortex_m::reg(cortex_m::scb_icsr) = cortex_m::icsr_pendstclr;
      restart_interval_locked(base);
   }
   else {
      tick_count = time;
   }

   cyros_port_irq_restore(token);
}

void cyros_port_time_register_isr_handler(cyros_port_isr_handler_t handler, void* arg)
{
   cyros_mask_token_t const token = cyros_port_irq_save();
   isr_handler  = handler;
   isr_argument = arg;
   cyros_port_irq_restore(token);
}

void cyros_port_time_irq_enable(void)
{
   cyros_mask_token_t const token = cyros_port_irq_save();

   delivery_enabled = true;
   if (active_mode == timer_mode::periodic) {
      cortex_m::reg(cortex_m::systick_ctrl) =
         cortex_m::reg(cortex_m::systick_ctrl) | cortex_m::systick_ctrl_tickint;
   }

   cyros_port_irq_restore(token);
}

void cyros_port_time_irq_disable(void)
{
   cyros_mask_token_t const token = cyros_port_irq_save();

   delivery_enabled = false;
   /* Tickless deliberately leaves TICKINT alone. The wrap interrupt is what
    * keeps now() moving, and stopping it would freeze the clock one period
    * later rather than immediately, which is a horrible way to find out. */
   if (active_mode == timer_mode::periodic) {
      cortex_m::reg(cortex_m::systick_ctrl) =
         cortex_m::reg(cortex_m::systick_ctrl) & ~cortex_m::systick_ctrl_tickint;
   }

   cyros_port_irq_restore(token);
}

void cyros_port_time_arm(std::uint64_t deadline)
{
   CYROS_ASSERT(active_mode == timer_mode::tickless);

   /* Must be safe with interrupts already disabled, per port_mcu.h. The
    * save/restore token makes that true rather than assumed. */
   cyros_mask_token_t const token = cyros_port_irq_save();

   /* "the port must ensure the earliest deadline is honored". */
   if (deadline < armed_deadline) {
      armed_deadline = deadline;
   }

   /* Only disturb the hardware if the interval currently running would END
    * AFTER the deadline. If it already ends at or before it, the wrap will
    * arrive in time and the ISR will size the next one, so there is nothing to
    * do here.
    *
    * This is not just an optimisation. Every restart writes SYST_CVR, and the
    * cycles between reading the counter and that write are DISCARDED, because
    * the write reloads from LOAD. Restarting on every arm therefore leaks a
    * little time each call, and it compounds: on a 200-iteration arm/cancel
    * loop, without this check, the clock runs 16x SLOW. */
   std::uint64_t const interval_end = base + std::uint64_t{current_reload} + 1u;
   if (armed_deadline < interval_end) {
      retime_locked();
   }

   cyros_port_irq_restore(token);
}

void cyros_port_time_disarm(void)
{
   CYROS_ASSERT(active_mode == timer_mode::tickless);

   cyros_mask_token_t const token = cyros_port_irq_save();

   armed_deadline = never;

   /* Deliberately does NOT touch the hardware. Clearing the deadline can only
    * LENGTHEN the interval that is wanted, and the one already running is
    * therefore still correct, merely shorter than necessary. It costs one
    * surplus interrupt, at which point the ISR sizes the next interval back up
    * to a full period. Restarting here would discard cycles for nothing. */

   cyros_port_irq_restore(token);
}

void cyros_port_send_time_ipi(std::uint32_t core_id)
{
   /* Single core: the time core is always this core, so there is nothing to
    * notify. port_mcu.h explicitly permits an empty implementation. */
   CYROS_ASSERT_OP(core_id, ==, 0u);
}


/* ============================================================================
 * The interrupt
 * ========================================================================= */

/**
 * @brief SysTick ISR. Named for the vector table the application supplies.
 *
 * Runs a full AIRCR.PRIGROUP preemption group above PendSV, so it preempts
 * threads and is never delayed by a preemption-disable, while any reschedule it
 * requests lands after it returns.
 */
extern "C" void SysTick_Handler(void)
{
   /* Already at interrupt priority: SysTick cannot preempt itself, and every
    * thread-context reader masks. */
   if (active_mode == timer_mode::tickless) {
      /* Consume the wrap FIRST. Reading CTRL clears COUNTFLAG, so the now()
       * call below does not count the same wrap a second time. Assigned rather
       * than cast to void, because a cast-to-void does not actually perform
       * the volatile access and GCC rejects it outright. */
      [[maybe_unused]] std::uint32_t const wrap_consumed =
         cortex_m::reg(cortex_m::systick_ctrl);

      base += std::uint64_t{current_reload} + 1u;

      bool const reached = (armed_deadline != never) && (base >= armed_deadline);
      if (reached) {
         armed_deadline = never;
      }

      /* Resize ONLY when the interval actually has to change.
       *
       * The counter has already auto-reloaded from LOAD by the time this
       * handler runs, and it has been counting ever since. Writing VAL would
       * restart it and throw those cycles away. Doing that unconditionally is
       * exactly the bug this handler shipped with: `now()` jumped BACKWARDS
       * just past every wrap boundary, because a reading taken between the
       * reload and this handler was larger than the one taken after it.
       *
       * When a resize is genuinely needed, the elapsed part is folded through
       * now() first, which is what keeps the invariant true. */
      std::uint64_t const n = now_tickless_locked();
      if (desired_reload(n) != current_reload) {
         base = n;
         cortex_m::reg(cortex_m::scb_icsr) = cortex_m::icsr_pendstclr;
         restart_interval_locked(base);
      }

      /* Delivered last, so the callback re-enters arm() against an interval
       * that is already consistent rather than one this handler half updated. */
      if (reached && delivery_enabled && isr_handler != nullptr) {
         isr_handler(isr_argument);
      }
      return;
   }

   ++tick_count;

   if (delivery_enabled && isr_handler != nullptr) {
      isr_handler(isr_argument);
   }
}
