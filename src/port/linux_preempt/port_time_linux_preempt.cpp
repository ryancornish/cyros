/**
 * @file port_time_linux_preempt.cpp
 * @brief Linux timer-driver port for the preemptive (sigctx) backend.
 *
 * Unlike the boost time port, which is a passive counter that tests pump by
 * hand, this one is a real asynchronous interrupt source. A POSIX timer fires a
 * dedicated signal at the time core, and the signal handler invokes the ISR the
 * time driver registered. That is what lets a timer preempt a running thread the
 * way real hardware does.
 *
 * Signal model
 * ------------
 * The timer uses its OWN signal (timer_signo), distinct from the reschedule
 * signal the main port uses. That separation is what makes the interrupt-versus-
 * preempt distinction real: the main port masks timer_signo under interrupt-
 * disable but leaves it unmasked under preempt-disable, so a timer interrupt
 * still fires inside a preempt-disabled region and only the reschedule it
 * requests is deferred.
 *
 * The timer handler runs with the reschedule signal masked (set in sa_mask
 * below), so a reschedule cannot nest into the middle of the ISR. Any wake the
 * ISR performs pends a reschedule that the kernel delivers when the handler
 * returns.
 *
 * The reverse direction is closed symmetrically by the main port. Its reschedule
 * interceptor is installed with sigctx's block_extra set to timer_signo, so the
 * timer stays masked for the whole interception, capture and handler alike. A
 * timer can therefore never land on the shared handler stack mid-reschedule, and
 * the two signals can never nest into each other in either order.
 *
 * Targeting
 * ---------
 * The timer is delivered via SIGEV_THREAD_ID to a specific kernel TID, which
 * is whichever thread called setup() (recorded there). The port makes no
 * assumption about which core that is and has no dependency on config: "the
 * time core" is simply defined as whoever set the timer up. A higher layer
 * that cares which core that ends up being controls it by controlling who
 * calls time::start().
 *
 * now() is backed by CLOCK_MONOTONIC, so time advances on its own. The
 * deterministic cyros_port_time_advance() hook the periodic and tickless driver
 * tests rely on is therefore unsupported here. Those suites stay on the boost
 * port.
 */

#include <cyros/port/port_time.h>

#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <pthread.h>
#include <unistd.h>
#include <sys/syscall.h>

namespace
{

/**
 * Must match timer_signo in port_linux_preempt.cpp. If these drift, interrupt-
 * disable will not actually hold off the timer ISR. SIGRTMIN is not a constant
 * expression, so this is a runtime-initialised const.
 */
const int timer_signo = SIGRTMIN;

/**
 * Must match preempt_signo in port_linux_preempt.cpp. The timer handler masks it
 * so a reschedule cannot nest into the ISR.
 */
const int reschedule_signo = SIGURG;

/** 1 tick == 1 microsecond, matching the boost port's convention. */
constexpr uint64_t tick_freq_hz = 1'000'000;

struct time_state
{
   std::atomic<cyros_port_isr_handler_t> isr{nullptr};
   std::atomic<void*>                    isr_arg{nullptr};

   // now() == base_ticks + (CLOCK_MONOTONIC now - epoch_ns) scaled to ticks.
   std::atomic<uint64_t> base_ticks{0};
   std::atomic<uint64_t> epoch_ns{0};

   std::atomic<uint64_t> armed_deadline{UINT64_MAX};

   timer_t  timer{};
   bool     timer_created{false};
   uint32_t tick_hz{0};        // 0 means tickless one-shot
   pid_t    time_core_tid{0};
};
time_state ts;

uint64_t monotonic_ns()
{
   struct timespec t;
   clock_gettime(CLOCK_MONOTONIC, &t);
   return static_cast<uint64_t>(t.tv_sec) * 1'000'000'000ull + static_cast<uint64_t>(t.tv_nsec);
}

uint64_t ns_to_ticks(uint64_t ns)
{
   return ns / (1'000'000'000ull / tick_freq_hz);
}

uint64_t ticks_to_ns(uint64_t ticks)
{
   return ticks * (1'000'000'000ull / tick_freq_hz);
}

void program_oneshot_ns(uint64_t rel_ns)
{
   if (!ts.timer_created) return;
   if (rel_ns == 0) rel_ns = 1; // 0 would disarm, so fire as soon as possible instead

   struct itimerspec its;
   its.it_value.tv_sec     = static_cast<time_t>(rel_ns / 1'000'000'000ull);
   its.it_value.tv_nsec    = static_cast<long>(rel_ns % 1'000'000'000ull);
   its.it_interval.tv_sec  = 0;
   its.it_interval.tv_nsec = 0;
   timer_settime(ts.timer, 0, &its, nullptr);
}

void interceptor_disarm()
{
   if (!ts.timer_created) return;
   struct itimerspec its;
   its.it_value.tv_sec     = 0;
   its.it_value.tv_nsec    = 0;
   its.it_interval.tv_sec  = 0;
   its.it_interval.tv_nsec = 0;
   timer_settime(ts.timer, 0, &its, nullptr);
}

void on_timer_signal(int, siginfo_t*, void*)
{
   // Runs on the time core with the reschedule signal masked, so a reschedule
   // cannot nest in here. Invoke the driver's ISR, which fires due callbacks and
   // may ready threads. Any reschedule that readying requests is pended and
   // delivered once this handler returns.
   auto handler = ts.isr.load(std::memory_order_acquire);
   if (handler) {
      handler(ts.isr_arg.load(std::memory_order_acquire));
   }
}

} // namespace


/* ============================================================================
 * Time Driver Port Contract API
 * ========================================================================= */

void cyros_port_time_setup(uint32_t tick_hz)
{
   // Whichever thread calls this becomes the time core, full stop. No port-layer
   // assumption about which core that is, and nothing to keep in sync with a
   // higher layer's config: this TID is just recorded, not validated against it.
   ts.time_core_tid = static_cast<pid_t>(syscall(SYS_gettid));
   ts.tick_hz = tick_hz;

   // The timer ISR runs with the reschedule signal masked so a switch cannot
   // nest into it. It runs on the altstack to keep off the interrupted thread's
   // stack.
   struct sigaction sa;
   memset(&sa, 0, sizeof(sa));
   sa.sa_sigaction = on_timer_signal;
   sigemptyset(&sa.sa_mask);
   sigaddset(&sa.sa_mask, reschedule_signo);
   sa.sa_flags = SA_SIGINFO | SA_RESTART | SA_ONSTACK;
   sigaction(timer_signo, &sa, nullptr);

   // Deliver only to the time core, so the ISR never runs on a thread the kernel
   // cannot place.
   struct sigevent sev;
   memset(&sev, 0, sizeof(sev));
   sev.sigev_notify = SIGEV_THREAD_ID;
   sev.sigev_signo  = timer_signo;
   // sigev_notify_thread_id is a glibc macro for _sigev_un._tid, gated on
   // _GNU_SOURCE having been visible before signal.h's first inclusion anywhere
   // in the translation unit. That's a transitive, include-order-dependent
   // condition this file cannot guarantee given upstream port/config headers, so
   // write the union member directly instead of relying on the macro existing.
   sev._sigev_un._tid = ts.time_core_tid;

   if (timer_create(CLOCK_MONOTONIC, &sev, &ts.timer) == 0) {
      ts.timer_created = true;
   }
}

uint64_t cyros_port_time_now(void)
{
   uint64_t base  = ts.base_ticks.load(std::memory_order_acquire);
   uint64_t epoch = ts.epoch_ns.load(std::memory_order_acquire);
   uint64_t cur   = monotonic_ns();
   uint64_t delta = (cur > epoch) ? (cur - epoch) : 0;
   return base + ns_to_ticks(delta);
}

uint64_t cyros_port_time_freq_hz(void)
{
   return tick_freq_hz;
}

void cyros_port_time_reset(uint64_t t)
{
   ts.epoch_ns.store(monotonic_ns(), std::memory_order_release);
   ts.base_ticks.store(t, std::memory_order_release);
   ts.armed_deadline.store(UINT64_MAX, std::memory_order_release);
}

void cyros_port_time_register_isr_handler(cyros_port_isr_handler_t h, void* arg)
{
   ts.isr_arg.store(arg, std::memory_order_relaxed);
   ts.isr.store(h, std::memory_order_release);
}

void cyros_port_time_irq_enable(void)
{
   if (!ts.timer_created) return;

   if (ts.tick_hz > 0) {
      // Periodic tick.
      uint64_t period_ns = 1'000'000'000ull / ts.tick_hz;
      struct itimerspec its;
      its.it_value.tv_sec     = static_cast<time_t>(period_ns / 1'000'000'000ull);
      its.it_value.tv_nsec    = static_cast<long>(period_ns % 1'000'000'000ull);
      its.it_interval         = its.it_value;
      timer_settime(ts.timer, 0, &its, nullptr);
   } else {
      // Tickless: nothing to start until a deadline is armed.
      uint64_t deadline = ts.armed_deadline.load(std::memory_order_acquire);
      if (deadline != UINT64_MAX) {
         uint64_t now = cyros_port_time_now();
         uint64_t rel = (deadline > now) ? (deadline - now) : 0;
         program_oneshot_ns(ticks_to_ns(rel));
      }
   }
}

void cyros_port_time_irq_disable(void)
{
   interceptor_disarm();
}

void cyros_port_time_arm(uint64_t deadline)
{
   // Keep the earliest deadline.
   uint64_t cur = ts.armed_deadline.load(std::memory_order_relaxed);
   while (deadline < cur &&
          !ts.armed_deadline.compare_exchange_weak(cur, deadline,
                                                   std::memory_order_release,
                                                   std::memory_order_relaxed))
   {}

   uint64_t now = cyros_port_time_now();
   uint64_t rel = (deadline > now) ? (deadline - now) : 0;
   program_oneshot_ns(ticks_to_ns(rel));
}

void cyros_port_time_disarm(void)
{
   ts.armed_deadline.store(UINT64_MAX, std::memory_order_release);
   interceptor_disarm();
}

// Linux-only test hook. The preempt port is backed by a real monotonic clock,
// so deterministic advance is unsupported. Pumped, deterministic time-driver
// tests should use the boost port.
extern void cyros_port_time_advance(uint64_t delta)
{
   (void)delta;
}

void cyros_port_send_time_ipi(uint32_t /*core_id*/)
{
   // Nudge the time core to do time work by raising its timer signal directly.
   if (ts.time_core_tid != 0) {
      syscall(SYS_tgkill, getpid(), ts.time_core_tid, timer_signo);
   }
}
