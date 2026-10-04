/**
 * @file main.cpp
 * @brief Seven cyros threads driving the Orbit BoosterPack, to show the kernel
 *        doing its job where a person can see it.
 *
 * | thread   | prio | what it does                                       |
 * |----------|------|----------------------------------------------------|
 * | input    | 1    | woken by the GPIO interrupts, debounces, updates   |
 * | sensor   | 2    | reads the potentiometer every 20 ms                |
 * | hog_high | 3    | burns the CPU while SW1 is up and SW2 is up        |
 * | display  | 4    | draws a frame every 50 ms, and at once when asked  |
 * | hog_low  | 5    | burns the CPU while SW1 is up and SW2 is down      |
 * | leds     | 6    | chases LD1 to LD4, faster as the knob turns up     |
 * | logger   | 7    | prints what the others send it to the console      |
 *
 * Lower is more urgent, and the idle thread is 15. What each part shows:
 *
 * - The buttons and switches interrupt, the interrupt sets an event flag, and
 *   the input thread wakes on it: a press is counted on the display and lights
 *   the LaunchPad's LED (BTN1 red, BTN2 blue) for as long as it is held.
 * - The green LED blinks from a kernel timer, whose callback runs in the
 *   SysTick interrupt, so it keeps time whatever the threads are doing.
 * - "cpu" is the share of the last second the core spent awake. The idle
 *   thread sleeps in WFI and the tickless clock only wakes it for a deadline,
 *   so with nothing to do it reads near zero.
 * - SW1 up starts a thread that never blocks. With SW2 down it sits below the
 *   display: the display stays live, cpu reads 100 and the LED chase stops,
 *   because the leds thread is the only one below it. With SW2 up it sits
 *   above the display, which freezes too, after the input thread has waited
 *   for one last frame showing "hog HIGH". The buttons still light the LED
 *   because the input thread outranks both. Presses made during the
 *   freeze are on the display once SW1 goes down.
 * - The LaunchPad's SW1 switches to a page of loops per second per thread,
 *   where the starved threads read 0. Its SW2 clears the press counts.
 *
 * Shared state is behind a cyros mutex, which the input thread (1) can want
 * while the display (4) holds it with a hog (3) runnable between them: the
 * priority inversion that inheritance resolves.
 */

#include "orbit.hpp"
#include "screen.hpp"
#include "tm4c123_regs.hpp"

#include <cyros/ch/channel.hpp>
#include <cyros/chrono/chrono.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/event_flags.hpp>
#include <cyros/sync/mutex.hpp>
#include <cyros/time/time.hpp>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

// The board's (board_clock.c and console.c).
extern "C" std::uint32_t cyros_port_systick_clock_hz(void);
extern "C" void cyros_bench_write(char const* text);

using namespace cyros;
using screen::hog_mode;
using screen::worker;

namespace
{

constexpr std::size_t thread_count = 7;
constexpr std::size_t stack_size   = thread::min_stack_size + 1024;
alignas(CYROS_PORT_STACK_ALIGN) std::array<std::array<std::byte, stack_size>, thread_count> stacks{};

// Set from the GPIO interrupts: some input changed.
event_flags input_edges{0};
constexpr std::uint32_t edge = 1u;

// Which hog may run, written by the input thread from SW1 and SW2.
event_flags hog_enable{0};
constexpr std::uint32_t hog_low_bit  = 1u;
constexpr std::uint32_t hog_high_bit = 2u;

// A frame on demand. The input thread sets frame_wanted, the display draws at
// once rather than at its next period, and sets frame_shown once the frame is
// on the panel.
event_flags display_sync{0};
constexpr std::uint32_t frame_wanted = 1u;
constexpr std::uint32_t frame_shown  = 2u;

// What the display shows that other threads produce.
struct shared_state
{
   std::uint32_t pot{0};
   std::uint32_t btn1{0};
   std::uint32_t btn2{0};
   hog_mode      hog{hog_mode::off};
   std::uint32_t page{0};
} state;
mutex        state_lock;

class guard
{
public:
   explicit guard(mutex& m) noexcept : mtx(m) { m.lock(); }
   ~guard() { mtx.unlock(); }
   guard(guard const&)            = delete;
   guard& operator=(guard const&) = delete;

private:
   mutex& mtx;
};

/* Each thread counts its own loops, and the display turns them into rates. */
std::array<std::atomic<std::uint32_t>, static_cast<std::size_t>(worker::count)> loops{};

void count(worker w) noexcept
{
   loops[static_cast<std::size_t>(w)].fetch_add(1u, std::memory_order_relaxed);
}

/* What the logger prints. Sent with try_send, so a starved logger costs
 * dropped lines, never a blocked sender. */
struct log_entry
{
   enum class kind : std::uint8_t { none, boot, input, status };
   kind             what{kind::none};
   std::uint64_t    ms{0};
   std::uint32_t    inputs{0};
   std::uint32_t    changed{0};
   screen::snapshot snap{};
};
ch::channel<log_entry, 8>  log_channel;
std::atomic<std::uint32_t> log_dropped{0};

void log(log_entry const& entry) noexcept
{
   if (!log_channel.try_send(entry)) {
      log_dropped.fetch_add(1u, std::memory_order_relaxed);
   }
}

void sleep_ms(std::uint32_t ms)
{
   this_thread::sleep_for(time::from_milliseconds(ms));
}

std::uint64_t now_ms() noexcept
{
   return time::to_milliseconds(time::duration{time::now().value});
}

/* ---------------------------------------------------------------------------
 * input, priority 1
 * ------------------------------------------------------------------------ */

void on_edge()
{
   (void)input_edges.set(edge);
}

void heartbeat(void*)
{
   static bool lit = false;
   lit = !lit;
   orbit::set_green(lit);
}

hog_mode hog_from(std::uint32_t inputs) noexcept
{
   if ((inputs & orbit::sw1) == 0u) { return hog_mode::off; }
   return (inputs & orbit::sw2) != 0u ? hog_mode::high : hog_mode::low;
}

void apply(std::uint32_t inputs, std::uint32_t changed)
{
   orbit::set_red((inputs & (orbit::btn1 | orbit::lp_sw1)) != 0u);
   orbit::set_blue((inputs & (orbit::btn2 | orbit::lp_sw2)) != 0u);

   std::uint32_t const pressed = inputs & changed;
   hog_mode const hog = hog_from(inputs);
   {
      guard const g{state_lock};
      if ((pressed & orbit::btn1) != 0u)   { ++state.btn1; }
      if ((pressed & orbit::btn2) != 0u)   { ++state.btn2; }
      if ((pressed & orbit::lp_sw1) != 0u) { state.page ^= 1u; }
      if ((pressed & orbit::lp_sw2) != 0u) { state.btn1 = 0u; state.btn2 = 0u; }
      state.hog = hog;
   }

   // Stop before start, so the two hogs never both hold the core.
   bool const entering_high = hog == hog_mode::high && (hog_enable.peek() & hog_high_bit) == 0u;
   if (hog != hog_mode::low)  { (void)hog_enable.clear(hog_low_bit); }
   if (hog != hog_mode::high) { (void)hog_enable.clear(hog_high_bit); }

   // Every change is drawn at once rather than at the next period. Before the
   // hog that outranks the display starts, wait until that frame is on the
   // panel, or the display freezes still showing what it showed before. The
   // timeout only covers a display not yet powered on, at boot.
   (void)display_sync.clear(frame_shown);
   (void)display_sync.set(frame_wanted);
   if (entering_high) {
      (void)display_sync.try_wait_for(frame_shown, time::from_milliseconds(250),
                                      sync::flags_match::any, sync::flags_exit::consume);
   }

   if (hog == hog_mode::low)  { (void)hog_enable.set(hog_low_bit); }
   if (hog == hog_mode::high) { (void)hog_enable.set(hog_high_bit); }
}

void input_main()
{
   // The first thread to run, being the most urgent, so the clock starts
   // before anything sleeps. In tickless mode a tick is a core cycle.
   time::initialise(cyros_port_systick_clock_hz());
   time::start();
   (void)time::schedule_recurring(time::from_milliseconds(500), heartbeat, nullptr);

   // At SysTick's priority, so a GPIO interrupt never preempts the tickless
   // driver halfway through updating its clock.
   orbit::enable_input_interrupts(tm4c::reg8(tm4c::shpr_systick), on_edge);

   std::uint32_t settled = orbit::read_inputs();
   apply(settled, 0u);
   log({.what = log_entry::kind::boot, .ms = now_ms(), .inputs = settled});

   while (true) {
      (void)input_edges.wait(edge, sync::flags_match::any, sync::flags_exit::consume);

      // Debounce: let the contacts settle, then forget the edges they made
      // meanwhile and read the level. An edge after the clear sets the flag
      // again and costs one more pass, so none is lost.
      sleep_ms(20u);
      (void)input_edges.clear(edge);
      std::uint32_t const inputs = orbit::read_inputs();
      count(worker::input);

      if (inputs != settled) {
         std::uint32_t const changed = inputs ^ settled;
         settled = inputs;
         apply(inputs, changed);
         log({.what = log_entry::kind::input, .ms = now_ms(), .inputs = inputs, .changed = changed});
      }
   }
}

/* ---------------------------------------------------------------------------
 * sensor, priority 2
 * ------------------------------------------------------------------------ */

void sensor_main()
{
   // A first-order filter in 1/16ths, a quarter of each new sample: the knob
   // still answers within about 100 ms, and the last digit stops flickering.
   std::uint32_t filtered = orbit::read_pot() << 4;
   time::time_point next = time::now();
   while (true) {
      next = next + time::from_milliseconds(20);
      this_thread::sleep_until(next);
      filtered = filtered - filtered / 4u + (orbit::read_pot() << 4) / 4u;
      {
         guard const g{state_lock};
         state.pot = filtered >> 4;
      }
      count(worker::sensor);
   }
}

/* ---------------------------------------------------------------------------
 * hogs, priorities 3 and 5
 * ------------------------------------------------------------------------ */

void burn() noexcept
{
   for (std::uint32_t i = 0; i < 1000u; ++i) {
      __asm__ volatile("");
   }
}

// Blocks until allowed, then never blocks until disallowed. Only a more
// urgent thread can take the core from it meanwhile, which is the point.
void hog(std::uint32_t bit, worker who)
{
   while (true) {
      (void)hog_enable.wait(bit, sync::flags_match::any, sync::flags_exit::keep);
      while ((hog_enable.peek() & bit) != 0u) {
         burn();
         count(who);
      }
   }
}

void hog_high_main() { hog(hog_high_bit, worker::hog_high); }
void hog_low_main()  { hog(hog_low_bit, worker::hog_low); }

/* ---------------------------------------------------------------------------
 * display, priority 4
 * ------------------------------------------------------------------------ */

screen::canvas canvas;

void display_main()
{
   orbit::oled_power_on(sleep_ms);

   std::uint32_t const hz = cyros_port_systick_clock_hz();
   screen::snapshot s;
   s.mhz = hz / 1'000'000u;

   // The measurement window, about a second. The awake counter stops while
   // the core sleeps in WFI and SysTick does not, so over a window the one
   // divided by the other is the share the core was awake (orbit.hpp).
   std::uint64_t window_start  = time::now().value;
   std::uint32_t window_cycles = orbit::awake_cycles();
   std::array<std::uint32_t, static_cast<std::size_t>(worker::count)> window_loops{};

   std::uint64_t next_status_s = 5u;
   time::duration const period = time::from_milliseconds(50);
   time::time_point next = time::now() + period;
   while (true) {
      // A frame every period, or at once when the input thread asks. Only a
      // period moves the schedule on, and never into the past: after a freeze,
      // one frame and back to the rate rather than a burst to catch up.
      bool const asked = display_sync.try_wait_until(frame_wanted, next, sync::flags_match::any,
                                                     sync::flags_exit::consume) != 0u;
      if (!asked) {
         next = next + period;
         if (time::time_point const now = time::now(); next <= now) { next = now + period; }
      }

      std::uint64_t const ticks  = time::now().value;
      std::uint32_t const cycles = orbit::awake_cycles();
      std::uint64_t const window = ticks - window_start;
      if (window >= hz) {
         std::uint64_t const busy = static_cast<std::uint32_t>(cycles - window_cycles);
         s.cpu_known   = window <= 0xFFFFFFFFu;  // Else the awake counter may have wrapped
         s.cpu_percent = static_cast<std::uint32_t>((busy * 100u + window / 2u) / window);
         if (s.cpu_percent > 100u) { s.cpu_percent = 100u; }
         for (std::size_t i = 0; i < window_loops.size(); ++i) {
            std::uint32_t const total = loops[i].load(std::memory_order_relaxed);
            s.loops_per_s[i] = static_cast<std::uint32_t>((std::uint64_t{total - window_loops[i]} * hz + window / 2u) / window);
            window_loops[i] = total;
         }
         window_start  = ticks;
         window_cycles = cycles;
      }

      s.uptime_s = ticks / hz;
      {
         guard const g{state_lock};
         s.pot          = state.pot;
         s.btn1_presses = state.btn1;
         s.btn2_presses = state.btn2;
         s.hog          = state.hog;
         s.page         = state.page;
      }
      screen::render(s, canvas);
      orbit::oled_write(canvas.pixels());
      if (asked) { (void)display_sync.set(frame_shown); }
      count(worker::display);

      if (s.uptime_s >= next_status_s) {
         next_status_s = s.uptime_s + 5u;
         log({.what = log_entry::kind::status, .ms = now_ms(), .snap = s});
      }
   }
}

/* ---------------------------------------------------------------------------
 * leds, priority 6
 * ------------------------------------------------------------------------ */

void leds_main()
{
   for (std::uint32_t step = 0;; ++step) {
      orbit::set_leds(1u << (step % 4u));
      std::uint32_t pot = 0;
      {
         guard const g{state_lock};
         pot = state.pot;
      }
      // 500 ms a step with the knob down, 25 ms with it up.
      sleep_ms(25u + ((4095u - pot) * 475u) / 4095u);
      count(worker::leds);
   }
}

/* ---------------------------------------------------------------------------
 * logger, priority 7
 * ------------------------------------------------------------------------ */

class line
{
public:
   line& operator<<(std::string_view s) noexcept
   {
      for (char const c : s) {
         if (length_ + 1u < text_.size()) { text_[length_++] = c; }
      }
      text_[length_] = '\0';
      return *this;
   }

   line& number(std::uint64_t value, std::size_t width = 0, char pad = ' ') noexcept
   {
      char digits[21];
      std::size_t n = 0;
      do {
         digits[n++] = static_cast<char>('0' + value % 10u);
         value /= 10u;
      } while (value != 0u);
      for (std::size_t i = n; i < width; ++i) { *this << std::string_view{&pad, 1}; }
      while (n > 0u) { *this << std::string_view{&digits[--n], 1}; }
      return *this;
   }

   [[nodiscard]] char const* c_str() const noexcept { return text_.data(); }

private:
   std::array<char, 128> text_{};
   std::size_t length_{0};
};

void stamp(line& l, std::uint64_t ms)
{
   l << "[";
   l.number(ms / 1000u, 6);
   l << ".";
   l.number(ms % 1000u, 3, '0');
   l << "] ";
}

std::string_view hog_name(hog_mode h)
{
   switch (h) {
      case hog_mode::off:  return "off";
      case hog_mode::low:  return "low";
      case hog_mode::high: return "HIGH";
   }
   return "?";
}

void describe_inputs(line& l, std::uint32_t inputs, std::uint32_t changed)
{
   struct input { std::uint32_t bit; std::string_view name; std::string_view set; std::string_view clear; };
   for (input const& i : { input{.bit=orbit::btn1,   .name="BTN1",   .set="pressed", .clear="released"},
                           input{.bit=orbit::btn2,   .name="BTN2",   .set="pressed", .clear="released"},
                           input{.bit=orbit::sw1,    .name="SW1",    .set="up",      .clear="down"},
                           input{.bit=orbit::sw2,    .name="SW2",    .set="up",      .clear="down"},
                           input{.bit=orbit::lp_sw1, .name="LP-SW1", .set="pressed", .clear="released"},
                           input{.bit=orbit::lp_sw2, .name="LP-SW2", .set="pressed", .clear="released"}, })
   {
      if ((changed & i.bit) != 0u) {
         l << i.name << " " << ((inputs & i.bit) != 0u ? i.set : i.clear) << "  ";
      }
   }
}

void logger_main()
{
   while (true) {
      std::optional<log_entry> const entry = log_channel.receive();
      if (!entry) continue;

      line l;
      stamp(l, entry->ms);
      switch (entry->what) {
         case log_entry::kind::none:
            break;
         case log_entry::kind::boot:
            l << "cyros orbit demo, ";
            l.number(cyros_port_systick_clock_hz() / 1'000'000u);
            l << " MHz, threads ";
            l.number(kernel::active_threads());
            l << ", SW1 " << ((entry->inputs & orbit::sw1) != 0u ? "up" : "down");
            l << ", SW2 " << ((entry->inputs & orbit::sw2) != 0u ? "up" : "down");
            break;
         case log_entry::kind::input:
            describe_inputs(l, entry->inputs, entry->changed);
            break;
         case log_entry::kind::status: {
            screen::snapshot const& s = entry->snap;
            auto rate = [&](worker w) { return s.loops_per_s[static_cast<std::size_t>(w)]; };
            l << "pot ";
            l.number((s.pot * 100u + 2047u) / 4095u);
            l << "%  cpu ";
            if (s.cpu_known) { l.number(s.cpu_percent); } else { l << "--"; }
            l << "%  btn1 ";
            l.number(s.btn1_presses);
            l << "  btn2 ";
            l.number(s.btn2_presses);
            l << "  hog " << hog_name(s.hog);
            l << "  loops/s in ";
            l.number(rate(worker::input));
            l << " sn ";
            l.number(rate(worker::sensor));
            l << " hH ";
            l.number(rate(worker::hog_high));
            l << " ds ";
            l.number(rate(worker::display));
            l << " hL ";
            l.number(rate(worker::hog_low));
            l << " ld ";
            l.number(rate(worker::leds));
            if (std::uint32_t const dropped = log_dropped.load(std::memory_order_relaxed); dropped != 0u) {
               l << "  dropped ";
               l.number(dropped);
            }
            break;
         }
      }
      l << "\n";
      cyros_bench_write(l.c_str());
   }
}

}  // namespace

// The board's entry point (startup_tm4c123.c), after the clock and the
// console are up.
extern "C" int cyros_bench_main()
{
   orbit::init(cyros_port_systick_clock_hz());
   orbit::start_awake_counter();

   kernel::initialise();

   thread input   {input_main,    stacks[0], thread::priority(1), core0};
   thread sensor  {sensor_main,   stacks[1], thread::priority(2), core0};
   thread hog_high{hog_high_main, stacks[2], thread::priority(3), core0};
   thread display {display_main,  stacks[3], thread::priority(4), core0};
   thread hog_low {hog_low_main,  stacks[4], thread::priority(5), core0};
   thread leds    {leds_main,     stacks[5], thread::priority(6), core0};
   thread logger  {logger_main,   stacks[6], thread::priority(7), core0};

   kernel::start();

   cyros_bench_write("kernel::start() returned\n");
   return 5;
}
