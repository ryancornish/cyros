/**
 * @file orbit.cpp
 * @brief The Orbit BoosterPack's drivers. Pin table and contract in orbit.hpp.
 */

#include "orbit.hpp"
#include "tm4c123_regs.hpp"

#include <initializer_list>

using namespace tm4c;

namespace
{

constexpr std::uint32_t pin(unsigned n) { return 1u << n; }

/* Called from the GPIO interrupts. Written once, before they are enabled. */
void (*edge_callback)() = nullptr;

void ssi_put(std::uint8_t byte) noexcept
{
   while ((reg(ssi3_sr) & ssi_sr_tnf) == 0u) {}
   reg(ssi3_dr) = byte;
}

void ssi_drain() noexcept
{
   while ((reg(ssi3_sr) & ssi_sr_bsy) != 0u) {}
}

/* One transfer with chip select held low throughout. D/C may only change
 * while the bus is idle, since the panel samples it with each byte's last bit. */
template<typename Bytes>
void oled_send(bool is_data, Bytes const& bytes) noexcept
{
   ssi_drain();
   gpio_d.data(pin(7)) = is_data ? pin(7) : 0u;
   gpio_d.data(pin(1)) = 0u;
   for (std::uint8_t const byte : bytes) {
      ssi_put(byte);
   }
   ssi_drain();
   gpio_d.data(pin(1)) = pin(1);
}

void oled_command(std::initializer_list<std::uint8_t> bytes) noexcept
{
   oled_send(false, bytes);
}

/* Acknowledge before calling out, then read back, so the clear has reached the
 * port before the exception returns. Otherwise a write still in flight on the
 * APB re-pends the interrupt that is returning. */
void acknowledge(gpio_port const& port) noexcept
{
   std::uint32_t const pending = port.mis();
   port.icr() = pending;
   [[maybe_unused]] std::uint32_t const settled = port.mis();
   if (edge_callback != nullptr) {
      edge_callback();
   }
}

}  // namespace

extern "C" void GPIOA_Handler(void) { acknowledge(gpio_a); }
extern "C" void GPIOD_Handler(void) { acknowledge(gpio_d); }
extern "C" void GPIOE_Handler(void) { acknowledge(gpio_e); }
extern "C" void GPIOF_Handler(void) { acknowledge(gpio_f); }

namespace orbit
{

void init(std::uint32_t sysclk_hz) noexcept
{
   reg(sysctl_rcgcgpio) |= 0x3Fu;     /* ports A to F */
   reg(sysctl_rcgcssi)  |= 1u << 3;   /* SSI3 */
   reg(sysctl_rcgcadc)  |= 1u << 0;   /* ADC0 */
   while ((reg(sysctl_prgpio) & 0x3Fu) != 0x3Fu) {}
   while ((reg(sysctl_prssi) & (1u << 3)) == 0u) {}
   while ((reg(sysctl_pradc) & 1u) == 0u) {}

   /* PD7 and PF0 are locked at reset (PD7 is NMI). One write each to GPIOCR,
    * after which the lock goes back on and the pins stay ordinary GPIOs. */
   gpio_d.lock() = gpio_lock_key;
   gpio_d.cr() |= pin(7);
   gpio_d.lock() = 0u;
   gpio_f.lock() = gpio_lock_key;
   gpio_f.cr() |= pin(0);
   gpio_f.lock() = 0u;

   /* The OLED's controls. Each level is latched BEFORE its pin becomes an
    * output, so neither supply glitches on and reset is never pulsed by
    * accident: supplies off (high), reset released (high), chip select off
    * (high), D/C command (low). */
   gpio_e.data(pin(1) | pin(2) | pin(5)) = pin(1) | pin(2) | pin(5);
   gpio_e.dir() |= pin(1) | pin(2) | pin(5);
   gpio_d.data(pin(1) | pin(7)) = pin(1);
   gpio_d.dir() |= pin(1) | pin(7);

   /* SSI3 on PD0 (clock) and PD3 (transmit), port control value 1. PD2 is
    * SSI3's receive pin too, and stays a GPIO for BTN1: the panel is write
    * only. */
   gpio_d.pctl() = (gpio_d.pctl() & ~0x0000F00Fu) | 0x00001001u;
   gpio_d.afsel() |= pin(0) | pin(3);

   /* The LEDs, dark. */
   gpio_c.data(pin(6) | pin(7)) = 0u;
   gpio_c.dir() |= pin(6) | pin(7);
   gpio_c.den() |= pin(6) | pin(7);
   gpio_d.data(pin(6)) = 0u;
   gpio_d.dir() |= pin(6);
   gpio_b.data(pin(5)) = 0u;
   gpio_b.dir() |= pin(5);
   gpio_b.den() |= pin(5);
   gpio_f.data(pin(1) | pin(2) | pin(3)) = 0u;
   gpio_f.dir() |= pin(1) | pin(2) | pin(3);

   /* The inputs. The manual has the Orbit's buttons and switches reading 0
    * when released or down, so the board holds them low itself, and the
    * internal pull-downs only keep an unplugged BoosterPack from reading as
    * pressed. The LaunchPad's switches close to ground and need the pull-ups. */
   gpio_d.pdr() |= pin(2);
   gpio_e.pdr() |= pin(0);
   gpio_a.pdr() |= pin(6) | pin(7);
   gpio_a.den() |= pin(6) | pin(7);
   gpio_f.pur() |= pin(0) | pin(4);

   gpio_d.den() |= pin(0) | pin(1) | pin(2) | pin(3) | pin(6) | pin(7);
   gpio_e.den() |= pin(0) | pin(1) | pin(2) | pin(5);
   gpio_f.den() |= pin(0) | pin(1) | pin(2) | pin(3) | pin(4);

   /* PE3 is AIN0: analog, digital input off. */
   gpio_e.afsel() |= pin(3);
   gpio_e.den() &= ~pin(3);
   gpio_e.amsel() |= pin(3);

   /* SSI3: master, Freescale SPI mode 0, 8 bits, the system clock divided by
    * the smallest even prescaler that keeps it at or under the panel's
    * 10 MHz: 8 at 80 MHz, 2 at 16 MHz (8 MHz). Digilent's demo ran it at
    * exactly 10 MHz. */
   std::uint32_t prescale = (sysclk_hz + 9'999'999u) / 10'000'000u;
   prescale += prescale & 1u;
   if (prescale < 2u) { prescale = 2u; }
   reg(ssi3_cr1)  = 0u;
   reg(ssi3_cc)   = 0u;
   reg(ssi3_cpsr) = prescale;
   reg(ssi3_cr0)  = 0x07u;
   reg(ssi3_cr1)  = ssi_cr1_sse;

   /* ADC0 sequencer 3, one sample of AIN0 per software trigger, clocked from
    * the 16 MHz internal oscillator so it is the same at either core clock. */
   reg(adc0_cc)     = 1u;
   reg(adc0_actss) &= ~adc_ss3;
   reg(adc0_emux)  &= ~0xF000u;
   reg(adc0_ssmux3) = 0u;
   reg(adc0_ssctl3) = 0x6u;           /* IE0 | END0 */
   reg(adc0_actss) |= adc_ss3;
}

void enable_input_interrupts(std::uint8_t priority, void (*on_edge)()) noexcept
{
   edge_callback = on_edge;

   struct source { gpio_port port; std::uint32_t pins; std::uint32_t irq; };
   for (source const& s : { source{gpio_a, pin(6) | pin(7), irq_gpio_a},
                            source{gpio_d, pin(2),          irq_gpio_d},
                            source{gpio_e, pin(0),          irq_gpio_e},
                            source{gpio_f, pin(0) | pin(4), irq_gpio_f} }) {
      /* Masked while the sense changes, which can latch a spurious edge,
       * then cleared, then unmasked. */
      s.port.im()  &= ~s.pins;
      s.port.is()  &= ~s.pins;
      s.port.ibe() |= s.pins;
      s.port.icr()  = s.pins;
      s.port.im()  |= s.pins;
      reg8(nvic_ipr + s.irq) = priority;
      reg(nvic_iser0) = 1u << s.irq;
   }
}

std::uint32_t read_inputs() noexcept
{
   std::uint32_t inputs = 0u;
   if (gpio_d.data(pin(2)) != 0u) { inputs |= btn1; }
   if (gpio_e.data(pin(0)) != 0u) { inputs |= btn2; }
   if (gpio_a.data(pin(7)) != 0u) { inputs |= sw1; }
   if (gpio_a.data(pin(6)) != 0u) { inputs |= sw2; }
   if (gpio_f.data(pin(4)) == 0u) { inputs |= lp_sw1; }
   if (gpio_f.data(pin(0)) == 0u) { inputs |= lp_sw2; }
   return inputs;
}

void set_leds(std::uint32_t mask) noexcept
{
   gpio_c.data(pin(6)) = (mask & 1u) != 0u ? pin(6) : 0u;
   gpio_c.data(pin(7)) = (mask & 2u) != 0u ? pin(7) : 0u;
   gpio_d.data(pin(6)) = (mask & 4u) != 0u ? pin(6) : 0u;
   gpio_b.data(pin(5)) = (mask & 8u) != 0u ? pin(5) : 0u;
}

void set_red(bool on) noexcept   { gpio_f.data(pin(1)) = on ? pin(1) : 0u; }
void set_blue(bool on) noexcept  { gpio_f.data(pin(2)) = on ? pin(2) : 0u; }
void set_green(bool on) noexcept { gpio_f.data(pin(3)) = on ? pin(3) : 0u; }

std::uint32_t read_pot() noexcept
{
   reg(adc0_pssi) = adc_ss3;
   while ((reg(adc0_ris) & adc_ss3) == 0u) {}
   std::uint32_t const sample = reg(adc0_ssfifo3) & 0xFFFu;
   reg(adc0_isc) = adc_ss3;
   return sample;
}

void start_awake_counter() noexcept
{
   reg(sysctl_rcgctimer) |= 1u << 0;
   while ((reg(sysctl_prtimer) & 1u) == 0u) {}

   reg(sysctl_scgcgpio)  |= 0x3Fu;
   reg(sysctl_scgcuart)  |= 1u << 0;
   reg(sysctl_scgctimer) &= ~(1u << 0);
   reg(sysctl_rcc)       |= rcc_acg;      /* TivaWare's SysCtlPeripheralClockGating(true) */

   /* 32-bit, periodic, counting up through the whole range, and stalled while
    * a debugger halts the core, so a halt is not counted as work. */
   reg(timer0_ctl)   = 0u;
   reg(timer0_cfg)   = 0u;
   reg(timer0_tamr)  = 0x2u | (1u << 4);  /* periodic, TACDIR up */
   reg(timer0_tailr) = 0xFFFFFFFFu;
   reg(timer0_ctl)   = (1u << 0) | (1u << 1);   /* TAEN, TASTALL */
}

std::uint32_t awake_cycles() noexcept
{
   return reg(timer0_tav);
}

void oled_power_on(void (*sleep_ms)(std::uint32_t)) noexcept
{
   gpio_e.data(pin(2)) = 0u;          /* VDD on */
   sleep_ms(1u);
   oled_command({0xAE});              /* display off */

   gpio_e.data(pin(5)) = 0u;          /* reset, 3 us at least */
   sleep_ms(1u);
   gpio_e.data(pin(5)) = pin(5);

   oled_command({0x8D, 0x14,          /* charge pump on */
                 0xD9, 0xF1});        /* pre-charge period */

   gpio_e.data(pin(1)) = 0u;          /* VBAT on */
   sleep_ms(100u);

   /* Digilent's orientation and COM wiring for this panel, then horizontal
    * addressing, so a frame is one burst of 512 bytes. */
   oled_command({0xA1,                /* column 127 is segment 0 */
                 0xC8,                /* scan COM from the bottom */
                 0xDA, 0x20,          /* sequential COM, left/right remap */
                 0x20, 0x00});        /* horizontal addressing */

   oled_write(frame{});
   oled_command({0xAF});              /* display on */
}

void oled_write(frame const& pixels) noexcept
{
   oled_command({0x21, 0, 127,        /* columns 0 to 127 */
                 0x22, 0, 3});        /* pages 0 to 3 */
   oled_send(true, pixels);
}

}  // namespace orbit
