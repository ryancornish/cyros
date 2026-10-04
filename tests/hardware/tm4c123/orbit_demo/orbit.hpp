/**
 * @file orbit.hpp
 * @brief The Digilent Orbit BoosterPack on the EK-TM4C123GXL, register level.
 *
 * Pins, from the Orbit reference manual's appendix A matched header position
 * by header position against TI SPMU296, because the manual was written for
 * the Stellaris LaunchPad and names four pins this part does not have. The
 * OLED's were confirmed on this board by halting Digilent's own demo and
 * reading its GPIO and SSI registers back.
 *
 * | what                 | pin  | notes                                      |
 * |----------------------|------|--------------------------------------------|
 * | OLED clock, data     | PD0, PD3 | SSI3, 10 MHz at most               |
 * | OLED chip select     | PD1  | GPIO, active low                           |
 * | OLED data/command    | PD7  | GPIO, an NMI pin, locked at reset          |
 * | OLED reset           | PE5  | active low (the manual says PF5)           |
 * | OLED VBAT, VDD       | PE1, PE2 | active low (the manual says PF1 for VBAT) |
 * | BTN1, BTN2           | PD2, PE0 | read 1 when pressed                    |
 * | SW1, SW2 (slides)    | PA7, PA6 | read 1 when up, toward the OLED        |
 * | LD1 to LD4           | PC6, PC7, PD6, PB5 | high is on                   |
 * | potentiometer        | PE3  | AIN0                                       |
 * | LaunchPad SW1, SW2   | PF4, PF0 | read 0 when pressed, PF0 locked        |
 * | LaunchPad RGB LED    | PF1, PF2, PF3 | red, blue, green, high is on      |
 *
 * PD0 and PD1 are also wired to PB6 and PB7 through the LaunchPad's R9 and
 * R10, so PB6 and PB7 stay inputs. PC0 to PC3 are JTAG and are never written.
 * Nothing here knows about cyros: the one thing that needs time, the OLED's
 * power-on sequence, is handed a sleep.
 */

#ifndef ORBIT_DEMO_ORBIT_HPP
#define ORBIT_DEMO_ORBIT_HPP

#include <array>
#include <cstdint>

namespace orbit
{

/* The inputs, normalised so that 1 means pressed, or up for a slide switch. */
inline constexpr std::uint32_t btn1   = 1u << 0;
inline constexpr std::uint32_t btn2   = 1u << 1;
inline constexpr std::uint32_t sw1    = 1u << 2;
inline constexpr std::uint32_t sw2    = 1u << 3;
inline constexpr std::uint32_t lp_sw1 = 1u << 4;
inline constexpr std::uint32_t lp_sw2 = 1u << 5;

inline constexpr std::size_t frame_bytes = 128u * 32u / 8u;
using frame = std::array<std::uint8_t, frame_bytes>;

/**
 * @brief Clock and configure every pin, SSI3 and ADC0, with the OLED's
 *        supplies off and every LED dark. Interrupts stay disabled.
 * @param sysclk_hz the core clock, which sets the SSI3 divider.
 */
void init(std::uint32_t sysclk_hz) noexcept;

/**
 * @brief Interrupt on both edges of all six inputs.
 * @param priority the NVIC priority byte for the four GPIO port IRQs.
 * @param on_edge  called from the interrupt, after the edge is acknowledged.
 */
void enable_input_interrupts(std::uint8_t priority, void (*on_edge)()) noexcept;

[[nodiscard]] std::uint32_t read_inputs() noexcept;

/** @brief LD1 to LD4 from bits 0 to 3. */
void set_leds(std::uint32_t mask) noexcept;

void set_red(bool on) noexcept;
void set_green(bool on) noexcept;
void set_blue(bool on) noexcept;

/** @brief One conversion of the potentiometer, 0 to 4095. About 2 us. */
[[nodiscard]] std::uint32_t read_pot() noexcept;

/**
 * @brief Start a counter that runs at the core clock only while the core is
 *        awake, and stops while it sleeps in WFI.
 *
 * Timer 0 is clocked in run mode and gated in sleep, with auto clock gating
 * on (RCC.ACG). Gating applies to every peripheral, so the GPIO ports keep
 * their sleep clocks, to see an edge while the core sleeps, and so does UART0,
 * to drain the console's FIFO. SSI3 and ADC0 are only used polled to
 * completion, so they are gated with the core.
 *
 * The DWT cycle counter cannot do this here. It ran on through WFI on this
 * board with the core asleep (DHCSR S_SLEEP set), with or without halting
 * debug enabled.
 */
void start_awake_counter() noexcept;

/** @brief Core cycles spent awake, modulo 2^32. */
[[nodiscard]] std::uint32_t awake_cycles() noexcept;

/**
 * @brief The UG-2832's power-on sequence from the Orbit manual, ending with
 *        a blank screen and the display on. Takes a little over 100 ms.
 */
void oled_power_on(void (*sleep_ms)(std::uint32_t)) noexcept;

/**
 * @brief Send a whole frame: page-major, 128 bytes a page, bit 0 the top row
 *        of the page. About 0.5 ms at 10 MHz, polled.
 */
void oled_write(frame const& pixels) noexcept;

}  // namespace orbit

#endif
