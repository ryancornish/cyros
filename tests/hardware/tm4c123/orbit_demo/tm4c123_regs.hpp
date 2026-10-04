/**
 * @file tm4c123_regs.hpp
 * @brief The TM4C123GH6PM registers the Orbit demo touches, and nothing else.
 *
 * Addresses and bits from TI's TM4C123GH6PM datasheet (SPMS376E). The board
 * files next door spell their few registers out as C macros. The demo needs
 * a whole GPIO port's worth, so it names a port once and indexes it.
 */

#ifndef ORBIT_DEMO_TM4C123_REGS_HPP
#define ORBIT_DEMO_TM4C123_REGS_HPP

#include <cstdint>

namespace tm4c
{

[[nodiscard]] inline volatile std::uint32_t& reg(std::uintptr_t address) noexcept
{
   return *reinterpret_cast<volatile std::uint32_t*>(address);
}

[[nodiscard]] inline volatile std::uint8_t& reg8(std::uintptr_t address) noexcept
{
   return *reinterpret_cast<volatile std::uint8_t*>(address);
}

/* System control: run-mode clock gating and the matching ready bits. A
 * peripheral's registers fault with a bus error until its ready bit is set. */
inline constexpr std::uintptr_t sysctl_rcc       = 0x400FE060u;
inline constexpr std::uintptr_t sysctl_rcgctimer = 0x400FE604u;
inline constexpr std::uintptr_t sysctl_prtimer   = 0x400FEA04u;
inline constexpr std::uintptr_t sysctl_rcgcgpio = 0x400FE608u;
inline constexpr std::uintptr_t sysctl_rcgcssi  = 0x400FE61Cu;
inline constexpr std::uintptr_t sysctl_rcgcadc  = 0x400FE638u;
inline constexpr std::uintptr_t sysctl_prgpio   = 0x400FEA08u;
inline constexpr std::uintptr_t sysctl_prssi    = 0x400FEA1Cu;
inline constexpr std::uintptr_t sysctl_pradc    = 0x400FEA38u;

/* Sleep-mode clock gating. With RCC.ACG set, a peripheral keeps its clock
 * while the core sleeps only if its bit here is set. */
inline constexpr std::uint32_t  rcc_acg          = 1u << 27;
inline constexpr std::uintptr_t sysctl_scgctimer = 0x400FE704u;
inline constexpr std::uintptr_t sysctl_scgcgpio  = 0x400FE708u;
inline constexpr std::uintptr_t sysctl_scgcuart  = 0x400FE718u;

/* Timer 0, used whole as one 32-bit counter. */
inline constexpr std::uintptr_t timer0_cfg   = 0x40030000u;
inline constexpr std::uintptr_t timer0_tamr  = 0x40030004u;
inline constexpr std::uintptr_t timer0_ctl   = 0x4003000Cu;
inline constexpr std::uintptr_t timer0_tailr = 0x40030028u;
inline constexpr std::uintptr_t timer0_tav   = 0x40030050u;

/* One GPIO port on the APB aperture. DATA is address-masked: bits 9:2 of the
 * address select which pins a read sees or a write changes, so writing one pin
 * never needs a read-modify-write and cannot race another writer. */
struct gpio_port
{
   std::uintptr_t base;

   [[nodiscard]] volatile std::uint32_t& data(std::uint32_t pins) const noexcept { return reg(base + (pins << 2)); }
   [[nodiscard]] volatile std::uint32_t& dir()   const noexcept { return reg(base + 0x400u); }
   [[nodiscard]] volatile std::uint32_t& is()    const noexcept { return reg(base + 0x404u); }
   [[nodiscard]] volatile std::uint32_t& ibe()   const noexcept { return reg(base + 0x408u); }
   [[nodiscard]] volatile std::uint32_t& im()    const noexcept { return reg(base + 0x410u); }
   [[nodiscard]] volatile std::uint32_t& mis()   const noexcept { return reg(base + 0x418u); }
   [[nodiscard]] volatile std::uint32_t& icr()   const noexcept { return reg(base + 0x41Cu); }
   [[nodiscard]] volatile std::uint32_t& afsel() const noexcept { return reg(base + 0x420u); }
   [[nodiscard]] volatile std::uint32_t& pur()   const noexcept { return reg(base + 0x510u); }
   [[nodiscard]] volatile std::uint32_t& pdr()   const noexcept { return reg(base + 0x514u); }
   [[nodiscard]] volatile std::uint32_t& den()   const noexcept { return reg(base + 0x51Cu); }
   [[nodiscard]] volatile std::uint32_t& lock()  const noexcept { return reg(base + 0x520u); }
   [[nodiscard]] volatile std::uint32_t& cr()    const noexcept { return reg(base + 0x524u); }
   [[nodiscard]] volatile std::uint32_t& amsel() const noexcept { return reg(base + 0x528u); }
   [[nodiscard]] volatile std::uint32_t& pctl()  const noexcept { return reg(base + 0x52Cu); }
};

inline constexpr gpio_port gpio_a{0x40004000u};
inline constexpr gpio_port gpio_b{0x40005000u};
inline constexpr gpio_port gpio_c{0x40006000u};
inline constexpr gpio_port gpio_d{0x40007000u};
inline constexpr gpio_port gpio_e{0x40024000u};
inline constexpr gpio_port gpio_f{0x40025000u};

/* Writing this to GPIOLOCK opens GPIOCR for one write, which is how PD7 (an
 * NMI pin) and PF0 become ordinary GPIOs. */
inline constexpr std::uint32_t gpio_lock_key = 0x4C4F434Bu;

/* SSI3, the OLED's SPI master. */
inline constexpr std::uintptr_t ssi3_cr0  = 0x4000B000u;
inline constexpr std::uintptr_t ssi3_cr1  = 0x4000B004u;
inline constexpr std::uintptr_t ssi3_dr   = 0x4000B008u;
inline constexpr std::uintptr_t ssi3_sr   = 0x4000B00Cu;
inline constexpr std::uintptr_t ssi3_cpsr = 0x4000B010u;
inline constexpr std::uintptr_t ssi3_cc   = 0x4000BFC8u;
inline constexpr std::uint32_t  ssi_cr1_sse = 1u << 1;
inline constexpr std::uint32_t  ssi_sr_tnf  = 1u << 1;
inline constexpr std::uint32_t  ssi_sr_bsy  = 1u << 4;

/* ADC0, sample sequencer 3: one sample per trigger. */
inline constexpr std::uintptr_t adc0_actss   = 0x40038000u;
inline constexpr std::uintptr_t adc0_ris     = 0x40038004u;
inline constexpr std::uintptr_t adc0_isc     = 0x4003800Cu;
inline constexpr std::uintptr_t adc0_emux    = 0x40038014u;
inline constexpr std::uintptr_t adc0_pssi    = 0x40038028u;
inline constexpr std::uintptr_t adc0_ssmux3  = 0x400380A0u;
inline constexpr std::uintptr_t adc0_ssctl3  = 0x400380A4u;
inline constexpr std::uintptr_t adc0_ssfifo3 = 0x400380A8u;
inline constexpr std::uintptr_t adc0_cc      = 0x40038FC8u;
inline constexpr std::uint32_t  adc_ss3      = 1u << 3;

/* The core's interrupt controller and SysTick's priority. */
inline constexpr std::uintptr_t nvic_iser0   = 0xE000E100u;
inline constexpr std::uintptr_t nvic_ipr     = 0xE000E400u;   /* one byte per IRQ */
inline constexpr std::uintptr_t shpr_systick = 0xE000ED23u;

/* Device IRQ numbers (datasheet table 2-9). */
inline constexpr std::uint32_t irq_gpio_a = 0;
inline constexpr std::uint32_t irq_gpio_d = 3;
inline constexpr std::uint32_t irq_gpio_e = 4;
inline constexpr std::uint32_t irq_gpio_f = 30;

}  // namespace tm4c

#endif
