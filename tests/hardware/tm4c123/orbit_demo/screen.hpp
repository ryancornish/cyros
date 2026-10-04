/**
 * @file screen.hpp
 * @brief What the demo draws, as pure functions of a snapshot of its state.
 *
 * Nothing here touches hardware or cyros, so the pages can be rendered and
 * checked on the host. The frame is the SSD1306's layout, 4 pages of 128
 * columns, bit 0 of a byte the top row of its page. Text is the 5x7 font in
 * 6x8 cells: 21 columns by 4 rows.
 */

#ifndef ORBIT_DEMO_SCREEN_HPP
#define ORBIT_DEMO_SCREEN_HPP

#include <array>
#include <cstdint>
#include <string_view>

namespace screen
{

inline constexpr int width  = 128;
inline constexpr int height = 32;
inline constexpr int columns = width / 6;
inline constexpr int rows    = height / 8;

using frame = std::array<std::uint8_t, width * height / 8>;

/* The threads whose loops the second page counts, in priority order. */
enum class worker : std::uint8_t { input, sensor, hog_high, display, hog_low, leds, count };

enum class hog_mode : std::uint8_t { off, low, high };

struct snapshot
{
   std::uint32_t mhz{0};
   std::uint64_t uptime_s{0};
   std::uint32_t cpu_percent{0};          ///< over the last second, 0 to 100
   bool          cpu_known{false};        ///< false until a whole second is measured
   std::uint32_t pot{0};                  ///< 0 to 4095
   std::uint32_t btn1_presses{0};
   std::uint32_t btn2_presses{0};
   hog_mode      hog{hog_mode::off};
   std::uint32_t page{0};                 ///< 0 the dashboard, 1 the threads
   std::array<std::uint32_t, static_cast<std::size_t>(worker::count)> loops_per_s{};
};

class canvas
{
public:
   void clear() noexcept { pixels_.fill(0u); }
   void set(int x, int y) noexcept;
   void text(int column, int row, std::string_view s) noexcept;
   /** Right-aligned in `width` cells ending at `column + width - 1`, `#`s if it does not fit. */
   void number(int column, int row, std::uint64_t value, int width) noexcept;
   void frame_rect(int x0, int y0, int x1, int y1) noexcept;
   void fill_rect(int x0, int y0, int x1, int y1) noexcept;
   void invert_row(int row) noexcept;

   [[nodiscard]] frame const& pixels() const noexcept { return pixels_; }

private:
   frame pixels_{};
};

void render(snapshot const& s, canvas& c) noexcept;

}  // namespace screen

#endif
