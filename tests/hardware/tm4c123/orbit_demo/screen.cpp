/**
 * @file screen.cpp
 * @brief The demo's two pages. Layout notes in screen.hpp.
 */

#include "screen.hpp"
#include "font5x7.hpp"

#include <initializer_list>

namespace screen
{

void canvas::set(int x, int y) noexcept
{
   if (x < 0 || x >= width || y < 0 || y >= height) { return; }
   pixels_[static_cast<std::size_t>((y / 8) * width + x)] |= static_cast<std::uint8_t>(1u << (y % 8));
}

void canvas::text(int column, int row, std::string_view s) noexcept
{
   if (row < 0 || row >= rows) { return; }
   for (char ch : s) {
      if (column >= columns) { return; }
      if (ch < font_first || ch > font_last) { ch = '?'; }
      auto const& glyph = font5x7[static_cast<std::size_t>(ch - font_first)];
      std::size_t const at = static_cast<std::size_t>(row * width + column * 6);
      for (std::size_t i = 0; i < glyph.size(); ++i) {
         pixels_[at + i] = glyph[i];
      }
      pixels_[at + 5] = 0u;
      ++column;
   }
}

void canvas::number(int column, int row, std::uint64_t value, int cells) noexcept
{
   char digits[21];
   int  length = 0;
   do {
      digits[length++] = static_cast<char>('0' + value % 10u);
      value /= 10u;
   } while (value != 0u && length < 20);
   if (value != 0u || length > cells) {
      for (int i = 0; i < cells; ++i) { text(column + i, row, "#"); }
      return;
   }
   for (int i = 0; i < cells - length; ++i) { text(column + i, row, " "); }
   for (int i = 0; i < length; ++i) {
      char const one[1] = { digits[length - 1 - i] };
      text(column + cells - length + i, row, std::string_view{one, 1});
   }
}

void canvas::frame_rect(int x0, int y0, int x1, int y1) noexcept
{
   for (int x = x0; x <= x1; ++x) { set(x, y0); set(x, y1); }
   for (int y = y0; y <= y1; ++y) { set(x0, y); set(x1, y); }
}

void canvas::fill_rect(int x0, int y0, int x1, int y1) noexcept
{
   for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) { set(x, y); }
   }
}

void canvas::invert_row(int row) noexcept
{
   for (int x = 0; x < width; ++x) {
      std::size_t const at = static_cast<std::size_t>(row * width + x);
      pixels_[at] = static_cast<std::uint8_t>(~pixels_[at]);
   }
}

namespace
{

void two_digits(canvas& c, int column, int row, std::uint64_t value)
{
   char const pair[2] = { static_cast<char>('0' + (value / 10u) % 10u),
                          static_cast<char>('0' + value % 10u) };
   c.text(column, row, std::string_view{pair, 2});
}

void cpu(canvas& c, int column, int row, snapshot const& s)
{
   c.text(column, row, "cpu");
   if (s.cpu_known) {
      c.number(column + 4, row, s.cpu_percent, 3);
   } else {
      c.text(column + 4, row, " --");
   }
   c.text(column + 7, row, "%");
}

/*  cyros 80MHz 00:01:23     title, inverted
 *  pot [#######    ]  61    the bar is drawn, not text
 *  btn1    3  btn2    0
 *  hog off      cpu   2%  */
void dashboard(snapshot const& s, canvas& c)
{
   c.text(0, 0, "cyros");
   c.number(6, 0, s.mhz, 2);
   c.text(8, 0, "MHz");
   std::uint64_t const hours = s.uptime_s / 3600u;
   two_digits(c, 13, 0, hours);
   c.text(15, 0, ":");
   two_digits(c, 16, 0, (s.uptime_s / 60u) % 60u);
   c.text(18, 0, ":");
   two_digits(c, 19, 0, s.uptime_s % 60u);
   c.invert_row(0);

   c.text(0, 1, "pot");
   int constexpr bar_left = 21, bar_right = 103;
   c.frame_rect(bar_left, 9, bar_right, 14);
   int const filled = static_cast<int>((s.pot * static_cast<std::uint32_t>(bar_right - bar_left - 1)) / 4095u);
   if (filled > 0) {
      c.fill_rect(bar_left + 1, 10, bar_left + filled, 13);
   }
   c.number(18, 1, (s.pot * 100u + 2047u) / 4095u, 3);

   c.text(0, 2, "btn1");
   c.number(5, 2, s.btn1_presses, 4);
   c.text(11, 2, "btn2");
   c.number(16, 2, s.btn2_presses, 4);

   switch (s.hog) {
      case hog_mode::off:  c.text(0, 3, "hog off");  break;
      case hog_mode::low:  c.text(0, 3, "hog low");  break;
      case hog_mode::high: c.text(0, 3, "hog HIGH"); break;
   }
   cpu(c, 13, 3, s);
}

/*  loops/s      cpu   2%    title, inverted
 *  in1      0  sn2     50   each name is the thread and its priority
 *  hH3      0  ds4     20
 *  hL5      0  ld6      4  */
void threads(snapshot const& s, canvas& c)
{
   c.text(0, 0, "loops/s");
   cpu(c, 13, 0, s);
   c.invert_row(0);

   struct cell { std::string_view name; worker who; int column; int row; };
   for (cell const& k : { cell{.name="in1", .who=worker::input,    .column=0,  .row=1}, cell{.name="sn2", .who=worker::sensor,  .column=11, .row=1},
                          cell{.name="hH3", .who=worker::hog_high, .column=0,  .row=2}, cell{.name="ds4", .who=worker::display, .column=11, .row=2},
                          cell{.name="hL5", .who=worker::hog_low,  .column=0,  .row=3}, cell{.name="ld6", .who=worker::leds,    .column=11, .row=3}, })
   {
      c.text(k.column, k.row, k.name);
      c.number(k.column + 4, k.row, s.loops_per_s[static_cast<std::size_t>(k.who)], 6);
   }
}

}  // namespace

void render(snapshot const& s, canvas& c) noexcept
{
   c.clear();
   if (s.page == 0u) {
      dashboard(s, c);
   } else {
      threads(s, c);
   }
}

}  // namespace screen
