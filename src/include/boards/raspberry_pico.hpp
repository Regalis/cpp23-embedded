/*
 *
 * Copyright (C) 2023-2024 Patryk Jaworski (blog.regalis.tech)
 *
 * Author: Patryk Jaworski <regalis@regalis.tech>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <cstdint>

namespace board {

namespace pll {
constexpr uint32_t common_refdiv = 1UL;
constexpr uint32_t pll_sys_vco_freq_khz = 1500'000UL;
constexpr uint32_t pll_usb_vco_freq_khz = 1200'000UL;
constexpr uint32_t pll_sys_postdiv1 = 6UL;
constexpr uint32_t pll_sys_postdiv2 = 2UL;
constexpr uint32_t pll_usb_postdiv1 = 5UL;
constexpr uint32_t pll_usb_postdiv2 = 5UL;
}

namespace clocks {
constexpr uint32_t sys_clk_hz = 125'000'000UL;
constexpr uint32_t peri_clk_hz = 125'000'000UL;
constexpr uint32_t usb_clk_hz = 48'000'000UL;
constexpr uint32_t rtc_clock_hz = usb_clk_hz / 1024;
constexpr uint32_t rosc_clock_hz = 6'500'000;
}

}
