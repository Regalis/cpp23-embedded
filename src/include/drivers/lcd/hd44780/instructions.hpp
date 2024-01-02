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

#ifndef HD44780_INSTRUCTIONS_HPP
#define HD44780_INSTRUCTIONS_HPP

#include "rp2040.hpp"

namespace drivers::lcd::hd44780 {

// display on/off params
enum class power : uint8_t
{
    off = 0,
    on = 0x04,
};

enum class cursor : uint8_t
{
    off = 0,
    on = 0x02,
};

enum class blink : uint8_t
{
    off = 0,
    on = 0x01,
};

// entry_mode_set params
enum class mode : uint8_t
{
    decrement = 0,
    increment = 0x02
};
#include <utility>

enum class shift : uint8_t
{
    off = 0,
    on = 0x01,
};

// function set params
enum class interface : uint8_t
{
    parallel_4_bit = 0,
    parallel_8_bit = 0x10,
};

enum class lines : uint8_t
{
    one_line = 0,
    two_lines = 0x08,
};

enum class font : uint8_t
{
    font_5x8 = 0,
    font_5x11 = 0x04,
};

namespace instructions {

namespace detail {
constexpr uint8_t build_instruction(uint8_t instruction_code,
                                    const auto&... arguments)
{
    return instruction_code | (std::to_underlying(arguments) | ...);
}

}

constexpr auto clear_display [[maybe_unused]] = [] {
    return 0x01;
};

constexpr auto return_home [[maybe_unused]] = [] {
    return 0x02;
};

constexpr auto entry_mode_set [[maybe_unused]] = [](mode mode, shift shift) {
    return detail::build_instruction(0x04, mode, shift);
};

constexpr auto display_on_off
  [[maybe_unused]] = [](power power, cursor cursor, blink blink) {
      return detail::build_instruction(0x08, power, cursor, blink);
  };

constexpr auto function_set
  [[maybe_unused]] = [](interface interface, lines lines, font font) {
      return detail::build_instruction(0x20, interface, lines, font);
  };

constexpr auto ddram_set [[maybe_unused]] = [](uint8_t address) {
    return bitwise_or(0x80, bitwise_and(address, 0x7F));
};

}
}

#endif
