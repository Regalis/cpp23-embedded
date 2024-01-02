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

#ifndef HD44780_HPP
#define HD44780_HPP

#include <algorithm>
#include <chrono>
#include <ranges>
#include <type_traits>
#include <utility>

#include "gpio.hpp"
#include "rp2040.hpp"
#include "timer.hpp"

#include "instructions.hpp"

namespace drivers::lcd::hd44780 {

struct configuration
{
    uint8_t columns = 16;
    uint8_t lines = 2;
    font font_size = font::font_5x8;
};

template<typename Interface, configuration Config>
class hd44780
{
  public:
    using interface = Interface;
    static constexpr configuration config = Config;
    static_assert(
      config.lines != 1 || config.lines != 2 || config.lines != 4,
      "Invalid value for configuration::lines (select one of 1, 2 or 4");
    static_assert(config.columns <= 40,
                  "Invalid value for configuration::columns. Maximum "
                  "supported value is 40");

    static constexpr void init()
    {
        using namespace std::chrono_literals;
        using namespace instructions;
        interface::init_mcu_interface();
        interface::init_lcd_interface();

        constexpr lines number_of_lines =
          (Config.lines > 1 ? lines::two_lines : lines::one_line);

        interface::send_instruction(instructions::function_set(
          interface::lcd_interface, number_of_lines, config.font_size));

        interface::delay(1ms);

        interface::send_instruction(
          instructions::display_on_off(power::off, cursor::off, blink::off));

        interface::delay(1ms);
        interface::send_instruction(instructions::clear_display());
        interface::delay(2ms);

        interface::send_instruction(
          instructions::entry_mode_set(mode::increment, shift::off));

        interface::delay(1ms);

        interface::send_instruction(
          instructions::display_on_off(power::on, cursor::off, blink::off));
        interface::delay(1ms);
    }

    /**
     * Clear display and return home (x = 0, y = 0)
     */
    static constexpr void clear()
    {
        interface::send_instruction(instructions::clear_display());
        home();
    }

    static constexpr void home()
    {
        interface::send_instruction(instructions::return_home());
    }

    static constexpr void cursor_goto(uint8_t x, uint8_t y)
    {
        uint8_t addr = x & 0x3f;
        if (y == 1 || y == 3) {
            addr += 0x40;
        }
        if (y == 2 || y == 3) {
            addr += config.columns;
        }
        interface::send_instruction(instructions::ddram_set(addr));
    }

    static constexpr void display_on(cursor cursor = cursor::off,
                                     blink blink = blink::off)
    {
        interface::send_instruction(
          instructions::display_on_off(power::on, cursor, blink));
    }

    static constexpr void display_off()
    {
        interface::send_instruction(power::off, cursor::off, blink::off);
    }

    static constexpr void clear_line(uint8_t line)
    {
        cursor_goto(0, line);
        constexpr uint8_t start = 0;
        constexpr uint8_t stop = config.columns;
        std::ranges::for_each(std::views::iota(start, stop), [](auto) {
            putc(' ');
        });
        cursor_goto(0, line);
    }

    static constexpr void putc(char character)
    {
        interface::send_data(static_cast<uint8_t>(character));
    }

    static constexpr void puts(std::string_view str)
    {
        std::ranges::for_each(str, putc);
    }

    static constexpr void soft_puts(std::string_view str)
    {
        std::ranges::for_each(str, [](const char character) {
            putc(character);
            interface::delay(std::chrono::milliseconds(50));
        });
    }
};

namespace detail {

template<auto T>
struct interface_for
{
    static_assert(true,
                  "Undefined interface for the specified descriptor. This is "
                  "probably a driver bug.");
    using type = std::false_type;
};

}

template<auto T>
using interface_for = detail::interface_for<T>::type;
}

#endif
