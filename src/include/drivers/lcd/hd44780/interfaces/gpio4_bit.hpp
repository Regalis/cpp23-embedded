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

#ifndef HD44780_GPIO_4BIT_HPP
#define HD44780_GPIO_4BIT_HPP
#include <algorithm>
#include <ranges>

#include "gpio.hpp"
#include "rp2040.hpp"

#include "../instructions.hpp"

namespace drivers::lcd::hd44780 {

namespace interfaces {

namespace detail {
struct gpio_4bit_descriptor
{
    platform::pins register_select;
    platform::pins enable;
    platform::pins data4;
    platform::pins data5;
    platform::pins data6;
    platform::pins data7;
};

template<gpio_4bit_descriptor InterfaceDescriptor>
class gpio_4bit
{
  public:
    static constexpr auto descriptor = InterfaceDescriptor;
    static constexpr interface lcd_interface = interface::parallel_4_bit;

    static constexpr auto pin_register_select =
      gpio::pin<descriptor.register_select>{};
    static constexpr auto pin_enable = gpio::pin<descriptor.enable>{};
    static constexpr auto pin_data4 = gpio::pin<descriptor.data4>{};
    static constexpr auto pin_data5 = gpio::pin<descriptor.data5>{};
    static constexpr auto pin_data6 = gpio::pin<descriptor.data6>{};
    static constexpr auto pin_data7 = gpio::pin<descriptor.data7>{};

    static constexpr void init_mcu_interface()
    {
        gpio::function_select(gpio::functions::sio,
                              pin_register_select,
                              pin_enable,
                              pin_data4,
                              pin_data5,
                              pin_data6,
                              pin_data7);
        gpio::set_as_output(pin_register_select,
                            pin_enable,
                            pin_data4,
                            pin_data5,
                            pin_data6,
                            pin_data7);
    }

    static constexpr void init_lcd_interface()
    {
        using namespace std::chrono_literals;

        // Wait for more than 40ms after power rises to a valid level
        delay(100ms);

        // The first step - switch to 8-bit mode
        send_nibble(true, 0x03);

        // Wait for initialization for more than 4.1ms
        delay(5ms);

        // Repeat the previous command (interface is 8-bit)
        enable();

        // Wait for initialization for more than 100us
        delay(150us);

        // Repeat the previous command (interface is 8-bit)
        enable();

        // Last delay before switching to 4-bit mode
        delay(1ms);

        // Switch to 4-bit mode
        send_nibble(true, 0x02);
        delay(1ms);
    }

    static constexpr void send_instruction(uint8_t instruction)
    {
        send_nibble(true, instruction >> 4);
        send_nibble(true, instruction & 0x0f);
    }

    static constexpr void send_data(uint8_t data)
    {
        send_nibble(false, data >> 4);
        send_nibble(false, data & 0x0f);
    }

    static constexpr void enable()
    {
        using namespace std::chrono_literals;
        delay(50us);
        pin_enable.set_high();
        delay(5us);
        pin_enable.set_low();
    }

    static constexpr void delay(std::chrono::microseconds delay)
    {
        using namespace std::chrono_literals;
        timer::delay(delay);
    }

    // TODO: Optimize to use a single call to the gpio::set_high() function.
    //       Should be possible by converting std::array into a tuple using
    //       std::make_index_sequence
    static constexpr void send_nibble(bool is_instruction, uint8_t data)
    {
        if (is_instruction) {
            pin_register_select.set_low();
        } else {
            pin_register_select.set_high();
        }
        gpio::set_low(pin_data4, pin_data5, pin_data6, pin_data7);

        constexpr std::array pins{pin_data4.pin_no,
                                  pin_data5.pin_no,
                                  pin_data6.pin_no,
                                  pin_data7.pin_no};

        auto is_high = [&data](const auto& bit_pos_and_pin) -> bool {
            const auto& [bit_position, pin] = bit_pos_and_pin;
            return read_bits(data, bit_position);
        };

        constexpr auto set_pin_high = [](const platform::pins pin) {
            gpio::set_high(pin);
        };

        auto pins_to_set = std::views::enumerate(pins) |
                           std::views::filter(is_high) |
                           std::views::elements<1>;

        std::ranges::for_each(pins_to_set, set_pin_high);

        enable();
    }
};
}

using gpio4_bit = detail::gpio_4bit_descriptor;

}

namespace detail {
template<interfaces::gpio4_bit descriptor>
struct interface_for<descriptor>
{
    using type = interfaces::detail::gpio_4bit<descriptor>;
};
}

}

#endif
