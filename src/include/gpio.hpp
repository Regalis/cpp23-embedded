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

#ifndef GPIO_HPP
#define GPIO_HPP

#include "rp2040.hpp"
#include <type_traits>

namespace gpio {
enum class functions : uint8_t
{
    spi = 1,
    uart,
    i2c,
    pwm,
    sio,
    pio0,
    pio1,
    clock,
    usb,
};

template<platform::pins pin_no>
static constexpr void function_select(functions func)
{
    using ctrl_reg = platform::registers::gpio_ctrl<pin_no>;
    ctrl_reg::set_value(std::to_underlying(func));
}

static constexpr void set_as_output(const valid_bit_position auto&... pin_no)
{
    platform::registers::gpio_oe_set::set_bits(pin_no...);
}

static constexpr void set_high(const valid_bit_position auto&... pin_no)
{
    platform::registers::gpio_out_set::set_bits(pin_no...);
}

static constexpr void set_low(const valid_bit_position auto&... pin_no)
{
    platform::registers::gpio_out_clr::set_bits(pin_no...);
}

static constexpr void toggle(const valid_bit_position auto&... pin_no)
{
    platform::registers::gpio_out_xor::set_bits(pin_no...);
}

template<platform::pins Pin>
class pin
{
  public:
    static constexpr platform::pins pin_no = Pin;

    static constexpr void function_select(gpio::functions func)
    {
        gpio::function_select<pin_no>(func);
    }

    static constexpr void set_as_output()
    {
        gpio::set_as_output(pin_no);
    }

    static constexpr void set_high()
    {
        gpio::set_high(pin_no);
    }

    static constexpr void set_low()
    {
        gpio::set_low(pin_no);
    }

    static constexpr void toggle()
    {
        gpio::toggle(pin_no);
    }
};

// Todo: add more constraints
template<typename T>
concept gpio_pin = requires(T t) { T::pin_no; };

constexpr void set_as_output(const gpio_pin auto&... gpio_pins)
{
    set_as_output(gpio_pins.pin_no...);
}

constexpr void set_high(const gpio_pin auto&... gpio_pins)
{
    set_high(gpio_pins.pin_no...);
}

constexpr void set_low(const gpio_pin auto&... gpio_pins)
{
    set_low(gpio_pins.pin_no...);
}

constexpr void toggle(const gpio_pin auto&... gpio_pins)
{
    toggle(gpio_pins.pin_no...);
}

constexpr void function_select(functions func,
                               const gpio_pin auto&... gpio_pins)
{
    ((function_select<gpio_pins.pin_no>(func)), ...);
}

}

#endif
