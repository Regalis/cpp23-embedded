/*
 *
 * Copyright (C) 2023 Patryk Jaworski (blog.regalis.tech)
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

#ifndef PWM_HPP
#define PWM_HPP

#include "gpio.hpp"
#include "hwio.hpp"
#include "rp2040.hpp"

#include <limits>
#include <type_traits>
#include <utility>

namespace pwm {

using clkdiv_mode = platform::pwm::csr_region_divmode_values;

struct frequency_config
{
    uint16_t wrap;
    uint16_t integer_divisor;
    uint16_t fractional_divisor;
};

consteval bool verify_frequency_config(frequency_config cfg)
{
    if (cfg.integer_divisor > 255) {
        return false;
    }

    if (cfg.fractional_divisor > 15) {
        return false;
    }

    return true;
}

constexpr frequency_config get_frequency_config_for(uint32_t target_frequency)
{
    constexpr uint32_t top_max = std::numeric_limits<uint16_t>::max() - 1;
    constexpr auto calculate_top = [](uint32_t target_hz) {
        return ((board::clocks::sys_clk_hz + target_hz / 2) / target_hz);
    };
    uint64_t target_div;
    uint64_t target_wrap;

    if (calculate_top(target_frequency) < top_max) {
        target_div = 16;
        target_wrap = calculate_top(target_frequency) - 1;
    } else {
        // TODO: demonkey the following section 🐒
        // TODO: learn more about rounding...
        target_div = (16 * static_cast<uint64_t>(board::clocks::sys_clk_hz) +
                      ((top_max * target_frequency) - 1)) /
                     (top_max * target_frequency);
        target_wrap =
          ((16 * static_cast<uint64_t>(board::clocks::sys_clk_hz) +
            ((target_div * static_cast<uint64_t>(board::clocks::sys_clk_hz)) /
             2)) /
           (target_div * static_cast<uint64_t>(board::clocks::sys_clk_hz))) -
          1;
    }

    return {.wrap = static_cast<uint16_t>(target_wrap),
            .integer_divisor = static_cast<uint16_t>((target_div >> 4)),
            .fractional_divisor = static_cast<uint16_t>((target_div & 0xf))};
}

constexpr uint32_t get_frequency_from_config(frequency_config config)
{
    return (board::clocks::sys_clk_hz /
            ((config.wrap + 1UL) *
             (config.integer_divisor + (config.fractional_divisor / 16UL))));
}

namespace detail {

template<hwio::hwio_region channel_region>
struct channel
{
    using region = channel_region;
};

template<typename T>
struct pwm_slice
{
    using descriptor = T;

    static constexpr void set_frequency(uint32_t target_frequency)
    {
        const auto frequency_config =
          get_frequency_config_for(target_frequency);
        set_frequency(frequency_config);
    }

    static constexpr void set_frequency(frequency_config config)
    {
        set_clkdiv(config.integer_divisor, config.fractional_divisor);
        set_wrap(config.wrap);
    }

    static constexpr void set_wrap(uint16_t wrap)
    {
        descriptor::top::set_value(wrap);
    }

    static constexpr void set_channel_levels(const auto&... channel)
    {
        descriptor::cc::update_regions(channel...);
    }

    static constexpr void enable()
    {
        descriptor::csr::set_bits(platform::pwm::csr_bits::en);
    }

    static constexpr void disable()
    {
        descriptor::csr::reset_bits(platform::pwm::csr_bits::en);
    }

    static constexpr void set_clkdiv_mode(clkdiv_mode mode)
    {
        descriptor::csr::update_regions(
          platform::pwm::csr_region_divmode{mode});
    }

    static constexpr void set_clkdiv(uint16_t integer_divisor,
                                     uint16_t fractional_divisor = 0)
    {
        descriptor::div::update_regions(
          platform::pwm::div_region_int{integer_divisor},
          platform::pwm::div_region_frac{fractional_divisor});
    }
};

template<platform::pins Pin>
struct slice_for_pin
{
    constexpr static uint32_t slice_no = (std::to_underlying(Pin) >> 1) & 0x7;
    using type = pwm_slice<platform::pwm::detail::channel<slice_no>>;
};

enum class slice_channel
{
    channel_a,
    channel_b,
};

template<slice_channel channel>
consteval auto get_cc_register_region()
{
    if constexpr (channel == slice_channel::channel_a) {
        return platform::pwm::cc_region_a{};
    } else {
        return platform::pwm::cc_region_b{};
    }
};

template<platform::pins Pin>
struct channel_for_pin
{
    constexpr static slice_channel channel_no = (std::to_underlying(Pin) & 1)
                                                  ? slice_channel::channel_b
                                                  : slice_channel::channel_a;
    using type = std::decay_t<decltype(get_cc_register_region<channel_no>())>;
};

}

using channel_a = detail::channel<platform::pwm::cc_region_a>::region;
using channel_b = detail::channel<platform::pwm::cc_region_b>::region;

using slice0 = detail::pwm_slice<platform::pwm::ch0>;
using slice1 = detail::pwm_slice<platform::pwm::ch1>;
using slice2 = detail::pwm_slice<platform::pwm::ch2>;
using slice3 = detail::pwm_slice<platform::pwm::ch3>;
using slice4 = detail::pwm_slice<platform::pwm::ch4>;
using slice5 = detail::pwm_slice<platform::pwm::ch5>;
using slice6 = detail::pwm_slice<platform::pwm::ch6>;
using slice7 = detail::pwm_slice<platform::pwm::ch7>;

template<platform::pins Pin>
using slice_for_pin = detail::slice_for_pin<Pin>::type;

template<platform::pins Pin>
using channel_for_pin = detail::channel_for_pin<Pin>::type;

template<gpio::gpio_pin Gpio>
using channel_for_gpio = detail::channel_for_pin<Gpio::pin_no>::type;

template<typename T>
consteval auto from_gpio(T)
{
    return slice_for_pin<T::pin_no>{};
}

namespace channel {
template<typename T>
consteval auto from_gpio(T)
{
    return channel_for_gpio<T>{};
}
}

constexpr uint32_t frequency_maximum [[maybe_unused]] =
  board::clocks::sys_clk_hz;

constexpr uint32_t frequency_minimum [[maybe_unused]] =
  get_frequency_from_config(
    {.wrap = 0xffff, .integer_divisor = 0xff, .fractional_divisor = 0xf});

}

#endif
