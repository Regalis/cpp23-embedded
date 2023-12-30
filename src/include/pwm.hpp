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
#include <utility>

namespace pwm {

using clkdiv_mode = platform::pwm::csr_region_divmode_values;

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

    static constexpr void set_clkdiv(uint16_t integer_divisor)
    {
        descriptor::div::update_regions(
          platform::pwm::div_region_int{integer_divisor});
    }
};

template<platform::pins Pin>
struct slice_for_pin
{
    constexpr static uint32_t slice_no = (std::to_underlying(Pin) >> 1) & 0x7;
    using type = pwm_slice<platform::pwm::detail::channel<slice_no>>;
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
using slice_for = detail::slice_for_pin<Pin>::type;

template<typename T>
consteval auto from_gpio(T)
{
    return slice_for<T::pin_no>{};
}

}

#endif
