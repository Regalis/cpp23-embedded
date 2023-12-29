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

#ifndef UART_HPP
#define UART_HPP

#include <algorithm>
#include <bits/ranges_algo.h>
#include <cstdint>
#include <string_view>
#include <tuple>

#include "reset.hpp"
#include "rp2040.hpp"

namespace uart {

struct baudrate_descriptor
{
    uint32_t integer_divisor;
    uint32_t fractional_divisor;
};

using baudrate_calculation = std::tuple<baudrate_descriptor, uint32_t>;
using parity = platform::uart::uartlcr_h_region_parity_values;
using stop_bits = platform::uart::uartlcr_h_region_stop_bits_values;
using word_length = platform::uart::uartlcr_h_region_wlen_values;

constexpr baudrate_calculation baudrate_calculate(uint32_t requested_baudrate)
{
    uint32_t baud_rate_div = (8 * board::peri_clk_hz / requested_baudrate);
    uint32_t integer_divisor = baud_rate_div >> 7;
    uint32_t fractional_divisor;

    if (integer_divisor == 0) {
        integer_divisor = 1;
        fractional_divisor = 0;
    } else if (integer_divisor >= 65535) {
        integer_divisor = 65535;
        fractional_divisor = 0;
    } else {
        fractional_divisor = ((baud_rate_div & 0x7f) + 1) / 2;
    }

    uint32_t real_baudrate =
      (4 * board::peri_clk_hz / (64 * integer_divisor + fractional_divisor));

    return {baudrate_descriptor{.integer_divisor = integer_divisor,
                                .fractional_divisor = fractional_divisor},
            real_baudrate};
}

namespace detail {
template<typename T>
class uart
{
  public:
    using descriptor = T;

    /**
     * Initialize UART and configure to work at the requested baudrate.
     *
     * @return the real configured baudrate
     */
    static constexpr uint32_t init(
      uint32_t requested_baudrate,
      word_length data_bits = word_length::word_8_bits,
      stop_bits stop_bits = stop_bits::one,
      parity parity = parity::odd)
    {
        reset::reset_subsystem(descriptor::reset_bit);
        reset::release_subsystem_wait(descriptor::reset_bit);

        uint32_t real_baudrate = set_baudrate(requested_baudrate);
        set_format(data_bits, stop_bits, parity);
        descriptor::uartcr::set_bits(platform::uart::uartcr_bits::uarten,
                                     platform::uart::uartcr_bits::txe,
                                     platform::uart::uartcr_bits::rxe);

        return real_baudrate;
    };

    /**
     * Set baudrate
     *
     * WARNING! Reinitialization of a currently enabled UART is NOT supported.
     *
     */
    static constexpr uint32_t set_baudrate(uint32_t requested_baudrate)
    {
        const auto& [baud, real_baud] = baudrate_calculate(requested_baudrate);
        descriptor::uartibrd::set_value(baud.integer_divisor);
        descriptor::uartfbrd::set_value(baud.fractional_divisor);
        // Dummy write to latch in the divisors
        descriptor::uartlcr_h::set_value(descriptor::uartlcr_h::value());
        return real_baud;
    }

    static constexpr void set_format(word_length data_bits,
                                     stop_bits stop_bits,
                                     parity parity)
    {
        descriptor::uartlcr_h::update_regions(
          platform::uart::uartlcr_h_region_wlen{data_bits},
          platform::uart::uartlcr_h_region_stop_bits{stop_bits},
          platform::uart::uartlcr_h_region_parity{parity});
    }

    static constexpr bool is_readable()
    {
        return !descriptor::uartfr::get_bit(platform::uart::uartfr_bits::rxfe);
    }

    static constexpr bool is_writable()
    {
        return !descriptor::uartfr::get_bit(platform::uart::uartfr_bits::txff);
    }

    static constexpr void putc(char character)
    {
        while (!is_writable()) {
            // wait
        }
        descriptor::uartdr::set_value(character);
    }

    static constexpr char getc()
    {
        while (!is_readable()) {
            // wait
        }
        return static_cast<char>(descriptor::uartdr::value());
    }

    static constexpr void puts(std::string_view data)
    {
        std::ranges::for_each(data, [](char character) {
            putc(character);
        });
    }
};
}

using uart0 = detail::uart<platform::uart::uart0>;
using uart1 = detail::uart<platform::uart::uart1>;

constexpr uart0 uart0_tag [[maybe_unused]] = uart0{};
constexpr uart1 uart1_tag [[maybe_unused]] = uart1{};

};

#endif
