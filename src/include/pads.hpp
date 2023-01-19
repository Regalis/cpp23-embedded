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

#ifndef PADS_HPP
#define PADS_HPP

#include "rp2040.hpp"

namespace pads {
struct gpio
{
    enum class pad : uint8_t
    {
        gpio0 = 0,
        gpio1,
        gpio2,
        gpio3,
        gpio4,
        gpio5,
        gpio6,
        gpio7,
        gpio8,
        gpio9,
        gpio10,
        gpio11,
        gpio12,
        gpio13,
        gpio14,
        gpio15,
        gpio16,
        gpio17,
        gpio18,
        gpio19,
        gpio20,
        gpio21,
        gpio22,
        gpio23,
        gpio24,
        gpio25,
        gpio26,
        gpio27,
        gpio28,
        gpio29,
        swclk,
        swd,
    };
    constexpr static platform::reg_ptr_t base_addr = 0x4001c000;
};

struct qspi
{
    enum class pad : uint8_t
    {
        qspi_sclk = 0,
        qspi_sd0,
        qspi_sd1,
        qspi_sd2,
        qspi_sd3,
        qspi_ss,
    };
    constexpr static platform::reg_ptr_t base_addr = 0x40020000;
};

template<typename T>
concept pad_descriptor = requires
{
    typename T::pad;
    requires std::is_scoped_enum_v<typename T::pad>;

    // clang-format off
    { T::base_addr } -> std::same_as<const platform::reg_ptr_t&>;
    // clang-format on
};

enum class drive_strength : platform::reg_val_t
{
    strength_2mA = 0x00,
    strength_4mA = 0x01,
    strength_8mA = 0x02,
    strength_12mA = 0x03,
};

enum class slew_rate : platform::reg_val_t
{
    slow = 0,
    fast,
};

template<pad_descriptor T, typename T::pad specific_pad>
class pad
{
  private:
    // Calculate PAD registers offset
    consteval static platform::reg_ptr_t offset_addr(
      typename T::pad requested_pad)
    {
        return sizeof(platform::reg_val_t) +
               (std::to_underlying(requested_pad) *
                sizeof(platform::reg_val_t));
    }

  public:
    using pad_bits = platform::registers::gpio_pads_bits;
    using pad_reg =
      platform::rw_reg<T::base_addr, offset_addr(specific_pad), pad_bits>;

    constexpr static void output_enable()
    {
        pad_reg::reset_bit(pad_bits::od);
    }

    constexpr static void output_disable()
    {
        pad_reg::set_bit(pad_bits::od);
    }

    constexpr static void input_enable()
    {
        pad_reg::set_bit(pad_bits::ie);
    }

    constexpr static void input_disable()
    {
        pad_reg::reset_bit(pad_bits::ie);
    }

    constexpr static void set_drive_strength(drive_strength strength)
    {
        constexpr platform::reg_val_t drive_bits_mask =
          (1 << std::to_underlying(pad_bits::drive0)) |
          (1 << std::to_underlying(pad_bits::drive1));

        pad_reg::set_value((pad_reg::value() & ~drive_bits_mask) |
                           (std::to_underlying(strength)
                            << std::to_underlying(pad_bits::drive0)));
    }

    constexpr static void pull_up_enable()
    {
        pad_reg::set_bit(pad_bits::pue);
    }

    constexpr static void pull_up_disable()
    {
        pad_reg::reset_bit(pad_bits::pue);
    }

    constexpr static void pull_down_enable()
    {
        pad_reg::set_bit(pad_bits::pde);
    }

    constexpr static void pull_down_disable()
    {
        pad_reg::reset_bit(pad_bits::pde);
    }

    constexpr static void schmitt_trigger_enbale()
    {
        pad_reg::set_bit(pad_bits::schmitt);
    }

    constexpr static void schmitt_trigger_disable()
    {
        pad_reg::reset_bit(pad_bits::schmitt);
    }

    constexpr static void set_slew_date(slew_rate rate)
    {
        if (rate == slew_rate::slow) {
            pad_reg::reset_bit(pad_bits::slewfast);
        } else {
            pad_reg::set_bit(pad_bits::slewfast);
        }
    }
};

template<typename qspi::pad specific_pad>
using qspi_pad = pad<qspi, specific_pad>;
using qspi_sclk = qspi_pad<qspi::pad::qspi_sclk>;
using qspi_sd0 = qspi_pad<qspi::pad::qspi_sd0>;
using qspi_sd1 = qspi_pad<qspi::pad::qspi_sd1>;
using qspi_sd2 = qspi_pad<qspi::pad::qspi_sd2>;
using qspi_sd3 = qspi_pad<qspi::pad::qspi_sd3>;
using qspi_ss = qspi_pad<qspi::pad::qspi_ss>;

template<typename gpio::pad specific_pad>
using gpio_pad = pad<gpio, specific_pad>;

using gpio0 = gpio_pad<gpio::pad::gpio0>;
using gpio1 = gpio_pad<gpio::pad::gpio1>;
using gpio2 = gpio_pad<gpio::pad::gpio2>;
using gpio3 = gpio_pad<gpio::pad::gpio3>;
using gpio4 = gpio_pad<gpio::pad::gpio4>;
using gpio5 = gpio_pad<gpio::pad::gpio5>;
using gpio6 = gpio_pad<gpio::pad::gpio6>;
using gpio7 = gpio_pad<gpio::pad::gpio7>;
using gpio8 = gpio_pad<gpio::pad::gpio8>;
using gpio9 = gpio_pad<gpio::pad::gpio9>;
using gpio10 = gpio_pad<gpio::pad::gpio10>;
using gpio11 = gpio_pad<gpio::pad::gpio11>;
using gpio12 = gpio_pad<gpio::pad::gpio12>;
using gpio13 = gpio_pad<gpio::pad::gpio13>;
using gpio14 = gpio_pad<gpio::pad::gpio14>;
using gpio15 = gpio_pad<gpio::pad::gpio15>;
using gpio16 = gpio_pad<gpio::pad::gpio16>;
using gpio17 = gpio_pad<gpio::pad::gpio17>;
using gpio18 = gpio_pad<gpio::pad::gpio18>;
using gpio19 = gpio_pad<gpio::pad::gpio19>;
using gpio20 = gpio_pad<gpio::pad::gpio20>;
using gpio21 = gpio_pad<gpio::pad::gpio21>;
using gpio22 = gpio_pad<gpio::pad::gpio22>;
using gpio23 = gpio_pad<gpio::pad::gpio23>;
using gpio24 = gpio_pad<gpio::pad::gpio24>;
using gpio25 = gpio_pad<gpio::pad::gpio25>;
using gpio26 = gpio_pad<gpio::pad::gpio26>;
using gpio27 = gpio_pad<gpio::pad::gpio27>;
using gpio28 = gpio_pad<gpio::pad::gpio28>;
using gpio29 = gpio_pad<gpio::pad::gpio29>;
using swclk = gpio_pad<gpio::pad::swclk>;
using swd = gpio_pad<gpio::pad::swd>;

}

#endif
