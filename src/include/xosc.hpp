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

#ifndef XOSC_HPP
#define XOSC_HPP

#include "rp2040.hpp"

class xosc
{
  public:
    using ctrl = platform::xosc::ctrl;
    using status = platform::xosc::status;
    using startup = platform::xosc::startup;
    static constexpr uint32_t xosc_freq = platform::xosc::frequency_khz;
    static constexpr uint32_t startup_delay = (((xosc_freq + 128) / 256) * 64);

    static constexpr void init()
    {
        using namespace platform::xosc;

        ctrl::update_regions(regions::ctrl::freq_range::range_1_15_mhz);
        startup::update_regions(startup_region_delay{startup_delay});
        ctrl::update_regions(regions::ctrl::enable::enable);

        while (!status::get_bit(status_bits::stable)) {
            // wait for xosc to become stable
        }
    }

    static constexpr void disable()
    {
        using namespace platform::xosc;
        ctrl::update_regions(regions::ctrl::enable::disable);
        while (status::get_bit(status_bits::stable)) {
            // wait for clock to become unstable (stable flag needs to go away)
        }
    }
};

#endif
