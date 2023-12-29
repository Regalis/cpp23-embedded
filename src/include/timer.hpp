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

#ifndef TIMER_HPP
#define TIMER_HPP

#include "rp2040.hpp"
#include <chrono>

namespace timer {

constexpr std::chrono::microseconds ticks_since_start()
{
    uint32_t hi = platform::timer::timerawh::value();
    uint32_t lo;
    do {
        lo = platform::timer::timerawl::value();
        uint32_t new_hi = platform::timer::timerawh::value();
        if (hi == new_hi) {
            break;
        }
        hi = new_hi;
    } while (true);
    return std::chrono::microseconds{(static_cast<uint64_t>(hi) << 32) | lo};
}

constexpr void delay(std::chrono::microseconds us)
{
    std::chrono::microseconds target_value = ticks_since_start() + us;
    while (ticks_since_start() < target_value) {
        // wait
    }
}

}

#endif
