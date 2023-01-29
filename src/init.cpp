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

#include <algorithm>
#include <cstdint>
#include <span>
#include <utility>

#include "delay.hpp"
#include "gpio.hpp"
#include "reset.hpp"
#include "rp2040.hpp"

int main();

extern "C"
{

    void __regalis_init()
    {
        extern std::uint8_t __data_start;
        extern std::uint8_t __data_end;
        extern std::uint8_t __data_lma_start;

        extern std::uint8_t __bss_start;
        extern std::uint8_t __bss_end;

        const std::size_t data_size =
          static_cast<std::size_t>(&__data_end - &__data_start);

        // Copy .data section from FLASH to SRAM
        std::copy(
          &__data_lma_start, &__data_lma_start + data_size, &__data_start);

        // Zero-fill .bss section
        std::fill(&__bss_start, &__bss_end, 0);

        main();
        std::unreachable();
    }
}

