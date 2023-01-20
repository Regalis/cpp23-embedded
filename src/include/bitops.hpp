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

#ifndef BITOPS_HPP
#define BITOPS_HPP

#include <concepts>
#include <type_traits>
#include <utility>

template<typename T>
concept strongly_typed_bit = std::is_scoped_enum_v<T>;

template<typename T>
concept weakly_typed_bit = std::integral<T>;

template<typename T>
concept valid_bit_position = weakly_typed_bit<T> || strongly_typed_bit<T>;

constexpr auto bit(const weakly_typed_bit auto& bit_position)
{
    return bit_position;
}

constexpr auto bit(const strongly_typed_bit auto& bit_position)
{
    return std::to_underlying(bit_position);
}

constexpr auto bit_value(const valid_bit_position auto&... bit_position)
{
    return ((1UL << bit(bit_position)) | ...);
}

constexpr auto bitwise_or(const std::semiregular auto&... value)
{
    return (value | ...);
}

constexpr auto bitwise_and(const std::semiregular auto&... value)
{
    return (value & ...);
}

constexpr auto bitwise_xor(const std::semiregular auto&... value)
{
    return (value ^ ...);
}

constexpr void set_bits(auto& lhs,
                        const valid_bit_position auto&... bit_position)
{
    lhs = bitwise_or(lhs, (bit_value(bit_position), ...));
}

constexpr void reset_bits(auto& lhs,
                          const valid_bit_position auto&... bit_position)
{
    lhs = lhs & ~(bitwise_or((bit_value(bit_position), ...)));
}

constexpr void toggle_bits(auto& lhs,
                           const valid_bit_position auto&... bit_position)
{
    lhs = lhs ^ bitwise_or((bit_value(bit_position), ...));
}

constexpr auto& read_bits(auto& lhs,
                          const valid_bit_position auto&... bit_position)
{
    return (lhs & bitwise_or((bit_value(bit_position), ...)));
}

constexpr auto bitmask(const valid_bit_position auto&... bit_positon)
{
    return bitwise_or((bit_value(bit_positon), ...));
}

#endif
