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
#ifndef HWIO_HPP
#define HWIO_HPP

#include <concepts>
#include <cstdint>
#include <type_traits>
#include <utility>

namespace hwio {

template<typename RegPtrType,
         typename RegValueType,
         RegPtrType BaseAddr,
         RegPtrType Offset = 0,
         typename BitsType = unsigned int>
class reg
{
  public:
    using reg_value_t = RegValueType;
    using reg_addr_t = std::decay_t<std::remove_pointer_t<RegPtrType>>;
    using reg_ptr_t = RegPtrType*;
    using bits_t = BitsType;

    constexpr static reg_addr_t base = BaseAddr;
    constexpr static reg_addr_t offset = Offset;
    constexpr static reg_addr_t addr = (base + offset);

    constexpr static reg_ptr_t ptr()
    {
        return reinterpret_cast<reg_ptr_t>(addr);
    }

    constexpr static auto& ref()
    {
        return *ptr();
    }

    constexpr static const auto& cref()
    {
        return *ptr();
    }
};

template<typename RegPtrType,
         typename RegValueType,
         RegPtrType BaseAddr,
         RegPtrType Offset = 0,
         typename BitsType = unsigned int>
using volatile_reg =
  reg<volatile RegPtrType, RegValueType, BaseAddr, Offset, BitsType>;

template<typename T>
concept hwio_reg = requires
{
    typename T::reg_value_t;
    typename T::reg_addr_t;
    typename T::reg_ptr_t;
    typename T::bits_t;

    // clang-format off
    { T::base } -> std::same_as<const typename T::reg_addr_t&>;
    { T::offset } -> std::same_as<const typename T::reg_addr_t&>;
    { T::addr } -> std::same_as<const typename T::reg_addr_t&>;
    // clang-format on
};

template<typename T>
concept strong_bits = requires(T t)
{
    std::to_underlying(t);
};

template<typename T>
class ro
{
  public:
    constexpr static typename T::reg_value_t value()
    {
        return T::cref();
    }

    constexpr static typename T::reg_value_t get_bit(typename T::bits_t bit_no)
    {
        return T::cref() & (1 << bit_no);
    }

    constexpr static typename T::reg_value_t get_bits_with_mask(
      typename T::reg_value_t mask)
    {
        return T::cref() & mask;
    }
};

template<typename T>
class wo
{
  public:
    constexpr static void set_bit(std::integral auto bit_no)
    {
        T::ref() = T::ref() | (1 << bit_no);
    }

    constexpr static void set_bit(strong_bits auto bit_no)
    {
        set_bit(std::to_underlying(bit_no));
    }

    constexpr static void set_bits(auto... bit_no)
    {
        T::ref() = T::ref() | ((1 << bit_no) | ...);
    }

    constexpr static void set_bits(strong_bits auto... bit_no)
    {
        T::ref() = T::ref() | ((1 << std::to_underlying(bit_no)) | ...);
    }

    constexpr static void set_bits(std::integral auto... bit_no)
    {
        T::ref() = T::ref() | ((1 << bit_no) | ...);
    }

    constexpr static void reset_bit(std::integral auto bit_no)
    {
        T::ref() = T::ref() & ~(1 << bit_no);
    }

    constexpr static void reset_bit(strong_bits auto bit_no)
    {
        T::ref() = T::ref() & ~(1 << std::to_underlying(bit_no));
    }

    constexpr static void reset_bits(strong_bits auto... bit_no)
    {
        T::ref() = T::ref() & ~((1 << std::to_underlying(bit_no)) | ...);
    }

    constexpr static void toggle(typename T::bits_t bit_no)
    {
        T::ref() = T::ref() ^ (1 << bit_no);
    }

    constexpr static void set_value(typename T::reg_value_t value)
    {
        T::ref() = value;
    }

    constexpr auto& operator=(typename T::reg_value_t value) const
    {
        set_value(value);
        return *this;
    }
};

template<typename T>
class rw : public ro<T>, public wo<T>
{
};

}
#endif
