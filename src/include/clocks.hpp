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

#ifndef CLOCKS_HPP
#define CLOCKS_HPP

#include "reset.hpp"
#include "rp2040.hpp"
#include "xosc.hpp"

#include <concepts>
#include <cstdint>
#include <utility>

namespace clocks {

template<typename T>
concept valid_clock = requires {
    typename T::ctrl_bits;
    typename T::ctrl;
    typename T::selected;
    typename T::auxsrc;
    typename T::region_auxsrc;
    // clang-format off
    { T::has_glitchless_mux } -> std::same_as<const bool&>;
    // clang-format on
};

template<typename T>
concept clock_with_glitchless_mux = requires {
    requires T::has_glitchless_mux == true;
    typename T::div_bits;
    typename T::region_src;
    typename T::src;
};

template<typename T>
concept clock_without_glitchless_mux = !clock_with_glitchless_mux<T>;

template<typename T>
concept valid_clock_with_glitchless_mux =
  valid_clock<T> && clock_with_glitchless_mux<T>;

template<typename T>
concept valid_clock_without_glitchless_mux =
  valid_clock<T> && clock_without_glitchless_mux<T>;

namespace detail {
template<valid_clock Clock>
class clock_base
{};

template<valid_clock_with_glitchless_mux Clock>
class clock_base<Clock>
{};

template<valid_clock_without_glitchless_mux Clock>
class clock_base<Clock>
{};
}

template<valid_clock Clock>
class clock
{
  public:
    using descriptor = Clock;

    static constexpr void stop()
    {
        descriptor::ctrl::reset_bits(descriptor::ctrl_bits::enable);
    }

    static constexpr void start()
    {
        descriptor::ctrl::set_bits(descriptor::ctrl_bits::enable);
    }

    static constexpr bool has_glitchless_mux()
    {
        return descriptor::has_glitchless_mux;
    }

    static constexpr uint32_t calculate_divisor(uint32_t src_freq,
                                                uint32_t freq)
    {
        // TODO: this fragment requires more in-depth analysis
        // Ref:
        // https://github.com/raspberrypi/pico-sdk/blob/6a7db34ff63345a7badec79ebea3aaef1712f374/src/rp2_common/hardware_clocks/clocks.c#L57
        return static_cast<uint32_t>(
          (static_cast<uint64_t>(src_freq)
           << std::to_underlying(Clock::div_bits::int0)) /
          freq);
    }

    template<valid_clock_with_glitchless_mux C = Clock>
    static constexpr void switch_away_from_aux_source()
    {
        Clock::ctrl::clear_regions(typename Clock::region_src{});
    }

    template<valid_clock_with_glitchless_mux C = Clock>
    static constexpr bool is_selected(C::src src)
    {
        return Clock::selected::get_bit(std::to_underlying(src));
    }

    template<valid_clock_with_glitchless_mux C = Clock>
    static constexpr bool configure(C::src src,
                                    Clock::auxsrc auxsrc,
                                    uint32_t src_freq,
                                    uint32_t freq)
    {
        if (freq > src_freq) {
            return false;
        }
        uint32_t div = calculate_divisor(src_freq, freq);

        if (div > Clock::div::value()) {
            Clock::div::set_value(div);
        }

        Clock::ctrl::clear_regions(typename Clock::region_src{});
        while (!Clock::selected::get_bit(0)) {
            // wait
        }

        Clock::ctrl::update_regions(typename Clock::region_auxsrc{auxsrc});
        Clock::ctrl::update_regions(typename Clock::region_src{src});
        while (!is_selected(src)) {
            // wait
        }

        Clock::div::set_value(div);
        return true;
    }

    template<valid_clock_without_glitchless_mux C = Clock>
    static constexpr bool configure(Clock::auxsrc auxsrc,
                                    uint32_t src_freq,
                                    uint32_t freq)
    {
        if (freq > src_freq) {
            return false;
        }
        uint32_t div = calculate_divisor(src_freq, freq);
        if (div > Clock::div::value()) {
            Clock::div::set_value(div);
        }

        stop();

        // TODO: can we calculate this based on real values?
        // For now, let's be safe - asssume maximum frequency...
        volatile uint32_t delay_counter = ((125'000 / 48'000) + 1) * 3;
        // wait for ENABLE propagation
        while (delay_counter) {
            delay_counter = delay_counter - 1;
        }

        Clock::ctrl::update_regions(typename Clock::region_auxsrc{auxsrc});
        start();
        Clock::div::set_value(div);

        return true;
    }
};

// TODO: use strong types (something like freq::mhz, freq::hz)
template<typename P>
constexpr void pll_init(uint32_t refdiv,
                        uint32_t vco_freq,
                        uint32_t post_div,
                        uint32_t post_div2)
{
    namespace pll = platform::pll;
    const uint32_t ref_freq = platform::xosc::frequency_khz * 1000 / refdiv;
    const uint32_t fbdiv = vco_freq / ref_freq;
    reset::reset_subsystem(P::reset_bit);
    reset::release_subsystem_wait(P::reset_bit);

    P::cs::update_regions(pll::cs_region_refdiv{refdiv});
    P::fbdiv_int::update_regions(pll::fbdiv_int_region_value{fbdiv});

    P::pwr::reset_bits(pll::pwr_bits::pd, pll::pwr_bits::vcopd);

    while (!P::cs::get_bit(pll::cs_bits::lock)) {
        // wait for PLL to lock
    }

    P::prim::update_regions(pll::prim_region_postdiv1{post_div},
                            pll::prim_region_postdiv2{post_div2});

    P::pwr::reset_bits(pll::pwr_bits::postdivpd);
}

void init()
{
    using namespace platform::clocks;
    using namespace board::pll;
    using namespace board::clocks;

    xosc::init();

    clock<clk_sys>::switch_away_from_aux_source();
    while (clk_sys::selected::value() != 0x01) {
        // wait
    }

    clock<clk_ref>::switch_away_from_aux_source();
    while (clk_ref::selected::value() != 0x01) {
        // wait
    }

    pll_init<platform::pll::sys>(common_refdiv,
                                 pll_sys_vco_freq_khz * 1000,
                                 pll_sys_postdiv1,
                                 pll_sys_postdiv2);
    pll_init<platform::pll::usb>(common_refdiv,
                                 pll_usb_vco_freq_khz * 1000,
                                 pll_usb_postdiv1,
                                 pll_usb_postdiv2);

    clock<clk_ref>::configure(
      clk_ref::src::xosc_clksrc,
      clk_ref::auxsrc::clksrc_pll_usb, // ignored, we use xosc instead of
                                       // auxsrc
      platform::xosc::frequency_khz * 1000,
      platform::xosc::frequency_khz * 1000);

    clock<clk_sys>::configure(clk_sys::src::clksrc_clk_sys_aux,
                              clk_sys::auxsrc::clksrc_pll_sys,
                              sys_clk_hz,
                              sys_clk_hz);
    clock<clk_usb>::configure(
      clk_usb::auxsrc::clksrc_pll_usb, usb_clk_hz, usb_clk_hz);
    clock<clk_adc>::configure(
      clk_adc::auxsrc::clksrc_pll_usb, usb_clk_hz, usb_clk_hz);
    clock<clk_rtc>::configure(
      clk_rtc::auxsrc::clksrc_pll_usb, usb_clk_hz, rtc_clock_hz);
    clock<clk_peri>::configure(
      clk_peri::auxsrc::clk_sys, peri_clk_hz, peri_clk_hz);
}

/**
 * The watchdog reference clock, clk_tick, is driven from clk_ref. Ideally
 * clk_ref will be configured to use the Crystal Oscillator so that it provides
 * an accurate reference clock. The reference clock is divided internally to
 * generate a tick (nominally 1μs) to use as the watchdog tick.
 */
constexpr void watchdog_start(
  uint32_t clk_ref_frequency_khz = board::clocks::rosc_clock_hz / 1000)
{
    platform::watchdog::tick::update_regions(
      platform::watchdog::tick_region_cycles{clk_ref_frequency_khz / 1000});
    platform::watchdog::tick::set_bits(platform::watchdog::tick_bits::enable);
}

}

#endif
