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

#include "clocks.hpp"
#include "drivers/lcd/hd44780/hd44780.hpp"
#include "drivers/lcd/hd44780/interfaces/gpio4_bit.hpp"
#include "gpio.hpp"
#include "reset.hpp"
#include "rp2040.hpp"
#include "timer.hpp"

#include <chrono>
#include <string_view>

using namespace std::chrono_literals;
using namespace std::string_view_literals;
using namespace drivers::lcd;

int main()
{
    // We need watchdog to use timer::delay()
    clocks::init();
    clocks::watchdog_start(platform::xosc::frequency_khz);

    // Every peripheral is held in reset at power-up
    // We need to release io_bank0 from the reset state to be able to use GPIOs
    reset::release_subsystem_wait(reset::subsystems::io_bank0);

    gpio::pin<platform::pins::gpio25> led0;
    led0.function_select(gpio::functions::sio);
    led0.set_as_output();

    // Prepere descriptor for the selected interface
    constexpr auto descriptor =
      hd44780::interfaces::gpio4_bit{.register_select = platform::pins::gpio16,
                                     .enable = platform::pins::gpio17,
                                     .data4 = platform::pins::gpio18,
                                     .data5 = platform::pins::gpio19,
                                     .data6 = platform::pins::gpio20,
                                     .data7 = platform::pins::gpio21};

    // Define your LCD hardware layout
    constexpr auto configuration = hd44780::configuration{
      .columns = 16, .lines = 2, .font_size = hd44780::font::font_5x8};

    // Get type of the driver based on your descriptor and configuration
    using lcd =
      hd44780::hd44780<hd44780::interface_for<descriptor>, configuration>;

    timer::delay(500ms);

    // Initialize both MCU interface (in this case - the GPIOs) and the LCD
    // itself
    lcd::init();

    lcd::soft_puts("Hello world");
    lcd::cursor_goto(0, 1);
    lcd::soft_puts("blog.regalis.tech");

    while (true) {
        led0.toggle();
        timer::delay(250ms);
    }
}
