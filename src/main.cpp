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

#include <cstdint>

#include "clocks.hpp"
#include "delay.hpp"
#include "gpio.hpp"
#include "reset.hpp"
#include "rp2040.hpp"
#include "timer.hpp"

int main()
{
    clocks::init();
    clocks::watchdog_start(platform::xosc::frequency_khz);
    reset::release_subsystem_wait(reset::subsystems::io_bank0);
    gpio::pin<platform::pins::gpio25> led0;
    led0.function_select(gpio::functions::sio);
    led0.set_as_output();

    while (1) {
        for (int i = 0; i < 10; ++i) {
            led0.toggle();
            timer::delay_ms(500);
        }

        for (int i = 0; i < 20; ++i) {
            led0.toggle();
            timer::delay_ms(250);
        }

        for (int i = 0; i < 50; ++i) {
            led0.toggle();
            timer::delay_ms(100);
        }
    }
}
