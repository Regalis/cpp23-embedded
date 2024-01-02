/*
 *
 * Copyright (C) 2023-2024 Patryk Jaworski (blog.regalis.tech)
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
#include "gpio.hpp"
#include "pwm.hpp"
#include "reset.hpp"
#include "timer.hpp"
#include "uart.hpp"

using namespace std::chrono_literals;

int main()
{
    clocks::init();
    clocks::watchdog_start(platform::xosc::frequency_khz);

    // For GPIO
    reset::release_subsystem_wait(reset::subsystems::io_bank0);
    // For PWM
    reset::release_subsystem_wait(reset::subsystems::pwm);

    gpio::pin<platform::pins::gpio25> led0;

    // Change PIN function to PWM
    led0.function_select(gpio::functions::pwm);

    // Get the right PWM instance (called a slice) for the specified GPIO pin
    auto pwm_slice = pwm::from_gpio(led0);
    auto pwm_channel = pwm::channel::from_gpio(led0);
    using pwm_channel_t = decltype(pwm_channel);

    // Set the prescaler
    pwm_slice.set_clkdiv(100);

    // Set the overflow value
    pwm_slice.set_wrap(100);

    // Enable signal generation
    pwm_slice.enable();

    constexpr unsigned short duty_max = 101;

    while (true) {
        pwm_slice.set_channel_levels(pwm_channel_t{0});
        led0.function_select(gpio::functions::pwm);
        for (unsigned short i = 0; i <= duty_max; ++i) {
            pwm_slice.set_channel_levels(pwm_channel_t{i});
            timer::delay(20ms);
        }
        for (unsigned short i = 0; i <= duty_max; ++i) {
            pwm_slice.set_channel_levels(pwm_channel_t{duty_max - i});
            timer::delay(20ms);
        }
        timer::delay(1s);
    }
}
