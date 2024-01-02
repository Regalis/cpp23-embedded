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
#include "rp2040.hpp"
#include "timer.hpp"
#include "uart.hpp"
#include "utils.hpp"

#include <ranges>

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
    led0.function_select(gpio::functions::sio);
    led0.set_as_output();

    gpio::pin<platform::pins::gpio15> servo;

    // Change PIN function to PWM
    servo.function_select(gpio::functions::pwm);

    // Get the right PWM instance (called a slice) for the specified GPIO pin
    auto pwm_slice = pwm::from_gpio(servo);

    // Get the right channel for the specified GPIO pin
    using pwm_channel = pwm::channel_for_pin<servo.pin_no>;

    // Obtain frequency configuration for 50Hz
    constexpr auto freq_config = pwm::get_frequency_config_for(50);
    pwm_slice.set_frequency(freq_config);

    // 100% duty cycle = counter wrap value + 1
    constexpr uint32_t max_channel_value = freq_config.wrap + 1;

    // Calculate channel value for angle = 0 (duty cycle 1000us = 1ms)
    // Linear interpolation:
    // Input value: 1000 (1ms)
    // Input range (duty cycle in us): [0, 20000] = [0ms, 20ms]
    // Output range [0, max_channel_value]
    constexpr uint16_t servo_0 =
      utils::map(1000, 0, 20000, 0, max_channel_value);

    // Calculate channel value for angle = 180 (duty cycle 2500us = 2.5ms)
    // Linear interpolation:
    // Input value: 2500 (2.5ms)
    // Input range (duty cycle in us): [0, 20000] = [0ms, 20ms]
    // Output range [0, max_channel_value]
    constexpr uint16_t servo_180 =
      utils::map(2500, 0, 20000, 0, max_channel_value);

    // NOTE! All the above calculations will be done at compile time!

    // Enable signal generation
    pwm_slice.enable();

    constexpr std::array servo_positions{0, 45, 90, 135, 180};

    while (true) {
        led0.toggle();
        timer::delay(1s);
        std::ranges::for_each(servo_positions, [&](auto angle) {
            uint16_t channel_value =
              utils::map(angle, 0, 180, servo_0, servo_180);
            led0.toggle();
            pwm_slice.set_channel_levels(pwm_channel{channel_value});
            timer::delay(3s);
        });
        timer::delay(1s);
    }
}
