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
#include "gpio.hpp"
#include "reset.hpp"
#include "timer.hpp"
#include "uart.hpp"

using namespace std::chrono_literals;

int main()
{
    clocks::init();
    clocks::watchdog_start(platform::xosc::frequency_khz);

    // Every peripheral is held in reset at power-up
    // We need to release io_bank0 from the reset state to be able to use GPIOs
    // In this case - we need to use io_bank0 to change the function of the RX,
    // TX pins
    reset::release_subsystem_wait(reset::subsystems::io_bank0);

    gpio::pin<platform::pins::gpio0> tx;
    gpio::pin<platform::pins::gpio1> rx;
    rx.function_select(gpio::functions::uart);
    tx.function_select(gpio::functions::uart);
    uart::uart0::init(9600);

    using led = gpio::pin<platform::pins::gpio25>;
    led::function_select(gpio::functions::sio);
    led::set_as_output();

    uart::uart0::puts("Hello world :-)\r\n");
    uart::uart0::puts("-> send '1' to turn on the LED\r\n");
    uart::uart0::puts("-> send '0' to turn off the LED\r\n");

    while (true) {
        if (char command = uart::uart0::getc(); command == '1') {
            led::set_high();
            uart::uart0::puts("LED ON\r\n");
        } else if (command == '0') {
            led::set_low();
            uart::uart0::puts("LED OFF\n\r");
        } else {
            uart::uart0::puts("Unknown command :(\r\n");
        }
    }
}
