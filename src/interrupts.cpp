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

#include <cstdint>

#include "delay.hpp"
#include "gpio.hpp"
#include "reset.hpp"

#include "init.hpp"

using interrupt_handler_t = std::uintptr_t;

extern "C" void default_isr()
{
    using namespace platform;
    reset::release_subsystem_wait(reset::subsystems::io_bank0);
    gpio::pin<pins::gpio25> led0;
    led0.function_select(gpio::functions::sio);
    led0.set_as_output();
    delay(0x100000 / 8);
    while (true) {
        led0.toggle();
        delay(0x100000 / 8);
    }
}

extern "C" void nmi_isr()
{
    while (1) {
    }
}

extern "C" void svcall_isr()
{
    while (1) {
    }
}

extern "C" void pendsv_isr()
{
    while (1) {
    }
}

extern "C" void systick_isr()
{
    while (1) {
    }
}

extern "C" void hardfault_isr()
{
    while (1) {
    }
}

constexpr interrupt_handler_t irq_t(const auto ptr)
{
    return reinterpret_cast<interrupt_handler_t>(ptr);
}

extern std::uint32_t __stack_pointer;

struct __attribute__((packed)) vector_table
{
    uint32_t* stack_pointer = &__stack_pointer;
    interrupt_handler_t reset_handler = irq_t(__regalis_init);
    interrupt_handler_t isr_nmi = irq_t(nmi_isr);
    interrupt_handler_t isr_hardfault = irq_t(hardfault_isr);
    uint32_t invalid0[7] = {};
    interrupt_handler_t isr_svcall = irq_t(svcall_isr);
    uint32_t invalid1[2] = {};
    interrupt_handler_t isr_pendsv = irq_t(pendsv_isr);
    interrupt_handler_t isr_systick = irq_t(systick_isr);
    interrupt_handler_t isr_irq0 = irq_t(default_isr);
    interrupt_handler_t isr_irq1 = irq_t(default_isr);
    interrupt_handler_t isr_irq2 = irq_t(default_isr);
    interrupt_handler_t isr_irq3 = irq_t(default_isr);
    interrupt_handler_t isr_irq4 = irq_t(default_isr);
    interrupt_handler_t isr_irq5 = irq_t(default_isr);
    interrupt_handler_t isr_irq6 = irq_t(default_isr);
    interrupt_handler_t isr_irq7 = irq_t(default_isr);
    interrupt_handler_t isr_irq8 = irq_t(default_isr);
    interrupt_handler_t isr_irq9 = irq_t(default_isr);
    interrupt_handler_t isr_irq10 = irq_t(default_isr);
    interrupt_handler_t isr_irq11 = irq_t(default_isr);
    interrupt_handler_t isr_irq12 = irq_t(default_isr);
    interrupt_handler_t isr_irq13 = irq_t(default_isr);
    interrupt_handler_t isr_irq14 = irq_t(default_isr);
    interrupt_handler_t isr_irq15 = irq_t(default_isr);
    interrupt_handler_t isr_irq16 = irq_t(default_isr);
    interrupt_handler_t isr_irq17 = irq_t(default_isr);
    interrupt_handler_t isr_irq18 = irq_t(default_isr);
    interrupt_handler_t isr_irq19 = irq_t(default_isr);
    interrupt_handler_t isr_irq20 = irq_t(default_isr);
    interrupt_handler_t isr_irq21 = irq_t(default_isr);
    interrupt_handler_t isr_irq22 = irq_t(default_isr);
    interrupt_handler_t isr_irq23 = irq_t(default_isr);
    interrupt_handler_t isr_irq24 = irq_t(default_isr);
    interrupt_handler_t isr_irq25 = irq_t(default_isr);
    interrupt_handler_t isr_irq26 = irq_t(default_isr);
    interrupt_handler_t isr_irq27 = irq_t(default_isr);
    interrupt_handler_t isr_irq28 = irq_t(default_isr);
    interrupt_handler_t isr_irq29 = irq_t(default_isr);
    interrupt_handler_t isr_irq30 = irq_t(default_isr);
    interrupt_handler_t isr_irq31 = irq_t(default_isr);
};

const volatile __attribute__((section("__vector_table")))
vector_table default_vtable{};
