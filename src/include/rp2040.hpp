#ifndef __RP2040_HPP__
#define __RP2040_HPP__

#include <cstdint>

#include "hwio.hpp"

namespace platform {
using reg_val_t = uint32_t;
using reg_ptr_t = uint32_t;

template<reg_ptr_t Base, reg_ptr_t Offset, typename BitsType = unsigned int>
using reg_base =
  hwio::volatile_reg<reg_ptr_t, reg_val_t, Base, Offset, BitsType>;

template<reg_ptr_t Base, reg_ptr_t Offset, typename BitsType = unsigned int>
using rw_reg = hwio::rw<reg_base<Base, Offset, BitsType>>;

template<reg_ptr_t Addr, typename BitsType = unsigned int>
using rw_reg_direct = hwio::rw<reg_base<Addr, 0, BitsType>>;

enum pins : uint8_t
{
    gpio0 = 0,
    gpio1,
    gpio2,
    gpio3,
    gpio4,
    gpio5,
    gpio6,
    gpio7,
    gpio8,
    gpio9,
    gpio10,
    gpio11,
    gpio12,
    gpio13,
    gpio14,
    gpio15,
    gpio16,
    gpio17,
    gpio18,
    gpio19,
    gpio20,
    gpio21,
    gpio22,
    gpio23,
    gpio24,
    gpio25,
    gpio26,
    gpio27,
    gpio28,
    gpio29,
};

namespace registers {
namespace addrs {
constexpr static platform::reg_ptr_t io_bank0_base = 0x40014000;
constexpr static platform::reg_ptr_t sio_base = 0xd0000000;
constexpr static platform::reg_ptr_t resets_base = 0x4000c000;

template<uint8_t pin_no>
struct gpio_ctrl_for
{
    constexpr static reg_ptr_t base = io_bank0_base;
    constexpr static reg_ptr_t offset =
      ((sizeof(platform::reg_val_t) * 2) * pin_no +
       sizeof(platform::reg_val_t));
    constexpr static reg_ptr_t addr = base + offset;
};

template<uint8_t pin_no>
struct gpio_status_for
{
    constexpr static reg_ptr_t base = io_bank0_base;
    constexpr static reg_ptr_t offset =
      ((sizeof(platform::reg_val_t) * 2) * pin_no);
};
}

template<uint8_t pin_no>
using gpio_ctrl = rw_reg_direct<addrs::gpio_ctrl_for<pin_no>::addr>;

template<uint8_t pin_no>
using gpio_status = rw_reg_direct<addrs::gpio_status_for<pin_no>::addr>;

using cpuid = rw_reg<addrs::sio_base, 0>;

using gpio_in = rw_reg<addrs::sio_base, 0x004>;
using gpio_hi_in = rw_reg<addrs::sio_base, 0x008>;

using gpio_out = rw_reg<addrs::sio_base, 0x010>;
using gpio_out_set = rw_reg<addrs::sio_base, 0x014>;
using gpio_out_clr = rw_reg<addrs::sio_base, 0x018>;
using gpio_out_xor = rw_reg<addrs::sio_base, 0x01c>;

using gpio_oe = rw_reg<addrs::sio_base, 0x020>;
using gpio_oe_set = rw_reg<addrs::sio_base, 0x024>;
using gpio_oe_clr = rw_reg<addrs::sio_base, 0x028>;
using gpio_oe_xor = rw_reg<addrs::sio_base, 0x02c>;

enum reset_bits : uint8_t
{
    adc = 0,
    busctrl,
    dma,
    i2c0,
    i2c1,
    io_bank0,
    io_qspi,
    jtag,
    pads_bank0,
    pads_qspi,
    pio0,
    pio1,
    pll_sys,
    pll_usb,
    pwm,
    rtc,
    spi0,
    spi1,
    syscfg,
    sysinfo,
    tbman,
    timer,
    uart0,
    uart1,
    usbctrl
};
using reset = rw_reg<addrs::resets_base, 0, reset_bits>;
using wdsel = rw_reg<addrs::resets_base, 0x4, reset_bits>;
using reset_done = rw_reg<addrs::resets_base, 0x8, reset_bits>;

}

}

#endif
