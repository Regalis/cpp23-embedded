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

#ifndef RP2040_HPP
#define RP2040_HPP

#include <cstdint>

#include "bitops.hpp"
#include "hwio.hpp"

namespace platform {
using reg_val_t = uint32_t;
using reg_ptr_t = uint32_t;

template<reg_ptr_t Base, reg_ptr_t Offset, typename BitsType = unsigned int>
using reg_base =
  hwio::volatile_reg<reg_ptr_t, reg_val_t, Base, Offset, BitsType>;

template<reg_ptr_t Base, reg_ptr_t Offset, typename BitsType = unsigned int>
using ro_reg = hwio::ro<reg_base<Base, Offset, BitsType>>;

template<reg_ptr_t Base, reg_ptr_t Offset, typename BitsType = unsigned int>
using rw_reg = hwio::rw<reg_base<Base, Offset, BitsType>>;

template<reg_ptr_t Addr, typename BitsType = unsigned int>
using rw_reg_direct = hwio::rw<reg_base<Addr, 0, BitsType>>;

enum class pins : platform::reg_val_t
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
constexpr static platform::reg_ptr_t xip_base = 0x10000000;
constexpr static platform::reg_ptr_t xip_ssi_base = 0x18000000;
constexpr static platform::reg_ptr_t sio_base = 0xd0000000;
constexpr static platform::reg_ptr_t resets_base = 0x4000c000;
constexpr static platform::reg_ptr_t pads_qspi_base = 0x40020000;
constexpr static platform::reg_ptr_t io_bank0_base = 0x40014000;
constexpr static platform::reg_ptr_t ppb_base = 0xe0000000;

// TODO: move to a dedicated header file
constexpr static platform::reg_ptr_t m0plus_vtor_offset = 0x0000ed08;

template<platform::pins pin_no>
struct gpio_ctrl_for
{
    constexpr static reg_ptr_t base = io_bank0_base;
    constexpr static reg_ptr_t offset =
      ((sizeof(platform::reg_val_t) * 2) * bit_pos(pin_no) +
       sizeof(platform::reg_val_t));
    constexpr static reg_ptr_t addr = base + offset;
};

template<platform::pins pin_no>
struct gpio_status_for
{
    constexpr static reg_ptr_t base = io_bank0_base;
    constexpr static reg_ptr_t offset =
      ((sizeof(platform::reg_val_t) * 2) * bit_pos(pin_no));
};
}

template<platform::pins pin_no>
using gpio_ctrl = rw_reg_direct<addrs::gpio_ctrl_for<pin_no>::addr>;

template<platform::pins pin_no>
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

enum class reset_bits : platform::reg_val_t
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

namespace ssi {
enum class ssienr_bits : reg_val_t
{
    ssi_en = 0,
};

enum class ctrlr0_bits : reg_val_t
{
    dfs0 = 0,
    dsf1,
    dfs2,
    dfs4,
    frf0,
    frf1,
    scph,
    scpol,
    tmod0,
    tmod1,
    slv_oe,
    srl,
    cfs0,
    cfs1,
    cfs2,
    cfs3,
    dfs_32_0,
    dfs_32_1,
    dfs_32_2,
    dfs_32_3,
    dfs_32_4,
    spi_frf0,
    spi_frf1,
    sste = 24
};

enum class spi_ctrlr0_bits : reg_val_t
{
    trans_type0 = 0,
    trans_type1,
    addr_l0,
    addr_l1,
    addr_l2,
    addr_l3,
    inst_l0 = 8,
    inst_l1,
    wait_cycles0 = 11,
    wait_cycles1,
    wait_cycles2,
    wait_cycles3,
    wait_cycles4,
    spi_ddr_en,
    inst_ddr_en,
    spi_rxds_en,
    xip_cmd0 = 24,
    xip_cmd1,
    xip_cmd2,
    xip_cmd3,
    xip_cmd4,
    xip_cmd5,
    xip_cmd6,
    xip_cmd7,
};

enum class sr_bits : reg_val_t
{
    busy = 0,
    tfnf,
    tfe,
    rfne,
    rff,
    txt,
    dcol,
};

using ctrlr0 = rw_reg<addrs::xip_ssi_base, 0x00, ctrlr0_bits>;
using ctrlr1 = rw_reg<addrs::xip_ssi_base, 0x04>;
using ssienr = rw_reg<addrs::xip_ssi_base, 0x08>;
using mwcr = rw_reg<addrs::xip_ssi_base, 0x0c>;
using ser = rw_reg<addrs::xip_ssi_base, 0x10>;
using baudr = rw_reg<addrs::xip_ssi_base, 0x14>;
using txftlr = rw_reg<addrs::xip_ssi_base, 0x18>;
using rxftlr = rw_reg<addrs::xip_ssi_base, 0x1c>;
using txflr = rw_reg<addrs::xip_ssi_base, 0x20>;
using rxflr = rw_reg<addrs::xip_ssi_base, 0x24>;
using sr = rw_reg<addrs::xip_ssi_base, 0x28, sr_bits>;
using imr = rw_reg<addrs::xip_ssi_base, 0x2c>;
using isr = rw_reg<addrs::xip_ssi_base, 0x30>;
using risr = rw_reg<addrs::xip_ssi_base, 0x34>;
using txoicr = rw_reg<addrs::xip_ssi_base, 0x38>;
using rxoicr = rw_reg<addrs::xip_ssi_base, 0x3c>;
using rxuicr = rw_reg<addrs::xip_ssi_base, 0x40>;
using msticr = rw_reg<addrs::xip_ssi_base, 0x44>;
using icr = rw_reg<addrs::xip_ssi_base, 0x48>;
using dmacr = rw_reg<addrs::xip_ssi_base, 0x4c>;
using dmatdlr = rw_reg<addrs::xip_ssi_base, 0x50>;
using dmardlr = rw_reg<addrs::xip_ssi_base, 0x54>;
using idr = rw_reg<addrs::xip_ssi_base, 0x58>;
using ssi_version_id = rw_reg<addrs::xip_ssi_base, 0x5c>;
using dr0 = rw_reg<addrs::xip_ssi_base, 0x60>;
using rx_sample_dly = rw_reg<addrs::xip_ssi_base, 0xf0>;
using spi_ctrlr0 = rw_reg<addrs::xip_ssi_base, 0xf4, spi_ctrlr0_bits>;
using txd_drive_edge = rw_reg<addrs::xip_ssi_base, 0xf8>;
}

enum class gpio_pads_bits : reg_val_t
{
    slewfast = 0,
    schmitt,
    pde,
    pue,
    drive0,
    drive1,
    ie,
    od,
};

enum class voltage_select_bits : reg_val_t
{
    voltage = 0,
};

using volate_select = rw_reg<addrs::pads_qspi_base, 0x00, voltage_select_bits>;
using gpio_qspi_sclk = rw_reg<addrs::pads_qspi_base, 0x04, gpio_pads_bits>;
using gpio_qspi_sd0 = rw_reg<addrs::pads_qspi_base, 0x08, gpio_pads_bits>;
using gpio_qspi_sd1 = rw_reg<addrs::pads_qspi_base, 0x0c, gpio_pads_bits>;
using gpio_qspi_sd2 = rw_reg<addrs::pads_qspi_base, 0x10, gpio_pads_bits>;
using gpio_qspi_sd3 = rw_reg<addrs::pads_qspi_base, 0x14, gpio_pads_bits>;
using gpio_qspi_ss = rw_reg<addrs::pads_qspi_base, 0x18, gpio_pads_bits>;

}

}

#endif
