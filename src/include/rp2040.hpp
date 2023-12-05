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

template<reg_ptr_t Base,
         reg_ptr_t Offset,
         typename BitsType = unsigned int,
         hwio::hwio_region... Region>
using reg_base =
  hwio::volatile_reg<reg_ptr_t, reg_val_t, Base, Offset, BitsType, Region...>;

template<reg_ptr_t Base,
         reg_ptr_t Offset,
         typename BitsType = unsigned int,
         hwio::hwio_region... Region>
using ro_reg = hwio::ro<reg_base<Base, Offset, BitsType, Region...>>;

template<reg_ptr_t Base,
         reg_ptr_t Offset,
         typename BitsType = unsigned int,
         hwio::hwio_region... Region>
using rw_reg = hwio::rw<reg_base<Base, Offset, BitsType, Region...>>;

template<reg_ptr_t Addr,
         typename BitsType = unsigned int,
         hwio::hwio_region... Region>
using rw_reg_direct = hwio::rw<reg_base<Addr, 0, BitsType, Region...>>;

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
constexpr static platform::reg_ptr_t pll_sys_base = 0x40028000;
constexpr static platform::reg_ptr_t pll_usb_base = 0x4002c000;
constexpr static platform::reg_ptr_t xosc_base = 0x40024000;
constexpr static platform::reg_ptr_t clocks_base = 0x40008000;

// TODO: move to a dedicated header file
constexpr static platform::reg_ptr_t m0plus_vtor_offset = 0xed08;

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

using voltage_select =
  rw_reg<addrs::pads_qspi_base, 0x00, voltage_select_bits>;
using gpio_qspi_sclk = rw_reg<addrs::pads_qspi_base, 0x04, gpio_pads_bits>;
using gpio_qspi_sd0 = rw_reg<addrs::pads_qspi_base, 0x08, gpio_pads_bits>;
using gpio_qspi_sd1 = rw_reg<addrs::pads_qspi_base, 0x0c, gpio_pads_bits>;
using gpio_qspi_sd2 = rw_reg<addrs::pads_qspi_base, 0x10, gpio_pads_bits>;
using gpio_qspi_sd3 = rw_reg<addrs::pads_qspi_base, 0x14, gpio_pads_bits>;
using gpio_qspi_ss = rw_reg<addrs::pads_qspi_base, 0x18, gpio_pads_bits>;

}

namespace pll {

enum class cs_bits : reg_val_t
{
    refdiv0 = 0,
    refdiv1,
    refdiv2,
    refdiv3,
    refdiv4,
    refdiv5,
    bypass = 8,
    lock = 31,
};

enum class pwr_bits : reg_val_t
{
    pd = 0,
    dsmpd = 2,
    postdivpd = 3,
    vcopd = 5,
};

enum class fbdiv_int_bits : reg_val_t
{
    fbdiv_int0 = 0,
    fbdiv_int1,
    fbdiv_int2,
    fbdiv_int3,
    fbdiv_int4,
    fbdiv_int5,
    fbdiv_int6,
    fbdiv_int7,
    fbdiv_int8,
    fbdiv_int9,
    fbdiv_int10,
    fbdiv_int11,
};

enum class prim_bits : reg_val_t
{
    postdiv2_0 = 12,
    postdiv2_1,
    postdiv2_2,
    postdiv1_0 = 16,
    postdiv1_1,
    postdiv1_2,
};

using cs_region_refdiv = hwio::region<reg_val_t, 0, 6>;

// TODO: datasheet does not provide the name for this region
using fbdiv_int_region_value = hwio::region<reg_val_t, 0, 12>;

using prim_region_postdiv2 = hwio::region<reg_val_t, 12, 3>;
using prim_region_postdiv1 = hwio::region<reg_val_t, 16, 3>;

struct sys
{

    using cs =
      rw_reg<registers::addrs::pll_sys_base, 0x0, cs_bits, cs_region_refdiv>;

    using pwr = rw_reg<registers::addrs::pll_sys_base, 0x4, pwr_bits>;

    using fbdiv_int = rw_reg<registers::addrs::pll_sys_base,
                             0x8,
                             fbdiv_int_bits,
                             fbdiv_int_region_value>;

    using prim = rw_reg<registers::addrs::pll_sys_base,
                        0xc,
                        prim_bits,
                        prim_region_postdiv1,
                        prim_region_postdiv2>;
};

struct usb
{

    using cs =
      rw_reg<registers::addrs::pll_usb_base, 0x0, cs_bits, cs_region_refdiv>;

    using pwr = rw_reg<registers::addrs::pll_usb_base, 0x4, pwr_bits>;

    using fbdiv_int = rw_reg<registers::addrs::pll_usb_base,
                             0x8,
                             fbdiv_int_bits,
                             fbdiv_int_region_value>;

    using prim = rw_reg<registers::addrs::pll_usb_base,
                        0xc,
                        prim_bits,
                        prim_region_postdiv1,
                        prim_region_postdiv2>;
};

}

namespace xosc {

enum class ctrl_region_freq_range_values : reg_val_t
{
    RANGE_1_15_MHz = 0xaa0,
};

enum class ctrl_region_enable_values : reg_val_t
{
    DISABLE = 0xd1e,
    ENABLE = 0xfab,
};

using ctrl_region_freq_range =
  hwio::region<ctrl_region_freq_range_values, 0, 12>;
using ctrl_region_enbale = hwio::region<ctrl_region_enable_values, 12, 12>;

enum class status_bits : reg_val_t
{
    FREQ_RANGE0 = 0,
    FREQ_RANGE1,
    ENABLED = 12,
    BADWRITE = 24,
    STABLE = 31,
};

enum class status_region_freq_range_values : reg_val_t
{
    RANGE_1_15_MHz = 0x0,
};

using status_region_freq_range =
  hwio::region<status_region_freq_range_values, 0, 2>;

enum class dormant_region_values : reg_val_t
{
    DORMANT = 0x636f6d61,
    WAKE = 0x77616b65,
};

using dormant_region = hwio::region<dormant_region_values, 0, 32>;

enum class startup_bits : reg_val_t
{
    X4 = 20,
};

using startup_region_delay = hwio::region<reg_val_t, 0, 14>;

using count_region = hwio::region<reg_val_t, 0, 8>;

using ctrl = rw_reg<registers::addrs::xosc_base,
                    0x00,
                    reg_val_t,
                    ctrl_region_enbale,
                    ctrl_region_freq_range>;
using status = rw_reg<registers::addrs::xosc_base,
                      0x04,
                      status_bits,
                      status_region_freq_range>;
using dormant =
  rw_reg<registers::addrs::xosc_base, 0x08, reg_val_t, dormant_region>;
using startup = rw_reg<registers::addrs::xosc_base,
                       0x0c,
                       startup_bits,
                       startup_region_delay>;
using count =
  rw_reg<registers::addrs::xosc_base, 0x1c, reg_val_t, count_region>;

}

namespace clocks {

enum class clk_gpout_ctrl_bits : reg_val_t
{
    auxsrc0 = 5,
    auxsrc1,
    auxsrc2,
    auxsrc3,
    kill = 10,
    enable,
    dc50,
    phase0 = 16,
    phase1,
    nudge = 20
};

enum class clk_gpout_ctrl_region_auxsrc_values : reg_val_t
{
    clksrc_pll_sys = 0,
    clksrc_gpin0,
    clksrc_gpin1,
    clksrc_pll_usb,
    rosc_clksrc,
    xosc_clksrc,
    clk_sys,
    clk_usb,
    clk_adc,
    clk_rtc,
    clk_ref,
};

using clk_gpout_ctrl_region_auxsrc =
  hwio::region<clk_gpout_ctrl_region_auxsrc_values, 5, 4>;
using clk_gpout_ctrl_region_phase = hwio::region<reg_val_t, 16, 2>;

using clk_div_region_frac = hwio::region<reg_val_t, 0, 8>;
using clk_div_region_int = hwio::region<reg_val_t, 8, 24>;

using clk_gpout_div_region_frac = clk_div_region_frac;
using clk_gpout_div_region_int = clk_div_region_int;

enum class clk_ref_ctrl_bits
{
    src0 = 0,
    src1,
    auxsrc0 = 5,
    auxsrc1,
};

enum class clk_ref_ctrl_region_src_values : reg_val_t
{
    rosc_clksrc_ph = 0,
    clksrc_clk_ref_aux,
    xosc_clksrc,
};

enum class clk_ref_ctrl_region_auxsrc_values : reg_val_t
{
    clksrc_pll_usb = 0,
    clksrc_gpin0,
    clksrc_gpin1,
};

using clk_ref_ctrl_region_src =
  hwio::region<clk_ref_ctrl_region_src_values, 0, 2>;
using clk_ref_ctrl_region_auxsrc =
  hwio::region<clk_ref_ctrl_region_auxsrc_values, 5, 2>;

enum class clk_ref_div_bits : reg_val_t
{
    int0 = 8,
    int1,
};

using clk_ref_div_region_int = hwio::region<reg_val_t, 8, 2>;

enum class clk_sys_ctrl_bits : reg_val_t
{
    src = 0,
    auxsrc0 = 5,
    auxsrc1,
    auxsrc2,
};

enum class clk_sys_ctrl_region_src_values : reg_val_t
{
    clk_ref = 0,
    clksrc_clk_sys_aux,
};

enum class clk_sys_ctrl_region_auxsrc_values : reg_val_t
{
    clksrc_pll_sys = 0,
    clksrc_pll_usb,
    rosc_clksrc,
    xosc_clksrc,
    clksrc_gpin0,
    clksrc_gpin1,
};

using clk_sys_ctrl_region_src =
  hwio::region<clk_sys_ctrl_region_src_values, 0, 1>;
using clk_sys_ctrl_region_auxsrc =
  hwio::region<clk_sys_ctrl_region_auxsrc_values, 5, 3>;

enum class clk_sys_div_bits : reg_val_t
{
    frac0 = 0,
    frac1,
    frac2,
    frac3,
    frac4,
    frac5,
    frac6,
    frac7,
    int0 = 8,
    int1,
    int2,
    int3,
    int4,
    int5,
    int6,
    int7,
    int8,
    int9,
    int10,
    int11,
    int12,
    int13,
    int14,
    int15,
    int16,
    int17,
    int18,
    int19,
    int20,
    int21,
    int22,
    int23,
};

enum class clk_peri_ctrl_bits : reg_val_t
{
    auxsrc0 = 5,
    auxsrc1,
    auxsrc2,
    kill = 10,
    enable = 11,
};

enum class clk_peri_ctrl_region_auxsrc_values : reg_val_t
{
    clk_sys = 0,
    clksrc_pll_sys,
    clksrc_pll_usb,
    rosc_clksrc_ph,
    xosc_clksrc,
    clksrc_gpin0,
    clksrc_gpin1,
};

using clk_peri_ctrl_region_auxsrc =
  hwio::region<clk_peri_ctrl_region_auxsrc_values, 5, 3>;

enum class clk_usb_ctrl_bits : reg_val_t
{
    auxsrc0 = 5,
    auxsrc1,
    auxsrc2,
    kill = 10,
    enable,
    phase0 = 16,
    phase1,
    nudge = 20
};

enum class clk_usb_ctrl_region_auxsrc_values : reg_val_t
{
    clksrc_pll_usb = 0,
    clksrc_pll_sys,
    rosc_clksrc_ph,
    xosc_clksrc,
    clksrc_gpin0,
    clksrc_gpin1,
};

using clk_usb_ctrl_region_auxsrc =
  hwio::region<clk_usb_ctrl_region_auxsrc_values, 5, 3>;

enum class clk_usb_div_bits : reg_val_t
{
    int0 = 8,
    int1,
};

using clk_usb_div_region_int = hwio::region<reg_val_t, 8, 2>;

enum class clk_adc_ctrl_bits : reg_val_t
{
    auxsrc0 = 5,
    auxsrc1,
    auxsrc2,
    kill = 10,
    enable,
    phase0 = 16,
    phase1,
    nudge = 20,
};

enum class clk_adc_ctrl_region_auxsrc_values
{
    clksrc_pll_usb = 0,
    clksrc_pll_sys,
    rosc_clksrc_ph,
    xosc_clksrc,
    clksrc_gpin0,
    clksrc_gpin1,
};

using clk_adc_ctrl_region_auxsrc =
  hwio::region<clk_adc_ctrl_region_auxsrc_values, 5, 2>;

enum class clk_adc_div_bits : reg_val_t
{
    int0 = 8,
    int1,
};

using clk_adc_div_region_int = hwio::region<reg_val_t, 8, 2>;

enum class clk_sys_resus_ctrl_bits : reg_val_t
{
    timeout0 = 0,
    timeout1,
    timeout2,
    timeout3,
    timeout4,
    timeout5,
    timeout6,
    timeout7,
    enable = 8,
    frce = 12,
    clear = 16,
};

enum class clk_sys_resus_status_bits : reg_val_t
{
    resussed = 0,
};

enum class fc0_src_region_values : reg_val_t
{
    null = 0,
    pll_sys_clksrc_primary,
    pll_usb_clksrc_primary,
    rosc_clksrc,
    rosc_clksrc_ph,
    xosc_clksrc,
    clksrc_gpin0,
    clksrc_gpin1,
    clk_ref,
    clk_sys,
    clk_peri,
    clk_usb,
    clk_adc,
    clk_rtc,
};

enum class fc0_status_bits : reg_val_t
{
    pass = 0,
    done = 4,
    running = 8,
    waiting = 12,
    fail = 16,
    slow = 20,
    fast = 24,
    died = 28,
};

enum class wake_en0_bits : reg_val_t
{
    clk_sys_clocks = 0,
    clk_adc_adc,
    clk_sys_adc,
    clk_sys_busctrl,
    clk_sys_busfabric,
    clk_sys_dma,
    clk_sys_i2c0,
    clk_sys_i2c1,
    clk_sys_io,
    clk_sys_jtag,
    clk_sys_vreg_and_chip_reset,
    clk_sys_pads,
    clk_sys_pio0,
    clk_sys_pio1,
    clk_sys_pll_sys,
    clk_sys_pll_usb,
    clk_sys_psm,
    clk_sys_pwm,
    clk_sys_resets,
    clk_sys_rom,
    clk_sys_rosc,
    clk_rtc_rtc,
    clk_sys_rtc,
    clk_sys_sio,
    clk_peri_spi0,
    clk_sys_spi0,
    clk_peri_spi1,
    clk_sys_spi1,
    clk_sys_sram0,
    clk_sys_sram1,
    clk_sys_sram2,
    clk_sys_sram3,
};

enum class wake_en1_bits : reg_val_t
{
    clk_sys_sram4 = 0,
    clk_sys_sram5,
    clk_sys_syscfg,
    clk_sys_sysinfo,
    clk_sys_tbman,
    clk_sys_timer,
    clk_peri_uart0,
    clk_sys_uart0,
    clk_peri_uart1,
    clk_sys_uart1,
    clk_sys_usbctrl,
    clk_usb_usbctrl,
    clk_sys_watchdog,
    clk_sys_xip,
    clk_sys_xosc,
};

enum class intr_bits : reg_val_t
{
    clk_sys_resus = 0,
};

enum class inte_bits : reg_val_t
{
    clk_sys_resus = 0,
};

enum class intf_bits : reg_val_t
{
    clk_sys_resus = 0,
};

enum class ints_bits : reg_val_t
{
    clk_sys_resus = 0,
};

namespace detail {

template<uint8_t index>
struct clk_gpout
{
    using ctrl = rw_reg<registers::addrs::clocks_base,
                        (index * 12UL) + 0x00UL,
                        clk_gpout_ctrl_bits,
                        clk_gpout_ctrl_region_phase,
                        clk_gpout_ctrl_region_auxsrc>;
    using div = rw_reg<registers::addrs::clocks_base,
                       (index * 12UL) + 0x04UL,
                       unsigned int,
                       clk_gpout_div_region_int,
                       clk_gpout_div_region_frac>;
    using selected = ro_reg<registers::addrs::clocks_base,
                            (index * 12UL) + 0x08UL,
                            unsigned int>;
};

}

using clk_gpout0 = detail::clk_gpout<0>;
using clk_gpout1 = detail::clk_gpout<1>;
using clk_gpout2 = detail::clk_gpout<2>;
using clk_gpout3 = detail::clk_gpout<3>;

struct clk_ref
{
    using ctrl = rw_reg<registers::addrs::clocks_base,
                        0x30,
                        clk_ref_ctrl_bits,
                        clk_ref_ctrl_region_src,
                        clk_ref_ctrl_region_auxsrc>;
    using div = rw_reg<registers::addrs::clocks_base,
                       0x34,
                       clk_ref_div_bits,
                       clk_ref_div_region_int>;
    using selected = ro_reg<registers::addrs::clocks_base, 0x38, reg_val_t>;
};

struct clk_sys
{
    using ctrl = rw_reg<registers::addrs::clocks_base,
                        0x3c,
                        clk_sys_ctrl_bits,
                        clk_sys_ctrl_region_src,
                        clk_sys_ctrl_region_auxsrc>;
    using div = rw_reg<registers::addrs::clocks_base,
                       0x40,
                       clk_sys_div_bits,
                       clk_div_region_int,
                       clk_div_region_frac>;
    using selected = ro_reg<registers::addrs::clocks_base, 0x44, reg_val_t>;
};

struct clk_peri
{
    using ctrl = rw_reg<registers::addrs::clocks_base,
                        0x48,
                        clk_peri_ctrl_bits,
                        clk_peri_ctrl_region_auxsrc>;
    // TODO: The datasheet does not use this register...
    using div = rw_reg<registers::addrs::clocks_base,
                       0x4c,
                       clk_sys_div_bits,
                       clk_div_region_int,
                       clk_div_region_frac>;
    using selected = ro_reg<registers::addrs::clocks_base, 0x50, reg_val_t>;
};

struct clk_usb
{
    using ctrl = rw_reg<registers::addrs::clocks_base,
                        0x54,
                        clk_usb_ctrl_bits,
                        clk_usb_ctrl_region_auxsrc>;
    using div = rw_reg<registers::addrs::clocks_base,
                       0x58,
                       clk_usb_div_bits,
                       clk_usb_div_region_int>;
    using selected = ro_reg<registers::addrs::clocks_base, 0x5c, reg_val_t>;
};

struct clk_adc
{
    using ctrl = rw_reg<registers::addrs::clocks_base,
                        0x60,
                        clk_adc_ctrl_bits,
                        clk_adc_ctrl_region_auxsrc>;
    using div = rw_reg<registers::addrs::clocks_base,
                       0x64,
                       clk_adc_div_bits,
                       clk_adc_div_region_int>;
    using selected = ro_reg<registers::addrs::clocks_base, 0x68, reg_val_t>;
};

struct clk_rtc
{
    using ctrl = rw_reg<registers::addrs::clocks_base,
                        0x6c,
                        clk_adc_ctrl_bits,
                        clk_adc_ctrl_region_auxsrc>;
    using div = rw_reg<registers::addrs::clocks_base,
                       0x70,
                       clk_adc_div_bits,
                       clk_adc_div_region_int>;
    using selected = ro_reg<registers::addrs::clocks_base, 0x74, reg_val_t>;
};

struct clk_sys_resus
{
    using ctrl =
      rw_reg<registers::addrs::clocks_base, 0x78, clk_sys_resus_ctrl_bits>;
    using status =
      ro_reg<registers::addrs::clocks_base, 0x7c, clk_sys_resus_status_bits>;
};

// TODO: add fc0 registers

using wake_en0 = rw_reg<registers::addrs::clocks_base, 0xa0, wake_en0_bits>;
using wake_en1 = rw_reg<registers::addrs::clocks_base, 0xa4, wake_en1_bits>;
// TODO: add sleep_en0
// TODO: add sleep_en1

using intr = rw_reg<registers::addrs::clocks_base, 0xb8, intr_bits>;
using inte = rw_reg<registers::addrs::clocks_base, 0xbc, inte_bits>;
using intf = rw_reg<registers::addrs::clocks_base, 0xc0, intf_bits>;
using ints = rw_reg<registers::addrs::clocks_base, 0xc4, ints_bits>;

}

}

#endif
