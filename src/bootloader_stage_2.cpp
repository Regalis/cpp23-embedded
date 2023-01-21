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

#include "bitops.hpp"
#include "pads.hpp"
#include "rp2040.hpp"

constexpr static inline void configure_pads()
{
    pads::qspi_sclk::set_value(pads::slew_rate::fast,
                               pads::schmitt_trigger::default_after_reset,
                               pads::pull_down::default_after_reset,
                               pads::pull_up::default_after_reset,
                               pads::drive_strength::strength_8mA,
                               pads::input::default_after_reset,
                               pads::output::default_after_reset);

    const auto sd_value = pads::qspi_sd0::calculate_value(
      pads::slew_rate::default_after_reset,
      pads::schmitt_trigger::disable,
      pads::pull_down::default_after_reset,
      pads::pull_up::default_after_reset,
      pads::drive_strength::default_after_reset,
      pads::input::default_after_reset,
      pads::output::default_after_reset);

    // TODO: unfortunetly - this requires further optimization;
    //       GCC generates multiple LDR + STR instructions;
    //       we can do better:
    //       str rX [reg_with_QSPI_BASE, SD0_OFFSET]
    //       str rX [reg_with_QSPI_BASE, SD1_OFFSET]
    //       str rX [reg_with_QSPI_BASE, SD2_OFFSET]
    //       str rX [reg_with_QSPI_BASE, SD3_OFFSET]
    //
    pads::qspi_sd0::pad_reg::set_value(sd_value);
    pads::qspi_sd1::pad_reg::set_value(sd_value);
    pads::qspi_sd2::pad_reg::set_value(sd_value);
    pads::qspi_sd3::pad_reg::set_value(sd_value);
}

constexpr static void wait_ssi_ready()
{
    constexpr uint32_t transmit_fifo_empty =
      bit_value(platform::registers::ssi::sr_bits::tfe);
    constexpr uint32_t busy_flag =
      bit_value(platform::registers::ssi::sr_bits::busy);

    while (true) {
        platform::reg_val_t status_register =
          platform::registers::ssi::sr::value();
        if (!(status_register & transmit_fifo_empty)) {
            continue;
        }
        if (!(status_register & busy_flag)) {
            break;
        }
    }
}

enum class read_commands : uint8_t
{
    read_status = 0x05,
    read_status2 = 0x35,
};

enum class write_commands : uint8_t
{
    cmd_write_status = 0x01,
    write_enable = 0x06,
};

// TODO: provide abstractions for Winbond W25Q080 (and compatible)
constexpr static uint32_t read_flash_sreg(read_commands cmd)
{
    platform::registers::ssi::dr0::set_value(std::to_underlying(cmd));
    // Dummy byte:
    platform::registers::ssi::dr0::set_value(std::to_underlying(cmd));
    wait_ssi_ready();
    // Dummy read:
    platform::registers::ssi::dr0::value();
    return platform::registers::ssi::dr0::value();
}

// TODO: provide abstractions for Winbond W25Q080 (and compatible)
constexpr static void configure_flash(uint32_t expected_sreg2_value)
{
    platform::registers::ssi::dr0::set_value(
      std::to_underlying(write_commands::write_enable));
    wait_ssi_ready();
    // Discard the response
    platform::registers::ssi::dr0::value();
    platform::registers::ssi::dr0::set_value(
      std::to_underlying(write_commands::cmd_write_status));
    platform::registers::ssi::dr0::set_value(0);
    platform::registers::ssi::dr0::set_value(expected_sreg2_value);
    wait_ssi_ready();
    platform::registers::ssi::dr0::value();
    platform::registers::ssi::dr0::value();
    platform::registers::ssi::dr0::value();

    constexpr uint32_t expected_status = 0x1;

    while (read_flash_sreg(read_commands::read_status) != expected_status) {
        // just wait for the expected status...
    }
}

static inline void load_main_program()
{
    // Vector table should be placed right after the bootloader
    constexpr platform::reg_val_t vector_table_addr =
      platform::registers::addrs::xip_base + 0x100;
    constexpr platform::reg_val_t vtor_table_reg_addr =
      platform::registers::addrs::ppb_base +
      platform::registers::addrs::m0plus_vtor_offset;

    // TODO: Set stack pointer
    // TODO: Relocate vector table
    // TODO: Call reset handler via the vector table

    std::unreachable();
}

extern "C"
{
    //
    // An example bootloader (stage #2) for the Raspberry Pi Pico board
    //
    // The main task is to configure internal XIP so that the CPUs can fetch
    // and execute instructions directly from an external flash memory.
    //
    // The external flash will be addressed and accessed by the system as
    // though it were internal memory.
    //
    // Bus reads to a 16MB memory window starting at 0x10000000 are translated
    // into a serial flash transfer, and the result is returned to the master
    // that initiated the read.
    //
    // In my case, the bootloader will assume that we are interfacing with
    // Winbond W25Q080 flash via the QSPI.
    //
    //
    // TODO: provide abstractions for XIP/SSI
    // TODO: provide abstractions for Winbond W25Q080 (and compatible)
    //
    // TODO: THIS IS WORK IN PROGRESS;
    // TODO: demonkey the following code (get rid of direct access to
    // registers) - some things have been left as a placeholders for further
    // work.
    //
    void __attribute__((naked)) __regalis_bootloader_stage2()
    {
        using namespace platform;
        constexpr uint32_t flash_clk_div = 2;

        configure_pads();

        // 8 bits per data frame:
        constexpr uint32_t data_frame_size_in_32b_transfer_mode = 0x7;
        constexpr uint32_t ctrlr0_value =
          (data_frame_size_in_32b_transfer_mode
           << std::to_underlying(registers::ssi::ctrlr0_bits::dfs_32_0));

        // Disable SSI (required for reconfiguration)
        registers::ssi::ssienr::set_value(0);

        // Set baud rate
        registers::ssi::baudr::set_value(flash_clk_div);

        // Set 1-cycle sample delay
        registers::ssi::rx_sample_dly::set_value(1);

        // Configure data frame
        registers::ssi::ctrlr0::set_value(ctrlr0_value);

        // Enable SSI
        registers::ssi::ssienr::set_value(1);

        constexpr uint32_t expected_sreg2_value = 0x02UL;

        if (read_flash_sreg(read_commands::read_status2) !=
            expected_sreg2_value) {
            configure_flash(expected_sreg2_value);
        }

        // Disable SSI (required for reconfiguration)
        registers::ssi::ssienr::set_value(0);

        using registers::ssi::ctrlr0_bits;
        constexpr uint32_t xip_quad_spi_half_duplex_mode = 0x2;
        constexpr uint32_t xip_32_data_bits = 31;
        constexpr uint32_t xip_eeprom_read_mode = 0x3;
        constexpr uint32_t ssi_ctrlr0_enter_xip_value =
          (xip_quad_spi_half_duplex_mode << bit_pos(ctrlr0_bits::spi_frf0)) |
          (xip_32_data_bits << bit_pos(ctrlr0_bits::dfs_32_0)) |
          (xip_eeprom_read_mode << bit_pos(ctrlr0_bits::tmod0));

        registers::ssi::ctrlr0::set_value(ssi_ctrlr0_enter_xip_value);

        using registers::ssi::spi_ctrlr0_bits;
        constexpr uint32_t ssi_spi_ctrlr0_enter_xip_value =
          (8UL << bit_pos(spi_ctrlr0_bits::addr_l0)) |
          (4UL << bit_pos(spi_ctrlr0_bits::wait_cycles0)) |
          (0x2UL << bit_pos(spi_ctrlr0_bits::inst_l0)) |
          (0x1UL << bit_pos(spi_ctrlr0_bits::trans_type0));

        registers::ssi::spi_ctrlr0::set_value(ssi_spi_ctrlr0_enter_xip_value);

        // Enable SSI
        registers::ssi::ssienr::set_value(1);

        // Configure the flash - put it into the continous read mode
        constexpr uint8_t w25q080_cmd_read = 0xeb;
        constexpr uint8_t w25q080_mode_continous_read = 0xa0;
        registers::ssi::dr0::set_value(w25q080_cmd_read);
        registers::ssi::dr0::set_value(w25q080_mode_continous_read);
        wait_ssi_ready();

        // Disable SSI (required for reconfiguration)
        registers::ssi::ssienr::set_value(0);

        // Configure SSI to use continous read mode
        constexpr uint32_t spi_ctrlr0_final_value =
          (0xa0UL << bit_pos(spi_ctrlr0_bits::xip_cmd0)) |
          (8UL << bit_pos(spi_ctrlr0_bits::addr_l0)) |
          (4UL << bit_pos(spi_ctrlr0_bits::wait_cycles0)) |
          (0UL << bit_pos(spi_ctrlr0_bits::inst_l0)) |
          (0x2UL << bit_pos(spi_ctrlr0_bits::trans_type0));

        registers::ssi::spi_ctrlr0::set_value(spi_ctrlr0_final_value);

        // Enable SSI
        registers::ssi::ssienr::set_value(1);

        load_main_program();
        std::unreachable();
    }
}
