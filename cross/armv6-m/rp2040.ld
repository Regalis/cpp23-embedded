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

MEMORY {
    XIP(rx) : ORIGIN = 0x10000000, LENGTH = 2048K
    SRAM(rwx) : ORIGIN = 0x20000000, LENGTH = 256K
    SRAM_BOOT2(rwx) : ORIGIN = ORIGIN(SRAM) + LENGTH(SRAM) - 256, LENGTH = 256
    SRAM_BANK_A(rwx) : ORIGIN = 0x20040000, LENGTH = 4k
    SRAM_BANK_B(rwx) : ORIGIN = 0x20041000, LENGTH = 4k
}

REGION_ALIAS("REGION_TEXT", SRAM)

SECTIONS {
    .regalis_bootloader : {
        __boot2_start__ = .;
        *(.text.*__regalis_bootloader_stage2)
        *(.text.*__regalis_bootloader_*)
        . = ALIGN(256);
        __boot2_end__ = .;
    } > SRAM_BOOT2 AT> XIP

    ASSERT(__boot2_end__ - __boot2_start__ == 256,
        "ERROR: Second stage bootloader must be 256 bytes in size")

    .vector_table : {
        *(__vector_table)
    } > XIP

    .text : {
        . = ALIGN(4);
        *(.text.*_isr)
        *(.text.*__regalis_init)
        *(.text*)
        . = ALIGN(4);
    } > XIP

    .data : {
        /* Start of data section (VMA) */
        . = ALIGN(4);
        __data_start = .;
        *(.data)
        *(.data*)
        /* End of data section (VMA) */
        . = ALIGN(4);
        __data_end = .;
    } > SRAM AT> XIP

    __data_size = __data_end - __data_start;

    /* Start of data section (LMA) */
    __data_lma_start = LOADADDR(.data);

    .bss  : {
        . = ALIGN(4);
        __bss_start = .;
        *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.bss*)))
        . = ALIGN(4);
        __bss_end = .;
    } > SRAM

    __stack_pointer = ORIGIN(SRAM_BANK_B) + LENGTH(SRAM_BANK_B);
}

ENTRY(__regalis_init)
