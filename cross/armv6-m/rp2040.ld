MEMORY {
    XIP : ORIGIN = 0x10000000, LENGTH = 16M
    SRAM : ORIGIN = 0x20000000, LENGTH = 256K
}

REGION_ALIAS("REGION_TEXT", SRAM)

SECTIONS {
    .text : {
        *(.text.*__regalis_init)
        *(.text*)
    } > REGION_TEXT
}

ENTRY(__regalis_init)
