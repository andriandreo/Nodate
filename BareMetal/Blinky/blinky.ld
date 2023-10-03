/* Define MEMORY regions */
MEMORY {
    flash (RX)  : ORIGIN = 0x08000000, LENGTH = 64K /* Read-only and eXecutable */
    sram (! RX) : ORIGIN = 0x20000000, LENGTH = 20K /* Read-Write and NOT eXecutable */
}

SECTIONS {
    .vectors    : { *(.vectors) } > flash 
    .text       : { *(.text) } > flash /* Code section */
    .rodata     : { *(.rodata) } > flash /* Read-Only Data */
    .bss        : { *(.bss) } > sram /* Read-Write memory into 'sram' */
}