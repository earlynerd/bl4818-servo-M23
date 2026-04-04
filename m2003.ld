/*
 * M2003FC1AE Linker Script (32KB Flash, 4KB SRAM)
 */

MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 32K
    RAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 4K
}

ENTRY(Reset_Handler)

SECTIONS
{
    .text :
    {
        KEEP(*(.vectors))
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    .data :
    {
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    _sidata = LOADADDR(.data);

    .bss :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /* Stack starts at the end of RAM */
    _estack = ORIGIN(RAM) + LENGTH(RAM);
}
