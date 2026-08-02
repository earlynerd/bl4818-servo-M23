/* M2003FC1AE permanent recovery-loader linker script. */

MEMORY
{
    LDROM   (rx)  : ORIGIN = 0x00100000, LENGTH = 0x1000
    RAM     (rwx) : ORIGIN = 0x20000000, LENGTH = 0x0FF0
    MAILBOX (rw)  : ORIGIN = 0x20000FF0, LENGTH = 0x0010
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
    } > LDROM

    .data :
    {
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > LDROM

    _sidata = LOADADDR(.data);

    .bss :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    .boot_mailbox (NOLOAD) :
    {
        KEEP(*(.boot_mailbox))
    } > MAILBOX

    _estack = ORIGIN(RAM) + LENGTH(RAM);

    ASSERT((LOADADDR(.data) + SIZEOF(.data)) <= (ORIGIN(LDROM) + LENGTH(LDROM)),
           "LDROM loader exceeds 4096 bytes")
    ASSERT(_ebss <= (ORIGIN(RAM) + LENGTH(RAM)),
           "LDROM loader RAM overlaps the boot mailbox")
}
