/* SPDX-License-Identifier: GPL-2.0-or-later */

ENTRY(_start)

SECTIONS
{
    . = 0x00001000;
    .text : { *(.text*) }
    .rodata : { *(.rodata*) }
    .data : { *(.data*) }
    .bss : { *(.bss*) *(COMMON) }
}
