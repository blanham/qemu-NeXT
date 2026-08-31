/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * MC68030 PMOVE privilege violation regression.
 */

        .text
        .globl  _start
_start:
        movea.l #0x00008000,%sp
        lea     privilege_handler,%a0
        move.l  %a0,0x20           /* privilege-violation vector (8) */
        lea     tc_value,%a0

        /* PMOVE is supervisor-only; the handler is the only success path. */
        move.w  #0,%sr
        .word   0xf010              /* PMOVE (%a0),TC */
        .word   0x4000
        bra.s   fail

privilege_handler:
        movea.l #0xff009004,%a0
        moveq   #2,%d0             /* VIRT_CTRL_CMD_HALT */
        move.l  %d0,(%a0)
1:
        bra.s   1b

fail:
2:
        bra.s   2b

        .data
tc_value:
        .long   0x80000000
