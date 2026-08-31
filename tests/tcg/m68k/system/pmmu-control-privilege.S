/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * MC68030 PMMU control-instruction privilege regression.
 */

        .text
        .globl  _start
_start:
        lea     privilege_handler,%a0
        move.l  %a0,0x20           /* privilege-violation vector (8) */

        /* PFLUSH is supervisor-only; the handler is the only success path. */
        move.w  #0,%sr
        .word   0xf000              /* PFLUSHA */
        .word   0x2400
        bra.s   fail

privilege_handler:
        movea.l #0xff009004,%a0
        moveq   #2,%d0              /* VIRT_CTRL_CMD_HALT */
        move.l  %d0,(%a0)
1:
        bra.s   1b

fail:
2:
        bra.s   2b
