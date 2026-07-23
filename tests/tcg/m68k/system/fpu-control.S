/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * 68040 floating-point control register regression.
 */

        .text
        .globl  _start
_start:
        move.l  #0x12345678,%d0
        fmove.l %d0,%fpiar
        moveq   #0,%d1
        fmove.l %fpiar,%d1

        cmp.l   %d0,%d1
        bne.s   fail
        movea.l #0xff009004,%a0
        moveq   #2,%d2           /* VIRT_CTRL_CMD_HALT */
        move.l  %d2,(%a0)
fail:
1:
        bra.s   1b
