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

        move.l  #0x05000000,%d0  /* NAN + ZERO condition codes */
        fmove.l %d0,%fpsr
        fmove.d %fp0,result
        fmove.l %fpsr,%d1
        andi.l  #0x0f000000,%d1
        cmpi.l  #0x05000000,%d1
        bne.s   fail

        moveq   #0,%d0
        fmove.l %d0,%fpsr
        fmove.d qnan,%fp0
        fmove.d one,%fp1
        fcmp.x  %fp1,%fp0
        fmove.l %fpsr,%d1
        move.l  %d1,%d0
        andi.l  #0x01000000,%d0
        beq.s   fail
        andi.l  #0x00000080,%d1
        bne.s   fail

        movea.l #0xff009004,%a0
        moveq   #2,%d2           /* VIRT_CTRL_CMD_HALT */
        move.l  %d2,(%a0)
fail:
1:
        bra.s   1b

        .data
        .balign 4
result:
        .long   0, 0
qnan:
        .long   0x7ff80000, 0
one:
        .long   0x3ff00000, 0
