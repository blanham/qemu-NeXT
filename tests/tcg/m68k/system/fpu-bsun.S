/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * 68040 signaling unordered-condition (BSUN) regression.
 */

        .text
        .globl  _start
_start:
        lea     stack_end,%sp
        lea     handler,%a0
        move.l  %a0,48*4

        fmove.d qnan,%fp0
        ftst.x  %fp0

        moveq   #0,%d0
        .word   0xf240, 0x0018   /* signaling unordered, D0 */
        cmpi.b  #0xff,%d0
        bne.s   fail
        fmove.l %fpsr,%d1
        andi.l  #0x00008080,%d1  /* BSUN exception + accrued IOP */
        cmpi.l  #0x00008080,%d1
        bne.s   fail

        move.l  #0x00008000,%d0
        fmove.l %d0,%fpcr
        .word   0xf240, 0x0018   /* must vector before modifying D0 */
        bra.s   fail

handler:
        movea.l #0xff009004,%a0
        moveq   #2,%d0
        move.l  %d0,(%a0)
fail:
1:
        bra.s   1b

        .data
        .balign 4
qnan:
        .long   0x7ff80000, 0
        .space  1024
stack_end:
