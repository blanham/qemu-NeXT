/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * 68040 arithmetic exception status, deferred delivery, and state frame.
 */

        .text
        .globl  _start
_start:
        lea     stack_end,%sp
        lea     dz_handler,%a0
        move.l  %a0,50*4

        fmove.d zero,%fp0
        fmove.d one,%fp1
        fdiv.x  %fp0,%fp1
        fmove.l %fpsr,%d0
        andi.l  #0x00000410,%d0
        cmpi.l  #0x00000410,%d0
        bne.s   fail

        moveq   #0,%d0
        fmove.l %d0,%fpsr
        move.l  #0x00000400,%d0
        fmove.l %d0,%fpcr
        fmove.d one,%fp1
        fdiv.x  %fp0,%fp1

        moveq   #0x12,%d7        /* integer work before deferred delivery */
        ftst.x  %fp1             /* delivers the pending divide-by-zero */
        bra.s   fail

dz_handler:
        cmpi.b  #0x12,%d7
        bne.s   fail
        fsave  -(%sp)
        cmpi.b  #0x60,1(%sp)     /* 100-byte busy frame */
        bne.s   fail
        movea.l #0xff009004,%a0
        moveq   #2,%d0
        move.l  %d0,(%a0)
fail:
1:
        bra.s   1b

        .data
        .balign 4
zero:
        .long   0, 0
one:
        .long   0x3ff00000, 0
        .space  2048
stack_end:
