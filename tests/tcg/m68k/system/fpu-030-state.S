/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * MC68030 external-FPU FSAVE/FRESTORE dispatch and idle-frame regression.
 */

        .text
        .globl  _start

_start:
        lea     null_frame+4,%a0
        fsave   -(%a0)
        cmpa.l  #null_frame,%a0
        bne.w   fail
        tst.l   (%a0)
        bne.w   fail

        lea     null_frame,%a0
        frestore (%a0)

        fmove.d one,%fp0
        lea     idle_frame+60,%a0
        fsave   -(%a0)
        cmpa.l  #idle_frame,%a0
        bne.w   fail
        cmpi.l  #0x3f380000,(%a0)
        bne.w   fail

        lea     idle_frame,%a0
        frestore (%a0)+
        cmpa.l  #idle_frame+60,%a0
        bne.w   fail

        movea.l #0xff009004,%a0
        moveq   #2,%d0           /* VIRT_CTRL_CMD_HALT */
        move.l  %d0,(%a0)
success:
        bra.s   success

fail:
1:
        bra.s   1b

        .data
        .balign 4
null_frame:
        .space  4
idle_frame:
        .space  60
one:
        .long   0x3ff00000, 0
