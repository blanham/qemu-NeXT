/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * 68040 FSAVE/FRESTORE frame and address-writeback regression.
 */

        .text
        .globl  _start
_start:
        lea     busy_frame,%a0
        frestore (%a0)+
        cmpa.l  #busy_frame+100,%a0
        bne.s   fail

        lea     saved_frame+100,%a1
        fsave  -(%a1)
        cmpa.l  #saved_frame,%a1
        bne.s   fail

        lea     busy_frame,%a0
        moveq   #24,%d0
compare:
        cmpm.l  (%a0)+,(%a1)+
        dbne    %d0,compare
        bne.s   fail

        lea     idle_frame+4,%a1
        fsave  -(%a1)
        cmpa.l  #idle_frame,%a1
        bne.s   fail
        cmpi.l  #0x41000000,(%a1)
        bne.s   fail

        movea.l #0xff009004,%a0
        moveq   #2,%d0           /* VIRT_CTRL_CMD_HALT */
        move.l  %d0,(%a0)
fail:
1:
        bra.s   1b

        .data
        .balign 4
busy_frame:
        .long   0x40600000
        .long   0x01020304, 0x05060708, 0x090a0b0c, 0x0d0e0f10
        .long   0x11121314, 0x15161718, 0x191a1b1c, 0x1d1e1f20
        .long   0x21222324, 0x25262728, 0x292a2b2c, 0x2d2e2f30
        .long   0x31323334, 0x35363738, 0x393a3b3c, 0x3d3e3f40
        .long   0x41424344, 0x45464748, 0x494a4b4c, 0x4d4e4f50
        .long   0x51525354, 0x55565758, 0x595a5b5c, 0x5d5e5f60
saved_frame:
        .space  100
idle_frame:
        .space  4
