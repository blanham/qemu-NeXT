/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * MC68030 MOVEC CAAR architectural read/write regression.
 */

        .text
        .globl  _start

_start:
        moveq   #0,%d0
        .word   0x4e7a                  /* MOVEC CAAR,D1 */
        .word   0x1802
        cmp.l   %d0,%d1
        bne.w   fail

        move.l  #0x12345678,%d0
        .word   0x4e7b                  /* MOVEC D0,CAAR */
        .word   0x0802
        .word   0x4e7a                  /* MOVEC CAAR,D1 */
        .word   0x1802
        cmp.l   %d0,%d1
        bne.w   fail

        move.l  #0xffffffff,%d0
        .word   0x4e7b                  /* MOVEC D0,CAAR */
        .word   0x0802
        .word   0x4e7a                  /* MOVEC CAAR,D1 */
        .word   0x1802
        cmp.l   %d0,%d1
        bne.w   fail

        move.l  #2,0xff009004           /* VIRT_CTRL_CMD_HALT */
success:
        bra.s   success

fail:
1:
        bra.s   1b
