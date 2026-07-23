/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Minimal system-mode regression for the 68040 packed-decimal FMOVE input.
 *
 * This uses the exact instruction and +1 encoding used by NeXT libc's
 * _pdfptodbl routine:
 *
 *     fmovep  (%a0),%fp0        f210 4c00
 */

        .text
        .globl  _start
_start:
        lea     vectors,%a1
        moveq   #7,%d2
vector_loop:
        movea.l %a1,%a0
        .word   0xf210, 0x4c00
        fmove.d %fp0,result

        moveq   #3,%d0           /* VIRT_CTRL_CMD_PANIC */
        move.l  12(%a1),%d1
        cmp.l   result,%d1
        bne.s   exit
        move.l  16(%a1),%d1
        cmp.l   result+4,%d1
        bne.s   exit
        adda.l  #20,%a1
        dbra    %d2,vector_loop

        lea     packed_qnan,%a0
        .word   0xf210, 0x4c00
        fmove.d %fp0,result
        move.l  result,%d1
        andi.l  #0x7ff00000,%d1
        cmpi.l  #0x7ff00000,%d1
        bne.s   exit
        move.l  result,%d1
        andi.l  #0x000fffff,%d1
        or.l    result+4,%d1
        beq.s   exit

        moveq   #2,%d0           /* VIRT_CTRL_CMD_HALT */

exit:
        movea.l #0xff009004,%a0  /* m68k virt power controller */
        move.l  %d0,(%a0)
1:
        bra.s   1b

        .data
        .balign 4
vectors:
        /* packed input                         expected IEEE double */
        .long   0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
        .long   0x80000000, 0x00000000, 0x00000000, 0x80000000, 0x00000000
        .long   0x00000001, 0x00000000, 0x00000000, 0x3ff00000, 0x00000000
        .long   0x80000001, 0x00000000, 0x00000000, 0xbff00000, 0x00000000
        .long   0x00020001, 0x23500000, 0x00000000, 0x405ee000, 0x00000000
        .long   0x00200001, 0x00000000, 0x00000000, 0x4415af1d, 0x78b58c40
        .long   0x40030001, 0x00000000, 0x00000000, 0x3f50624d, 0xd2f1a9fc
        .long   0x7fff0000, 0x00000000, 0x00000000, 0x7ff00000, 0x00000000
packed_qnan:
        .long   0x7fff0001, 0x00000000, 0x00000000
result:
        .long   0, 0
