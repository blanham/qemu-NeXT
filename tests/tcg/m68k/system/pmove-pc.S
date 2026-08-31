/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Opcode-driven MC68030 PMOVE PC-relative effective-address regression.
 * Control-alterable d16(PC) and indexed PC-relative forms are distinct from
 * the absolute and address-register forms exercised by pmove.S.
 */

        .text
        .globl  _start
_start:
        movea.l #0x00008000,%sp
        lea     illegal_handler,%a0
        move.l  %a0,0x10           /* illegal-instruction vector */
        lea     linef_handler,%a0
        move.l  %a0,0x2c           /* F-line vector */

        /* d16(PC), TT0: memory to control and control to memory. */
pc_tt0_read:
        .word   0xf03a                  /* PMOVE d16(PC),TT0 */
        .word   0x0800
        .word   pc_tt0_value - (pc_tt0_read + 4)

pc_tt0_write:
        .word   0xf03a                  /* PMOVE TT0,d16(PC) */
        .word   0x0a00
        .word   pc_tt0_out - (pc_tt0_write + 4)
        move.l  pc_tt0_out,%d0
        cmpi.l  #0x11223344,%d0
        bne.w   fail

        /* (d8,PC,D1.W), TT1: both transfer directions. */
        moveq   #0,%d1
pc_tt1_read:
        .word   0xf03b                  /* PMOVE d8(PC,D1.W),TT1 */
        .word   0x0c00
        .word   0x1800 + (pc_tt1_value - (pc_tt1_read + 4))

        moveq   #0,%d1
pc_tt1_write:
        .word   0xf03b                  /* PMOVE TT1,d8(PC,D1.W) */
        .word   0x0e00
        .word   0x1800 + (pc_tt1_out - (pc_tt1_write + 4))
        move.l  pc_tt1_out,%d0
        cmpi.l  #0x55667788,%d0
        bne.w   fail

        /* Immediate EA remains illegal for PMOVE and must take F-line. */
        lea     linef_expected_handler,%a0
        move.l  %a0,0x2c
        .word   0xf03c                  /* PMOVE #imm,TT0 (invalid EA) */
        .word   0x0800
        bra.w   fail

        movea.l #0xff009004,%a0
        moveq   #2,%d0                  /* VIRT_CTRL_CMD_HALT */
        move.l  %d0,(%a0)
success:
        bra.s   success

fail:
1:
        bra.s   1b

illegal_handler:
2:
        bra.s   2b

linef_handler:
3:
        bra.s   3b

linef_expected_handler:
        movea.l #0xff009004,%a0
        moveq   #2,%d0                  /* VIRT_CTRL_CMD_HALT */
        move.l  %d0,(%a0)
4:
        bra.s   4b

        /* Keep the indexed targets within the signed eight-bit PC range. */
pc_tt0_value:
        .long   0x11223344
pc_tt0_out:
        .long   0
pc_tt1_value:
        .long   0x55667788
pc_tt1_out:
        .long   0
