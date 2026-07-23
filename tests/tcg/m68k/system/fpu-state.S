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
        bne.w   fail

        lea     saved_frame+100,%a1
        fsave  -(%a1)
        cmpa.l  #saved_frame,%a1
        bne.w   fail

        lea     busy_frame,%a0
        moveq   #24,%d0
compare:
        cmpm.l  (%a0)+,(%a1)+
        dbne    %d0,compare
        bne.w   fail

        /*
         * A four-byte frame with a zero version byte is the architected
         * NULL state; its remaining bytes are undefined.  It must
         * restore without a format fault, advance postincrement by four,
         * reset the FPU programming model, and cause the next FSAVE to
         * produce another NULL frame.
         */
        lea     busy_frame,%a0
        frestore (%a0)
        fmove.d one,%fp0
        move.l  #0x30,%d0
        fmove.l %d0,%fpcr
        move.l  #0x05000208,%d0
        fmove.l %d0,%fpsr
        move.l  #0x12345678,%d0
        fmove.l %d0,%fpiar

        lea     null_frame,%a0
        frestore (%a0)+
        cmpa.l  #null_frame+4,%a0
        bne.w   fail

        lea     null_saved_frame+4,%a1
        fsave  -(%a1)
        cmpa.l  #null_saved_frame,%a1
        bne.w   fail
        tst.b   (%a1)
        bne.w   fail

        fmove.l %fpcr,%d0
        bne.w   fail
        fmove.l %fpsr,%d0
        bne.w   fail
        fmove.l %fpiar,%d0
        bne.w   fail
        fmove.x %fp0,null_fp0
        cmpi.l  #0x7fff0000,null_fp0
        bne.w   fail
        move.l  null_fp0+4,%d0
        andi.l  #0x40000000,%d0
        cmpi.l  #0x40000000,%d0
        bne.w   fail

        /*
         * NeXT Mach probes the FPU with this exact sequence.  The FMOVE is
         * the first nonconditional floating-point instruction, so it moves
         * the restored NULL state to IDLE before the following FSAVE.
         */
        lea     null_frame,%a0
        frestore (%a0)
        fmove.l null_frame,%fp0
        lea     idle_saved_frame+4,%a1
        fsave  -(%a1)
        cmpa.l  #idle_saved_frame,%a1
        bne.w   fail
        cmpi.l  #0x41000000,(%a1)
        bne.w   fail

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
null_frame:
        .long   1               /* exact NeXT Mach _fpu_version backing word */
null_saved_frame:
        .long   0
idle_saved_frame:
        .long   0
null_fp0:
        .long   0, 0, 0
one:
        .long   0x3ff00000, 0
