/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * System-mode regression for 68040 packed-decimal FMOVE input and output.
 *
 * This uses the exact instruction and +1 encoding used by NeXT libc's
 * _pdfptodbl routine:
 *
 *     fmovep  (%a0),%fp0        f210 4c00
 */

        .text
        .globl  _start
_start:
        lea     stack_end,%sp
        lea     vectors,%a1
        moveq   #8,%d2
vector_loop:
        movea.l %a1,%a0
        .word   0xf210, 0x4c00
        fmove.d %fp0,result

        move.l  12(%a1),%d1
        cmp.l   result,%d1
        bne.w   fail
        move.l  16(%a1),%d1
        cmp.l   result+4,%d1
        bne.w   fail
        adda.l  #20,%a1
        dbra    %d2,vector_loop

        lea     packed_qnan,%a0
        .word   0xf210, 0x4c00
        fmove.d %fp0,result
        move.l  result,%d1
        andi.l  #0x7ff00000,%d1
        cmpi.l  #0x7ff00000,%d1
        bne.w   fail
        move.l  result,%d1
        andi.l  #0x000fffff,%d1
        or.l    result+4,%d1
        beq.w   fail

        /*
         * Packed output, including both dynamic and static k-factors.
         * 123.5 with 17 digits is exact; three significant digits and
         * zero decimal places both round the tie to the even value 124.
         */
        fmove.d value_123_5,%fp0
        moveq   #0,%d1
        fmove.l %d1,%fpsr
        lea     packed_result,%a0
        moveq   #17,%d0
        .word   0xf210, 0x7c00
        cmpi.l  #0x00020001,packed_result
        bne.w   fail
        cmpi.l  #0x23500000,packed_result+4
        bne.w   fail
        tst.l   packed_result+8
        bne.w   fail
        fmove.l %fpsr,%d1
        andi.l  #0x0000ff08,%d1
        bne.w   fail

        moveq   #0,%d1
        fmove.l %d1,%fpsr
        lea     packed_result,%a0
        .word   0xf210, 0x6c03
        cmpi.l  #0x00020001,packed_result
        bne.w   fail
        cmpi.l  #0x24000000,packed_result+4
        bne.w   fail
        tst.l   packed_result+8
        bne.w   fail
        fmove.l %fpsr,%d1
        andi.l  #0x0000ff08,%d1
        cmpi.l  #0x00000208,%d1
        bne.w   fail

        moveq   #0,%d1
        fmove.l %d1,%fpsr
        lea     packed_result,%a0
        .word   0xf210, 0x6c00
        cmpi.l  #0x00020001,packed_result
        bne.w   fail
        cmpi.l  #0x24000000,packed_result+4
        bne.w   fail
        tst.l   packed_result+8
        bne.w   fail

        moveq   #0,%d1
        fmove.l %d1,%fpsr
        lea     packed_result,%a0
        moveq   #-1,%d0
        .word   0xf210, 0x7c00
        cmpi.l  #0x00020001,packed_result
        bne.w   fail
        cmpi.l  #0x23500000,packed_result+4
        bne.w   fail
        tst.l   packed_result+8
        bne.w   fail

        /*
         * For k <= 0, FPSP clamps ILOG to k after computing LEN.  This
         * very small normal therefore rounds to a zero mantissa, whose
         * packed exponent is forced to +1.
         */
        fmove.d value_tiny,%fp0
        lea     packed_result,%a0
        moveq   #-2,%d0
        .word   0xf210, 0x7c00
        cmpi.l  #0x00010000,packed_result
        bne.w   fail
        tst.l   packed_result+4
        bne.w   fail
        tst.l   packed_result+8
        bne.w   fail

        /*
         * Positive k-factors above 17 are clamped and signal OPERR/AIOP.
         */
        moveq   #0,%d1
        fmove.l %d1,%fpsr
        lea     packed_result,%a0
        .word   0xf210, 0x6c12
        fmove.l %fpsr,%d1
        andi.l  #0x00002080,%d1
        cmpi.l  #0x00002080,%d1
        bne.w   fail

        /*
         * Directed rounding applies to the signed source, even though the
         * decimal conversion works with its magnitude.
         */
        moveq   #0x30,%d1        /* round toward plus infinity */
        fmove.l %d1,%fpcr
        fmove.d value_123_1,%fp0
        lea     packed_result,%a0
        .word   0xf210, 0x6c03
        cmpi.l  #0x00020001,packed_result
        bne.w   fail
        cmpi.l  #0x24000000,packed_result+4
        bne.w   fail

        moveq   #0x20,%d1        /* round toward minus infinity */
        fmove.l %d1,%fpcr
        fmove.d value_neg_123_1,%fp0
        lea     packed_result,%a0
        .word   0xf210, 0x6c03
        cmpi.l  #0x80020001,packed_result
        bne.w   fail
        cmpi.l  #0x24000000,packed_result+4
        bne.w   fail
        moveq   #0,%d1
        fmove.l %d1,%fpcr

        fmove.d ieee_inf,%fp0
        lea     packed_result,%a0
        .word   0xf210, 0x6c00
        cmpi.l  #0x7fff0000,packed_result
        bne.w   fail
        tst.l   packed_result+4
        bne.w   fail
        tst.l   packed_result+8
        bne.w   fail

        /*
         * Packed special output preserves the extended NaN sign and payload.
         * A quiet NaN does not set an exception; a signaling NaN sets
         * SNAN/AIOP without being canonicalized.
         */
        fmove.x ext_qnan,%fp0
        moveq   #0,%d1
        fmove.l %d1,%fpsr
        lea     packed_result,%a0
        .word   0xf210, 0x6c00
        cmpi.l  #0x7fff0000,packed_result
        bne.w   fail
        cmpi.l  #0xc1234567,packed_result+4
        bne.w   fail
        cmpi.l  #0x89abcdef,packed_result+8
        bne.w   fail
        fmove.l %fpsr,%d1
        andi.l  #0x00006080,%d1
        bne.w   fail

        fmove.x ext_snan,%fp0
        moveq   #0,%d1
        fmove.l %d1,%fpsr
        lea     packed_result,%a0
        .word   0xf210, 0x6c00
        cmpi.l  #0xffff0000,packed_result
        bne.w   fail
        cmpi.l  #0x81234567,packed_result+4
        bne.w   fail
        cmpi.l  #0x89abcdef,packed_result+8
        bne.w   fail
        fmove.l %fpsr,%d1
        andi.l  #0x00006080,%d1
        cmpi.l  #0x00004080,%d1
        bne.w   fail

        fmove.d ieee_neg_zero,%fp0
        lea     packed_result,%a0
        .word   0xf210, 0x6c00
        cmpi.l  #0x80000000,packed_result
        bne.w   fail
        tst.l   packed_result+4
        bne.w   fail
        tst.l   packed_result+8
        bne.w   fail

        /*
         * Size code 7 is the dynamic-k encoding for packed output only.
         * Using it for a source effective address must take the line-F
         * exception instead of being decoded as ordinary packed input.
         */
        lea     linef_handler,%a0
        move.l  %a0,11*4
        lea     packed_result,%a0
        .word   0xf210, 0x5c00
        bra.w   fail

linef_handler:
        movea.l #0xff009004,%a0  /* m68k virt power controller */
        moveq   #2,%d0           /* VIRT_CTRL_CMD_HALT */
        move.l  %d0,(%a0)
fail:
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
        .long   0x00001001, 0x00000000, 0x00000000, 0x7ff00000, 0x00000000
packed_qnan:
        .long   0x7fff0001, 0x00000000, 0x00000000
value_123_5:
        .long   0x405ee000, 0x00000000
value_123_1:
        .long   0x405ec666, 0x66666666
value_neg_123_1:
        .long   0xc05ec666, 0x66666666
value_tiny:
        .long   0x3f201f31, 0xf46ed246
ieee_inf:
        .long   0x7ff00000, 0x00000000
ext_qnan:
        .long   0x7fff0000, 0xc1234567, 0x89abcdef
ext_snan:
        .long   0xffff0000, 0x81234567, 0x89abcdef
ieee_neg_zero:
        .long   0x80000000, 0x00000000
packed_result:
        .long   0, 0, 0
result:
        .long   0, 0
        .space  1024
stack_end:
