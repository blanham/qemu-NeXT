/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * MC68030 PMOVE memory/control-register smoke test.  The two PMOVE
 * instructions are written as words so the test remains tied to the
 * architectural extension encodings instead of an assembler alias.
 */

        .text
        .globl  _start
_start:
        /* Exception entry now builds the real 68030 frame. */
        movea.l #0x00008000,%sp
        /* Undefined PMMU encodings are dispatched through the F-line
         * emulator vector; an illegal-instruction vector would fail this
         * check. */
        lea     illegal_handler,%a0
        move.l  %a0,0x10
        lea     linef_handler,%a0
        move.l  %a0,0x2c

        /* TC: register-sized memory transfers through (An). */
        lea     tc_value,%a0
        .word   0xf010                  /* PMOVE (%a0),TC */
        .word   0x4000

        lea     tc_out,%a0
        .word   0xf010                  /* PMOVE TC,(%a0) */
        .word   0x4200

        move.l  tc_out,%d0
        cmpi.l  #0x12345678,%d0
        bne.w   fail

        /* PMOVEFD updates TC without invalidating the ATC. */
        lea     tc_value,%a0
        .word   0xf010                  /* PMOVEFD (%a0),TC */
        .word   0x4100

        /* SRP: quad-word transfers through (d16,An). */
        lea     srp_value+4,%a0
        .word   0xf028                  /* PMOVE (-4,%a0),SRP */
        .word   0x4800
        .word   0xfffc

        lea     srp_out+4,%a0
        .word   0xf028                  /* PMOVE SRP,(-4,%a0) */
        .word   0x4a00
        .word   0xfffc
        move.l  srp_out,%d0
        cmpi.l  #0x00000003,%d0
        bne.w   fail
        move.l  srp_out+4,%d0
        cmpi.l  #0x00003000,%d0
        bne.w   fail

        lea     srp_value+4,%a0
        .word   0xf028                  /* PMOVEFD (-4,%a0),SRP */
        .word   0x4900
        .word   0xfffc

        /* CRP: quad-word transfers through absolute-long EAs. */
        .word   0xf039                  /* PMOVE (crp_value).L,CRP */
        .word   0x4c00
        .long   crp_value

        .word   0xf039                  /* PMOVE CRP,(crp_out).L */
        .word   0x4e00
        .long   crp_out
        move.l  crp_out,%d0
        cmpi.l  #0x00000002,%d0
        bne.w   fail
        move.l  crp_out+4,%d0
        cmpi.l  #0x00002000,%d0
        bne.w   fail

        .word   0xf039                  /* PMOVEFD (crp_value).L,CRP */
        .word   0x4d00
        .long   crp_value

        /* TT0: long-word transfers through a brief indexed EA. */
        moveq   #0,%d1
        lea     tt0_value,%a0
        .word   0xf030                  /* PMOVE (0,%a0,%d1.w),TT0 */
        .word   0x0800
        .word   0x1000

        moveq   #0,%d1
        lea     tt0_out,%a0
        .word   0xf030                  /* PMOVE TT0,(0,%a0,%d1.w) */
        .word   0x0a00
        .word   0x1000
        move.l  tt0_out,%d0
        cmpi.l  #0x00123456,%d0
        bne.w   fail

        moveq   #0,%d1
        lea     tt0_value,%a0
        .word   0xf030                  /* PMOVEFD (0,%a0,%d1.w),TT0 */
        .word   0x0900
        .word   0x1000

        /* TT1: long-word transfers through absolute-short EAs. */
        .word   0xf038                  /* PMOVE (tt1_value).W,TT1 */
        .word   0x0c00
        .word   tt1_value

        .word   0xf038                  /* PMOVE TT1,(tt1_out).W */
        .word   0x0e00
        .word   tt1_out
        move.l  tt1_out,%d0
        cmpi.l  #0x00004567,%d0
        bne.w   fail

        .word   0xf038                  /* PMOVEFD (tt1_value).W,TT1 */
        .word   0x0d00
        .word   tt1_value

        /* MMUSR: word transfers through (d16,An). */
        lea     mmusr_value+2,%a0
        .word   0xf028                  /* PMOVE (-2,%a0),MMUSR */
        .word   0x6000
        .word   0xfffe

        lea     mmusr_out+2,%a0
        .word   0xf028                  /* PMOVE MMUSR,(-2,%a0) */
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_out,%d0
        cmpi.w  #0xa55a,%d0
        bne.w   fail

        /* P=001 is not an MC68030 PMMU register encoding. */
        .word   0xf010                  /* undefined PMMU encoding */
        .word   0x4400
        bra.w   fail

        moveq   #2,%d0                  /* VIRT_CTRL_CMD_HALT */
        move.l  %d0,0xff009004
1:
        bra.s   1b

fail:
        /* A failed assertion deliberately does not halt the guest, so the
         * wrapper's timeout reports the test failure. */
2:
        bra.s   2b

illegal_handler:
3:
        bra.s   3b

linef_handler:
        moveq   #2,%d0
        move.l  %d0,0xff009004
4:
        bra.s   4b

        .data
        .balign 4
tc_value:
        .long   0x12345678
tc_out:
        .long   0
srp_value:
        .long   0x00000003,0x00003005
srp_out:
        .long   0,0
crp_value:
        .long   0x00000002,0x00002007
crp_out:
        .long   0,0
tt0_value:
        .long   0x00123456
tt0_out:
        .long   0
tt1_value:
        .long   0x00004567
tt1_out:
        .long   0
mmusr_value:
        .word   0xa55a
        .word   0
mmusr_out:
        .word   0
