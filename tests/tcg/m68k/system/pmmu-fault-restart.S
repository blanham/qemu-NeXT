/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * MC68030 PMMU access-error restart probe.
 *
 * Every fault is repaired by the bus-error handler and returned with RTE.
 * The handler checks the architectural format/vector, SSW, fault address,
 * and data output buffer before changing the table entry.  A monotonically
 * increasing case counter makes a second entry for any one access fail the
 * test: the original instruction must be retried exactly once.
 */

        .text
        .globl  _start

_start:
        movea.l #stack_top,%sp

        /* Access, address, format, and F-line exceptions all fail closed. */
        lea     access_handler,%a0
        move.l  %a0,0x08
        move.l  %a0,0x0c
        move.l  %a0,0x38
        move.l  %a0,0x2c

        /* Seed physical pages while translation is disabled. */
        movea.l #0x00020000,%a0
        move.l  #0x11223344,(%a0)
        movea.l #0x00021000,%a0
        move.l  #0x55667788,(%a0)
        movea.l #0x00022000,%a0
        move.b  #0x5a,(%a0)
        movea.l #0x00023000,%a0
        move.w  #0x6b7c,(%a0)
        movea.l #0x00024000,%a0
        move.l  #0x7d8e9faf,(%a0)
        movea.l #0x00025000,%a0
        move.l  #0xaabbccdd,(%a0)
        movea.l #0x00027000,%a0
        move.l  #0x11223344,(%a0)
        movea.l #0x00028000,%a0
        clr.l   (%a0)
        movea.l #0x00029000,%a0
        move.l  #0x22222222,(%a0)
        movea.l #0x0002a000,%a0
        clr.l   (%a0)
        movea.l #0x0002bffc,%a0
        move.l  #0x3ff00000,(%a0)
        movea.l #0x0002c000,%a0
        clr.l   (%a0)
        movea.l #0x0002dffa,%a0
        move.b  #0x11,(%a0)
        move.b  #0x22,2(%a0)
        move.b  #0x33,4(%a0)
        move.b  #0x44,0x0002e000
        movea.l #0x0002fffc,%a0
        move.l  #0x7fff0002,(%a0)
        movea.l #0x00027ffe,%a0
        move.w  #0xa1b2,(%a0)
        movea.l #0x00032000,%a0
        move.w  #0xc3d4,(%a0)
        movea.l #0x00033000,%a0
        move.b  #0x01,(%a0)
        movea.l #0x00034000,%a0
        move.b  #0x22,(%a0)
        movea.l #0x00035000,%a0
        move.l  #0x22222222,(%a0)
        movea.l #0x00036000,%a0
        move.l  #0x22222222,(%a0)
        movea.l #0x00037000,%a0
        move.l  #0x33333333,(%a0)
        movea.l #0x00038ffe,%a0
        move.w  #0xa1b2,(%a0)
        movea.l #0x00039000,%a0
        move.w  #0xc3d4,(%a0)
        movea.l #0x0003aff8,%a0
        move.l  #0x3fff0000,(%a0)
        move.l  #0x80000000,4(%a0)
        move.l  #0x12345678,8(%a0)
        fmove.x (%a0),%fp1
        movea.l #0x0003cff8,%a0
        move.l  #0x3fff0000,(%a0)
        move.l  #0x80000000,4(%a0)
        move.l  #0xdeadbeef,8(%a0)
        movea.l #0x0003f000,%a0
        move.w  #0xc3d4,(%a0)
        movea.l #0x00040fff,%a0
        move.b  #0x11,(%a0)
        movea.l #0x00041000,%a0
        move.w  #0x2233,(%a0)
        move.b  #0x44,2(%a0)
        movea.l #0x00042000,%a0
        move.l  #0x80000000,(%a0)
        move.l  #0x12345678,4(%a0)
        /* Seed one extended-precision operand across two physical pages for
         * the FMOVEM multi-cycle restart case. */
        movea.l #0x0002bffc,%a0
        fmove.d (%a0),%fp0
        movea.l #0x00030ff8,%a0
        fmovem.x %fp0,(%a0)

        /* Root descriptor selects the normal four-byte page table. */
        lea     root_table,%a0
        lea     page_table,%a1
        move.l  %a1,%d0
        ori.l   #2,%d0
        move.l  %d0,(%a0)

        /* Identity-map the image, tables, stack, and handler. */
        lea     page_table,%a1
        moveq   #1,%d0
        /* Include the seeded physical pages through 0x25000 as well. */
        moveq   #37,%d1
1:
        move.l  %d0,(%a1)+
        addi.l  #0x1000,%d0
        dbra    %d1,1b

        /* The separate table is reached through root index one. */
        lea     page_table2,%a1
        move.l  %a1,%d0
        ori.l   #2,%d0
        move.l  %d0,page_table2_root
        move.l  #0x00024001,(%a1)
        lea     root_table,%a1
        move.l  %a1,0x00300000

        /* Initial mappings used by the fault cases. */
        move.l  #0x00020005,page_table+0x400 /* 00100000: long write WP */
        move.l  #0x00000000,page_table+0x404 /* 00101000: long read invalid */
        move.l  #0x00022005,page_table+0x408 /* 00102000: byte write WP */
        move.l  #0x00000000,page_table+0x40c /* 00103000: word read invalid */
        move.l  #0xffff0001,page_table+0x410 /* 00104000: physical bus */
        move.l  #0x00026005,page_table+0x414 /* 00105000: RMW write WP */
        move.l  #0x00027001,page_table+0x418 /* 00106000: distinct read */
        move.l  #0x00028005,page_table+0x41c /* 00107000: distinct write WP */
        move.l  #0x00026005,page_table+0x420 /* 00108000: TAS WP */
        move.l  #0x00027005,page_table+0x424 /* 00109000: CAS WP */
        move.l  #0x0002a001,page_table+0x428 /* 0010a000: CAS2 first */
        move.l  #0x00029005,page_table+0x42c /* 0010b000: CAS2 second WP */
        move.l  #0x0002b001,page_table+0x434 /* 0010d000: FPU first */
        move.l  #0x00000000,page_table+0x438 /* 0010e000: FPU second */
        move.l  #0x0002d001,page_table+0x43c /* 0010f000: MOVEP first */
        move.l  #0x00000000,page_table+0x440 /* 00110000: MOVEP second */
        move.l  #0x0002f001,page_table+0x444 /* 00111000: PMOVE first */
        move.l  #0x00000000,page_table+0x448 /* 00112000: PMOVE second */
        move.l  #0x00030001,page_table+0x44c /* 00113000: FMOVEM first */
        move.l  #0x00000000,page_table+0x450 /* 00114000: FMOVEM second */
        move.l  #0x00027001,page_table+0x454 /* 00115000: split first */
        move.l  #0x00000000,page_table+0x458 /* 00116000: split second */
        move.l  #0x00033005,page_table+0x45c /* 00117000: TAS WP */
        move.l  #0x00033001,page_table+0x460 /* 00118000: TAS alias */
        move.l  #0x00030001,page_table+0x464 /* 00119000: FMOVEM alias */
        move.l  #0x00031001,page_table+0x468 /* 0011a000: FMOVEM alias */
        move.l  #0,page_table+0x46c         /* 0011b000: TAS read invalid */
        move.l  #0x00034001,page_table+0x470 /* 0011c000: TAS read alias */
        move.l  #0,page_table+0x474         /* 0011d000: CAS read invalid */
        move.l  #0x00035001,page_table+0x478 /* 0011e000: CAS read alias */
        move.l  #0,page_table+0x47c         /* 0011f000: CAS2 first invalid */
        move.l  #0x00037001,page_table+0x480 /* 00120000: CAS2 second */
        move.l  #0x00038001,page_table+0x484 /* 00121000: split-read first */
        move.l  #0,page_table+0x488          /* 00122000: split-read second */
        move.l  #0x0003a001,page_table+0x494 /* 00125000: FMOVEM read first */
        move.l  #0,page_table+0x498          /* 00126000: FMOVEM read second */
        move.l  #0x0003c001,page_table+0x49c /* 00127000: FMOVEM write first */
        move.l  #0,page_table+0x4a0          /* 00128000: FMOVEM write second */
        move.l  #0x0003c001,page_table+0x4a4 /* 00129000: FMOVEM first alias */
        move.l  #0x0003d001,page_table+0x4a8 /* 0012a000: FMOVEM second alias */
        move.l  #0,page_table+0x4ac          /* 0012b000: split first */
        move.l  #0x0003f001,page_table+0x4b0 /* 0012c000: split second */
        move.l  #0x00040001,page_table+0x4b4 /* 0012d000: split middle first */
        move.l  #0,page_table+0x4b8          /* 0012e000: split middle second */
        move.l  #0,page_table+0x4bc          /* 0012f000: FMOVEM first */
        move.l  #0x00042001,page_table+0x4c0 /* 00130000: FMOVEM next */
        move.l  #0,page_table+0x4c4          /* 00131000: FMOVEM write first */
        move.l  #0x00046001,page_table+0x4c8 /* 00132000: FMOVEM write next */
        move.l  #0,page_table+0x4cc          /* 00133000: CAS2 first */
        move.l  #0,page_table+0x4d0          /* 00134000: CAS2 second */
        move.l  #code_boundary_page+1,page_table+0xc00 /* 00300000: code */
        move.l  #0,page_table+0xc04          /* 00301000: B fetch fault */
        move.l  #0x00000000,page_table+0x800 /* 00200000: code invalid */

        /* Root index one is mapped for the limit-repair case. */
        move.l  page_table2_root,root_table+4
        /* Root index two initially points at an unmapped table: table bus. */
        move.l  #0xffff0002,root_table+8

        /* CRP is a 64-bit root descriptor: the low longword is the table. */
        lea     root_table,%a0
        move.l  %a0,crp_value+4
        move.l  %a0,crp_limit_value+4
        lea     crp_value,%a0
        .word   0xf010                  /* PMOVE (%a0),CRP */
        .word   0x4c00
        lea     tc_value,%a0
        .word   0xf010                  /* PMOVE (%a0),TC */
        .word   0x4000

        /* 1: long write, write-protected page. */
        move.l  #1,test_id
        move.l  #0x12345678,%d0
        move.l  %d0,0x00100000
        cmpi.l  #0x12345678,0x00020000
        bne.w   fail

        /* 2: long read, invalid page. */
        move.l  #2,test_id
        move.l  0x00101000,%d1
        cmpi.l  #0x55667788,%d1
        bne.w   fail

        /* 3: byte write, write-protected page. */
        move.l  #3,test_id
        move.l  #0xa5,%d0
        move.b  %d0,0x00102000
        cmpi.b  #0xa5,0x00022000
        bne.w   fail

        /* 4: word read, invalid page. */
        move.l  #4,test_id
        move.w  0x00103000,%d1
        cmpi.w  #0x6b7c,%d1
        bne.w   fail

        /* 5: long read whose translation succeeds but physical cycle faults. */
        move.l  #5,test_id
        move.l  0x00104000,%d1
        cmpi.l  #0xaabbccdd,%d1
        bne.w   fail

        /* 6: root limit violation, repaired by reloading the CRP. */
        move.l  #6,test_id
        lea     crp_limit_value,%a0
        .word   0xf010                  /* PMOVE (%a0),CRP */
        .word   0x4c00
        move.l  0x00400000,%d1
        cmpi.l  #0x7d8e9faf,%d1
        bne.w   fail

        /* 7: table descriptor bus error, repaired at the root. */
        move.l  #7,test_id
        move.l  0x00800000,%d1
        cmpi.l  #0x7d8e9faf,%d1
        bne.w   fail

        /* 8: instruction fetch from an invalid logical code page. */
        move.l  #8,test_id
        jmp     0x00200000

code_return:
        tst.l   code_ran
        beq.w   fail

        /* 9: complete the write half of a memory instruction in the handler,
         * clear DF, and require RTE to accept the repaired data once. */
        move.l  #9,test_id
        movea.l #0x00105000,%a1
        addq.l  #1,(%a1)
        cmpi.l  #1,0x00105000
        bne.w   fail

edited_rte_return:
        cmpi.l  #0xdeadbeef,0x00105000
        bne.w   fail

        /* 10: a distinct-address read completes before the write faults.
         * The handler invalidates the source after observing the DIB, so a
         * replay that issues the read again must enter the handler twice. */
        move.l  #10,test_id
        movea.l #0x00106000,%a1
        movea.l #0x00107000,%a2
        move.l  (%a1),(%a2)
        cmpi.l  #0x11223344,0x00107000
        bne.w   fail

        /* 11: translation of a two-word-immediate instruction crosses a
         * page boundary.  P+2 (C) is readable but P+4 (B) faults before
         * generated TCG executes, so both the saved pipeline image and its
         * address must survive translation-time exception entry. */
        move.l  #11,test_id
        move.l  #0,boundary_ran
        jmp     0x00300ffc

boundary_done:
        tst.l   boundary_ran
        beq.w   fail

        /* 12: TAS is an indivisible byte read-modify-write cycle. */
        move.l  #12,test_id
        movea.l #0x00108000,%a0
        tas     (%a0)
        /* Verify through the repaired logical mapping; 0x26000 is only the
         * physical target and is not part of the identity seed window. */
        cmpi.b  #0x80,0x00108000
        bne.w   fail

        /* 13: CAS is an indivisible long read-modify-write cycle. */
        move.l  #13,test_id
        move.l  #0x11223344,%d0
        move.l  #0x12345678,%d1
        movea.l #0x00109000,%a0
        .word   0x0ed0                  /* CAS.L D0,D1,(A0) */
        .word   0x0040
        cmpi.l  #0x12345678,0x00109000
        bne.w   fail

        /* 14: CAS2 resumes at its fourth bus phase.  The handler changes
         * the first operand after its read and first write have completed;
         * a whole-instruction replay would overwrite the sentinel. */
        move.l  #14,test_id
        moveq   #0,%d0
        move.l  #0x11111111,%d1
        move.l  #0x22222222,%d2
        move.l  #0x44444444,%d3
        movea.l #0x0010a000,%a0
        movea.l #0x0010b000,%a1
        .word   0x0efc                  /* CAS2.L D0:D1,D2:D3,(A0):(A1) */
        .word   0x8040
        .word   0x90c2
        cmpi.l  #0xdeadbeef,0x0010a000
        bne.w   fail
        cmpi.l  #0x44444444,0x0010b000
        bne.w   fail

        /* 15: the 68030 FPU double load is two long-word bus cycles. */
        move.l  #15,test_id
        movea.l #0x0010dffc,%a0
        fmove.d (%a0),%fp0
        fmove.d %fp0,fpu_result
        cmpi.l  #0x3ff00000,fpu_result
        bne.w   fail
        cmpi.l  #0,fpu_result+4
        bne.w   fail

        /* 16: MOVEP's fourth byte crosses into a repaired page. */
        move.l  #16,test_id
        movea.l #0x0010fffa,%a0
        movep.l 0(%a0),%d4
        cmpi.l  #0x11223344,%d4
        bne.w   fail

        /* 17: PMOVE CRP has two long-word memory cycles. */
        move.l  #17,test_id
        movea.l #0x00111ffc,%a0
        .word   0xf010                  /* PMOVE (A0),CRP */
        .word   0x4c00
        move.l  0x00101000,%d4
        cmpi.l  #0x55667788,%d4
        bne.w   fail

        /* 18: FMOVEM.X is three long-word bus cycles.  The first two are
         * committed before the third page translation faults. */
        move.l  #18,test_id
        movea.l #0x00113ff8,%a0
        fmovem.x %fp0,(%a0)
        /* The handler invalidates the original first page before RTE, so
         * inspect the committed operand through aliases of both physical
         * pages. */
        movea.l #0x00119ff8,%a0
        fmovem.x (%a0),%fp0
        fmove.d %fp0,fpu_result
        cmpi.l  #0x3ff00000,fpu_result
        bne.w   fail
        cmpi.l  #0,fpu_result+4
        bne.w   fail

        /* 19: a generic long load crossing a page commits its first word,
         * then faults on the second.  The handler removes the first mapping
         * before RTE, proving a retry cannot issue that read again. */
        move.l  #19,test_id
        move.l  0x00115ffe,%d4
        cmpi.l  #0xa1b2c3d4,%d4
        bne.w   fail

        /* 20: RM-only TAS.  The handler performs the complete byte update,
         * clears DF but leaves RM set, then invalidates the target.  RTE and
         * the resumed instruction must consume that completion without a
         * second read-modify-write fault; the valid alias observes the one
         * software completion and the condition codes still describe the
         * saved byte. */
        move.l  #20,test_id
        movea.l #0x00117000,%a0
        tas     (%a0)
        cmpi.b  #0x81,0x00118000
        bne.w   fail

        /* 21: RM-only TAS read.  The handler supplies a zero DIB and maps a
         * backing byte containing 0x22.  Re-reading the byte would lose the
         * saved Z condition; the repaired operation must use DIB and issue
         * only its write half. */
        move.l  #21,test_id
        movea.l #0x0011b000,%a0
        tas     (%a0)
        beq.s   1f
        bra.w   fail
1:
        cmpi.b  #0x80,0x0011c000
        bne.w   fail

        /* 22: CAS read completion consumes a handler-supplied DIB.  The
         * backing byte is deliberately a nonmatching value, so a replayed
         * read would suppress the update and expose the wrong result. */
        move.l  #22,test_id
        move.l  #0x11111111,%d0
        move.l  #0x12345678,%d1
        movea.l #0x0011d000,%a0
        .word   0x0ed0                  /* CAS.L D0,D1,(A0) */
        .word   0x0040
        cmpi.l  #0x12345678,0x0011e000
        bne.w   fail

        /* 23: CAS2 read completion supplies DIB for the first compare while
         * the second compare continues with its ordinary read. */
        move.l  #23,test_id
        move.l  #0x11111111,%d0
        move.l  #0xaaaaaaa1,%d1
        move.l  #0x33333333,%d2
        move.l  #0xbbbbbbb2,%d3
        movea.l #0x0011f000,%a0
        movea.l #0x00120000,%a1
        .word   0x0efc                  /* CAS2.L D0:D2,D1:D3,(A0):(A1) */
        .word   0x8040
        .word   0x90c2
        cmpi.l  #0xaaaaaaa1,0x0011f000
        bne.w   fail
        cmpi.l  #0xbbbbbbb2,0x00120000
        bne.w   fail

        /* 24: a DF-cleared split load supplies the failed word in DIB.  The
         * first word is made inaccessible before RTE, so a retry that does
         * not consume the committed fragment faults again; the second word
         * must come from the repaired frame image, not a stale zero slot. */
        move.l  #24,test_id
        move.l  0x00121ffe,%d4
        cmpi.l  #0xa1b2c3d4,%d4
        bne.w   fail

        /* 25: a DF-cleared FMOVEM.X load has two committed long-word
         * transfers and a handler-supplied third DIB word. */
        move.l  #25,test_id
        movea.l #0x00125ff8,%a0
        fmovem.x (%a0),%fp1
        fmove.x %fp1,fpu_extended
        cmpi.l  #0x3fff0000,fpu_extended
        bne.w   fail
        cmpi.l  #0x80000000,fpu_extended+4
        bne.w   fail
        cmpi.l  #0x12345678,fpu_extended+8
        bne.w   fail

        /* 26: a DF-cleared FMOVEM.X store completes its failed third word in
         * the handler.  The two earlier words are invalidated before RTE, so
         * replaying the committed prefix would fault a second time. */
        move.l  #26,test_id
        movea.l #0x00127ff8,%a0
        fmovem.x %fp1,(%a0)
        cmpi.l  #0x3fff0000,0x00129ff8
        bne.w   fail
        cmpi.l  #0x80000000,0x00129ffc
        bne.w   fail
        cmpi.l  #0x12345678,0x0012a000
        bne.w   fail

        /* 27: a DF-cleared split load faults on its first word.  The second
         * word is not yet executed and must remain a real bus cycle after
         * RTE rather than being discarded as part of the completed prefix. */
        move.l  #27,test_id
        move.l  0x0012bffe,%d4
        cmpi.l  #0xa1b2c3d4,%d4
        bne.w   fail

        /* 28: an odd long split load faults on its middle word.  The first
         * byte has committed and the final byte must still be transferred
         * after the handler supplies the failed middle DIB. */
        move.l  #28,test_id
        move.l  0x0012dfff,%d4
        cmpi.l  #0x11223344,%d4
        bne.w   fail

        /* 29: FMOVEM.X read faults on its first long word.  The handler's
         * DIB supplies word zero while words one and two remain pending. */
        move.l  #29,test_id
        movea.l #0x0012fffc,%a0
        fmovem.x (%a0),%fp2
        fmove.x %fp2,fpu_extended
        cmpi.l  #0x3fff0000,fpu_extended
        bne.w   fail
        cmpi.l  #0x80000000,fpu_extended+4
        bne.w   fail
        cmpi.l  #0x12345678,fpu_extended+8
        bne.w   fail

        /* 30: FMOVEM.X write faults on its first long word.  The handler
         * completes that word with DOB; later words must still be written. */
        move.l  #30,test_id
        movea.l #0x00131ffc,%a0
        fmovem.x %fp1,(%a0)
        cmpi.l  #0x3fff0000,0x00131ffc
        bne.w   fail
        cmpi.l  #0x80000000,0x00132000
        bne.w   fail
        cmpi.l  #0x12345678,0x00132004
        bne.w   fail

        /* 31: CAS2's first read is completed from a handler-supplied DIB,
         * then its second read faults.  The second frame must not inherit
         * the first DIB, and must consume its own repaired value exactly
         * once before both conditional writes execute. */
        move.l  #31,test_id
        move.l  #0x11111111,%d0
        move.l  #0xaaaaaaa1,%d1
        move.l  #0x22222222,%d2
        move.l  #0xbbbbbbb2,%d3
        movea.l #0x00133000,%a0
        movea.l #0x00134000,%a1
        .word   0x0efc                  /* CAS2.L D0:D1,D2:D3,(A0):(A1) */
        .word   0x8040
        .word   0x90c2
        cmpi.l  #0xaaaaaaa1,0x00133000
        bne.w   fail
        cmpi.l  #0xbbbbbbb2,0x00134000
        bne.w   fail

        /* 32: keep five late-handler faults live at once.  The fifth
         * architectural frame must still be stacked when the bounded
         * restart sidecar is full; it simply has no hidden continuation. */
        move.l  #32,test_id
        move.l  0x00150000,%d4
        cmpi.l  #0x12345678,%d4
        bne.w   fail
        cmpi.l  #37,fault_count
        bne.w   fail

        /* RTE must turn malformed long and unknown frames into vector 14
         * while leaving each attempted frame available to the handler. */
        lea     format_handler,%a0
        move.l  %a0,0x38
        move.l  #0,format_count
        movea.l #bad_frame_version,%sp
        rte

        /* Keep the halt MMIO outside the small translated test image. */
        lea     tc_disabled,%a0
        .word   0xf010                  /* PMOVE (%a0),TC */
        .word   0x4000
        movea.l #0xff009004,%a0
        moveq   #2,%d0                   /* VIRT_CTRL_CMD_HALT */
        move.l  %d0,(%a0)
success:
        bra.s   success

/*
 * The frame is still at the active SP on entry.  Save every general register
 * below it so the suspended instruction sees no handler side effects.
 */
access_handler:
        movem.l %d0-%d7/%a0-%a6,-(%sp)
        movea.l #60,%a0
        adda.l  %sp,%a0                  /* A0 = frame start */

        move.w  6(%a0),%d0               /* format/vector */
        move.w  %d0,last_format
        move.w  10(%a0),%d1              /* SSW */
        move.w  %d1,last_ssw
        move.l  16(%a0),%d2              /* fault address */
        move.l  %d2,last_address
        move.l  24(%a0),%d3              /* data output buffer */
        move.l  %d3,last_data_output
        move.w  12(%a0),%d5              /* instruction pipe stage C */
        move.w  14(%a0),%d6              /* instruction pipe stage B */
        move.l  test_id,%d4
        addq.l  #1,fault_count

        cmpi.l  #1,%d4
        beq.w   repair_write_long
        cmpi.l  #2,%d4
        beq.w   repair_read_long
        cmpi.l  #3,%d4
        beq.w   repair_write_byte
        cmpi.l  #4,%d4
        beq.w   repair_read_word
        cmpi.l  #5,%d4
        beq.w   repair_physical_read
        cmpi.l  #6,%d4
        beq.w   repair_limit
        cmpi.l  #7,%d4
        beq.w   repair_table_bus
        cmpi.l  #8,%d4
        beq.w   repair_code
        cmpi.l  #9,%d4
        beq.w   repair_write_rmw
        cmpi.l  #10,%d4
        beq.w   repair_write_distinct
        cmpi.l  #11,%d4
        beq.w   repair_boundary_fetch
        cmpi.l  #12,%d4
        beq.w   repair_tas
        cmpi.l  #13,%d4
        beq.w   repair_cas
        cmpi.l  #14,%d4
        beq.w   repair_cas2
        cmpi.l  #15,%d4
        beq.w   repair_fpu
        cmpi.l  #16,%d4
        beq.w   repair_movep
        cmpi.l  #17,%d4
        beq.w   repair_pmove
        cmpi.l  #18,%d4
        beq.w   repair_fmovem
        cmpi.l  #19,%d4
        beq.w   repair_split
        cmpi.l  #20,%d4
        beq.w   repair_tas_complete
        cmpi.l  #21,%d4
        beq.w   repair_tas_read
        cmpi.l  #22,%d4
        beq.w   repair_cas_read
        cmpi.l  #23,%d4
        beq.w   repair_cas2_read
        cmpi.l  #24,%d4
        beq.w   repair_split_read
        cmpi.l  #25,%d4
        beq.w   repair_fmovem_read
        cmpi.l  #26,%d4
        beq.w   repair_fmovem_write
        cmpi.l  #27,%d4
        beq.w   repair_split_first_read
        cmpi.l  #28,%d4
        beq.w   repair_split_middle_read
        cmpi.l  #29,%d4
        beq.w   repair_fmovem_first_read
        cmpi.l  #30,%d4
        beq.w   repair_fmovem_first_write
        cmpi.l  #31,%d4
        beq.w   repair_cas2_two_reads
        cmpi.l  #32,%d4
        beq.w   repair_capacity
        bra.w   fail

repair_write_long:
        cmpi.l  #1,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0105,%d1             /* DF, long write, FC=5 */
        bne.w   fail
        cmpi.l  #0x00100000,%d2
        bne.w   fail
        cmpi.l  #0x12345678,%d3
        bne.w   fail
        /* The translator must preserve the actual two words fetched while
         * decoding this instruction, rather than manufacturing zeroes. */
        cmpi.w  #0x0010,%d5
        bne.w   fail
        cmpi.w  #0x0000,%d6
        bne.w   fail
        move.l  #0x00020001,page_table+0x400
        movea.l #0x00100000,%a1
        bra.w   flush_and_return

repair_read_long:
        cmpi.l  #2,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0145,%d1             /* DF, long read, FC=5 */
        bne.w   fail
        cmpi.l  #0x00101000,%d2
        bne.w   fail
        move.l  #0x00021001,page_table+0x404
        movea.l #0x00101000,%a1
        bra.w   flush_and_return

repair_write_byte:
        cmpi.l  #3,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0115,%d1             /* DF, byte write, FC=5 */
        bne.w   fail
        cmpi.l  #0x00102000,%d2
        bne.w   fail
        cmpi.l  #0x000000a5,%d3
        bne.w   fail
        move.l  #0x00022001,page_table+0x408
        movea.l #0x00102000,%a1
        bra.w   flush_and_return

repair_read_word:
        cmpi.l  #4,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0165,%d1             /* DF, word read, FC=5 */
        bne.w   fail
        cmpi.l  #0x00103000,%d2
        bne.w   fail
        move.l  #0x00023001,page_table+0x40c
        movea.l #0x00103000,%a1
        bra.w   flush_and_return

repair_physical_read:
        cmpi.l  #5,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0145,%d1
        bne.w   fail
        cmpi.l  #0x00104000,%d2
        bne.w   fail
        move.l  #0x00025001,page_table+0x410
        movea.l #0x00104000,%a1
        bra.w   flush_and_return

repair_limit:
        cmpi.l  #6,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0145,%d1             /* DF, long read, FC=5 */
        bne.w   fail
        cmpi.l  #0x00400000,%d2
        bne.w   fail
        lea     crp_value,%a0
        .word   0xf010                  /* PMOVE (%a0),CRP */
        .word   0x4c00
        movea.l #0x00400000,%a1
        bra.w   flush_and_return

repair_table_bus:
        cmpi.l  #7,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0145,%d1
        bne.w   fail
        cmpi.l  #0x00800000,%d2
        bne.w   fail
        move.l  page_table2_root,root_table+8
        movea.l #0x00800000,%a1
        bra.w   flush_and_return

repair_code:
        cmpi.l  #8,fault_count
        bne.w   fail
        /* A pipeline fault may be format A at a boundary or format B while
         * the target stage is in flight. */
        cmpi.w  #0xa008,%d0
        beq.s   1f
        cmpi.w  #0xb008,%d0
        bne.w   fail
1:
        move.w  %d1,last_code_ssw
        /* Accept the repaired C/B images instead of asking RTE to rerun
         * their prefetches.  QEMU has no separate architectural pipe
         * registers, so the resumed PC is retranslated from the repaired
         * instruction page. */
        move.w  #0x4e71,12(%a0)
        move.w  #0x4e71,14(%a0)
        andi.w  #0x8fff,10(%a0)
        move.l  #code_repair+1,page_table+0x800
        movea.l #0x00200000,%a1
        bra.w   flush_code_and_return

repair_write_rmw:
        cmpi.l  #9,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0105,%d1             /* DF, long write, FC=5 */
        bne.w   fail
        cmpi.l  #0x00105000,%d2
        bne.w   fail
        cmpi.l  #1,%d3
        bne.w   fail
        /* Complete the failed write in software, then clear DF. */
        move.l  #0x00026001,page_table+0x414
        movea.l #0x00105000,%a1
        movea.l %a0,%a2
        movea.l %a1,%a0
        .word   0xf010
        .word   0x38f5
        move.l  %d3,(%a1)
        movea.l %a2,%a0
        andi.w  #0xfeff,10(%a0)
        move.l  #edited_rte_pc,2(%a0)
        movem.l (%sp)+,%d0-%d7/%a0-%a6
        rte

edited_rte_pc:
        move.l  #0xdeadbeef,0x00105000
        movea.l #edited_rte_return,%a0
        jmp     (%a0)

repair_write_distinct:
        cmpi.l  #10,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0105,%d1             /* DF, long write, FC=5 */
        bne.w   fail
        cmpi.l  #0x00107000,%d2
        bne.w   fail
        cmpi.l  #0x11223344,%d3
        bne.w   fail
        /* Make a repeated source read observable: it is no longer mapped
         * when RTE re-enters the instruction. */
        move.l  #0,page_table+0x418
        move.l  #0x00028001,page_table+0x41c
        movea.l %a0,%a2
        movea.l #0x00106000,%a1
        movea.l %a1,%a0
        .word   0xf010
        .word   0x38f5
        movea.l #0x00107000,%a1
        movea.l %a1,%a0
        .word   0xf010
        .word   0x38f5
        move.l  %d3,(%a1)
        movea.l %a2,%a0
        andi.w  #0xfeff,10(%a0)
        movem.l (%sp)+,%d0-%d7/%a0-%a6
        rte

repair_boundary_fetch:
        cmpi.l  #11,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x5000,%d1             /* FB/RB, supervisor program */
        bne.w   fail
        cmpi.l  #0x00301000,%d2          /* failed fetch at P + 4 */
        bne.w   fail
        cmpi.w  #0x1234,%d5             /* C = P + 2 */
        bne.w   fail
        cmpi.w  #0,%d6                   /* B was the failed fetch */
        bne.w   fail
        cmpi.l  #0x00301000,0x24(%a0)   /* B address = P + 4 */
        bne.w   fail
        /* The repaired tail deliberately contains a different low immediate
         * word.  RTE must consume these handler-supplied C/B values. */
        move.w  #0x1234,12(%a0)
        move.w  #0x5678,14(%a0)
        andi.w  #0xafff,10(%a0)          /* clear FB/RB */
        move.l  #code_boundary_tail+1,page_table+0xc04
        movea.l #0x00301000,%a1
        bra.w   flush_code_and_return

repair_tas:
        cmpi.l  #12,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0195,%d1             /* DF, RM, byte write, FC=5 */
        bne.w   fail
        cmpi.l  #0x00108000,%d2
        bne.w   fail
        cmpi.l  #0x00000080,%d3
        bne.w   fail
        move.l  #0x00026001,page_table+0x420
        movea.l #0x00108000,%a1
        bra.w   flush_and_return

repair_cas:
        cmpi.l  #13,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0185,%d1             /* DF, RM, long RMW, FC=5 */
        bne.w   fail
        cmpi.l  #0x00109000,%d2
        bne.w   fail
        cmpi.l  #0x12345678,%d3
        bne.w   fail
        move.l  #0x00027001,page_table+0x424
        movea.l #0x00109000,%a1
        bra.w   flush_and_return

repair_cas2:
        cmpi.l  #14,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0185,%d1             /* fourth CAS2 long RMW phase */
        bne.w   fail
        cmpi.l  #0x0010b000,%d2
        bne.w   fail
        move.l  #0x00029001,page_table+0x42c
        move.l  #0xdeadbeef,0x0010a000
        movea.l #0x0010b000,%a1
        bra.w   flush_and_return

repair_fpu:
        cmpi.l  #15,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0145,%d1             /* DF, long read, FC=5 */
        bne.w   fail
        cmpi.l  #0x0010e000,%d2
        bne.w   fail
        move.l  #0x0002c001,page_table+0x438
        movea.l #0x0010e000,%a1
        bra.w   flush_and_return

repair_movep:
        cmpi.l  #16,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0155,%d1             /* DF, byte read, FC=5 */
        bne.w   fail
        cmpi.l  #0x00110000,%d2
        bne.w   fail
        move.l  #0x0002e001,page_table+0x440
        movea.l #0x00110000,%a1
        bra.w   flush_and_return

repair_pmove:
        cmpi.l  #17,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0145,%d1             /* DF, long read, FC=5 */
        bne.w   fail
        cmpi.l  #0x00112000,%d2
        bne.w   fail
        move.l  #0x00300001,page_table+0x448
        movea.l #0x00112000,%a1
        bra.w   flush_and_return

repair_fmovem:
        cmpi.l  #18,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0105,%d1             /* DF, long write, FC=5 */
        bne.w   fail
        cmpi.l  #0x00114000,%d2
        bne.w   fail
        move.l  #0x00031001,page_table+0x450
        /* Invalidate both the committed first page and the repaired second
         * page before RTE.  A retry which re-issues either of the first two
         * long-word cycles must therefore fault again; the saved cycle
         * phase is the only way to complete this instruction. */
        move.l  #0,page_table+0x44c
        movea.l #0x00113000,%a0
        .word   0xf010
        .word   0x38f5
        movea.l #0x00114000,%a0
        .word   0xf010
        .word   0x38f5
        movem.l (%sp)+,%d0-%d7/%a0-%a6
        rte

repair_split:
        cmpi.l  #19,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0165,%d1             /* DF, word read, FC=5 */
        bne.w   fail
        cmpi.l  #0x00116000,%d2
        bne.w   fail
        /* Make the first, already committed fragment untranslatable. */
        move.l  #0,page_table+0x454
        move.l  #0x00032001,page_table+0x458
        movea.l #0x00115000,%a0
        .word   0xf010                  /* PFLUSH first page, FC=5 */
        .word   0x38f5
        movea.l #0x00116000,%a1
        bra.w   flush_and_return

repair_tas_complete:
        cmpi.l  #20,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0195,%d1             /* DF, RM, byte write, FC=5 */
        bne.w   fail
        cmpi.l  #0x00117000,%d2
        bne.w   fail
        cmpi.l  #0x00000080,%d3
        bne.w   fail
        movea.l %a0,%a2
        move.l  #0x00033001,page_table+0x45c
        movea.l #0x00117000,%a1
        movea.l %a1,%a0
        .word   0xf010                  /* PFLUSH target before software op */
        .word   0x38f5
        move.b  #0x81,(%a1)
        move.l  #0,page_table+0x45c
        movea.l %a1,%a0
        .word   0xf010                  /* no target access on resumed TAS */
        .word   0x38f5
        movea.l %a2,%a0
        andi.w  #0xfeff,10(%a0)          /* leave RM set, clear only DF */
        movem.l (%sp)+,%d0-%d7/%a0-%a6
        rte

repair_tas_read:
        cmpi.l  #21,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x01d5,%d1             /* RM, byte read, FC=5 */
        bne.w   fail
        cmpi.l  #0x0011b000,%d2
        bne.w   fail
        /* Supply DIB=0 while leaving RM set and clearing DF. */
        move.l  #0,0x2c(%a0)
        move.l  #0x00034001,page_table+0x46c
        movea.l #0x0011b000,%a1
        movea.l %a0,%a2
        movea.l %a1,%a0
        .word   0xf010
        .word   0x38f5
        movea.l %a2,%a0
        andi.w  #0xfeff,10(%a0)
        movem.l (%sp)+,%d0-%d7/%a0-%a6
        rte

repair_cas_read:
        cmpi.l  #22,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x01c5,%d1             /* RM, long read, FC=5 */
        bne.w   fail
        cmpi.l  #0x0011d000,%d2
        bne.w   fail
        move.l  #0x11111111,0x2c(%a0)  /* Handler-supplied DIB. */
        move.l  #0x00035001,page_table+0x474
        movea.l #0x0011d000,%a1
        movea.l %a0,%a2
        movea.l %a1,%a0
        .word   0xf010
        .word   0x38f5
        movea.l %a2,%a0
        andi.w  #0xfeff,10(%a0)
        movem.l (%sp)+,%d0-%d7/%a0-%a6
        rte

repair_cas2_read:
        cmpi.l  #23,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x01c5,%d1             /* RM, long read, FC=5 */
        bne.w   fail
        cmpi.l  #0x0011f000,%d2
        bne.w   fail
        move.l  #0x11111111,0x2c(%a0)  /* Handler-supplied first DIB. */
        move.l  #0x00036001,page_table+0x47c
        movea.l #0x0011f000,%a1
        movea.l %a0,%a2
        movea.l %a1,%a0
        .word   0xf010
        .word   0x38f5
        movea.l %a2,%a0
        andi.w  #0xfeff,10(%a0)
        movem.l (%sp)+,%d0-%d7/%a0-%a6
        rte

repair_split_read:
        cmpi.l  #24,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0165,%d1             /* DF, word read, FC=5 */
        bne.w   fail
        cmpi.l  #0x00122000,%d2
        bne.w   fail
        move.l  #0x0000c3d4,0x2c(%a0)   /* Handler-supplied failed DIB. */
        move.l  #0,page_table+0x484
        move.l  #0x00039001,page_table+0x488
        movea.l %a0,%a2
        movea.l #0x00121000,%a1
        movea.l %a1,%a0
        .word   0xf010                  /* flush removed first fragment */
        .word   0x38f5
        movea.l #0x00122000,%a1
        movea.l %a1,%a0
        .word   0xf010                  /* flush repaired second fragment */
        .word   0x38f5
        movea.l %a2,%a0
        andi.w  #0xfeff,10(%a0)          /* clear DF, preserve RW */
        movem.l (%sp)+,%d0-%d7/%a0-%a6
        rte

repair_fmovem_read:
        cmpi.l  #25,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0145,%d1             /* DF, long read, FC=5 */
        bne.w   fail
        cmpi.l  #0x00126000,%d2
        bne.w   fail
        cmpi.l  #0,%d3                   /* no DOB on a read fault */
        bne.w   fail
        cmpi.l  #0,0x2c(%a0)             /* DIB is filled by this handler */
        bne.w   fail
        move.l  #0x12345678,0x2c(%a0)    /* Supply the failed third word. */
        move.l  #0,page_table+0x494
        move.l  #0x0003d001,page_table+0x498
        movea.l %a0,%a2
        movea.l #0x00125000,%a1
        movea.l %a1,%a0
        .word   0xf010                  /* flush invalidated first operand */
        .word   0x38f5
        movea.l #0x00126000,%a1
        movea.l %a1,%a0
        .word   0xf010                  /* flush repaired failed operand */
        .word   0x38f5
        movea.l %a2,%a0
        andi.w  #0xfeff,10(%a0)          /* clear DF, preserve RW */
        movem.l (%sp)+,%d0-%d7/%a0-%a6
        rte

repair_fmovem_write:
        cmpi.l  #26,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0105,%d1             /* DF, long write, FC=5 */
        bne.w   fail
        cmpi.l  #0x00128000,%d2
        bne.w   fail
        cmpi.l  #0x12345678,%d3          /* DOB for the failed third word */
        bne.w   fail
        movea.l %a0,%a2
        move.l  #0,page_table+0x49c
        move.l  #0x0003d001,page_table+0x4a0
        movea.l #0x00127000,%a1
        movea.l %a1,%a0
        .word   0xf010                  /* flush invalidated first operand */
        .word   0x38f5
        movea.l #0x00128000,%a1
        movea.l %a1,%a0
        .word   0xf010                  /* flush repaired failed operand */
        .word   0x38f5
        movea.l #0x00128000,%a1
        move.l  %d3,(%a1)               /* complete the failed word in SW */
        movea.l %a2,%a0
        andi.w  #0xfeff,10(%a0)          /* clear DF, preserve write */
        movem.l (%sp)+,%d0-%d7/%a0-%a6
        rte

repair_split_first_read:
        cmpi.l  #27,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0165,%d1             /* DF, word read, FC=5 */
        bne.w   fail
        cmpi.l  #0x0012bffe,%d2
        bne.w   fail
        move.l  #0xa1b2,0x2c(%a0)       /* Complete the first word in DIB. */
        move.l  #0x0003e001,page_table+0x4ac
        movea.l %a0,%a2
        movea.l #0x0012b000,%a1
        movea.l %a1,%a0
        .word   0xf010                  /* flush repaired first fragment */
        .word   0x38f5
        movea.l %a2,%a0
        andi.w  #0xfeff,10(%a0)          /* clear DF, preserve RW */
        movem.l (%sp)+,%d0-%d7/%a0-%a6
        rte

repair_split_middle_read:
        cmpi.l  #28,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0165,%d1             /* DF, word read, FC=5 */
        bne.w   fail
        cmpi.l  #0x0012e000,%d2
        bne.w   fail
        move.l  #0x2233,0x2c(%a0)       /* Complete the middle word in DIB. */
        move.l  #0x00041001,page_table+0x4b8
        movea.l %a0,%a2
        movea.l #0x0012e000,%a1
        movea.l %a1,%a0
        .word   0xf010                  /* flush repaired middle fragment */
        .word   0x38f5
        movea.l %a2,%a0
        andi.w  #0xfeff,10(%a0)          /* clear DF, preserve RW */
        movem.l (%sp)+,%d0-%d7/%a0-%a6
        rte

repair_fmovem_first_read:
        cmpi.l  #29,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0145,%d1             /* DF, long read, FC=5 */
        bne.w   fail
        cmpi.l  #0x0012fffc,%d2
        bne.w   fail
        move.l  #0x3fff0000,0x2c(%a0)   /* Complete the first word in DIB. */
        move.l  #0x00043001,page_table+0x4bc
        movea.l %a0,%a2
        movea.l #0x0012f000,%a1
        movea.l %a1,%a0
        .word   0xf010                  /* flush repaired first operand */
        .word   0x38f5
        movea.l %a2,%a0
        andi.w  #0xfeff,10(%a0)          /* clear DF, preserve RW */
        movem.l (%sp)+,%d0-%d7/%a0-%a6
        rte

repair_fmovem_first_write:
        cmpi.l  #30,fault_count
        bne.w   fail
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0105,%d1             /* DF, long write, FC=5 */
        bne.w   fail
        cmpi.l  #0x00131ffc,%d2
        bne.w   fail
        cmpi.l  #0x3fff0000,%d3         /* DOB for the first long word */
        bne.w   fail
        move.l  #0x00047001,page_table+0x4c4
        movea.l %a0,%a2
        movea.l #0x00131000,%a1
        movea.l %a1,%a0
        .word   0xf010                  /* flush repaired first operand */
        .word   0x38f5
        movea.l #0x00131ffc,%a1
        move.l  %d3,(%a1)               /* complete the failed word in SW */
        movea.l %a2,%a0
        andi.w  #0xfeff,10(%a0)          /* clear DF, preserve write */
        movem.l (%sp)+,%d0-%d7/%a0-%a6
        rte

repair_cas2_two_reads:
        cmpi.l  #31,%d4
        bne.w   fail
        cmpi.l  #31,fault_count
        beq.w   repair_cas2_first_read
        cmpi.l  #32,fault_count
        beq.w   repair_cas2_second_read
        bra.w   fail

repair_cas2_first_read:
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x01c5,%d1             /* RM, long read, FC=5 */
        bne.w   fail
        cmpi.l  #0x00133000,%d2
        bne.w   fail
        move.l  #0x11111111,0x2c(%a0)   /* Complete the first read in DIB. */
        move.l  #0x00048001,page_table+0x4cc
        movea.l %a0,%a2
        movea.l #0x00133000,%a1
        movea.l %a1,%a0
        .word   0xf010                  /* flush repaired first read */
        .word   0x38f5
        movea.l %a2,%a0
        andi.w  #0xfeff,10(%a0)          /* clear DF, preserve RM */
        movem.l (%sp)+,%d0-%d7/%a0-%a6
        rte

repair_cas2_second_read:
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x01c5,%d1             /* RM, long read, FC=5 */
        bne.w   fail
        cmpi.l  #0x00134000,%d2
        bne.w   fail
        cmpi.l  #0,0x2c(%a0)            /* No stale first-read DIB allowed. */
        bne.w   fail
        move.l  #0x22222222,0x2c(%a0)   /* Complete the second read in DIB. */
        move.l  #0x00049001,page_table+0x4d0
        movea.l %a0,%a2
        movea.l #0x00134000,%a1
        movea.l %a1,%a0
        .word   0xf010                  /* flush repaired second read */
        .word   0x38f5
        movea.l %a2,%a0
        andi.w  #0xfeff,10(%a0)          /* clear DF, preserve RM */
        movem.l (%sp)+,%d0-%d7/%a0-%a6
        rte

repair_capacity:
        cmpi.w  #0xb008,%d0
        bne.w   fail
        cmpi.w  #0x0145,%d1             /* DF, long read, FC=5 */
        bne.w   fail
        cmpi.l  #33,fault_count
        beq.w   capacity_1
        cmpi.l  #34,fault_count
        beq.w   capacity_2
        cmpi.l  #35,fault_count
        beq.w   capacity_3
        cmpi.l  #36,fault_count
        beq.w   capacity_4
        cmpi.l  #37,fault_count
        beq.w   capacity_5
        bra.w   fail

capacity_1:
        cmpi.l  #0x00150000,%d2
        bne.w   fail
        move.l  0x00151000,%d7
        move.l  #0x00020001,page_table+0x540
        movea.l #0x00150000,%a1
        bra.w   flush_and_return

capacity_2:
        cmpi.l  #0x00151000,%d2
        bne.w   fail
        move.l  0x00152000,%d7
        move.l  #0x00020001,page_table+0x544
        movea.l #0x00151000,%a1
        bra.w   flush_and_return

capacity_3:
        cmpi.l  #0x00152000,%d2
        bne.w   fail
        move.l  0x00153000,%d7
        move.l  #0x00020001,page_table+0x548
        movea.l #0x00152000,%a1
        bra.w   flush_and_return

capacity_4:
        cmpi.l  #0x00153000,%d2
        bne.w   fail
        move.l  0x00154000,%d7
        move.l  #0x00020001,page_table+0x54c
        movea.l #0x00153000,%a1
        bra.w   flush_and_return

capacity_5:
        cmpi.l  #0x00154000,%d2
        bne.w   fail
        move.l  #0x00020001,page_table+0x550
        movea.l #0x00154000,%a1
        bra.w   flush_and_return

flush_and_return:
        /* PFLUSH page, supervisor data FC (#5), then RTE-owned retry. */
        movea.l %a1,%a0
        .word   0xf010
        .word   0x38f5
        movem.l (%sp)+,%d0-%d7/%a0-%a6
        rte

flush_code_and_return:
        /* Instruction fetches use supervisor program FC (#6). */
        movea.l %a1,%a0
        .word   0xf010
        .word   0x38f6
        movem.l (%sp)+,%d0-%d7/%a0-%a6
        rte

fail:
        bra.s   fail

        .balign 0x1000
code_repair:
        move.l  #1,code_ran
        /* This page is fetched at a relocated logical address.  Use an
         * absolute target so the hand-built physical image does not apply
         * the link-time PC-relative displacement twice. */
        movea.l #code_return,%a0
        jmp     (%a0)

format_handler:
        addq.l  #1,format_count
        cmpi.l  #1,format_count
        bne.s   1f
        cmpi.w  #0xb008,bad_frame_version+6
        bne.w   fail
        cmpi.w  #0x2000,bad_frame_version+0x36
        bne.w   fail
        bra.s   format_try_unknown
1:
        cmpi.l  #2,format_count
        bne.w   fail
        cmpi.w  #0xc008,bad_frame_unknown+6
        bne.w   fail
        bra.s   format_try_copro

format_try_unknown:
        movea.l #bad_frame_unknown,%sp
        rte

format_try_copro:
        movea.l #good_frame_coprocessor,%sp
        rte

format9_return:
        cmpi.l  #2,format_count
        bne.w   fail

format_success:
        lea     tc_disabled,%a0
        .word   0xf010                  /* PMOVE (%a0),TC */
        .word   0x4000
        movea.l #0xff009004,%a0
        moveq   #2,%d0                   /* VIRT_CTRL_CMD_HALT */
        move.l  %d0,(%a0)

format_unexpected_return:
        bra.w   fail

        .data
        .balign 4
crp_value:
        /* Valid 4-byte table root, limit covering both root indices used. */
        .long   0x7fff0002,0
crp_limit_value:
        /* Same table with a zero upper limit: root index one must fault. */
        .long   0x00000002,0
tc_value:
        .long   0x80c0aa00
tc_disabled:
        .long   0
page_table2_root:
        .long   0
test_id:
        .long   0
fault_count:
        .long   0
code_ran:
        .long   0
last_format:
        .word   0
last_ssw:
        .word   0
last_code_ssw:
        .word   0
last_address:
        .long   0
last_data_output:
        .long   0
fpu_result:
        .long   0, 0
fpu_extended:
        .long   0, 0, 0
format_count:
        .long   0
boundary_ran:
        .long   0

        .balign 0x100
bad_frame_version:
        .word   0x2700
        .long   format_unexpected_return
        .word   0xb008
        .space  0x2e
        .word   0x2000                  /* invalid MC68030 frame version */
        .space  0x24

        .balign 4
bad_frame_unknown:
        .word   0x2700
        .long   format_unexpected_return
        .word   0xc008                  /* unknown RTE frame format */
        .space  0x54

        .balign 4
good_frame_coprocessor:
        .word   0x2700
        .long   format9_return
        .word   0x9008
        .long   0x00123456              /* coprocessor instruction address */
        .space  8                         /* internal registers */

        .balign 0x1000
code_boundary_page:
        .space  0xffc
        .word   0x203c                  /* move.l #imm32,%d0 */
        .word   0x1234                  /* C at P + 2 */

        .balign 0x1000
code_boundary_tail:
        .word   0xdead                  /* differs from handler-supplied B */
        movea.l #boundary_return,%a0
        jmp     (%a0)

boundary_return:
        cmpi.l  #0x12345678,%d0
        bne.w   fail
        move.l  #1,boundary_ran
        movea.l #boundary_done,%a0
        jmp     (%a0)

        .balign 0x1000
root_table:
        .space  0x1000
page_table:
        .space  0x1000
page_table2:
        .space  0x1000

        .balign 0x1000
stack_storage:
        .space  0x1000
stack_top:
