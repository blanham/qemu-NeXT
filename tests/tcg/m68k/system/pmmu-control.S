/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * First MC68030 PMMU control-opcode probe.  PLOADR must perform only the
 * translation-table walk, install the resulting entry in the ATC, and leave
 * MMUSR untouched.  The later PFLUSH checks use several physical pages so
 * that both architectural ATC selection and the derived QEMU TLB can be
 * observed from guest code.
 */

        .text
        .globl  _start

        /* PTEST level zero exposes whether one architectural ATC entry is
         * still resident after a flush.  A hit reports the entry's M bit,
         * while a miss reports MMUSR.I. */
        .macro  check_ptest address, extension, expected
        movea.l #\address,%a0
        .word   0xf010
        .word   \extension
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #\expected,%d0
        bne.w   fail
        .endm

_start:
        lea     fault_handler,%a0
        move.l  %a0,0x08           /* bus error */
        move.l  %a0,0x0c           /* address error */
        move.l  %a0,0x10           /* illegal instruction */
        move.l  %a0,0x2c           /* F-line */
        move.l  %a0,0xe0           /* MMU configuration */

        /* Physical values used by the PFLUSH and derived-TLB checks. */
        movea.l #0x00020000,%a0
        move.l  #0x11112222,(%a0)
        movea.l #0x00030000,%a0
        move.l  #0x33334444,(%a0)
        movea.l #0x00040000,%a0
        move.l  #0x55556666,(%a0)

        /* Root descriptor zero selects the four-byte page table. */
        lea     root_table,%a0
        lea     page_table,%a1
        move.l  %a1,%d0
        ori.l   #2,%d0
        move.l  %d0,(%a0)

        /* Identity-map the image and tables, then map two test targets. */
        moveq   #1,%d0
        moveq   #31,%d1
1:
        move.l  %d0,(%a1)+
        addi.l  #0x1000,%d0
        dbra    %d1,1b
        lea     page_table,%a1
        move.l  #0x00020001,0x400(%a1)
        move.l  #0x00030001,0x404(%a1)

        lea     root_table,%a0
        lea     crp_value+4,%a1
        move.l  %a0,(%a1)

        /* CRP selects the root table; TC selects 4 KiB, 10+10-bit tables. */
        lea     crp_value,%a0
        .word   0xf010                  /* PMOVE (%a0),CRP */
        .word   0x4c00
        lea     tc_value,%a0
        .word   0xf010                  /* PMOVE (%a0),TC */
        .word   0x4000

        /* Seed MMUSR.  PLOAD must not alter this value. */
        lea     mmusr_value+2,%a0
        .word   0xf028                  /* PMOVE (-2,%a0),MMUSR */
        .word   0x6000
        .word   0xfffe

        /* PLOADR #1,(A0): F010 / 2211 (R/W=read, FC=10001). */
        movea.l #0x00100000,%a0
        .word   0xf010
        .word   0x2211

        /* Read back MMUSR before any instruction that can overwrite it. */
        lea     mmusr_after_pload+2,%a1
        .word   0xf029                  /* PMOVE MMUSR,(-2,%a1) */
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_pload,%d0
        cmpi.w  #0xa55a,%d0
        bne.w   fail

        /* PLOADW D0,(A0): F010 / 2008 (R/W=write, FC=01000). */
        moveq   #1,%d0
        movea.l #0x00100000,%a0
        .word   0xf010
        .word   0x2008

        /* PLOADW must also preserve MMUSR and set M in the ATC entry. */
        lea     mmusr_after_pload+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_pload,%d0
        cmpi.w  #0xa55a,%d0
        bne.w   fail

        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTR #1,(%a0),#0 */
        .word   0x8211
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0200,%d0              /* PLOADW marks the ATC modified. */
        bne.w   fail

        /* PLOADR SFC,(A0): F010 / 2200 (SFC is set to supervisor data FC=5). */
        moveq   #5,%d0
        .word   0x4e7b                  /* MOVEC D0,SFC */
        .word   0x0000
        movea.l #0x00100000,%a0
        .word   0xf010
        .word   0x2200

        /* PTESTR #5,(A0),#0 confirms a separate SFC-selected ATC entry.
         * The page's M bit is already set by the earlier PLOADW. */
        movea.l #0x00100000,%a0
        .word   0xf010
        .word   0x8215
        lea     mmusr_after_sfc+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_sfc,%d0
        cmpi.w  #0x0200,%d0
        bne.w   fail

        /* PLOADW DFC,(A0): F010 / 2001 (DFC is supervisor program FC=6). */
        moveq   #6,%d0
        .word   0x4e7b                  /* MOVEC D0,DFC */
        .word   0x0001
        movea.l #0x00100000,%a0
        .word   0xf010
        .word   0x2001

        /* PTESTR #6,(A0),#0 confirms DFC-selected PLOADW and M. */
        movea.l #0x00100000,%a0
        .word   0xf010
        .word   0x8216
        lea     mmusr_after_dfc+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_dfc,%d0
        cmpi.w  #0x0200,%d0
        bne.w   fail

        /* Level-zero PTEST must see the entry modified by PLOADW. */
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTR #1,(%a0),#0 */
        .word   0x8211
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029                  /* PMOVE MMUSR,(-2,%a1) */
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0200,%d0             /* ATC hit retains PLOADW's M bit. */
        bne.w   fail

        /* Table PTEST ignores the ATC and reports the number of descriptors
         * searched.  A two-level tree terminates at the page descriptor, so
         * levels 2..7 all report N=2 and the page's M bit. */
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTR #1,(A0),#1 */
        .word   0x8611
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0001,%d0
        bne.w   fail

        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTR #1,(A0),#2,A3 */
        .word   0x8b71
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0202,%d0
        bne.w   fail
        cmpa.l  #page_table+0x400,%a3
        bne.w   fail

        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTW #1,(A0),#1 */
        .word   0x8411
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0001,%d0
        bne.w   fail

        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTW #1,(A0),#2 */
        .word   0x8811
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0202,%d0
        bne.w   fail

        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTW #1,(A0),#3 */
        .word   0x8c11
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0202,%d0
        bne.w   fail

        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTR #1,(A0),#3 */
        .word   0x8e11
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0202,%d0
        bne.w   fail

        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTW #1,(A0),#4 */
        .word   0x9011
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0202,%d0
        bne.w   fail

        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTR #1,(A0),#4 */
        .word   0x9211
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0202,%d0
        bne.w   fail

        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTR #1,(A0),#5 */
        .word   0x9611
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0202,%d0
        bne.w   fail

        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTW #1,(A0),#5 */
        .word   0x9411
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0202,%d0
        bne.w   fail

        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTW #1,(A0),#6 */
        .word   0x9811
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0202,%d0
        bne.w   fail

        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTR #1,(A0),#6 */
        .word   0x9a11
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0202,%d0
        bne.w   fail

        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTR #1,(A0),#7 */
        .word   0x9e11
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0202,%d0
        bne.w   fail

        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTW #1,(A0),#7 */
        .word   0x9c11
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0202,%d0
        bne.w   fail

        /* The FC source forms are all legal for table PTEST. */
        moveq   #1,%d0
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTR D0,(A0),#1 */
        .word   0x8608
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0001,%d0
        bne.w   fail

        moveq   #1,%d0
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTR DFC,(A0),#1 */
        .word   0x8601
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0001,%d0
        bne.w   fail

        moveq   #1,%d0
        .word   0x4e7b                  /* MOVEC D0,SFC */
        .word   0x0000
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTR SFC,(A0),#1 */
        .word   0x8600
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0001,%d0
        bne.w   fail

        /* Level zero PTESTW remains an ATC-only probe. */
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PTESTW #1,(A0),#0 */
        .word   0x8011
        lea     mmusr_after_ptest+2,%a1
        .word   0xf029
        .word   0x6200
        .word   0xfffe
        move.w  mmusr_after_ptest,%d0
        cmpi.w  #0x0200,%d0
        bne.w   fail

        /* PLOAD must refresh the derived TLB when replacing an ATC entry. */
        movea.l #0x00100000,%a0
        move.l  (%a0),%d0
        cmpi.l  #0x11112222,%d0
        bne.w   fail
        lea     page_table,%a1
        move.l  #0x00040005,0x400(%a1)
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PLOADR #5,(A0) */
        .word   0x2215
        check_ptest 0x00100000, 0x8015, 0x0800
        move.l  (%a0),%d0
        cmpi.l  #0x55556666,%d0
        bne.w   fail
        lea     page_table,%a1
        /* Restore the M bit set by the earlier PLOADW walk. */
        move.l  #0x00020011,0x400(%a1)
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PFLUSH #5,#7,(A0) */
        .word   0x38f5

        /* PFLUSHA (F000/2400) invalidates every architectural ATC entry. */
        .word   0xf000
        .word   0x2400
        check_ptest 0x00100000, 0x8211, 0x0400
        check_ptest 0x00100000, 0x8215, 0x0400
        check_ptest 0x00100000, 0x8216, 0x0400

        /* Populate two logical pages under three function codes. */
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PLOADR #1,(A0) */
        .word   0x2211
        movea.l #0x00101000,%a0
        .word   0xf010                  /* PLOADR #1,(A0) */
        .word   0x2211

        moveq   #5,%d0
        .word   0x4e7b                  /* MOVEC D0,SFC */
        .word   0x0000
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PLOADR SFC,(A0) */
        .word   0x2200
        movea.l #0x00101000,%a0
        .word   0xf010                  /* PLOADR SFC,(A0) */
        .word   0x2200

        moveq   #6,%d0
        .word   0x4e7b                  /* MOVEC D0,DFC */
        .word   0x0001
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PLOADR DFC,(A0) */
        .word   0x2201
        movea.l #0x00101000,%a0
        .word   0xf010                  /* PLOADR DFC,(A0) */
        .word   0x2201

        /* Confirm the population itself before testing each flush scope. */
        check_ptest 0x00100000, 0x8211, 0x0200
        check_ptest 0x00101000, 0x8211, 0x0000
        check_ptest 0x00100000, 0x8215, 0x0200
        check_ptest 0x00101000, 0x8215, 0x0000
        check_ptest 0x00100000, 0x8216, 0x0200
        check_ptest 0x00101000, 0x8216, 0x0000

        /* FC/mask-only immediate form removes all FC=1 entries. */
        .word   0xf000
        .word   0x30f1                  /* PFLUSH #1,#7 */
        check_ptest 0x00100000, 0x8211, 0x0400
        check_ptest 0x00101000, 0x8211, 0x0400
        check_ptest 0x00100000, 0x8215, 0x0200
        check_ptest 0x00101000, 0x8215, 0x0000
        check_ptest 0x00100000, 0x8216, 0x0200
        check_ptest 0x00101000, 0x8216, 0x0000

        /* The SFC source selects FC=5 for the same FC/mask-only form. */
        .word   0xf000
        .word   0x30e0                  /* PFLUSH SFC,#7 */
        check_ptest 0x00100000, 0x8215, 0x0400
        check_ptest 0x00101000, 0x8215, 0x0400
        check_ptest 0x00100000, 0x8216, 0x0200
        check_ptest 0x00101000, 0x8216, 0x0000

        /* Refill FC=5, then exercise the DFC source with FC=6. */
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PLOADR SFC,(A0) */
        .word   0x2200
        movea.l #0x00101000,%a0
        .word   0xf010                  /* PLOADR SFC,(A0) */
        .word   0x2200
        .word   0xf000
        .word   0x30e1                  /* PFLUSH DFC,#7 */
        check_ptest 0x00100000, 0x8216, 0x0400
        check_ptest 0x00101000, 0x8216, 0x0400
        check_ptest 0x00100000, 0x8215, 0x0200
        check_ptest 0x00101000, 0x8215, 0x0000

        /* Refill FC=1 and select it through D0. */
        moveq   #1,%d0
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PLOADR #1,(A0) */
        .word   0x2211
        movea.l #0x00101000,%a0
        .word   0xf010                  /* PLOADR #1,(A0) */
        .word   0x2211
        .word   0xf000
        .word   0x30e8                  /* PFLUSH D0,#7 */
        check_ptest 0x00100000, 0x8211, 0x0400
        check_ptest 0x00101000, 0x8211, 0x0400
        check_ptest 0x00100000, 0x8215, 0x0200
        check_ptest 0x00101000, 0x8215, 0x0000

        /* A partial mask examines only FC2.  FC=1 is removed while FC=5
         * remains because their FC2 bits differ. */
        .word   0xf000
        .word   0x3091                  /* PFLUSH #1,#4 */
        check_ptest 0x00100000, 0x8211, 0x0400
        check_ptest 0x00101000, 0x8211, 0x0400
        check_ptest 0x00100000, 0x8215, 0x0200
        check_ptest 0x00101000, 0x8215, 0x0000

        /* Repopulate FC=1 and exercise the effective-address form. */
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PLOADR #1,(A0) */
        .word   0x2211
        movea.l #0x00101000,%a0
        .word   0xf010                  /* PLOADR #1,(A0) */
        .word   0x2211
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PFLUSH #5,#7,(A0) */
        .word   0x38f5
        check_ptest 0x00100000, 0x8215, 0x0400
        check_ptest 0x00101000, 0x8215, 0x0000
        check_ptest 0x00100000, 0x8211, 0x0200
        check_ptest 0x00101000, 0x8211, 0x0000

        /* The remaining FC sources also apply to the address form. */
        moveq   #5,%d0
        .word   0x4e7b                  /* MOVEC D0,SFC */
        .word   0x0000
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PLOADR SFC,(A0) */
        .word   0x2200
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PFLUSH SFC,#7,(A0) */
        .word   0x38e0
        check_ptest 0x00100000, 0x8215, 0x0400
        check_ptest 0x00101000, 0x8215, 0x0000

        moveq   #6,%d0
        .word   0x4e7b                  /* MOVEC D0,DFC */
        .word   0x0001
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PLOADR DFC,(A0) */
        .word   0x2201
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PFLUSH DFC,#7,(A0) */
        .word   0x38e1
        check_ptest 0x00100000, 0x8216, 0x0400

        moveq   #1,%d0
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PLOADR D0,(A0) */
        .word   0x2208
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PFLUSH D0,#7,(A0) */
        .word   0x38e8
        check_ptest 0x00100000, 0x8211, 0x0400

        /* Populate a normal supervisor-data translation, then change its
         * page descriptor.  Targeted PFLUSH must invalidate the derived
         * QEMU TLB as well as the FC=5 architectural entry. */
        movea.l #0x00100000,%a0
        move.l  (%a0),%d0
        cmpi.l  #0x11112222,%d0
        bne.w   fail
        lea     page_table,%a1
        move.l  #0x00040001,0x400(%a1)
        movea.l #0x00100000,%a0
        .word   0xf010                  /* PFLUSH #5,#7,(A0) */
        .word   0x38f5
        move.l  (%a0),%d0
        cmpi.l  #0x55556666,%d0
        bne.w   fail

        /* The first word is an EA field even for no-EA PFLUSH forms. */
        movea.l #0x00008000,%sp
        lea     no_ea_pflusha_alias,%a0
        move.l  %a0,0x2c
        .word   0xf010                  /* PFLUSHA with a nonzero EA. */
        .word   0x2400
        bra.w   fail

no_ea_pflusha_alias:
        lea     no_ea_pflush_alias,%a0
        move.l  %a0,0x2c
        .word   0xf001                  /* FC/mask PFLUSH with a nonzero EA. */
        .word   0x30f1
        bra.w   fail

no_ea_pflush_alias:
        /* Reserved PFLUSH extension bits must take the F-line vector. */
        lea     reserved_control,%a0
        move.l  %a0,0x2c
        .word   0xf000                  /* Reserved PFLUSH extension. */
        .word   0x2500
        bra.w   fail

reserved_control:
        /* PC-relative modes are not control-alterable on MC68030.  A
         * PC-relative FC+EA PFLUSH must take the F-line vector. */
        lea     invalid_control_ea,%a0
        move.l  %a0,0x2c
        .word   0xf03a                  /* PFLUSH #1,#7,(d16,PC) */
        .word   0x38f1
        .word   0x0000
        bra.w   fail

invalid_control_ea:
        /* The F-line vector is the expected architectural result. */
        lea     tc_disabled,%a0
        .word   0xf010                  /* PMOVE (%a0),TC */
        .word   0x4000
        movea.l #0xff009004,%a0
        moveq   #2,%d0                   /* VIRT_CTRL_CMD_HALT */
        move.l  %d0,(%a0)
success:
        bra.s   success

fail:
1:
        bra.s   1b

fault_handler:
2:
        bra.s   2b

        .data
tc_value:
        .long   0x80c0aa00
crp_value:
        .long   0x00000002,0
mmusr_value:
        .word   0xa55a
        .word   0
mmusr_after_pload:
        .word   0
        .word   0
mmusr_after_ptest:
        .word   0
        .word   0
mmusr_after_sfc:
        .word   0
        .word   0
mmusr_after_dfc:
        .word   0
        .word   0
tc_disabled:
        .long   0

        .balign 0x1000
root_table:
        .space  0x1000
page_table:
        .space  0x1000
