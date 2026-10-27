/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * M68040 MMU Modified-bit (M) regression.
 *
 * Mapping protocol (documented oracle):
 * - 68040 three-level tables (cpu.h): ROOT_INDEX=((addr>>23)&0x1fc),
 *   POINTER_INDEX=((addr>>16)&0x1fc), 4K_PAGE_INDEX=((addr>>10)&0xfc),
 *   8K_PAGE_INDEX=((addr>>11)&0x7c). For VA 0x00400000: ROOT=0x00,
 *   POINTER=0x40, PAGE=0x00. Hence ROOT[0x00]->PTR0 and PTR0[0x40]->PAGE1.
 * - 4K phase: TC=0x8000. Test VA 0x400000 -> PA 0x30000 clean
 *   PTE (U=0,M=0, writable). CPU read fills TLB (sets U, not M);
 *   CPU store to same VA without pflush must set M and change data.
 * - 8K phase: TC=0xC000, rebuilt tables (MMU disabled+flushed during
 *   rebuild while executing identity addresses), VA 0x400000 -> PA 0x38000.
 * - PTEs are inspected via identity-mapped PTE storage while MMU stays
 *   on, so the TLB entry under test is preserved (no flush between
 *   read and store). No qtest phys-access-only path is used; every
 *   access under test is a CPU VA access through the MMU/TLB.
 * - Buggy QEMU grants PAGE_WRITE on the read-filled TLB entry, so the
 *   later store hits the TLB and never updates PTE.M (data changes,
 *   M stays 0). Fixed QEMU must show M=1 after the store.
 * - Controls (genuine CPU/MMU): direct-first-store sets M, already-M
 *   stays set, read-only sets U not M, write-protected store faults
 *   with data unchanged and M clear, instruction-fetch-filled TLB
 *   followed by store sets M.
 * - Exit: fail_bits hex to goldfish-tty, then HALT (2) pass / PANIC (3)
 *   fail (runner uses -action panic=exit-failure). No spin/timeout relied upon.
 */

        .text
        .globl  _start
_start:
        lea     stack_top,%sp

        /* Disable all transparent translations (physical == logical off). */
        moveq   #0,%d0
        movec   %d0,%dtt0
        movec   %d0,%dtt1
        movec   %d0,%itt0
        movec   %d0,%itt1

        /* Install vector table. */
        lea     vectors,%a0
        movec   %a0,%vbr
        lea     vectors,%a0
        lea     unexpected_handler,%a1
        moveq   #0,%d0
vfill:
        move.l  %a1,(%a0)+
        addq.l  #1,%d0
        cmpi.l  #256,%d0
        bne.s   vfill
        lea     vectors,%a0
        lea     access_handler,%a1
        move.l  %a1,8(%a0)              /* vector 2: access fault */

        clr.l   fail_bits
        clr.l   fault_seen
        clr.l   unexpected
        clr.l   saved_d0
        clr.l   saved_a0

        bsr     setup_4k
        move.l  #0x8000,%d0
        movec   %d0,%tc
        pflusha
        bsr     test_4k

        /* Disable+flush before rebuilding 8K tables while running identity. */
        moveq   #0,%d0
        movec   %d0,%tc
        pflusha
        bsr     setup_8k
        move.l  #0xC000,%d0
        movec   %d0,%tc
        pflusha
        bsr     test_8k

        /* Distinct natural exits: HALT pass, PANIC fail.
         * Disable MMU first: virt-ctrl/TTY MMIO is not page-mapped.
         * Print fail_bits hex so RED is distinguishable from controls. */
        moveq   #0,%d0
        movec   %d0,%tc
        pflusha
        bsr     print_fail_bits
        tst.l   fail_bits
        bne.s   do_panic
        movea.l #0xff009004,%a0
        moveq   #2,%d0                  /* CMD_HALT */
        move.l  %d0,(%a0)
        stop    #0x2700
        illegal                         /* unreachable: HALT exits */
do_panic:
        movea.l #0xff009004,%a0
        moveq   #3,%d0                  /* CMD_PANIC */
        move.l  %d0,(%a0)
        stop    #0x2700
        illegal                         /* unreachable: PANIC exits via -action */

/* ---- table setup: 4K, identity 0..256K + test mappings ----
 * VA 0x00400000 -> ROOT[0x00] -> PTR0[0x40] -> PAGE1. */
setup_4k:
        /* ROOT at 0x20000: entry 0x00 -> PTR0. */
        movea.l #0x20000,%a0
        moveq   #0,%d0
        moveq   #0,%d1                  /* index */
rclr:
        clr.l   (%a0)+
        addq.l  #1,%d1
        cmpi.l  #128,%d1
        bne.s   rclr
        movea.l #0x20000,%a0
        move.l  #0x21002,0x00(%a0)

        /* PTR0 at 0x21000: entry 0x00 -> PAGE0, entry 0x40 -> PAGE1. */
        movea.l #0x21000,%a0
        moveq   #0,%d1
p0clr:
        clr.l   (%a0)+
        addq.l  #1,%d1
        cmpi.l  #128,%d1
        bne.s   p0clr
        movea.l #0x21000,%a0
        move.l  #0x22002,0x00(%a0)
        move.l  #0x22202,0x40(%a0)

        /* PAGE0 at 0x22000: 64 x 4K identity */
        movea.l #0x22000,%a0
        moveq   #0,%d1
        move.l  #0x00000001,%d0
pg0fill:
        move.l  %d0,(%a0)+
        addi.l  #0x1000,%d0
        addq.l  #1,%d1
        cmpi.l  #64,%d1
        bne.s   pg0fill

        /* PAGE1 at 0x22200: clear then test entries */
        movea.l #0x22200,%a0
        moveq   #0,%d1
pg1clr:
        clr.l   (%a0)+
        addq.l  #1,%d1
        cmpi.l  #64,%d1
        bne.s   pg1clr
        movea.l #0x22200,%a0
        move.l  #0x30001,0x00(%a0)       /* VA 0x400000 */
        move.l  #0x31001,0x04(%a0)       /* VA 0x401000 */
        move.l  #0x32001,0x08(%a0)       /* VA 0x402000 */
        move.l  #0x33005,0x0C(%a0)       /* VA 0x403000 WP */
        move.l  #0x34001,0x10(%a0)       /* VA 0x404000 fetch */

        /* Data pages: zero + presets */
        movea.l #0x30000,%a0
        clr.l   (%a0)
        movea.l #0x31000,%a0
        clr.l   (%a0)
        movea.l #0x32000,%a0
        clr.l   (%a0)
        movea.l #0x33000,%a0
        move.l  #0x11223344,(%a0)        /* WP preset */
        movea.l #0x34000,%a0
        move.l  #0x4E714E75,(%a0)        /* nop; rts for fetch test */
        clr.l   4(%a0)

        move.l  #0x20000,%d0
        movec   %d0,%srp
        movec   %d0,%urp
        rts

/* ---- table setup: 8K, identity 0..256K + test mappings ---- */
setup_8k:
        /* Runs with MMU off (caller disabled+flushed). Same ROOT/PTR fix. */
        movea.l #0x20000,%a0
        move.l  #0x21002,0x00(%a0)

        movea.l #0x21000,%a0
        moveq   #0,%d1
p0clr8:
        clr.l   (%a0)+
        addq.l  #1,%d1
        cmpi.l  #128,%d1
        bne.s   p0clr8
        movea.l #0x21000,%a0
        move.l  #0x22002,0x00(%a0)
        move.l  #0x22202,0x40(%a0)

        /* PAGE0: 32 x 8K identity */
        movea.l #0x22000,%a0
        moveq   #0,%d1
        move.l  #0x00000001,%d0
pg0fill8:
        move.l  %d0,(%a0)+
        addi.l  #0x2000,%d0
        addq.l  #1,%d1
        cmpi.l  #32,%d1
        bne.s   pg0fill8
        /* Clear remainder of page-table page */
        moveq   #0,%d1
pg0rest8:
        clr.l   (%a0)+
        addq.l  #1,%d1
        cmpi.l  #32,%d1
        bne.s   pg0rest8

        /* PAGE1: 8K entries */
        movea.l #0x22200,%a0
        moveq   #0,%d1
pg1clr8:
        clr.l   (%a0)+
        addq.l  #1,%d1
        cmpi.l  #64,%d1
        bne.s   pg1clr8
        movea.l #0x22200,%a0
        move.l  #0x38001,0x00(%a0)       /* VA 0x400000 */
        move.l  #0x3A001,0x04(%a0)       /* VA 0x402000 */
        move.l  #0x3C001,0x08(%a0)       /* VA 0x404000 */
        move.l  #0x3E005,0x0C(%a0)       /* VA 0x406000 WP */
        move.l  #0x40001,0x10(%a0)       /* VA 0x408000 fetch */

        movea.l #0x38000,%a0
        clr.l   (%a0)
        movea.l #0x3A000,%a0
        clr.l   (%a0)
        movea.l #0x3C000,%a0
        clr.l   (%a0)
        movea.l #0x3E000,%a0
        move.l  #0x55667788,(%a0)
        movea.l #0x40000,%a0
        move.l  #0x4E714E75,(%a0)
        clr.l   4(%a0)

        move.l  #0x20000,%d0
        movec   %d0,%srp
        movec   %d0,%urp
        rts

/* ---- 4K tests ---- */
test_4k:
        /* Main: read fills TLB (U, not M), store same VA must set M. */
        movea.l #0x400000,%a0
        movea.l #0x22200,%a1
        move.l  (%a0),%d0                /* CPU read via MMU */
        move.l  (%a1),%d1                /* PTE */
        andi.l  #0x18,%d1
        cmpi.l  #0x08,%d1                /* expect U=1,M=0 */
        bne.s   t4_main_readbad
        bra.s   t4_main_readok
t4_main_readbad:
        ori.l   #0x00000004,fail_bits     /* read wrongly set M */
t4_main_readok:
        move.l  #0xA5A5A5A5,%d0
        move.l  %d0,(%a0)                /* CPU store, no flush */
        move.l  (%a0),%d2
        cmp.l   %d0,%d2
        beq.s   t4_main_dataok
        ori.l   #0x00000002,fail_bits
t4_main_dataok:
        move.l  (%a1),%d1
        andi.l  #0x10,%d1
        bne.s   t4_main_mok
        ori.l   #0x00000001,fail_bits     /* RED bit: M missing after store */
t4_main_mok:

        /* Control: direct first store sets M. */
        movea.l #0x401000,%a0
        move.l  #0x12345678,%d0
        move.l  %d0,(%a0)
        movea.l #0x22204,%a1
        move.l  (%a1),%d1
        andi.l  #0x10,%d1
        bne.s   t4_direct_ok
        ori.l   #0x00000008,fail_bits
t4_direct_ok:

        /* Control: already-M stays set, data changes. */
        move.l  (%a0),%d1
        move.l  #0x87654321,%d0
        move.l  %d0,(%a0)
        move.l  (%a0),%d2
        cmp.l   %d0,%d2
        beq.s   t4_already_dataok
        ori.l   #0x00000020,fail_bits
t4_already_dataok:
        move.l  (%a1),%d1
        andi.l  #0x10,%d1
        bne.s   t4_already_ok
        ori.l   #0x00000010,fail_bits
t4_already_ok:

        /* Control: read-only sets U, not M. */
        movea.l #0x402000,%a0
        movea.l #0x22208,%a1
        move.l  (%a0),%d0
        move.l  (%a1),%d1
        andi.l  #0x18,%d1
        cmpi.l  #0x08,%d1
        beq.s   t4_readonly_ok
        ori.l   #0x00000040,fail_bits
t4_readonly_ok:

        /* Control: write-protected store faults, data unchanged, M clear. */
        clr.l   fault_seen
        movea.l #0x403000,%a0
        movea.l #0x2220C,%a1
        move.l  #0xDEADBEEF,%d0
        move.l  %d0,(%a0)                /* must fault (2-byte move.l Dn,(An)) */
        tst.l   fault_seen
        bne.s   t4_wp_faultok
        ori.l   #0x00000080,fail_bits
        bra.s   t4_wp_done
t4_wp_faultok:
        movea.l #0x403000,%a0
        move.l  (%a0),%d2
        cmpi.l  #0x11223344,%d2
        beq.s   t4_wp_dataok
        ori.l   #0x00000100,fail_bits
t4_wp_dataok:
        move.l  (%a1),%d1
        andi.l  #0x10,%d1
        beq.s   t4_wp_mok
        ori.l   #0x00000200,fail_bits
t4_wp_mok:
t4_wp_done:

        /* Control: fetch-filled TLB then store sets M. */
        movea.l #0x404000,%a0
        movea.l #0x22210,%a1
        jsr     (%a0)                    /* fetch via MMU, fills TLB */
        move.l  #0xCAFEBABE,%d0
        move.l  %d0,(%a0)
        move.l  (%a0),%d2
        cmp.l   %d0,%d2
        beq.s   t4_fetch_dataok
        ori.l   #0x00000800,fail_bits
t4_fetch_dataok:
        move.l  (%a1),%d1
        andi.l  #0x10,%d1
        bne.s   t4_fetch_ok
        ori.l   #0x00000400,fail_bits
t4_fetch_ok:
        rts

/* ---- 8K tests ---- */
test_8k:
        movea.l #0x400000,%a0
        movea.l #0x22200,%a1
        move.l  (%a0),%d0
        move.l  (%a1),%d1
        andi.l  #0x18,%d1
        cmpi.l  #0x08,%d1
        bne.s   t8_main_readbad
        bra.s   t8_main_readok
t8_main_readbad:
        ori.l   #0x00004000,fail_bits
t8_main_readok:
        move.l  #0x5A5A5A5A,%d0
        move.l  %d0,(%a0)
        move.l  (%a0),%d2
        cmp.l   %d0,%d2
        beq.s   t8_main_dataok
        ori.l   #0x00002000,fail_bits
t8_main_dataok:
        move.l  (%a1),%d1
        andi.l  #0x10,%d1
        bne.s   t8_main_mok
        ori.l   #0x00001000,fail_bits     /* RED bit (8K): M missing after store */
t8_main_mok:
t8_main_mdone:

        /* Direct store control on 8K VA 0x402000. */
        movea.l #0x402000,%a0
        move.l  #0x11112222,%d0
        move.l  %d0,(%a0)
        movea.l #0x22204,%a1
        move.l  (%a1),%d1
        andi.l  #0x10,%d1
        bne.s   t8_direct_ok
        ori.l   #0x00008000,fail_bits
t8_direct_ok:
        /* Already-M reuse. */
        move.l  (%a0),%d1
        move.l  #0x33334444,%d0
        move.l  %d0,(%a0)
        move.l  (%a1),%d1
        andi.l  #0x10,%d1
        bne.s   t8_already_ok
        ori.l   #0x00010000,fail_bits
t8_already_ok:

        /* Read-only on 8K VA 0x404000. */
        movea.l #0x404000,%a0
        movea.l #0x22208,%a1
        move.l  (%a0),%d0
        move.l  (%a1),%d1
        andi.l  #0x18,%d1
        cmpi.l  #0x08,%d1
        beq.s   t8_readonly_ok
        ori.l   #0x00020000,fail_bits
t8_readonly_ok:

        /* WP on 8K VA 0x406000. */
        clr.l   fault_seen
        movea.l #0x406000,%a0
        movea.l #0x2220C,%a1
        move.l  #0xDEADBEEF,%d0
        move.l  %d0,(%a0)
        tst.l   fault_seen
        bne.s   t8_wp_faultok
        ori.l   #0x00040000,fail_bits
        bra.s   t8_wp_done
t8_wp_faultok:
        movea.l #0x406000,%a0
        move.l  (%a0),%d2
        cmpi.l  #0x55667788,%d2
        beq.s   t8_wp_dataok
        ori.l   #0x00080000,fail_bits
t8_wp_dataok:
        move.l  (%a1),%d1
        andi.l  #0x10,%d1
        beq.s   t8_wp_mok
        ori.l   #0x00100000,fail_bits
t8_wp_mok:
t8_wp_done:

        /* Fetch then store on 8K VA 0x408000. */
        movea.l #0x408000,%a0
        movea.l #0x22210,%a1
        jsr     (%a0)
        move.l  #0xBEBAFECA,%d0
        move.l  %d0,(%a0)
        move.l  (%a1),%d1
        andi.l  #0x10,%d1
        bne.s   t8_fetch_ok
        ori.l   #0x00200000,fail_bits
t8_fetch_ok:
        rts

/* ---- handlers ---- */
access_handler:
        move.l  %d0,saved_d0
        move.l  %a0,saved_a0
        move.l  #1,fault_seen
        move.l  2(%sp),%d0
        addq.l  #2,%d0                   /* skip 2-byte faulting store */
        move.l  %d0,2(%sp)
        move.l  saved_d0,%d0
        move.l  saved_a0,%a0
        rte

/* Print fail_bits as FB=0x........ newline to goldfish-tty.
 * Runs with MMU off (physical). Uses 4-byte TTY writes. */
print_fail_bits:
        movem.l %d0-%d2/%a0,-(%sp)
        movea.l #0xff008000,%a0
        move.l  #0x46,%d2                /* 'F' */
        move.l  %d2,(%a0)
        move.l  #0x42,%d2                /* 'B' */
        move.l  %d2,(%a0)
        move.l  #0x3D,%d2                /* '=' */
        move.l  %d2,(%a0)
        move.l  #0x30,%d2                /* '0' */
        move.l  %d2,(%a0)
        move.l  #0x78,%d2                /* 'x' */
        move.l  %d2,(%a0)
        move.l  fail_bits,%d0
        moveq   #7,%d1
pfb_loop:
        rol.l   #4,%d0
        move.l  %d0,%d2
        andi.l  #0x0F,%d2
        cmpi.l  #10,%d2
        blt.s   pfb_digit
        addi.l  #0x37,%d2                /* 'A'-10 */
        bra.s   pfb_out
pfb_digit:
        addi.l  #0x30,%d2                /* '0' */
pfb_out:
        move.l  %d2,(%a0)
        dbf     %d1,pfb_loop
        move.l  #0x0A,%d2                /* newline */
        move.l  %d2,(%a0)
        movem.l (%sp)+,%d0-%d2/%a0
        rts

unexpected_handler:
        move.l  #1,unexpected
        ori.l   #0x80000000,fail_bits
        moveq   #0,%d0
        movec   %d0,%tc                 /* MMIO not page-mapped */
        pflusha
        bsr     print_fail_bits
        movea.l #0xff009004,%a0
        move.l  #3,(%a0)                 /* CMD_PANIC */
        stop    #0x2700
        illegal                         /* unreachable: PANIC exits via -action */

        .data
        .balign 4
fail_bits:
        .long   0
fault_seen:
        .long   0
unexpected:
        .long   0
saved_d0:
        .long   0
saved_a0:
        .long   0

        .bss
        .balign 1024
vectors:
        .space  1024
        .balign 16
stack_bottom:
        .space  4096
stack_top:
