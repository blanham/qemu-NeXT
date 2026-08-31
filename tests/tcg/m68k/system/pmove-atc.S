/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Opcode-driven MC68030 PMOVE/PMOVEFD ATC-flush regression.  The page
 * descriptor is changed while the translation is cached, then the same TC
 * value is loaded with and without the FD bit.  PMOVEFD must retain the old
 * translation; PMOVE must discard it and expose the new physical page.
 */

        .text
        .globl  _start
_start:
        movea.l #0x00008000,%sp
        /* Any unexpected exception spins so the runner reports a failure. */
        lea     fault_handler,%a0
        move.l  %a0,0x08           /* bus error */
        move.l  %a0,0x0c           /* address error */
        move.l  %a0,0x10           /* illegal instruction */
        move.l  %a0,0x2c           /* F-line */
        move.l  %a0,0xe0           /* MMU configuration */

        /* Install the two physical values before translation is enabled. */
        movea.l #0x00020000,%a0
        move.l  #0x11112222,(%a0)
        movea.l #0x00030000,%a0
        move.l  #0x33334444,(%a0)
        movea.l #0x00100000,%a0
        move.l  #0x77778888,(%a0)

        /* Root descriptor zero selects the four-byte page table. */
        lea     root_table,%a0
        lea     page_table,%a1
        move.l  %a1,%d0
        ori.l   #2,%d0
        move.l  %d0,(%a0)

        /* Identity-map the image and tables, then map VA 0x00100000. */
        moveq   #1,%d0
        moveq   #31,%d1
1:
        move.l  %d0,(%a1)+
        addi.l  #0x1000,%d0
        dbra    %d1,1b
        lea     page_table,%a1
        move.l  #0x00020001,0x400(%a1)

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

        /* This access fills the architectural ATC and derived QEMU TLB. */
        movea.l #0x00100000,%a0
        move.l  (%a0),%d0
        cmpi.l  #0x11112222,%d0
        bne.w   fail

        /* A changed TT must take priority over the populated ATC. */
        lea     tt_value,%a0
        .word   0xf010                  /* PMOVEFD (%a0),TT0 */
        .word   0x0900
        movea.l #0x00100000,%a0
        move.l  (%a0),%d0
        cmpi.l  #0x77778888,%d0
        bne.w   fail

        /* Change the table while transparent translation is active. */
        lea     page_table,%a1
        move.l  #0x00030001,0x400(%a1)
        lea     tt_disabled,%a0
        .word   0xf010                  /* PMOVEFD (%a0),TT0 */
        .word   0x0900
        movea.l #0x00100000,%a0
        move.l  (%a0),%d0
        cmpi.l  #0x11112222,%d0
        bne.w   fail

        /* Point the page at the second physical value without flushing. */
        lea     tc_value,%a0
        .word   0xf010                  /* PMOVEFD (%a0),TC */
        .word   0x4100

        /* FD suppression must leave the cached first mapping visible. */
        movea.l #0x00100000,%a0
        move.l  (%a0),%d0
        cmpi.l  #0x11112222,%d0
        bne.w   fail

        /* The identical ordinary PMOVE must flush and reload the mapping. */
        lea     tc_value,%a0
        .word   0xf010                  /* PMOVE (%a0),TC */
        .word   0x4000
        movea.l #0x00100000,%a0
        move.l  (%a0),%d0
        cmpi.l  #0x33334444,%d0
        bne.w   fail

        /* Disable translation before touching the virt halt register. */
        lea     tc_disabled,%a0
        .word   0xf010                  /* PMOVE (%a0),TC */
        .word   0x4000
        movea.l #0xff009004,%a0
        moveq   #2,%d0                  /* VIRT_CTRL_CMD_HALT */
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
        .balign 4
tc_value:
        .long   0x80c0aa00
tc_disabled:
        .long   0
tt_value:
        .long   0x00008107
tt_disabled:
        .long   0
crp_value:
        .long   0x00000002,0

        /* The first 32 identity entries cover all image/table references. */
        .balign 0x1000
root_table:
        .space  0x1000
page_table:
        .space  0x1000
