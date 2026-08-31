/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * MC68030 PMOVE invalid-TC configuration exception regression.
 */

        .text
        .globl  _start
_start:
        lea     config_handler,%a0
        move.l  %a0,0xe0           /* MMU configuration vector (56) */
        lea     bad_tc,%a0

        /* E=1 with PS=0 fails the TC consistency check. */
        .word   0xf010              /* PMOVE (%a0),TC */
        .word   0x4000
        bra.s   fail                /* PMOVE must not continue here */

config_handler:
        lea     tc_after,%a0
        .word   0xf010              /* PMOVE TC,(%a0) */
        .word   0x4200
        move.l  tc_after,%d0
        bne.s   fail
        movea.l #0xff009004,%a0
        moveq   #2,%d0             /* VIRT_CTRL_CMD_HALT */
        move.l  %d0,(%a0)
1:
        bra.s   1b

fail:
2:
        bra.s   2b

        .data
bad_tc:
        .long   0x80000000
tc_after:
        .long   0
