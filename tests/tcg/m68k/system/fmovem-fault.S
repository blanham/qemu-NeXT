/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * A faulting FMOVEM must report the FMOVEM instruction, even when it follows
 * another instruction in the same translation block.
 */

        .text
        .globl  _start
_start:
        lea     stack_end,%sp
        lea     access_handler,%a0
        move.l  %a0,2*4

        /* Keep LINK and FMOVEM in one TB: the first store must fault. */
        link    %a6,#-16
        movea.l #0x2000000c,%a0
faulting_fmovem:
        fmovem.x %fp0,-(%a0)
        bra.s   fail

access_handler:
        /* The access-error frame stores SR, PC, and format/vector. */
        move.w  2(%sp),%d0
        move.w  4(%sp),%d1
        swap    %d0
        move.w  %d1,%d0
        cmpi.l  #faulting_fmovem,%d0
        bne.s   fail

        movea.l #0xff009004,%a0
        moveq   #2,%d0
        move.l  %d0,(%a0)
fail:
1:
        bra.s   1b

        .data
        .space  2048
stack_end:
