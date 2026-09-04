/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * MC68030 access-frame ownership regression.
 *
 * A bus-error handler may discard the internal access-frame tail by changing
 * its format word to format 0 before RTE.  A second physical decode error
 * must still be deliverable after that RTE; it must not be mistaken for a
 * nested bus fault from the first frame.
 */

        .text
        .globl  _start

_start:
        movea.l #stack_top,%sp

        lea     access_handler,%a0
        move.l  %a0,0x08

        /* TC reset is disabled: this is a physical decode-error probe. */
        move.l  #1,test_id
        move.l  0xf0fffff0,%d0

first_probe_returned:
        move.l  #2,test_id
        move.l  0xf0fffff0,%d0

second_probe_returned:
        move.l  #3,test_id
        move.l  0xf0fffff0,%d0

third_probe_returned:
        move.l  #4,test_id
        move.l  0xf0fffff0,%d0

fourth_probe_returned:
        /* VIRT_CTRL_CMD_HALT terminates the test without a guest ABI. */
        movea.l #0xff009004,%a0
        moveq   #2,%d0
        move.l  %d0,(%a0)
success:
        bra.s   success

/*
 * The handler edits the saved PC so the failed load is not retried, then
 * changes format B to format 0.  This is the firmware's discard-frame path;
 * RTE consumes only SR/PC/format and leaves the long frame tail behind.
 */
access_handler:
        move.l  test_id,%d0
        cmpi.l  #1,%d0
        beq.s   first_fault
        cmpi.l  #2,%d0
        beq.s   second_fault
        cmpi.l  #3,%d0
        beq.s   third_fault
        cmpi.l  #4,%d0
        beq.s   fourth_fault
        cmpi.l  #5,%d0
        beq.s   nested_fault
        bra.w   unexpected_fault

first_fault:
        /* The entry path has already consumed its three guarded words.
         * This later handler access must be allowed to take a new frame. */
        move.l  #5,test_id
        move.l  0xf0fffff0,%d1

after_nested_fault:
        /* Relocate the common base frame to the tail of format B, as
         * Mach's access handler does before returning through RTE. */
        move.w  (%sp),%d2
        move.w  %d2,0x54(%sp)
        lea     first_probe_returned,%a0
        move.l  %a0,0x56(%sp)
        clr.w   0x5a(%sp)
        lea     0x54(%sp),%sp
        rte

nested_fault:
        move.w  (%sp),%d2
        move.w  %d2,0x54(%sp)
        lea     after_nested_fault,%a0
        move.l  %a0,0x56(%sp)
        clr.w   0x5a(%sp)
        lea     0x54(%sp),%sp
        rte

second_fault:
        move.w  (%sp),%d2
        move.w  %d2,0x54(%sp)
        lea     third_probe_returned,%a0
        move.l  %a0,0x56(%sp)
        clr.w   0x5a(%sp)
        lea     0x54(%sp),%sp
        rte

third_fault:
        move.w  (%sp),%d2
        move.w  %d2,0x54(%sp)
        lea     fourth_probe_returned,%a0
        move.l  %a0,0x56(%sp)
        clr.w   0x5a(%sp)
        lea     0x54(%sp),%sp
        rte

fourth_fault:
        move.w  (%sp),%d2
        move.w  %d2,0x54(%sp)
        lea     fourth_probe_returned,%a0
        move.l  %a0,0x56(%sp)
        clr.w   0x5a(%sp)
        lea     0x54(%sp),%sp
        rte

unexpected_fault:
        bra.s   unexpected_fault

        .data
        .balign 4
test_id:
        .long   0

        .balign 0x1000
stack_storage:
        .space  0x1000
stack_top:
