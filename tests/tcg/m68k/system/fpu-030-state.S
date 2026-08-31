/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * MC68030 external-FPU FSAVE/FRESTORE dispatch and idle-frame regression.
 */

        .text
        .globl  _start

_start:
        lea     null_frame+4,%a0
        fsave   -(%a0)
        cmpa.l  #null_frame,%a0
        bne.w   fail
        tst.l   (%a0)
        bne.w   fail

        lea     null_frame,%a0
        frestore (%a0)

        fmove.d one,%fp0
        lea     idle_frame+60,%a0
        fsave   -(%a0)
        cmpa.l  #idle_frame,%a0
        bne.w   fail
        cmpi.l  #0x3f380000,(%a0)
        bne.w   fail

        lea     idle_frame,%a0
        frestore (%a0)+
        cmpa.l  #idle_frame+60,%a0
        bne.w   fail

        /*
         * The MC68882's initial-production idle format is 0x1f38.  The
         * v41 FPU uses the same 60-byte layout with version 0x3f.  Restore
         * both forms and require FSAVE to preserve every frame byte.
         */
        lea     documented_idle_frame,%a0
        frestore (%a0)+
        cmpa.l  #documented_idle_frame+60,%a0
        bne.w   fail

        lea     restored_idle_frame+60,%a1
        fsave   -(%a1)
        cmpa.l  #restored_idle_frame,%a1
        bne.w   fail

        lea     documented_idle_frame,%a0
        moveq   #14,%d0
compare_idle:
        cmpm.l  (%a0)+,(%a1)+
        bne.w   fail
        dbra    %d0,compare_idle

        /* A documented 68882 busy frame is 216 bytes (state size 0xd4). */
        lea     documented_busy_frame,%a0
        frestore (%a0)+
        cmpa.l  #documented_busy_frame+216,%a0
        bne.w   fail

        lea     restored_busy_frame+216,%a1
        fsave   -(%a1)
        cmpa.l  #restored_busy_frame,%a1
        bne.w   fail

        lea     documented_busy_frame,%a0
        moveq   #53,%d0
compare_busy:
        cmpm.l  (%a0)+,(%a1)+
        bne.w   fail
        dbra    %d0,compare_busy

        /* An enabled exception must produce an external busy frame too. */
        lea     dz_handler,%a0
        move.l  %a0,50*4
        moveq   #0,%d0
        fmove.l %d0,%fpsr
        move.l  #0x00000400,%d0
        fmove.l %d0,%fpcr
        fmove.d zero,%fp0
        fmove.d one,%fp1
        fdiv.x  %fp0,%fp1
        lea     generated_busy_frame+216,%a1
        fsave   -(%a1)
        cmpa.l  #generated_busy_frame,%a1
        bne.w   fail
        cmpi.l  #0x3fd40000,(%a1)
        bne.w   fail

        movea.l #0xff009004,%a0
        moveq   #2,%d0           /* VIRT_CTRL_CMD_HALT */
        move.l  %d0,(%a0)
success:
        bra.s   success

dz_handler:
fail:
1:
        bra.s   1b

        .data
        .balign 4
null_frame:
        .space  4
idle_frame:
        .space  60
one:
        .long   0x3ff00000, 0

documented_idle_frame:
        .long   0x1f380000
        .long   0x10203040, 0x50607080, 0x90a0b0c0, 0xd0e0f000
        .long   0x11223344, 0x55667788, 0x99aabbcc, 0xddeeff00
        .long   0x12345678, 0x9abcdef0, 0x0badc0de, 0xc001d00d
        .long   0x31415926, 0x27182818
restored_idle_frame:
        .space  60

documented_busy_frame:
        .long   0x1fd40000
        .long   0x01020304, 0x05060708, 0x090a0b0c, 0x0d0e0f10
        .long   0x11121314, 0x15161718, 0x191a1b1c, 0x1d1e1f20
        .long   0x21222324, 0x25262728, 0x292a2b2c, 0x2d2e2f30
        .long   0x31323334, 0x35363738, 0x393a3b3c, 0x3d3e3f40
        .long   0x41424344, 0x45464748, 0x494a4b4c, 0x4d4e4f50
        .long   0x51525354, 0x55565758, 0x595a5b5c, 0x5d5e5f60
        .long   0x61626364, 0x65666768, 0x696a6b6c, 0x6d6e6f70
        .long   0x71727374, 0x75767778, 0x797a7b7c, 0x7d7e7f80
        .long   0x81828384, 0x85868788, 0x898a8b8c, 0x8d8e8f90
        .long   0x91929394, 0x95969798, 0x999a9b9c, 0x9d9e9fa0
        .long   0xa1a2a3a4, 0xa5a6a7a8, 0xa9aaabac, 0xadaeafb0
        .long   0xb1b2b3b4, 0xb5b6b7b8, 0xb9babbbc, 0xbdbebfc0
        .long   0xc1c2c3c4, 0xc5c6c7c8, 0xc9cacbcc, 0xcdcecfd0
        .long   0xd1d2d3d4, 0xd5d6d7d8, 0xd9dadbdc, 0xdddedfe0
        .long   0xe1e2e3e4
restored_busy_frame:
        .space  216
generated_busy_frame:
        .space  216
zero:
        .long   0, 0
