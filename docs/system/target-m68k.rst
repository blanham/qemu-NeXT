.. _ColdFire-System-emulator:

ColdFire System emulator
------------------------

Use the executable ``qemu-system-m68k`` to simulate a ColdFire machine.
The emulator is able to boot a uClinux kernel.

The M5208EVB emulation includes the following devices:

-  MCF5208 ColdFire V2 Microprocessor (ISA A+ with EMAC).

-  Three Two on-chip UARTs.

-  Fast Ethernet Controller (FEC)

The AN5206 emulation includes the following devices:

-  MCF5206 ColdFire V2 Microprocessor.

-  Two on-chip UARTs.

NeXT Cube system emulator
-------------------------

The ``next-cube`` machine has one shared DMA controller covering all twelve
physical channels.  Machine reset clears every channel register, pending
initial-pointer latch, staged SCSI byte, and DMA interrupt.  Migration saves
the same guest-visible and staged state and reconstructs each interrupt on
the destination.  A channel's DMA interrupt is asserted exactly when its
``COMPLETE`` status bit is set; register programming alone does not complete
a transfer.

The ESP SCSI controller and 82077 floppy controller share the physical SCSI
DMA channel at ``0x02000010``.  Selection and direction gates ensure that only
one controller can use the channel at a time.  Floppy reads and writes support
incremental descriptors and chained transfers through the channel's
``NEXT``/``LIMIT`` and ``START``/``STOP`` register pairs.  Descriptor
completion raises DMA interrupt bit 26; completion of the floppy command
raises peripheral interrupt bit 7.  A staged continuation is not promoted
when the floppy controller drops its DMA request exactly at the current
limit, so ROM transfer accounting continues to see the completed data
buffer.

The floppy controller register block is at ``0x02114100`` and its NeXT media
control register is at ``0x02114108``.  Attach raw 720 KiB, 1.44 MiB, or
2.88 MiB media with ``-drive if=floppy,format=raw,file=IMAGE``.  Media capacity
is reported through the control register, while status register A reports the
backend's live write permission on the active-low write-protect input.  An
82077 ``CONFIGURE`` command with ``DPOLL`` suppresses reset polling only when
it arrives within 250 microseconds and before any reset result was consumed.
The guest-visible eject bit is implemented as a latch, but it does not remove
media from the QEMU block backend.  The v66 ROM's SCSI/floppy control and
status window at ``0x02014020``--``0x02014021`` aliases the operating system
window at ``0x02114020``--``0x02114021``.
