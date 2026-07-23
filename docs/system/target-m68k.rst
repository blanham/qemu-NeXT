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

SCSI is the only functional DMA client at present.  The Ethernet transmit and
receive channels expose their register banks and reserve typed hooks for the
MB8795 implementation, but do not yet transfer packets.
