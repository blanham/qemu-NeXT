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

NeXT system emulators
---------------------

QEMU provides three 68040 NeXT machine types:

``next-cube``
  A 25 MHz 68040 X15 NeXTcube (machine type 2), with up to 64 MiB of
  RAM and a monochrome display.  It includes an NBIC and an unpopulated
  NextBus; no NextBus cards are implemented.

``next-station``
  A 25 MHz 68040 Warp 9 NeXTstation (machine type 1), with up to 64 MiB
  of RAM and a monochrome display.  It has no NextBus.

``next-station-color``
  A 25 MHz 68040 Warp 9C NeXTstation Color (machine type 3), with up to
  32 MiB of RAM and a 1120 x 832 direct RGB444 display.  Its VRAM starts
  at ``0x2c000000``.  Bt463 lookup-table and tag programming is retained,
  but does not transform the direct RGB444 scanout.  The display has an
  independent 68 Hz retrace interrupt on interrupt-status bit 13.  It has
  no NextBus.

The name ``next-computer`` is reserved for the original 68030 NeXT
Computer/Cube (machine type 0).  It is not available because QEMU does
not yet implement the 68030 PMMU required by that machine.

The default firmware for all three machines is the known v66 ROM image
conventionally named ``Rev_2.5_v66.bin``.  Its SHA-1 is
``b3534796abae238a0111299fc406a9349f7fee24``.  QEMU does not bundle this
firmware; the user must supply it.  ``-bios FILE`` may be used to override
the default firmware filename.

The implemented devices shared by these machines include one DMA controller,
ESP SCSI, an 82077 floppy controller, two serial channels, Ethernet,
RTC/NVRAM, keyboard and mouse input, and sound output.  The DMA controller
covers all twelve physical channels.  Machine reset clears every channel
register, pending initial-pointer latch, staged SCSI byte, and DMA interrupt.
Migration saves the same guest-visible and staged state and reconstructs each
interrupt on the destination.  A channel's DMA interrupt is asserted exactly
when its ``COMPLETE`` status bit is set; register programming alone does not
complete a transfer.

The NeXT keyboard/mouse device accepts absolute host pointer coordinates by
default and translates them into original-format relative NeXT mouse packets.
This compatibility mode compensates for NeXTSTEP's default nonlinear mouse
acceleration and works with unmodified guest software.  GTK with
``grab-on-hover=off`` is the recommended capture-free display backend.

Plan 9 and hardware-oriented diagnostics can retain the literal relative
input path with ``-global next-kbd.absolute-pointer=off``.  That mode preserves
the fixed divide-by-three translation used for Plan 9's mouse policy.

The RTC contains 32 bytes of NeXT nonvolatile configuration.  The bytes
survive ``system_reset``.  They are process-local by default; for example, use
``-M next-cube,nvram-file=PATH`` to retain them across QEMU runs.  The file is
an exact 32-byte raw NVRAM image, is created with mode 0600 when absent, and is
held under an exclusive lock.  QEMU preserves invalid checksums for firmware
diagnosis rather than repairing them.

ROM preference “serial port A is alternate console” is stored in this NVRAM.
With ``serial0`` attached, save the preference and reset to move the ROM
console to SCC channel A.  Channel A is ``serial0`` and channel B is
``serial1``; the current Mach driver uses interrupt-driven PIO rather than SCC
DMA.

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
