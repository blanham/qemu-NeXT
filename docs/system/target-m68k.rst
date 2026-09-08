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
  32 MiB of RAM and a 1120 x 832 Bt463-mediated RGB444 display.  Its VRAM
  starts at ``0x2c000000``.  Warp9C framebuffer words enter the Bt463 pixel
  port, which applies the programmed window-type table, palette RAM, cursor
  colors, overlay/underlay routing, read and blink masks, and blink phase
  before producing RGB scanout.  This includes the full 528-entry palette and
  the NeXTSTEP guest display driver updates the gamma and brightness LUTs via
  the DAC at ``0x02118100``.  The display has an independent 68 Hz retrace
  interrupt on status bit 13.  It has no NextBus.

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
``serial1``.  Both channels support interrupt-driven PIO and the shared SCC
DMA engine described below.

SCC serial DMA
~~~~~~~~~~~~~~

The NeXT serial controller exposes one shared, bidirectional SCC DMA engine at
CSR ``0x020000c0``.  It can service either SCC port, but cannot transfer both
ports at once.  When both ports request service simultaneously, channel A has
priority.  PIO interrupt 17 and DMA interrupt 21 are separate interrupt
sources.  PIO interrupt 17 is a polled IPL5 source and is delivered
independently of the global NeXT interrupt-mask register; DMA interrupt 21
remains mask-controlled.

The ESCC WR1 request gate controls which port may request DMA.  ``REQENABLE``
(``0x80``) and ``REQFUNC`` (``0x40``) must be set; ``REQRX`` (``0x20``)
selects receive, while clearing ``REQRX`` selects transmit.  The DMA CSR
``READ`` direction must match the WR1 direction.  A direction mismatch leaves
the request pending and does not create a bus exception.

The active segment uses ``NEXT``/``LIMIT`` at ``0x020040c0``/``0x020040c4``.
With ``SUPDATE`` set, reaching ``LIMIT`` promotes the segment in
``START``/``STOP`` (``0x020040c8``/``0x020040cc``).  Each segment completion
sets ``COMPLETE``.  A promoted segment keeps ``ENABLE`` set and waits for the
guest to clear ``COMPLETE`` before transferring the next segment; the final
segment clears ``ENABLE`` while leaving ``COMPLETE`` set.  An enabled
zero-length segment (``NEXT == LIMIT``) completes without a bus error.  If
``NEXT > LIMIT`` or a guest-memory transaction fails, the engine sets
``BUSEXC | COMPLETE``, clears the active state, and asserts DMA interrupt 21.

A channel ``RESET`` command cancels pending SCC work and clears
``ENABLE``, ``SUPDATE``, ``COMPLETE``, ``BUSEXC``, and the pending
``NEXT_INIT`` latch.  It does not consume a byte already held by the ESCC.
Machine reset clears the DMA channel state.  Migration saves the
guest-visible DMA pointers/status and ESCC register/data state; post-load
reconstructs PIO and DMA interrupt/request levels and resumes an enabled,
serviceable transfer at the destination.  If a character backend applies
backpressure, transmit DMA holds ``NEXT`` at the unaccepted byte and retries
when the backend becomes writable.

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

NetBSD 10.1 native SCSI install
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The official next68k RAMDISK boots with ``md0a`` as its rescue root. The
authenticated campaign performs a CD-only manual installation to target 0
``sd0`` with the CD at target 3, followed by two network-independent boots
from local ``sd0a``. Both boots read the same ``/NETBSD_SCSI_INSTALLED`` file
written during installation; the second boot is the persisted install-marker
check, not a check for a distinct ``/NETBSD_SCSI_PERSISTED`` file. The ISO is
not firmware-bootable, no official next68k install floppy is supplied, and the
r2/r3 captures are diagnostic failures.
The accepted QEMU commit is ``9817eefa75d8dc4e1eb59c0b21dedb135287f958``;
installed-disk, GIF, and master lossless-recording SHA-256 values are
``3206adb32525c7eb62653273197e873a44fccad868b85c2d24b1031e33295ff5``,
``9fe9cd975455c6b12fd3cfabf4c3ef33b2218fd1b39782c12c60c72f11c8d361``, and
``ffcd9502510eec06f7e1a7ab271e6782d3c2b5ac623a12afa33ea30951fe2dab``
respectively. The bounded published highlight FFV1 is
``d2e4cc5c32a2f7a66a083e3bf41ad8b64010ac9f63bde4914c5daa8cb48e7c7a``.
