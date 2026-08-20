# QEMU NeXT hardware support

## Background

I wrote the original NeXT Cube support for QEMU in 2011. Most of the hardware
was implemented on the `next-cube` branch, although it still had bugs and
omissions. Part of that work was merged into QEMU; the merged machine could
boot the firmware but could not fully boot into NeXTSTEP from disk or Ethernet
(dying either in SCSI init, or when attempting to mount the root filesystem).

I had the Plan 9 NeXT port in 2011, including work based on NeXT hardware
documentation, but not the NeXT boot ROM or Mach source. Preserved ROM and Mach
source trees later made it possible to check the hardware model against the
software that drove it. The current work also uses surviving manuals, firmware,
disk images, and tests against unmodified NeXT software.

I worked on and off on this multiple times...and then Codex laid it all bare in
a night's session. So it goes, lol.

## Boot blockers

The long-standing disk failure was mainly in NeXT DMA rather than the generic
ESP controller, though the lack of proper timeouts, which Mach uses to detect
connected drives...well, I think the problem there is obvious. NeXT SCSI DMA 
uses chained descriptors and a delayed four-word tail. Incorrect handling of
that 16-byte staging boundary corrupted reads, which appeared as duplicate SCSI
targets, bad disk labels, damaged superblocks, or a hang after disk detection.

After fixing the disk path, Mach exposed missing MC68040 floating-point state
handling in TCG. It also depended on system timers, the event counter, RTC and
NVRAM behavior, interrupt routing, and MMIO ranges that the old model did not
implement correctly. Network boot required support for the original NeXT BOOTP
vendor format rather than a normal DHCP reply.

These problems are fixed well enough to boot an unmodified NeXTSTEP system to
the desktop, but expect jank and crashes.

## Hardware support

| Hardware | Status | Notes |
|---|---|---|
| NeXTcube (MC68040, X15) | Working | Monochrome system; boots NeXTSTEP from SCSI disk. |
| NeXTstation (MC68040, Warp 9) | Working | Monochrome system. |
| NeXTstation Color (MC68040, Warp 9C) | Working | 1120 x 832 RGB444 display. Bt463 lookup and tag state is retained but does not alter direct-color scanout. |
| Original NeXT Computer/Cube (MC68030) | Not implemented | Blocked by the missing MC68030 PMMU and its translation registers and table format. The reserved machine name is `next-computer`. |
| Turbo systems | Not implemented | Turbo machine timing and board variants have not been modeled. |
| MC68040 CPU and FPU | Working | Includes the floating-point state frames and exceptions required by Mach. |
| DMA and interrupts | Working | One controller models all twelve channels, including SCSI, floppy, Ethernet, and sound paths. |
| ESP SCSI | Working | Supports disk boot and the NeXT-specific DMA staging behavior. |
| 82077 floppy | Working | Supports DMA and 720 KiB, 1.44 MiB, and 2.88 MiB raw media. Guest eject does not detach the host backend. |
| Monochrome video | Working | Includes vertical-retrace interrupts. |
| Color video | Working | Direct RGB444 output and independent 68 Hz retrace interrupt. |
| Keyboard and mouse | Working | Includes keyboard interrupt delivery, repeat suppression, and capture-free absolute host-pointer translation. |
| Sound output | Working | DMA output is paced on the virtual clock. Sound input is not implemented. |
| SCC serial ports | Working | Both channels support interrupt-driven PIO. SCC DMA data transfer is not implemented. |
| MB8795 Ethernet | Working | DMA transmit and receive, internal loopback, SLiRP networking, and NeXT ROM BOOTP are supported. |
| RTC, event counter, and NVRAM | Working | Both supported clock chips are modeled; NVRAM can be persisted in a file. |
| NextBus | Partial | The Cube NBIC and an empty bus are present. No NextBus cards are implemented. |
| Magneto-optical drive | Partial | The Cube formatter registers and firmware self-test behavior are modeled; media operation is not implemented. |
| DSP | Partial | Host-interface MMIO and DMA registers are present. DSP execution is not implemented. |
| Printer | Partial | The MMIO range is present. No printer data path or host backend is implemented. |

Detailed machine and device notes are in `docs/system/target-m68k.rst`.
