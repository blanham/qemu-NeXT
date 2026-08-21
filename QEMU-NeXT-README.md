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

I returned to this work to finish the missing device behavior and test it
against the original software.

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
the desktop. The model is not a claim of complete hardware fidelity.

## Plan 9

Native Plan 9 Second Edition support now boots the unmodified
`68020/9nextstation` kernel through the NeXT ROM. QEMU supplies the original
NeXT BOOTP reply, TFTP for the kernel, and a writable Second Edition 9P1 root
service over TCP port 564. I verified the path with the v66 ROM and
`archive/plan9/plan9-2e.tar.bz2` in a visible GTK run, through the Plan 9
terminal. The archive uses `tor` as its user and contains `/usr/tor`;
`aux/mouse` may time out while the terminal itself is usable.

### Build

From a clean checkout of the public branch:

```sh
git clone -b metachicken git@github.com:blanham/qemu-NeXT.git qemu-NeXT
cd qemu-NeXT
mkdir build-next
cd build-next
../configure --target-list=m68k-softmmu --enable-debug \
  --enable-trace-backends=log
./pyvenv/bin/meson setup --reconfigure --force-fallback-for=slirp . ..
ninja -j"$(nproc)" qemu-system-m68k qemu-img
```

The Plan 9 run needs a staged kernel directory containing
`68020/9nextstation` and a staged root tree extracted from the verified
Second Edition archive. With those paths set, the tested launch is:

```sh
QEMU="$PWD/qemu-system-m68k"
ROM=/path/to/Rev_2.5_v66.BIN
TFTP=/path/to/tftp
ROOT=/path/to/rootfs
QMP=/tmp/next-plan9.qmp

"$QEMU" -M next-station -bios "$ROM" -m 64M -display gtk \
  -qmp "unix:$QMP,server=on,wait=off" \
  -fsdev "local,id=plan9root,path=$ROOT,security_model=none" \
  -netdev "user,id=nextnet,ipv6=off,tftp=$TFTP,bootfile=68020/9nextstation" \
  -object "plan9-9p1-server,id=plan9fs,fsdev=plan9root,netdev=nextnet,guest-address=10.0.2.100,port=564" \
  -net "nic,model=next-mb8795,netdev=nextnet" \
  -no-reboot
```

At the ROM prompt enter `ben() 68020/9nextstation`. At the root source prompt
enter `tcp`, then enter `tor` at the Plan 9 `user[none]:` prompt. The companion
NeXT lab checkout automates archive verification and staging; the direct
command above is useful when those paths already exist.

## Hardware support

| Hardware | Status | Notes |
|---|---|---|
| NeXTcube (MC68040, X15) | Working | Monochrome system; boots NeXTSTEP from SCSI disk. |
| NeXTstation (MC68040, Warp 9) | Working | Monochrome system; boots NeXTSTEP from SCSI and Plan 9 Second Edition by ROM netboot. |
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

Plan 9 support is the original 9P1 protocol only. The current netboot profile
uses IPv4 SLiRP and the NeXT BOOTP vendor format; it is not a general Plan 9
network configuration. The 68030 PMMU, Turbo board variants, DSP execution,
sound input, SCC DMA, NextBus cards, magneto-optical media, and printer data
path remain future work.

Detailed machine and device notes are in `docs/system/target-m68k.rst`.
