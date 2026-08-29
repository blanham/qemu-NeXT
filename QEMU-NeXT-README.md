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

![NeXTSTEP 3.1 booting to Workspace Manager in QEMU](https://raw.githubusercontent.com/blanham/qemu-NeXT/metachicken/docs/boot/nextstep.gif)

<!-- next-rom-matrix:start -->
### Non-Turbo MC68040 ROM compatibility

Pass means the guest reached a usable Workspace Manager desktop after `bsd mach_kernel` from the shared authenticated NeXTSTEP disk. Earlier gates are diagnostic evidence, not a pass.

| ROM | `next-cube` | `next-station` | `next-station-color` |
| --- | --- | --- | --- |
| v58 | inconclusive | inconclusive | inconclusive |
| v59 | inconclusive | inconclusive | inconclusive |
| v65 | inconclusive | inconclusive | inconclusive |
| v66 | inconclusive | inconclusive | inconclusive |

#### Evidence and primary citations

- <code>v58--next-cube</code>: none
- <code>v58--next-station</code>: none
- <code>v58--next-station-color</code>: none
- <code>v59--next-cube</code>: none
- <code>v59--next-station</code>: none
- <code>v59--next-station-color</code>: none
- <code>v65--next-cube</code>: none
- <code>v65--next-station</code>: none
- <code>v65--next-station-color</code>: none
- <code>v66--next-cube</code>: none
- <code>v66--next-station</code>: none
- <code>v66--next-station-color</code>: none

Authenticated ROM catalog SHA-256: `fd0a97bab109833dc2e58771c4ea985445514ad2658e9a901746bba2464a24da`.

The table is generated from the verified lab campaign. Firmware and disk images are external inputs; Git contains their identities and reproduction procedure, not their bytes.
<!-- next-rom-matrix:end -->

The campaign tooling lives in the companion `lab` repository. Place the four files at
the relative paths recorded by `catalog/next-roms.json`, keep the canonical NeXTSTEP
disk at `assets/disks/next-old.img`, build a clean registered QEMU worktree through
`nextcube_lab.baseline_cli build`, and run the `verify` subcommand shown in the lab ROM
matrix documentation with the same explicit source, build, catalog, and ROM-root inputs. The
aggregate JSON is authoritative; the table above is generated from it.

## NetBSD/next68k NFS-root boot

NetBSD 1.5/next68k now boots through the NeXT v66 ROM and its standalone
`boot` program. The loader obtains `netbsd` with MOUNT v1 and NFSv2; the
kernel then mounts its writable root with MOUNT v3 and NFSv3. All RPC services
remain inside QEMU's user-mode network and do not open host NFS ports.

The shorter recording begins when the standalone loader switches to the
network. The full recording includes the ROM and TFTP bootstrap. Both end at
the root shell:

![NetBSD loading its kernel and NFS root over the network](https://raw.githubusercontent.com/blanham/qemu-NeXT/metachicken/docs/boot/netbsd-network.gif)

![NetBSD booting from the NeXT ROM to an NFS-root shell](https://raw.githubusercontent.com/blanham/qemu-NeXT/metachicken/docs/boot/netbsd-full.gif)

Stage the next68k standalone program as `$TFTP/boot`, and extract a disposable
NetBSD root tree at `$ROOT` with `netbsd` at its top level. Launch it with:

```sh
QEMU=$PWD/build-next/qemu-system-m68k
ROM=/path/to/Rev_2.5_v66.BIN
TFTP=/path/to/tftp
ROOT=/path/to/netbsd-root

"$QEMU" -M next-station \
  -global next-pc.system-timer-frequency=4456448 \
  -bios "$ROM" -m 64M -display gtk \
  -fsdev "local,id=netbsdroot,path=$ROOT,security_model=mapped-xattr" \
  -netdev "user,id=nextnet,ipv6=off,tftp=$TFTP,bootfile=boot" \
  -object "nfs-server,id=netbsdnfs,fsdev=netbsdroot,netdev=nextnet,writable=on" \
  -net "nic,model=next-mb8795,netdev=nextnet" \
  -no-reboot
```

At the ROM prompt enter `ben() boot`. At the standalone `boot:` prompt enter
`en()netbsd`; plain `netbsd` selects SCSI instead of the mounted network
device. The archived root deliberately has `rc_configured=NO`, so press Return
at its shell-path and terminal-type prompts to reach `/bin/sh`. The complete
NFS object contract is in `docs/system/devices/nfs-root.rst`.

## Plan 9

Native Plan 9 First and Second Edition support now boots each release's
unmodified `68020/9nextstation` kernel through the NeXT ROM. QEMU supplies the
original NeXT BOOTP reply, TFTP for the kernel, and a writable 9P1 root
service. The compatibility profile uses unauthenticated TCP port 564. The
historic profile uses authenticated IL (IPv4 protocol 40), with the ticket
service on IL port 566 and the file service on IL port 17008. I verified the
path with the v66 ROM and `plan9-2e.tar.bz2` in a visible GTK run, through the
Plan 9 terminal. The archive uses `tor` as its user and contains `/usr/tor`;
`aux/mouse` may time out while the terminal itself is usable.

TCP compatibility boot to the Plan 9 8½ desktop:

![Plan 9 Second Edition booting over TCP to 8½](https://raw.githubusercontent.com/blanham/qemu-NeXT/metachicken/docs/boot/plan9-tcp.gif)

Authenticated IL boot to the same desktop with `tor` / `password`:

![Plan 9 Second Edition booting over authenticated IL to 8½](https://raw.githubusercontent.com/blanham/qemu-NeXT/metachicken/docs/boot/plan9-il.gif)

The historical archives are mirrored by the Oregon State University Open
Source Lab at:

```text
https://ftp.osuosl.org/pub/plan9/history/
https://ftp.osuosl.org/pub/plan9/history/sha256sum.txt
```

The verified tar archives have these SHA-256 identities:

| Edition | Archive | SHA-256 |
|---|---|---|
| First | `plan9-1e.tar.bz2` | `8718e279aa35b10a9391f330976d31c177bad00de40e7f0f67232ffa54fb7d77` |
| Second | `plan9-2e.tar.bz2` | `0bb3c1446deb79b179f73886eb2419ecfac9f9964040683e3d5731f074bc2ce6` |

Both editions have been exercised through visible 8½ startup with their
archived kernels and root trees unchanged.

### Build

From a clean checkout of the public branch:

```sh
git clone -b metachicken https://github.com/blanham/qemu-NeXT.git qemu-NeXT
cd qemu-NeXT
mkdir build-next
cd build-next
../configure --target-list=m68k-softmmu --enable-debug \
  --enable-trace-backends=log
./pyvenv/bin/meson setup --reconfigure --force-fallback-for=slirp . ..
ninja -j"$(nproc)" qemu-system-m68k qemu-img qemu-plan9-keydb
```

The Plan 9 runs need a staged kernel directory containing
`68020/9nextstation` and a staged root tree extracted from the verified
Second Edition archive.

#### TCP compatibility launch

With those paths set, the compatibility launch is:

```sh
QEMU="$PWD/qemu-system-m68k"
ROM=/path/to/Rev_2.5_v66.BIN
TFTP=/path/to/tftp
ROOT=/path/to/rootfs
QMP=/tmp/next-plan9.qmp

"$QEMU" -M next-station \
  -global next-pc.system-timer-frequency=4456448 \
  -bios "$ROM" -m 64M -display gtk \
  -qmp "unix:$QMP,server=on,wait=off" \
  -fsdev "local,id=plan9root,path=$ROOT,security_model=none" \
  -netdev "user,id=nextnet,ipv6=off,tftp=$TFTP,bootfile=68020/9nextstation" \
  -object "plan9-9p1-server,id=plan9fs,fsdev=plan9root,netdev=nextnet,guest-address=10.0.2.100,port=564" \
  -net "nic,model=next-mb8795,netdev=nextnet" \
  -no-reboot
```

The frequency override compensates for the historical kernel loading `0xffff`
while configuring `HZ` as 68. It affects the system timer only; the event
counter remains at its hardware rate. Use the same override on both ends of a
migration; the frequency is machine configuration, not migrated guest state.

At the ROM prompt enter `ben() 68020/9nextstation`. At the root source prompt
enter `tcp`, then accept `none` at the Plan 9 `user[none]:` prompt. This archive
has no authentication key for `tor`, despite listing that name in `/adm/users`.
From the resulting shell, start the normal `tor` desktop with:

```rc
home=/usr/tor
. /usr/tor/lib/profile
```

The companion NeXT lab checkout automates archive verification and staging;
the direct command above is useful when those paths already exist.

#### Authenticated historical IL launch

Provision the native encrypted key database and its separate QEMU Secret in a
private directory. The tool requires that directory to be owned by the current
user with mode 0700; it creates both output files with mode 0600 and refuses to
overwrite either path.

```sh
KEYDIR=/path/to/private-plan9-keys
mkdir -p "$KEYDIR"
chmod 0700 "$KEYDIR"

./qemu-plan9-keydb create \
  --keydb "$KEYDIR/keys" \
  --secret "$KEYDIR/master.b64" \
  --server-id p9fs
```

Enter and confirm the password for `tor` at the controlling terminal. Do not
put the password or the decoded seven-byte master key in argv, environment
variables, QMP, logs, Git, or capture files.

Launch QEMU with the base64 Secret file and authenticated IL transport:

```sh
QEMU="$PWD/qemu-system-m68k"
ROM=/path/to/Rev_2.5_v66.BIN
TFTP=/path/to/tftp
ROOT=/path/to/rootfs
KEYDIR=/path/to/private-plan9-keys
QMP=/tmp/next-plan9-il.qmp

"$QEMU" -M next-station \
  -global next-pc.system-timer-frequency=4456448 \
  -bios "$ROM" -m 64M -display gtk \
  -qmp "unix:$QMP,server=on,wait=off" \
  -fsdev "local,id=plan9root,path=$ROOT,security_model=none" \
  -netdev "user,id=nextnet,ipv6=off,tftp=$TFTP,bootfile=68020/9nextstation" \
  -object "secret,id=plan9-master-key,format=base64,file=$KEYDIR/master.b64" \
  -object "plan9-9p1-server,id=plan9fs,fsdev=plan9root,netdev=nextnet,guest-address=10.0.2.100,transport=il,il-port=17008,auth-port=566,auth-id=p9fs,auth-domain=nextlab,keydb=$KEYDIR/keys,key-secret=plan9-master-key" \
  -net "nic,model=next-mb8795,netdev=nextnet" \
  -no-reboot
```

At the ROM prompt enter `ben() 68020/9nextstation`. At the root source prompt
enter `il`, accept `tor` at `user[tor]:`, and type the provisioned password
directly in the GTK guest console. The password must not be injected with QMP
because QMP transcripts record key events.

The DES ticket protocol is obsolete and unauthenticated encryption is not a
modern security boundary. Use this profile only on QEMU's private user-mode
SLiRP network. Do not expose IL/566 or IL/17008 through host forwarding or a
bridged/TAP network. Active IL connections block live migration and must
reconnect after guest reset.

## SCC serial DMA

The NeXT serial controller has one shared, bidirectional SCC DMA engine at CSR
`0x020000c0`. It can serve either SCC port, but not both at once; channel A has
priority when both request service. Normal SCC PIO interrupt 17 remains
independent of the mask-controlled SCC DMA interrupt 21.

The ESCC WR1 request gate selects receive or transmit service, and the DMA CSR
direction must agree with it. Transfers support the NeXT `NEXT`/`LIMIT`
segment, `START`/`STOP` promotion through `SUPDATE`, completion interrupts,
zero-length completion, guest-memory bus exceptions, and transmit-backend
backpressure. Reset cancels pending work, while migration preserves the
guest-visible pointers, status, and serial state and resumes a serviceable
transfer at the destination. Detailed register and lifecycle behavior is in
`docs/system/target-m68k.rst`.

## Hardware support

| Hardware | Status | Notes |
|---|---|---|
| NeXTcube (MC68040, X15) | Working | Monochrome system; boots NeXTSTEP from SCSI disk. |
| NeXTstation (MC68040, Warp 9) | Working | Monochrome system; boots NeXTSTEP from SCSI and Plan 9 First and Second Editions by ROM netboot. |
| NeXTstation Color (MC68040, Warp 9C) | Working | 1120 x 832 Bt463-mediated RGB444 display. Warp9C VRAM words pass through the Bt463 window-type tables, 528-entry palette, cursor/overlay routing, masks, and blink state; firmware gamma and brightness LUT updates affect scanout. |
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
| SCC serial ports and DMA | Working | Both channels support interrupt-driven PIO and the shared bidirectional SCC DMA engine. |
| MB8795 Ethernet | Working | DMA transmit and receive, internal loopback, SLiRP networking, and NeXT ROM BOOTP are supported. |
| RTC, event counter, and NVRAM | Working | Both supported clock chips are modeled; NVRAM can be persisted in a file. |
| NextBus | Partial | The Cube NBIC and an empty bus are present. No NextBus cards are implemented. |
| Magneto-optical drive | Partial | The Cube formatter registers and firmware self-test behavior are modeled; media operation is not implemented. |
| DSP | Partial | Host-interface MMIO and DMA registers are present. DSP execution is not implemented. |
| Printer | Partial | The MMIO range is present. No printer data path or host backend is implemented. |

Plan 9 support is the original 9P1 protocol only. The netboot profiles use IPv4
SLiRP and the NeXT BOOTP vendor format; they are not a general Plan 9 network
configuration. The 68030 PMMU, Turbo board variants, DSP execution,
sound input, NextBus cards, magneto-optical media, and printer data
path remain future work.

Detailed machine and device notes are in `docs/system/target-m68k.rst`.
