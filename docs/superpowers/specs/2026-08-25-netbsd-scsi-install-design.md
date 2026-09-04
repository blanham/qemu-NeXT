# NetBSD/next68k SCSI Install and Local Boot Design

> **Historical record:** Do not execute this document linearly.  Its final
> dated amendments describe the old feature endpoint; the current execution
> authority is the lab plan
> `2026-09-04-netbsd-scsi-current-integration.md`.

> **Status (2026-08-26):** The original design from `Goal` through
> `Verification and integration` is preserved as historical, non-normative
> context for completed Tasks 1--8 and the failed-closed Task 9 attempt.  The
> normative design begins at `Design amendment: official NetBSD 10.1 RAMDISK`.
> Where the two sections differ, the amendment controls.

## Goal

Boot an unmodified NetBSD 1.5/next68k system from a local SCSI disk on the
NeXT machine, starting from the existing authenticated NFS-root environment.
The same acceptance campaign must discover and read an attached SCSI CD-ROM.
The result is intended to provide a reproducible local-disk target for Rooster
bootloader development as well as stronger coverage of the NeXT ESP and DMA
hardware model.

## Historical boundary

NetBSD/next68k did not ship a supported automated local-disk installer.
NetBSD 1.5 describes SCSI support as incomplete and requires an NFS root;
current releases still list the onboard SCSI controller, floppy, and CD-ROM as
unsupported.  The 1.5 archive contains a standalone `boot`, kernel, and binary
sets but no next68k installation floppy.  The official NetBSD 10.1 next68k ISO
is distribution media, not a bootable next68k installer.

This work therefore uses a guest-native manual installation from the existing
NetBSD 1.5 NFS root.  It does not fabricate a historical floppy, describe the
10.1 ISO as bootable, or patch NetBSD to hide an emulation defect.

## Authenticated inputs

The primary guest inputs are:

- the existing NeXT v66 ROM catalog entry;
- NetBSD 1.5/next68k `usr/mdec/boot`, `kern.tgz`, `base.tgz`, `etc.tgz`, and
  `root-nfs.tgz` from the preserved release;
- a newly created blank SCSI disk used only through a disposable overlay; and
- the official `NetBSD-10.1-next68k.iso`, stored outside Git under
  `/mnt/e/disks`.

The preserved 1.5 `base.tgz`, `etc.tgz`, and `kern.tgz` already match the
official release MD5 records.  The 10.1 ISO must match the release SHA-512:

```text
537ec02cc0af9ed0d4b527337c7dea7673c511a34f9cc57b33c76b42d566565ba20a10ee63eadac9abff6a9083b71a80acb010aa6022518f67fd6814d8ed46a8
```

Every promoted run records stronger locally computed SHA-256 identities in
addition to the upstream checksum records.  ROM, media, set, source, build,
and derived-disk identities are immutable campaign inputs.

## Architecture

The existing authenticated NetBSD 1.5 standalone loader and NFS root form the
rescue environment.  QEMU attaches a blank SCSI hard disk at target 0 and the
read-only ISO at a distinct target, initially target 3.  The unmodified 1.5
kernel is the primary compatibility oracle.

Inside the NFS-root guest, the campaign creates a valid NeXT disklabel and FFS
root filesystem on the blank disk, extracts the authenticated kernel, base,
and configuration sets, installs the two redundant next68k standalone boot
program copies, and configures a local root.  It then performs a controlled
shutdown and starts a separate run through the v66 ROM with networking
unavailable.  The ROM must load the disk boot program, which must load
`/netbsd`; the kernel must mount `sd0a` as root and reach a shell.

The install-stage guest also discovers `cd0`, mounts the official ISO through
`mount_cd9660`, and verifies selected files against its checksum manifest.
This proves the CD data path without implying that the ISO can boot or install
the next68k port by itself.

## QEMU changes

NeXT machine changes remain hardware-generic.  The machine permits an
ordinary QEMU SCSI CD device to be attached alongside disks.  Any ESP, DMA,
interrupt, reset, or transfer fix must be justified by unmodified guest
traffic and expressed as controller behavior rather than a NetBSD-specific
shortcut.

Focused qtests cover at least:

- disk and CD inquiry and capacity responses on separate targets;
- disk writes and reads across the NeXT 16-byte DMA staging boundary;
- CD reads with full, short, odd, and chained DMA transfer shapes exercised
  by the guest;
- multiple-target selection, missing-target timeouts, status, and sense data;
- reset during or after a transfer; and
- preservation of existing NeXTSTEP disk behavior.

The exact emulation files cannot be fixed in advance: diagnosis may locate a
defect in the NeXT machine glue, shared ESP controller, or NeXT DMA channel.
No code outside the demonstrated fault should change.

## Lab workflow

The companion lab adds a `netbsd-scsi-install` campaign profile.  It:

1. authenticates or acquires all external inputs;
2. creates a fresh blank base disk and disposable writable overlay;
3. boots the existing NetBSD NFS rescue environment;
4. performs and records the guest-native disk preparation and installation;
5. verifies the CD-ROM data path in the same install-stage run;
6. shuts down through the run-owned control channel;
7. boots the installed overlay with no usable network root;
8. verifies the local root and persistent marker across a second controlled
   reboot; and
9. promotes evidence only when every required gate and identity is present.

Automation must use guest-visible prompts and outputs as evidence.  Host-side
inspection may corroborate the result but cannot substitute for a guest disk
write, guest filesystem check, ROM disk boot, or local-root mount.

## Acceptance gates

The install-stage run must prove:

1. `esp0`, `scsibus0`, `sd0`, and `cd0` attach;
2. repeated disk reads and writes complete without corruption or stalls;
3. the guest creates and rereads the intended NeXT disklabel;
4. guest `newfs`, set extraction, and `fsck_ffs` complete successfully;
5. both boot-program copies are installed at the next68k-defined locations
   and agree with the authenticated input;
6. the official ISO mounts through `cd0`; and
7. selected ISO files verify against recorded checksums.

The independent native-boot run must prove:

1. the v66 ROM reads the SCSI disk boot block;
2. the unmodified standalone program loads `/netbsd` from SCSI;
3. the kernel explicitly reports its root on `sd0a`, with no NFS fallback;
4. a usable local-root shell is reached;
5. the install-stage marker is present; and
6. a new marker survives a controlled reboot and second local-root boot.

A loader prompt, SCSI probe, kernel banner, or NFS-root shell alone is not a
pass.

## Failure handling and evidence

The canonical external media are read-only.  Installation writes go to a
fresh overlay, never the preserved image or ISO.  A checksum mismatch, build
identity drift, SCSI timeout, short or corrupt transfer, filesystem error,
unexpected NFS root, missing CD-ROM, or unauthenticated shutdown makes the
campaign evidence-incomplete.

The runner preserves the first failing trace and visible checkpoint rather
than silently retrying past it.  Evidence includes the exact command line,
QEMU source and build identity, artifact hashes, disk geometry and label,
boot-program identity, guest transcripts, QMP lifecycle, screenshots, and the
final derived-disk identity.  A boot GIF with the decorated QEMU frame ends at
the local-root shell and uses the established smooth pacing and final hold.

## Documentation and scope

Both the lab README and `QEMU-NeXT-README.md` document acquisition,
verification, installation, direct SCSI boot, CD-ROM proof, limitations, and
the evidence/GIF.  Large media and derived disks remain outside Git.

This tranche excludes NetBSD kernel changes, a fabricated floppy or sysinst
path, CD boot, magneto-optical media operation, MC68030/PMMU work, Turbo
machines, and Rooster integration itself.  Those remain separately reviewable
follow-on projects.

## Verification and integration

Implementation proceeds test-first.  It must pass focused NeXT SCSI qtests,
the existing NeXT QEMU regression set, lab unit and contract tests, the visible
install run, two network-independent local boots, and the full lab suite.
QEMU and lab changes remain independently reviewable.  After review, the lab
branch is merged locally; the QEMU branch is merged into `metachicken`, pushed
to GitHub, and verified against the remote commit.

---

## Design amendment: official NetBSD 10.1 RAMDISK (2026-08-26)

### Why the original guest plan stopped

Tasks 1--8 above were implemented and reviewed against the original six-input
NetBSD 1.5 design.  The first visible Task 9 run then disproved one assumption:
the exact 1.5 release kernel reached its NFS-root single-user shell but emitted
no `nextdma`, `esp`, `scsibus`, `sd`, or `cd` attachment.  Binary and
source inspection confirmed that the release kernel lacks those drivers and
CD9660.  The controller correctly sent no install command, and the preserved
run is diagnostic failure evidence.  Completed Tasks 1--8 remain historical;
they are not retroactively described as 10.1 work.

The corrected guest oracle is the official, unmodified NetBSD 10.1 RAMDISK
kernel from the already authenticated ISO.  Its configuration and symbols
contain memory-disk root, `nextdma`, ESP, SCSI bus, disk, CD, FFS, and CD9660
support.  Static preflight matches the artifact-real kernel format string
`root on %s` and the embedded-root token `ROOTDEV=/dev/md0a`; the exact
`root on md0a` line remains a runtime gate.  Its root image contains the
manual-install utilities, including `ifconfig` and `mount_nfs`, but not
`sysinst` or a checksum/SHA applet suitable for guest manifest verification.
Static utility preflight covers every external command used before extracting
the target sets: `cat`, `disklabel`, `dd`, `mkdir`, `mount_cd9660`, `newfs`,
`mount_ffs`, `pax`, `chroot`, `sync`, `umount`, `fsck_ffs`, and `halt`, plus
the controller's `ifconfig` and `mount_nfs`.  It deliberately does not require
`cmp` in the RAMDISK; target `/usr/bin/cmp` is supplied by `base.tgz`.
NetBSD still lists next68k onboard SCSI, CD-ROM, and floppy as
unsupported.  Passing this campaign validates QEMU's emulated hardware; it
does not claim new upstream NetBSD support, a bootable ISO, or an installer
floppy.

### Replacement media contract

The corrected campaign replaces the six NetBSD 1.5 inputs with four official
10.1 artifacts:

| Artifact | Bytes | SHA-256 | Release/outer SHA-512 |
| --- | ---: | --- | --- |
| `next68k/installation/boot` | 43,564 | `408927b96731f3ecdb4d58c4d042320a24a0c9b84d9e6089c547f517c6babd53` | ISO-anchored; computed `6513f86eec75d3f8705fea8a9bb78a41751695cbc4c7e761c6f7b15adc52eb76d657eb2f76ba0431ea06d4a6c98713127180d4663ec747fc6846136e91dd830a` |
| `next68k/binary/kernel/netbsd-RAMDISK.gz` | 1,539,241 | `f04fc2329d2e9282b8e4c434d18b90d69be83322da948964c7cae9be97808aee` | `0071b656421ccf1e036afde2946d239964263289742034d7b21d1379f603d21b767afd10504f32bdd68b8eaa194a456c5d6ba31e57b1bd93c00d0890003406e3` |
| `next68k/binary/kernel/netbsd-RAMDISK.symbols.gz` | 100,573 | `884dd06236a4cf0519b914399547a9084e56dc045f9016b8f36849e8119335e5` | `0f92d97ec7a2309ce48de5e25cec1e298a7dce848947de3087ef9f0e7fba464d7023390eda20b0c71814841510afc9d245c8295435ad8c1d490fa64a3adf0ae7` |
| `NetBSD-10.1-next68k.iso` | 289,308,672 | `a0a74335c87dff3d4e2255ea70f9ddf14f02d07cad9f325a06fc080879578089` | `537ec02cc0af9ed0d4b527337c7dea7673c511a34f9cc57b33c76b42d566565ba20a10ee63eadac9abff6a9083b71a80acb010aa6022518f67fd6814d8ed46a8` |

The ISO has no separate checksum entry for `installation/boot`; the official
outer ISO SHA-512 authenticates it, while both extracted-file digests are
recorded.  The exact host gunzip program is descriptor-pinned and recorded.
The derived kernel must be 3,143,304 bytes with SHA-256
`1047b0b70bb27016a32b111e2a9dae0c515be78f2d725dac291b0dfe99d7fa53`;
that identity becomes part of staging and campaign evidence.

### Corrected boot and installation flow

The initial run stages only the official standalone `boot` in TFTP plus the
pinned-gunzip-derived RAMDISK kernel and generated control script in a minimal
NFS export.  NFS transports the kernel and control script only: the guest root
must be `md0a`, and no release set is supplied over NFS.  QEMU attaches the
writable disk at SCSI target 0 and the read-only ISO at target 3.

That staging tree has exactly three files: `tftp/boot`,
`rootfs/netbsd-RAMDISK`, and `rootfs/netbsd-scsi-install.sh`.  The control
script creates `/tmp/disklabel.proto` inside the guest with a single-quoted
heredoc containing the exact 2 GiB geometry; no fourth staged disklabel file
exists.  The official boot program must produce ROM tuple `T=41071`, `D=2456`,
`B=13840` (`Loading boot at 0x4380000: 41071+2456+13840`) and loader banner
`>> NetBSD/next68k BOOT [1.8 (Mon Dec 16 13:08:11 UTC 2024) #0]`.  Both the
install and local-boot observers use these 10.1 values rather than the
historical 1.5 tuple/banner.

Before any RAMDISK shell/terminal response or install command, the controller
requires exact runtime `root on md0a` and visible `nextdma`, `esp`, `scsibus`,
`sd0`, and `cd0`; the DMA observation must be the attachment whose line ends
in `(scsi)`, never either Ethernet DMA channel.  An early shell prompt is
evidence-incomplete.  At the authenticated shell it runs, with a newly
observed prompt after each command, `mkdir -p /mnt2`, then
`ifconfig xe0 10.0.2.15 netmask 255.255.255.0 up` and
`mount_nfs -o ro 10.0.2.2:/ /mnt2`, requiring the intervening shell prompts
before invoking `/mnt2/netbsd-scsi-install.sh`.  The guest then mounts `cd0`,
labels and creates FFS on `sd0`, extracts the official 10.1
`base.tgz`, `etc.tgz`, and `kern-GENERIC.tgz` directly from CD, installs
the official standalone boot program, configures local root, checks the
filesystem, and halts.  The guest has no checksum applet, so media authenticity
comes from the host-authenticated, immutable staged ISO; successful guest
mount and set extraction prove the CD data path without pretending to rehash
the ISO in the RAMDISK.  This is a CD-only guest manual install, not `sysinst`.

The 43,564-byte boot program has 85 complete 512-byte sectors plus a 44-byte
tail.  Each `conv=sync` copy occupies 86 sectors (44,032 bytes): sectors
64--149 at 32 KiB and sectors 192--277 at 96 KiB.  Both remain below the
320-sector front porch.  Verification compares the 43,520-byte head and
44-byte tail of each copy to the authenticated source.  Tail reads use
`bs=1 skip=43520 count=44` for the source and absolute disk byte skips 76,288
and 141,824; those disk skips are not sector counts after selecting `bs=1`.
Because the RAMDISK lacks `cmp`, the script writes the expected head/tail and
both disk readback pairs into `/mnt/tmp` before extracting any set.  It then
extracts `base.tgz` and `etc.tgz` and compares all four pairs with
`chroot /mnt /usr/bin/cmp`; only four successful target comparisons permit
`NETBSD_SCSI_BOOTBLOCKS_OK`.  The kernel set is extracted after that gate.

Implementation is intentionally additive until cutover.  Tasks 12--14 add
parallel `NetBSDSCSI101Artifacts`, staging, classifier, and controller APIs
while the reviewed 1.5 campaign remains active.  Task 15 changes every caller
and test in one commit, rejects mixed authority, and then removes the legacy
six-input paths.  No intermediate production state accepts both contracts.

The next two runs attach only target-0 disk and `-nic none`.  Each must prove
the ROM SCSI load, the authenticated 1.8 standalone banner followed by exact
`entry 0x4001000 esym 0x[0-9a-fA-F]+`, `root on sd0a`, a usable local shell,
and the install or persistence marker.  The entry line is the `loader-disk`
gate:
the official loader does not emit the historical `booting disk
sd(0,0,0)netbsd.` or `open: sd(0,0,0)` debug strings, and either old string is
fatal in the 10.1 classifier.  An entry before the banner, a malformed or
duplicate entry, or a missing entry cannot advance the gate.  The corrected
visible three-phase run and decorated-frame GIF supersede the failed 1.5
capture for publication, while retaining that capture as labelled diagnostic
evidence.

### Task 17 runtime correction (2026-08-26)

This note supersedes every earlier NetBSD 10.1 runtime command and oracle
statement in implementation-plan Tasks 5, 6, 14, and 17; those statements
remain historical and must not be combined with this corrected contract.
The preserved r2 attempt disproved those runtime oracles without weakening
any guest-success gate.  Its fail-closed run is
`/home/blanham/projects/NeXT/lab/work/runs/20260826T221806Z-nbsd-scsi-8058c823baa7-8173b37e`
(`manifest.sha256` SHA-256
`1fae535d921460b3294aef995c5bb133d1672f56f5cf31b49d2d0ea6f9d1b2d9`),
and its 326.2-second decorated FFV1 capture is
`/mnt/e/disks/netbsd-scsi-task9-20260826/netbsd-scsi-101-3phase-r2-ffv1.mkv`
(181,903,368 bytes, SHA-256
`a6a5e60d71e3ff0bc28a2799f33e253571359a3b42353b46ad602ad0ae50f97d`).
The run recorded zero accepted gates and remains diagnostic evidence, not a
pass.

For INSTALL, after the exact ROM command `ben() boot`, advancement requires
the exact observed TFTP path `boot en(0,0,0)boot`.  BOOTP chatter and transfer
progress remain visible but are non-advancing.  INSTALL has no numeric SCSI
load tuple: any `Loading boot at ...` numeric tuple is fatal.  The next
advancing observations are the exact release banner
`>> NetBSD/next68k BOOT [1.8 (Mon Dec 16 13:08:11 UTC 2024) #2]` and then the
exact `boot: ` prompt; only then may the controller send
`en()netbsd-RAMDISK`.  Build `#0`, every other banner, a missing path, or a
prompt before the authenticated banner is fatal or evidence-incomplete as
appropriate.

For each LOCAL boot, the ROM command is exactly `bsd() netbsd`, and the ROM
must report `boot sd(0,0,0)netbsd`, then `booting SCSI target 0, lun 0`, then
the authenticated numeric tuple
`Loading boot at 0x4380000: 41071+2456+13840`, and the same build-`#2`
banner.  Empty-label fallback such as a path without `netbsd`, the wrong
target, any other numeric tuple, or any other banner is fatal.

The original NeXT ROM `scsi_load()` source prints the numeric tuple without a
trailing newline before transferring control to the standalone loader.
Therefore the canonical LOCAL observation is the exact concatenation of the
tuple and banner, and the classifier/controller must consume that compound
form as two ordered observations atomically.  Exact separate tuple and banner
fragments are also permitted solely as a PTY/operator-segmentation
representation of those same consecutive writes: after consuming the exact
full tuple, the next consumed observation must be the exact full build-`#2`
banner.  No intervening observation of any kind is allowed, including ignored
or otherwise non-advancing text.  A display-row wrap is visual layout only; it
does not authorize substring observations, an inserted newline, or additional
whitespace.  No partial prefix, altered compound, unexpected tuple, or wrong
banner is tolerated.

### Task 17 r3 hardware correction (2026-08-26)

This second runtime correction controls the device-oracle and ESP/DMA portions
of the earlier design and plans; the r2 boot-command correction above remains
normative.  Task 17 r3 is **INVALID operator evidence** because the operator
fed `nextdma0 at intio0 addr 0x20000140: channel 0 (scsi)` while the captured
frame proves the guest displayed `nextdma0 at intio0 addr 0x2000010: channel
0 (scsi)`.  R3 cannot satisfy any acceptance gate.  Its exact diagnostic-only
artifacts are:

- frame
  `/home/blanham/projects/NeXT/lab/work/netbsd-scsi-task17-r3-build/operator-logs/r3-kernel-device-full-sequence.png`,
  48,699 bytes, SHA-256
  `0fe490a96781035d0c10a0e96074219f1b26a33bbe56dd7e544d483c12fa240a`;
- capture
  `/mnt/e/disks/netbsd-scsi-task9-20260826/netbsd-scsi-101-3phase-r3-ffv1.mkv`,
  909,053,732 bytes and 539.133 seconds, SHA-256
  `7a58c87502cca49b1f5f0ddd506b3fceca2bebbc27f92afa206ae2a2a0a2df41`.

With kernel timestamp prefixes omitted, the exact visible sequence was:

```text
esp0 at intio0 addr 0x2114000
nextdma0 at intio0 addr 0x2000010: channel 0 (scsi)
nextdma1 at intio0 addr 0x2000110: channel 1 (enetx)
nextdma2 at intio0 addr 0x2000150: channel 2 (enetr)
esp0: ESP200, 20MHz, SCSI ID 7
scsibus0 at esp0: 8 targets, 8 luns per target
esp0: using DMA channel nextdma0
sd0 at scsibus0 target 0 lun 0: <, > disk fixed
sd0(esp0:0:0:0): unsupported sector size: 0x0.  Defaulting to 512 bytes.
sd0: 512, 0 cyl, 64 head, 32 sec, 512 bytes/sect x 1 sectors
sd1 at scsibus0 target 3 lun 0: <, > disk fixed
sd1(esp0:0:3:0): unsupported sector size: 0x0.  Defaulting to 512 bytes.
sd1: 512, 0 cyl, 64 head, 32 sec, 512 bytes/sect x 1 sectors
root on md0a dumps on md0b
```

The two Ethernet DMA lines are authentic intervening attachment text, not
SCSI advancement gates.  The corrected successful order is exact `esp0` at
intio, exact SCSI `nextdma0`, exact ESP200 identity, `scsibus0`, exact `esp0:
using DMA channel nextdma0`, nonzero-capacity `sd0` at target 0, and `cd0` at
target 3, followed by `root on md0a`.  Target-3 `sd1`, any zero
capacity/sector-size evidence, or a missing `cd0` is fatal.

The preserved frame also exposed a real hardware-model fault.  NeXT machine
realization unconditionally sets `esp->dma_enabled = 1`.  Consequently the
guest's data-phase `TRANS | DMA` command consumes an INQUIRY or READ CAPACITY
response before NetBSD enables the NeXT DMA channel and writes DCTL `0xf8`
(`SCSICSR_CPUDMA`).  The early NeXT DMA callback has nowhere valid to place the
transfer, drops it, and leaves zero-filled inquiry/capacity buffers; both
targets are consequently misidentified as zero-capacity disks.

The hardware fix is deliberately narrow: DCTL `SCSICSR_CPUDMA` drives
`esp_dma_enable()`, machine reset holds that input low, and reset/post-load and
qtest helper paths preserve the same derived state.  Authentic NetBSD-order
SELATN with IDENTIFY `0xc0` qtests must first prove target-0 and target-3
INQUIRY prefixes, exact READ CAPACITY bytes, and response retention before
DCTL.  Only after those tests fail for the diagnosed early-consumption reason
may the one machine-glue fix be implemented.  No SCSI payload fabrication,
device-type override, guest workaround, or weakened `cd0` gate is permitted.

#### R4 evidence and DMA ownership addendum (2026-08-26)

R4 operators feed each complete visible timestamped kernel line.  The sole
permitted classification normalization removes one leading prefix matching
`\A\[\s*[0-9]+\.[0-9]{6}\] `; malformed bracketed prefixes are fatal and no
other whitespace or presentation rewrite is allowed.  Evidence retains the
raw fed bytes and normalized value.  Before each required device line is
submitted, a unique QEMU-frame screenshot must be captured, transcribed and
verified character-for-character, SHA-256 hashed, and bound to that raw
observation.  Missing, reused, inferred, or unverified device evidence fails
the run.

The QEMU contract has two independent gates.  Existing disabled-NeXT-DMA
coverage holds DCTL CPUDMA high while the NeXT DMA channel remains disabled;
new coverage separately holds CPUDMA low.  Owner reset must drive CPUDMA's
derived ESP input low before a new TI after reset.  The low migration source
proves a live high-to-low CSR transition, then migrates with no pending TI; the
destination reads back low, retains a new TI, and releases it on a later
CPUDMA-high write.  The high migration source likewise has no pending callback
and its test issues a new TI after load and requires immediate transfer.  High
plus pending is unreachable because issuing TI while high consumes/resumes
immediately.  Moreover, `vmstate_esp` does not migrate or reconstruct
`dma_cb`, `async_len`, or `async_buf`; therefore NeXTSCSI fails migration
safely while either transient is live instead of producing a destination that
can stall or overrun a target buffer.  Generic ESP exposes only the semantic
unsafe-state query, while NeXTSCSI owns the `pre_save_errp` policy and
descriptive error.  Rejection does not mutate source state: a pending low-gate
TI remains releasable by `0xf8`, and migration can be retried after completion.
Transparent migration of live ESP I/O remains deferred pending a generic
ESP/SCSI migration design.

A defensive post-load `esp_dma_enable()` may still synchronously resume
compatibility or future state and therefore runs only after the CSR and NeXT
DMA channel/control state are restored.  CSR store/restore precedes
`next_dma_set_scsi_control()`, which precedes `esp_dma_enable()`.

The gate implementation used two explicit TDD cycles in one coherent commit.
Cycle A first records RED for authentic NetBSD-order inquiry,
capacity, and CPUDMA-low retention, then changes only live CSR-write gating and
records GREEN.  Cycle B then records reset and migration-high RED against that
intermediate implementation while migration-low may be a GREEN guard, adds
reset-low and post-load derived synchronization, and records full GREEN.  The
four TAP checkpoint logs are retained for review.  A separate strict-TDD
follow-up adds fail-safe transient migration rejection without changing the
generic ESP migration ABI.

The final QEMU suite has ten unique task regressions and 39 total registered
tests.  The two rejection cases separately cover pending `dma_cb` and a
pre-TI live `async_len`; idle CPUDMA-high, idle CPUDMA-low, and low/no-pending
migrations remain positive coverage.

Exact SCSI response bytes come from pinned fixture properties: a 512-KiB,
512-byte-block target-0 `scsi-hd` and a four-sector, 2,048-byte-block target-3
`scsi-cd`, both channel 0/lun 0 with explicit `QEMU` vendor, `QEMU HARDDISK` or
`QEMU CD-ROM` product, `2.5+` revision, and SCSI version 5.  Every focused qtest
uses its libqtest runtime `/m68k/next-cube/scsi/...` path and its TAP output
must contain the exact requested name exactly once; zero selected tests is
failure.  Source registrations omit the target prefix, which libqtest adds at
runtime.
