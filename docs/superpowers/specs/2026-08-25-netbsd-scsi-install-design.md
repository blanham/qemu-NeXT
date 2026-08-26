# NetBSD/next68k SCSI Install and Local Boot Design

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
