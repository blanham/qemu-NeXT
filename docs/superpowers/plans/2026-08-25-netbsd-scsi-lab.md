# NetBSD/next68k SCSI Install Lab Implementation Plan

> **Historical record:** Do not execute this document linearly.  Its final
> dated amendments describe the old feature endpoint; the current execution
> authority is the lab plan
> `2026-09-04-netbsd-scsi-current-integration.md`.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a reproducible, authenticated campaign that installs official NetBSD/next68k media onto a blank SCSI disk, verifies the ISO through `cd0`, and proves two network-independent local-disk boots.  Tasks 1--8 record the original 1.5 approach; the 2026-08-26 amendment after Task 11 migrates it to the proven official 10.1 RAMDISK path.

**Architecture:** Add a focused NetBSD SCSI module for media contracts, staging, deterministic disk layout, guest installation scripts, launch arguments, and gate parsing. Integrate it as a new interactive baseline case/profile while keeping the existing NFS-only case unchanged. A campaign coordinator binds the install run, first local boot, and persistence boot into one fail-closed aggregate with immutable media/build/disk identities.  The amended install phase uses NFS only to transport an official 10.1 RAMDISK kernel and reads all install sets from CD.

**Tech Stack:** Python 3.11+, `unittest`, existing `nextcube_lab` baseline/QMP/evidence APIs, QEMU `qemu-img`, NetBSD guest shell utilities, X11/GTK capture and GIF pipeline.

---

## Repositories and dependencies

Lab changes are developed in:

```text
/home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-lab
```

QEMU changes and both plan documents live in:

```text
/home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-qemu
```

The QEMU plan's Tasks 1–3 must pass before the visible guest campaign.  The
lab's existing `netbsd-netboot` / `netbsd-nfs` behavior and evidence schema
remain valid and unchanged.

The historical fixed values used by this plan are:

```python
NETBSD_15_BASE_MD5 = "7feae4818f7af9054f46d196163808aa"
NETBSD_15_ETC_MD5 = "f7171fbcf30504770c6f1771219a7992"
NETBSD_15_KERN_MD5 = "4c4da9a5449135a456d542b0b72671e6"
NETBSD_101_ISO_SHA512 = (
    "537ec02cc0af9ed0d4b527337c7dea7673c511a34f9cc57b33c76b42d566565b"
    "a20a10ee63eadac9abff6a9083b71a80acb010aa6022518f67fd6814d8ed46a8"
)
NETBSD_101_ISO_SHA256 = (
    "a0a74335c87dff3d4e2255ea70f9ddf14f02d07cad9f325a06fc080879578089"
)
NETBSD_15_BOOT_BYTES = 26112
NETBSD_SCSI_DISK_BYTES = 2 * 1024 * 1024 * 1024
NETBSD_SCSI_SECTOR_BYTES = 512
NETBSD_SCSI_FRONT_PORCH = 320
NETBSD_SCSI_BOOT_BLOCKS = (64, 192)  # sectors: 32 KiB and 96 KiB
NETBSD_SCSI_SECTORS_PER_TRACK = 32
NETBSD_SCSI_TRACKS_PER_CYLINDER = 8
NETBSD_SCSI_SWAP_SECTORS = 131072
```

### Task 1: Define and verify the complete media identity contract

**Files:**

- Create: `nextcube_lab/netbsd_scsi.py`
- Create: `tests/test_netbsd_scsi.py`

- [ ] **Step 1: Write failing artifact-contract tests**

Create `NetBSDSCSIArtifactsTests` with temporary regular files.  Cover:

The exact test methods are `test_verify_records_every_digest`,
`test_verify_rejects_wrong_release_md5`,
`test_verify_rejects_wrong_iso_sha512`,
`test_verify_rejects_wrong_user_sha256`,
`test_verify_rejects_relative_symlink_and_multiply_linked_paths`, and
`test_verify_rejects_change_while_hashing`.

The success assertion requires records named `boot`, `root_archive`,
`base_set`, `etc_set`, `kernel_set`, and `iso`; every record contains absolute
`path`, integer `size`, lowercase `sha256`, and the applicable upstream MD5 or
SHA-512.

- [ ] **Step 2: Run the new module tests and confirm import failure**

```sh
python3 -m unittest tests.test_netbsd_scsi -v
```

Expected: FAIL because `nextcube_lab.netbsd_scsi` does not exist.

- [ ] **Step 3: Implement point-in-time media identity verification**

Define:

```python
@dataclass(frozen=True)
class NetBSDSCSIArtifacts:
    boot: Path
    boot_sha256: str
    root_archive: Path
    root_archive_sha256: str
    base_set: Path
    base_set_sha256: str
    etc_set: Path
    etc_set_sha256: str
    kernel_set: Path
    kernel_set_sha256: str
    iso: Path
    iso_sha256: str
```

Implement `verify_netbsd_scsi_artifacts(artifacts: NetBSDSCSIArtifacts)` with
return type `dict[str, dict[str, int | str]]`.

The returned dictionary is authenticated point-in-time identity evidence, not
durable authority over mutable pathnames.  No finite sequence of pathname
checks can make a caller-owned file immutable after this function returns.
Task 2 therefore reopens every consumed input into a descriptor-backed group,
copies only from those pinned descriptors, revalidates them after consumption,
and launches QEMU only with private staged outputs.

Use `lstat`, `os.open` with `O_RDONLY | O_NOFOLLOW | O_CLOEXEC`, `fstat`, a
1 MiB streaming digest loop, and before/after identity checks.  Require
absolute, single-link regular files and 64-character lowercase caller
SHA-256 values.  Simultaneously calculate MD5 for the three 1.5 sets and
SHA-512 for the ISO, comparing them with the constants above.  Close every
descriptor on every error path.  Raise `BaselineError` with the artifact name
and reason, but never include file content.

- [ ] **Step 4: Run contract tests**

```sh
python3 -m unittest tests.test_netbsd_scsi -v
```

Expected: PASS.

- [ ] **Step 5: Commit the media contract**

```sh
git add nextcube_lab/netbsd_scsi.py tests/test_netbsd_scsi.py
git commit -m "netbsd: authenticate SCSI install media"
```

### Task 2: Stage the rescue root and guest-native installer

**Files:**

- Modify: `nextcube_lab/netbsd_scsi.py`
- Modify: `nextcube_lab/netbsd_netboot.py`
- Modify: `tests/test_netbsd_scsi.py`
- Modify: `tests/test_netbsd_netboot.py`

- [ ] **Step 1: Write failing staging and script tests**

Add tests which build tiny tar fixtures and assert:

The exact test methods are
`test_stage_scsi_install_reuses_hardened_nfs_extraction`,
`test_stage_scsi_install_copies_sets_and_boot_without_symlinks`,
`test_stage_scsi_install_writes_exact_disklabel_and_script`,
`test_stage_scsi_install_cleans_partial_tree_on_failure`, and
`test_install_script_has_fail_closed_markers`,
`test_stage_scsi_install_uses_pinned_sources_and_stages_iso`,
`test_stage_scsi_install_rejects_source_change_during_copy`, and
`test_stage_scsi_install_closes_source_descriptors_once`,
`test_stage_scsi_install_rejects_wrong_boot_size`,
`test_stage_scsi_install_preserves_replaced_outputs_on_rollback`, and
`test_stage_scsi_install_derives_layout_from_public_constants`.

Require the staged root to contain:

```text
install/boot
install/base.tgz
install/etc.tgz
install/kern.tgz
install/disklabel.proto
install/netbsd-scsi-install.sh
```

- [ ] **Step 2: Run the tests and verify missing APIs**

```sh
python3 -m unittest tests.test_netbsd_scsi.NetBSDSCSIStagingTests -v
```

Expected: FAIL because the staging APIs are undefined.

- [ ] **Step 3: Implement staging around the existing hardened extractor**

Define:

```python
@dataclass(frozen=True)
class NetBSDSCSIStaging:
    tftp: Path
    rootfs: Path
    install_dir: Path
    iso: Path
    artifacts: dict[str, dict[str, int | str]]
    tree_identities: dict[str, tuple[int, int, int]]
    file_identities: dict[str, tuple[int, int, int, int, int, int]]
    disklabel_sha256: str
    install_script_sha256: str
```

Implement `stage_netbsd_scsi_install(root: Path,
artifacts: NetBSDSCSIArtifacts) -> NetBSDSCSIStaging`.

Refactor the internals of existing `stage_netbsd_artifacts()` so a private,
trusted transactional extension can run while its authenticated `run_fd`,
`tftp_fd`, and `root_fd` remain open.  The public API and existing NFS behavior
must remain byte-for-byte compatible.  The core owns and closes those three
descriptors; the extension borrows but never closes them.

The SCSI extension creates `install` relative to retained `root_fd` and
`media` relative to retained `run_fd`, immediately opens each child, and
requires the child `fstat` identity to equal the parent-relative `lstat`
identity.  It registers rollback state immediately after each identity is
established, not only on success.  Extension-owned descriptors have explicit
exact-once ownership.

Securely copy newly authenticated, descriptor-pinned set and boot inputs into
`install/`.  Copy the ISO through its pinned descriptor to a private
`media/NetBSD-10.1-next68k.iso`, and record that path as `staging.iso`; later
QEMU launch code must use this staged ISO rather than the caller-owned source.
Never reopen a source pathname after authenticating its descriptor.  Rewind
each descriptor with `os.lseek(fd, 0, os.SEEK_SET)` before copying, hash while
copying, and revalidate both descriptor and pathname metadata after its final
consumption.  Adopt each returned source descriptor with the guarded
`current = verify(); append(current); current = None` pattern so an exception
between open and list insertion cannot leak it.  Close every descriptor
exactly once on every path.

On failure, preserve authority descriptors until rollback finishes.  Reuse
the authenticated quarantine/removal machinery from `netbsd_netboot` for the
recorded `install` and `media` inodes; never reopen an absolute staging path or
unlink an unverified replacement.  Pre-existing or swapped-in paths remain
untouched when detected before authenticated quarantine.  The private mode
0700 run tree is a quiescent namespace during staging and rollback: concurrent
same-UID mutation after an authenticated entry is renamed to its random
quarantine name is outside the contract because POSIX provides no atomic
compare-and-unlink primitive.  Do not claim stronger deletion atomicity.
Register the extension's directories and files with the shared
`MAX_STAGED_FILESYSTEM_ENTRIES` counter, run the existing final actual-tree
count after the extension, and preserve the original exception while
attempting every extension and base-tree rollback.

Require the boot input to be exactly `NETBSD_15_BOOT_BYTES`.  Derive the
disklabel partition sizes/offsets and every boot-copy/verification offset from
the public geometry and boot constants.  Assert the two padded boot copies do
not overlap and both end before `NETBSD_SCSI_FRONT_PORCH`.
Create files with `O_CREAT | O_EXCL | O_NOFOLLOW`, mode 0444 for inputs and
0555 for the script, fsync files and directories, and remove the entire new
staging tree on failure.

Fsync in order: leaf files, `install`/`media`, then retained `root_fd`/`run_fd`,
followed by final identity checks.  Persist authenticated identities for
`tftp`, `rootfs`, `install`, `media`, and every staged input file in
`NetBSDSCSIStaging`; later launch code must reauthenticate these identities
before use rather than treating returned pathnames as authority.

The durable commit boundary is only after those fsyncs, final identity checks,
construction of the complete `NetBSDSCSIStaging` record, and atomic batch
adoption of every remaining cleanup-only authority lease into the central
descriptor ownership ledger.  Interruption before or during batch adoption
remains pre-commit and must roll back.  Before that
boundary, every ordinary or control failure is deferred until authenticated
rollback and exact-once descriptor cleanup finish.  After that boundary,
remaining authority-lease close outcomes are cleanup-only: route them through
the central descriptor ownership ledger, retain uncertain descriptors, emit a
`ResourceWarning`, and still return the authenticated staging record.  A late
close interruption must never leave a committed staging tree unreported.

Use this exact disklabel template for the initial 2 GiB disk:

```text
type: SCSI
disk: QEMU SCSI
label: NetBSD15
flags:
bytes/sector: 512
sectors/track: 32
tracks/cylinder: 8
sectors/cylinder: 256
cylinders: 16384
total sectors: 4194304
rpm: 3600
interleave: 1
trackskew: 0
cylinderskew: 0
headswitch: 0
track-to-track seek: 0
drivedata: 0
8 partitions:
#        size    offset     fstype  [fsize bsize cpg]
  a:  4062912       320     4.2BSD    1024  8192    16
  b:   131072   4063232       swap
  c:  4194304         0     unused       0     0
```

- [ ] **Step 4: Generate the exact guest installer script**

The staged script must be `/bin/sh`, `set -e`, and emit unique markers only
after each command succeeds:

```sh
#!/bin/sh
set -e
PATH=/sbin:/bin:/usr/sbin:/usr/bin
export PATH

echo NETBSD_SCSI_INSTALL_BEGIN
disklabel -R -r sd0 /install/disklabel.proto
disklabel -r sd0
echo NETBSD_SCSI_LABEL_OK

dd if=/install/boot of=/dev/rsd0c bs=1024 seek=32 conv=sync
dd if=/install/boot of=/dev/rsd0c bs=1024 seek=96 conv=sync
dd if=/install/boot bs=1024 count=25 of=/tmp/boot.expected.head
dd if=/install/boot bs=512 skip=50 count=1 of=/tmp/boot.expected.tail
dd if=/dev/rsd0c bs=1024 skip=32 count=25 of=/tmp/boot.1.head
dd if=/dev/rsd0c bs=512 skip=114 count=1 of=/tmp/boot.1.tail
dd if=/dev/rsd0c bs=1024 skip=96 count=25 of=/tmp/boot.2.head
dd if=/dev/rsd0c bs=512 skip=242 count=1 of=/tmp/boot.2.tail
cmp /tmp/boot.expected.head /tmp/boot.1.head
cmp /tmp/boot.expected.tail /tmp/boot.1.tail
cmp /tmp/boot.expected.head /tmp/boot.2.head
cmp /tmp/boot.expected.tail /tmp/boot.2.tail
echo NETBSD_SCSI_BOOTBLOCKS_OK

newfs /dev/rsd0a
mount -t ffs /dev/sd0a /mnt
cd /mnt
pax -rzpe -f /install/base.tgz
pax -rzpe -f /install/etc.tgz
pax -rzpe -f /install/kern.tgz
echo '/dev/sd0a / ffs rw 1 1' > /mnt/etc/fstab
echo 'swap /tmp mfs rw,-s=8192 0 0' >> /mnt/etc/fstab
echo 'rc_configured=NO' >> /mnt/etc/rc.conf
echo installed-by-nextcube-lab > /mnt/NETBSD_SCSI_INSTALLED
cd /
sync
umount /mnt
fsck_ffs -fn /dev/rsd0a
echo NETBSD_SCSI_FILESYSTEM_OK

mkdir -p /mnt/cd
mount_cd9660 /dev/cd0a /mnt/cd
set -- `cksum /mnt/cd/next68k/INSTALL.txt`
test "$1" = 1028267151
test "$2" = 66373
set -- `cksum /mnt/cd/next68k/binary/kernel/SHA512`
test "$1" = 3551046318
test "$2" = 482
set -- `cksum /mnt/cd/next68k/installation/boot`
test "$1" = 3958082764
test "$2" = 43564
umount /mnt/cd
echo NETBSD_SCSI_CD_OK
echo NETBSD_SCSI_INSTALL_COMPLETE
halt -p
```

The boot verification deliberately compares the first 25 KiB and final 512
bytes separately because `conv=sync` pads the 26,112-byte source to 26 KiB.
The CD checks are the exact POSIX CRC and byte counts extracted from the
authenticated ISO named in Task 8.

- [ ] **Step 5: Run tests and commit**

```sh
python3 -m unittest tests.test_netbsd_scsi -v
git diff --check
git add nextcube_lab/netbsd_scsi.py tests/test_netbsd_scsi.py
git commit -m "netbsd: stage guest-native SCSI installer"
```

### Task 3: Create a blank base disk and disposable overlay safely

**Files:**

- Modify: `nextcube_lab/netbsd_scsi.py`
- Modify: `tests/test_netbsd_scsi.py`

- [ ] **Step 1: Write failing disk-lifecycle tests**

Add:

The exact test methods are
`test_create_disk_uses_exact_raw_size_and_qcow2_backing`,
`test_create_disk_rejects_existing_or_symlink_outputs`,
`test_create_disk_records_qemu_img_and_file_identities`, and
`test_revalidate_disk_rejects_base_or_overlay_replacement`.

Use a fake `qemu-img` executable in tests and assert exact argv, not shell
strings.

- [ ] **Step 2: Implement disk lifecycle**

Define:

```python
@dataclass(frozen=True)
class NetBSDSCSIDisk:
    base: Path
    overlay: Path
    size: int
    base_identity: tuple[int, int, int, int]
    overlay_identity: tuple[int, int, int]
    qemu_img: Path
    qemu_img_sha256: str
    qemu_img_identity: tuple[int, int, int, int, int, int]
```

Implement `create_netbsd_scsi_disk(run_root: Path, qemu_img: Path) ->
NetBSDSCSIDisk` and `revalidate_netbsd_scsi_disk(disk: NetBSDSCSIDisk) ->
None`.

Create the base and overlay with exact argv:

```python
[str(qemu_img), "create", "-f", "raw", str(base), "2147483648"]
[
    str(qemu_img), "create", "-f", "qcow2", "-F", "raw",
    "-b", str(base), str(overlay),
]
```

Use private run-owned paths, no shell, no overwrite flag, captured stdout and
stderr, and a finite timeout.  Require regular non-symlink outputs, exact raw
virtual size from `qemu-img info --output=json`, and stable identities.

Authenticate and retain one read-only `qemu-img` descriptor before creating
either output.  Hash and record that descriptor, require its pathname identity
to remain stable, and execute all raw-create, overlay-create, and info calls
through `/proc/self/fd/<descriptor>` with `pass_fds` while preserving the
required textual path as `argv[0]`.  A pathname replacement between calls must
not change the executed binary.

All run-directory, executable, reservation, and quarantine descriptors use the
central exact-once ownership ledger from `baseline.authority`; raw `os.open` /
`os.close` ownership is not sufficient.  The raw base identity includes its
exact physical size because it remains immutable.  The qcow2 overlay identity
contains only device, inode, and regular-file type: ordinary guest writes may
grow its physical size and must pass revalidation, while replacement or type
change must fail.

- [ ] **Step 3: Run tests and commit**

```sh
python3 -m unittest tests.test_netbsd_scsi.NetBSDSCSIDiskTests -v
git add nextcube_lab/netbsd_scsi.py tests/test_netbsd_scsi.py
git commit -m "netbsd: create disposable SCSI install disks"
```

### Task 4: Build exact install and local-boot QEMU arguments

**Files:**

- Modify: `nextcube_lab/netbsd_scsi.py`
- Modify: `tests/test_netbsd_scsi.py`

- [ ] **Step 1: Write failing argv tests**

Assert byte-for-byte tuples for:

The exact test methods are
`test_build_install_argv_has_nfs_disk_and_read_only_cd`,
`test_build_local_boot_argv_has_disk_and_no_network_backend`, and
`test_argv_rejects_non_absolute_or_aliasing_paths`.

- [ ] **Step 2: Implement the launch builders**

Define `build_netbsd_scsi_install_argv(qemu: Path, rom: Path, staging:
NetBSDSCSIStaging, disk: NetBSDSCSIDisk, qmp: Path, display: str)`
and `build_netbsd_scsi_boot_argv(qemu: Path, rom: Path, overlay: Path, qmp:
Path, display: str)`.  Both return immutable tuples of strings.

The install tuple contains these ordered device arguments:

```text
-drive file=<overlay>,if=scsi,index=0,format=qcow2
-drive file=<staging.iso>,if=scsi,index=3,format=raw,media=cdrom,readonly=on
```

and the existing NetBSD NFS-root `-fsdev`, `-netdev`, `-object nfs-server`,
and `next-mb8795` arguments.  The local-boot tuple contains only target-0
overlay storage and `-nic none`; it must not contain `-fsdev`, `-netdev`,
`nfs-server`, TFTP, or a NIC device.  Both use `-M next-station`, v66 ROM,
64 MiB, GTK, run-owned QMP, and `-no-reboot`.

- [ ] **Step 3: Run tests and commit**

```sh
python3 -m unittest tests.test_netbsd_scsi.NetBSDSCSIArgvTests -v
git add nextcube_lab/netbsd_scsi.py tests/test_netbsd_scsi.py
git commit -m "netbsd: define SCSI install and boot launches"
```

### Task 5: Add the baseline case, CLI contract, and run IDs

**Files:**

- Modify: `nextcube_lab/baseline/model.py`
- Modify: `nextcube_lab/baseline/runner.py`
- Modify: `nextcube_lab/baseline_cli.py`
- Modify: `tests/test_baseline_model.py`
- Modify: `tests/test_baseline_runner.py`

- [ ] **Step 1: Write failing model and parser tests**

Add `netbsd-scsi` as a case and `netbsd-scsi-install` plus
`netbsd-scsi-boot` as launch profiles.  Assert run token `nbsd-scsi` and
reject the profiles for automated/headless mode or any other case.

Add required CLI paths and SHA-256 flags for boot, root archive, three sets,
ISO, and persistent campaign disk.  Tests require every value for install,
forbid set/ISO/NFS inputs for local boot, and retain the existing NFS-only
parser behavior byte-for-byte.

- [ ] **Step 2: Run focused tests and confirm rejection**

```sh
python3 -m unittest \
  tests.test_baseline_model \
  tests.test_baseline_runner.BaselineParserTests -v
```

Expected: FAIL because the new case/profile is unknown.

- [ ] **Step 3: Extend types and validation without changing old profiles**

Add `netbsd-scsi` to the case literals and run-path map.  Extend `RunConfig`
with optional SCSI artifact and campaign-disk fields.  `_validate_run_config`
must enforce:

```python
if config.case == "netbsd-scsi":
    if config.mode != "interactive" or config.display != "gtk":
        raise BaselineError("NetBSD SCSI acceptance requires interactive GTK")
    if config.launch_profile not in (
        "netbsd-scsi-install", "netbsd-scsi-boot"
    ):
        raise BaselineError("NetBSD SCSI launch profile is invalid")
```

The install profile owns ROM command `ben() boot`, then standalone command
`en()netbsd`; the boot profile owns ROM command `bsd()`.  Do not accept user
overrides for those three commands.

- [ ] **Step 4: Run tests and commit**

```sh
python3 -m unittest \
  tests.test_baseline_model tests.test_baseline_runner -v
git add nextcube_lab/baseline/model.py nextcube_lab/baseline/runner.py \
  nextcube_lab/baseline_cli.py tests/test_baseline_model.py \
  tests/test_baseline_runner.py
git commit -m "baseline: add NetBSD SCSI launch profiles"
```

### Task 6: Add visible install and local-root gate controllers

**Files:**

- Modify: `nextcube_lab/netbsd_scsi.py`
- Modify: `nextcube_lab/baseline/runner.py`
- Modify: `tests/test_netbsd_scsi.py`
- Modify: `tests/test_baseline_runner.py`

- [ ] **Step 1: Write failing gate-parser tests**

The install run requires, in order:

```python
NETBSD_SCSI_INSTALL_GATES = (
    "devices", "label", "bootblocks", "filesystem", "cdrom", "complete"
)
```

The local boot requires:

```python
NETBSD_SCSI_BOOT_GATES = (
    "rom-disk", "loader-disk", "kernel-sd0a", "local-shell", "marker"
)
```

Tests cover correct order, duplicate markers, skipped markers, NFS-root text,
timeout after the last visible gate, process exit before authenticated QMP
shutdown, and a screenshot that does not change between adjacent gates.

- [ ] **Step 2: Implement strict transcript recognizers**

Define pure functions `classify_netbsd_scsi_install_line(line: str) -> str |
None` and `classify_netbsd_scsi_boot_line(line: str) -> str | None`.

Accept exact generated markers for install steps.  For boot, require observed
text matching the ROM disk load, the `>> NetBSD/next68k BOOT` banner with an
`sd(` pathname, `root on sd0a`, a shell prompt, and marker file content.
Explicitly return a fatal classification for `root on xe`, `root on nfs`, or
the NFS server address.

- [ ] **Step 3: Integrate controlled visible interactions**

Reuse the existing QMP key injection and screenshot/register capture.  The
install controller sends `ben() boot`, then `en()netbsd`, accepts the two
single-user prompts with Return, and types:

```text
/install/netbsd-scsi-install.sh
```

The local controller sends `bsd()`, accepts single-user prompts, then types:

```text
cat /NETBSD_SCSI_INSTALLED
echo persisted-after-local-boot > /NETBSD_SCSI_PERSISTED
sync
halt -p
```

On the second local boot it types:

```text
cat /NETBSD_SCSI_INSTALLED /NETBSD_SCSI_PERSISTED
halt -p
```

Never inject commands before the corresponding visible prompt gate.  QMP
transcripts must contain events and lifecycle commands but no secrets.

- [ ] **Step 4: Run controller tests and commit**

```sh
python3 -m unittest \
  tests.test_netbsd_scsi tests.test_baseline_runner -v
git add nextcube_lab/netbsd_scsi.py nextcube_lab/baseline/runner.py \
  tests/test_netbsd_scsi.py tests/test_baseline_runner.py
git commit -m "baseline: gate NetBSD SCSI installation and boot"
```

### Task 7: Bind three runs into a fail-closed campaign

**Files:**

- Create: `nextcube_lab/netbsd_scsi_campaign.py`
- Create: `nextcube_lab/netbsd_scsi_campaign_cli.py`
- Create: `tests/test_netbsd_scsi_campaign.py`
- Modify: `nextcube_lab/baseline/runner.py`
- Modify: `tests/test_baseline_runner.py`

> **Execution amendment (2026-08-26):** Tasks 2--5 created the authenticated
> SCSI staging, disk-lifecycle, launch-argument, profile, and controller APIs,
> but the baseline runner was not yet bound to the first three.  Task 7 must
> add that missing composition boundary before implementing the coordinator.
> The campaign owns one `NetBSDSCSIDisk` record for all three phases; the
> runner must consume and revalidate that record rather than create, adopt, or
> rediscover a disk from a pathname.  The install phase must transactionally
> stage the six authenticated inputs in its run root and call
> `build_netbsd_scsi_install_argv`; both boot phases must call
> `build_netbsd_scsi_boot_argv` with no CD-ROM or network backend.  Require the
> configured campaign-disk pathname to equal the owned overlay, revalidate the
> base/overlay/qemu-img identity before argument construction, immediately
> before launch, and after authenticated shutdown, and record the SCSI staging
> and disk identity in raw evidence.  Preserve the existing two-argument
> `run_baseline` API for every non-campaign caller; any added authority input
> must be keyword-only and rejected for non-SCSI runs.  Add focused runner
> regressions proving the generic no-disk fallthrough is impossible and that
> missing, mismatched, replaced, or phase-inappropriate SCSI authority fails
> closed.

- [ ] **Step 0: Bind authenticated SCSI launches into the baseline runner**

Compose the existing Task 2--4 APIs as described in the execution amendment.
Do not duplicate their staging, disk authentication, or argv logic in the
campaign module.

- [ ] **Step 1: Write failing aggregate tests**

Cover successful install + boot 1 + boot 2, failure in every phase, artifact
drift, build drift, overlay replacement, incorrect phase order, reused run,
missing QMP shutdown, missing persistent marker, and aggregate overwrite.

- [ ] **Step 2: Implement the campaign record**

Define:

```python
@dataclass(frozen=True)
class NetBSDSCSICampaign:
    schema: int
    status: str
    install_run: Path
    first_boot_run: Path
    persistence_boot_run: Path
    qemu_commit: str
    artifact_sha256: Mapping[str, str]
    installed_disk_sha256: str
```

Implement `verify_netbsd_scsi_campaign(path: Path, *, workspace: Path,
qemu_source: Path, build_metadata: Path) -> NetBSDSCSICampaign`.

The coordinator creates the disk once, launches the three phases in order,
revalidates all sources and disk identities between phases, and writes a new
aggregate JSON with `O_EXCL` only after every raw bundle independently
verifies.  A failure retains phase evidence and writes no passing aggregate.

- [ ] **Step 3: Add CLI subcommands**

Implement `run` and `verify` in `netbsd_scsi_campaign_cli.py`.  Use the same
explicit workspace, source, build metadata, work/evidence/results roots, ROM,
artifact paths, hashes, display, and timeout contracts as the baseline tools.
No defaults may search the filesystem for ROMs or media.

- [ ] **Step 4: Run campaign tests and commit**

```sh
python3 -m unittest tests.test_netbsd_scsi_campaign -v
git add nextcube_lab/netbsd_scsi_campaign.py \
  nextcube_lab/netbsd_scsi_campaign_cli.py \
  tests/test_netbsd_scsi_campaign.py
git commit -m "netbsd: bind SCSI install and boot evidence"
```

### Task 8: Acquire and authenticate official media

**Files:**

- External output: `/mnt/e/disks/NetBSD-10.1-next68k.iso`
- No Git-tracked media

- [ ] **Step 1: Download from the official NetBSD CDN**

Run with approved external-write/network access:

```sh
curl --fail --location --continue-at - \
  --output /mnt/e/disks/NetBSD-10.1-next68k.iso \
  https://cdn.netbsd.org/pub/NetBSD/images/10.1/NetBSD-10.1-next68k.iso
```

- [ ] **Step 2: Verify upstream SHA-512 and record SHA-256**

```sh
sha512sum /mnt/e/disks/NetBSD-10.1-next68k.iso
sha256sum /mnt/e/disks/NetBSD-10.1-next68k.iso
```

Expected SHA-512 is the `NETBSD_101_ISO_SHA512` constant at the top of this
plan; expected SHA-256 is `NETBSD_101_ISO_SHA256`.  Stop on mismatch.  Do not
download a floppy: the official next68k release contains none.

- [ ] **Step 3: Verify preserved NetBSD 1.5 inputs**

Run the lab verifier over `usr/mdec/boot`, `root-nfs.tgz`, `base.tgz`,
`etc.tgz`, and `kern.tgz`.  Require the three upstream MD5 values and record
all SHA-256 values in the campaign command and evidence.

### Task 9: Run the visible installation and two local boots

> **Historical execution record (2026-08-26):** This task was attempted with
> the six-input NetBSD 1.5 contract.  The authentic release kernel reached an
> NFS-root single-user shell but attached no ESP/SCSI/disk/CD devices because
> those facilities are absent from that kernel.  The controller failed closed
> without starting the installer.  Preserve the run as diagnostic evidence;
> do not mark this task complete or weaken its gates.  Tasks 12--18 below
> replace the guest/media assumption before a new three-phase execution.

**Files:**

- Generated evidence only under configured lab work/results roots

- [ ] **Step 1: Build the registered QEMU worktree**

Use `nextcube-baseline build` with the exact QEMU worktree, a new build
directory, pinned SLiRP fallback, and a new metadata summary.  Require the
source tree to be clean and the required NeXT tests to pass.

- [ ] **Step 2: Run the campaign under the real X11 GTK display**

Start the campaign CLI with explicit v66 ROM, six NetBSD inputs and hashes,
build metadata, and `/mnt/e/disks/NetBSD-10.1-next68k.iso`.  Record a lossless
decorated-window MKV covering ROM boot through the final local shell.

- [ ] **Step 3: Diagnose the first real divergence systematically**

If any real gate fails, preserve it and invoke `superpowers:systematic-debugging`.
Classify the cause as QEMU hardware, guest install script/media layout, or lab
observation.  Add the smallest failing unit/qtest before changing its owner.
Do not retry with weaker gates, NFS fallback, or a host-built filesystem.

- [ ] **Step 4: Verify the promoted campaign independently**

Run the campaign `verify` subcommand using the same explicit source, build,
ROM, and media inputs.  Require three verified raw bundles and matching final
disk identity.

### Task 10: Publish evidence, GIF, and documentation

> **Superseded after the failed Task 9 run:** Do not execute this historical
> 1.5 publication task.  Task 18 replaces it after the corrected 10.1 campaign.

**Files:**

- Modify: `README.md`
- Add: `docs/boot/netbsd-scsi.gif`
- Modify: `tests/test_boot_gif.py`
- Modify: `tests/test_netbsd_scsi_campaign.py`

- [ ] **Step 1: Add documentation contract tests**

Assert the README includes the exact official ISO URL and SHA-512, explains
the absence of a next68k install floppy, shows the install and verification
commands, uses `bsd()` for local boot, names `sd0a` and `cd0`, links the GIF,
and does not call the ISO bootable.

- [ ] **Step 2: Encode and verify the boot GIF from the lossless capture**

Use the established 15 fps two-pass palette workflow.  Accelerate static ROM
and long disk-copy spans, retain readable prompts and the `root on sd0a`
transition at normal speed, and hold the final local shell for exactly three
seconds.  Preserve the decorated QEMU window, keep the output below 20 MiB,
and run the standalone GIF verifier.

- [ ] **Step 3: Document reproduction and limitations**

Add a `NetBSD SCSI install and local boot` section containing authenticated
inputs, target IDs, disk geometry, three-phase workflow, exact commands,
acceptance gates, evidence paths, GIF, and limitations.  State that this is a
manual guest-native installation enabled by revived SCSI, not historical
sysinst or floppy media.

- [ ] **Step 4: Run docs/GIF tests and commit**

```sh
python3 -m unittest \
  tests.test_boot_gif tests.test_netbsd_scsi_campaign -v
git diff --check
git add README.md docs/boot/netbsd-scsi.gif tests/test_boot_gif.py \
  tests/test_netbsd_scsi_campaign.py
git commit -m "docs: publish NetBSD SCSI boot evidence"
```

### Task 11: Final lab verification

> **Superseded after the failed Task 9 run:** Do not execute this historical
> 1.5 final-verification task.  Tasks 16--18 replace it for the corrected 10.1
> campaign.

**Files:** None expected

- [ ] **Step 1: Run all focused tests**

```sh
python3 -m unittest \
  tests.test_netbsd_scsi \
  tests.test_netbsd_scsi_campaign \
  tests.test_baseline_model \
  tests.test_baseline_runner \
  tests.test_baseline_bundle \
  tests.test_boot_gif -v
```

- [ ] **Step 2: Run the complete lab suite**

```sh
python3 -m unittest discover -s tests -p 'test_*.py'
```

Expected baseline before this work was 1,177 passing tests and one skip.  The
new total must pass with no additional skip unless the skip is independently
reviewed and justified.

- [ ] **Step 3: Reverify real evidence and repository hygiene**

Run the campaign verifier again, then:

```sh
git diff --check
git status --short --branch
git log --oneline --decorate -12
```

Expected: clean feature worktree; external ISO and generated disk/evidence are
not tracked.

---

## Implementation amendment: official 10.1 RAMDISK path (2026-08-26)

Tasks 1--8 above are completed historical work.  The first Task 9 execution
failed closed because the official 1.5 release kernel has no SCSI/CD drivers.
Tasks 10--11 are superseded and must not be executed.  Execute Tasks 12--18 in
order with a fresh implementer and spec/quality review after each
implementation task.

The exact replacement constants are:

```python
NETBSD_101_BOOT_BYTES = 43_564
NETBSD_101_BOOT_SHA256 = "408927b96731f3ecdb4d58c4d042320a24a0c9b84d9e6089c547f517c6babd53"
NETBSD_101_BOOT_SHA512 = "6513f86eec75d3f8705fea8a9bb78a41751695cbc4c7e761c6f7b15adc52eb76d657eb2f76ba0431ea06d4a6c98713127180d4663ec747fc6846136e91dd830a"
NETBSD_101_RAMDISK_BYTES = 1_539_241
NETBSD_101_RAMDISK_SHA256 = "f04fc2329d2e9282b8e4c434d18b90d69be83322da948964c7cae9be97808aee"
NETBSD_101_RAMDISK_SHA512 = "0071b656421ccf1e036afde2946d239964263289742034d7b21d1379f603d21b767afd10504f32bdd68b8eaa194a456c5d6ba31e57b1bd93c00d0890003406e3"
NETBSD_101_RAMDISK_SYMBOLS_BYTES = 100_573
NETBSD_101_RAMDISK_SYMBOLS_SHA256 = "884dd06236a4cf0519b914399547a9084e56dc045f9016b8f36849e8119335e5"
NETBSD_101_RAMDISK_SYMBOLS_SHA512 = "0f92d97ec7a2309ce48de5e25cec1e298a7dce848947de3087ef9f0e7fba464d7023390eda20b0c71814841510afc9d245c8295435ad8c1d490fa64a3adf0ae7"
NETBSD_101_RAMDISK_UNCOMPRESSED_BYTES = 3_143_304
NETBSD_101_RAMDISK_UNCOMPRESSED_SHA256 = "1047b0b70bb27016a32b111e2a9dae0c515be78f2d725dac291b0dfe99d7fa53"
```

The ISO constants remain the 289,308,672-byte SHA-256/SHA-512 values at the
top of this plan.

### Task 12: Add the parallel 10.1 media contract and capability preflight

**Files:**

- Modify: `nextcube_lab/netbsd_scsi.py`
- Modify: `tests/test_netbsd_scsi.py`

- [ ] **Step 1: Write the failing four-artifact contract tests**

Add exact methods to `NetBSDSCSI101ArtifactsTests`:

```python
test_verify_101_records_exact_four_artifacts_and_upstream_hashes
test_verify_101_rejects_each_wrong_size
test_verify_101_rejects_each_wrong_sha512
test_verify_101_rejects_ramdisk_without_md_root_and_scsi_capabilities
test_verify_101_rejects_symbols_without_esp_sd_cd_and_cd9660
test_verify_101_rejects_ramdisk_without_control_network_utilities
test_verify_101_rejects_each_missing_preextraction_script_utility
test_verify_101_does_not_require_cmp_in_ramdisk
test_verify_101_rejects_capability_input_drift
```

The success record has only `boot`, `ramdisk`, `ramdisk_symbols`, and `iso`.
Require the exact sizes and digests above.  For `boot`, require both fixed
computed digests; the independently authenticated outer ISO is a separate
fixed identity because the ISO contains no per-file boot SHA-512 entry.  Do
not claim or test byte-for-byte ISO member comparison in this tranche.
Decompressed capability evidence must include the
artifact-real format string `root on %s`, embedded-root token
`ROOTDEV=/dev/md0a`, `nextdma_esp_intr`, `esp_ca`, `scsibus_ca`, `sd_ca`,
`cd_ca` and `cd9660_mount`.  The embedded RAMDISK utility preflight must
separately require every external command used before target-set extraction:
`cat`, `disklabel`, `dd`, `mkdir`, `mount_cd9660`, `newfs`, `mount_ffs`, `pax`,
`chroot`, `sync`, `umount`, `fsck_ffs`, and `halt`, plus controller commands
`ifconfig` and `mount_nfs`.  Shell builtins (`cd`, `echo`, `export`, `set`, and
redirection) are not external-command tokens.  Do not require `cmp` in the
RAMDISK: `/usr/bin/cmp` becomes available only inside the target after the
authenticated CD's `base.tgz` is successfully extracted.  Absence of any
required pre-extraction token is fatal.  Do not require static `root on md0a`:
only the running kernel can produce that substituted line.

- [ ] **Step 2: Verify RED**

```sh
cd /home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-lab
python3 -m unittest \
  tests.test_netbsd_scsi.NetBSDSCSI101ArtifactsTests -v
```

Expected: FAIL because the 10.1 fields and capability API do not exist.

- [ ] **Step 3: Implement a parallel contract**

Leave the completed `NetBSDSCSIArtifacts` and
`verify_netbsd_scsi_artifacts()` APIs untouched through Tasks 12--14.  Add:

```python
@dataclass(frozen=True)
class NetBSDSCSI101Artifacts:
    boot: Path
    boot_sha256: str
    ramdisk: Path
    ramdisk_sha256: str
    ramdisk_symbols: Path
    ramdisk_symbols_sha256: str
    iso: Path
    iso_sha256: str
```

Retain the descriptor-pinned open/hash/revalidation discipline from Task 1.
Add `verify_netbsd_scsi_101_artifacts(artifacts:
NetBSDSCSI101Artifacts) -> dict[str, dict[str, int | str]]` and
`verify_netbsd_101_ramdisk_capabilities(kernel: bytes, symbols: bytes) ->
frozenset[str]`; the latter consumes pinned, decompressed kernel and symbol
streams and returns the exact matched capability names for evidence.  Keep all
legacy 1.5 constants, MD5 checks, tests, and active production routing intact
until Task 15 performs the atomic cutover.

- [ ] **Step 4: Verify GREEN and commit**

```sh
python3 -m unittest \
  tests.test_netbsd_scsi.NetBSDSCSI101ArtifactsTests -v
git diff --check
git add nextcube_lab/netbsd_scsi.py tests/test_netbsd_scsi.py
git commit -m "netbsd: authenticate 10.1 RAMDISK media"
```

### Task 13: Stage the minimal RAMDISK rescue and CD-only installer

**Files:**

- Modify: `nextcube_lab/netbsd_scsi.py`
- Modify: `tests/test_netbsd_scsi.py`

- [ ] **Step 1: Write failing staging and 43,564-byte layout tests**

Add exact methods:

```python
test_stage_101_contains_only_boot_kernel_and_control_script
test_stage_101_pins_gunzip_and_records_derived_kernel
test_stage_101_rejects_gunzip_or_source_replacement
test_stage_101_rolls_back_partial_decompression
test_101_install_script_reads_all_sets_from_cd
test_101_install_script_never_reads_release_sets_from_nfs
test_101_install_script_creates_exact_disklabel_with_quoted_heredoc
test_101_install_script_uses_no_unavailable_checksum_applet
test_101_install_writes_all_readbacks_before_base_extraction
test_101_install_extracts_base_and_etc_before_target_cmp
test_101_install_uses_chroot_target_cmp_for_all_four_pairs
test_101_bootblocks_marker_follows_all_target_cmp_commands
test_101_boot_layout_has_85_sector_head_and_44_byte_tail
test_101_boot_tail_reads_use_bs_1_and_exact_disk_byte_offsets
test_101_boot_copies_end_at_sectors_150_and_278
```

Add `NetBSDSCSI101Staging` and
`stage_netbsd_scsi_101_install(root: Path, artifacts:
NetBSDSCSI101Artifacts, gunzip: Path) -> NetBSDSCSI101Staging` alongside,
without changing, the legacy staging API.  Tasks 13--14 test these parallel
10.1 APIs directly; production routing remains legacy until Task 15.  The
expected 10.1 transport tree is exactly `tftp/boot`,
`rootfs/netbsd-RAMDISK`, and `rootfs/netbsd-scsi-install.sh`; the latter is
control input, while every release set is read from CD.  The uncompressed
kernel is exactly 3,143,304 bytes.

- [ ] **Step 2: Verify RED**

```sh
python3 -m unittest \
  tests.test_netbsd_scsi.NetBSDSCSIStagingTests -v
```

Expected: FAIL because the parallel 10.1 staging API and boot math do not exist.

- [ ] **Step 3: Implement pinned decompression and the manual script**

Add an explicit absolute `gunzip`-program path to staging.  Open it with
`O_NOFOLLOW|O_CLOEXEC`, require a single-link regular executable, hash it,
execute only the pinned descriptor with exact arguments `-dc --` and the
pinned RAMDISK descriptor exposed through `/proc/self/fd`, and pass both file
descriptors explicitly.  Stream stdout into the transaction-owned
`rootfs/netbsd-RAMDISK`, then revalidate executable and input identities.
Record executable SHA-256, derived-kernel size/SHA-256, and capability results
in `NetBSDSCSI101Staging`.  The production command selects the real
`/usr/bin/gzip` binary for this `gunzip` role, not its shell wrapper.

Generate a `/bin/sh` script that mounts `/dev/cd0a`, uses `disklabel`, `newfs`,
and `mount_ffs` on `sd0`, and extracts only
`/mnt/cd/next68k/binary/sets/base.tgz`, `etc.tgz`, and
`kern-GENERIC.tgz`.  The RAMDISK has no usable `cksum`, `sha256`, or `sha512`
applet: do not generate a guest manifest verifier or claim guest rehashing.
Instead, explicitly rely on the host-authenticated immutable staged ISO and
require successful guest mount plus extraction of all three sets as CD data-
path evidence.  Write `fstab`, `rc.conf`, and the install marker, unmount, run
`fsck_ffs -fn`, emit the existing ordered markers, and halt.

The three-file staging tree has no `disklabel.proto`.  The generated script
must create `/tmp/disklabel.proto` with this exact quoted heredoc before
running `disklabel -R -r sd0 /tmp/disklabel.proto`:

```sh
cat > /tmp/disklabel.proto <<'NETBSD_SCSI_DISKLABEL'
type: SCSI
disk: QEMU SCSI
label: NetBSD101
flags:
bytes/sector: 512
sectors/track: 32
tracks/cylinder: 8
sectors/cylinder: 256
cylinders: 16384
total sectors: 4194304
rpm: 3600
interleave: 1
trackskew: 0
cylinderskew: 0
headswitch: 0
track-to-track seek: 0
drivedata: 0
8 partitions:
#        size    offset     fstype  [fsize bsize cpg]
  a:  4062912       320     4.2BSD    1024  8192    16
  b:   131072   4063232       swap
  c:  4194304         0     unused       0     0
NETBSD_SCSI_DISKLABEL
disklabel -R -r sd0 /tmp/disklabel.proto
```

Use 512-byte boot-copy units.  `43_564 = 85 * 512 + 44`; each synced copy
occupies 86 sectors.  Copy the authenticated CD boot file with exact commands
`dd if=/mnt/cd/next68k/installation/boot of=/dev/rsd0c bs=512 seek=64
conv=sync` and the same command with `seek=192`.  After `newfs`, mounting the
target at `/mnt`, and `mkdir -p /mnt/tmp`, create every comparison input before
extracting any set, using this exact order:

```sh
dd if=/mnt/cd/next68k/installation/boot of=/mnt/tmp/boot.expected.head bs=512 count=85
dd if=/mnt/cd/next68k/installation/boot of=/mnt/tmp/boot.expected.tail bs=1 skip=43520 count=44
dd if=/dev/rsd0c of=/mnt/tmp/boot.1.head bs=512 skip=64 count=85
dd if=/dev/rsd0c of=/mnt/tmp/boot.1.tail bs=1 skip=76288 count=44
dd if=/dev/rsd0c of=/mnt/tmp/boot.2.head bs=512 skip=192 count=85
dd if=/dev/rsd0c of=/mnt/tmp/boot.2.tail bs=1 skip=141824 count=44
```

The disk heads begin at absolute byte offsets 32,768 and 98,304; `skip=64`
and `skip=192` are interpreted in 512-byte input blocks.  The tail skips 76,288
and 141,824 are absolute bytes because those two commands use `bs=1`.

The RAMDISK has no `cmp`.  Next extract `base.tgz` and `etc.tgz` into `/mnt`,
then run exactly:

```sh
chroot /mnt /usr/bin/cmp /tmp/boot.expected.head /tmp/boot.1.head
chroot /mnt /usr/bin/cmp /tmp/boot.expected.tail /tmp/boot.1.tail
chroot /mnt /usr/bin/cmp /tmp/boot.expected.head /tmp/boot.2.head
chroot /mnt /usr/bin/cmp /tmp/boot.expected.tail /tmp/boot.2.tail
echo NETBSD_SCSI_BOOTBLOCKS_OK
```

Every successful `cmp` therefore comes from the extracted target base set;
the marker follows all four comparisons.  Extract `kern-GENERIC.tgz` only
after this gate.  Assert exclusive padded ending sectors 150 and 278 are below
`NETBSD_SCSI_FRONT_PORCH == 320`.

- [ ] **Step 4: Verify GREEN and commit**

```sh
python3 -m unittest \
  tests.test_netbsd_scsi.NetBSDSCSIStagingTests \
  tests.test_netbsd_scsi.NetBSDSCSIArgvTests -v
git diff --check
git add nextcube_lab/netbsd_scsi.py tests/test_netbsd_scsi.py
git commit -m "netbsd: stage 10.1 RAMDISK CD installer"
```

### Task 14: Make the install controller fail closed on md0a and devices

**Files:**

- Modify: `nextcube_lab/netbsd_scsi.py`
- Modify: `nextcube_lab/baseline/runner.py`
- Modify: `tests/test_netbsd_scsi.py`
- Modify: `tests/test_baseline_runner.py`

- [ ] **Step 1: Write failing controller tests**

Add exact tests:

```python
test_101_install_classifies_md0a_nextdma_esp_sd_and_cd
test_101_install_classifies_official_rom_load_tuple
test_101_install_classifies_official_loader_banner
test_101_install_rejects_legacy_rom_load_tuple
test_101_local_boot_classifies_official_rom_load_tuple
test_101_local_boot_classifies_official_loader_banner
test_101_local_boot_classifies_entry_as_loader_disk_after_banner
test_101_local_boot_rejects_entry_before_banner
test_101_local_boot_without_entry_never_records_loader_disk
test_101_local_boot_rejects_malformed_entry
test_101_local_boot_rejects_legacy_loader_path_debug_line
test_101_local_boot_rejects_legacy_loader_open_debug_line
test_101_install_requires_nextdma_scsi_channel_not_ethernet
test_101_install_early_shell_sends_no_return_or_command
test_101_install_md0a_without_all_devices_sends_nothing
test_101_install_rejects_xe_or_nfs_root
test_101_install_configures_exact_xe0_only_after_device_gate
test_101_install_mounts_control_nfs_read_only_after_ifconfig_prompt
test_101_install_runs_script_only_after_authenticated_mount_prompt
test_101_install_rejects_mount_prompt_before_ifconfig_prompt
```

Use a recording fake QMP and assert its `send_text` call list is empty for both
early-shell cases.  For the success case assert its network/control subsequence
is exactly ordered as `mkdir -p /mnt2`, then
`ifconfig xe0 10.0.2.15 netmask 255.255.255.0 up`, then
`mount_nfs -o ro 10.0.2.2:/ /mnt2`, then
`/mnt2/netbsd-scsi-install.sh`, with a newly observed shell prompt between
each command.  The amended install gates are exactly:

```python
("ramdisk-root", "devices", "cdrom", "label", "bootblocks",
 "filesystem", "complete")
```

- [ ] **Step 2: Verify RED**

```sh
python3 -m unittest \
  tests.test_netbsd_scsi.NetBSDSCSIVisibleLineTests \
  tests.test_baseline_runner.NetBSDSCSI101ControllerTests -v
```

Expected: FAIL because the parallel 10.1 classifiers/controller do not exist.

- [ ] **Step 3: Implement the strict RAMDISK sequence**

Add parallel `classify_netbsd_scsi_101_install_line()`,
`classify_netbsd_scsi_101_boot_line()`, and `NetBSDSCSI101Controller` APIs and
test them directly.  Do not select them from the production runner until Task
15.  The official 43,564-byte boot program's ROM load tuple is exactly
`T=41071`, `D=2456`, `B=13840`, producing this exact line:

```text
Loading boot at 0x4380000: 41071+2456+13840
```

Its matching loader banner is exactly:

```text
>> NetBSD/next68k BOOT [1.8 (Mon Dec 16 13:08:11 UTC 2024) #0]
```

Accept those values in both corrected install and local-boot classifiers; do
not let the legacy `24504+1576+25872` / Revision 1.2 oracle reject them.  Keep
the legacy classifier unchanged until Task 15, and fail closed on any other
tuple or banner.

The official 10.1 loader does not emit the legacy exact observations
`booting disk sd(0,0,0)netbsd.` or `open: sd(0,0,0)`.  In the parallel 10.1
local-boot classifier, classify those old debug strings as fatal.  After the
authenticated 1.8 banner, require exactly one line matching:

```python
re.fullmatch(r"entry 0x4001000 esym 0x[0-9a-fA-F]+", visible)
```

Classify it as `loader-entry` and let that observation, not a pathname/open
debug line, satisfy the `loader-disk` gate.  A missing entry cannot advance the
gate; an entry before the banner, a duplicate, or a malformed entry is fatal.
The following `root on sd0a` remains the independent kernel/local-root gate.

Send `ben() boot`, then `en()netbsd-RAMDISK`.  Recognize exact runtime
`root on md0a`, `esp0`, `scsibus0`, `sd0`, and `cd0`; the DMA observation must
pass `re.fullmatch(r"nextdma[0-9]+ at intio0.*: channel [0-9]+ \(scsi\)",
visible)` and must not be satisfied by `(enetx)` or `(enetr)`.  Reject NFS/xe
roots.  Do not answer the RAMDISK shell or terminal prompts until
`ramdisk-root` and `devices` are recorded.  At the authenticated shell, send
exactly `mkdir -p /mnt2`.  Only after its next shell prompt send exactly
`ifconfig xe0 10.0.2.15 netmask 255.255.255.0 up`.  Only after its next shell
prompt send exactly `mount_nfs -o ro 10.0.2.2:/ /mnt2`; only after the next
shell prompt invoke `/mnt2/netbsd-scsi-install.sh`.  Any missing, duplicate, or
out-of-order prompt/command is evidence-incomplete.  Keep secrets out of QMP
and preserve existing local-boot behavior.

- [ ] **Step 4: Verify GREEN and commit**

```sh
python3 -m unittest tests.test_netbsd_scsi \
  tests.test_baseline_runner.NetBSDSCSI101ControllerTests -v
git diff --check
git add nextcube_lab/netbsd_scsi.py nextcube_lab/baseline/runner.py \
  tests/test_netbsd_scsi.py tests/test_baseline_runner.py
git commit -m "baseline: gate NetBSD RAMDISK SCSI install"
```

### Task 15: Migrate runner, CLI, and campaign composition

**Files:**

- Modify: `nextcube_lab/baseline/model.py`
- Modify: `nextcube_lab/baseline/runner.py`
- Modify: `nextcube_lab/baseline_cli.py`
- Modify: `nextcube_lab/netbsd_scsi_campaign.py`
- Modify: `nextcube_lab/netbsd_scsi_campaign_cli.py`
- Modify: `tests/test_baseline_model.py`
- Modify: `tests/test_baseline_runner.py`
- Modify: `tests/test_netbsd_scsi_campaign.py`

- [ ] **Step 1: Write failing migration tests**

Add exact tests for required `--netbsd-ramdisk`,
`--netbsd-ramdisk-sha256`, `--netbsd-ramdisk-symbols`,
`--netbsd-ramdisk-symbols-sha256`, and `--gunzip`; rejection of every old
root/base/etc/kernel option; four-name artifact evidence; derived-kernel and
gunzip identities in the install bundle; no CD/NFS in either boot argv; and
aggregate verification failure when any new identity drifts.

Also add these exact atomic-cutover regressions:

```python
test_runner_selects_101_controller_and_staging_after_cutover
test_campaign_rejects_mixed_legacy_and_101_artifact_authority
test_cli_rejects_every_legacy_scsi_media_option_after_cutover
test_no_legacy_scsi_artifact_or_staging_api_remains_referenced
```

- [ ] **Step 2: Verify RED**

```sh
python3 -m unittest tests.test_baseline_model \
  tests.test_baseline_runner tests.test_netbsd_scsi_campaign -v
```

- [ ] **Step 3: Migrate the active contract without compatibility aliases**

At the start of this task, production still uses only the legacy APIs while
the 10.1 classes from Tasks 12--14 are parallel and test-only.  In one reviewed
commit, thread `NetBSDSCSI101Artifacts`, `NetBSDSCSI101Staging`, the 10.1
classifiers/controller, and the explicit gunzip path through `RunConfig`, both
CLIs, runner staging, raw evidence, coordinator, and verifier.  Update every
caller and test in the same change, then delete the legacy
`NetBSDSCSIArtifacts`, legacy verifier/stager/script/classifier/controller
paths and their six-input-only constants after `rg` proves no production
reference remains.  There must be no released mixed state and no compatibility
alias accepting a legacy artifact record.

Keep the three-phase ownership and atomic aggregate publication from Task 7.
The install argv retains NFS plus targets 0/3; boot argv remains target 0 plus
`-nic none`.  Do not search the filesystem or accept deprecated aliases.

- [ ] **Step 4: Verify GREEN and commit**

```sh
python3 -m unittest tests.test_baseline_model \
  tests.test_baseline_runner tests.test_netbsd_scsi_campaign -v
git diff --check
git add nextcube_lab/baseline/model.py nextcube_lab/baseline/runner.py \
  nextcube_lab/baseline_cli.py nextcube_lab/netbsd_scsi_campaign.py \
  nextcube_lab/netbsd_scsi_campaign_cli.py tests/test_baseline_model.py \
  tests/test_baseline_runner.py tests/test_netbsd_scsi_campaign.py
git commit -m "netbsd: migrate SCSI campaign to 10.1 RAMDISK"
```

### Task 16: Run focused and complete verification before the VM

**Files:** None expected

- [ ] **Step 1: Run all affected lab tests**

```sh
cd /home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-lab
python3 -m unittest tests.test_netbsd_scsi \
  tests.test_netbsd_scsi_campaign tests.test_baseline_model \
  tests.test_baseline_runner tests.test_baseline_bundle -v
python3 -m unittest discover -s tests -p 'test_*.py'
git diff --check
git status --short --branch
```

Expected: every test passes, no new skip, and the worktree is clean.

- [ ] **Step 2: Re-run the registered NeXT QEMU tests**

```sh
cd /home/blanham/projects/NeXT/lab/work/netbsd-scsi-task9-20260826/builds/20260826T134535Z-00ee3eb43982-884ee036
./tests/qtest/next-scsi-test
meson test next-scsi --print-errorlogs
```

Expected: 29/29 direct qtests and the Meson wrapper pass.

### Task 17: Run and independently verify the corrected visible campaign

**Files:** Generated external media/evidence only

- [ ] **Step 1: Extract the three ISO members into stable external paths**

Create `/mnt/e/disks/netbsd-10.1-next68k-official/` and extract exactly
`next68k/installation/boot`, `netbsd-RAMDISK.gz`, and
`netbsd-RAMDISK.symbols.gz` from the authenticated ISO.  Run `stat`,
`sha256sum`, and `sha512sum`; require the constants in this amendment before
launching.

- [ ] **Step 2: Build the final clean QEMU source identity**

```sh
cd /home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-lab
PYTHONPATH=. python3 -m nextcube_lab.baseline_cli build \
  --qemu-source /home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-qemu \
  --work-root /home/blanham/projects/NeXT/lab/work/netbsd-scsi-task9-101 \
  --jobs 4 \
  --summary /home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-lab/results/netbsd-scsi-task9-101-build.json \
  --force-slirp-fallback
```

Require a clean source and passing registered tests.

- [ ] **Step 3: Capture one lossless decorated-frame three-phase run**

Use a proven Xvfb window manager and explicitly unset Wayland:

```sh
env -u WAYLAND_DISPLAY GDK_BACKEND=x11 DISPLAY=:202 \
  python3 -m nextcube_lab.netbsd_scsi_campaign_cli run \
  --campaign /home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-lab/results/netbsd-scsi-101.json \
  --workspace /home/blanham/projects/NeXT \
  --qemu-source /home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-qemu \
  --build-metadata /home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-lab/results/netbsd-scsi-task9-101-build.json \
  --work-root /home/blanham/projects/NeXT/lab/work \
  --evidence-root /home/blanham/projects/NeXT/evidence/runs \
  --results-root /home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-lab/results \
  --disk-root /home/blanham/projects/NeXT/lab/work/netbsd-scsi-101-disk \
  --rom /home/blanham/projects/NeXT/assets/firmware/Rev_2.5_v66.BIN \
  --rom-sha256 1b753890b67095b73e104c939ddf62eca9e7d0aedde5108e3893b0ed9d8000a4 \
  --netbsd-boot /mnt/e/disks/netbsd-10.1-next68k-official/boot \
  --netbsd-boot-sha256 408927b96731f3ecdb4d58c4d042320a24a0c9b84d9e6089c547f517c6babd53 \
  --netbsd-ramdisk /mnt/e/disks/netbsd-10.1-next68k-official/netbsd-RAMDISK.gz \
  --netbsd-ramdisk-sha256 f04fc2329d2e9282b8e4c434d18b90d69be83322da948964c7cae9be97808aee \
  --netbsd-ramdisk-symbols /mnt/e/disks/netbsd-10.1-next68k-official/netbsd-RAMDISK.symbols.gz \
  --netbsd-ramdisk-symbols-sha256 884dd06236a4cf0519b914399547a9084e56dc045f9016b8f36849e8119335e5 \
  --netbsd-iso /mnt/e/disks/NetBSD-10.1-next68k.iso \
  --netbsd-iso-sha256 a0a74335c87dff3d4e2255ea70f9ddf14f02d07cad9f325a06fc080879578089 \
  --gunzip /usr/bin/gzip --display gtk --timeout 3600
```

Record FFV1/MKV to `/mnt/e/disks/netbsd-scsi-task9-20260826/` with the full
decorated QEMU window at a smooth source frame rate.  If a gate diverges,
preserve the first failure and return to systematic debugging; do not inject
unobserved lines.

- [ ] **Step 4: Independently verify with the same explicit identities**

Run the `verify` subcommand with the same options except `--disk-root`.
Require three independently verified raw bundles, exact final disk identity,
and one atomic passing aggregate.

#### Task 17 runtime-correction note (2026-08-26)

This dated note supersedes every earlier NetBSD 10.1 runtime command and oracle
statement in Tasks 5, 6, 14, and 17, including the Task 17 retry procedure;
the earlier text remains an historical record and must not be combined with
this corrected contract.  The fail-closed r2 run is preserved at
`/home/blanham/projects/NeXT/lab/work/runs/20260826T221806Z-nbsd-scsi-8058c823baa7-8173b37e`
with `manifest.sha256` SHA-256
`1fae535d921460b3294aef995c5bb133d1672f56f5cf31b49d2d0ea6f9d1b2d9`.
Its decorated capture is
`/mnt/e/disks/netbsd-scsi-task9-20260826/netbsd-scsi-101-3phase-r2-ffv1.mkv`
(181,903,368 bytes, 326.2 seconds, SHA-256
`a6a5e60d71e3ff0bc28a2799f33e253571359a3b42353b46ad602ad0ae50f97d`).

- [ ] **Correction step 1: Add the exact failing regressions**

Add these tests to `tests/test_netbsd_scsi.py` and
`tests/test_baseline_runner.py`, placing pure classifier assertions in the
former and state/command assertions in the latter:

```text
test_101_install_accepts_observed_release_build_2
test_101_install_rejects_assumed_build_0
test_101_install_advances_without_scsi_rom_tuple
test_101_install_requires_exact_rom_network_path_before_banner
test_101_install_progress_lines_do_not_advance_state
test_101_install_rejects_numeric_disk_load_tuple
test_101_local_boot_sends_explicit_netbsd_filename
test_101_local_boot_requires_exact_rom_disk_path
test_101_local_boot_keeps_authenticated_scsi_tuple
test_101_local_boot_accepts_exact_compound_tuple_banner
test_101_local_boot_rejects_empty_label_bootfile_failure
test_101_local_boot_accepts_adjacent_exact_tuple_and_banner
test_101_local_boot_rejects_intervening_nonadvancing_observation
test_101_local_boot_rejects_partial_wrapped_or_whitespace_compound_tuple_banner
```

The assertions use only these exact corrected observations:

```text
boot en(0,0,0)boot
boot sd(0,0,0)netbsd
booting SCSI target 0, lun 0
Loading boot at 0x4380000: 41071+2456+13840
>> NetBSD/next68k BOOT [1.8 (Mon Dec 16 13:08:11 UTC 2024) #2]
boot: <one ASCII space>
```

INSTALL must advance from its exact network path directly to the build-`#2`
banner and loader prompt while ignoring BOOTP/progress lines.  Any numeric
load tuple is fatal in INSTALL.  LOCAL must send `bsd() netbsd`, require the
exact disk path, target, numeric tuple, and build-`#2` banner, and reject an
empty-label path.  Assert that build `#0`, wrong banners, wrong paths/targets,
and unexpected tuples cannot advance.

- [ ] **Correction step 2: Verify RED**

```sh
cd /home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-lab
python3 -m unittest tests.test_netbsd_scsi \
  tests.test_baseline_runner.NetBSDSCSI101ControllerTests -v
```

Expected: the named regressions fail against the assumed build-`#0`, INSTALL
tuple gate, implicit local filename, and line-only tuple/banner handling.

- [ ] **Correction step 3: Implement the minimal classifier/controller fix**

In `nextcube_lab/netbsd_scsi.py`, replace the runtime banner oracle with exact
build `#2`; add exact INSTALL network-path classification; remove only the
INSTALL numeric-tuple success state; and classify every INSTALL numeric tuple
and every wrong banner as fatal.  BOOTP and progress text returns no
classification.  Require the build-`#2` banner followed by `boot: ` before
sending `en()netbsd-RAMDISK`.

For LOCAL, keep the exact target and numeric tuple, change the exact ROM path
to `boot sd(0,0,0)netbsd`, and change the owned ROM command in
`nextcube_lab/baseline/runner.py` and its campaign caller to `bsd() netbsd`.
Support this exact compound token as atomic ordered `rom-load` then
`loader-banner` observations:

```text
Loading boot at 0x4380000: 41071+2456+13840>> NetBSD/next68k BOOT [1.8 (Mon Dec 16 13:08:11 UTC 2024) #2]
```

Also accept the two exact full fragments when PTY or operator segmentation
presents them separately, but enter a strict pending-banner state after the
full tuple: the very next consumed observation must be the exact full
build-`#2` banner.  Any intervening observation is fatal even when it would
normally be ignored or classified as non-advancing.  Display-row wrapping
does not split the logical observation and cannot authorize substrings,
inserted newlines, or whitespace changes.  Reject partial or altered
compounds, unexpected numeric tuples, wrong banners, and empty-label bootfile
failures.

- [ ] **Correction step 4: Verify GREEN and commit the lab fix**

```sh
python3 -m unittest tests.test_netbsd_scsi \
  tests.test_baseline_runner.NetBSDSCSI101ControllerTests -v
python3 -m unittest discover -s tests -p 'test_*.py'
git diff --check
git status --short --branch
git add nextcube_lab/netbsd_scsi.py nextcube_lab/baseline/runner.py \
  nextcube_lab/netbsd_scsi_campaign.py tests/test_netbsd_scsi.py \
  tests/test_baseline_runner.py tests/test_netbsd_scsi_campaign.py
git commit -m "netbsd: correct 10.1 runtime boot oracles"
```

Expected: focused and full suites pass with no new skip and the lab worktree
is clean.

- [ ] **Correction step 5: Retry through PTY/operator observation**

Repeat Task 17 Steps 3--4 with a new disk root, result path, and FFV1 filename;
do not reuse or overwrite r2.  Keep the decorated QEMU window and operator
input visible.  At each PTY observation prompt, enter only text actually seen
in the QEMU frame: for INSTALL, `NeXT>`, the exact network path, the exact
build-`#2` banner, and `boot: `; for both LOCAL phases, `NeXT>`, the exact disk
path, target, exact tuple/banner compound (or the two exact PTY fragments),
then the existing loader-entry, `sd0a`, shell, and marker evidence.  Do not
enter BOOTP/progress chatter merely to move state, do not synthesize a newline
between a compound tuple/banner, and do not split a visually wrapped display
row.  When the PTY/operator interface exposes the tuple and banner separately,
enter the exact full tuple followed immediately by the exact full banner; no
other observation may be entered between them.  Do not inject any unobserved
line.  Stop at the first mismatch, preserve that run and capture, and return
to diagnosis.  Only three complete raw bundles plus independent verification
may publish the aggregate.

#### Task 17 second runtime-correction note (2026-08-26)

This note supersedes the earlier 10.1 device oracle and the next campaign
retry only; the r2 boot-command/tuple correction remains normative.  Task 17
r3 is **INVALID operator evidence** because the operator fed
`nextdma0 at intio0 addr 0x20000140: channel 0 (scsi)` instead of the exact
visible `nextdma0 at intio0 addr 0x2000010: channel 0 (scsi)`.  Preserve, but
never promote, its diagnostic frame
`/home/blanham/projects/NeXT/lab/work/netbsd-scsi-task17-r3-build/operator-logs/r3-kernel-device-full-sequence.png`
(48,699 bytes, SHA-256
`0fe490a96781035d0c10a0e96074219f1b26a33bbe56dd7e544d483c12fa240a`)
and capture
`/mnt/e/disks/netbsd-scsi-task9-20260826/netbsd-scsi-101-3phase-r3-ffv1.mkv`
(909,053,732 bytes, 539.133 seconds, SHA-256
`7a58c87502cca49b1f5f0ddd506b3fceca2bebbc27f92afa206ae2a2a0a2df41`).

With timestamp presentation prefixes omitted, r3 visibly emitted this exact
device sequence:

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

The corrected success oracle is exact `esp0 at intio0 addr 0x2114000`, exact
SCSI `nextdma0` at `0x2000010`, exact ESP200 identity, exact `scsibus0`, exact
`esp0: using DMA channel nextdma0`, then target-0 `sd0` and target-3 `cd0`,
before `root on md0a`.  The two Ethernet DMA lines are authentic ignored
attachments and must not advance or reorder the SCSI gate.  Any zero-capacity
or zero-sector-size disk line, target-3 `sd1`, or root transition without
target-3 `cd0` is fatal.

- [ ] **R3 correction step 1: Add the exact lab RED regressions**

Add these exact tests to `tests/test_netbsd_scsi.py` for pure classification
and `tests/test_baseline_runner.py` for ordered controller state:

```text
test_101_install_classifies_exact_esp_intio_dma_sequence
test_101_install_ignores_enet_dma_lines_without_reordering_scsi
test_101_install_rejects_wrong_scsi_nextdma_address
test_101_install_rejects_nextdma_before_esp_intio
test_101_install_requires_esp200_before_scsibus
test_101_install_requires_using_dma_before_sd0
test_101_install_rejects_zero_capacity_sd0
test_101_install_rejects_target3_sd1
test_101_install_requires_target3_cd0_before_md0a
```

Use the exact r3 lines above for the negative zero-capacity and `sd1` cases.
For the successful fixture, replace only the broken device results with:

```text
sd0 at scsibus0 target 0 lun 0: <QEMU, QEMU HARDDISK, 2.5+> disk fixed
cd0 at scsibus0 target 3 lun 0: <QEMU, QEMU CD-ROM, 2.5+> cdrom removable
root on md0a dumps on md0b
```

The classifier must match the full exact intio/nextdma/ESP200/scsibus/DMA-use
lines.  It may match the stable target/lun/vendor/device-type fields of the
successful `sd0` and `cd0` attachments, but must not accept a blank inquiry
identity, zero capacity, a different target, or `sd1` as an optical alias.

For r4 the operator must feed the complete visible timestamped kernel line,
not the timestamp-free diagnostic transcription above.  Production and test
code may normalize exactly one prefix matching
`\A\[\s*[0-9]+\.[0-9]{6}\] ` before exact classification; it must strip
nothing else.  A line beginning with `[` but carrying a malformed timestamp
prefix is fatal, including the wrong fractional width, missing single space
after `]`, an extra space inside the closing bracket, a sign, or extra digits.
Preserve the raw fed input byte-for-byte in evidence beside the normalized
line; normalization must never rewrite the retained raw observation.

Add these exact RED tests as part of this step:

```text
test_101_install_accepts_exact_timestamped_device_sequence
test_101_install_strips_only_canonical_kernel_timestamp_prefix
test_101_install_rejects_malformed_kernel_timestamp_prefixes
test_101_install_preserves_raw_timestamped_observation_evidence
test_101_install_requires_verified_screenshot_hash_per_device_line
```

Use this literal successful input fixture (followed by the existing prompt and
shell fixtures); assertions compare both every raw string and its normalized
classification:

```text
[     12.345678] esp0 at intio0 addr 0x2114000
[     12.345679] nextdma0 at intio0 addr 0x2000010: channel 0 (scsi)
[     12.345680] nextdma1 at intio0 addr 0x2000110: channel 1 (enetx)
[     12.345681] nextdma2 at intio0 addr 0x2000150: channel 2 (enetr)
[     12.345682] esp0: ESP200, 20MHz, SCSI ID 7
[     12.345683] scsibus0 at esp0: 8 targets, 8 luns per target
[     12.345684] esp0: using DMA channel nextdma0
[     12.345685] sd0 at scsibus0 target 0 lun 0: <QEMU, QEMU HARDDISK, 2.5+> disk fixed
[     12.345686] cd0 at scsibus0 target 3 lun 0: <QEMU, QEMU CD-ROM, 2.5+> cdrom removable
[     12.345687] root on md0a dumps on md0b
```

The malformed-prefix table must include `[12.34567] `,
`[12.345678]` without the following space, `[12.345678 ] `,
`[+12.345678] `, and `[12.3456789] `, each prepended to an otherwise exact
required device line and each asserted fatal.

- [ ] **R3 correction step 2: Verify the lab RED phase**

```sh
cd /home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-lab
python3 -m unittest \
  tests.test_netbsd_scsi.NetBSDSCSIVisibleLineTests \
  tests.test_baseline_runner.NetBSDSCSI101ControllerTests -v
```

Expected: the named tests fail because the current oracle expects `esp0 at
nextdma...`, omits the ESP200 and DMA-use transitions, accepts the wrong
`nextdma0` address, and has no fatal zero-capacity/target-3-`sd1` handling.

- [ ] **R3 correction step 3: Implement only the corrected device oracle**

In `nextcube_lab/netbsd_scsi.py`, replace the old device sequence with these
ordered classifications:

```python
(
    "device-esp-intio",
    "device-nextdma-scsi",
    "device-esp200",
    "device-scsibus0",
    "device-esp-using-dma",
    "device-sd0",
    "device-cd0",
)
```

Require the exact five fixed hardware lines recorded above.  Classify exact
`nextdma1` `(enetx)` and `nextdma2` `(enetr)` lines as non-advancing and allow
them only between the SCSI `nextdma0` and ESP200 observations.  Preserve the
independent `ramdisk-root` gate after all seven device observations.  Return
fatal for the wrong `nextdma0` address, any `unsupported sector size: 0x0`,
any geometry ending `x 1 sectors`, any `sd1 at scsibus0 target 3`, and any
`root on md0a` before target-3 `cd0`.  Do not infer `cd0` from QEMU argv.

Update `NetBSDSCSI101Controller` in
`nextcube_lab/baseline/runner.py` to enforce that exact order.  Preserve the
existing boot-command, banner, prompt, shell, install-marker, and fail-closed
behavior.

Implement the canonical-prefix normalization once in
`nextcube_lab/netbsd_scsi.py` and use it from both classifier and controller;
do not duplicate a more permissive controller regex.  Extend the operator
observation record in `nextcube_lab/baseline/runner.py` to keep raw input,
normalized input, screenshot path, screenshot SHA-256, and a positive
transcription-verification flag.  Before submitting each of the seven required
device observations, the operator must capture a unique QEMU-frame PNG,
transcribe the entire visible timestamped line, verify that transcription
against the frame character-for-character, hash the PNG, and record all five
fields.  A missing image, hash mismatch, reused image, or unverified
transcription is fatal.  This pre-submit record prevents an expected address,
identity, or device type from being inferred from argv or plan prose.

- [ ] **R3 correction step 4: Verify GREEN and commit the lab oracle**

```sh
python3 -m unittest \
  tests.test_netbsd_scsi.NetBSDSCSIVisibleLineTests \
  tests.test_baseline_runner.NetBSDSCSI101ControllerTests -v
python3 -m unittest discover -s tests -p 'test_*.py'
git diff --check
git add nextcube_lab/netbsd_scsi.py nextcube_lab/baseline/runner.py \
  tests/test_netbsd_scsi.py tests/test_baseline_runner.py
git commit -m "netbsd: correct 10.1 SCSI device oracle"
```

Expected: focused and complete lab suites pass with no new skip.  Obtain a
fresh specification-compliance review followed by a fresh code-quality review;
resolve every blocker test-first before building.

- [ ] **R3 correction step 5: Build the reviewed QEMU fix from clean source**

After QEMU-plan Task 7 passes its qtests and both reviews, run:

```sh
cd /home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-lab
PYTHONPATH=. python3 -m nextcube_lab.baseline_cli build \
  --qemu-source /home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-qemu \
  --work-root /home/blanham/projects/NeXT/lab/work/netbsd-scsi-task17-r4-build \
  --jobs 4 \
  --summary /home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-lab/results/netbsd-scsi-task17-r4-build.json \
  --force-slirp-fallback
```

Require clean source, the new QEMU commit identity, the complete registered
NeXT required set, all nine new NetBSD-order/owner qtests, and the preserved
disabled-NeXT-DMA gate.  Do not reuse the r3
binary, disk root, run directory, campaign result, or recorder output.

- [ ] **R3 correction step 6: Run and independently verify r4**

Start a fresh decorated Xvfb/xfwm session and FFV1 recorder, explicitly unset
Wayland, and use a new disk/result/capture identity.  Run:

```sh
env -u WAYLAND_DISPLAY GDK_BACKEND=x11 DISPLAY=:205 \
  python3 -m nextcube_lab.netbsd_scsi_campaign_cli run \
  --campaign /home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-lab/results/netbsd-scsi-101-r4.json \
  --workspace /home/blanham/projects/NeXT \
  --qemu-source /home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-qemu \
  --build-metadata /home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-lab/results/netbsd-scsi-task17-r4-build.json \
  --work-root /home/blanham/projects/NeXT/lab/work \
  --evidence-root /home/blanham/projects/NeXT/evidence/runs \
  --results-root /home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-lab/results \
  --disk-root /home/blanham/projects/NeXT/lab/work/netbsd-scsi-101-disk-r4 \
  --rom /home/blanham/projects/NeXT/assets/firmware/Rev_2.5_v66.BIN \
  --rom-sha256 1b753890b67095b73e104c939ddf62eca9e7d0aedde5108e3893b0ed9d8000a4 \
  --netbsd-boot /mnt/e/disks/netbsd-10.1-next68k-official/boot \
  --netbsd-boot-sha256 408927b96731f3ecdb4d58c4d042320a24a0c9b84d9e6089c547f517c6babd53 \
  --netbsd-ramdisk /mnt/e/disks/netbsd-10.1-next68k-official/netbsd-RAMDISK.gz \
  --netbsd-ramdisk-sha256 f04fc2329d2e9282b8e4c434d18b90d69be83322da948964c7cae9be97808aee \
  --netbsd-ramdisk-symbols /mnt/e/disks/netbsd-10.1-next68k-official/netbsd-RAMDISK.symbols.gz \
  --netbsd-ramdisk-symbols-sha256 884dd06236a4cf0519b914399547a9084e56dc045f9016b8f36849e8119335e5 \
  --netbsd-iso /mnt/e/disks/NetBSD-10.1-next68k.iso \
  --netbsd-iso-sha256 a0a74335c87dff3d4e2255ea70f9ddf14f02d07cad9f325a06fc080879578089 \
  --gunzip /usr/bin/gzip --display gtk --timeout 3600
```

At each PTY prompt feed only an exact full logical line visible in the frame.
Require target-0 nonzero `sd0`, target-3 `cd0`, `root on md0a`, all install
markers, and both network-independent local boots.  Stop and preserve the
first mismatch; never repair a fed observation after the fact.  Finalize the
new recorder as
`/mnt/e/disks/netbsd-scsi-task9-20260826/netbsd-scsi-101-3phase-r4-ffv1.mkv`.
Run the `verify` subcommand with the same artifact/build identities and require
three valid raw bundles plus one atomic passing aggregate before Task 18.

### Task 18: Publish corrected docs and GIF, then finish verification

**Files:**

- Modify: `README.md`
- Modify: `QEMU-NeXT-README.md`
- Add: `docs/boot/netbsd-scsi.gif`
- Modify: `tests/test_boot_gif.py`
- Modify: `tests/test_netbsd_scsi_campaign.py`

- [ ] **Step 1: Write failing documentation tests**

Require both READMEs to link the GIF and state: official 10.1 RAMDISK,
`md0a` rescue root, CD-only manual install, `sd0a` local root, targets 0/3,
the exact ISO SHA-512, no floppy/sysinst/firmware-CD boot, upstream SCSI still
unsupported, and the exact run/verify commands.  Require the old 1.5 failure
to be labelled diagnostic rather than passing evidence.

- [ ] **Step 2: Encode the final GIF**

Use the established 15 fps two-pass palette workflow.  Speed up static ROM,
kernel-load, and set-extraction spans; retain device discovery, `root on md0a`,
CD install completion, ROM disk load, and `root on sd0a` legibly; hold the final
persistent shell for exactly three seconds.  Preserve the decorated QEMU frame,
keep the GIF below 20 MiB, and run its standalone verifier.

- [ ] **Step 3: Verify and commit publication**

```sh
python3 -m unittest tests.test_boot_gif \
  tests.test_netbsd_scsi_campaign -v
python3 -m unittest discover -s tests -p 'test_*.py'
git diff --check
git add README.md QEMU-NeXT-README.md docs/boot/netbsd-scsi.gif \
  tests/test_boot_gif.py tests/test_netbsd_scsi_campaign.py
git commit -m "docs: publish NetBSD 10.1 SCSI install evidence"
```

Then independently reverify the aggregate, inspect both worktrees for hygiene,
run the required spec and quality reviews, and only then use the finishing-
branch workflow to merge and push.
