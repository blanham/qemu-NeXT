# NetBSD/next68k SCSI QEMU Support Implementation Plan

> **Historical record:** Do not execute this document linearly.  Its final
> dated amendments describe the old feature endpoint; the current execution
> authority is the lab plan
> `2026-09-04-netbsd-scsi-current-integration.md`.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Expose ordinary SCSI CD-ROM media on NeXT machines and prove that the NeXT ESP/DMA path safely supports the disk and optical transfer shapes required by unmodified NetBSD/next68k.

**Architecture:** Keep all changes in generic QEMU machine/controller behavior. First enable the standard CD-ROM command-line path and expand the existing NeXT SCSI qtest fixture to carry a disk and a patterned read-only optical image on separate targets. Then exercise inquiry, capacity, data, chaining, status, reset, and target-selection behavior before running the unmodified guest acceptance supplied by the companion lab plan.

**Tech Stack:** QEMU C/QOM machine configuration, QEMU ESP and SCSI devices, NeXT DMA MMIO, libqtest, Meson/Ninja.

---

## Repositories and ordering

Work only in:

```text
/home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-qemu
```

The branch is `feature/netbsd-scsi-install`, based on QEMU commit
`34bf708d2333e78da256cce4dfdd009643b7ff73`.  The configured build directory
is `build/`.  The companion lab worktree is:

```text
/home/blanham/projects/NeXT/lab/.worktrees/netbsd-scsi-lab
```

Tasks 1–3 produce a complete QEMU unit-tested change.  Task 4 is performed
after the lab plan supplies the real-guest launch.  If Task 4 exposes a QEMU
fault not represented below, stop speculative editing, invoke
`superpowers:systematic-debugging`, capture the exact guest transaction, add a
failing qtest for that transaction, and amend this plan with the evidenced
minimal fix before continuing.

### Task 1: Characterize standard NeXT SCSI CD-ROM attachment

**Files:**

- Modify: `tests/qtest/next-cube-scsi-test.c:82-185`

- [ ] **Step 1: Add a machine-start characterization test for `-cdrom`**

Extend the fixture with an optical-image size and a media object:

```c
#define NEXT_CD_SECTOR_SIZE 2048
#define NEXT_CD_SECTORS     4
#define NEXT_CD_SIZE        (NEXT_CD_SECTOR_SIZE * NEXT_CD_SECTORS)

typedef struct TestMedia {
    int rom_fd;
    int disk_fd;
    int cd_fd;
    char *rom_path;
    char *disk_path;
    char *cd_path;
} TestMedia;
```

Add `next_cube_scsi_cdrom_start()` which creates a zeroed 128 KiB ROM and an
8192-byte regular file, quotes both paths with `g_shell_quote()`, and calls:

```c
qtest_initf("-machine next-cube -bios %s -cdrom %s",
            quoted_rom_path, quoted_cd_path);
```

Register `test_scsi_cdrom_command_line` at
`/next-cube/scsi/cdrom-command-line`; success is simply reaching and quitting
the qtest machine.  Ensure cleanup closes descriptors and unlinks both files.

- [ ] **Step 2: Run the new test and confirm explicit CD-ROM attachment works**

Run:

```sh
ninja -C build tests/qtest/next-cube-scsi-test
output="$(QTEST_QEMU_BINARY=build/qemu-system-m68k \
  build/tests/qtest/next-cube-scsi-test \
  -p /next-cube/scsi/cdrom-command-line)"
printf '%s\n' "$output"
test "$(printf '%s\n' "$output" | rg -Fxc \
  'ok 1 /next-cube/scsi/cdrom-command-line')" -eq 1
test "$(printf '%s\n' "$output" | rg -c \
  '^(ok|not ok) [0-9]+ ')" -eq 1
```

Expected: PASS.  `MachineClass.no_cdrom` controls creation of the default
empty CD-ROM drive; it does not reject an explicit `-cdrom` option.  This is
an existing behavior characterization, so there is deliberately no production
change and no manufactured RED phase in this task.

- [ ] **Step 3: Keep the machine's default-drive policy unchanged**

Retain:

```c
mc->no_cdrom = true;
```

Changing this flag would add an empty default optical drive rather than enable
the explicit attachment already accepted by QEMU.  Do not create a
NeXT-specific CD device or alter generic SCSI command data.

- [ ] **Step 4: Run the full NeXT SCSI qtest binary**

Run:

```sh
QTEST_QEMU_BINARY=build/qemu-system-m68k \
  build/tests/qtest/next-cube-scsi-test
```

Expected: all 24 tests PASS.

- [ ] **Step 5: Commit the focused characterization test**

```sh
git add tests/qtest/next-cube-scsi-test.c
git commit -m "tests/qtest: cover NeXT CD-ROM command line"
```

### Task 2: Prove disk and optical devices coexist on separate targets

**Files:**

- Modify: `tests/qtest/next-cube-scsi-test.c:82-260`

- [ ] **Step 1: Add a failing two-device fixture**

Replace the one-purpose disk fixture only where needed with
`next_cube_scsi_media_start(TestMedia *media)`.  It creates a 128 KiB ROM, a
512 KiB zeroed disk, and a four-sector CD file.  Before closing the CD file it
writes all 8192 bytes, with byte `i` equal to `(i ^ 0xa5) & 0xff`.  Quote all
three paths and return this exact launch:

```c
return qtest_initf(
    "-machine next-cube -bios %s "
    "-drive file=%s,if=scsi,index=0,format=raw "
    "-drive file=%s,if=scsi,index=3,format=raw,media=cdrom,readonly=on",
    quoted_rom_path, quoted_disk_path, quoted_cd_path);
```

Generalize command submission to take a target:

```c
static void issue_inquiry_dma(QTestState *qts, uint8_t target,
                              uint8_t length);
```

and write `target` to `NEXT_ESP_BUSID` rather than the literal zero.

Add `/next-cube/scsi/disk-and-cd-inquiry`.  Consume unit attention on targets
0 and 3, issue a 36-byte INQUIRY to each, and assert the peripheral device
type byte is `0x00` for target 0 and `0x05` for target 3.  Also select target 2
and assert the existing selection-timeout behavior remains intact.

- [ ] **Step 2: Run the coexistence test and verify the test fails before all helper changes are complete**

Run:

```sh
ninja -C build tests/qtest/next-cube-scsi-test
output="$(QTEST_QEMU_BINARY=build/qemu-system-m68k \
  build/tests/qtest/next-cube-scsi-test \
  -p /next-cube/scsi/disk-and-cd-inquiry)"
printf '%s\n' "$output"
test "$(printf '%s\n' "$output" | rg -c \
  '^(ok|not ok) 1 /next-cube/scsi/disk-and-cd-inquiry$')" -eq 1
test "$(printf '%s\n' "$output" | rg -c \
  '^(ok|not ok) [0-9]+ ')" -eq 1
```

Expected: FAIL until target-aware inquiry and media setup are implemented.

- [ ] **Step 3: Implement the fixture and target-aware command helpers**

Preserve the existing `TestDisk` helpers for tests which need direct host
inspection of disk writes.  Add `cleanup_test_media()` for the three-file
fixture, use `qtest_add_abrt_handler()` and `g_test_queue_destroy()`, and set
all file descriptors to `-1` immediately after closing them.

Add a reusable data-in completion helper:

```c
static void read_scsi_dma(QTestState *qts, uint8_t target,
                          const uint8_t *cdb, size_t cdb_len,
                          hwaddr guest_buffer, size_t transfer_len)
{
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, guest_buffer);
    qtest_writel(qts, NEXT_DMA_LIMIT, guest_buffer + transfer_len);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);
    qtest_writeb(qts, NEXT_ESP_BUSID, target);
    for (size_t i = 0; i < cdb_len; i++) {
        qtest_writeb(qts, NEXT_ESP_FIFO, cdb[i]);
    }
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_SEL);
    qtest_readb(qts, NEXT_ESP_INTR);
    qtest_writeb(qts, NEXT_ESP_TCLO, transfer_len & 0xff);
    qtest_writeb(qts, NEXT_ESP_TCMID, (transfer_len >> 8) & 0xff);
    qtest_writeb(qts, NEXT_ESP_TCHI, (transfer_len >> 16) & 0xff);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_TI_DMA);
    finish_scsi_command(qts);
}
```

Use a checked cast or assertion before writing the 24-bit transfer count.

- [ ] **Step 4: Run the new test and the complete existing SCSI executable**

Run:

```sh
QTEST_QEMU_BINARY=build/qemu-system-m68k \
  build/tests/qtest/next-cube-scsi-test
```

Expected: all prior 23 tests plus the two new tests PASS.

- [ ] **Step 5: Commit the multi-target fixture**

```sh
git add tests/qtest/next-cube-scsi-test.c
git commit -m "tests/qtest: cover NeXT disk and CD targets"
```

### Task 3: Cover optical capacity, reads, DMA chaining, and reset

**Files:**

- Modify: `tests/qtest/next-cube-scsi-test.c`

- [ ] **Step 1: Add failing READ CAPACITY and READ(10) tests**

Add these CDBs:

```c
static const uint8_t read_capacity_10[10] = { 0x25 };
static const uint8_t read_10_cd_sector_1[10] = {
    0x28, 0, 0, 0, 0, 1, 0, 0, 1, 0,
};
```

Register:

```text
/next-cube/scsi/cd-read-capacity
/next-cube/scsi/cd-read-10
/next-cube/scsi/cd-read-10-chain
/next-cube/scsi/cd-reset-after-staged-inquiry
```

`cd-read-capacity` reads eight bytes and asserts a last LBA of 3 and a block
length of 2048, both big-endian.  `cd-read-10` reads sector 1 and compares all
2048 bytes with the fixture pattern at file offset 2048.

`cd-read-10-chain` programs two 1024-byte DMA descriptors using `NEXT`,
`LIMIT`, `START`, `STOP`, `DMA_SETSUPDATE`, and `DMA_DEV2M`; it asserts both
halves, final `NEXT`, `DMA_COMPLETE`, and the expected ESP status/message
completion.

`cd-reset-after-staged-inquiry` performs a 36-byte target-3 inquiry that leaves a
four-byte staged tail after two 16-byte DMA beats, asserts ESP chip reset and
NeXT DMA reset, then asserts the NeXT SCSI CSR reset which owns the staging
FIFO before reading a full CD sector and comparing all data.  DMA reset alone
must preserve a staged continuation because Mach uses that sequence while
reprogramming the final transfer window; the SCSI CSR reset is what discards
old staged state before the next optical command.

- [ ] **Step 2: Build and run each new test independently**

Run the build once, then run all four paths with `-p` as in earlier tasks.

Expected: each test must initially fail for a test-observable reason before
the helper or assertion implementation is completed; none may be marked green
without executing the SCSI command through NeXT DMA.

- [ ] **Step 3: Implement only fixture/helper corrections required by the tests**

Do not modify `hw/scsi/scsi-disk.c`: generic `scsi-cd` already owns INQUIRY,
READ CAPACITY, and READ(10).  Modify `hw/m68k/next-cube.c` or `hw/scsi/esp.c`
only if a new test demonstrates a NeXT/ESP state defect; in that event, keep
the failing test in the same commit and document the exact state transition in
a source comment.

- [ ] **Step 4: Run the focused and registered qtest suites**

Run:

```sh
QTEST_QEMU_BINARY=build/qemu-system-m68k \
  build/tests/qtest/next-cube-scsi-test
meson test -C build --print-errorlogs qtest-m68k/next-cube-scsi-test
```

Expected: PASS with no regression in the original 23 tests.

- [ ] **Step 5: Check formatting and commit**

Run:

```sh
git clang-format --diff HEAD~1
git diff --check
```

Review any formatting diff, apply it if needed, rerun the tests, then commit:

```sh
git add tests/qtest/next-cube-scsi-test.c hw/m68k/next-cube.c hw/scsi/esp.c
git commit -m "tests/qtest: exercise NeXT optical DMA"
```

Only add hardware files that actually changed.

### Task 4: Run unmodified NetBSD guest characterization

**Files:**

- No planned source changes
- Evidence input: companion lab `netbsd-scsi-install` campaign

- [ ] **Step 1: Build a clean QEMU identity for the campaign**

Run the lab build command from its README using this registered source
worktree and store build metadata below the lab worktree's `work/` directory.

- [ ] **Step 2: Run the install-stage guest with disk target 0 and CD target 3**

Use the exact command generated by the companion plan.  Require visible guest
evidence for `esp0`, `scsibus0`, `sd0`, and `cd0`, plus successful disk writes
and CD reads.

Expected: all install-stage gates pass with the unmodified NetBSD 1.5 kernel.

- [ ] **Step 3: Handle any divergence by evidence, not guesses**

If the guest fails, preserve the first trace and screenshot, identify the
first incorrect ESP/DMA transition, and invoke `superpowers:systematic-debugging`.
Before changing QEMU, add one minimal failing qtest which reproduces that exact
transition.  Update this plan with the concrete file, assertion, minimal fix,
and test command.  Do not weaken the guest gate or patch NetBSD.

- [ ] **Step 4: Run two network-independent SCSI boots**

Require ROM `bsd()`, standalone-loader SCSI `/netbsd`, `root on sd0a`, a local
shell, and marker persistence across the second boot.

- [ ] **Step 5: Commit any evidenced controller fix separately**

The commit message names the controller behavior, never NetBSD policy.  Rerun
Tasks 1–3 after the fix.

### Task 5: Update QEMU documentation after acceptance

**Files:**

- Modify: `QEMU-NeXT-README.md`
- Modify: `docs/system/target-m68k.rst`
- Add: `docs/boot/netbsd-scsi.gif`

- [ ] **Step 1: Add the proven local-disk and CD-ROM instructions**

Document the exact authenticated media, target IDs, install-stage launch,
`bsd()` disk boot, local-root evidence, CD read proof, and unsupported areas.
State explicitly that no official next68k install floppy exists and that the
10.1 ISO is not a bootable next68k installer.

- [ ] **Step 2: Update the hardware table only to the proven level**

Change ESP SCSI notes to include the tested NetBSD disk path.  Add SCSI CD-ROM
as readable if and only if the guest gate passed.  Do not change the separate
magneto-optical entry.

- [ ] **Step 3: Add the verified decorated-window GIF**

Use the lab-produced GIF without re-encoding it in QEMU.  Link it from the
NetBSD section and describe its endpoint as the local `sd0a` shell.

- [ ] **Step 4: Verify docs, tests, and tree hygiene**

Run:

```sh
git diff --check
QTEST_QEMU_BINARY=build/qemu-system-m68k \
  build/tests/qtest/next-cube-scsi-test
```

Confirm `git status --short` contains only intended source, documentation, and
GIF changes; `build/` remains ignored.

- [ ] **Step 5: Commit documentation**

```sh
git add QEMU-NeXT-README.md docs/system/target-m68k.rst \
  docs/boot/netbsd-scsi.gif
git commit -m "docs: describe NetBSD SCSI local boot"
```

### Task 6: Final QEMU verification

**Files:** None expected

- [ ] **Step 1: Run focused tests from the feature build**

```sh
ninja -C build qemu-system-m68k tests/qtest/next-cube-scsi-test
QTEST_QEMU_BINARY=build/qemu-system-m68k \
  build/tests/qtest/next-cube-scsi-test
meson test -C build --print-errorlogs qtest-m68k/next-cube-scsi-test
```

- [ ] **Step 2: Run the established NeXT required set**

Run the exact 18-test NeXT required set recorded by the companion lab build
metadata and require 18/18 PASS.

- [ ] **Step 3: Verify repository state**

```sh
git diff --check
git status --short --branch
git log --oneline --decorate -8
```

Expected: clean feature worktree except ignored build outputs, with scoped and
reviewable commits.

### Task 7: Correct NeXT DCTL-to-ESP DMA gating after Task 17 r3

> **Runtime correction (2026-08-26):** This task supersedes the earlier
> assumption that the completed synthetic DMA tests were sufficient for the
> NetBSD data-phase order.  Preserve the earlier tasks as history and execute
> this task before another visible campaign.

**Files:**

- Modify: `tests/qtest/next-cube-scsi-test.c`
- Modify: `hw/m68k/next-cube.c`

R3 is invalid operator evidence because its fed `nextdma0` address was wrong.
Keep only its diagnostic artifacts: frame
`/home/blanham/projects/NeXT/lab/work/netbsd-scsi-task17-r3-build/operator-logs/r3-kernel-device-full-sequence.png`
(48,699 bytes, SHA-256
`0fe490a96781035d0c10a0e96074219f1b26a33bbe56dd7e544d483c12fa240a`)
and capture
`/mnt/e/disks/netbsd-scsi-task9-20260826/netbsd-scsi-101-3phase-r3-ffv1.mkv`
(909,053,732 bytes, 539.133 seconds, SHA-256
`7a58c87502cca49b1f5f0ddd506b3fceca2bebbc27f92afa206ae2a2a0a2df41`).
Neither artifact is passing evidence.

The Task 7 fixture is normative rather than host-default-dependent.  Create a
512-KiB zeroed raw disk and an 8,192-byte (four 2,048-byte sectors) raw CD,
and instantiate target 0 as
`scsi-hd,channel=0,scsi-id=0,lun=0,vendor=QEMU,product=QEMU HARDDISK,ver=2.5+,scsi_version=5,logical_block_size=512,physical_block_size=512`
and target 3 as
`scsi-cd,channel=0,scsi-id=3,lun=0,vendor=QEMU,product=QEMU CD-ROM,ver=2.5+,scsi_version=5,logical_block_size=2048,physical_block_size=2048`.
Those pinned properties and sizes, not incidental QEMU defaults, justify the
exact INQUIRY and READ CAPACITY bytes below.

- [ ] **Cycle A, Step 1: Add authentic NetBSD-order and CPUDMA-low RED qtests**

Add these exact C test names and registrations:

```text
test_scsi_netbsd_order_disk_inquiry
/next-cube/scsi/netbsd-order-disk-inquiry
test_scsi_netbsd_order_cd_inquiry
/next-cube/scsi/netbsd-order-cd-inquiry
test_scsi_netbsd_order_disk_read_capacity
/next-cube/scsi/netbsd-order-disk-read-capacity
test_scsi_netbsd_order_cd_read_capacity
/next-cube/scsi/netbsd-order-cd-read-capacity
test_scsi_netbsd_order_retains_response_until_dctl_cpudma
/next-cube/scsi/netbsd-order-retains-response-until-dctl-cpudma
test_scsi_cpudma_low_retains_response
/next-cube/scsi/cpudma-low-retains-response
```

Use the existing target-0 disk/target-3 CD fixture and consume each device's
power-on unit attention before the command under test.  Add one helper which
matches the NetBSD driver order exactly: write non-DMA DCTL `0xe8`, put
IDENTIFY byte `0xc0` followed by the CDB in the ESP FIFO, select with
`ESP_CMD_SELATN` (`0x42`), program the ESP transfer count, and issue
`ESP_CMD_TI_DMA` (`0x90`) while both the NeXT DMA channel and DCTL CPUDMA are
disabled.  Poison the guest buffer before the command.  After proving
retention, program and enable the NeXT data-in DMA window, prove retention
again, and only then write DCTL `0xf8` to `NEXT_SCSI_CSR`.

Before DCTL `0xf8`, assert the poisoned buffer is unchanged.  Before the NeXT
DMA channel is enabled, also assert its programmed current pointer cannot
advance; after it is enabled but before DCTL `0xf8`, assert both buffer and
pointer still remain unchanged.  Data-in DCTL `0xe8` lacks
`SCSI_CSR_CPUDMA == 0x10` and must not release the response.  Only the
transition to `0xf8` may release the queued response and advance the DMA
window.  This is the retention oracle; do not use a sleep or accept all-zero
data.

Assert these exact first 32 INQUIRY bytes:

```c
static const uint8_t netbsd_disk_inquiry_prefix[32] = {
    0x00, 0x00, 0x05, 0x12, 0x1f, 0x00, 0x00, 0x10,
    'Q', 'E', 'M', 'U', ' ', ' ', ' ', ' ',
    'Q', 'E', 'M', 'U', ' ', 'H', 'A', 'R',
    'D', 'D', 'I', 'S', 'K', ' ', ' ', ' ',
};
static const uint8_t netbsd_cd_inquiry_prefix[32] = {
    0x05, 0x80, 0x05, 0x12, 0x1f, 0x00, 0x00, 0x10,
    'Q', 'E', 'M', 'U', ' ', ' ', ' ', ' ',
    'Q', 'E', 'M', 'U', ' ', 'C', 'D', '-',
    'R', 'O', 'M', ' ', ' ', ' ', ' ', ' ',
};
```

Assert the complete eight-byte READ CAPACITY responses for the existing
512-KiB disk and four-sector CD fixture:

```c
static const uint8_t netbsd_disk_capacity[8] = {
    0x00, 0x00, 0x03, 0xff, 0x00, 0x00, 0x02, 0x00,
};
static const uint8_t netbsd_cd_capacity[8] = {
    0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x08, 0x00,
};
```

Keep the existing `test_scsi_disabled_dma_does_not_complete` as the independent
NeXT-channel gate: it must write data-in DCTL `0xf8` so CPUDMA is **high** while
the NeXT DMA channel remains reset/disabled, and it must still retain the
response.  Do not turn that test into another CPUDMA-low test.

The new CPUDMA-low test covers the converse gate.  Do not add reset or
migration tests yet: Cycle A must isolate the baseline early-consumption bug
and the live CSR-write gate before testing owner reconstruction.

- [ ] **Cycle A, Step 2: Run each new test and preserve the diagnosed RED checkpoint**

```sh
ninja -C build qemu-system-m68k tests/qtest/next-cube-scsi-test
mkdir -p build/task7-checkpoints
: > build/task7-checkpoints/cycle-a-red.tap
for path in \
  netbsd-order-disk-inquiry \
  netbsd-order-cd-inquiry \
  netbsd-order-disk-read-capacity \
  netbsd-order-cd-read-capacity \
  netbsd-order-retains-response-until-dctl-cpudma \
  cpudma-low-retains-response
do
  output="$(QTEST_QEMU_BINARY=build/qemu-system-m68k \
    build/tests/qtest/next-cube-scsi-test -p "/next-cube/scsi/$path")"
  printf '%s\n' "$output" | tee -a build/task7-checkpoints/cycle-a-red.tap
  test "$(printf '%s\n' "$output" | rg -c \
    "^(ok|not ok) 1 /next-cube/scsi/$path$")" -eq 1
  test "$(printf '%s\n' "$output" | rg -c \
    '^(ok|not ok) [0-9]+ ')" -eq 1
done
test "$(rg -c '^not ok 1 /next-cube/scsi/(netbsd-order-|cpudma-low-retains-response)' \
  build/task7-checkpoints/cycle-a-red.tap)" -eq 6
```

Save this complete TAP stream as the Cycle A RED checkpoint.  Expected: the
five NetBSD-order tests and the independent CPUDMA-low test FAIL because
`next_scsi_realize()` currently forces `esp->dma_enabled = 1`; `TRANS | DMA`
consumes the response
before the NeXT DMA channel is enabled and before DCTL `0xf8`, so the callback
drops it and the poisoned buffer cannot later acquire the exact
prefix/capacity bytes.  A compile error, fixture failure, zero selected tests,
or a TAP stream without the exact registered test name and exactly one
execution is not the required RED result.  The command above applies that
exact-name/exactly-one TAP check to every newly registered test; in GREEN, the
matching row must be `ok`.

- [ ] **Cycle A, Step 3: Implement only the live DCTL CPUDMA gate**

Remove the unconditional `esp->dma_enabled = 1` from
`next_scsi_realize()`.  Add this machine-local derived-state helper in
`hw/m68k/next-cube.c`:

```c
static void next_scsi_update_esp_dma(NeXTSCSI *s)
{
    ESPState *esp = &SYSBUS_ESP(&s->sysbus_esp)->esp;

    esp_dma_enable(esp, 0, !!(s->scsi_csr_1 & SCSICSR_CPUDMA));
}
```

In `next_scsi_csr_write()`, store `scsi_csr_1`, propagate that value with
`next_dma_set_scsi_control()`, and only then call the helper so DCTL
`SCSICSR_CPUDMA` is the sole live ESP DMA-enable authority.  Do **not** change
`next_scsi_reset()` or `next_scsi_post_load()` in Cycle A.  Do not change
generic `hw/scsi/esp.c` payload handling or synthesize inquiry/capacity bytes.

Update every existing qtest helper which expects ESP DMA completion to write
the appropriate DCTL value after enabling/programming the NeXT DMA channel:
`0xf8` for data-in and `0xf0` for data-out.  The Cycle A CPUDMA-retention test
explicitly leaves `SCSI_CSR_CPUDMA` clear; the existing disabled-NeXT-DMA
test explicitly keeps CPUDMA high while the channel remains disabled.
Preserve all existing transfer, tail, chaining, reset, IRQ, disk, and CD
assertions.

- [ ] **Cycle A, Step 4: Verify the six tests GREEN and checkpoint them**

```sh
ninja -C build qemu-system-m68k tests/qtest/next-cube-scsi-test
output="$(QTEST_QEMU_BINARY=build/qemu-system-m68k \
  build/tests/qtest/next-cube-scsi-test)"
status=$?
printf '%s\n' "$output" | tee build/task7-checkpoints/cycle-a-green.tap
test "$status" -eq 0
for path in \
  netbsd-order-disk-inquiry \
  netbsd-order-cd-inquiry \
  netbsd-order-disk-read-capacity \
  netbsd-order-cd-read-capacity \
  netbsd-order-retains-response-until-dctl-cpudma \
  cpudma-low-retains-response
do
  test "$(rg -c "^ok [0-9]+ /next-cube/scsi/$path$" \
    build/task7-checkpoints/cycle-a-green.tap)" -eq 1
done
git diff --check
```

Expected: all five NetBSD-order regressions and the CPUDMA-low regression pass.
Retain the Cycle A RED and GREEN TAP logs as review checkpoints; do not commit
between cycles.

- [ ] **Cycle B, Step 5: Add owner reset/migration tests against the Cycle A implementation**

Add these exact tests and registrations only after Cycle A is GREEN:

```text
test_scsi_reset_forces_cpudma_low_for_new_ti
/next-cube/scsi/reset-forces-cpudma-low-for-new-ti
test_scsi_migration_restores_cpudma_low_for_new_ti
/next-cube/scsi/migration-restores-cpudma-low-for-new-ti
test_scsi_migration_restores_cpudma_high_for_new_ti
/next-cube/scsi/migration-restores-cpudma-high-for-new-ti
```

For reset, first write `0xf8` and prove CPUDMA high, perform a system reset,
consume the reset unit attention, program and enable a poisoned NeXT DMA
window, then issue a **new** authenticated SELATN/IDENTIFY-`0xc0` TI without a
new DCTL write.  Require reset-low retention; writing `0xf8` afterward must
release the exact response once.

For migration-low, prove the source's live owner gate by writing `0xf8` and
then `0xe8`, assert the CSR reads back low, and migrate/save with **no pending
TI**.  After load, assert the destination CSR is still low, program and enable
a poisoned DMA window, and issue a **new** authentic SELATN/IDENTIFY-`0xc0` TI.
Require poison and pointer retention; writing `0xf8` must then release one
exact response.  For migration-high, set up a valid CPUDMA-high state with
**no pending callback**, migrate/save and load, then issue a new TI and require
immediate exact transfer.  A high-plus-pending source state is unreachable:
with CPUDMA high, issuing TI immediately consumes or resumes it.  Do not
fabricate that impossible source just to test post-load.

`vmstate_esp` does not migrate `dma_cb`, `async_len`, or `async_buf`, so a
transparent migration cannot safely preserve an ESP transfer while either a
deferred callback or target data window is live.  The bounded policy is to
reject those transient migrations before writing NeXTSCSI state.  Generic ESP
owns a read-only semantic query covering both conditions; NeXTSCSI owns the
`pre_save_errp` policy and descriptive error.  Idle CPUDMA-high, idle
CPUDMA-low, and low-with-no-pending-TI migration remain supported.  Transparent
pending-I/O migration is deferred until generic ESP/SCSI migration state can
represent both the continuation and its active data window.

- [ ] **Cycle B, Step 6: Prove the intermediate implementation RED**

Run each of the three registered `/next-cube/scsi/...` paths with the same
exact-name/exactly-one TAP assertions from Cycle A and preserve the combined
stream as `task7-cycle-b-red.tap`.  The reset and migration-high tests must
both fail against the Cycle A-only implementation because reset/post-load do
not restore the owner-derived ESP level.  Migration-low is expected to be a
GREEN guard here when the destination's default derived level is already low;
it still proves CSR-low readback and new-TI retention.  Zero selected tests, a
fixture failure, or failure for any other reason is not the required
checkpoint.

```sh
: > build/task7-checkpoints/cycle-b-red.tap
for path in \
  reset-forces-cpudma-low-for-new-ti \
  migration-restores-cpudma-low-for-new-ti \
  migration-restores-cpudma-high-for-new-ti
do
  output="$(QTEST_QEMU_BINARY=build/qemu-system-m68k \
    build/tests/qtest/next-cube-scsi-test -p "/next-cube/scsi/$path")"
  printf '%s\n' "$output" | tee -a build/task7-checkpoints/cycle-b-red.tap
  test "$(printf '%s\n' "$output" | rg -c \
    "^(ok|not ok) 1 /next-cube/scsi/$path$")" -eq 1
  test "$(printf '%s\n' "$output" | rg -c \
    '^(ok|not ok) [0-9]+ ')" -eq 1
done
test "$(rg -c '^not ok 1 /next-cube/scsi/(reset-forces-cpudma-low-for-new-ti|migration-restores-cpudma-high-for-new-ti)$' \
  build/task7-checkpoints/cycle-b-red.tap)" -eq 2
test "$(rg -c '^ok 1 /next-cube/scsi/migration-restores-cpudma-low-for-new-ti$' \
  build/task7-checkpoints/cycle-b-red.tap)" -eq 1
```

- [ ] **Cycle B, Step 7: Implement reset-low and post-load derived synchronization**

In `next_scsi_reset()`, clear both CSR bytes, propagate zero to the NeXT DMA
control, then call `next_scsi_update_esp_dma()` so reset is low.  In
`next_scsi_post_load()`, wait until `scsi_csr_1` and all NeXT DMA
channel/control state are restored, call `next_dma_set_scsi_control()` with
the restored CSR, and only then call `next_scsi_update_esp_dma()`.

The reachable migration-high test has no pending callback.  Nevertheless,
keep this defensive ordering because `esp_dma_enable()` may synchronously
resume a pending callback from a compatibility stream or future state; it must
never run before the CSR and DMA control destination are valid.

- [ ] **Cycle B, Step 8: Verify all GREEN, the complete binary, and Meson**

```sh
ninja -C build qemu-system-m68k tests/qtest/next-cube-scsi-test
output="$(QTEST_QEMU_BINARY=build/qemu-system-m68k \
  build/tests/qtest/next-cube-scsi-test)"
status=$?
printf '%s\n' "$output" | tee build/task7-checkpoints/cycle-b-green.tap
test "$status" -eq 0
for test_path in \
  netbsd-order-disk-inquiry \
  netbsd-order-cd-inquiry \
  netbsd-order-disk-read-capacity \
  netbsd-order-cd-read-capacity \
  cpudma-low-retains-response \
  reset-forces-cpudma-low-for-new-ti \
  migration-restores-cpudma-low-for-new-ti \
  migration-restores-cpudma-high-for-new-ti \
  migration-rejects-pending-cpudma-low-ti \
  migration-rejects-pre-ti-async-window
do
  test "$(rg -c "^ok [0-9]+ /m68k/next-cube/scsi/$test_path$" \
    build/task7-checkpoints/cycle-b-green.tap)" -eq 1
done
test "$(rg -c '^ok [0-9]+ /m68k/' \
  build/task7-checkpoints/cycle-b-green.tap)" -eq 39
meson test -C build --print-errorlogs qtest-m68k/next-cube-scsi-test
git diff --check
```

Expected: all ten unique task regressions and all 39 registered tests pass.
Although source registrations use `/next-cube/...`, libqtest adds the target
architecture and emits `/m68k/next-cube/...` in focused and full TAP output.
The retained disabled-NeXT-DMA test still exercises CPUDMA high/channel
disabled, and the retention tests prove poison-before-release and exact bytes.
The former duplicate retention registration is repurposed as the pending-TI
migration-rejection test; the second rejection test stops before TI at
non-DMA `SELATN`, isolating the live `async_len` predicate.  Rejected migration
must leave the source runnable and unchanged so `0xf8` completes the exact
response, after which a retry migration succeeds.
Preserve all four Cycle A/B RED/GREEN logs and the two fail-safe migration
RED/GREEN checkpoint pairs through review.

- [ ] **Step 9: Commit the fail-safe follow-up separately and obtain review**

```sh
git add docs/superpowers/plans/2026-08-25-netbsd-scsi-qemu.md \
  docs/superpowers/specs/2026-08-25-netbsd-scsi-install-design.md \
  hw/m68k/next-cube.c hw/scsi/esp.c include/hw/scsi/esp.h \
  tests/qtest/next-cube-scsi-test.c
git commit -m "next: reject migration with transient ESP DMA"
git status --short --branch
```

Run a fresh specification-compliance review, then a fresh code-quality review.
The live/reset/owner gate implementation remains in its existing coherent
commit; this bounded migration-rejection policy is a separate follow-up.  The
ignored checkpoint logs provide staged RED/GREEN evidence and are not partial
commits.
Resolve each blocker test-first and rerun Step 8 after every fix.  Only an
approved clean QEMU commit may be supplied to the lab's clean-build and r4
steps; r3 must never be retried or promoted in place.
