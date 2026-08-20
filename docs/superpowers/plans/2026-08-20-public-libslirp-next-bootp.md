# Public libslirp NeXT BOOTP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish the working NeXT BOOTP compatibility code as normal commits in `blanham/libslirp` and pin qemu-NeXT to the tested public commit instead of a generated overlay patch.

**Architecture:** A dedicated `next-bootp` libslirp branch starts from QEMU's exact pinned upstream revision and contains one vendor-protocol commit followed by one unicast-routing commit. QEMU's wrap file fetches the public fork by full commit SHA; a fresh detached QEMU worktree proves that no generated local subproject masks the dependency.

**Tech Stack:** Git worktrees and GitHub remotes, libslirp C/Meson tests, QEMU Meson fallback subprojects, m68k qtests, Python unittest.

---

## File map

libslirp worktree `/home/blanham/projects/NeXT/lab/.worktrees/libslirp-next-bootp`:

- Modify `meson.build`: register the focused BOOTP test executable.
- Modify `src/bootp.c`: recognize and encode NeXT vendor replies, then preserve the assigned unicast destination for NeXT version 1.
- Create `test/bootptest.c`: exercise NeXT vendor replies, fallback behavior, and L2/L3 destinations.

QEMU worktree `/home/blanham/projects/NeXT/lab/.worktrees/plan9-rom-netboot-qemu`:

- Modify `subprojects/slirp.wrap`: fetch `blanham/libslirp` at the public full commit SHA.
- Delete `subprojects/packagefiles/slirp-next-bootp.patch`: remove the superseded generated overlay.

Lab worktree `/home/blanham/projects/NeXT/lab/.worktrees/plan9-next-netboot`:

- Test `tests/test_plan9_netboot.py`: re-run the existing visible-launch and immutable-staging contract tests; no source change is expected.

### Task 1: Prepare the public libslirp branch and initial compatibility commit

**Files:**
- Modify: `/home/blanham/projects/NeXT/lab/.worktrees/libslirp-next-bootp/meson.build`
- Modify: `/home/blanham/projects/NeXT/lab/.worktrees/libslirp-next-bootp/src/bootp.c`
- Create: `/home/blanham/projects/NeXT/lab/.worktrees/libslirp-next-bootp/test/bootptest.c`

- [ ] **Step 1: Create an isolated branch at QEMU's exact libslirp base**

Verify the destination does not already exist. Then create it from the Meson-owned libslirp repository without modifying that repository's dirty checkout:

```bash
test ! -e /home/blanham/projects/NeXT/lab/.worktrees/libslirp-next-bootp
git -C /home/blanham/projects/NeXT/lab/.worktrees/plan9-rom-netboot-qemu/subprojects/slirp \
  worktree add -b next-bootp \
  /home/blanham/projects/NeXT/lab/.worktrees/libslirp-next-bootp \
  26be815b86e8d49add8c9a8b320239b9594ff03d
git -C /home/blanham/projects/NeXT/lab/.worktrees/libslirp-next-bootp \
  remote add github https://github.com/blanham/libslirp.git
```

Expected: the new worktree is clean on `next-bootp`, `HEAD` is exactly `26be815b86e8d49add8c9a8b320239b9594ff03d`, and the original generated checkout remains dirty and detached exactly as before.

- [ ] **Step 2: Apply the already-reviewed initial overlay as source**

Use the patch content from QEMU commit `e331ab7465`; this is the green pre-unicast implementation, not the later routing change:

```bash
git -C /home/blanham/projects/NeXT/lab/.worktrees/plan9-rom-netboot-qemu \
  show e331ab7465:subprojects/packagefiles/slirp-next-bootp.patch |
git -C /home/blanham/projects/NeXT/lab/.worktrees/libslirp-next-bootp apply -
git -C /home/blanham/projects/NeXT/lab/.worktrees/libslirp-next-bootp status --short
```

Expected status:

```text
 M meson.build
 M src/bootp.c
?? test/bootptest.c
```

- [ ] **Step 3: Build and run the initial protocol tests**

```bash
meson setup build
ninja -C build
meson test -C build bootp --print-errorlogs
```

Run from `/home/blanham/projects/NeXT/lab/.worktrees/libslirp-next-bootp`. Expected: `bootp` passes; the generated `build/` directory remains ignored.

- [ ] **Step 4: Verify scope and commit the initial source change**

```bash
git diff --check
git status --short
git add -- meson.build src/bootp.c test/bootptest.c
git diff --cached --check
git commit -m "bootp: support NeXT vendor replies"
```

Expected: the commit changes exactly the three named paths and keeps the existing source/test licenses.

### Task 2: Add the unicast regression and minimal routing fix

**Files:**
- Modify: `/home/blanham/projects/NeXT/lab/.worktrees/libslirp-next-bootp/test/bootptest.c`
- Modify: `/home/blanham/projects/NeXT/lab/.worktrees/libslirp-next-bootp/src/bootp.c`

- [ ] **Step 1: Add destination constants and assertions**

Add after `DHCP_OPTION_CAPACITY`:

```c
#define IPV4_DESTINATION_OFFSET 16
#define IPV4_ADDRESS_LEN 4
```

Add after `check_common_reply()`:

```c
static void check_unicast_destination(const Capture *capture)
{
    assert(memcmp(capture->packet, client_mac, sizeof(client_mac)) == 0);
    assert(memcmp(capture->packet + ETHERNET_HEADER_LEN +
                      IPV4_DESTINATION_OFFSET,
                  capture->packet + BOOTP_OFFSET + BOOTP_YIADDR_OFFSET,
                  IPV4_ADDRESS_LEN) == 0);
}

static void check_broadcast_destination(const Capture *capture)
{
    static const uint8_t broadcast[6] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    };

    assert(memcmp(capture->packet, broadcast, sizeof(broadcast)) == 0);
    assert(memcmp(capture->packet + ETHERNET_HEADER_LEN +
                      IPV4_DESTINATION_OFFSET,
                  broadcast, IPV4_ADDRESS_LEN) == 0);
}
```

Call `check_unicast_destination(&capture);` immediately after
`check_common_reply(&capture);` in `test_next_v1_reply()`. Call
`check_broadcast_destination(&capture);` at the same location in both
`test_next_unknown_version_falls_back()` and
`test_rfc1533_reply_unchanged()`.

- [ ] **Step 2: Prove the focused test is red for the routing defect**

```bash
ninja -C build
meson test -C build bootp --print-errorlogs
```

Expected: `bootp` aborts in `check_unicast_destination()` because the NeXT-v1 reply is still sent to the broadcast Ethernet/IP destination. The two fallback requests remain broadcast.

- [ ] **Step 3: Implement the minimal recognized-client exception**

In `src/bootp.c`, replace the unconditional destination overwrite near the end of `bootp_reply()`:

```c
daddr.sin_addr.s_addr = 0xffffffffu;
```

with:

```c
if (!next_bootp) {
    daddr.sin_addr.s_addr = 0xffffffffu;
}
```

Do not change address assignment, packet construction, unknown NeXT versions, or RFC1533 handling.

- [ ] **Step 4: Run the focused and complete libslirp tests**

```bash
ninja -C build
meson test -C build bootp --print-errorlogs
meson test -C build --print-errorlogs
```

Expected: `bootp`, `ping`, and `ncsi` all pass. NeXT-v1 is unicast; both fallback cases remain broadcast.

- [ ] **Step 5: Confirm exact equivalence with the working QEMU overlay**

The current worktree source must match the files produced by applying QEMU commit `538944022e` to the same base. Use a temporary index-free comparison or inspect the three file hashes from the existing applied subproject:

```bash
sha256sum meson.build src/bootp.c test/bootptest.c
sha256sum \
  /home/blanham/projects/NeXT/lab/.worktrees/plan9-rom-netboot-qemu/subprojects/slirp/meson.build \
  /home/blanham/projects/NeXT/lab/.worktrees/plan9-rom-netboot-qemu/subprojects/slirp/src/bootp.c \
  /home/blanham/projects/NeXT/lab/.worktrees/plan9-rom-netboot-qemu/subprojects/slirp/test/bootptest.c
```

Expected: corresponding hashes are identical. If they differ, inspect the diff and correct only an unintended discrepancy before committing.

- [ ] **Step 6: Commit the tested unicast change**

```bash
git diff --check
git add -- src/bootp.c test/bootptest.c
git diff --cached --check
git commit -m "bootp: unicast NeXT v1 replies"
git log --oneline --decorate -3
```

Expected: two Bryce Lanham commits follow the exact `26be815b` base, and the worktree is clean.

### Task 3: Publish and authenticate `blanham/libslirp:next-bootp`

**Files:**
- Remote branch: `https://github.com/blanham/libslirp`, `next-bootp`

- [ ] **Step 1: Re-run the publication gate**

```bash
git status --short
git diff --check
meson test -C build --print-errorlogs
git log --format=fuller -2
```

Expected: clean status, all tests pass, both commits are authored by Bryce Lanham, and no generated files are tracked.

- [ ] **Step 2: Confirm the target branch is not already public**

```bash
GIT_CONFIG_NOSYSTEM=1 git ls-remote --heads \
  https://github.com/blanham/libslirp.git refs/heads/next-bootp
```

Expected: no output. If the branch exists, stop and compare its SHA instead of overwriting it.

- [ ] **Step 3: Push only the reviewed feature branch**

```bash
GIT_CONFIG_NOSYSTEM=1 git push --set-upstream github next-bootp
```

Expected: GitHub creates `refs/heads/next-bootp`; `ios-support` is not updated.

- [ ] **Step 4: Verify the public branch identity**

```bash
git rev-parse HEAD
GIT_CONFIG_NOSYSTEM=1 git ls-remote --heads \
  https://github.com/blanham/libslirp.git refs/heads/next-bootp
```

Expected: the local and remote full SHA values are identical. Record this value as the immutable QEMU wrap revision.

### Task 4: Replace QEMU's overlay with the public commit pin

**Files:**
- Modify: `/home/blanham/projects/NeXT/lab/.worktrees/plan9-rom-netboot-qemu/subprojects/slirp.wrap`
- Delete: `/home/blanham/projects/NeXT/lab/.worktrees/plan9-rom-netboot-qemu/subprojects/packagefiles/slirp-next-bootp.patch`

- [ ] **Step 1: Capture and validate the published SHA**

```bash
libslirp_tip=$(git -C \
  /home/blanham/projects/NeXT/lab/.worktrees/libslirp-next-bootp \
  rev-parse HEAD)
test "$libslirp_tip" = "$(GIT_CONFIG_NOSYSTEM=1 git ls-remote --heads \
  https://github.com/blanham/libslirp.git refs/heads/next-bootp |
  awk '{print $1}')"
```

Expected: the equality test succeeds and `libslirp_tip` contains a 40-character commit ID.

- [ ] **Step 2: Convert the QEMU wrap contract**

Using `apply_patch`, change the URL to
`https://github.com/blanham/libslirp.git`, replace the existing revision with
the literal 40-character value printed by `printf '%s\n' "$libslirp_tip"`, and
remove the `diff_files` line. Preserve the existing `[wrap-git]` and
`[provide]` sections and the `slirp = libslirp_dep` provider declaration.

Using the same patch, delete
`subprojects/packagefiles/slirp-next-bootp.patch`. Do not modify the generated
`subprojects/slirp` checkout.

Verify that the revision was transcribed as a literal SHA rather than the
descriptive text in the example:

```bash
test "$(sed -n 's/^revision = //p' subprojects/slirp.wrap)" = \
  "$libslirp_tip"
```

- [ ] **Step 3: Check the conversion diff before verification**

```bash
git diff --check
git diff -- subprojects/slirp.wrap \
  subprojects/packagefiles/slirp-next-bootp.patch
git status --short
```

Expected: one wrap edit and one patch deletion; the documentation commits and all unrelated QEMU files remain untouched.

### Task 5: Prove the QEMU pin from a clean fallback checkout

**Files:**
- Runtime only: a unique directory below `/tmp/qemu-next-public-slirp-*`
- Test: QEMU `slirp:bootp` and `tests/qtest/next-mb8795-test`

- [ ] **Step 1: Create a detached, disposable QEMU source worktree**

```bash
verify_root=$(mktemp -d /tmp/qemu-next-public-slirp-XXXXXXXX)
mkdir -p "$verify_root/build"
git worktree add --detach "$verify_root/source" HEAD
git diff -- subprojects/slirp.wrap \
  subprojects/packagefiles/slirp-next-bootp.patch |
git -C "$verify_root/source" apply -
test ! -e "$verify_root/source/subprojects/slirp"
```

Run the final three commands from the QEMU feature worktree. Expected: the detached source contains the uncommitted wrap conversion and has no local SLiRP source directory.

- [ ] **Step 2: Fetch only through the public wrap**

```bash
/home/blanham/projects/NeXT/lab/.worktrees/plan9-rom-netboot-qemu/build/pyvenv/bin/meson \
  subprojects download slirp
git -C "$verify_root/source/subprojects/slirp" rev-parse HEAD
git -C "$verify_root/source/subprojects/slirp" remote -v
```

Run from `$verify_root/source`. Expected: `HEAD` equals the validated public
`libslirp_tip`, and the checkout remote names
`https://github.com/blanham/libslirp.git`.

- [ ] **Step 3: Configure QEMU and force the fallback dependency**

```bash
cd "$verify_root/build"
"$verify_root/source/configure" \
  --target-list=m68k-softmmu --enable-slirp --enable-download
./pyvenv/bin/meson setup --reconfigure --force-fallback-for=slirp \
  . "$verify_root/source"
```

Expected: Meson reports `slirp` from the fallback subproject, not a system pkg-config installation.

- [ ] **Step 4: Build and run the dependency and Ethernet regressions**

```bash
ninja -C "$verify_root/build" \
  qemu-system-m68k tests/qtest/next-mb8795-test
"$verify_root/build/pyvenv/bin/meson" test \
  -C "$verify_root/build" slirp:bootp --print-errorlogs
QTEST_QEMU_BINARY="$verify_root/build/qemu-system-m68k" \
  "$verify_root/build/tests/qtest/next-mb8795-test"
```

Expected: `slirp:bootp` passes and all 24 MB8795 qtests pass.

- [ ] **Step 5: Commit the verified QEMU dependency conversion**

Return to `/home/blanham/projects/NeXT/lab/.worktrees/plan9-rom-netboot-qemu` and run:

```bash
git diff --check
git add -- subprojects/slirp.wrap \
  subprojects/packagefiles/slirp-next-bootp.patch
git diff --cached --check
git commit -m "net: consume public NeXT libslirp fork"
```

Expected: the commit pins the public full SHA and removes the package overlay.

### Task 6: Cross-repository verification and handoff to retrace work

**Files:**
- Test: `/home/blanham/projects/NeXT/lab/.worktrees/plan9-next-netboot/tests/test_plan9_netboot.py`

- [ ] **Step 1: Run the lab netboot contract**

```bash
python3 -B -m unittest tests.test_plan9_netboot -v
git diff --check
git status --short
```

Run from `/home/blanham/projects/NeXT/lab/.worktrees/plan9-next-netboot`. Expected: all five Plan 9 archive/staging/visible-launch tests pass and the worktree remains clean.

- [ ] **Step 2: Audit both public-source and consumer history**

```bash
git -C /home/blanham/projects/NeXT/lab/.worktrees/libslirp-next-bootp \
  log --oneline --decorate -3
git -C /home/blanham/projects/NeXT/lab/.worktrees/libslirp-next-bootp \
  status --short
git -C /home/blanham/projects/NeXT/lab/.worktrees/plan9-rom-netboot-qemu \
  log --oneline --decorate -6
git -C /home/blanham/projects/NeXT/lab/.worktrees/plan9-rom-netboot-qemu \
  status --short
```

Expected: libslirp is clean and tracks public `github/next-bootp`; QEMU is clean and its latest dependency commit follows the approved SLiRP and retrace design/plan documentation.

- [ ] **Step 3: Preserve the visible acceptance boundary**

Do not launch a headless Plan 9 boot. The next implementation cycle resumes
`docs/superpowers/plans/2026-08-20-next-mono-video-retrace.md`; its final GTK
run will use the public-pinned QEMU build and will simultaneously reconfirm
BOOTP, TFTP, and progress beyond the Plan 9 memory-statistics screen. It must
park at `NeXT>` until the user confirms they are watching.
