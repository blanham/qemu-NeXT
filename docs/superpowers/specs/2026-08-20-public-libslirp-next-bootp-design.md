# Public libslirp NeXT BOOTP Design

## Goal

Publish the NeXT BOOTP compatibility work as ordinary source commits in
Bryce Lanham's public libslirp fork, and make qemu-NeXT consume that exact
public revision instead of carrying an opaque generated overlay patch.

## Repository boundaries

The existing `blanham/libslirp` GitHub repository is public and is a fork of
`qemu/libslirp`.  Its default branch is `ios-support`; that branch is outside
this work and remains unchanged.

The new public branch is `next-bootp`.  It starts at
`26be815b86e8d49add8c9a8b320239b9594ff03d`, the exact libslirp revision
currently named by qemu-NeXT's `subprojects/slirp.wrap`.  This avoids mixing
the NeXT change with the fork's unrelated default-branch history.

The consuming QEMU repository is `blanham/qemu-NeXT`.  Its current local
feature branch contains the working compatibility implementation in
`subprojects/packagefiles/slirp-next-bootp.patch`; the public `metachicken`
branch does not yet contain that work.

## Public libslirp history

The `next-bootp` branch contains two reviewable commits authored by Bryce
Lanham:

1. `bootp: support NeXT vendor replies` adds recognition and encoding of the
   NeXT version-1 BOOTP vendor area, preserves ordinary DHCP/BOOTP behavior,
   registers the focused test executable, and adds the initial compatibility
   regressions.
2. `bootp: unicast NeXT v1 replies` leaves the assigned client address in the
   packet destination for recognized NeXT version-1 clients and adds link-
   and network-layer destination assertions.  Unrecognized NeXT versions and
   ordinary RFC1533 clients remain broadcast.

Only `meson.build`, `src/bootp.c`, and `test/bootptest.c` belong to these
commits.  Meson's generated `.meson-subproject-wrap-hash.txt` is never
committed.  Existing libslirp source licensing is preserved, and the new test
retains its BSD-3-Clause declaration and Bryce Lanham copyright.

## QEMU dependency pin

Publication is ordered so QEMU never points at a nonexistent dependency:

1. Build and test the two libslirp commits locally.
2. Push only `next-bootp` to `blanham/libslirp`.
3. Verify that the public branch resolves to the tested local tip.
4. Change qemu-NeXT's `subprojects/slirp.wrap` URL to
   `https://github.com/blanham/libslirp.git` and its `revision` to the full
   public tip SHA.
5. Remove the `diff_files = slirp-next-bootp.patch` line and delete
   `subprojects/packagefiles/slirp-next-bootp.patch` from the QEMU tree.

The full commit pin, rather than the mutable branch name, is the build
contract.  The branch makes the source discoverable; the SHA makes builds
reproducible.  The earlier QEMU overlay commits remain available in Git
history but are absent from the final tree.

## Isolation and failure handling

The public libslirp commits are prepared in a dedicated clean worktree based
on the exact revision.  The Meson-created, dirty subproject directories in
existing QEMU builds are treated as generated state and are not used to
prove the public dependency.

The fork branch is published before QEMU is modified.  A failed fork push or
a remote-SHA mismatch stops the work with QEMU still using its existing
overlay.  A failed clean clone, fallback configuration, or test also prevents
the QEMU conversion from being committed.  Neither repository's default
branch is changed during this work.

No process discovery or broad signaling is part of this work.  Any emulator
acceptance run remains controlled through its exact run-owned QMP session.

## Verification

The libslirp branch must pass its focused `bootp` Meson test.  The test proves:

- a recognized NeXT version-1 request gets the 64-byte NeXT vendor reply;
- that reply is unicast to the requester's Ethernet address and assigned IP;
- an unrecognized NeXT vendor version retains the ordinary broadcast path;
- an RFC1533/DHCP request retains the ordinary broadcast path; and
- the boot filename and assigned client address remain correct.

QEMU verification uses a fresh fallback checkout that has no preexisting
`subprojects/slirp` directory.  Its configured source revision must equal the
published full SHA.  The QEMU SLiRP test, NeXT MB8795 qtests, and Plan 9
netboot contract tests must pass.  A transport acceptance must still show a
NeXT BOOTP request, unicast reply, TFTP request for
`68020/9nextstation`, data transfer, and final acknowledgement.

The later visible Plan 9 scheduler/retrace acceptance remains a separate
change.  Publishing libslirp does not alter the already approved monochrome
retrace design or authorize a headless substitute for its visible GTK run.
