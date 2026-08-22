# Native Plan 9 IL and Authentication Roadmap

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver a private, historically compatible IL/566 authentication service and authenticated IL/17008 9P1 root service for the unmodified Plan 9 Second Edition NeXT guest while preserving unauthenticated TCP/564.

**Architecture:** Implement wire-level IL in the public libslirp fork, consume it through a record-oriented QEMU adapter, add native Plan 9 ticket/key-database authentication to the existing 9P1 server, then add a lab profile that proves both password rejection and successful root mounting. The tracks land in that order and each repository remains independently testable.

**Tech Stack:** C11, libslirp, IPv4 protocol 40, QEMU QOM/QAPI and Crypto, 9P1, Plan 9 DES tickets, Meson/Ninja, GLib tests, QTest, Python unittest, classic PCAP, authenticated QMP.

---

## Authoritative documents

- Design: `docs/superpowers/specs/2026-08-21-qemu-plan9-il-auth-design.md`
- IL plan: `docs/superpowers/plans/2026-08-21-public-libslirp-il.md`
- QEMU plan: `docs/superpowers/plans/2026-08-21-qemu-plan9-il-auth.md`
- Lab plan: `docs/superpowers/plans/2026-08-21-next-plan9-il-auth-lab.md`
- Historical oracle: `/home/blanham/projects/NeXT/evidence/runs/20260821T055758Z-p9net-4ca9deb10c8f-d653378c/rootfs`

When prose and code differ, use `sys/src/9/port/stil.c` for IL and the staged `sys/include/auth.h`, `sys/src/libauth`, `sys/src/fs/port/auth.c`, and `sys/src/9/port/auth.c` for authentication.

## Worktrees and branches

- libslirp: `/home/blanham/projects/NeXT/lab/.worktrees/plan9-il-libslirp`, branch `feature/plan9-il`, based on public BOOTP commit `0454d236f6386eade10946145ffdf3bd7e28d409`.
- QEMU: `/home/blanham/projects/NeXT/lab/.worktrees/plan9-il-auth-qemu`, branch `feature/plan9-il-auth`; design commit `8de2e2ec84` is already present.
- lab: `/home/blanham/projects/NeXT/lab/.worktrees/plan9-il-auth-lab`, branch `feature/plan9-il-auth-lab`, based on the current lab branch before implementation.

Do not touch unrelated untracked files in the primary QEMU or lab checkouts. Do not stage QEMU `TODO.md`; it is local hardware follow-up state excluded through the common Git directory.

## Dependency and review order

### Phase 1: Public libslirp IL

- [ ] Execute every task in `2026-08-21-public-libslirp-il.md` with a fresh implementer per task.
- [ ] After every task, run a specification review followed by a code-quality review; resolve and re-review findings before continuing.
- [ ] Run the full libslirp suite with AddressSanitizer when supported.
- [ ] Publish the reviewed branch and record its immutable full commit SHA.

Exit gate: public headers, symbol map, deterministic IL tests, and all existing libslirp tests are green. The public API preserves record boundaries and no host raw socket is used.

### Phase 2: QEMU adapter and authenticated server

- [ ] Pin `subprojects/slirp.wrap` to the reviewed public SHA.
- [ ] Execute every task in `2026-08-21-qemu-plan9-il-auth.md`, retaining the same two-stage review after each task.
- [ ] Prove an ordinary TCP/564 build and object lifecycle still work when the system libslirp lacks IL.
- [ ] Prove the IL-enabled fallback build exposes no host TCP or raw-protocol listener.

Exit gate: focused unit/qtests pass, the full m68k qtest slice passes, TCP mode is byte-compatible, and sensitive material is absent from argv, logs, QMP, and tests.

### Phase 3: Lab controller and acceptance

- [ ] Execute every task in `2026-08-21-next-plan9-il-auth-lab.md` with two-stage review.
- [ ] Run the existing TCP Plan 9 profile as a regression.
- [ ] Provision a fresh local key database and Secret file.
- [ ] Run the wrong-password profile first; prove root did not mount.
- [ ] Start a clean VM and run the correct-password profile; prove root mounted and the guest reached the configured terminal/desktop gate.
- [ ] Parse capture metadata and prove IPv4 protocol 40 traffic used IL destination ports 566 and 17008.

Exit gate: the paired evidence bundle is non-overwriting, secret-free, and accepted by the promotion gate.

## Cross-repository completion gate

Run from the three worktrees:

```bash
meson test -C build --print-errorlogs
meson test -C build-plan9-il-auth --print-errorlogs \
  test-slirp-il test-plan9-auth test-plan9-9p1-codec \
  test-plan9-9p1-server test-plan9-9p1-server-local
meson test -C build-plan9-il-auth --print-errorlogs \
  qemu:qtest-m68k/plan9-9p1-object-test
python3 -B -m unittest discover -s tests
git diff --check
```

Expected: every command exits zero. Then inspect the final argv/QMP/log/manifest/PCAP metadata for the literal password, base64 master key, seven-byte master key, server key, and conversation keys. No match is acceptable.

Before claiming completion, invoke `superpowers:verification-before-completion`, capture the exact command output, and report any test that could not run rather than inferring success.
