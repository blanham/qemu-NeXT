# Complete Writable NetInfo Server Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete every published NetInfo v2 and binder v1 operation with historical authentication, crash-safe persistence, multiple domain relationships, and replay-safe mutation.

**Architecture:** Mutable domains transact against the ordered in-memory model, append a checksummed operation to a write-ahead journal before acknowledging, and compact into canonical JSON5 snapshots. Binder lifecycle and replication procedures use the same domain abstraction; master/clone behavior remains explicit. Writable mode stays opt-in and continues to block live migration.

**Tech Stack:** C11, GLib/QObject, QEMU crypto/CRC and file APIs, JSON5, ONC RPC/XDR, NetInfo v2, QTest, fuzzing.

---

## Fixed persistence and authorization contract

- Writable startup requires `writable=on,state=/directory`; otherwise QOM
  realization fails. The seed remains immutable.
- State contains `snapshot.json5` and `journal.bin`. Journal records are:
  `QNIJ`, format u32, payload length u32, CRC32C u32, sequence u64, then one
  canonical UTF-8 JSON mutation payload. Integers are little-endian, payloads
  are at most 64 KiB, and sequences increase by one.
- A mutation is validated on a clone, appended and fdatasynced, then published
  atomically in memory and acknowledged. Compaction writes/fsyncs a temporary
  JSON5 snapshot, renames it, fsyncs the directory, then replaces the journal.
- Only a torn final journal record is ignored. A sequence gap, checksum error,
  invalid mutation, or corruption before EOF refuses writable startup.
- Historical write authentication is compatibility behavior: AUTH_SYS UID
  selects `/users` by `uid`; the bitwise-inverted password is carried in the
  machine-name field and checked against the stored `passwd` crypt hash.
  `_writers` and `_writers_<property>` govern access. No default seed contains
  credentials. Local-root/reserved-port bypass is disabled unless explicitly
  configured with `trusted-local-root=on`.
- UDP mutation replay is keyed by peer, XID, program/version/procedure, and
  SHA-256 of the full call. It retains 256 replies for 60 seconds of virtual
  time and never commits an exact retransmission twice.

## File map

- `hw/netinfo/netinfo-db.{c,h}`: transactions, mutations, checksums, roles.
- `hw/netinfo/netinfo-auth.{c,h}`: historical credential and writer checks.
- `hw/netinfo/netinfo-journal.{c,h}`: snapshot/journal/compaction/recovery.
- `hw/netinfo/netinfo-server.{c,h}`, `netinfo-xdr.{c,h}`: remaining procedures,
  replay cache, master/clone exchange.
- `hw/netinfo/netinfo-object.c`, `qapi/qom.json`: writable/state/domain options.
- `tests/unit/test-netinfo-{db,auth,journal,server}.c`, qtest and fuzz configs:
  deterministic verification.

### Task 1: Add transactional database mutations

**Files:** Modify `hw/netinfo/netinfo-db.{c,h}`, `tests/unit/test-netinfo-db.c`.

- [ ] Write failing tests for CREATE(5), DESTROY(6), WRITE(8), CREATEPROP(12),
  DESTROYPROP(13), WRITEPROP(15), RENAMEPROP(16), CREATENAME(18),
  DESTROYNAME(19), and WRITENAME(21), covering insertion positions, stale
  instances, wrong parent, nonempty deletion, bounds, and rollback.
- [ ] Introduce `NetInfoTxn`: deep-copy only touched nodes, validate all IDs and
  resulting limits, increment each changed node's instance exactly once, and
  allocate object IDs monotonically without reuse until restart recovery.
- [ ] Make commit return an immutable `NetInfoMutation` containing operation,
  before-instance, after-instance, arguments, allocated ID, and sequence. Make
  rollback free it without changing the published root/index.
- [ ] Run `test-netinfo-db`; expected all mutation and existing read tests pass.
- [ ] Commit with `git commit -m "netinfo: add transactional database writes"`.

### Task 2: Implement historical authentication and writer ACLs

**Files:** Create `hw/netinfo/netinfo-auth.{c,h}`, `tests/unit/test-netinfo-auth.c`; modify `hw/netinfo/meson.build`.

- [ ] Add failing cases for AUTH_NULL anonymous reads, AUTH_SYS missing user,
  wrong password, empty passwd property, matching crypt password, `_writers`,
  `_writers_name`, superuser, and disabled/enabled trusted-local-root.
- [ ] Decode the AUTH_SYS UID and machine field already bounded by ONC RPC.
  Invert each machine-name byte into a temporary cleartext buffer, call the
  host crypt helper against the stored hash, zero the buffer, then apply the
  nearest directory `_writers` and property `_writers_<name>` lists.
- [ ] Return `NI_NOUSER`, `NI_AUTHERROR`, or `NI_PERM` distinctly. Never trust
  guest GID/groups and never alter host credentials.
- [ ] Run auth tests plus `test-onc-rpc`; expected all pass.
- [ ] Commit with `git commit -m "netinfo: enforce historical write authorization"`.

### Task 3: Add crash-safe snapshot and journal persistence

**Files:** Create `hw/netinfo/netinfo-journal.{c,h}`, `tests/unit/test-netinfo-journal.c`; modify Meson files.

- [ ] Write failing tests for empty state creation, ordered replay, torn final
  header/payload, bad CRC, sequence gap, oversized payload, invalid mutation,
  fdatasync failure, snapshot rename interruption, directory-sync failure, and
  successful compaction/reopen.
- [ ] Use `openat()` relative to an opened state directory with no symlink
  following. Refuse nonregular files and group/world-writable state directories.
- [ ] Serialize every `NetInfoMutation` as canonical strict JSON inside the
  binary record. Append with `writev` retry loops and `fdatasync`; only then call
  `netinfo_txn_publish()`.
- [ ] Compact after 1,024 records or 8 MiB. Emit deterministic JSON5 ordered by
  object/property/value storage order and include schema version, next ID,
  sequence, role, tag, and parent binding.
- [ ] Run journal tests under normal execution and fault injection; expected all pass.
- [ ] Commit with `git commit -m "netinfo: persist mutations with a checksummed journal"`.

### Task 4: Wire every database procedure and replay protection

**Files:** Modify `hw/netinfo/netinfo-xdr.{c,h}`, `netinfo-server.{c,h}`, `tests/unit/test-netinfo-xdr.c`, `test-netinfo-server.c`.

- [ ] Add golden argument/result coverage for all mutation procedures and
  LISTALL(23), READALL(25), CRASHED(26), RESYNC(27), including recursive object
  lists and checksum-equal READALL behavior.
- [ ] For every mutation: decode, authenticate, begin transaction, validate,
  append+sync, publish, encode the new ID. Map exact database failures to the
  published `ni_status` values.
- [ ] Implement LISTALL from a stable snapshot. Implement READALL only for a
  master, bounded by the configured 1 MiB record; return `NI_NOTMASTER` on a
  clone. CRASHED records the peer checksum; RESYNC triggers the explicit clone
  state machine without blocking the QEMU main loop.
- [ ] Add the bounded replay cache before dispatch; cache only completed mutation
  replies and clear it on reset.
- [ ] Run XDR/server/database/journal tests; expected all procedures pass.
- [ ] Commit with `git commit -m "netinfo: complete database RPC procedures"`.

### Task 5: Complete binder lifecycle and multi-domain hierarchy

**Files:** Modify `hw/netinfo/netinfo-server.{c,h}`, `netinfo-object.c`, `netinfo-db.{c,h}`, `qapi/qom.json`, tests.

- [ ] Extend JSON5/QOM configuration with an ordered `domains` array; each entry
  has a unique tag, role (`master` or `clone`), deterministic port pair, and
  optional parent `{address,tag}`. Keep the single-domain schema accepted as
  shorthand.
- [ ] Write failing binder tests for REGISTER(1), UNREGISTER(2), CREATEMASTER(5),
  CREATECLONE(6), DESTROYDOMAIN(7), 32-registration limit, duplicate tags,
  live-port conflicts, nonempty destroy refusal, and atomic unwind.
- [ ] Implement lifecycle calls as authenticated journaled transactions.
  CREATECLONE requires a resolvable master binding; DESTROY closes registrations
  only after durable commit. GETREGISTER/LISTREG read one coherent registry snapshot.
- [ ] Implement RPARENT from configured/cached binding and use BIND+CALLIT for
  discovery. A root returns `NI_NETROOT`; failed parent contact returns
  `NI_NORESPONSE` without corrupting the cache.
- [ ] Run server/QOM tests with two domains and two isolated netdevs; expected all pass.
- [ ] Commit with `git commit -m "netinfo: add domain lifecycle and hierarchy"`.

### Task 6: Expose writable configuration safely

**Files:** Modify `hw/netinfo/netinfo-object.c`, `qapi/qom.json`, `tests/qtest/netinfo-server-object-test.c`, `docs/system/devices/netinfo.rst`.

- [ ] Add properties `writable`, `state`, `trusted-local-root`, and compaction
  thresholds. Reject `state` without writable mode only if it contains mutable
  state; reject writable mode without state.
- [ ] Add qtests for every property combination, bad permissions/symlinks,
  corrupt recovery, delete/recreate, reset, NFS coexistence, and migration
  rejection during an active writable transaction.
- [ ] Keep the migration blocker for read-only and writable instances. Document
  that snapshots are portable only after clean shutdown/compaction, not as live
  migration state.
- [ ] Run qtests and update public docs with a credential-free example.
- [ ] Commit with `git commit -m "netinfo: expose opt-in persistent writable domains"`.

### Task 7: Fuzz, crash, compatibility, and full regression gates

**Files:** Create `tests/unit/fuzz-netinfo-xdr.c`, `tests/unit/fuzz-netinfo-json5.c`; modify fuzz Meson registration and tests as defects are found.

- [ ] Add fuzz entry points that cap input and exercise ONC RPC record assembly,
  every NetInfo decoder, JSON5 loading, and journal replay without filesystem
  escape or unbounded allocation.
- [ ] Run at least 10 minutes per seed corpus under ASan/UBSan; every crash or
  leak becomes a deterministic unit regression before repair.
- [ ] Run kill-point recovery across every journal/compaction syscall and compare
  reopened database checksums with either the pre- or post-transaction state.
- [ ] Exercise `niutil`/historical clients for every read/write class and capture
  protocol traces without storing passwords.
- [ ] Run the complete QEMU unit suite, m68k qtests, NFS/9P/libslirp suites,
  `git diff --check`, and checkpatch. Verify no host listeners.
- [ ] Verify the public README and all five boot GIFs are unchanged; add only
  concise public NetInfo status/instructions with owner-approved evidence.
- [ ] Commit regressions and final documentation separately.
