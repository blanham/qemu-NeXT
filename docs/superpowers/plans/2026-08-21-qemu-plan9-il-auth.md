# QEMU Plan 9 IL Authentication Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Consume public libslirp IL through a QEMU record adapter and extend `plan9-9p1-server` with private IL/566 ticket authentication and authenticated IL/17008 service without changing default TCP/564 behavior.

**Architecture:** A fakeable QEMU IL registry keeps `SlirpState` private and mirrors the hardened callback-forward lifecycle. The 9P1 server gains explicit stream/record transport modes. A shared Plan 9 auth module implements historical encryption, ticket codecs, native key DB loading, replay protection, and a provisioning tool; QOM wires both guest-only IL listeners and BOOTP addresses.

**Tech Stack:** QEMU C/QOM/QAPI, libslirp optional API, QEMU Crypto Secret/random/cipher APIs, Meson/Ninja, GLib unit tests, QTest.

---

## Compatibility contract

The existing `transport=tcp,port=564` path remains the default and retains zero-auth `Rsession` plus unauthenticated `Tattach`. Build the IL adapter only when the dependency exposes the IL feature macro; an IL object on an older/system libslirp fails completion with a clear error, while TCP builds and tests continue to work.

The new QOM properties are `transport=tcp|il`, `il-port=17008`, `auth-port=566`, `auth-id`, `auth-domain`, `keydb`, and `key-secret`. IL requires all four authentication properties. Same numeric TCP and IL ports do not conflict because their namespaces differ.

### Task 1: Add the fakeable QEMU IL adapter contract

**Files:** Create `include/net/slirp-il.h`, `net/slirp-il-internal.h`, `net/slirp-il.c`, `tests/unit/test-slirp-il.c`; modify `net/meson.build`, `tests/unit/meson.build`.

- [ ] Write failing fake-backend tests for subnet/reserved validation, IPv4-disabled, duplicate IL tuple, independent connections, atomic send/error propagation, send-ready, callback-time listener removal, repeated close, and registry invalidation.
- [ ] Define opaque `QemuSlirpILListener` and `QemuSlirpILConnection` APIs with open/record/can-send/close callbacks. Keep backend registration behind an internal vtable.
- [ ] Implement unlink-first, callback-depth, deferred-free ownership. No adapter code may call `socket()`, bind a host FD, or expose `SlirpState`.
- [ ] Run `meson test -C build-plan9-il-auth test-slirp-il --print-errorlogs`; expected: all fake cases pass.
- [ ] Commit with `git commit -m "net: add a record-oriented SLiRP IL adapter"`.

### Task 2: Connect the adapter to public libslirp

**Files:** Modify `net/slirp.c`, `net/meson.build`, `subprojects/slirp.wrap`, `tests/unit/test-slirp-il.c`.

- [ ] Pin the reviewed full libslirp IL commit SHA in `subprojects/slirp.wrap`; configure once with the fallback and once with an ordinary non-IL system libslirp.
- [ ] Add a private IL registry to `SlirpState`. Register backend calls against public libslirp only under the feature check.
- [ ] Flush deferred callbacks after `slirp_input()` and polling; issue readiness after poll progress; invalidate QEMU listeners/connections before `slirp_cleanup()`.
- [ ] Test that TCP-only configuration builds with old libslirp and `transport=il` reports the unavailable feature rather than a link error.
- [ ] Run `test-slirp-il` and `test-slirp-guestfwd`; commit with `git commit -m "net: wire IL listeners into user networking"`.

### Task 3: Separate 9P stream and record transports

**Files:** Modify `hw/9pfs/plan9-9p1-server.[ch]`, `tests/unit/test-plan9-9p1-server.c`, `tests/unit/plan9-9p1-slirp-stub.c`.

- [ ] Add failing tests named `/record-exact-frame`, `/record-rejects-empty`, `/record-rejects-partial`, `/record-rejects-trailing`, `/record-atomic-reply`, `/record-backpressure`, and `/record-close-reset`.
- [ ] Add `PLAN9P1_TRANSPORT_STREAM` and `PLAN9P1_TRANSPORT_RECORD` to the transport ops. Stream ingress continues through `plan9p1_stream_feed()` and partial reply delivery unchanged.
- [ ] Implement a record ingress entry that decodes exactly one full request; malformed or multiple frames fail the transport. In record mode, send the entire encoded reply or retain all of it on `-EAGAIN`; a short success is fatal.
- [ ] Reset fids, challenges, replay, request/reply queues, and framing state on record connection close before accepting another.
- [ ] Run codec/server unit suites and commit with `git commit -m "9pfs: add atomic record transport to 9P1"`.

### Task 4: Implement historical Plan 9 auth primitives

**Files:** Create `hw/9pfs/plan9-auth.[ch]`, `tests/unit/test-plan9-auth.c`; modify `hw/9pfs/meson.build`, `tests/unit/meson.build`.

- [ ] Add failing immutable golden tests for `passtokey`, packed 7-to-8 byte DES expansion, variable-length Plan 9 encrypt/decrypt, little-endian ticket request/ticket/authenticator codecs, and exact sizes 141/72/13.
- [ ] Transcribe behavior from the staged Second Edition sources, not modern Plan 9. Use QEMU's DES primitive only beneath the historical overlapping-block algorithm.
- [ ] Bound fixed strings to `NAMELEN=28` and `DOMLEN=48`; reject unrepresentable helper passwords instead of truncating silently.
- [ ] Explicitly wipe password, expanded keys, plaintext tickets, and conversation keys on every exit path.
- [ ] Run `test-plan9-auth`; commit with `git commit -m "9pfs: add Plan 9 ticket cryptography"`.

### Task 5: Load and validate native encrypted `/adm/keys`

**Files:** Modify `hw/9pfs/plan9-auth.[ch]`, `tests/unit/test-plan9-auth.c`.

- [ ] Add failing tests for exact independently encrypted 41-byte records and rejection of partial, duplicate, invalid-status, disabled/expired server, symlink, empty, changed-during-read, and oversized databases.
- [ ] Implement read-only no-follow loading of `name[28],key[7],status,warnings,expiry_le32`; require complete records and immutable in-memory copies.
- [ ] Substitute lookup misses only in ticket service. Startup itself must fail when `auth-id` is missing, disabled, or expired.
- [ ] Run focused keydb tests under ASan; commit with `git commit -m "9pfs: load native encrypted Plan 9 keys"`.

### Task 6: Add the local provisioning tool

**Files:** Create `tools/qemu-plan9-keydb.c`; modify `tools/meson.build`, `hw/9pfs/plan9-auth.[ch]`, `tests/unit/test-plan9-auth.c`.

- [ ] Add failing tests for tty-only password input, password confirmation/mismatch, two-record output (`tor`, server identity), modes, collision refusal, atomic rename, and base64 Secret decoding to exactly seven bytes.
- [ ] Implement `qemu-plan9-keydb create --keydb PATH --secret PATH --server-id p9fs`. Generate independent master/server keys with QEMU crypto random; prompt for tor's password without echo.
- [ ] Write temporary files in the destination directories with mode 0600, fsync contents and containing directory, then rename without overwriting an existing target. Output no secret bytes to stdout/stderr.
- [ ] Share auth encryption/keydb encoding code with the server; wipe all temporary secrets.
- [ ] Run focused tests and commit with `git commit -m "tools: provision native Plan 9 key databases"`.

### Task 7: Implement IL/566 ticket service

**Files:** Modify `hw/9pfs/plan9-auth.[ch]`, `hw/9pfs/plan9-9p1-server.[ch]`, `tests/unit/test-plan9-auth.c`, `tests/unit/test-plan9-9p1-server.c`.

- [ ] Add failing tests for one 141-byte `AuthTreq` record, one 145-byte `AuthOK+2 tickets` reply, challenge/key agreement, `hostid==uid`, speaks-for rejection to `none`, malformed request, privacy substitution, backpressure, and close-after-reply.
- [ ] Implement independent short-lived auth connections. Generate a fresh seven-byte conversation key and encrypt client/server tickets with the appropriate principal/server keys.
- [ ] On missing/disabled/expired client principals, use a fresh random substitute key and normal-size response. Never disclose which lookup failed over the wire.
- [ ] Logs may contain only failure class and peer address; ban principals and all decrypted material from tracepoints.
- [ ] Run auth/server tests; commit with `git commit -m "9pfs: serve Plan 9 authentication tickets over IL"`.

### Task 8: Authenticate `Tsession` and `Tattach`

**Files:** Modify `hw/9pfs/plan9-9p1-server.[ch]`, `tests/unit/test-plan9-9p1-server.c`.

- [ ] Add failing tests for IL `Rsession` challenge/auth-id/domain, valid attach, wrong server ticket, wrong server challenge, wrong uname, replay ID, stale ID outside the 32-ID window, reconnect reset, and `AuthAs` client challenge/ID.
- [ ] Implement the historical 32-ID sliding replay bitmap. `Tsession` resets connection state, stores client challenge, and creates a server challenge.
- [ ] Decode `AuthTs` with the server key, then `AuthAc` with the conversation key; validate type, server challenge, replay, and attach user equals ticket client. Return `Rattach` with encrypted `AuthAs`.
- [ ] Keep TCP `Rsession` zero and TCP attach unauthenticated. IL rejects attach before a successful session and clears state on every connection failure.
- [ ] Run all 9P1 unit tests; commit with `git commit -m "9pfs: authenticate IL 9P1 sessions"`.

### Task 9: Wire QOM, Secret, IL listeners, and BOOTP

**Files:** Modify `hw/9pfs/plan9-9p1-server.[ch]`, `qapi/qom.json`, `tests/qtest/plan9-9p1-object-test.c`, `tests/unit/plan9-9p1-slirp-stub.c`, `docs/system/devices/9p.rst`.

- [ ] Add failing qtests for TCP defaults/recreate, IL defaults, incomplete auth config, invalid names/ports, unavailable IL ABI, wrong Secret length, invalid keydb, listener collision, completion unwind, one active file connection, and BOOTP file/auth address.
- [ ] Resolve `key-secret` with `qcrypto_secret_lookup()`, require returned length exactly seven, copy it, wipe the allocated lookup buffer, and load the DB. Never use string functions on Secret bytes.
- [ ] Register auth port 566 and file port 17008 only inside the named user netdev. Set both Plan 9 BOOTP file and authentication addresses to `guest-address`; TCP leaves auth zero.
- [ ] Unwind in reverse order and invalidate listeners before freeing auth/server state. Explicitly reject live migration with an IL connection and require guest reconnect after reset.
- [ ] Run unit/qtests and commit with `git commit -m "9pfs: expose authenticated 9P1 over private IL"`.

### Task 10: Documentation and QEMU regression gate

**Files:** Modify `QEMU-NeXT-README.md`, `docs/system/devices/9p.rst`; create local untracked `TODO.md`; modify common `.git/info/exclude` only as local state.

- [ ] Document TCP compatibility launch, provisioning, QEMU Secret file launch, IL ports, private-network boundary, and `https://ftp.osuosl.org/pub/plan9/history/`.
- [ ] Add `/TODO.md` to QEMU's common `.git/info/exclude`, verify `git check-ignore -v TODO.md`, then create a local-only hardware follow-up file. Confirm `git status --short` never lists it.
- [ ] Build targets `qemu-system-m68k`, `qemu-plan9-keydb`, all new unit tests, and the object qtest.
- [ ] Run all focused suites plus existing `test-slirp-guestfwd`, codec/server/local server, and m68k Plan 9 object tests.
- [ ] Search tracked source and test output for passwords, master keys, tickets, and secret payloads; inspect QMP query output and argv fixtures.
- [ ] Commit tracked docs with `git commit -m "docs: describe authenticated Plan 9 IL service"`; do not add `TODO.md` or `.git/info/exclude`.
