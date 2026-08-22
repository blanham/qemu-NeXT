# Public libslirp IL Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add historically compatible, reliable, record-oriented Plan 9 IL over IPv4 protocol 40 to the public libslirp fork, with a bounded listener API for QEMU.

**Architecture:** `src/il.c` owns all packets, state, queues, sequence arithmetic, retransmission, and one stack-wide deadline timer. IPv4 input dispatches reassembled protocol-40 datagrams into IL. Public opaque listeners and connections deliver complete records and accept only atomic sends; removal is callback-safe and opens no host socket.

**Tech Stack:** C11, libslirp mbuf/IP output, Meson, GLib-free assertion test harness, deterministic packet injection and synthetic monotonic time.

---

## File map

- Create `src/il.h`: private packet/state/queue declarations and IPv4 input/timer hooks.
- Create `src/il.c`: checksum, codec, listener registry, connection state machine, reliability, bounds, timers, and cleanup.
- Modify `src/ip_input.c`: dispatch IPv4 protocol 40 after normal fragment reassembly.
- Modify `src/slirp.h` and `src/slirp.c`: own IL state, initialize/clean it, and route its single timer.
- Modify `src/libslirp.h`: publish opaque IL types, callbacks, result constants, and APIs.
- Modify `src/libslirp.map`: export the IL symbols.
- Modify `meson.build`: compile IL and register `iltest`.
- Create `test/iltest.c`: deterministic packet, clock, output, callback, and reentrancy coverage.

Use this public shape unless a failing ABI test demonstrates a necessary adjustment:

```c
typedef struct SlirpILListener SlirpILListener;
typedef struct SlirpILConnection SlirpILConnection;

typedef struct SlirpILCallbacks {
    void *(*connected)(SlirpILConnection *connection, void *listener_opaque);
    void (*record)(SlirpILConnection *connection, const uint8_t *data,
                   size_t len, void *connection_opaque);
    void (*can_send)(SlirpILConnection *connection, void *connection_opaque);
    void (*closed)(SlirpILConnection *connection, void *connection_opaque);
} SlirpILCallbacks;

SlirpILListener *slirp_il_listen(Slirp *slirp, struct in_addr address,
                                 uint16_t port,
                                 const SlirpILCallbacks *callbacks,
                                 void *opaque);
void slirp_il_listener_remove(SlirpILListener *listener);
int slirp_il_send_record(SlirpILConnection *connection,
                         const uint8_t *data, size_t len);
void slirp_il_connection_close(SlirpILConnection *connection);
```

Return zero on accepted atomic send and negative `errno` values such as `-EAGAIN`, `-ENOTCONN`, and `-EMSGSIZE`. Never return a short count.

### Task 1: Establish deterministic wire primitives

**Files:** Create `src/il.[ch]`, create `test/iltest.c`, modify `meson.build`.

- [ ] Add failing checksum and 18-byte header tests using fixed golden packets, odd payloads, bad specifier, short/overlong length, and checksum corruption.
- [ ] Run `meson setup build && meson test -C build il --print-errorlogs`; expected: compile fails because IL primitives are absent.
- [ ] Implement the Internet checksum and strict header decode/encode. Fields are big-endian; checksum and specifier are zero during calculation; total length includes the header.
- [ ] Re-run the focused test; expected: all wire primitive cases pass.
- [ ] Commit only these files with `git commit -m "il: add packet codec and checksum"`.

### Task 2: Publish bounded listener lifecycle

**Files:** Modify `src/il.[ch]`, `src/libslirp.h`, `src/libslirp.map`, `src/slirp.[ch]`, `meson.build`, `test/iltest.c`.

- [ ] Add failing tests for bind, duplicate tuple rejection, separate ports, listener removal, removal during `connected`, and stack cleanup. Assert no host FD is created.
- [ ] Implement opaque listener/connection ownership with unlink-before-callback and callback-depth deferred free. Copy the callback table; do not retain caller-owned stack storage.
- [ ] Add public symbols and an IL feature/version macro. Increment libslirp's additive ABI metadata according to the existing Meson version convention.
- [ ] Run `meson test -C build il guestfwd --print-errorlogs`; expected: all pass, including reentrant removal.
- [ ] Commit with `git commit -m "il: publish listener and record API"`.

### Task 3: Accept the historical three-way handshake

**Files:** Modify `src/ip_input.c`, `src/il.[ch]`, `test/iltest.c`.

- [ ] Add failing injection tests for sync, sync acknowledgement, state completion, duplicate sync, wrong address/port, wrong ACK, and no listener.
- [ ] Dispatch protocol 40 from `ip_input.c` only after IPv4 validation/reassembly. Implement passive handshake states from `stil.c`; initial sequence IDs are random 24-bit values.
- [ ] Capture outgoing mbufs and assert exact IPv4 protocol, tuple reversal, type, sequence, ACK, length, and checksum.
- [ ] Run `meson test -C build il --print-errorlogs`; expected: handshake cases pass without invoking `record` early.
- [ ] Commit with `git commit -m "il: implement passive connection handshake"`.

### Task 4: Deliver ordered atomic records

**Files:** Modify `src/il.[ch]`, `test/iltest.c`.

- [ ] Add failing tests for one packet/one record, consecutive records, duplicate data, reorder within and beyond the 20-record window, wrap-safe sequence comparison, and callback isolation across two connections.
- [ ] Implement a bounded receive window of 20 records, cumulative ACK state, strictly ordered callback delivery, and duplicate/out-of-window discard. Preserve each record allocation and length until its single callback.
- [ ] Add explicit maximum record and aggregate receive-memory bounds; assert over-limit input does not allocate or callback.
- [ ] Run the focused test and an ASan build when available.
- [ ] Commit with `git commit -m "il: preserve ordered record delivery"`.

### Task 5: Add atomic sends, acknowledgements, and backpressure

**Files:** Modify `src/il.[ch]`, `test/iltest.c`.

- [ ] Add failing tests for send before established, one record per data packet, ACK release, cumulative ACK, queue full `-EAGAIN`, oversized `-EMSGSIZE`, `can_send` edge notification, and callback-time close.
- [ ] Implement a bounded 20-record transmit window and bounded pending bytes. Copy an accepted record exactly once; reject it wholly otherwise. Free cumulatively acknowledged records and emit `can_send` only after capacity transitions from unavailable to available.
- [ ] Piggyback ACKs where possible and schedule a standalone ACK after 200 ms.
- [ ] Run `meson test -C build il --print-errorlogs`; expected: no partial acceptance and no duplicate readiness callback.
- [ ] Commit with `git commit -m "il: add reliable atomic record sends"`.

### Task 6: Implement loss recovery and historical timers

**Files:** Modify `src/il.[ch]`, `src/slirp.c`, `src/libslirp.h`, `test/iltest.c`.

- [ ] Add failing synthetic-clock tests for 200 ms delayed ACK, 400 ms first retry and historical backoff, dataquery/state go-back-N recovery, 35 s no-progress failure, 60 s idle query, 6 s probes, and unanswered-query closure.
- [ ] Add one `SLIRP_TIMER_IL` stack timer. Each state mutation recomputes the earliest deadline and arms/modifies/frees only that timer through existing callbacks.
- [ ] Implement dataquery/state and query/state exactly as `stil.c`, including cumulative retransmit and probe budgets. A backward synthetic clock must not underflow deadlines.
- [ ] Run `meson test -C build il ping --print-errorlogs`; expected: exact boundary timestamps pass and existing timer tests stay green.
- [ ] Commit with `git commit -m "il: add recovery and compatibility timers"`.

### Task 7: Close, teardown, and resource hardening

**Files:** Modify `src/il.[ch]`, `src/slirp.c`, `test/iltest.c`.

- [ ] Add failing tests for local close retransmit, peer close, simultaneous close, listener removal with live connections, repeated close/remove, malformed packet floods, connection-table exhaustion, and `slirp_cleanup()` from no callback.
- [ ] Implement close states and retransmission. Enforce fixed limits for listeners, connections, per-connection records, and total record bytes. Refuse new connections deterministically after limits.
- [ ] Ensure every public callback may remove its listener or close its connection without use-after-free, double callback, or post-removal readiness.
- [ ] Run the focused test under ASan/UBSan and Valgrind if locally available.
- [ ] Commit with `git commit -m "il: harden close and teardown lifecycle"`.

### Task 8: Migration refusal, regression, API documentation, and publication

**Files:** Modify `src/state.c`, `src/libslirp.h`, `CHANGELOG.md`, `meson.build`; no wire-protocol behavior change.

- [ ] Add a failing state test proving save/load returns a documented unsupported error while an IL listener or connection is live; do not serialize partial IL state. Prove normal state save/load remains unchanged when IL is inactive.
- [ ] Implement the explicit live-IL state refusal in `src/state.c` without changing the serialized format version.
- [ ] Document address/port byte order, callback lifetime, record ownership, return codes, reentrancy, limits, timer context, and IPv4-only status in the public header.
- [ ] Run `ninja -C build`, `meson test -C build --print-errorlogs`, `git diff --check`, and inspect `nm -D build/src/libslirp.so | rg 'slirp_il_'`.
- [ ] Confirm BOOTP, guestfwd, TCP, UDP, ICMP, NCSI, and ping tests remain green.
- [ ] Commit with `git commit -m "docs: describe the public IL API"`.
- [ ] Push `feature/plan9-il`, verify the remote full SHA with `git ls-remote`, and pass that immutable SHA to the QEMU plan. Never pin a branch name.
