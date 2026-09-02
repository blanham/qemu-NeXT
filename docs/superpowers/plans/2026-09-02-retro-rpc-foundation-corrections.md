# Retro RPC Foundation Corrections Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Correct TCP-service reclamation, bounded TCP RPC output, per-netdev portmapper ownership, and fallback libslirp integration before the RPC foundation is handed off.

**Architecture:** libslirp reclaims retired opaque TCP handles only at safe outer dispatch boundaries. QEMU keeps libslirp's atomic stream ABI, but serializes RPC reply records in a bounded per-connection FIFO and submits only chunks that fit current send space. The first RPC registration owns both portmapper transports until the last unregister, and QEMU's fallback pin is updated only after the corrected libslirp commit is reachable from its configured remote.

**Tech Stack:** C, GLib, libslirp TCP/UDP guest services, QEMU SLiRP adapters, ONC RPC/XDR, Meson, qtest.

---

## Repository and file map

libslirp worktree: `/home/blanham/projects/NeXT/lab/.worktrees/netinfo-libslirp`

- `src/tcp-service.c`: opaque TCP listener/connection ownership and deferred reclamation.
- `src/tcp-service.h`: internal reclamation seam.
- `src/libslirp.h`: public handle-lifetime contract.
- `test/tcpservicetest.c`: churn and reentrant-lifetime regressions.

QEMU worktree: `/home/blanham/projects/NeXT/lab/.worktrees/netinfo-server-qemu`

- `net/slirp-rpc.c`: TCP reply queue, PMAP endpoint ownership, and dispatch routing.
- `tests/unit/test-onc-rpc.c`: fake-adapter queue/PMAP regressions.
- `tests/unit/test-slirp-il-integration.c`: real-libslirp large-reply integration.
- `meson.build`: internal fallback capability flags.
- `subprojects/slirp.wrap`: published immutable libslirp pin.
- `tests/qtest/nfs-server-object-test.c`: force-fallback shared PMAP regression gate.

### Task 1: Reclaim retired libslirp TCP service handles

**Files:**
- Modify: `/home/blanham/projects/NeXT/lab/.worktrees/netinfo-libslirp/src/tcp-service.c`
- Modify: `/home/blanham/projects/NeXT/lab/.worktrees/netinfo-libslirp/src/tcp-service.h`
- Modify: `/home/blanham/projects/NeXT/lab/.worktrees/netinfo-libslirp/src/libslirp.h`
- Modify: `/home/blanham/projects/NeXT/lab/.worktrees/netinfo-libslirp/test/tcpservicetest.c`

- [ ] **Step 1: Write failing sequential-churn and safe-lifetime tests**

Add a churn helper that creates, closes, and fully acknowledges at least
`TEST_CONNECTION_LIMIT * 4` connections one at a time.  Do not retain closed
handles in `state.records`; reuse one record slot after each close.  After each
outer packet/public-call boundary assert:

```c
assert(state.slirp->tcp_service_dead_connections == NULL);
assert(state.slirp->tcp_service_connection_count == 0);
```

Add listener churn for `TCP_SERVICE_MAX_LISTENERS * 4` sequential
listen/remove cycles and assert `tcp_service_dead_listeners == NULL` after each
remove.  Preserve reentrant coverage by asserting a second close/remove made
from the same callback is harmless.  Remove existing post-return uses of a
closed connection or removed listener; those handles become invalid when the
outermost dispatch returns.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
meson test -C /tmp/netinfo-task4a-libslirp-build.einKBj tcp-service --print-errorlogs
```

Expected: FAIL because the retired connection/listener lists remain populated
until `slirp_cleanup()`.

- [ ] **Step 3: Document the public lifetime and add a safe drain**

Document above the public TCP APIs in `src/libslirp.h`:

```c
/*
 * Listener and connection handles remain valid through the public operation
 * and any callbacks it invokes.  A removed listener or closed connection is
 * invalid after the outermost libslirp dispatch returns and must not be reused.
 */
```

Replace cleanup-only freeing with a drain that frees dead connections before
dead listeners, and only at zero callback and dispatch depth:

```c
static void tcp_service_drain_retired(Slirp *slirp)
{
    SlirpTcpConnection **connectionp;
    SlirpTcpListener **listenerp;

    if (slirp->tcp_service_callback_depth ||
        slirp->tcp_service_dispatch_depth) {
        return;
    }
    connectionp = &slirp->tcp_service_dead_connections;
    while (*connectionp) {
        SlirpTcpConnection *connection = *connectionp;

        if (connection->refs != 1 || connection->callback_depth) {
            connectionp = &connection->dead_next;
            continue;
        }
        *connectionp = connection->dead_next;
        g_free(connection);
    }
    listenerp = &slirp->tcp_service_dead_listeners;
    while (*listenerp) {
        SlirpTcpListener *listener = *listenerp;

        if (listener->refs != 1 || listener->callback_depth) {
            listenerp = &listener->dead_next;
            continue;
        }
        *listenerp = listener->dead_next;
        g_free(listener);
    }
}
```

Call the drain from the non-cleanup path of the outermost
`tcp_service_dispatch_leave()`.  Keep `tcp_service_free_deferred()` for final
stack destruction, but make it assert that no callback is active and use the
same connection-before-listener ordering.  Never drain from
`tcp_service_callback_leave()` because callback dispatchers still release
temporary refs afterward.

- [ ] **Step 4: Run focused, sanitizer, and complete libslirp tests**

Run:

```bash
meson test -C /tmp/netinfo-task4a-libslirp-build.einKBj tcp-service --print-errorlogs
meson test -C /tmp/netinfo-task4a-libslirp-build.einKBj --print-errorlogs
```

Expected: focused TCP service and complete 7-test suite PASS.  Run the focused
test under Valgrind or the existing sanitizer build and require no invalid
access or definite leak.

- [ ] **Step 5: Commit the libslirp correction**

```bash
git add src/tcp-service.c src/tcp-service.h src/libslirp.h test/tcpservicetest.c
git commit -m "tcp: reclaim retired guest service handles"
```

### Task 2: Serialize and bound QEMU TCP RPC replies

**Files:**
- Modify: `net/slirp-rpc.c`
- Modify: `tests/unit/test-onc-rpc.c`
- Modify: `tests/unit/test-slirp-il-integration.c`

- [ ] **Step 1: Write failing fake-stream queue tests**

Extend the fake stream backend in `tests/unit/test-onc-rpc.c` so tests can set
available send space and capture each accepted byte chunk.  Add cases that:

1. reply with `128 * 1024 + 4096` payload bytes while initial space is 128 KiB;
2. restore space repeatedly and verify one record marker plus the exact payload;
3. queue two replies and verify their records never interleave;
4. exceed `ONC_RPC_MAX_TCP_RECORD + 4` aggregate unsent bytes and verify the
   stream closes and retained requests are released;
5. close/invalidate the registry with a partially sent record and verify one
   terminal transition and no retained request.

The large-reply dispatch double must call `onc_rpc_request_reply()` with a
deterministic byte pattern so the reconstructed output can be compared exactly.

- [ ] **Step 2: Run the ONC RPC tests and verify RED**

```bash
meson test -C /tmp/netinfo-task4a-qemu-build3.Xe5Nyc test-onc-rpc --print-errorlogs
```

Expected: FAIL because a record larger than current send space is left pending
without progress, and independent request objects do not enforce FIFO or an
aggregate bound.

- [ ] **Step 3: Move pending output from requests to a connection FIFO**

In `net/slirp-rpc.c`, replace `OncRpcRequest.pending_reply*` with:

```c
typedef struct QemuSlirpRpcTcpReply {
    QTAILQ_ENTRY(QemuSlirpRpcTcpReply) entry;
    OncRpcRequest *request;
    uint8_t *data;
    size_t length;
    size_t offset;
} QemuSlirpRpcTcpReply;

struct QemuSlirpRpcTcpConnection {
    /* existing fields */
    QTAILQ_HEAD(, QemuSlirpRpcTcpReply) replies;
    size_t queued_bytes;
    bool flushing;
};
```

Initialize the queue on accept.  Enqueue one fully encoded record per reply,
retain its request, and reject/close when this expression is false:

```c
record_length <= ONC_RPC_MAX_TCP_RECORD + 4 &&
connection->queued_bytes <=
    (ONC_RPC_MAX_TCP_RECORD + 4) - record_length
```

Use `queued_bytes` for unsent bytes, decrementing it after each accepted chunk.
Only mark a request terminal and release the queue ref after its entire record
has been accepted.

- [ ] **Step 4: Implement non-interleaving partial progress**

Replace the global request-list retry loop with a head-only connection flush:

```c
static bool rpc_tcp_flush(QemuSlirpRpcTcpConnection *connection)
{
    if (connection->flushing) {
        return true;
    }
    connection->flushing = true;
    while (!connection->closed && !QTAILQ_EMPTY(&connection->replies)) {
        QemuSlirpRpcTcpReply *reply = QTAILQ_FIRST(&connection->replies);
        size_t space = qemu_slirp_stream_can_send(connection->stream);
        size_t chunk;
        int ret;

        if (!space) {
            break;
        }
        chunk = MIN(space, reply->length - reply->offset);
        ret = qemu_slirp_stream_send(connection->stream,
                                     reply->data + reply->offset, chunk);
        if (ret == -EAGAIN) {
            break;
        }
        if (ret != 0) {
            connection->flushing = false;
            return false;
        }
        reply->offset += chunk;
        connection->queued_bytes -= chunk;
        if (reply->offset == reply->length) {
            QTAILQ_REMOVE(&connection->replies, reply, entry);
            rpc_request_terminal(reply->request);
            onc_rpc_request_unref(reply->request);
            g_free(reply->data);
            g_free(reply);
        }
    }
    connection->flushing = false;
    return !connection->closed;
}
```

Hold a connection ref across the flush because `qemu_slirp_stream_send()` may
reenter close callbacks.  On connection close, detach the FIFO first, mark each
request terminal, release exactly one retained ref, and free each buffer.  Have
`rpc_tcp_can_send()` flush only its connection and close the stream on a fatal
result.

- [ ] **Step 5: Add a real-libslirp large-reply integration test**

In `tests/unit/test-slirp-il-integration.c`, register a TCP RPC program whose
handler returns more than 128 KiB, drive guest ACKs until all segments arrive,
reassemble the stream, and verify the record marker, XID, accepted-success
envelope, payload pattern, and absence of a second/interleaved marker.

- [ ] **Step 6: Run focused QEMU tests and commit**

```bash
meson test -C /tmp/netinfo-task4a-qemu-build3.Xe5Nyc --print-errorlogs \
  test-onc-rpc test-slirp-stream test-slirp-il-integration
git diff --check
git add net/slirp-rpc.c tests/unit/test-onc-rpc.c \
  tests/unit/test-slirp-il-integration.c
git commit -m "net: bound queued TCP RPC replies"
```

Expected: all focused tests PASS and diff check is clean.

### Task 3: Make portmapper ownership per netdev

**Files:**
- Modify: `net/slirp-rpc.c`
- Modify: `tests/unit/test-onc-rpc.c`
- Modify: `tests/unit/test-slirp-il-integration.c`

- [ ] **Step 1: Write failing ownership, discovery, rollback, and routing tests**

Add tests for these exact cases:

- a UDP-only first program creates both UDP/111 and TCP/111;
- a TCP-only first program is discoverable through UDP GETPORT;
- removing one of multiple registrations retains both portmapper endpoints;
- removing the final registration releases both endpoints;
- failure to acquire the second portmapper endpoint leaves neither endpoint nor
  service endpoint registered;
- a PMAP NULL/GETPORT call sent to a non-111 registered service port receives
  `PROG_UNAVAIL`, not a PMAP reply.

- [ ] **Step 2: Run the ONC RPC test and verify RED**

```bash
meson test -C /tmp/netinfo-task4a-qemu-build3.Xe5Nyc test-onc-rpc --print-errorlogs
```

Expected: FAIL because port 111 is acquired/released per transport and PMAP
dispatch currently checks only program number.

- [ ] **Step 3: Acquire and release the portmapper as one registry resource**

Replace transport-relative acquisition with a transactional helper:

```c
static int rpc_portmap_acquire(QemuSlirpRpcRegistry *registry, Error **errp)
{
    if (registry->udp_portmap || registry->tcp_portmap) {
        g_assert(registry->udp_portmap && registry->tcp_portmap);
        return 0;
    }
    registry->udp_portmap = rpc_endpoint_new(
        registry, ONC_RPC_PORTMAP_PORT, ONC_RPC_TRANSPORT_UDP,
        QEMU_SLIRP_UDP_LISTEN_BROADCAST, errp);
    if (!registry->udp_portmap) {
        return -1;
    }
    registry->tcp_portmap = rpc_endpoint_new(
        registry, ONC_RPC_PORTMAP_PORT, ONC_RPC_TRANSPORT_TCP,
        QEMU_SLIRP_UDP_LISTEN_DEFAULT, errp);
    if (!registry->tcp_portmap) {
        QemuSlirpRpcEndpoint *udp = registry->udp_portmap;

        registry->udp_portmap = NULL;
        rpc_endpoint_release(udp);
        return -1;
    }
    return 0;
}
```

Call it once for every first registration, before linking the registration.
Release both endpoints only when `QTAILQ_EMPTY(&registry->registrations)` after
an unregister or failed registration rollback.  Do not consult the registered
program's transport mask when deciding portmapper lifetime.

- [ ] **Step 4: Restrict PMAP dispatch to endpoint port 111**

Change the request dispatch condition to:

```c
if (request->call.program == ONC_RPC_PORTMAP_PROGRAM &&
    request->tcp_connection ?
        request->tcp_connection->port == ONC_RPC_PORTMAP_PORT :
        request->udp_listener == registry->udp_portmap->udp_listener) {
    /* existing version check and rpc_dispatch_portmap() */
}
```

Factor the endpoint check into a parenthesized helper to avoid conditional
operator precedence ambiguity.  A program-100000 call on any other endpoint
continues through normal exact/range lookup and produces `PROG_UNAVAIL`.

- [ ] **Step 5: Run focused and real-SLiRP tests, then commit**

```bash
meson test -C /tmp/netinfo-task4a-qemu-build3.Xe5Nyc --print-errorlogs \
  test-onc-rpc test-slirp-il-integration test-nfs2-server \
  'qtest-m68k/nfs-server-object-test'
git diff --check
git add net/slirp-rpc.c tests/unit/test-onc-rpc.c \
  tests/unit/test-slirp-il-integration.c
git commit -m "net: make RPC portmapper ownership per netdev"
```

Expected: all focused tests PASS, including cross-transport discovery and the
shared NFS PMAP DUMP.

### Task 4: Publish and pin the corrected libslirp fallback

**Files:**
- Modify: `subprojects/slirp.wrap`
- Modify: `meson.build`

- [ ] **Step 1: Verify both repositories are clean and capture the libslirp commit**

```bash
git -C /home/blanham/projects/NeXT/lab/.worktrees/netinfo-libslirp status --short
git -C /home/blanham/projects/NeXT/lab/.worktrees/netinfo-libslirp rev-parse HEAD
git -C /home/blanham/projects/NeXT/lab/.worktrees/netinfo-server-qemu status --short
```

Expected: both status outputs are empty.  Record the full 40-character
libslirp HEAD as `published_libslirp_commit` for the following exact checks.

- [ ] **Step 2: Obtain explicit user authorization, publish, and verify reachability**

Do not push without explicit user authorization.  After approval:

```bash
git -C /home/blanham/projects/NeXT/lab/.worktrees/netinfo-libslirp \
  push -u origin feature/netinfo-services
git -C /home/blanham/projects/NeXT/lab/.worktrees/netinfo-libslirp \
  ls-remote origin refs/heads/feature/netinfo-services
```

Expected: the remote branch hash exactly equals `published_libslirp_commit`.
Stop without changing the wrap if it does not.

- [ ] **Step 3: Pin the reachable commit and enable internal capabilities**

Set `subprojects/slirp.wrap` `revision` to the verified full hash.  In the
`slirp.type_name() == 'internal'` branch of `meson.build`, set:

```meson
slirp_udp_listen_full = true
slirp_tcp_service = true
```

Update the adjacent comment to state that the pinned fallback exports IL,
Plan 9 BOOTP, UDP/full-listen, TCP guest service, and BOOTP root APIs.

- [ ] **Step 4: Configure a fresh force-fallback build and verify features**

```bash
meson setup /tmp/netinfo-rpc-fallback-qemu \
  -Dwrap_mode=forcefallback -Dslirp=enabled -Ddefault_targets=m68k-softmmu
rg 'CONFIG_SLIRP_(UDP_LISTEN_FULL|TCP_SERVICE)' \
  /tmp/netinfo-rpc-fallback-qemu/config-host.mak
meson test -C /tmp/netinfo-rpc-fallback-qemu --print-errorlogs \
  test-onc-rpc test-slirp-stream test-slirp-il-integration \
  test-nfs2-server 'qtest-m68k/nfs-server-object-test'
```

Expected: both feature macros are `y`; every focused test passes and the m68k
object qtest reports 9 subtests.

- [ ] **Step 5: Re-run the external-libslirp focused matrix and commit**

```bash
meson test -C /tmp/netinfo-task4a-qemu-build3.Xe5Nyc --print-errorlogs \
  test-onc-rpc test-slirp-stream test-slirp-il-integration \
  test-nfs2-server 'qtest-m68k/nfs-server-object-test'
git diff --check
git add meson.build subprojects/slirp.wrap
git commit -m "build: pin RPC-capable libslirp fallback"
```

Expected: external and fallback matrices both pass.

### Task 5: Foundation correction verification

**Files:**
- Modify: none unless a verification-only documentation correction is proven necessary.

- [ ] **Step 1: Run complete suites**

```bash
meson test -C /tmp/netinfo-task4a-libslirp-build.einKBj --print-errorlogs
meson test -C /tmp/netinfo-task4a-qemu-build3.Xe5Nyc --suite unit --print-errorlogs
meson test -C /tmp/netinfo-task4a-qemu-build3.Xe5Nyc --print-errorlogs \
  'qtest-m68k/nfs-server-object-test'
```

Expected: libslirp 7/7, QEMU unit suite with zero failures, and m68k NFS object
qtest 9/9.

- [ ] **Step 2: Run static checks on each correction commit**

```bash
git diff --check
scripts/checkpatch.pl --git HEAD~3..HEAD
```

Expected: no errors.  Investigate every warning; do not dismiss a new warning
as one of the pre-existing MAINTAINERS warnings without comparing commit output.

- [ ] **Step 3: Repeat live no-host-socket verification**

Launch the NFS object on `-netdev user`, keep it alive through QMP, resolve its
PID, and inspect both `/proc/$pid/fd` and `/proc/$pid/net/{tcp,tcp6,udp,udp6}`.
Expected: no host TCP/UDP socket attributable to embedded services and no local
ports 111, 635, or 2049; only the configured QMP Unix socket and expected fsdev
directory FD are added.

- [ ] **Step 4: Verify protected artifacts and clean worktrees**

```bash
git diff 04f068101760d59aeb6c50fd7764649bae00516d..HEAD -- \
  QEMU-NeXT-README.md docs/boot/netbsd-full.gif \
  docs/boot/netbsd-network.gif docs/boot/nextstep.gif \
  docs/boot/plan9-il.gif docs/boot/plan9-tcp.gif
git status --short
git -C /home/blanham/projects/NeXT/lab/.worktrees/netinfo-libslirp status --short
```

Expected: protected diff and both status outputs are empty.

- [ ] **Step 5: Request final cross-repository review**

Give the reviewer the libslirp base `50402cbd57f28e2fd6523dd4fc07c654a06efdf6`,
the corrected libslirp HEAD, QEMU base `04f068101760d59aeb6c50fd7764649bae00516d`,
QEMU HEAD, this plan, and the correction design.  Require explicit review of
retired-handle reclamation, real large-reply progress/queue bounds, both
portmapper transports, PMAP endpoint routing, fallback feature macros, and
protected/no-host-socket evidence.  The foundation is complete only after the
review returns APPROVED with no Critical or Important findings.
