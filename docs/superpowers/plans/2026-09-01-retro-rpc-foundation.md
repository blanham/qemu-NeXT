# Retro RPC Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Provide broadcast UDP and accepted-connection TCP guest services, a shared per-SLiRP ONC RPC/portmapper registry, and move embedded NFS onto it without guest-visible regressions.

**Architecture:** The libslirp fork gains bounded in-stack UDP-broadcast and TCP-listener APIs that never create host sockets. QEMU wraps those APIs in lifecycle-safe named-netdev adapters, then owns one ONC RPC registry and portmapper v2 instance per user netdev. NFS registers MOUNT and NFS programs with that registry instead of owning UDP/111.

**Tech Stack:** C11, GLib, libslirp IPv4/TCP/UDP, QEMU QOM, ONC RPC v2, XDR, Meson, QTest.

---

## Fixed contracts

- Preserve `slirp_udp_listen()` ABI and add an opt-in broadcast flag API.
- TCP services bind only inside a `Slirp` stack, support concurrent connections,
  expose peer identity, apply explicit receive/send/connection limits, and create
  no host file descriptor.
- Portmapper program `100000`, version `2`, owns UDP/TCP port `111` once per
  netdev and supports NULL, GETPORT, DUMP, and UDP CALLIT. SET and UNSET return
  false; unsupported procedures return `PROC_UNAVAIL`.
- Registrations are keyed by program, version, protocol, and port. Identical
  registrations conflict; several versions may share one endpoint.
- RPC handlers may reply asynchronously. A refcounted request retains its UDP
  peer or TCP connection until reply/drop.
- NFS remains UDP-only on ports 635 and 2049 in this phase; its command line,
  BOOTP lease, duplicate cache, reset, and migration blocker do not change.

## File map

### libslirp fork

- `src/libslirp.h`, `src/libslirp.map`: public UDP-full and TCP-service ABI.
- `src/udp-service.c`: directed-broadcast matching with unicast reply source.
- `src/tcp-service.h`, `src/tcp-service.c`: listener/connection ownership.
- `src/slirp.h`, `src/slirp.c`, `src/socket.h`, `src/tcp_input.c`,
  `src/tcp_subr.c`: packet-path integration and teardown.
- `test/udpservicetest.c`, `test/tcpservicetest.c`, `meson.build`: coverage.

### QEMU

- `include/net/slirp-stream.h`, `net/slirp-stream-internal.h`,
  `net/slirp-stream.c`: lifecycle-safe TCP adapter.
- `include/net/slirp-udp.h`, `net/slirp-udp-internal.h`, `net/slirp-udp.c`,
  `net/slirp.c`: broadcast flag bridge and per-stack registries.
- `include/net/onc-rpc.h`, `net/onc-rpc-xdr.c`, `net/slirp-rpc-internal.h`,
  `net/slirp-rpc.c`: XDR, RPC envelopes, request ownership, registry, portmapper.
- `hw/nfs/nfs2-{xdr,protocol,server,object}.{c,h}`: retain only NFS-specific
  protocol and register MOUNT/NFS with shared RPC.
- `meson.build`, `net/meson.build`, `tests/unit/meson.build`: feature probes and
  targets.
- `tests/unit/test-slirp-stream.c`, `tests/unit/test-onc-rpc.c`, existing NFS
  and SLiRP tests: focused coverage.

### Task 1: Create the libslirp feature worktree

**Files:** Verify only `/home/blanham/projects/NeXT/libslirp`; create worktree under `/mnt/build/NeXT/worktrees`.

- [ ] Run `git -C /home/blanham/projects/NeXT/libslirp status --short --branch` and record existing changes without modifying them.
- [ ] Run `git -C /home/blanham/projects/NeXT/libslirp worktree add /mnt/build/NeXT/worktrees/netinfo-libslirp -b feature/netinfo-services next-bootp`.
- [ ] Configure `/mnt/build/NeXT/builds/netinfo-libslirp` and run `meson test -C /mnt/build/NeXT/builds/netinfo-libslirp --print-errorlogs`.
- [ ] Expected: clean feature worktree and all existing libslirp tests pass.

### Task 2: Add directed-broadcast delivery to libslirp UDP services

**Files:** Modify `src/libslirp.h`, `src/libslirp.map`, `src/udp-service.c`; test `test/udpservicetest.c`.

- [ ] Add a failing test that registers `10.0.2.2:111`, injects a datagram for
  `10.0.2.255:111`, observes one callback, and verifies the reply source is
  `10.0.2.2:111`. Verify the legacy API does not receive broadcasts.
- [ ] Run `meson test -C /mnt/build/NeXT/builds/netinfo-libslirp udp-service --print-errorlogs`; expected failure is the missing full-listen API.
- [ ] Add this public contract and export the new symbol:

```c
typedef enum SlirpUdpListenFlags {
    SLIRP_UDP_LISTEN_DEFAULT = 0,
    SLIRP_UDP_LISTEN_BROADCAST = 1U << 0,
} SlirpUdpListenFlags;

SlirpUdpListener *slirp_udp_listen_full(
    Slirp *slirp, struct in_addr address, uint16_t port,
    SlirpUdpListenFlags flags, const SlirpUdpCallbacks *callbacks,
    void *opaque);
```

- [ ] Make `slirp_udp_listen()` call the full API with `DEFAULT`. Store flags
  in the listener; match the subnet's directed broadcast only when
  `BROADCAST` is set, while retaining the configured unicast address as the
  reply source. Reject unknown flags.
- [ ] Add tests for duplicate endpoints, broadcast-vs-unicast precedence,
  reentrant removal, cleanup during callback, and restricted mode.
- [ ] Run the focused test and full libslirp suite; expected all pass.
- [ ] Commit with `git commit -am "udp: support guest-service broadcast delivery"`.

### Task 3: Add a bounded TCP guest-service API to libslirp

**Files:** Create `src/tcp-service.h`, `src/tcp-service.c`, `test/tcpservicetest.c`; modify `src/libslirp.h`, `src/libslirp.map`, `src/slirp.h`, `src/slirp.c`, `src/socket.h`, `src/tcp_input.c`, `src/tcp_subr.c`, `meson.build`.

- [ ] Write failing tests for listen/accept, two simultaneous connections,
  ordered receive, partial send/backpressure, half-close, reset, listener
  removal during receive, stack cleanup, endpoint collision, and zero host-FD
  growth.
- [ ] Run `ninja -C /mnt/build/NeXT/builds/netinfo-libslirp tcpservicetest`; expected compilation failure for missing types.
- [ ] Add this public ownership contract and export every function:

```c
typedef struct SlirpTcpListener SlirpTcpListener;
typedef struct SlirpTcpConnection SlirpTcpConnection;
typedef struct SlirpTcpCallbacks {
    void (*accepted)(SlirpTcpListener *, SlirpTcpConnection *,
                     const struct sockaddr_in *peer, void *opaque);
    void (*receive)(SlirpTcpConnection *, const uint8_t *, size_t,
                    void *opaque);
    void (*can_send)(SlirpTcpConnection *, void *opaque);
    void (*closed)(SlirpTcpConnection *, void *opaque);
} SlirpTcpCallbacks;

SlirpTcpListener *slirp_tcp_listen(Slirp *, struct in_addr, uint16_t,
                                   const SlirpTcpCallbacks *, void *);
size_t slirp_tcp_connection_can_send(SlirpTcpConnection *);
int slirp_tcp_connection_send(SlirpTcpConnection *, const uint8_t *, size_t);
void slirp_tcp_connection_close(SlirpTcpConnection *);
void slirp_tcp_listener_remove(SlirpTcpListener *);
```

- [ ] Store a listener list on `Slirp` and a service-connection pointer on each
  TCP socket. Match service endpoints before normal vhost handling. Give each
  accepted connection one listener ref and one stack/socket ref; make close
  and removal idempotent and defer reclamation across callbacks.
- [ ] Cap listeners at 64, active service connections at 256, callback chunks
  at 64 KiB, and queued server-to-guest bytes at the existing TCP send-buffer
  limit. Return `-EAGAIN`, `-ENOTCONN`, or `-EINVAL` as applicable.
- [ ] Run `meson test -C /mnt/build/NeXT/builds/netinfo-libslirp tcp-service udp-service guestfwd --print-errorlogs`, then the full suite; expected all pass.
- [ ] Commit with `git add ... && git commit -m "tcp: add guest-only service listeners"`.

### Task 4: Add QEMU's named-netdev TCP adapter and UDP broadcast flag

**Files:** Create `include/net/slirp-stream.h`, `net/slirp-stream-internal.h`, `net/slirp-stream.c`, `tests/unit/test-slirp-stream.c`; modify `include/net/slirp-udp.h`, `net/slirp-udp-internal.h`, `net/slirp-udp.c`, `net/slirp.c`, `net/meson.build`, `meson.build`, `tests/unit/meson.build`, `subprojects/slirp.wrap`.

- [ ] Pin the two libslirp commits, add compile/link probes for
  `slirp_udp_listen_full` and `slirp_tcp_listen`, and define
  `CONFIG_SLIRP_TCP_SERVICE` only when the complete ABI links.
- [ ] Write fake-backend tests matching `test-slirp-udp.c` for endpoint
  validation, concurrent connection ownership, send backpressure, reentrant
  close/remove, netdev teardown, and unavailable stubs.
- [ ] Run the new target; expected compilation failure for missing QEMU APIs.
- [ ] Add the QEMU contract:

```c
typedef struct QemuSlirpStreamListener QemuSlirpStreamListener;
typedef struct QemuSlirpStream QemuSlirpStream;
typedef struct QemuSlirpStreamOps {
    void (*connected)(QemuSlirpStream *, const struct sockaddr_in *, void *);
    void (*receive)(QemuSlirpStream *, const uint8_t *, size_t, void *);
    void (*can_send)(QemuSlirpStream *, void *);
    void (*closed)(QemuSlirpStream *, void *);
} QemuSlirpStreamOps;
```

  Expose `qemu_slirp_stream_listen()`, `qemu_slirp_stream_can_send()`,
  `qemu_slirp_stream_send()`, `qemu_slirp_stream_close()`, and
  `qemu_slirp_stream_listener_remove()`. Add `qemu_slirp_udp_listen_full()` and
  keep the current function as a default wrapper.
- [ ] Make `SlirpState` own both registries, initialize them after addresses are
  known, invalidate before `slirp_cleanup()`, and free after callback teardown.
- [ ] Run `test-slirp-stream`, `test-slirp-udp`, `test-slirp-guestfwd`, and
  `test-slirp-il-integration`; expected all pass.
- [ ] Commit with `git commit -m "net: add SLiRP guest stream services"`.

### Task 5: Extract generic XDR and ONC RPC envelopes

**Files:** Create `include/net/onc-rpc.h`, `net/onc-rpc-xdr.c`, `tests/unit/test-onc-rpc.c`; modify `hw/nfs/nfs2-xdr.{c,h}`, `hw/nfs/nfs2-protocol.h`, `net/meson.build`, `tests/unit/meson.build`.

- [ ] Move the existing bounded reader/writer, AUTH_NULL/AUTH_SYS call decoder,
  and accepted/denied reply builders into generic `OncRpc*` names. Add XDR
  boolean, signed 32-bit, counted array, and bounded string writer helpers.
- [ ] Keep NFS argument codecs in `nfs2-xdr.c`; update callers directly rather
  than leaving permanent compatibility macros.
- [ ] Move existing golden RPC envelope tests into `test-onc-rpc.c` and add TCP
  record-marker tests: fragmented header, multi-fragment record, oversized
  fragment, two records in one receive, and truncated close.
- [ ] Run `test-onc-rpc` first; expected RED until extraction is complete, then
  run `test-nfs2-xdr` and `test-nfs2-server`; expected all pass.
- [ ] Commit with `git commit -m "net: extract bounded ONC RPC codecs"`.

### Task 6: Add the per-netdev RPC registry and portmapper

**Files:** Create `net/slirp-rpc-internal.h`, `net/slirp-rpc.c`; modify `include/net/onc-rpc.h`, `net/slirp.c`, `net/meson.build`, `tests/unit/test-onc-rpc.c`, `tests/unit/test-slirp-il-integration.c`.

- [ ] Write failing tests for registration/collision, version ranges, UDP/TCP
  GETPORT, ordered DUMP, CALLIT success and silent drop, asynchronous reply,
  request cancellation, teardown inside a callback, and no host listener.
- [ ] Define registration and dispatch explicitly:

```c
typedef enum OncRpcDispatchResult {
    ONC_RPC_DISPATCH_ASYNC,
    ONC_RPC_DISPATCH_REPLIED,
    ONC_RPC_DISPATCH_DROP,
} OncRpcDispatchResult;
typedef struct OncRpcProgram {
    uint32_t program, version_low, version_high;
    uint16_t port;
    unsigned transports;
    OncRpcDispatchResult (*dispatch)(OncRpcRequest *, const OncRpcCall *,
                                     void *opaque);
} OncRpcProgram;
```

  Expose register/unregister plus request reply/drop/ref/unref. Registry-owned
  requests enforce 32 KiB UDP and 1 MiB TCP record limits.
- [ ] Construct one registry in each `SlirpState`. Lazily acquire UDP/TCP port
  111 with the first program and release it with the last. Implement PMAP NULL,
  GETPORT, DUMP, and CALLIT; CALLIT invokes only UDP programs and wraps the
  target result as `port + counted opaque`.
- [ ] Run `test-onc-rpc` and real SLiRP integration; expected all pass.
- [ ] Commit with `git commit -m "net: add shared SLiRP ONC RPC registry"`.

### Task 7: Move NFS onto the shared registry

**Files:** Modify `hw/nfs/nfs2-object.c`, `hw/nfs/nfs2-server.{c,h}`, `hw/nfs/nfs2-protocol.h`, `tests/unit/test-nfs2-server.c`, `tests/qtest/nfs-server-object-test.c`.

- [ ] Change NFS tests to inject decoded `OncRpcRequest` objects and remove the
  private portmapper cases. Add a qtest creating NFS plus a second fake RPC
  program and verifying both appear in one PMAP DUMP.
- [ ] Run `test-nfs2-server`; expected RED because the old transport interface
  remains.
- [ ] Remove `NFS2_SERVICE_PORTMAP`, `dispatch_portmap()`, and private mapping
  constants. Register MOUNT program 100005 versions 1..3 on UDP/635 and NFS
  program 100003 versions 2..3 on UDP/2049. Preserve exact version rejection in
  each program handler.
- [ ] Have the coroutine retain `OncRpcRequest`, use its raw bytes/peer for the
  existing duplicate key, call `onc_rpc_request_reply()`, then unref. Unregister
  programs before freeing the server during cleanup.
- [ ] Run the six baseline commands from the design worktree; expected the same
  6 tests and 58 reported subtests pass.
- [ ] Commit with `git commit -m "nfs: use the shared ONC RPC registry"`.

### Task 8: Foundation verification

**Files:** Modify `docs/system/devices/nfs-root.rst` only if its portmapper ownership wording is now inaccurate.

- [ ] Run `git diff --check` and `scripts/checkpatch.pl` on the foundation commits.
- [ ] Run `meson test -C /mnt/build/NeXT/builds/netinfo-server --suite unit --print-errorlogs`.
- [ ] Run the complete libslirp suite and the m68k NFS object qtest.
- [ ] Launch NFS with `-netdev user` and verify `/proc/$pid/fd` contains no new host TCP/UDP socket attributable to embedded services.
- [ ] Verify the public README and five protected GIFs are unchanged.
- [ ] Commit any test/documentation-only corrections separately.
