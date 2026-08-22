# NetBSD NFS Root Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Boot the archived NetBSD/next68k path from the NeXT ROM through `/boot`, load its kernel over NFSv2, mount a disposable writable NFS root, and reach a login or shell without opening any host network listener.

**Architecture:** The public libslirp fork gains a bounded guest UDP datagram-listener API and a standard BOOTP root-path lease. QEMU consumes those APIs through named-user-netdev adapters and exposes an in-process `nfs-server` object backed by an existing `-fsdev`; it serves portmapper v2 on UDP/111, mountd v1 on UDP/635, and NFSv2 on UDP/2049 at the SLiRP virtual-host address. The lab stages a mapped-xattr NetBSD root into a private per-run directory, captures the complete boot exchange, and promotes evidence only after the ROM, bootstrap, NFS mount, and visible login/shell gates are recorded.

**Tech Stack:** C11, GLib, libslirp IPv4/UDP and BOOTP, QEMU QOM/QAPI, QEMU `V9fsBackend`/`FileOperations`, QEMU coroutines and crypto HMAC-SHA-256, ONC RPC v2/XDR, mount protocol v1, NFSv2, Python 3 standard library, unittest, QTest, Meson/Ninja, PCAP.

---

## Fixed protocol and safety contract

- NetBSD compatibility is pinned to the `netbsd-1-5` source behavior: the standalone loader calls portmapper v2 over UDP, mountd v1, then NFSv2 with 32-byte file handles and 1 KiB reads. The kernel may probe mount/NFS v3 first, so unsupported v3 `GETPORT` returns zero and direct wrong-version RPC calls return `PROG_MISMATCH`; the client then falls back cleanly to mount v1 and NFSv2.
- Guest endpoints are the named SLiRP stack's virtual-host IPv4 address, normally `10.0.2.2`. No host socket, host port, TAP device, `rpcbind`, `mountd`, or `nfsd` is created.
- Fixed ports are UDP/111 for portmapper, UDP/635 for mountd, and UDP/2049 for NFS. They are intentionally not configurable in the first implementation.
- The standard BOOTP reply supplies option 17 with `/` and leaves the NeXT and Plan 9 vendor reply formats byte-for-byte unchanged.
- RPC accepts `AUTH_NULL` and syntactically valid `AUTH_SYS`, but never treats asserted UID, GID, hostname, or groups as authority and never calls `setuid()`. Export access remains constrained by the selected `fsdev`; mapped metadata is presented to the guest.
- `nfs-server` defaults to read-only even when its `fsdev` is writable. `writable=on` is accepted only when the `fsdev` is not `readonly=on`. The lab enables writes only on its newly staged disposable root.
- File handles are 32 bytes: magic `QN2F` (4), version (4), per-run handle ID (8), generation (4), and a 12-byte HMAC-SHA-256 truncation over the first 20 bytes. The random 32-byte HMAC key never leaves QEMU, is not migrated, and is regenerated per process.
- Every parser is length-bounded. Limits are: UDP payload 65,507 bytes in libslirp; RPC datagram 32 KiB; NFS read/write payload 8 KiB; path 1,024 bytes; name 255 bytes; AUTH_SYS machine name 255 bytes; AUTH_SYS auxiliary groups 16; file-handle records 65,536; duplicate mutation cache 256 entries for 60 seconds of virtual time.
- Exact duplicate mutation requests are keyed by peer IPv4, peer UDP port, XID, and SHA-256 of the complete request. A duplicate received while the first request is in flight is dropped; a duplicate received after completion gets the cached reply. This prevents retransmitted UDP writes from running twice.

## Repository and integration order

Use three independent branches and worktrees; never develop this feature in the dirty canonical lab checkout.

1. Public libslirp: branch `feature/netbsd-nfs` from `next-bootp` in `/home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-libslirp`; push to `origin/feature/netbsd-nfs` first.
2. QEMU: continue branch `feature/netbsd-nfs` in `/home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-qemu`; pin `subprojects/slirp.wrap` to the pushed libslirp commit, then push to `github/feature/netbsd-nfs`.
3. Lab: branch `feature/netbsd-nfs` from `metachicken` in `/home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-lab`; its build metadata must name the exact QEMU feature worktree and commit. The lab currently has no remote, so merge locally and push only after a remote is explicitly configured.
4. Merge libslirp into `next-bootp` first, update the QEMU pin if the merge creates a different commit, merge QEMU into `metachicken`, rebuild from a clean QEMU worktree, complete the live lab acceptance, and merge the lab branch last.

## File responsibility map

### Public libslirp fork

- `src/libslirp.h`, `src/libslirp.map`: public BOOTP-root and UDP-listener ABI.
- `src/slirp.h`, `src/slirp.c`: per-stack ownership, validation, setup, and teardown.
- `src/bootp.c`: standard BOOTP option 17 and root-server address; vendor replies remain isolated.
- `src/udp-service.h`, `src/udp-service.c`: guest-only UDP endpoint registry, reentrant removal, receive dispatch, and reply injection.
- `src/udp.c`: one dispatch hook before restricted-mode egress.
- `test/bootptest.c`, `test/udpservicetest.c`, `meson.build`: wire-format, lifecycle, bounds, and no-host-socket coverage.

### QEMU

- `meson.build`, `subprojects/slirp.wrap`: public API feature probes and immutable fork revision.
- `include/net/slirp-udp.h`, `net/slirp-udp-internal.h`, `net/slirp-udp.c`, `net/slirp.c`, `net/meson.build`: named-netdev UDP adapter and libslirp bridge.
- `include/net/slirp-bootp.h`, `net/slirp-bootp-internal.h`, `net/slirp-bootp.c`: single-owner standard BOOTP root lease.
- `hw/nfs/nfs2-protocol.h`: ONC RPC, mount, and NFSv2 constants and public request structures.
- `hw/nfs/nfs2-xdr.h`, `hw/nfs/nfs2-xdr.c`: strict cursor-based XDR decoder and bounded encoder.
- `hw/nfs/nfs2-handle.h`, `hw/nfs/nfs2-handle.c`: authenticated handle encoding and bounded path/alias table.
- `hw/nfs/nfs2-server.h`, `hw/nfs/nfs2-server.c`: RPC dispatch, duplicate cache, coroutine-backed filesystem operations, and reply generation.
- `hw/nfs/nfs2-object.c`, `hw/nfs/meson.build`, `hw/meson.build`: user-creatable object, three UDP listeners, BOOTP ownership, reset, and unwind.
- `qapi/qom.json`: `NfsServerProperties` and `nfs-server` object schema.
- `tests/unit/test-slirp-udp.c`, `tests/unit/test-nfs2-xdr.c`, `tests/unit/test-nfs2-handle.c`, `tests/unit/test-nfs2-server.c`, `tests/unit/test-nfs2-server-local.c`: focused unit and real local-backend coverage.
- `tests/qtest/nfs-server-object-test.c`, `tests/unit/meson.build`, `tests/qtest/meson.build`: QOM/netdev lifecycle and no-host-listener proof.
- `docs/system/devices/nfs-root.rst`, `docs/system/devices/index.rst`: concise operator contract and example.

### Lab

- `nextcube_lab/netbsd_netboot.py`: immutable artifact validation, safe mapped-xattr root extraction, argv construction, and ROM boot command.
- `nextcube_lab/netbsd_nfs_pcap.py`: bounded BOOTP/RPC/mount/NFS evidence decoder.
- `nextcube_lab/baseline/model.py`, `nextcube_lab/baseline/runner.py`, `nextcube_lab/baseline_cli.py`, `nextcube_lab/baseline/bundle.py`: case/profile validation, run lifecycle, visible gates, manifest, and promotion.
- `tests/test_netbsd_netboot.py`, `tests/test_netbsd_nfs_pcap.py`, `tests/test_baseline_model.py`, `tests/test_baseline_runner.py`, `tests/test_baseline_bundle.py`, `tests/helpers/fake_qemu.py`: staging, argv, capture, evidence, and failure paths.
- `README.md`: concise artifact and IL-adjacent NetBSD NFS-root launch instructions.

### Task 1: Create the isolated implementation branches

**Files:**
- Verify only: `/home/blanham/projects/NeXT/libslirp`
- Verify only: `/home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-qemu`
- Verify only: `/home/blanham/projects/NeXT/lab`

- [ ] **Step 1: Confirm the QEMU plan worktree is clean except for this plan**

Run:

```bash
git -C /home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-qemu status --short --branch
```

Expected: branch `feature/netbsd-nfs`; the only uncommitted path is `docs/superpowers/plans/2026-08-22-netbsd-nfs-root.md`.

- [ ] **Step 2: Create the libslirp feature worktree**

Run:

```bash
git -C /home/blanham/projects/NeXT/libslirp fetch origin
git -C /home/blanham/projects/NeXT/libslirp worktree add /home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-libslirp -b feature/netbsd-nfs next-bootp
```

Expected: the new worktree is on `feature/netbsd-nfs` at the current `next-bootp` commit and `git status --short` is empty.

- [ ] **Step 3: Create the lab feature worktree without touching its dirty canonical checkout**

Run:

```bash
git -C /home/blanham/projects/NeXT/lab worktree add /home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-lab -b feature/netbsd-nfs metachicken
```

Expected: the new lab worktree is clean; the existing changes and untracked result files remain only in `/home/blanham/projects/NeXT/lab`.

- [ ] **Step 4: Commit the implementation plan on the QEMU feature branch**

```bash
git add docs/superpowers/plans/2026-08-22-netbsd-nfs-root.md
git commit -m "docs: plan private NetBSD NFS root service"
```

### Task 2: Add standard BOOTP NFS-root configuration to libslirp

**Files:**
- Modify: `src/libslirp.h`
- Modify: `src/libslirp.map`
- Modify: `src/slirp.h`
- Modify: `src/slirp.c`
- Modify: `src/bootp.c`
- Test: `test/bootptest.c`

- [ ] **Step 1: Write failing BOOTP root tests**

Add tests that configure `/` at `10.0.2.2`, send a normal RFC1048 BOOTP request, and assert `bp_siaddr == 10.0.2.2` plus the exact option bytes `17, 1, '/'`. Add separate assertions that NeXT v1 and Plan 9 vendor replies remain byte-identical, paths without a leading slash fail, 256-byte paths fail, a zero server fails, and `NULL` clears the lease.

```c
static void test_bootp_root_option(void)
{
    const SlirpBootpRootConfig root = {
        .server = { htonl(0x0a000202) },
        .path = "/",
    };
    struct capture capture = { 0 };

    assert(slirp_set_bootp_root(slirp, &root));
    send_request(slirp, normal_bootp_request, sizeof(normal_bootp_request),
                 &capture);
    assert(load_be32(capture.packet + BOOTP_OFFSET + 20) == 0x0a000202);
    assert(find_option(capture.packet, capture.length, RFC1533_ROOTPATH,
                       (const uint8_t *)"/", 1));
}
```

- [ ] **Step 2: Run the focused test and observe RED**

Run:

```bash
meson setup build-netbsd-nfs
meson test -C build-netbsd-nfs bootp --print-errorlogs
```

Expected: compilation fails because `SlirpBootpRootConfig` and `slirp_set_bootp_root()` do not exist.

- [ ] **Step 3: Add the public lease API and owned stack state**

Add this exact public contract and export the function in `src/libslirp.map`:

```c
typedef struct SlirpBootpRootConfig {
    struct in_addr server;
    const char *path;
} SlirpBootpRootConfig;

#define SLIRP_HAVE_BOOTP_ROOT 1

SLIRP_EXPORT
bool slirp_set_bootp_root(Slirp *slirp,
                          const SlirpBootpRootConfig *config);
```

Store an owned `char *bootp_root_path` and `struct in_addr bootp_root_server` in `Slirp`. In `slirp_set_bootp_root()`, reject a null stack, zero server, empty or non-absolute path, or a path longer than 255 bytes; duplicate the accepted path before replacing the old value. Clear and free it in `slirp_cleanup()`.

- [ ] **Step 4: Emit option 17 only in the standard reply branch**

In `bootp_reply()`, keep the existing NeXT and Plan 9 branches untouched. In the normal RFC1533 branch, set `bp_siaddr` and append the root path only when the lease is active:

```c
if (slirp->bootp_root_path) {
    val = strlen(slirp->bootp_root_path);
    if (q + val + 2 >= end) {
        g_warning("DHCP packet size exceeded, omitting root path option.");
    } else {
        rbp->bp_siaddr = slirp->bootp_root_server;
        *q++ = RFC1533_ROOTPATH;
        *q++ = val;
        memcpy(q, slirp->bootp_root_path, val);
        q += val;
    }
}
```

- [ ] **Step 5: Run BOOTP tests and the complete libslirp suite**

Run:

```bash
meson test -C build-netbsd-nfs bootp --print-errorlogs
meson test -C build-netbsd-nfs --print-errorlogs
```

Expected: all BOOTP cases pass and the complete suite remains green.

- [ ] **Step 6: Commit the BOOTP lease**

```bash
git add src/libslirp.h src/libslirp.map src/slirp.h src/slirp.c src/bootp.c test/bootptest.c
git commit -m "bootp: advertise a guest NFS root"
```

### Task 3: Add the bounded public guest UDP listener to libslirp

**Files:**
- Create: `src/udp-service.h`
- Create: `src/udp-service.c`
- Create: `test/udpservicetest.c`
- Modify: `src/libslirp.h`
- Modify: `src/libslirp.map`
- Modify: `src/slirp.h`
- Modify: `src/slirp.c`
- Modify: `src/udp.c`
- Modify: `meson.build`

- [ ] **Step 1: Write failing public API and packet-path tests**

Cover endpoint creation on `10.0.2.2:2049`, exact source peer delivery, one reply with source `10.0.2.2:2049`, duplicate endpoint rejection, ports 0/67/69 rejection, network/broadcast/out-of-subnet rejection, payload 65,508 rejection, reply-to-outside-subnet rejection, restricted-mode delivery, reentrant listener removal, cleanup during a callback, and proof that registration creates no host file descriptor.

```c
static void received(SlirpUdpListener *listener,
                     const struct sockaddr_in *peer,
                     const uint8_t *data, size_t len, void *opaque)
{
    struct state *s = opaque;
    s->peer = *peer;
    s->calls++;
    assert(slirp_udp_listener_send(listener, peer, data, len) == 0);
}

static const SlirpUdpCallbacks callbacks = {
    .receive = received,
};
```

- [ ] **Step 2: Run the test and observe RED**

Run:

```bash
meson setup --reconfigure build-netbsd-nfs
ninja -C build-netbsd-nfs udpservicetest
```

Expected: compilation fails because the public UDP service types and functions are absent.

- [ ] **Step 3: Define the public API**

Add this exact ABI to `src/libslirp.h`, define `SLIRP_HAVE_UDP_SERVICE 1`, and export all three symbols:

```c
#define SLIRP_HAVE_UDP_SERVICE 1
#define SLIRP_UDP_SERVICE_MAX_PAYLOAD 65507

typedef struct SlirpUdpListener SlirpUdpListener;
typedef struct SlirpUdpCallbacks {
    void (*receive)(SlirpUdpListener *listener,
                    const struct sockaddr_in *peer,
                    const uint8_t *data, size_t len,
                    void *opaque);
} SlirpUdpCallbacks;

SLIRP_EXPORT SlirpUdpListener *slirp_udp_listen(
    Slirp *slirp, struct in_addr address, uint16_t port,
    const SlirpUdpCallbacks *callbacks, void *opaque);
SLIRP_EXPORT int slirp_udp_listener_send(
    SlirpUdpListener *listener, const struct sockaddr_in *peer,
    const uint8_t *data, size_t len);
SLIRP_EXPORT void slirp_udp_listener_remove(SlirpUdpListener *listener);
```

- [ ] **Step 4: Implement registry ownership and reentrant removal**

`SlirpUdpListener` owns its endpoint, callback copy, opaque pointer, reference count, callback depth, and `valid`/`remove_pending` flags. Store listeners in a `QTAILQ` on `Slirp`. `slirp_udp_listener_remove()` unlinks immediately, defers free until callback depth reaches zero, and is safe when called from `receive`. `slirp_cleanup()` invalidates all listeners before UDP socket cleanup.

```c
struct SlirpUdpListener {
    QTAILQ_ENTRY(SlirpUdpListener) entry;
    Slirp *slirp;
    struct sockaddr_in local;
    SlirpUdpCallbacks callbacks;
    void *opaque;
    unsigned refs;
    unsigned callback_depth;
    bool valid;
    bool remove_pending;
};
```

- [ ] **Step 5: Intercept and reply without a host socket**

Call `udp_service_input()` in `udp_input()` after BOOTP/TFTP handling and before the `restricted` check. It matches destination address and port, copies the payload to callback-owned temporary storage for the callback duration, and returns true; the caller's existing `bad:` path frees the mbuf exactly once. `slirp_udp_listener_send()` allocates an mbuf, copies the bounded payload, and calls `udp_output(NULL, ...)` with the listener as source and the captured peer as destination.

```c
if (udp_service_input(slirp, &ip->ip_src, uh->uh_sport,
                      &ip->ip_dst, uh->uh_dport,
                      (uint8_t *)(uh + 1), len - sizeof(*uh))) {
    goto bad;
}
```

- [ ] **Step 6: Run the focused and full suites under ASan**

Run:

```bash
meson setup build-netbsd-nfs-asan -Db_sanitize=address,undefined
meson test -C build-netbsd-nfs-asan udp-service bootp --print-errorlogs
meson test -C build-netbsd-nfs-asan --print-errorlogs
```

Expected: all cases pass without sanitizer diagnostics.

- [ ] **Step 7: Commit the UDP service**

```bash
git add src/libslirp.h src/libslirp.map src/slirp.h src/slirp.c src/udp.c src/udp-service.h src/udp-service.c test/udpservicetest.c meson.build
git commit -m "udp: add bounded guest service listeners"
```

### Task 4: Publish and pin the libslirp dependency

**Files:**
- Modify: `subprojects/slirp.wrap`
- Modify: `meson.build`

- [ ] **Step 1: Verify and push the public fork branch**

Run in the libslirp worktree:

```bash
git diff --check
meson test -C build-netbsd-nfs --print-errorlogs
git push -u origin feature/netbsd-nfs
git rev-parse HEAD
```

Expected: tests pass, push succeeds, and the final command prints the immutable commit to pin.

- [ ] **Step 2: Write a failing QEMU feature probe**

In QEMU `meson.build`, introduce `slirp_udp_service` and `slirp_bootp_root` link probes that take addresses of the exact public functions and set `CONFIG_SLIRP_UDP_SERVICE` and `CONFIG_SLIRP_BOOTP_ROOT`. Initially leave `subprojects/slirp.wrap` unchanged. In this RED step, initialize both booleans false for the internal fallback; use the link probes for an external libslirp.

```meson
slirp_udp_service = cc.links('''
  #include <libslirp.h>
  #ifndef SLIRP_HAVE_UDP_SERVICE
  #error libslirp was built without guest UDP services
  #endif
  int main(void) {
    return !(slirp_udp_listen && slirp_udp_listener_send &&
             slirp_udp_listener_remove);
  }''', dependencies: slirp, name: 'libslirp guest UDP service API')
```

- [ ] **Step 3: Reconfigure and observe RED against the old pin**

Run:

```bash
meson setup build-netbsd-nfs-qemu-oldpin -Dslirp=enabled
```

Expected: the guest UDP service and BOOTP-root probes report `NO` against the old fallback revision.

- [ ] **Step 4: Pin the pushed libslirp commit and make internal detection explicit**

Replace `revision` in `subprojects/slirp.wrap` with the exact hash printed in Step 1. For `slirp.type_name() == 'internal'`, set both feature booleans true because the immutable fallback is now the tested fork revision; for external libslirp, retain the link probes.

- [ ] **Step 5: Reconfigure and observe GREEN**

Run:

```bash
meson setup build-netbsd-nfs-qemu -Dslirp=enabled
```

Expected: both feature booleans are enabled and configuration completes; external builds report both link probes `YES`.

- [ ] **Step 6: Commit the QEMU dependency contract**

```bash
git add meson.build subprojects/slirp.wrap
git commit -m "build: require libslirp NFS root primitives"
```

### Task 5: Add QEMU named-netdev UDP and BOOTP adapters

**Files:**
- Create: `include/net/slirp-udp.h`
- Create: `net/slirp-udp-internal.h`
- Create: `net/slirp-udp.c`
- Create: `include/net/slirp-bootp.h`
- Create: `net/slirp-bootp-internal.h`
- Create: `net/slirp-bootp.c`
- Create: `tests/unit/test-slirp-udp.c`
- Modify: `net/slirp.c`
- Modify: `net/meson.build`
- Modify: `tests/unit/test-slirp-il-integration.c`
- Modify: `tests/unit/meson.build`

- [ ] **Step 1: Write failing registry tests**

Test missing/non-user netdevs, IPv4 disabled, endpoint collisions, fixed virtual-host address selection, three simultaneous distinct ports, datagram source preservation, reply direction, reentrant removal, backend invalidation, one-owner BOOTP root claims, claim unwind, and no backend availability.

```c
typedef struct QemuSlirpUdpListenerOps {
    void (*datagram)(QemuSlirpUdpListener *listener,
                     const struct sockaddr_in *peer,
                     const uint8_t *data, size_t len,
                     void *opaque);
} QemuSlirpUdpListenerOps;
```

- [ ] **Step 2: Build the focused test and observe RED**

Run:

```bash
ninja -C build-netbsd-nfs-qemu tests/unit/test-slirp-udp
```

Expected: compilation fails because the adapter headers and sources do not exist.

- [ ] **Step 3: Implement the UDP registry API**

Expose only named-netdev operations; callers cannot choose an address, because every service lives at the stack's virtual host:

```c
int qemu_slirp_udp_listen(const char *netdev_id, uint16_t port,
                          const QemuSlirpUdpListenerOps *ops, void *opaque,
                          QemuSlirpUdpListener **listener, Error **errp);
int qemu_slirp_udp_send(QemuSlirpUdpListener *listener,
                        const struct sockaddr_in *peer,
                        const uint8_t *data, size_t len);
void qemu_slirp_udp_listener_remove(QemuSlirpUdpListener *listener);
```

Mirror the proven IL registry reference/deferred-removal pattern. The backend bridge in `net/slirp.c` converts between `SlirpUdpListener` and `QemuSlirpUdpListener`, and `SlirpState` owns the registry until netdev teardown.

- [ ] **Step 4: Implement the single-owner BOOTP root lease**

Expose:

```c
bool qemu_slirp_bootp_root_claim(const char *netdev_id, const char *root_path,
                                 QemuSlirpBootpRootLease **lease,
                                 Error **errp);
void qemu_slirp_bootp_root_release(QemuSlirpBootpRootLease **lease);
```

The registry supplies its virtual-host address as `SlirpBootpRootConfig.server`, validates an absolute 1-255 byte path, rejects a second owner, and clears libslirp before freeing the lease.

- [ ] **Step 5: Run unit and real-libslirp integration tests**

Run:

```bash
ninja -C build-netbsd-nfs-qemu tests/unit/test-slirp-udp tests/unit/test-slirp-il-integration
build-netbsd-nfs-qemu/tests/unit/test-slirp-udp
build-netbsd-nfs-qemu/tests/unit/test-slirp-il-integration
```

Expected: both binaries pass; integration injects a guest UDP frame and observes the QEMU callback/reply through real libslirp.

- [ ] **Step 6: Commit the adapters**

```bash
git add include/net/slirp-udp.h include/net/slirp-bootp.h net/slirp-udp-internal.h net/slirp-udp.c net/slirp-bootp-internal.h net/slirp-bootp.c net/slirp.c net/meson.build tests/unit/test-slirp-udp.c tests/unit/test-slirp-il-integration.c tests/unit/meson.build
git commit -m "net: bridge private UDP services into slirp"
```

### Task 6: Implement strict ONC RPC and NFSv2 XDR codecs

**Files:**
- Create: `hw/nfs/nfs2-protocol.h`
- Create: `hw/nfs/nfs2-xdr.h`
- Create: `hw/nfs/nfs2-xdr.c`
- Create: `hw/nfs/meson.build`
- Create: `tests/unit/test-nfs2-xdr.c`
- Modify: `hw/meson.build`
- Modify: `tests/unit/meson.build`

- [ ] **Step 1: Write golden-vector and rejection tests**

Use literal big-endian vectors for portmapper `GETPORT`, mount v1 `MNT "/"`, NFSv2 `GETATTR`, `LOOKUP "netbsd"`, `READ`, and AUTH_SYS root. Reject truncated words, integer overflow, non-zero XDR padding, strings over their limit, group counts above 16, file handles not exactly 32 bytes, trailing procedure bytes, RPC version other than 2, reply overflow, reads/writes above 8 KiB, and datagrams above 32 KiB.

```c
static const uint8_t pmap_getport[] = {
    0x12,0x34,0x56,0x78, 0,0,0,0, 0,0,0,2,
    0,1,0x86,0xa0, 0,0,0,2, 0,0,0,3,
    0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
    0,1,0x86,0xa3, 0,0,0,2, 0,0,0,17, 0,0,0,0,
};
```

- [ ] **Step 2: Build and observe RED**

Run:

```bash
ninja -C build-netbsd-nfs-qemu tests/unit/test-nfs2-xdr
```

Expected: compilation fails because `nfs2-xdr.h` is absent.

- [ ] **Step 3: Define cursor APIs and protocol constants**

Use explicit readers/writers that never advance on failure:

```c
typedef struct Nfs2XdrReader {
    const uint8_t *cursor;
    const uint8_t *end;
} Nfs2XdrReader;

typedef struct Nfs2XdrWriter {
    uint8_t *cursor;
    uint8_t *end;
} Nfs2XdrWriter;

bool nfs2_xdr_u32(Nfs2XdrReader *r, uint32_t *value);
bool nfs2_xdr_opaque(Nfs2XdrReader *r, uint8_t *out, size_t exact);
bool nfs2_xdr_string(Nfs2XdrReader *r, char *out, size_t capacity,
                     size_t maximum);
bool nfs2_xdr_put_u32(Nfs2XdrWriter *w, uint32_t value);
bool nfs2_xdr_put_opaque(Nfs2XdrWriter *w, const void *data, size_t len);
```

Define program numbers 100000/100003/100005, the exact NFSv2 procedure numbers 0-17, accepted/rejected RPC statuses, mount v1, NFSv2 status values, file types, and the fixed bounds from this plan.

- [ ] **Step 4: Decode RPC authentication without granting authority**

`nfs2_rpc_decode_call()` returns XID, program, version, procedure, a body cursor, and informational AUTH_SYS fields. It bounds and consumes the credential and verifier, records UID/GID only for trace diagnostics, and never exposes a credential-switching callback.

```c
typedef struct Nfs2RpcCall {
    uint32_t xid, program, version, procedure;
    uint32_t auth_flavor, uid, gid;
    uint32_t groups[16];
    size_t group_count;
    Nfs2XdrReader body;
} Nfs2RpcCall;
```

- [ ] **Step 5: Encode accepted, mismatch, and garbage-arguments replies**

Add helpers for accepted-success, `PROG_UNAVAIL`, `PROG_MISMATCH(low, high)`, `PROC_UNAVAIL`, `GARBAGE_ARGS`, and AUTH errors. Every helper returns false on insufficient capacity and leaves the caller to drop the datagram.

- [ ] **Step 6: Run codec tests under sanitizers**

Run:

```bash
ninja -C build-netbsd-nfs-qemu tests/unit/test-nfs2-xdr
build-netbsd-nfs-qemu/tests/unit/test-nfs2-xdr
```

Expected: all golden vectors and every malformed input case pass.

- [ ] **Step 7: Commit the codec**

```bash
git add hw/meson.build hw/nfs/meson.build hw/nfs/nfs2-protocol.h hw/nfs/nfs2-xdr.h hw/nfs/nfs2-xdr.c tests/unit/test-nfs2-xdr.c tests/unit/meson.build
git commit -m "nfs: add bounded NFSv2 XDR codec"
```

### Task 7: Add authenticated per-run NFS file handles

**Files:**
- Create: `hw/nfs/nfs2-handle.h`
- Create: `hw/nfs/nfs2-handle.c`
- Create: `tests/unit/test-nfs2-handle.c`
- Modify: `hw/nfs/meson.build`
- Modify: `tests/unit/meson.build`

- [ ] **Step 1: Write failing handle-table tests**

Inject a fixed 32-byte key and verify the exact 32-byte root handle, successful resolution, one-bit forgery rejection, wrong version rejection, stale generation rejection, table bound 65,536, path bound 1,024, rename-prefix updates, removal invalidation, hard-link alias fallback, and complete key/table clearing on free.

```c
static const uint8_t key[32] = { 0, 1, 2, 3, 4, 5, 6, 7 };
Nfs2HandleTable *table = nfs2_handle_table_new(key, sizeof(key), &error_abort);
Nfs2FileHandle root;
g_assert_true(nfs2_handle_create(table, 1, "/", &root, &error_abort));
```

- [ ] **Step 2: Build and observe RED**

Run:

```bash
ninja -C build-netbsd-nfs-qemu tests/unit/test-nfs2-handle
```

Expected: compilation fails because the handle table API is absent.

- [ ] **Step 3: Implement the exact handle wire layout**

Use network byte order and HMAC-SHA-256 truncated to 12 bytes:

```c
typedef struct Nfs2FileHandle {
    uint8_t bytes[32];
} Nfs2FileHandle;

/* bytes 0..3 "QN2F"; 4..7 version=1; 8..15 id;
 * 16..19 generation; 20..31 MAC(first 20 bytes). */
```

Use `qcrypto_hmac_new(QCRYPTO_HASH_ALG_SHA256, ...)`, constant-time MAC comparison, `qcrypto_random_bytes()` for production keys, and an injected key only in tests.

- [ ] **Step 4: Implement bounded path and alias records**

Each ID owns a generation, canonical `V9fsPath`, and zero or more hard-link aliases. Resolution verifies MAC first, then returns a copied path. Rename rewrites exact and descendant prefixes atomically; remove drops that alias and increments generation only when no alias remains. External host-side replacements are detected later by the server's `lstat` identity check and return `NFSERR_STALE`.

- [ ] **Step 5: Run handle and crypto tests**

Run:

```bash
ninja -C build-netbsd-nfs-qemu tests/unit/test-nfs2-handle
build-netbsd-nfs-qemu/tests/unit/test-nfs2-handle
```

Expected: all handle vectors, bounds, and mutation cases pass.

- [ ] **Step 6: Commit the handle table**

```bash
git add hw/nfs/nfs2-handle.h hw/nfs/nfs2-handle.c hw/nfs/meson.build tests/unit/test-nfs2-handle.c tests/unit/meson.build
git commit -m "nfs: authenticate bounded file handles"
```

### Task 8: Serve portmapper, mountd, and read-side NFSv2

**Files:**
- Create: `hw/nfs/nfs2-server.h`
- Create: `hw/nfs/nfs2-server.c`
- Create: `tests/unit/test-nfs2-server.c`
- Create: `tests/unit/test-nfs2-server-local.c`
- Modify: `hw/nfs/meson.build`
- Modify: `tests/unit/meson.build`

- [ ] **Step 1: Write failing protocol-dispatch tests with a fake backend**

Cover portmapper NULL/GETPORT, zero for mount v2/v3 and NFSv3, mount v1 exact `/`, denial of every other path, NFS NULL/GETATTR/LOOKUP/READLINK/READ/READDIR/STATFS, obsolete ROOT/WRITECACHE success, wrong program/version/procedure replies, truncated procedure bodies, 8 KiB bounds, EOF behavior, and errno-to-NFS status mapping.

```c
typedef enum Nfs2Service {
    NFS2_SERVICE_PORTMAP,
    NFS2_SERVICE_MOUNT,
    NFS2_SERVICE_NFS,
} Nfs2Service;

typedef struct Nfs2TransportOps {
    int (*send)(Nfs2Service service, const struct sockaddr_in *peer,
                const uint8_t *data, size_t len, void *opaque);
} Nfs2TransportOps;
```

- [ ] **Step 2: Run the server test and observe RED**

Run:

```bash
ninja -C build-netbsd-nfs-qemu tests/unit/test-nfs2-server
```

Expected: compilation fails because `nfs2-server.h` does not exist.

- [ ] **Step 3: Add server construction and datagram dispatch**

Expose a transport-independent server. `receive()` copies at most 32 KiB, queues one coroutine on the main AioContext, and returns before filesystem work begins.

```c
Nfs2Server *nfs2_server_new(const char *fsdev_id, bool writable,
                            const Nfs2TransportOps *transport,
                            void *transport_opaque, Error **errp);
int nfs2_server_receive(Nfs2Server *server, Nfs2Service service,
                        const struct sockaddr_in *peer,
                        const uint8_t *data, size_t len, Error **errp);
bool nfs2_server_busy(const Nfs2Server *server);
void nfs2_server_begin_close(Nfs2Server *server);
void nfs2_server_free(Nfs2Server *server);
```

- [ ] **Step 4: Implement exact portmapper and mount behavior**

For UDP only, return 111 for `(100000,2)`, 635 for `(100005,1)`, 2049 for `(100003,2)`, and zero for every unsupported tuple. Mount v1 accepts only `/`, returns the authenticated root handle, and returns `NFSERR_ACCES` for any other path.

- [ ] **Step 5: Add coroutine-backed read operations**

Initialize `V9fsBackend` with `v9fs_backend_init()`. Wrap blocking `FileOperations` in `thread_pool_submit_co()` workers, copying `V9fsPath` and buffers into request-owned storage. Implement:

```text
GETATTR  lstat -> fixed NFSv2 fattr
LOOKUP   name_to_path + lstat -> handle + fattr
READLINK readlink -> bounded XDR string
READ     open + preadv + close -> fattr + byte count + data
READDIR  opendir/readdir -> bounded cookie/name/fileid list
STATFS   statfs -> tsize=8192 and 32-bit-clamped block counts
```

Map backend metadata to NFSv2 types/modes, preserve mapped-xattr UID/GID/rdev, clamp 64-bit sizes/times safely, and map only documented errno values to NFS status; unknown errors become `NFSERR_IO`.

- [ ] **Step 6: Prove the real local backend and symlink confinement**

In `test-nfs2-server-local.c`, register a temporary `-fsdev local` with `security_model=mapped-xattr`, create directories, a file, and a symlink, then issue encoded mount/lookup/read/readdir requests. Assert a symlink outside the export is returned as a link and is never followed by LOOKUP/READ.

- [ ] **Step 7: Run focused server tests**

Run:

```bash
ninja -C build-netbsd-nfs-qemu tests/unit/test-nfs2-server tests/unit/test-nfs2-server-local
build-netbsd-nfs-qemu/tests/unit/test-nfs2-server
build-netbsd-nfs-qemu/tests/unit/test-nfs2-server-local
```

Expected: both binaries pass and every request completes without host-path escape.

- [ ] **Step 8: Commit the read-side server**

```bash
git add hw/nfs/nfs2-server.h hw/nfs/nfs2-server.c hw/nfs/meson.build tests/unit/test-nfs2-server.c tests/unit/test-nfs2-server-local.c tests/unit/meson.build
git commit -m "nfs: serve read-only NFSv2 roots"
```

### Task 9: Add safe mutations and duplicate-XID suppression

**Files:**
- Modify: `hw/nfs/nfs2-server.h`
- Modify: `hw/nfs/nfs2-server.c`
- Modify: `tests/unit/test-nfs2-server.c`
- Modify: `tests/unit/test-nfs2-server-local.c`

- [ ] **Step 1: Write failing read-only and mutation tests**

Assert all mutating procedures return `NFSERR_ROFS` by default. With writable enabled, cover SETATTR, WRITE, CREATE, REMOVE, RENAME, LINK, SYMLINK, MKDIR, and RMDIR against a mapped-xattr temporary export. Assert AUTH_SYS UID/GID/groups do not alter host process credentials. Send each encoded mutation twice with the same peer/XID/body and prove the backend mutation counter increments once; send the same XID with a different body and prove it is a new request.

- [ ] **Step 2: Run the exact mutation tests and observe RED**

Run:

```bash
build-netbsd-nfs-qemu/tests/unit/test-nfs2-server -p /nfs2/mutations
build-netbsd-nfs-qemu/tests/unit/test-nfs2-server-local -p /nfs2/local-writable
```

Expected: failures report unimplemented procedures or read-only replies in writable mode.

- [ ] **Step 3: Implement writable gating before decoding mutation bodies**

At server creation, reject `writable=true` when `backend.ctx.export_flags & V9FS_RDONLY`. At dispatch, classify procedures with one table and return `NFSERR_ROFS` before scheduling workers when writable is false.

```c
static const bool nfs2_mutating[NFS2_PROC_STATFS + 1] = {
    [NFS2_PROC_SETATTR] = true, [NFS2_PROC_WRITE] = true,
    [NFS2_PROC_CREATE] = true,  [NFS2_PROC_REMOVE] = true,
    [NFS2_PROC_RENAME] = true,  [NFS2_PROC_LINK] = true,
    [NFS2_PROC_SYMLINK] = true, [NFS2_PROC_MKDIR] = true,
    [NFS2_PROC_RMDIR] = true,
};
```

- [ ] **Step 4: Implement each NFSv2 mutation through `FileOperations`**

Use `open2/pwritev/fsync/close`, `chmod/truncate/utimensat`, `unlinkat`, `renameat`, `link`, `symlink`, and `mkdir`. Never call `setuid`; AUTH_SYS is diagnostic only. Preserve mapped ownership supplied by the staged root and ignore SETATTR UID/GID changes while honoring mode, size, and time fields. Update handle paths/aliases only after backend success.

- [ ] **Step 5: Implement the bounded in-flight/reply cache**

Before mutation execution, hash the complete request and search by peer address, peer port, XID, and digest. Drop exact in-flight duplicates. Cache the encoded reply before the first send; replay exact completed duplicates. Expire entries after 60,000 ms of `QEMU_CLOCK_VIRTUAL`, evict least-recently-used completed entries at 256, and return `RPC_SYSTEM_ERR` without mutating if all entries are in flight.

- [ ] **Step 6: Run mutation, server, and backend tests**

Run:

```bash
ninja -C build-netbsd-nfs-qemu tests/unit/test-nfs2-server tests/unit/test-nfs2-server-local
build-netbsd-nfs-qemu/tests/unit/test-nfs2-server
build-netbsd-nfs-qemu/tests/unit/test-nfs2-server-local
```

Expected: read-only rejection, all writable operations, and exact-once duplicate behavior pass.

- [ ] **Step 7: Commit writable NFSv2**

```bash
git add hw/nfs/nfs2-server.h hw/nfs/nfs2-server.c tests/unit/test-nfs2-server.c tests/unit/test-nfs2-server-local.c
git commit -m "nfs: add retransmission-safe NFSv2 writes"
```

### Task 10: Expose the guest-only `nfs-server` QOM object

**Files:**
- Create: `hw/nfs/nfs2-object.c`
- Create: `tests/qtest/nfs-server-object-test.c`
- Modify: `hw/nfs/meson.build`
- Modify: `qapi/qom.json`
- Modify: `tests/qtest/meson.build`

- [ ] **Step 1: Write failing QTest object lifecycle cases**

Cover CLI creation after `-fsdev` and `-netdev`, QMP create/delete/recreate, missing fsdev/netdev, non-user netdev, IPv4 disabled, missing libslirp features, root-path validation, writable object over read-only fsdev rejection, port collision unwind, BOOTP owner collision, deletion refusal during filesystem work, reset, and a no-host-listener proof that binds host loopback UDP/2049 before creating the object. The libslirp unit test separately verifies that registering all guest endpoints creates no host descriptor.

```c
qts = qtest_initf(
    "-machine none -nodefaults "
    "-fsdev local,id=root,path=%s,security_model=mapped-xattr "
    "-netdev user,id=nextnet "
    "-object nfs-server,id=nfs,fsdev=root,netdev=nextnet,writable=on",
    test_root);
```

- [ ] **Step 2: Build the QTest and observe RED**

Run:

```bash
ninja -C build-netbsd-nfs-qemu tests/qtest/nfs-server-object-test
```

Expected: QAPI generation or compilation fails because `nfs-server` is unknown.

- [ ] **Step 3: Add the QAPI schema**

Add:

```qapi
{ 'struct': 'NfsServerProperties',
  'data': { 'fsdev': 'str', 'netdev': 'str',
            '*root-path': 'str', '*writable': 'bool' },
  'if': { 'all': [ 'CONFIG_VIRTFS', 'CONFIG_SLIRP',
                    'CONFIG_SLIRP_UDP_SERVICE',
                    'CONFIG_SLIRP_BOOTP_ROOT' ] } }
```

Add `nfs-server` to `ObjectType` and its feature-gated property mapping. `root-path` defaults to `/`; `writable` defaults false.

- [ ] **Step 4: Implement ordered completion and reverse unwind**

`complete()` validates immutable properties, verifies both libslirp features before acquiring filesystem resources, initializes `Nfs2Server`, claims BOOTP `/`, then listens on 111, 635, and 2049 in that order. On failure or object removal, stop input, remove listeners in reverse order, release BOOTP, wait/refuse deletion while server work is pending, clear the file-handle key/cache, and clean the backend.

```c
static const struct {
    Nfs2Service service;
    uint16_t port;
} endpoints[] = {
    { NFS2_SERVICE_PORTMAP, 111 },
    { NFS2_SERVICE_MOUNT, 635 },
    { NFS2_SERVICE_NFS, 2049 },
};
```

- [ ] **Step 5: Disable migration with live server state**

Register the object as resettable, clear UDP peers and duplicate cache on guest reset while retaining the per-process handle table, and reject migration while the object is complete because listener state and per-run handles are intentionally non-migratable.

- [ ] **Step 6: Run QOM/QTest and focused unit suites**

Run:

```bash
ninja -C build-netbsd-nfs-qemu qemu-system-m68k tests/qtest/nfs-server-object-test
build-netbsd-nfs-qemu/tests/qtest/nfs-server-object-test
meson test -C build-netbsd-nfs-qemu --suite unit --print-errorlogs
```

Expected: object lifecycle cases and all unit suites pass.

- [ ] **Step 7: Commit the object**

```bash
git add hw/nfs/nfs2-object.c hw/nfs/meson.build qapi/qom.json tests/qtest/nfs-server-object-test.c tests/qtest/meson.build
git commit -m "nfs: expose a private slirp NFSv2 server"
```

### Task 11: Document and publish the QEMU feature

**Files:**
- Create: `docs/system/devices/nfs-root.rst`
- Modify: `docs/system/devices/index.rst`

- [ ] **Step 1: Add concise QEMU documentation**

Document this exact standalone example and state that all three RPC services are guest-only, AUTH_SYS is not trusted, NFSv3 falls back to v2, and writes require both a writable fsdev and `writable=on`:

```text
-fsdev local,id=netbsdroot,path=/run/netbsd-root,security_model=mapped-xattr
-netdev user,id=nextnet,ipv6=off,tftp=/run/tftp,bootfile=boot
-object nfs-server,id=netbsdnfs,fsdev=netbsdroot,netdev=nextnet,writable=on
-net nic,model=next-mb8795,netdev=nextnet
```

- [ ] **Step 2: Run the complete focused production gate**

Run:

```bash
git diff --check
ninja -C build-netbsd-nfs-qemu qemu-system-m68k
meson test -C build-netbsd-nfs-qemu bootp udp-service --print-errorlogs
build-netbsd-nfs-qemu/tests/unit/test-slirp-udp
build-netbsd-nfs-qemu/tests/unit/test-slirp-il-integration
build-netbsd-nfs-qemu/tests/unit/test-nfs2-xdr
build-netbsd-nfs-qemu/tests/unit/test-nfs2-handle
build-netbsd-nfs-qemu/tests/unit/test-nfs2-server
build-netbsd-nfs-qemu/tests/unit/test-nfs2-server-local
build-netbsd-nfs-qemu/tests/qtest/nfs-server-object-test
build-netbsd-nfs-qemu/tests/qtest/plan9-9p1-object-test
```

Expected: every command exits zero; existing Plan 9 TCP/IL coverage remains green.

- [ ] **Step 3: Commit docs and push the QEMU feature branch**

```bash
git add docs/system/devices/nfs-root.rst docs/system/devices/index.rst
git commit -m "docs: describe private NetBSD NFS roots"
git push -u github feature/netbsd-nfs
```

### Task 12: Stage immutable NetBSD boot and mapped-xattr root artifacts in the lab

**Files:**
- Create: `nextcube_lab/netbsd_netboot.py`
- Create: `tests/test_netbsd_netboot.py`

- [ ] **Step 1: Write failing artifact and extraction tests**

Use synthetic tar archives to cover exact SHA-256 verification, regular `/boot`, root `netbsd`, directories, files, symlinks, hard links, character/block devices represented as mapped-xattr placeholders, mode/uid/gid/rdev preservation, path traversal, absolute members, duplicate members, hard-link cycles, device major/minor overflow, FIFO/socket rejection, member/depth/file/expanded-size limits, symlink-parent substitution, and cleanup after failure.

```python
@dataclass(frozen=True)
class NetBSDArtifacts:
    boot: Path
    boot_sha256: str
    root_archive: Path
    root_sha256: str
```

- [ ] **Step 2: Run the module tests and observe RED**

Run:

```bash
python3 -B -m unittest tests.test_netbsd_netboot
```

Expected: import fails because `nextcube_lab.netbsd_netboot` does not exist.

- [ ] **Step 3: Implement immutable artifact verification**

Require absolute regular non-symlink paths plus explicit 64-hex SHA-256 values. Open each with `O_RDONLY|O_NOFOLLOW|O_CLOEXEC`, hash through the descriptor, revalidate inode metadata after reading, and reject a changed or multiply linked input. Record path, size, and digest in `run.json`.

- [ ] **Step 4: Implement safe mapped-xattr extraction**

Extract under the private run root without following symlinks. For every node set little-endian `user.virtfs.uid`, `user.virtfs.gid`, and `user.virtfs.mode`; for device placeholders also set `user.virtfs.rdev`. Device nodes are zero-length host regular files whose mapped mode retains `S_IFCHR` or `S_IFBLK`.

```python
os.setxattr(path, b"user.virtfs.uid", struct.pack("<I", member.uid),
            follow_symlinks=False)
os.setxattr(path, b"user.virtfs.gid", struct.pack("<I", member.gid),
            follow_symlinks=False)
os.setxattr(path, b"user.virtfs.mode", struct.pack("<I", mapped_mode),
            follow_symlinks=False)
os.setxattr(path, b"user.virtfs.rdev", struct.pack("<Q", rdev),
            follow_symlinks=False)
```

Require `netbsd` as a regular executable root member and stage the separately verified bootstrap as `tftp/boot`. Fsync files and directories before launch.

- [ ] **Step 5: Add the exact ROM command and QEMU argv builder**

Return `ben() boot` from `netbsd_rom_boot_command()`. Build:

```python
[
    str(qemu), "-M", "next-station",
    "-global", "next-pc.system-timer-frequency=4456448",
    "-bios", str(rom), "-m", "64M", "-display", "gtk",
    "-qmp", f"unix:{qmp},server=on,wait=off",
    "-fsdev", f"local,id=netbsdroot,path={rootfs},security_model=mapped-xattr",
    "-netdev", f"user,id=nextnet,ipv6=off,tftp={tftp},bootfile=boot",
    "-object", "nfs-server,id=netbsdnfs,fsdev=netbsdroot,netdev=nextnet,writable=on",
    "-net", "nic,model=next-mb8795,netdev=nextnet",
    "-object", f"filter-dump,id=nextdump,netdev=nextnet,file={pcap}",
    "-no-reboot",
]
```

- [ ] **Step 6: Run tests and commit**

Run:

```bash
python3 -B -m unittest tests.test_netbsd_netboot
git add nextcube_lab/netbsd_netboot.py tests/test_netbsd_netboot.py
git commit -m "lab: stage disposable NetBSD NFS roots"
```

### Task 13: Add the `netbsd-netboot` lab case and visible gate controller

**Files:**
- Modify: `nextcube_lab/baseline/model.py`
- Modify: `nextcube_lab/baseline/runner.py`
- Modify: `nextcube_lab/baseline_cli.py`
- Modify: `tests/test_baseline_model.py`
- Modify: `tests/test_baseline_runner.py`

- [ ] **Step 1: Write failing CLI/config/argv tests**

Add `netbsd-netboot` and launch profile `netbsd-nfs`. Require interactive GTK, the owned `ben() boot` command, `--netbsd-boot`, `--netbsd-boot-sha256`, `--netbsd-root-archive`, and `--netbsd-root-sha256`; reject these flags for every other case. Assert run IDs use token `nbsd`, no disk overlay is created, exact artifact identities are recorded, and argv matches Task 12 byte-for-byte.

- [ ] **Step 2: Run focused tests and observe RED**

Run:

```bash
python3 -B -m unittest tests.test_baseline_model tests.test_baseline_runner
```

Expected: parser/config validation rejects `netbsd-netboot` and `netbsd-nfs`.

- [ ] **Step 3: Extend the closed model and parser**

Extend all case literals/sets to include `netbsd-netboot`, add the four artifact fields to `RunConfig`, and derive the owned boot command before `_validate_launch_profile()`. Preserve existing Plan 9 and disk behavior.

```python
case: Literal["rom", "disk", "plan9-netboot", "netbsd-netboot"]
```

- [ ] **Step 4: Stage and launch the NetBSD profile in `run_baseline()`**

After creating `RunPaths`, validate and stage artifacts into `paths.root / "tftp"` and `paths.root / "rootfs"`, authenticate `qemu-system-m68k`, launch the exact argv, and treat the staged tree as disposable run evidence rather than a protected source asset. Always remove it during successful publication; retain it after failure for diagnosis.

- [ ] **Step 5: Add four ordered visible gates**

For `netbsd-netboot`, accept only:

```text
gate 1 ROM loaded /boot
gate 2 NetBSD bootstrap loaded kernel
gate 3 kernel mounted root on NFS
gate 4 NetBSD login or shell visible
```

The operator still uses `boot` followed by exact uppercase `SEND`; the harness transmits only `ben() boot`. A run cannot become complete without all four gates and a final screenshot/register capture.

- [ ] **Step 6: Run tests and commit**

Run:

```bash
python3 -B -m unittest tests.test_baseline_model tests.test_baseline_runner
git add nextcube_lab/baseline/model.py nextcube_lab/baseline/runner.py nextcube_lab/baseline_cli.py tests/test_baseline_model.py tests/test_baseline_runner.py
git commit -m "lab: launch NetBSD from a private NFS root"
```

### Task 14: Prove BOOTP, portmapper, mountd, and NFS in bounded PCAP evidence

**Files:**
- Create: `nextcube_lab/netbsd_nfs_pcap.py`
- Create: `tests/test_netbsd_nfs_pcap.py`
- Modify: `tests/helpers/fake_qemu.py`

- [ ] **Step 1: Write failing PCAP proof tests**

Construct little- and big-endian PCAPs containing normal BOOTP option 17 `/`, portmapper v2 GETPORT for mount v1 and NFSv2, mount v1 MNT `/`, NFSv2 LOOKUP `netbsd`, READ, and kernel GETATTR/READDIR/STATFS. Reject missing steps, NFSv3-only traffic, wrong virtual-host address, wrong ports, malformed XDR, fragments, non-UDP traffic, truncated captures, packet counts over 1,000,000, file size over 256 MiB, and a capture changed during inspection.

- [ ] **Step 2: Run the module test and observe RED**

Run:

```bash
python3 -B -m unittest tests.test_netbsd_nfs_pcap
```

Expected: import fails because `nextcube_lab.netbsd_nfs_pcap` does not exist.

- [ ] **Step 3: Implement a metadata-only streaming inspector**

Parse Ethernet/IPv4/UDP and only the bounded RPC fields needed for proof. Emit no file data, names other than exact `netbsd`, AUTH_SYS hostnames, UIDs, or GIDs. Return:

```python
@dataclass(frozen=True)
class NetBSDNFSProof:
    bootp_root_path: bool
    portmap_mount_v1: bool
    portmap_nfs_v2: bool
    mount_root: bool
    bootstrap_lookup: bool
    bootstrap_read: bool
    kernel_root_ops: bool
    packet_count: int
    sha256: str
```

- [ ] **Step 4: Teach fake QEMU to emit a complete synthetic exchange**

When argv contains `nfs-server,id=netbsdnfs`, write a bounded PCAP with the seven required facts so runner tests exercise real evidence validation rather than mocking the inspector.

- [ ] **Step 5: Run tests and commit**

Run:

```bash
python3 -B -m unittest tests.test_netbsd_nfs_pcap
git add nextcube_lab/netbsd_nfs_pcap.py tests/test_netbsd_nfs_pcap.py tests/helpers/fake_qemu.py
git commit -m "lab: inspect NetBSD NFS root captures"
```

### Task 15: Bind NFS proof into evidence promotion

**Files:**
- Modify: `nextcube_lab/baseline/runner.py`
- Modify: `nextcube_lab/baseline/bundle.py`
- Modify: `tests/test_baseline_runner.py`
- Modify: `tests/test_baseline_bundle.py`

- [ ] **Step 1: Write failing completion and tamper tests**

Assert `netbsd-netboot` remains `evidence-incomplete` when any visible gate or PCAP fact is missing, QEMU exits before QMP, the capture changes, source/build identity drifts, the staged boot/root digest differs, or shutdown is unauthenticated. Assert complete runs include the raw `traces/netbsd-nfs-network.pcap`, a metadata proof JSON, four checkpoints, artifact identities, and no extracted rootfs in promoted evidence.

- [ ] **Step 2: Run focused tests and observe RED**

Run:

```bash
python3 -B -m unittest tests.test_baseline_runner tests.test_baseline_bundle
```

Expected: new completion assertions fail because NFS proof is not required or recorded.

- [ ] **Step 3: Inspect the capture before declaring success**

After QEMU shutdown and final capture close, authenticate the PCAP inode, call `inspect_netbsd_nfs_pcap_descriptor()`, require all seven booleans, and record only the returned proof fields and digest in `run.json`. A parser error or incomplete proof sets `outcome=evidence-incomplete` and retains raw state.

- [ ] **Step 4: Extend bundle and summary validation**

Permit `netbsd-netboot`, require the PCAP/proof/checkpoints, cross-check the proof digest against the manifest, forbid `rootfs/` and `tftp/` from promoted evidence, and render a compact protocol table in the Markdown summary.

- [ ] **Step 5: Run the full lab test suite and commit**

Run:

```bash
python3 -B -m unittest discover -s tests
git add nextcube_lab/baseline/runner.py nextcube_lab/baseline/bundle.py tests/test_baseline_runner.py tests/test_baseline_bundle.py
git commit -m "lab: require NetBSD NFS root evidence"
```

Expected: all lab tests pass.

### Task 16: Document, integrate, and run live acceptance

**Files:**
- Modify: `README.md`
- Create during execution only: ignored `work/` build/run data and `evidence/runs/` bundles

- [ ] **Step 1: Add concise README instructions**

Add one short NetBSD NFS-root subsection giving the required boot/root artifacts and SHA flags, the build command, and this launch shape:

```bash
python3 -B -m nextcube_lab.baseline_cli run \
  --workspace /home/blanham/projects/NeXT \
  --qemu-source /home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-qemu \
  --work-root /home/blanham/projects/NeXT/lab/work \
  --evidence-root /home/blanham/projects/NeXT/evidence/runs \
  --results-root /home/blanham/projects/NeXT/lab/results \
  --build-metadata /absolute/path/to/build.json \
  --case netbsd-netboot --launch-profile netbsd-nfs \
  --mode interactive --display gtk \
  --netbsd-boot /absolute/path/to/boot \
  --netbsd-boot-sha256 <64-lowercase-hex> \
  --netbsd-root-archive /absolute/path/to/netbsd-root.tar.gz \
  --netbsd-root-sha256 <64-lowercase-hex>
```

- [ ] **Step 2: Commit and verify the lab branch**

Run:

```bash
git add README.md
git commit -m "docs: add NetBSD NFS root launch"
git diff --check
python3 -B -m unittest discover -s tests
```

Expected: clean diff and all tests pass.

- [ ] **Step 3: Merge and push libslirp first**

Run from the canonical libslirp checkout after confirming it is clean:

```bash
git switch next-bootp
git merge --no-ff feature/netbsd-nfs
git push origin next-bootp
git rev-parse HEAD
```

Expected: the public branch is pushed. If the merge commit differs from the feature commit, update QEMU's `slirp.wrap`, reconfigure, rerun Task 11's gate, commit the pin, and push the QEMU feature branch again.

- [ ] **Step 4: Merge and push canonical QEMU**

Run from `/home/blanham/projects/NeXT/qemu` after preserving its existing untracked build directories:

```bash
git switch metachicken
git merge --no-ff feature/netbsd-nfs
git push github metachicken
git -C /home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-qemu merge --ff-only metachicken
```

Expected: canonical QEMU contains the NFS service, the remote advances, and the clean feature worktree now points at the exact canonical merge commit used for the acceptance build.

- [ ] **Step 5: Build from a clean QEMU worktree and capture metadata**

Run from the lab feature worktree:

```bash
python3 -B -m nextcube_lab.baseline_cli build \
  --qemu-source /home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-qemu \
  --work-root /home/blanham/projects/NeXT/lab/work \
  --jobs 8 \
  --summary /home/blanham/projects/NeXT/lab/work/netbsd-nfs-current.json
```

Expected: build metadata records the exact QEMU source path/commit and `qemu-system-m68k` with GTK and SLiRP.

- [ ] **Step 6: Run the headful acceptance and record all gates truthfully**

Run the README command with the exact artifact paths/digests and the new build metadata. In the operator console, record gates only after the GTK window visibly shows each fact, capture the final login/shell screen, then stop cleanly.

Expected PCAP sequence:

```text
BOOTP reply option 17 "/"
UDP/111 GETPORT mount v1 -> 635
UDP/635 MOUNT "/" -> 32-byte handle
UDP/111 GETPORT NFS v2 -> 2049
UDP/2049 LOOKUP "netbsd" and READ
UDP/2049 kernel GETATTR/READDIR/STATFS
```

- [ ] **Step 7: Verify promoted evidence**

Run:

```bash
python3 -B -m nextcube_lab.baseline_cli verify-bundle --run-root /absolute/path/to/promoted-run
```

Expected: verification reports all four visible gates, all seven NFS proof facts, matching artifact/source/build hashes, authenticated shutdown, and no promoted rootfs/TFTP tree.

- [ ] **Step 8: Merge the lab branch locally**

First inspect the canonical lab checkout with `git status --short`. If any pre-existing tracked change overlaps the feature branch, stop and ask the user how to preserve it; do not stash or overwrite it. Otherwise run from `/home/blanham/projects/NeXT/lab`:

```bash
git switch metachicken
git merge --no-ff feature/netbsd-nfs
```

Expected: local `metachicken` advances. Do not invent a push destination; configure or obtain an explicit lab remote before running `git push`.

## Final verification matrix

Before declaring the feature complete, retain the exact outputs of:

```bash
meson test -C /home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-libslirp/build-netbsd-nfs --print-errorlogs
meson test -C /home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-qemu/build-netbsd-nfs-qemu --suite unit --print-errorlogs
/home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-qemu/build-netbsd-nfs-qemu/tests/qtest/nfs-server-object-test
/home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-qemu/build-netbsd-nfs-qemu/tests/qtest/plan9-9p1-object-test
python3 -B -m unittest discover -s /home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-lab/tests
git -C /home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-libslirp diff --check
git -C /home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-qemu diff --check
git -C /home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-lab diff --check
```

Completion requires the live promoted bundle in addition to green automated tests. A protocol-only unit success, a bootloader-only screenshot, or an NFS capture without the visible login/shell gate is not sufficient.
