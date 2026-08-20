# Native Plan 9 9P1 Root Service Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Use superpowers:test-driven-development for every production change and superpowers:verification-before-completion before claiming a task is complete.

**Goal:** Let the archived Plan 9 Second Edition NeXT kernel obtain its network configuration through BOOTP, mount a writable matching release tree over native 9P1/TCP inside QEMU user networking, and execute `/68020/init` without an external file server or host listener.

**Architecture:** A user-creatable QEMU `plan9-9p1-server` object binds a named `-fsdev` to a callback-backed guest-only endpoint on a named SLiRP `-netdev`. A standalone old-protocol codec frames the Second Edition wire format, a server layer translates 16-bit fid operations through QEMU's existing filesystem backend, and a public libslirp extension supplies Plan 9 BOOTP vendor data plus safe callback-forward teardown. The lab controller stages a private writable root tree and performs the final GTK-visible acceptance through authenticated QMP.

**Tech Stack:** QEMU QOM/QAPI, QEMU fsdev/FileOperations and coroutines, libslirp callback guest forwarding and BOOTP, GLib unit tests, QTest, Meson/Ninja, Python unittest, authenticated QMP, Plan 9 Second Edition 9P1/TCP.

---

## Repositories and branches

- QEMU implementation worktree: `/home/blanham/projects/NeXT/lab/.worktrees/plan9-9p1-qemu`, branch `feature/plan9-9p1-root`, based on `79d4d6a66aebbc622cc62b1732279394fe690016`.
- Public libslirp repository: `/home/blanham/projects/NeXT/libslirp`, branch `next-bootp`, currently published at `248600520cafba0e3704c353924de4def9cb7c6f`.
- Lab implementation worktree: `/home/blanham/projects/NeXT/lab/.worktrees/plan9-next-netboot`, branch `feature/plan9-next-netboot`, currently at `7c0eb33a06199a2fc92f93e5cb77d31b614baaef`.
- Immutable protocol oracle: `/home/blanham/projects/NeXT/archive/plan9/plan9-2e.tar.bz2`, SHA-256 `0bb3c1446deb79b179f73886eb2419ecfac9f9964040683e3d5731f074bc2ce6`.

Do not signal an existing QEMU process. Automated tests may stop only QEMU children they create. The final guest run must use GTK and authenticated QMP and must pause at each visible gate for Bryce.

## File map

### Public libslirp

- Modify `include/libslirp.h`: publish Plan 9 BOOTP configuration and safe callback-forward removal APIs.
- Modify `src/bootp.c`, `src/bootp.h`, `src/libslirp.c`, and `src/slirp.c`: retain configured Plan 9 addresses, recognize the `p9  ` request, encode the exact four-field response, and close callback sockets before removing their forwarding record.
- Modify `src/socket.h`, `src/tcp_subr.c`, and `src/misc.c`: track callback-forward socket ownership and make removal leave no dangling socket pointer.
- Modify `meson.build` and `test/bootptest.c`; create `test/guestfwdtest.c`: prove exact vendor bytes, ordinary BOOTP isolation, NeXT vendor compatibility, configured addresses, and active-socket teardown.

### QEMU

- Create `hw/9pfs/plan9-9p1-codec.h` and `hw/9pfs/plan9-9p1-codec.c`: independent Second Edition 9P1 types, stream framing, decode, and encode.
- Create `tests/unit/test-plan9-9p1-codec.c`; modify `tests/unit/meson.build`: golden-vector and malformed-stream coverage.
- Create `include/net/slirp-guestfwd.h`; modify `net/slirp.c` and `net/meson.build`: opaque named-netdev callback-forward adapter.
- Create `tests/qtest/slirp-guestfwd-test.c`; modify `tests/qtest/meson.build`: adapter lookup, validation, byte-direction, collision, and teardown coverage.
- Create `hw/9pfs/9p-backend.h` and `hw/9pfs/9p-backend.c`; modify `hw/9pfs/9p.h` and `hw/9pfs/9p.c`: reusable fsdev initialization and cleanup without a 9P2000 transport.
- Create `hw/9pfs/plan9-9p1-server.h` and `hw/9pfs/plan9-9p1-server.c`: fid table, filesystem handlers, reply queue, coroutine scheduling, and QOM lifecycle.
- Create `tests/unit/test-plan9-9p1-server.c`: synthetic transport and temporary export coverage of read-only boot and writable namespace operations.
- Modify `qapi/qom.json`, `hw/9pfs/meson.build`, and `tests/qtest/meson.build`; create `tests/qtest/plan9-9p1-object-test.c`: command-line/QMP object creation and lifecycle.
- Modify `subprojects/slirp.wrap`: pin the new public libslirp commit.
- Modify `docs/system/devices/9p.rst`: document the guest-only 9P1 service and command line.

### Lab

- Modify `nextcube_lab/plan9_netboot.py`: safe full-release extraction, root-tree identity, and exact QEMU arguments.
- Modify `nextcube_lab/baseline/model.py`, `nextcube_lab/baseline/runner.py`, and `nextcube_lab/baseline_cli.py`: authenticated Plan 9 run profile and evidence metadata.
- Modify `tests/test_plan9_netboot.py` and `tests/test_baseline_runner.py`: extraction, launch, visible-gate, and shutdown contracts.
- Add the final non-overwriting result pair under `results/plan9/` only after visible acceptance.

## Authoritative 9P1 contract

Every message starts with one type byte and a little-endian 16-bit tag; there is no total-size prefix. Fixed strings are NUL-padded arrays. `Qid` is `path:U32, vers:U32`. `Dir` is exactly 116 bytes: `name[28], uid[28], gid[28], qid[8], mode:U32, atime:U32, mtime:U32, length-low:U32, length-high-zero:U32, type:U16, dev:U16`.

The Second Edition type/length table is:

```text
Tnop 50/3       Rnop 51/3       Rerror 55/67
Tflush 56/5     Rflush 57/3
Tclone 60/7     Rclone 61/5     Twalk 62/33      Rwalk 63/13
Topen 64/6      Ropen 65/13     Tcreate 66/38    Rcreate 67/13
Tread 68/15     Rread 69/(8+n)  Twrite 70/(16+n) Rwrite 71/7
Tclunk 72/5     Rclunk 73/5     Tremove 74/5     Rremove 75/5
Tstat 76/5      Rstat 77/121    Twstat 78/121    Rwstat 79/5
Tclwalk 80/35   Rclwalk 81/13
Tsession 84/11  Rsession 85/87  Tattach 86/146   Rattach 87/26
```

`Rread` stores count at byte 5 and includes one zero pad byte before data. `Twrite` stores count at byte 13 and includes one zero pad byte before data. Reject payloads above 8192 bytes. Unknown types are fatal framing errors. A `Tsession` resets fids and returns zero challenge/authid/authdom, which disables the archived boot program's authentication exchange.

### Task 1: Make callback guest-forward removal safe in public libslirp

**Files:**
- Modify: `include/libslirp.h`
- Modify: `src/misc.c`
- Modify: `src/socket.h`
- Modify: `src/tcp_subr.c`
- Create: `test/guestfwdtest.c`
- Modify: `meson.build`

- [ ] **Step 1: Add a failing active-socket removal regression**

Construct a callback guest forward, establish its TCP socket with the existing libslirp test harness, remove the forward while that socket is active, drive timers/poll once more, and assert that the socket is closed or detached without calling the freed callback. Run the focused test under AddressSanitizer when the local build supports it.

Run:

```bash
meson test -C build guestfwd --print-errorlogs
```

Expected: the new active-removal case exposes the current stale `so->guestfwd` reference or fails its close/detach assertion.

- [ ] **Step 2: Add owner-safe removal**

Keep `slirp_remove_guestfwd(Slirp *, struct in_addr, int)` source-compatible, but before freeing the matching rule iterate the active TCP sockets and close every socket whose `so->guestfwd` points at that rule. Clear the pointer as part of the close path. Return `false` when no rule matches and `true` only after all owners are detached.

If the current socket list cannot be traversed from `misc.c`, add a private helper in `tcp_subr.c`:

```c
void tcp_remove_guestfwd_sockets(Slirp *slirp, SlirpGuestFwd *guestfwd);
```

The helper is private; do not expose `SlirpGuestFwd` publicly.

- [ ] **Step 3: Verify callback-forward compatibility**

Run the focused forward test plus the complete libslirp suite:

```bash
meson test -C build guestfwd --print-errorlogs
meson test -C build --print-errorlogs
```

Expected: all pass, with the removal regression proving no callback is reached after removal.

- [ ] **Step 4: Commit the lifecycle fix**

```bash
git add include/libslirp.h src/misc.c src/socket.h src/tcp_subr.c test/guestfwdtest.c meson.build
git commit -m "slirp: safely remove active callback guest forwards"
```

### Task 2: Add opt-in Plan 9 BOOTP vendor replies to public libslirp

**Files:**
- Modify: `include/libslirp.h`
- Modify: `src/bootp.c`
- Modify: `src/bootp.h`
- Modify: `src/libslirp.c`
- Modify: `src/slirp.c`
- Modify: `test/bootptest.c`
- Modify: `meson.build`

- [ ] **Step 1: Add byte-exact failing BOOTP tests**

Send a BOOTP request whose vendor field begins `p9  `. Assert that an enabled configuration returns exactly:

```text
p9  255.255.255.0 10.0.2.100 0.0.0.0 10.0.2.2
```

including the original transaction ID and client hardware address. Add cases proving a disabled configuration does not emit Plan 9 data, non-`p9  ` BOOTP/DHCP is unchanged, the NeXT ROM vendor test remains byte-identical, and custom subnet/server/gateway values are reflected.

- [ ] **Step 2: Publish a typed opt-in configuration API**

Add an API that copies addresses into `Slirp` rather than retaining caller pointers:

```c
typedef struct SlirpPlan9BootpConfig {
    struct in_addr netmask;
    struct in_addr file_server;
    struct in_addr auth_server;
    struct in_addr gateway;
} SlirpPlan9BootpConfig;

bool slirp_set_plan9_bootp(Slirp *slirp,
                           const SlirpPlan9BootpConfig *config);
```

Passing `NULL` disables the extension. Validate that the file server is inside the configured IPv4 virtual network and that mask/gateway agree with that network.

- [ ] **Step 3: Encode the Second Edition vendor response**

Recognize only a request beginning with the four bytes `p9  `. Format the four dotted-quad fields into the BOOTP vendor area with bounded writes and no DHCP option encoding. Preserve the existing NeXT BPOP path and generic RFC/DHCP path.

- [ ] **Step 4: Verify, commit, and publish the public dependency**

```bash
meson test -C build --print-errorlogs
git add include/libslirp.h src/bootp.c src/bootp.h src/libslirp.c src/slirp.c test/bootptest.c meson.build
git commit -m "bootp: support Plan 9 root server replies"
git -c core.sshCommand='ssh -F /dev/null' push origin next-bootp
git -c core.sshCommand='ssh -F /dev/null' ls-remote origin refs/heads/next-bootp
```

Expected: the full suite passes and the remote branch resolves to the new commit. Record that exact hash for Task 8.

### Task 3: Add an opaque QEMU SLiRP guest-forward adapter

**Files:**
- Create: `include/net/slirp-guestfwd.h`
- Modify: `net/slirp.c`
- Modify: `net/meson.build`
- Create: `tests/qtest/slirp-guestfwd-test.c`
- Modify: `tests/qtest/meson.build`

- [ ] **Step 1: Add failing adapter contract tests**

Cover missing netdev, a non-user netdev, address outside the virtual subnet, reserved host/DNS addresses, duplicate address/port, successful add/remove, guest-to-owner bytes, owner-to-guest bytes with backpressure, and removal while connected. Verify the path opens no host listener.

Run:

```bash
ninja -C build tests/qtest/slirp-guestfwd-test
build/tests/qtest/slirp-guestfwd-test
```

Expected: compilation fails because the adapter API is absent; after declarations exist, behavior cases remain red until implemented.

- [ ] **Step 2: Define the private-handle API**

Use these public-to-QEMU-internal signatures:

```c
typedef struct QemuSlirpGuestFwd QemuSlirpGuestFwd;

typedef struct QemuSlirpGuestFwdOps {
    ssize_t (*write)(const uint8_t *buf, size_t len, void *opaque);
} QemuSlirpGuestFwdOps;

int qemu_slirp_guestfwd_add(const char *netdev_id,
                            struct in_addr guest_addr,
                            uint16_t guest_port,
                            const QemuSlirpGuestFwdOps *ops,
                            void *opaque,
                            QemuSlirpGuestFwd **handle,
                            Error **errp);
size_t qemu_slirp_guestfwd_can_send(QemuSlirpGuestFwd *handle);
int qemu_slirp_guestfwd_send(QemuSlirpGuestFwd *handle,
                             const uint8_t *buf, size_t len);
bool qemu_slirp_guestfwd_set_plan9_bootp(
    QemuSlirpGuestFwd *handle,
    const SlirpPlan9BootpConfig *config,
    Error **errp);
void qemu_slirp_guestfwd_remove(QemuSlirpGuestFwd *handle);
```

The header must not expose `SlirpState` or `Slirp *`.

- [ ] **Step 3: Implement lookup, validation, transport, and idempotent teardown**

Resolve `qemu_find_netdev(netdev_id)` and require `NET_CLIENT_DRIVER_USER`. Store the opaque rule handle in QEMU's `SlirpState` forwarding list so global cleanup and object cleanup share one idempotent path. Use only `slirp_add_guestfwd`, `slirp_socket_can_recv`, `slirp_socket_recv`, `slirp_set_plan9_bootp`, and the now-safe `slirp_remove_guestfwd`; never call a host-forward API.

- [ ] **Step 4: Verify and commit**

```bash
ninja -C build tests/qtest/slirp-guestfwd-test
build/tests/qtest/slirp-guestfwd-test
git add include/net/slirp-guestfwd.h net/slirp.c net/meson.build tests/qtest
git commit -m "net/slirp: expose callback guest forwards internally"
```

### Task 4: Implement the independent Second Edition 9P1 codec

**Files:**
- Create: `hw/9pfs/plan9-9p1-codec.h`
- Create: `hw/9pfs/plan9-9p1-codec.c`
- Create: `tests/unit/test-plan9-9p1-codec.c`
- Modify: `tests/unit/meson.build`

- [ ] **Step 1: Add golden-vector and stream-framing tests**

Use literal vectors for at least:

```text
Tnop tag 0x1234:                    32 34 12
Tsession tag 1 challenge 1..8:      54 01 00 01 02 03 04 05 06 07 08
Twalk tag 2 fid 0x2a name "etc":    3e 02 00 2a 00 65 74 63 [25 zeroes]
Tread tag 3 fid 0x2a off 0x100 n512:44 03 00 2a 00 00 01 00 00 00 00 00 00 00 02
Twrite tag 4 fid 0x2a data "abc":   46 04 00 2a 00 [8 offset zeroes] 03 00 00 61 62 63
Rread tag 4 fid 0x2a data "abc":    45 04 00 2a 00 03 00 00 61 62 63
```

Cover every request/response type in the authoritative table, fixed 28/64-byte strings, 8-byte qids, 116-byte dirs, byte-at-a-time fragmentation, all split points, two coalesced messages, 8192-byte payloads, 8193 rejection, malformed count, unknown type, and trailing preservation.

- [ ] **Step 2: Prove the codec tests red for absence only**

```bash
ninja -C build tests/unit/test-plan9-9p1-codec
```

Expected: compilation fails on the missing codec header/source, not on unrelated QEMU configuration.

- [ ] **Step 3: Implement bounded primitives and framing**

Define `Plan9P1Fcall`, `Plan9P1Qid`, `Plan9P1Dir`, and `Plan9P1Stream`. Use checked cursor helpers for `u8`, little-endian `u16/u32`, old `u64`, fixed arrays, and payload slices. Provide:

```c
void plan9p1_stream_init(Plan9P1Stream *stream);
void plan9p1_stream_reset(Plan9P1Stream *stream);
int plan9p1_stream_feed(Plan9P1Stream *stream,
                        const uint8_t *buf, size_t len,
                        Plan9P1FrameFn emit, void *opaque,
                        Error **errp);
int plan9p1_decode(const uint8_t *buf, size_t len,
                   Plan9P1Fcall *fcall, Error **errp);
ssize_t plan9p1_encode(uint8_t *buf, size_t capacity,
                       const Plan9P1Fcall *fcall, Error **errp);
```

Do not copy historical implementation text. The new standalone files use the NCSA license and Bryce Lanham's copyright.

- [ ] **Step 4: Verify and commit**

```bash
ninja -C build tests/unit/test-plan9-9p1-codec
build/tests/unit/test-plan9-9p1-codec
git add hw/9pfs/plan9-9p1-codec.* tests/unit/test-plan9-9p1-codec.c tests/unit/meson.build
git commit -m "9pfs: add Plan 9 Second Edition codec"
```

### Task 5: Extract reusable fsdev backend lifecycle

**Files:**
- Create: `hw/9pfs/9p-backend.h`
- Create: `hw/9pfs/9p-backend.c`
- Modify: `hw/9pfs/9p.h`
- Modify: `hw/9pfs/9p.c`
- Modify: `hw/9pfs/meson.build`
- Extend: existing virtio-9p qtests

- [ ] **Step 1: Add lifecycle regressions around existing 9P2000 devices**

Exercise local and synth fsdev realization, missing export root, non-directory root, security-model validation, xattr limits, throttle setup, and unrealize. These tests freeze existing behavior before refactoring.

- [ ] **Step 2: Introduce the backend-only abstraction**

Define a transport-independent owner:

```c
typedef struct V9fsBackend {
    FileOperations *ops;
    FsContext ctx;
    struct stat root_st;
    GHashTable *dev_map;
    bool initialized;
} V9fsBackend;

int v9fs_backend_init(V9fsBackend *backend,
                      const char *fsdev_id,
                      Error **errp);
void v9fs_backend_cleanup(V9fsBackend *backend);
```

Move only lookup, copied export configuration, backend `ops->init`, root stat/type validation, device-map ownership, throttle ownership, and cleanup. Keep PDU pools, negotiated protocol, transport callbacks, and request state in `V9fsState`.

- [ ] **Step 3: Convert current 9P2000 realization to the shared lifecycle**

Embed `V9fsBackend` in `V9fsState` and update call sites without changing command-line or migration-visible behavior. Change `get_fsdev_fsentry` to accept `const char *` while retaining the process-lifetime pointer semantics.

- [ ] **Step 4: Verify all existing 9P tests and commit**

```bash
ninja -C build qemu-system-m68k tests/qtest/virtio-9p-test
meson test -C build --suite qemu:unit --print-errorlogs
build/tests/qtest/virtio-9p-test
git add fsdev/qemu-fsdev.* hw/9pfs/9p-backend.* hw/9pfs/9p.[ch] hw/9pfs/meson.build tests
git commit -m "9pfs: separate fsdev backend lifecycle"
```

### Task 6: Implement read-only 9P1 boot operations with synthetic transport

**Files:**
- Create: `hw/9pfs/plan9-9p1-server.h`
- Create: `hw/9pfs/plan9-9p1-server.c`
- Create: `tests/unit/test-plan9-9p1-server.c`
- Modify: `tests/unit/meson.build`
- Modify: `hw/9pfs/meson.build`

- [ ] **Step 1: Add a failing boot-sequence server test**

Build a temporary local export containing `/68020/init`, a directory, and a regular file. Feed a byte stream through a synthetic send callback and assert exact responses for `Tsession`, `Tattach`, `Tclone`, `Twalk`, `Topen`, `Tread`, `Tstat`, `Tclunk`, `Tflush`, and `Tclwalk`. Cover unknown/duplicate fids, walk of `..`, missing names, opening directories, directory reads in 116-byte records, permission errors, reconnect reset through `Tsession`, fragmented requests, ordered replies, and transport backpressure.

- [ ] **Step 2: Define server/fid and transport interfaces**

```c
typedef struct Plan9P1Server Plan9P1Server;

typedef struct Plan9P1TransportOps {
    size_t (*can_send)(void *opaque);
    int (*send)(const uint8_t *buf, size_t len, void *opaque);
} Plan9P1TransportOps;

Plan9P1Server *plan9p1_server_new(V9fsBackend *backend,
                                  const Plan9P1TransportOps *ops,
                                  void *opaque);
int plan9p1_server_receive(Plan9P1Server *server,
                           const uint8_t *buf, size_t len,
                           Error **errp);
void plan9p1_server_reset(Plan9P1Server *server);
bool plan9p1_server_busy(const Plan9P1Server *server);
void plan9p1_server_free(Plan9P1Server *server);
```

The fid table key is the full 16-bit fid. Each fid owns a confined `V9fsPath`, open kind/state, and qid. Never construct an unchecked host path; resolve through `FileOperations.name_to_path`.

- [ ] **Step 3: Implement session, identity, qids, and read-only operations**

Run blocking backend work with QEMU's coroutine worker machinery. Preserve reply order with a single request queue. `Tsession` discards fids and returns zeroed legacy auth fields. `Tattach` accepts the supplied Plan 9 user but leaves host credential policy to the fsdev. Map qid paths as directory bit 31, seven device-map bits, and 24 inode bits; reject device-map exhaustion. Derive `qid.vers` from modification time.

- [ ] **Step 4: Verify and commit the boot subset**

```bash
ninja -C build tests/unit/test-plan9-9p1-server
build/tests/unit/test-plan9-9p1-server
git add hw/9pfs/plan9-9p1-server.* hw/9pfs/meson.build tests/unit
git commit -m "9pfs: serve Plan 9 9P1 read operations"
```

### Task 7: Complete writable 9P1 namespace operations

**Files:**
- Modify: `hw/9pfs/plan9-9p1-server.c`
- Modify: `tests/unit/test-plan9-9p1-server.c`

- [ ] **Step 1: Add a failing writable lifecycle**

Exercise `Tcreate`, `Twrite`, `Twstat`, `Tremove`, and their responses in one sequence: create a file, write across two offsets, stat it, truncate/rename/mode-update through wstat, reopen/read it, remove it, and prove the host export reflects the result. Add directory creation, invalid open modes, attempts to mutate the root, remove-open-fid cleanup, read-only export rejection, offset/count overflow, short writes, and error-string truncation to 64 bytes.

- [ ] **Step 2: Implement create/write/wstat/remove**

Translate old open bits to host flags explicitly. Use backend operations (`open2`, `pwritev`, `rename`, `chmod`/`chown`, `truncate`, `unlinkat`/`remove`) rather than direct host syscalls. Preserve old-protocol semantics: successful remove clunks the fid, stat length encodes a 32-bit value plus four zero bytes, and reply fid fields echo the request where the protocol requires them.

- [ ] **Step 3: Run server and codec suites under sanitizers where available**

```bash
ninja -C build tests/unit/test-plan9-9p1-server tests/unit/test-plan9-9p1-codec
build/tests/unit/test-plan9-9p1-server
build/tests/unit/test-plan9-9p1-codec
```

Expected: every writable and malformed case passes without leaks, invalid accesses, or pending coroutine work.

- [ ] **Step 4: Commit writable support**

```bash
git add hw/9pfs/plan9-9p1-server.c tests/unit/test-plan9-9p1-server.c
git commit -m "9pfs: complete writable Plan 9 9P1 service"
```

### Task 8: Expose `plan9-9p1-server` through QOM/QAPI and pin libslirp

**Files:**
- Modify: `hw/9pfs/plan9-9p1-server.h`
- Modify: `hw/9pfs/plan9-9p1-server.c`
- Modify: `hw/9pfs/meson.build`
- Modify: `qapi/qom.json`
- Modify: `subprojects/slirp.wrap`
- Create: `tests/qtest/plan9-9p1-object-test.c`
- Modify: `tests/qtest/meson.build`
- Modify: `docs/system/devices/9p.rst`

- [ ] **Step 1: Add failing QOM/QMP lifecycle tests**

Cover command-line creation after `-fsdev`/`-netdev`, QMP `object-add`/`object-del`, default `guest-address=10.0.2.100`, default `port=564`, missing properties, missing fsdev, missing/non-user netdev, invalid/out-of-subnet/reserved address, duplicate endpoint, backend-init unwind, active-connection delete safety, and recreation after delete.

- [ ] **Step 2: Add the generated QAPI branch**

Add:

```json
{ 'struct': 'Plan9P1ServerProperties',
  'data': { 'fsdev': 'str', 'netdev': 'str',
            '*guest-address': 'str', '*port': 'uint16' } }
```

and the `'plan9-9p1-server': 'Plan9P1ServerProperties'` branch to `ObjectOptions`. Do not edit generated build files.

- [ ] **Step 3: Implement `UserCreatable` completion and teardown**

Create the QOM type with `fsdev`, `netdev`, `guest-address`, and `port` properties. In `complete()`, validate properties, initialize `V9fsBackend`, register the callback endpoint, configure Plan 9 BOOTP, then create the protocol server. Unwind in reverse order on failure. In `prepare_delete()`, reject only while coroutine work is genuinely pending; otherwise stop acceptance of new bytes, safely remove the SLiRP endpoint, clunk fids, and clean the backend. Make `finalize()` idempotent.

- [ ] **Step 4: Pin the exact public libslirp commit and document usage**

Replace `revision` in `subprojects/slirp.wrap` with the exact remote hash produced by Task 2. Document that the service is guest-only and requires a named user netdev and fsdev.

- [ ] **Step 5: Verify and commit QEMU integration**

```bash
ninja -C build qemu-system-m68k tests/qtest/plan9-9p1-object-test tests/qtest/slirp-guestfwd-test
build/tests/qtest/plan9-9p1-object-test
build/tests/qtest/slirp-guestfwd-test
git add qapi/qom.json hw/9pfs include/net net subprojects/slirp.wrap tests docs/system/devices/9p.rst
git commit -m "9pfs: expose guest-only Plan 9 9P1 root service"
```

### Task 9: Stage a safe writable Plan 9 root tree in the lab

**Files:**
- Modify: `nextcube_lab/plan9_netboot.py`
- Modify: `tests/test_plan9_netboot.py`

- [ ] **Step 1: Add failing extraction and argv tests**

Test full fixture extraction without its `plan9-2e/` prefix; reject absolute paths, `..`, prefix escapes, duplicates, symlinks, hard links, devices, FIFOs, sockets, excessive members, excessive expanded bytes, and oversized members. Prove the source archive is unchanged, executable modes survive, destinations are writable, and root identity is deterministic. Assert exact arguments:

```text
-fsdev local,id=plan9root,path=<rootfs>,security_model=none
-netdev user,id=nextnet,ipv6=off,tftp=<tftp>,bootfile=68020/9nextstation
-object plan9-9p1-server,id=plan9fs,fsdev=plan9root,netdev=nextnet,guest-address=10.0.2.100,port=564
-net nic,model=next-mb8795,netdev=nextnet
```

- [ ] **Step 2: Implement `RootTreeIdentity` and confined extraction**

```python
@dataclass(frozen=True)
class RootTreeIdentity:
    path: Path
    sha256: str
    file_count: int
    byte_count: int

def stage_rootfs(archive: Path,
                 spec: ReleaseSpec,
                 rootfs: Path) -> RootTreeIdentity:
    return extract_verified_release_tree(archive, spec, rootfs)
```

Call `verify_release_archive()` first, manually copy regular files with exclusive creation, create directories with mode `0700` during extraction, then apply preserved executable/read mode bits plus owner write permission. Hash a canonical sequence of relative path, type, mode, size, and file digest. Do not use `extractall()`.

- [ ] **Step 3: Extend `build_plan9_argv()`**

Require absolute existing QEMU, ROM, TFTP, and rootfs paths; retain GTK display, named QMP socket, PCAP, TFTP, and NeXT NIC. Add the fsdev and 9P1 object exactly once.

- [ ] **Step 4: Verify and commit lab staging**

```bash
python3 -B -m unittest tests.test_plan9_netboot -v
git add nextcube_lab/plan9_netboot.py tests/test_plan9_netboot.py
git commit -m "lab: stage writable Plan 9 root exports"
```

### Task 10: Integrate the authenticated visible Plan 9 controller

**Files:**
- Modify: `nextcube_lab/baseline/model.py`
- Modify: `nextcube_lab/baseline/runner.py`
- Modify: `nextcube_lab/baseline_cli.py`
- Modify: `tests/test_baseline_runner.py`

- [ ] **Step 1: Add failing lifecycle and gate tests**

With `FakeQMP`, `QueuedCommandSource`, and `tests/helpers/fake_qemu.py`, prove the runner stages TFTP/rootfs inside a mode-0700 unique run directory, records archive/rootfs/kernel hashes and exact argv, authenticates peer PID and socket identity, sends the ROM command once, sends `tcp` only after the visible root-method gate, handles QMP loss safely, and stops only its own child through QMP then exact-child fallback.

- [ ] **Step 2: Reuse the baseline owned-process path**

Add a closed `plan9-netboot` profile to the existing runner rather than creating a second `subprocess.Popen` or QMP client. Reuse `OwnedProcess.launch`, `_wait_for_qmp_socket`, `QMPClient.connect`, `InteractiveController`, transcripts, screenshots, registers, PCAP, and manifest finalization.

- [ ] **Step 3: Resolve and freeze the ROM command spelling**

Use the command proven by the visible ROM monitor and current successful TFTP run. The existing helper says `ben() 68020/9nextstation`, while an older written plan says `ben 68020/9nextstation`; verify against retained QMP transcript evidence and set exactly one spelling in `rom_boot_command()` and its test.

- [ ] **Step 4: Verify and commit controller integration**

```bash
python3 -B -m unittest tests.test_baseline_runner tests.test_plan9_netboot -v
python3 -B -m unittest discover -s tests -v
git add nextcube_lab tests
git commit -m "lab: run authenticated Plan 9 root-mount acceptance"
```

### Task 11: Run complete non-headful regression verification

**Files:**
- No production edits unless a regression exposes a defect

- [ ] **Step 1: Verify public libslirp**

```bash
meson test -C build --print-errorlogs
git status --short
```

- [ ] **Step 2: Verify QEMU build and focused suites**

```bash
ninja -C build qemu-system-m68k
build/tests/unit/test-plan9-9p1-codec
build/tests/unit/test-plan9-9p1-server
build/tests/qtest/slirp-guestfwd-test
build/tests/qtest/plan9-9p1-object-test
build/tests/qtest/next-fb-test
build/tests/qtest/next-dma-test
build/tests/qtest/next-color-video-test
build/tests/qtest/next-machine-test
build/tests/qtest/next-mb8795-test
build/tests/qtest/next-cube-timer-test
```

Expected: all focused tests pass; the established NeXT matrix still totals 81 passing subtests before adding the new suites.

- [ ] **Step 3: Verify the complete lab suite and source hygiene**

```bash
python3 -B -m unittest discover -s tests -v
git diff --check
git status --short
```

Keep pre-existing untracked result files and build directories untouched. Do not stage them.

### Task 12: Perform the GTK-visible Second Edition acceptance

**Files:**
- Create: a unique ignored run under `lab/work/plan9-runs/`
- Create after success: a non-overwriting JSON/Markdown pair under `lab/results/plan9/`

- [ ] **Step 1: Launch visibly and stop at the ROM prompt**

Start the authenticated Plan 9 profile with GTK. Capture the ROM prompt, registers, QMP socket identity, argv, hashes, and initial screenshot. Do not inject the boot command until Bryce explicitly confirms the prompt is visible.

- [ ] **Step 2: Boot the kernel and stop at the root-method prompt**

After confirmation, inject the exact one-shot ROM boot command. Capture TFTP packets, progress beyond memory statistics, and the visible `root is from (il, tcp)[il]:` prompt. Do not inject `tcp` until Bryce explicitly confirms that prompt.

- [ ] **Step 3: Mount root and reach userspace**

After confirmation, inject `tcp`. Capture the `p9  ` BOOTP exchange, guest TCP connection to `10.0.2.100:564`, 9P1 request/response activity, successful root mount, execution of `/68020/init`, and the desktop or normal first-login screen.

- [ ] **Step 4: Shut down only through authenticated ownership**

Request `quit` over the run's authenticated QMP connection. Allow `OwnedProcess.shutdown()` to use exact-child TERM/KILL fallback only if that owned child ignores QMP. Never signal any other QEMU.

- [ ] **Step 5: Promote evidence and record exact revisions**

Promote a non-overwriting result pair containing the QEMU commit, public libslirp pin, lab commit, archive/kernel/rootfs identities, command line, QMP transcript, screenshot checksums, PCAP checksum, and visible-gate outcomes.

### Task 13: Final review, integration, and publication

**Files:**
- All changed files in the three repositories

- [ ] **Step 1: Request a whole-change specification review**

Check every design requirement against the final diffs and visible evidence, including no host listener, complete writable operation set, 2E rather than 1E message numbering, BOOTP isolation, coroutine/backpressure behavior, teardown safety, licensing, and process ownership.

- [ ] **Step 2: Request a whole-change quality review**

Review memory ownership, coroutine cancellation, fid transitions, integer bounds, path confinement, QOM unwind, generated QAPI compatibility, test determinism, and documentation.

- [ ] **Step 3: Merge and publish QEMU**

After clean verification, fast-forward or merge `feature/plan9-9p1-root` into QEMU `metachicken`, then:

```bash
git -c core.sshCommand='ssh -F /dev/null' push github metachicken
git ls-remote github refs/heads/metachicken
```

Confirm the remote hash exactly matches local `metachicken`.

- [ ] **Step 4: Merge lab locally and report its remote blocker accurately**

Merge `feature/plan9-next-netboot` into lab `metachicken`. Run the full 690-test-plus-new-tests suite from the merged checkout. The lab repository currently has no remote; do not claim it is pushed unless a remote is configured and verified.

## Completion criteria

Completion requires all of the following:

1. Public libslirp branch contains and publishes safe callback-forward teardown plus Plan 9 BOOTP support.
2. QEMU pins that public commit and exposes a documented `plan9-9p1-server` object without a host listener.
3. Exact 2E codec, filesystem, QOM, SLiRP, and regression tests pass.
4. The complete lab suite passes with safe root extraction and authenticated process control.
5. Bryce witnesses the visible ROM and root-method gates before input is injected.
6. The archived Second Edition kernel mounts the staged matching tree and executes `/68020/init` to the desktop or normal first-login state.
7. QEMU is merged and pushed with local/remote hashes verified; lab is merged and either pushed to a configured remote or explicitly reported as local-only.
