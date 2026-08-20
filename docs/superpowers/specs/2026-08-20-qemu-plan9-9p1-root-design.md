# Native Plan 9 9P1 Root Service Design

## Goal

Boot the archived Plan 9 First and Second Edition NeXT kernels from TFTP,
configure their network through BOOTP, mount an exported Plan 9 tree over the
9P1 protocol, and continue into `/68020/init` without an external file-server
daemon or a privileged host TCP port.

The first acceptance target is the Second Edition `9nextstation` kernel and a
writable, per-run extraction of the matching Second Edition archive. The
design must not change the existing NeXTSTEP disk-boot path or QEMU's existing
9P2000.u and 9P2000.L implementations.

## Chosen architecture

QEMU will gain a user-creatable `plan9-9p1-server` object in `hw/9pfs`. The
object references an existing `-fsdev` export and a user-mode `-netdev`. It
registers a guest-only TCP endpoint with that SLiRP instance; it does not open
a host socket.

An invocation will have this shape:

```text
-fsdev local,id=plan9root,path=/run/rootfs,security_model=none
-netdev user,id=nextnet,tftp=/run/tftp,bootfile=68020/9nextstation
-object plan9-9p1-server,id=plan9fs,fsdev=plan9root,netdev=nextnet,
        guest-address=10.0.2.100,port=564
-net nic,model=next-mb8795,netdev=nextnet
```

`plan9-9p1-server` registers two services on `nextnet`:

1. A callback-backed SLiRP guest forward for `10.0.2.100:564`. Bytes received
   from the guest enter the 9P1 stream decoder, and encoded replies return
   through the same SLiRP connection.
2. Plan 9 BOOTP vendor information naming the guest-visible file server,
   netmask, disabled authentication server, and gateway.

This keeps the network and filesystem boundaries separate. The 9P1 server
depends on a small, generic internal SLiRP guest-forward interface rather than
reaching into libslirp state. No listener is exposed on the host, port 564 does
not require privilege, and the server remains usable by any guest with a 9P1
TCP client.

## Component boundaries

### QEMU SLiRP adapter

`net/slirp.c` will expose an internal registration API that:

- resolves a named user-mode netdev;
- rejects non-SLiRP netdevs and duplicate guest address/port pairs;
- forwards guest-to-server bytes through a receive callback;
- reports available reply capacity and sends server-to-guest bytes;
- removes the forwarding entry when the owning object is finalized; and
- configures the Plan 9 BOOTP vendor values for that SLiRP instance.

The public libslirp API already supplies callback-backed guest forwarding.
QEMU's adapter will own the opaque libslirp handle and keep `SlirpState`
private to `net/slirp.c`.

### 9P1 codec

The codec is independent of filesystem operations. It decodes and encodes the
Second Edition wire format, including:

- the type/fid/tag header used before 9P2000;
- little-endian integer fields;
- fixed 28-byte names and 64-byte error strings;
- the old 32-bit-path/32-bit-version `Qid`;
- fixed 116-byte directory records;
- the variable payloads of `Twrite` and `Rread`; and
- all 9P1 messages needed for a writable root: nop, flush, session, attach,
  clone, walk, open, create, read, write, clunk, remove, stat, wstat, and
  clwalk.

The stream decoder accepts arbitrarily fragmented input and multiple requests
in one callback. It determines a message's complete length from its type and,
where applicable, its encoded count. The maximum data payload is 8192 bytes,
matching the historical protocol.

The protocol definitions and golden vectors come from the published Second
Edition `sys/src/cmd/unix/u9fs/9p.h` and `conv.c`. The implementation will be
independent rather than copied from the historical server.

### 9P1 filesystem server

The server owns a 16-bit fid table and translates decoded requests to QEMU's
existing `FsContext` and `FileOperations` backend. It does not route 9P1
messages through the 9P2000 parser, because their framing and request semantics
are materially different.

The historical qid layout will be preserved: the high bit marks directories,
seven bits identify the host device within the export, and the low 24 bits
identify the inode. `qid.vers` derives from modification time. A per-server
device map detects exhaustion rather than silently aliasing devices.

`Tsession` resets connection-local protocol state and returns zeroed legacy
authentication fields. With the BOOTP authentication address set to
`0.0.0.0`, the Second Edition boot program does not perform a separate auth
exchange. `Tattach` accepts the Plan 9 user name while filesystem access follows
the selected QEMU fsdev security model. The initial launcher uses
`security_model=none` and a private per-run tree.

Filesystem work that may block runs through QEMU's coroutine/cofile machinery;
the SLiRP receive callback only frames requests and schedules service work.
Each queued operation holds a QOM reference to its server until its completion
callback has retired the operation. Replies remain ordered on the single 9P1
connection; a partially delivered reply retains its byte offset and resumes
only after the SLiRP adapter reports fresh receive capacity.

### Plan 9 BOOTP support in libslirp

The public `blanham/libslirp` fork will gain an opt-in Plan 9 BOOTP response.
When a request carries the four-byte vendor prefix `p9  `, the reply will carry
exactly four space-separated values after that prefix:

```text
255.255.255.0 10.0.2.100 0.0.0.0 10.0.2.2
```

These are, in order, the mask, filesystem server, authentication server, and
gateway expected by the Second Edition boot program. Addresses are derived
from the configured SLiRP network and the server object rather than hard-coded
inside the BOOTP packet builder. The existing NeXT ROM vendor response and
ordinary BOOTP/DHCP behavior remain unchanged.

QEMU will pin the resulting public libslirp commit in `subprojects/slirp.wrap`.

### Lab launcher

The lab launcher will prepare a unique run directory containing:

- the verified TFTP kernel;
- a writable extraction of the matching Plan 9 release as `rootfs/`;
- the exact QEMU binary and source commit identifiers;
- the archive and staged-kernel checksums;
- a named QMP socket and controller ownership record; and
- packet capture, QMP transcript, and screenshots.

It will pass the `-fsdev` and `plan9-9p1-server` options automatically. The
NeXT kernel still prompts for the root method because its built-in boot
arguments contain only the kernel path. Acceptance therefore injects `tcp`
through authenticated QMP after the prompt is visibly confirmed.

## Lifecycle and error handling

- QEMU startup fails with a precise error if the fsdev is missing, the netdev
  is not SLiRP, the guest address is outside the virtual network, or the
  address/port is already registered.
- Unknown message types are fatal framing errors, matching the historical
  stream module, and reset the stream session. Impossible lengths and payloads
  larger than 8192 bytes produce an old-protocol `Rerror` when a valid known
  type and tag can be recovered, then reset the stream. Invalid fid
  transitions produce `Rerror` without resetting the connection.
- A new `Tsession` discards stale fids and pending connection-local state, so a
  reconnect starts cleanly even though libslirp's legacy callback API has no
  explicit connection-open notification.
- Path traversal and symlink handling remain confined by the selected QEMU
  fsdev backend. The 9P1 layer never constructs an unchecked host path.
- Every pending coroutine holds a server reference, so object memory and the
  backend remain live until its completion retires. Explicit object deletion
  is refused while work is pending. Once idle, teardown unregisters the guest
  forward, clunks open fids, and frees buffers/backend state in that order.
- Live 9P1 TCP connections are not migrated in the first version. After
  migration or reconnect, the guest must establish a new session. Starting a
  VM with the same object configuration recreates the service endpoint.
- Only controller-owned QEMU instances may be stopped, and only through their
  authenticated QMP socket. No launcher or test may signal an unrecognized
  process.

## Testing

### Codec tests

Unit tests will compare every supported request and response against fixed
9P1 byte vectors. They will cover fragmented headers, fragmented data,
coalesced requests, maximum-size payloads, malformed counts, and endian
conversion.

### Server tests

A synthetic stream transport and temporary fsdev export will exercise the
complete writable sequence: session, attach, clone/walk, open/read, create,
write/stat, clunk, and remove. Tests will cover unknown fids, duplicate fids,
directory record encoding, permission errors, path confinement, reconnect via
`Tsession`, reply ordering, and transport backpressure.

### Network and BOOTP tests

Libslirp tests will prove that:

- a `p9  ` request receives the four required Plan 9 vendor fields;
- the reply retains the correct transaction and client hardware identifiers;
- the filesystem and gateway addresses follow configuration;
- NeXT ROM BOOTP tests remain green; and
- ordinary DHCP/BOOTP behavior is unchanged.

QEMU tests will verify registration failures, teardown, and byte flow across
the internal SLiRP guest-forward adapter without opening a host listener.

### Visible acceptance

The final acceptance run is headful GTK and pauses at each human-visible gate.
Using the Second Edition kernel, success requires:

1. ROM TFTP load of `68020/9nextstation`;
2. progress beyond the former memory-statistics stall;
3. selection of `tcp` at the root-method prompt;
4. automatic Plan 9 BOOTP configuration without manual IP questions;
5. a successful TCP/564 connection and root mount;
6. execution of `/68020/init`; and
7. arrival at the Plan 9 desktop or its normal first-login state.

The First Edition kernel is a follow-up compatibility run against the same
9P1 service.

## Licensing

New standalone QEMU 9P1 and adapter files will use the NCSA license with Bryce
Lanham's copyright. Small changes to existing GPL QEMU 9pfs files retain those
files' existing licenses. Changes to libslirp retain libslirp's existing
license. The historical Plan 9 sources are protocol references and test
oracles; their implementation is not copied into QEMU.

## Out of scope

- Serving 9P1 on an externally reachable host TCP socket.
- Implementing the IL transport or a Plan 9 authentication server.
- Replacing or removing QEMU's Virtio/Xen 9P2000 transports.
- Migrating an active TCP/9P1 session.
- Modifying the archived Plan 9 kernel solely to bypass the root-method
  prompt.
