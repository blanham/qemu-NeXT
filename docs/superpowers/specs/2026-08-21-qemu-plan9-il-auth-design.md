# Native Plan 9 IL and Authentication Design

## Goal

Extend the existing NeXT Plan 9 root service so an unmodified Second Edition
kernel can use its historical authenticated network path: IL over IPv4 protocol
40, ticket authentication on IL port 566, and 9P1 file service on IL port
17008. Preserve the current unauthenticated TCP/564 path as the default
compatibility mode.

The acceptance target is a private QEMU user-mode network. No IL or ticket
service may listen on a host socket or become reachable outside that virtual
network.

## Chosen architecture

The work is one integration with three ordered tracks:

1. The public `blanham/libslirp` fork implements IL as a real reliable,
   record-oriented IPv4 protocol and exposes listener/connection APIs.
2. QEMU adds a narrow SLiRP IL adapter, authentication/key-database helpers,
   and an authenticated IL mode to the existing `plan9-9p1-server` object.
3. The NeXT lab provisions private key material, launches the authenticated
   profile, and proves both rejection and success while retaining packet and
   visible-boot evidence.

The existing TCP callback guest-forward remains unchanged. IL is not tunneled
through TCP, translated into TCP, or exposed through `hostfwd`.

Two alternatives were rejected. A second Plan 9 authentication/fileserver VM
would be historically purer but would add another bootable image, lifecycle,
and trust boundary to every run. A raw host socket or TAP daemon would require
privileges or host networking and would defeat the self-contained SLiRP lab.

## Historical protocol contract

The Second Edition kernel implementation in `sys/src/9/port/stil.c` is the
compatibility authority when its behavior differs from the older prose in
`sys/doc/il/il.ms`.

IL packets use IPv4 protocol 40 and an 18-byte header containing, in order, a
big-endian Internet checksum, total IL length, type, zero specifier, source
port, destination port, sequence ID, and acknowledgement ID. The checksum
covers the IL header and payload with the checksum and specifier zeroed; an
odd final byte is padded with zero for calculation. The supported types are
sync, data, dataquery, ack, query, state, and close.

Each application write is one IL data record. The implementation must preserve
that boundary from the sender API through retransmission and into the receiver
callback. It must never present IL as an arbitrary byte stream.

The state machine must reproduce the historical three-way sync, duplicate-sync
handling, cumulative acknowledgements, strictly ordered delivery, a 20-record
receive window, go-back-N recovery through dataquery/state, query/state idle
probing, and close retransmission. Initial sequence values are limited to 24
bits, matching the guest. The compatibility timers are:

- delayed acknowledgement after 200 ms when it cannot be piggybacked;
- first retransmission after 400 ms, followed by the historical backoff;
- connection failure after 35 seconds without progress; and
- idle query after 60 seconds, then six-second probes until the historical
  unanswered-query budget is exhausted.

IPv4 fragmentation is reassembled by libslirp before IL validation. A bad
length, checksum, specifier, state transition, duplicate/out-of-window record,
or packet without a matching listener is discarded without exposing data to
QEMU. Bounds cover every receive queue, send queue, connection table, and
record allocation.

## Public libslirp IL API

Libslirp owns all wire state: sequence numbers, checksums, windows, queues,
retransmissions, and timers. Its public API exposes opaque listener and
connection handles plus callbacks for connection establishment, one complete
received record, renewed send capacity, and connection closure.

Listener registration binds a guest-visible IPv4 address and IL port within a
specific `Slirp` instance. An establishment callback supplies a distinct
application context for each accepted connection. Application sends are
atomic: a complete record is either accepted for reliable delivery or rejected
with a bounded error such as not-established, would-block, or message-too-big.
Partial record acceptance is forbidden. Removing a listener prevents new
connections, closes existing connections, and is safe when requested from an
IL callback.

One stack-wide IL timer, integrated through libslirp's existing timer callback
contract, schedules the earliest connection deadline. This avoids a host
thread and keeps all callbacks on QEMU's network/main-loop context. The public
symbol map and API version are updated without changing existing callers.

Unit tests use deterministic packet injection, a controllable monotonic clock,
and captured output packets. They cover the handshake, checksum vectors,
record boundaries, delayed ACKs, loss, reorder, duplicate sync/data, window
limits, dataquery/state recovery, idle probes, close, callback-time listener
removal, queue exhaustion, and multiple independent connections. Existing
BOOTP, TCP, UDP, ICMP, and callback guest-forward tests remain green.

## QEMU SLiRP IL adapter

QEMU adds an internal adapter parallel to `net/slirp-guestfwd.c`. It resolves a
named user-mode netdev, registers an IL listener through public libslirp, and
wraps libslirp listener/connection handles in QEMU-owned opaque objects. The
adapter exposes only record receive, atomic record send, send-ready, open, and
close operations. `SlirpState` remains private to `net/slirp.c`.

The adapter validates that IPv4 is enabled, the service address lies inside
the virtual subnet, and no listener already owns the address/port tuple. Its
reference and callback-depth rules mirror the hardened TCP guest-forward
adapter: callbacks may initiate removal, but final detach is deferred until
dispatch unwinds. SLiRP teardown invalidates listeners and connections before
freeing their QEMU owners.

Tests use a fake libslirp backend to prove registration conflicts, record
atomicity, backpressure, connection isolation, reentrant removal, and netdev
teardown. No adapter operation opens a host file descriptor.

## QEMU object modes and 9P1 transport

`plan9-9p1-server` gains an explicit `transport` property:

- `tcp` remains the default. It retains the existing guest callback forward,
  port 564, zero `Rsession.authid`, and unauthenticated `Tattach` behavior.
- `il` registers the historical file listener on `il-port` (default 17008)
  and requires the authentication configuration below. `auth-port` defaults
  to 566. The existing `port` property continues to describe TCP only.

The 9P1 server transport contract is extended to distinguish stream and record
transports. TCP continues to accept fragmented/coalesced frames and partially
send replies. IL ingress must contain exactly one complete 9P1 request per
record, and each encoded 9P1 reply is submitted as one atomic IL record. An IL
record containing a partial frame, trailing second frame, or oversized payload
is a protocol error and closes that IL connection.

The object accepts one active file-service connection at a time, which matches
the single guest supplied by a QEMU user network. A clean IL close or a new
connection after teardown resets fids, challenges, authenticator replay state,
request queues, and reply queues. Concurrent file connections are rejected at
the transport boundary rather than sharing connection-local 9P state. The
ticket listener uses short-lived independent connections and remains available
while the file connection is active.

## Authentication and ticket service

IL mode requires four additional object properties:

- `auth-id`: the fixed 28-byte-compatible file-server principal;
- `auth-domain`: the fixed 48-byte-compatible authentication domain;
- `keydb`: the path to the encrypted native `/adm/keys` database; and
- `key-secret`: the ID of a QEMU Secret containing the seven-byte database
  master key.

At object completion, QEMU resolves the Secret as binary data, requires exactly
seven decoded bytes, opens the database read-only without following a symlink,
and loads complete 41-byte records. Each record is independently decrypted in
the native Plan 9 format and decoded as `name[28], key[7], status, warnings,
expiry_le32`. Duplicate names, invalid status values, partial records, a
missing/disabled/expired server identity, or an empty database fail startup.
The database is immutable for the life of the object; reprovisioning requires
a new QEMU run.

The IL/566 service accepts exactly one 141-byte `AuthTreq` record and returns
one 145-byte response: `AuthOK`, a client ticket encrypted with the requesting
host principal's key, and a server ticket encrypted with `auth-id`'s key. Both
tickets contain the request challenge and the same freshly generated seven-byte
conversation key. The minimal speaks-for policy permits only `hostid == uid`;
all other requests receive tickets whose server user is `none`. Unsupported
request types and malformed records receive a fixed `AuthErr` where the
historical client can consume one safely, then the connection closes.

Missing, disabled, or expired principals follow the historical privacy rule:
the service substitutes a random key and emits an otherwise normal-sized
reply, so the caller cannot distinguish which lookup failed. Logs identify
only the class of failure and peer address, never principal keys, passwords,
master keys, conversation keys, tickets, authenticators, or decrypted record
bytes.

In IL mode, `Tsession` resets connection-local state, stores the client's
challenge, generates a fresh server challenge, and returns `auth-id` and
`auth-domain`. `Tattach` must:

1. decrypt the `AuthTs` ticket with the file-server key;
2. require the ticket challenge to equal the current server challenge;
3. require the ticket client name to equal the attach user;
4. decrypt and validate `AuthAc` with the ticket's conversation key;
5. require its challenge to match and its ID to be fresh in the historical
   32-entry replay window; and
6. return `AuthAs`, encrypted with the conversation key, containing the
   client's `Tsession` challenge and the accepted authenticator ID.

Only after all checks pass may the fid attach to the exported root. Failure
returns a generic old-protocol authentication error and leaves no fid behind.
The translated server user from the ticket becomes the authenticated Plan 9
identity; the selected fsdev continues to define host-side credential mapping.
TCP mode never enters this code path.

The password-to-key conversion, packed-key expansion, ticket/authenticator
serialization, and variable-length Plan 9 encryption are implemented from the
published behavior and verified against golden vectors produced by the staged
Second Edition sources. Historical source is an interoperability oracle, not
copied implementation text. All temporary plaintext key material is explicitly
zeroed before release.

## Native key provisioning and secret boundary

A local `qemu-plan9-keydb` helper shares the QEMU authentication/key-database
implementation. Its `create` command receives output paths and principal names
as ordinary arguments, prompts for the `tor` password twice on `/dev/tty`
without echo, derives the exact historical seven-byte password key, and
generates random keys for the database master and file-server identity.

The helper atomically creates a two-record encrypted `/adm/keys` database for
`tor` and the configured file-server identity, plus a base64 master-key file.
It refuses existing destinations, symlinks, duplicate/overlong principals,
unrepresentable passwords, and non-private parent directories. The parent
directory is mode 0700 and both files are mode 0600; writes are flushed before
rename. It prints paths and fingerprints only, never key or password values.

The authenticated launcher passes only:

```text
-object secret,id=plan9keymaster,format=base64,file=/private/run/adm-keys.master
-object plan9-9p1-server,...,transport=il,keydb=/private/run/adm-keys,
        key-secret=plan9keymaster,auth-id=p9fs,auth-domain=nextlab
```

The master key is never placed in argv, an environment variable, QMP, a log,
Git, result metadata, screenshots, or packet captures. The launcher validates
ownership, regular-file identity, and restrictive permissions before starting
QEMU. It does not copy the secret into an evidence bundle. QEMU looks up and
copies the Secret during object completion, then zeroes the temporary lookup
buffer; the Secret object and the key database remain read-only.

## BOOTP and guest boot flow

The existing Plan 9 BOOTP structure already carries netmask, file-server,
authentication-server, and gateway addresses. TCP mode continues advertising
an all-zero authentication address. IL mode advertises the guest-visible IL
service address for both file and authentication servers.

The authenticated boot flow is:

1. ROM BOOTP/TFTP loads the verified Second Edition kernel.
2. Plan 9 BOOTP learns both service addresses.
3. The guest opens IL/17008 and sends `Tsession`.
4. The file service returns nonzero authentication identity/domain fields and
   a fresh challenge.
5. The guest opens IL/566, submits `AuthTreq`, and receives both tickets.
6. The password-derived `tor` host key decrypts the client ticket locally.
7. The guest submits the server ticket and `AuthAc` in `Tattach`.
8. QEMU validates them, returns `AuthAs`, mounts root, and continues through
   `/68020/init` to the terminal/desktop.

An incorrect password cannot decrypt a valid client ticket. The guest's
fallback attach still lacks a server-valid ticket/authenticator and is rejected
by QEMU, so root must not mount.

## Lab profile and acceptance evidence

The lab gains a distinct authenticated Plan 9 profile rather than changing the
current `plan9-netboot` contract. It stages the verified Second Edition kernel
and writable root exactly as today, requires pre-provisioned private key files,
uses GTK and authenticated QMP control, selects `il` at the guest root-method
prompt, and retains the existing ownership-safe shutdown rules.

Acceptance is two ordered runs against the same immutable key database:

1. Enter a known incorrect password. Prove that authentication fails and root
   is not mounted.
2. Relaunch from a clean run directory, enter the correct `tor` password, and
   prove that root mounts and Plan 9 reaches its normal terminal/desktop.

Each run captures the visible gates, QMP transcript, hashes, and a network
capture. A bounded parser must find IPv4 protocol 40 traffic to both IL port
566 and IL port 17008, validate IL lengths/checksums, and report only packet
metadata. It must not decode or publish ticket/authenticator payloads. The
promoted success summary records both run IDs and refuses success unless the
negative and positive assertions, both ports, and final visible gate all pass.

Unit tests cover argument construction, permission/identity checks, secret
exclusion from metadata and transcripts, password-entry sequencing, negative
and positive gate classification, IL PCAP parsing, and bundle publication.

## Documentation and local-only notes

`QEMU-NeXT-README.md` will document the OSUOSL history archive URL
`https://ftp.osuosl.org/pub/plan9/history/`, the native TCP compatibility
launch, key provisioning, the authenticated IL launch, the private-network
limitation, and the two-run acceptance command.

The hardware follow-up list belongs in `qemu/TODO.md` but remains local-only.
The root QEMU worktree's `.git/info/exclude` will contain `/TODO.md`; the file
must never be staged or committed. Because Git's info exclude is shared by all
QEMU worktrees, this is performed once and verified with `git check-ignore`.

## Error handling and lifecycle

- IL or authenticated object configuration fails before machine start if the
  required libslirp API is unavailable or any address, port, key, database, or
  principal contract is invalid.
- Runtime protocol errors close only the offending IL connection. They do not
  terminate QEMU or affect TCP compatibility mode.
- Pending 9P backend work retains the QOM server exactly as in the current
  implementation. Object deletion is refused until backend work drains; IL
  listener teardown then prevents callbacks before authentication and 9P
  state are zeroed and freed.
- Ticket requests, 9P requests, and queued IL records have fixed cardinality
  and byte limits. Exhaustion applies backpressure or closes the peer; it never
  grows an unbounded queue.
- Live IL state is not migrated. A destination recreates listeners from object
  configuration, and the guest must establish a new authenticated session.

## Verification gates

Implementation is complete only when all of the following pass:

1. libslirp IL unit tests, existing libslirp tests, and ABI/symbol checks;
2. QEMU IL-adapter, key-database, crypto-vector, ticket-service, 9P1 auth, QOM,
   and existing 9P1/TCP tests;
3. lab unit tests proving secure launch and evidence behavior;
4. a regression run of the existing unauthenticated TCP/564 profile;
5. the authenticated wrong-password failure run;
6. the authenticated correct-password root mount and terminal/desktop run; and
7. capture validation showing protocol 40 with ports 566 and 17008.

## Licensing

New libslirp files retain libslirp's license. New standalone QEMU adapter and
9P1/authentication files use the same NCSA license and Bryce Lanham attribution
as the existing 9P1 service where their dependencies allow it; modifications
to GPL/LGPL QEMU components retain their existing headers. Historical Plan 9
source supplies protocol descriptions and test oracles only. No archived
implementation is copied into the public forks.

## Out of scope

- Modifying the archived Plan 9 kernel or root tree to bypass authentication.
- Exposing IL, 9P1, or the ticket service through a host listener.
- Supporting AuthChal, password changes, general `speaksfor` databases, or
  administration of the key database while QEMU is running.
- Replacing the current TCP/564 compatibility path.
- Migrating live IL or 9P1 connections.
- Treating the obsolete DES protocol as suitable outside this isolated
  historical lab.
