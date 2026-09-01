# Embedded NetInfo Server Design

## Goal

Add a QEMU-hosted NetInfo service that lets an unmodified NeXTSTEP guest bind
to a read-only domain and complete boot.  The first useful release is minimal,
but its transport, protocol, database, and persistence boundaries must support
a complete mutable NetInfo implementation later.

This is also the first use of reusable embedded retro-network-service
infrastructure.  It must coexist with the existing embedded NFS server and
leave room for additional services without creating a monolithic service
object.

## User interface and safety defaults

Each service remains an independent QOM object attached to one user-mode
network backend:

```text
-netdev user,id=retro0
-object netinfo-server,id=ni0,netdev=retro0
```

The zero-configuration form supplies a deterministic, minimal, read-only
domain.  An optional versioned JSON5 seed describes a custom domain:

```text
-object netinfo-server,id=ni0,netdev=retro0,config=/path/domain.json5
```

Mutation is enabled only when both `writable=on` and a state location are
provided:

```text
-object netinfo-server,id=ni0,netdev=retro0,config=/path/domain.json5,\
         writable=on,state=/path/state
```

The service is reachable only by guests inside the selected SLiRP network.  It
does not open a host listener unless the user separately configures forwarding.
Each `-netdev user` receives an isolated service instance.  Sharing one domain
between QEMU processes is deferred.

## Architecture

The existing NFS object privately owns portmapper behavior.  That cannot scale
to multiple ONC RPC services, so the first change is a shared registry owned by
each SLiRP netdev:

```text
-netdev user,id=retro0
  |-- SLiRP UDP/TCP endpoint layer
  `-- per-netdev ONC RPC registry
       |-- one portmapper v2 endpoint
       |-- NFS and MOUNT registrations
       |-- NetInfo binder registration
       |-- NetInfo domain registration
       `-- future RPC service registrations
```

The implementation has five boundaries:

1. A unified SLiRP endpoint layer delivers guest UDP datagrams and TCP streams.
2. A reusable ONC RPC/XDR dispatcher handles calls, replies, authentication
   envelopes, protocol errors, and TCP record marking.
3. A per-netdev registry owns portmapper v2 and rejects program/port conflicts.
4. Independent NFS and NetInfo protocol engines register programs with it.
5. Separate QOM objects own configuration and lifecycle; there is no aggregate
   `retro-services` object.

NFS is moved onto the shared registry first, with compatibility tests proving
that its command line and guest-visible behavior remain unchanged.

## NetInfo protocol rollout

The wire implementation will be derived from published ONC RPC and historical
NetInfo protocol definitions, not copied from an implementation.

### Minimal boot target

The first NetInfo milestone provides one binder and one read-only domain over
UDP and TCP.  It supports static and broadcast discovery, deterministic binder
and domain ports, and the read/discovery operations used during NeXTSTEP boot:
ping, root, self, parent, children, lookup, list, read, property/name reads, and
statistics.  Unsupported mutation calls return the protocol's correct
read-only or authorization result rather than appearing to be absent.

The milestone succeeds when an unmodified NeXTSTEP guest discovers the binder,
binds the domain, reads its configuration, and reaches the GUI/login path while
NFS continues to pass its existing tests.

### Complete server

The later complete implementation adds every published binder and database
procedure, including transactional node/property/name creation and deletion.
It enforces NetInfo object instance/version checks so stale writes fail
correctly.  It then adds historical authentication and authorization behavior,
multiple domains, parent/child hierarchy, master/clone roles, and the complete
registration lifecycle.

Captured guest traffic may reveal compatibility requirements, but any behavior
not represented by the published protocol must be isolated and documented as a
compatibility quirk rather than embedded in the general RPC layer.

## Data model and JSON5 seed

The in-memory database is an ordered tree.  Nodes retain explicit object and
instance identifiers; properties and their value lists retain ordering and
duplicate values.  These details are observable through NetInfo and therefore
cannot be normalized away.

```json5
{
  version: 1,
  domain: { tag: "network", name: "/" },
  nodes: [
    {
      id: 0x0,
      instance: 0x1,
      parent: null,
      properties: { name: ["/"] },
    },
  ],
}
```

The loader strictly validates schema version, identifiers, references, size
limits, and domain invariants.  The built-in seed contains only the network
configuration needed for binding and boot; it creates no password or
privileged user.  Deterministic network defaults can be overridden by JSON5.

## Persistence and mutation

The configured JSON5 file is a seed, not a file rewritten after every RPC.
Writable state consists of a canonical snapshot plus a checksummed write-ahead
journal.  Each mutation is validated and applied as one transaction; the
journal record is flushed before success is returned.  Periodic compaction
writes and syncs a replacement snapshot before atomically publishing it.

Recovery replays complete, valid records and ignores only a torn final record.
Checksum failure or structural corruption prevents writable startup rather
than silently losing data.  UDP mutation requests use a bounded replay cache so
retransmission cannot apply a committed operation twice.

## Errors, limits, and migration

Malformed XDR, unknown versions/programs/procedures, invalid credentials,
missing or stale objects, and illegal mutations receive protocol-correct RPC or
NetInfo errors.  Request size, XDR strings and arrays, node/property counts,
tree depth, outstanding TCP records, and replay state are explicitly bounded.

The object keeps transport, database, and persistence state separable so a
future VMState description is possible.  Initially an active NetInfo object
installs a migration blocker: moving a live directory service without its
external persistence and peer state would claim a consistency guarantee that
the first version cannot provide.

## Verification

- Golden XDR vectors cover the published binder and database wire definitions.
- Unit tests cover tree operations, lookup semantics, ordering, duplicates,
  instance changes, stale writes, and JSON5 validation.
- Registry tests run NFS and NetInfo simultaneously and cover collisions,
  teardown, and legacy NFS behavior.
- SLiRP integration tests exercise UDP, broadcast discovery, TCP RPC framing,
  retransmission, malformed requests, and bounded-resource failures.
- Persistence tests interrupt journal writes and compaction, then verify exact
  recovery and corruption refusal.
- Fuzz targets cover RPC framing and NetInfo XDR decoding.
- Existing NFS, 9P, and general networking suites remain regression gates.
- A captured end-to-end NeXTSTEP boot demonstrates binder discovery, binding,
  database reads, and successful GUI/login startup.
