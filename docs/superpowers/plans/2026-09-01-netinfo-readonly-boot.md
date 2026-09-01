# Read-Only NetInfo Boot Server Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make an unmodified NeXTSTEP guest discover and bind a QEMU-hosted NetInfo domain, perform its startup reads over TCP, and reach GUI/login without running guest `nibindd`.

**Architecture:** A `netinfo-server` QOM object owns one immutable ordered-tree database and registers binder program 200100001/v1 plus database program 200100000/v2 with the shared per-netdev RPC registry. Binder discovery works through direct portmapper lookup and broadcast CALLIT; data reads use TCP record-marked RPC. A bounded JSON5 loader optionally replaces the safe built-in seed.

**Tech Stack:** C11, GLib/QObject, JSON5, ONC RPC/XDR, NetInfo v2, QOM/QAPI, QTest, PCAP.

---

## Fixed wire contract

- Binder ports are UDP/659 and TCP/661; the single domain uses UDP/660 and
  TCP/662. All are internal guest ports and registered with portmapper.
- Binder implements PING(0), GETREGISTER(3), LISTREG(4), and BIND(8).
  REGISTER/UNREGISTER/CREATE/CLONE/DESTROY return `NI_RDONLY`. BIND is silent
  for a nonmatching tag, as required by PMAP CALLIT.
- Database v2 implements PING(0), STATISTICS(1), ROOT(2), SELF(3), PARENT(4),
  READ(7), CHILDREN(9), LOOKUP(10), LIST(11), READPROP(14), LISTPROPS(17),
  READNAME(20), RPARENT(22), BIND(24), and LOOKUPREAD(28).
- Data RPC permits PING and BIND on UDP. Other database calls over UDP return
  RPC `AUTH_TOOWEAK`; TCP serves reads. All mutation/replication procedures
  return NetInfo `NI_RDONLY` in a correctly shaped result.
- Protocol maxima are historical, but configurable service limits are lower:
  4,096 nodes, 256 properties/node, 1,024 values/property, 4 KiB names/values,
  1 MiB encoded TCP record, and tree depth 128.

## File map

- `include/qobject/qjson5.h`, `qobject/qjson5.c`: isolated JSON5 parser returning QObject.
- `hw/netinfo/netinfo-protocol.h`: clean-room constants and wire structures.
- `hw/netinfo/netinfo-xdr.{c,h}`: binder/database argument and result codecs.
- `hw/netinfo/netinfo-db.{c,h}`: ordered immutable tree, indexes, validation.
- `hw/netinfo/netinfo-server.{c,h}`: binder and database dispatch.
- `hw/netinfo/netinfo-object.c`, `hw/netinfo/meson.build`, `hw/meson.build`:
  QOM lifecycle and RPC registrations.
- `qapi/qom.json`: `NetInfoServerProperties` and object union member.
- `tests/unit/test-qjson5.c`, `test-netinfo-xdr.c`, `test-netinfo-db.c`,
  `test-netinfo-server.c`: focused tests.
- `tests/qtest/netinfo-server-object-test.c`: CLI/QMP lifecycle, isolation,
  migration blocker, and no-host-listener tests.
- `docs/system/devices/netinfo.rst`, `docs/system/devices/index.rst`: public usage.

### Task 1: Implement the bounded JSON5 parser

**Files:** Create `include/qobject/qjson5.h`, `qobject/qjson5.c`, `tests/unit/test-qjson5.c`; modify `qobject/meson.build`, `tests/unit/meson.build`.

- [ ] Write table-driven failing cases for comments, unquoted identifier keys,
  single/double strings, escapes, hexadecimal integers, signed numbers,
  trailing commas, arrays/objects, NaN/Infinity, and duplicate-key rejection.
  Add failures for integer overflow, invalid Unicode, trailing input, and
  nesting 129; the NetInfo schema rejects non-finite numbers in its fields.
- [ ] Run `meson test -C /mnt/build/NeXT/builds/netinfo-server test-qjson5 --print-errorlogs`; expected missing target/API.
- [ ] Export one ownership-safe entry point:

```c
QObject *qobject_from_json5(const char *text, size_t length, Error **errp);
```

  Implement a dedicated lexer and recursive-descent parser; do not relax QEMU's
  QMP JSON parser. Cap input at 4 MiB, nesting at 128, tokens at 1 MiB, and
  reject duplicate object keys. Store integers as `QNum` without precision
  loss, including `0xffffffff`.
- [ ] Run `test-qjson5`, `check-json-parser`, and `check-qjson`; expected all pass.
- [ ] Commit with `git commit -m "qobject: add bounded JSON5 configuration parser"`.

### Task 2: Define and test clean-room NetInfo XDR

**Files:** Create `hw/netinfo/netinfo-protocol.h`, `hw/netinfo/netinfo-xdr.{c,h}`, `tests/unit/test-netinfo-xdr.c`; modify `hw/netinfo/meson.build`, `hw/meson.build`, `tests/unit/meson.build`.

- [ ] Define `NIBIND_PROG 200100001`, `NIBIND_VERS 1`, `NI_PROG 200100000`,
  `NI_VERS 2`, all procedure numbers 0..28, and statuses 0..21 plus 9999 from
  the preserved `.x` files. Cite those definitions and RFC 1831/1832 in file
  comments; copy no generated or implementation code.
- [ ] Add golden tests for GETREGISTER(`network`), registration ports,
  BIND arguments, ROOT `{0,0x24}`, LOOKUP, READ, pointer-presence booleans,
  empty lists, maxima, truncation, and extra trailing fields.
- [ ] Run the target; expected RED until codecs are present.
- [ ] Implement bounded codecs using `OncRpcXdrReader/Writer`, with explicit
  cleanup helpers for every allocated name/list/property.
- [ ] Run `test-netinfo-xdr` and `test-onc-rpc`; expected all pass.
- [ ] Commit with `git commit -m "netinfo: add clean-room protocol codecs"`.

### Task 3: Implement the ordered read-only database

**Files:** Create `hw/netinfo/netinfo-db.{c,h}`, `tests/unit/test-netinfo-db.c`; modify Meson files.

- [ ] Define nodes as object/instance/parent plus ordered `GPtrArray` properties,
  where each property owns a name and ordered value array. Maintain an
  object-ID hash index, but return results in stored order.
- [ ] Write failing tests for root/self/parent/children/read, lookup by exact
  property/value, list projections, readprop/listprops/readname, lookupread,
  duplicate values, missing IDs/properties/names, and ignored instance values
  on reads.
- [ ] Add strict construction APIs and these read operations:

```c
NiStatus netinfo_db_root(NetInfoDb *, NiId *);
NiStatus netinfo_db_read(NetInfoDb *, NiId *, NiPropertyList *);
NiStatus netinfo_db_children(NetInfoDb *, NiId *, NiIdList *);
NiStatus netinfo_db_lookup(NetInfoDb *, NiId *, const char *, const char *, NiIdList *);
NiStatus netinfo_db_lookup_read(NetInfoDb *, NiId *, const char *, const char *, NiId *, NiPropertyList *);
```

- [ ] Build the default `network` domain with root `name=/` and
  `master=localhost/network`, `/machines/localhost` containing
  `ip_address=10.0.2.2` and `serves=./network`, and no users/passwords.
- [ ] Run `test-netinfo-db`; expected all pass under ASan-compatible ownership.
- [ ] Commit with `git commit -m "netinfo: add ordered read-only database"`.

### Task 4: Load and validate versioned JSON5 domains

**Files:** Modify `hw/netinfo/netinfo-db.{c,h}`, `tests/unit/test-netinfo-db.c`; create `tests/data/netinfo/minimal-domain.json5`, `tests/data/netinfo/invalid-*.json5`.

- [ ] Add failing load tests for the approved schema, hexadecimal IDs,
  property/value ordering, duplicate values, dangling parents, duplicate IDs,
  cycles, multiple roots, limit violations, unknown keys, and schema versions.
- [ ] Preserve the approved property-object schema and iterate its `QDict`
  entries in source order; duplicate property keys remain a JSON5 parse error:

```json5
{ version: 1, domain: { tag: 'network', name: '/' }, nodes: [
  { id: 0x0, instance: 0x1, parent: null,
    properties: { name: ['/'] } },
] }
```

- [ ] Implement `netinfo_db_load_json5(path, defaults, errp)` using
  `qobject_from_json5`; validate the complete temporary model before publishing
  it so errors cannot leave partial state.
- [ ] Run database and JSON5 tests; expected all pass.
- [ ] Commit with `git commit -m "netinfo: load versioned JSON5 domains"`.

### Task 5: Implement binder discovery and read-only database RPC

**Files:** Create `hw/netinfo/netinfo-server.{c,h}`, `tests/unit/test-netinfo-server.c`; modify Meson files.

- [ ] Build a fake RPC registry harness and write failing tests for the fixed
  binder/database matrix, TCP-only reads, `AUTH_TOOWEAK`, `NI_RDONLY`, silent
  BIND mismatch, `NI_NETROOT` RPARENT, malformed bodies, and exact golden replies.
- [ ] Register binder v1 on UDP/659 and TCP/661 and database v2 on UDP/660 and
  TCP/662. GETREGISTER returns domain ports only for the configured tag;
  LISTREG returns exactly one entry.
- [ ] Dispatch read procedures into `NetInfoDb`; update returned IDs to the
  current instance where required. STATISTICS returns deterministic counts and
  no host-sensitive data. BIND succeeds only for the configured local server.
- [ ] Run `test-netinfo-server`, `test-onc-rpc`, and `test-nfs2-server`;
  expected all pass with NFS and NetInfo registered together.
- [ ] Commit with `git commit -m "netinfo: serve binder and read-only domain RPC"`.

### Task 6: Add the `netinfo-server` QOM object

**Files:** Create `hw/netinfo/netinfo-object.c`, `tests/qtest/netinfo-server-object-test.c`; modify `qapi/qom.json`, `hw/netinfo/meson.build`, `tests/qtest/meson.build`.

- [ ] Add QAPI properties `netdev: str`, optional `config: str`, optional
  `domain-tag: str`, and fixed-default port properties for conflict testing.
  Reserve `writable` and `state` for the full-server plan but reject writable
  mode in this milestone.
- [ ] Write qtests for zero-config creation, JSON5 creation, missing/wrong
  netdev, duplicate object, NFS coexistence, port collision unwind,
  delete/recreate, reset, migration blocker, and `/proc` no-host-listener proof.
- [ ] Implement UserCreatable/Resettable lifecycle modeled on `nfs2-object.c`.
  Load the database before RPC registration, unregister in reverse order, and
  add blocker text `NetInfo server state is not migratable` only after success.
- [ ] Run `qtest-m68k/netinfo-server-object-test` and the existing NFS qtest;
  expected all pass.
- [ ] Commit with `git commit -m "netinfo: add embedded server object"`.

### Task 7: Document and prove NeXTSTEP boot

**Files:** Create `docs/system/devices/netinfo.rst`; modify `docs/system/devices/index.rst`; add only bounded public evidence explicitly approved by the owner.

- [ ] Document safe defaults, JSON5 schema, ports, isolation, migration blocker,
  and an NFS+NetInfo command line. Do not mention private lab paths.
- [ ] Boot a disposable NeXTSTEP overlay with guest `nibindd` disabled and the
  embedded object enabled. Capture PMAP GETPORT/CALLIT, GETREGISTER, TCP NI
  reads, and GUI/login arrival.
- [ ] If a packet differs, add a failing golden/integration test before changing
  code; document any published-protocol compatibility quirk beside its test.
- [ ] Run unit, qtest, NFS/9P regression, `git diff --check`, and checkpatch.
- [ ] Verify `QEMU-NeXT-README.md` lost no section/reference and all five
  `docs/boot/*.gif` files exist.
- [ ] Commit docs/evidence separately with `git commit -m "docs: describe embedded NetInfo service"`.
