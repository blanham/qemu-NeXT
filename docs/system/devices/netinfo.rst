Embedded NetInfo service
========================

QEMU can provide a read-only NetInfo domain for legacy NeXTSTEP guests.  The
``netinfo-server`` object puts the binder and database RPC services in a named
user-mode network stack.  It is intended for a guest that expects the
NeXTSTEP NetInfo protocol, not for general-purpose directory service use.

This is an embedded service.  It does not open host UDP or TCP listeners, and
the service is reachable only from guests attached to the selected ``user``
network.  Separate user-mode networks have separate portmapper and NetInfo
registries.

Quick start
-----------

The minimal configuration is::

  -netdev user,id=nextnet,ipv6=off \
  -object netinfo-server,id=nextinfo,netdev=nextnet \
  -net nic,model=next-mb8795,netdev=nextnet

The embedded service in this milestone requires SLiRP's default virtual-host
address ``10.0.2.2``; the object has no address property.  A guest with a
static network configuration must route its NetInfo requests to that address.

An NFS root and NetInfo service can share the same user-mode network.  This
example keeps the NFS export read-only (the default)::

  -fsdev local,id=nextroot,path=/path/to/root,security_model=mapped-xattr \
  -netdev user,id=nextnet,ipv6=off \
  -object nfs-server,id=nextnfs,fsdev=nextroot,netdev=nextnet \
  -object netinfo-server,id=nextinfo,netdev=nextnet \
  -net nic,model=next-mb8795,netdev=nextnet

The ``fsdev`` path is an operator-selected export; do not put credentials or
private host paths in shared command lines.  See :doc:`nfs-root` for the NFS
root service's export and authentication details.

Safe defaults and object properties
-----------------------------------

``netdev`` is required and must name an IPv4-enabled ``user`` network.  The
object constructs and validates the complete domain before registering any
endpoint.  With no ``config`` property it creates a deterministic, immutable
``network`` domain containing the root, ``machines``, and ``localhost``
nodes.  No guest-supplied data is merged into that seed.

The supported properties are:

``config``
  Optional path to one complete version-1 JSON5 domain.  It replaces the
  built-in seed; it is not merged with it.  The file must be a regular file
  and is bounded before parsing.

``domain-tag``
  Optional domain tag.  Without ``config`` it changes the built-in tag (the
  default is ``network``).  With ``config`` it must exactly match the tag in
  the file.

``binder-udp-port`` / ``binder-tcp-port``
  Guest-only binder ports, defaulting to UDP ``659`` and TCP ``661``.

``database-udp-port`` / ``database-tcp-port``
  Guest-only database ports, defaulting to UDP ``660`` and TCP ``662``.

``writable``
  Defaults to ``off``.  ``on`` is rejected: this object implements only the
  read-only service.

``state``
  Reserved for the complete service and does not enable persistence in this
  read-only implementation.

The database remains immutable across a guest reset.  In normal operation,
object creation succeeds and a later migration is blocked with the exact
error ``NetInfo server state is not migratable``.  With ``-only-migratable``,
object creation itself fails with that error.

JSON5 domain schema
-------------------

``config`` is a complete JSON5 document whose top-level object has exactly
these keys: ``version``, ``domain``, and ``nodes``.  ``version`` must be the
integer ``1``.  The ``domain`` object has exactly ``tag`` and ``name`` keys,
and each object in the ``nodes`` array has exactly ``id``, ``instance``,
``parent``, and ``properties`` keys.  Comments, unquoted JSON5 keys, single
quotes, and hexadecimal integer literals are accepted by the bounded JSON5
parser.  A small domain is shown below::

  {
    version: 1,
    domain: { tag: 'network', name: '/' },
    nodes: [
      {
        id: 0x0,
        instance: 0x24,
        parent: null,
        properties: {
          name: ['/'],
          master: ['localhost/network'],
        },
      },
      {
        id: 0x1,
        instance: 0x1,
        parent: 0x0,
        properties: { name: ['machines'] },
      },
      {
        id: 0x2,
        instance: 0x1,
        parent: 0x1,
        properties: {
          name: ['localhost'],
          ip_address: ['10.0.2.2'],
          serves: ['./network'],
        },
      },
    ],
  }

The ``domain.tag`` is a non-empty service name and ``domain.name`` must be
``/``.  ``nodes`` must contain one root (``parent: null``), whose object ID is
zero and whose ``name`` property is ``/``.  Node ``id`` and ``instance``
fields, and every non-null ``parent``, are finite, non-negative u32 integers.
``UINT32_MAX`` is reserved for the null object value and is rejected for node
IDs and non-null parents.  ``properties`` is an ordered object mapping
non-empty property names to ordered arrays of strings; property and value
order is retained.  Duplicate keys in any JSON5 object, including duplicate
property keys, are rejected, as are duplicate node object IDs.  Unknown keys
are rejected.

The parser accepts at most 4 MiB of input, 128 levels of nesting, and 1 MiB
per token.  The service additionally bounds a domain to 4096 nodes, 256
properties per node, 1024 values per property, 4096 bytes for each domain tag,
property name, and property value, and 128 parent levels.  TCP RPC records are
limited to 1 MiB.  These bounds apply before the database is exposed to a
guest.

Wire services and isolation
---------------------------

The object registers these ONC RPC programs with the user-mode network's
guest-only portmapper (PMAP) on port ``111``:

======================  =========  =======  ================
Service                 Program    Version  Guest ports
======================  =========  =======  ================
NetInfo binder          200100001  1        UDP 659, TCP 661
NetInfo database        200100000  2        UDP 660, TCP 662
======================  =========  =======  ================

PMAP ``GETPORT`` discovers the binder and database ports.  PMAP ``CALLIT``
uses the UDP binder path and returns the wrapped binder result.  The binder
answers ``GETREGISTER`` and ``LISTREG`` for the configured tag.  A matching
database ``BIND`` succeeds; a tag or address mismatch is dropped as required
by the historical client contract.

Database ``PING`` and ``BIND`` are available over UDP.  Other database calls
over UDP return ``AUTH_TOOWEAK``; read calls use TCP record framing.  The
read-only implementation covers root, self, parent, children, lookup, list,
property/name reads, statistics, and the compatibility procedures needed by
legacy clients.  Mutating procedures return ``NI_RDONLY``.  ``NI_CRASHED`` is
the historical compatibility exception: its historical result is void, so a
valid call succeeds with a void body and performs no operation instead of
returning ``NI_RDONLY``.

The portmapper and both NetInfo programs are owned by the selected user-mode
network.  The object opens no host listener, and a service registered on one
user-mode network cannot be used from another.

Verification boundary
---------------------

The repository tests provide bounded protocol evidence for PMAP ``GETPORT``
and ``CALLIT``, binder registration, TCP database reads, UDP authentication,
read-only responses, no-host-listener behavior, port collisions, and NFS
coexistence.  The relevant test groups are ``/onc-rpc/registry/portmapper``,
``/onc-rpc/registry/callit-async-cancel``,
``/netinfo-server/database/tcp-read-udp-auth``, and
``/m68k/netinfo-server-object/custom-binder-registration-ports``.  The direct
no-host-listener check is ``/m68k/netinfo-server-object/no-host-listener`` in
``tests/qtest/netinfo-server-object-test.c``; NFS coexistence is
``/m68k/netinfo-server-nfs-object/nfs-coexistence`` in
``tests/qtest/netinfo-server-nfs-object-test.c``.

A live aggregate validation used a fresh disposable overlay.  On that
overlay, guest ``nibindd`` was disabled and the guest was configured for
SLiRP's default usernet host address ``10.0.2.2``.  Multiple runs against the
same prepared overlay established:

* PMAP v2 ``GETPORT`` for binder ``200100001/v1/UDP`` returning ``659``;
* binder ``GETREGISTER`` for tag ``network`` returning ``NI_OK`` and
  advertising database UDP ``660`` and TCP ``662``;
* record-marked TCP NetInfo v2 ``ROOT`` succeeding with ``{0, 0x24}``, and
  database procedure 11 ``LIST`` for property ``name`` returning child
  ``machines``;
* broadcast PMAP ``CALLIT`` wrapping binder ``PING`` and returning port
  ``659``;
* later normal boots reaching the setup GUI and LoginWindow.

The protocol capture came from direct guest clients in a single-user proof
run.  Every cited direct-client protocol run and each separate normal boot
reaching the setup GUI and LoginWindow launched QEMU with the embedded
``netinfo-server`` object enabled.  The GUI arrivals came from separate normal
boots on the same prepared overlay; the capture was not generated during
normal boot, and these observations were not one uninterrupted run.  This is
aggregate validation, not evidence of one uninterrupted boot sequence.

No raw packet capture, screenshots, or guest disk is shipped with this
documentation.
