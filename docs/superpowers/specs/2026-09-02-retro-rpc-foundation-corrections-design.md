# Retro RPC Foundation Corrections Design

## Goal

Close the cross-repository defects found during final review of the embedded
ONC RPC foundation without changing its guest-visible protocol or public QEMU
configuration.  The corrections must preserve the existing bounded-resource
contracts, keep embedded services inside SLiRP, and make QEMU's fallback build
equivalent to a build against the matching external libslirp.

## Scope and sequencing

The work is split at repository boundaries and integrated in dependency order:

1. libslirp safely reclaims retired TCP service objects.
2. QEMU serializes and bounds TCP RPC output while using the existing atomic
   stream-send API in chunks that fit current send space.
3. QEMU makes portmapper ownership per netdev and restricts PMAP dispatch to
   port 111.
4. After the libslirp branch is published, QEMU pins that immutable reachable
   revision and enables the same feature probes for fallback builds.

The fallback pin is never updated to an unpublished commit.  Publishing the
libslirp branch and changing the remote pin are explicit integration steps,
not hidden prerequisites of an otherwise local commit.

## Safe libslirp reclamation

Listener and connection removal must remain idempotent and safe during nested
callbacks.  Unlinking moves an object to a retired list, as today, but retired
objects are drained whenever the outermost TCP-service callback/public-call
dispatch boundary exits.  An object is freed only after it is unlinked, has no
active callback depth, and has no temporary internal references.

Whole-stack cleanup uses the same drain path after forcibly closing active
services.  Close/removal stays idempotent during the public call and any
reentrant callbacks it triggers; after the outermost dispatch boundary returns,
the opaque handle is invalid and callers must not reuse it.  The public header
documents that lifetime explicitly.  This permits deterministic reclamation
without retaining one tombstone per historical handle.  Tests exercise many
more sequential closures than the active-connection cap and verify the retired
population returns to zero under normal operation and reentrant callbacks.

## Ordered, bounded TCP RPC output

The public libslirp send call remains atomic; its ABI and return values do not
change.  QEMU's RPC layer instead owns one FIFO output queue per TCP RPC
connection.  Each reply is encoded as one record-marker plus payload buffer.
Only the head record is written, and it advances in chunks no larger than
`qemu_slirp_stream_can_send()`.  Records are never interleaved.  When space
reaches zero, transmission resumes from the existing `can_send` callback.

The aggregate queued bytes per connection are capped at one maximum TCP record
including its four-byte marker.  A reply that would exceed the cap, or an
additional reply that cannot fit, closes that RPC connection and releases all
queued requests.  This makes memory independent of pipelined request count
while retaining the existing 1 MiB inbound and outbound record limit.  Reply
completion makes its request terminal only after the complete record has been
accepted by the stream adapter.

Tests cover a reply larger than libslirp's 128 KiB send buffer, partial progress
over repeated readiness callbacks, strict FIFO/non-interleaving, queue overflow,
close during progress, and teardown with queued replies.  At least one
integration test uses real libslirp rather than only the fake stream backend.

## Per-netdev portmapper ownership

The first RPC program registered on a netdev acquires both UDP and TCP port 111,
regardless of that program's own transports.  Both endpoints remain until the
last program registration is removed.  Registration is transactional: failure
to acquire either endpoint rolls back both portmapper endpoints and any service
endpoint acquired by that call.

UDP and TCP PMAP GETPORT can therefore discover programs on either transport.
PMAP calls are dispatched only when their destination endpoint is port 111;
program 100000 sent to another registered service port receives normal
program-unavailable handling.  Existing UDP broadcast CALLIT behavior and its
UDP-only target rule remain unchanged.

Tests cover UDP-only and TCP-only first registrations, cross-transport GETPORT,
both port-111 endpoints persisting until the final unregister, acquisition
rollback, and rejection of PMAP calls on non-111 service ports.

## Fallback integration

Once the corrected libslirp commits are available from the configured remote,
`subprojects/slirp.wrap` pins their immutable commit ID.  QEMU's internal
dependency path then enables `CONFIG_SLIRP_UDP_LISTEN_FULL` and
`CONFIG_SLIRP_TCP_SERVICE`; the external dependency path continues to use
compile/link probes.

A force-fallback configure/build must compile the ONC RPC registry and NFS
object with both features enabled, run the focused RPC/NFS tests, and pass one
real PMAP DUMP.  The external-libslirp build remains a separate gate.  Neither
configuration may open host TCP or UDP listeners for embedded services.

## Error handling and lifecycle

- All allocation and endpoint-acquisition failures unwind in reverse order.
- Closing a TCP RPC connection cancels queued replies and releases each
  retained request exactly once.
- Reentrant stream close, registry teardown, and netdev cleanup remain safe.
- Migration remains rejected while TCP guest services are active.
- No limits are silently raised and no host-facing socket behavior is added.

## Verification

- New regressions demonstrate each failure before its correction and pass
  afterward.
- Complete libslirp and QEMU unit suites pass.
- Real-libslirp and force-fallback QEMU builds both pass focused ONC RPC, SLiRP,
  NFS, and m68k object qtests.
- Sanitizer or Valgrind coverage includes TCP churn and queued-reply teardown.
- `git diff --check` and checkpatch pass for each correction commit.
- Live `/proc` inspection again confirms no host TCP/UDP sockets or ports
  111/635/2049.
- The public README and five protected boot GIFs remain unchanged.
