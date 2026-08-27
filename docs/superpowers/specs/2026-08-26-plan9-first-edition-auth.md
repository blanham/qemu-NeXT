# Plan 9 First Edition authentication compatibility

## Goal

Boot the archived First Edition NeXT kernel through its original 9P root
mounts without modifying the preserved root tree.  It must first support the
native `none` identity and then faithfully implement the First Edition
challenge/ticket exchange used by `mount -a`.

## Wire contract

First Edition uses fixed, unframed 9P messages:

- `Tauth` (`82`): 69 bytes: tag, fid, uname, and an encrypted 36-byte
  `[FScchal, client challenge, requested server]`.
- `Rauth` (`83`): 35 bytes: tag, fid, and an encrypted 30-byte pair of a
  client ticket and a server ticket.
- First Edition `Tattach`/`Rattach` retain their 89/13-byte framing.

All DES operations use the existing packed 56-bit key and overlapping-block
Plan 9 transform.  The preserved First Edition source is the protocol
authority: `sys/src/9/port/devmnt.c`, `sys/src/auth/fsauth.srv.c`, and
`sys/src/fs/port/auth.c`.

## Behavior

- An unauthenticated First Edition `Tattach` for `none` with an empty ticket
  is accepted when file authentication is not configured, matching the native
  file server's boot identity behavior.
- When legacy authentication is configured, `Tauth` validates the client
  request against its key, generates a one-time server challenge and ticket
  key, and returns the exact encrypted 30-byte reply.
- A subsequent `Tattach` must present the issued server ticket; it is
  single-use and binds the authenticated user.  Malformed, wrong-key,
  mismatched, and replayed requests produce the generic historical auth
  error.
- Second Edition framing and its existing ticket protocol remain unchanged.

## Verification

Unit tests must first fail for literal First Edition Tauth/Rauth frames, then
cover successful ticket attach plus wrong key, wrong challenge, and replay.
The full 9P unit suite, rebuilt m68k QEMU, and a visible GTK boot of the
archived kernel are required before completion.
