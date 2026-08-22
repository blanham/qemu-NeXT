# NetBSD NFSv2 + NFSv3 Addendum

This addendum supersedes every v3-fallback-only statement in
`2026-08-22-netbsd-nfs-root.md`.

The exact NetBSD 1.5 next68k boot chain uses both protocol versions:

- `sys/lib/libsa/nfs.c` is hard-coded to NFSv2 while the standalone loader
  fetches `netbsd`.
- `sys/nfs/nfs_boot.c` initializes diskless root with `NFSMNT_NFSV3`, asks
  portmapper for MOUNT v3 and NFSv3, and falls back only after a mount protocol
  version mismatch.

## Revised server contract

- UDP portmapper advertises MOUNT v1 and v3 on 635, and NFSv2 and v3 on 2049.
- MOUNT v1 and v3 accept only `/`.  V1 returns the fixed 32-byte handle.  V3
  returns the same handle as a bounded variable opaque plus AUTH_SYS.
- NFSv2 retains the complete Task 8/9 procedure surface for the standalone
  loader and compatibility clients.
- NFSv3 implements all RFC 1813 procedures.  Task 8 adds the read side:
  NULL, GETATTR, LOOKUP, ACCESS, READLINK, READ, READDIR, READDIRPLUS, FSSTAT,
  FSINFO, and PATHCONF.  Task 9 adds SETATTR, WRITE, CREATE, MKDIR, SYMLINK,
  MKNOD, REMOVE, RMDIR, RENAME, LINK, and COMMIT.
- Both versions use the same authenticated handle table and mapped-xattr
  backend.  NFSv3 uses 64-bit offsets, sizes, file IDs, and cookies, bounded
  32-byte handles, post-operation attributes, weak cache consistency data,
  an 8 KiB transfer ceiling, and 32 KiB datagram ceiling.
- The duplicate-XID cache covers every v2 and v3 mutation, including COMMIT.

## Revised evidence contract

A complete capture proves both ordered phases:

1. BOOTP option 17 `/`.
2. MOUNT v1 and NFSv2 portmapper replies, successful MOUNT v1 `/`, then NFSv2
   LOOKUP `netbsd` and READ using the returned handles.
3. MOUNT v3 and NFSv3 portmapper replies, successful MOUNT v3 `/` with a
   32-byte handle and AUTH_SYS, then successful NFSv3 GETATTR, FSINFO, ACCESS,
   and READDIRPLUS correlated to that root.

V2-only and v3-only traces are both evidence-incomplete.

