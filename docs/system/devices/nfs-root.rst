Private NFS root service
========================

QEMU can serve a ``-fsdev`` export as an NFS root through a named user-mode
network stack.  The portmapper, mount, and NFS UDP endpoints exist only at the
stack's virtual-host address; they do not open host sockets.

For example, this configuration exports a disposable NetBSD root and provides
its standalone bootstrap over TFTP::

  -fsdev local,id=netbsdroot,path=/run/netbsd-root,security_model=mapped-xattr \
  -netdev user,id=nextnet,ipv6=off,tftp=/run/tftp,bootfile=boot \
  -object nfs-server,id=netbsdnfs,fsdev=netbsdroot,netdev=nextnet,writable=on \
  -net nic,model=next-mb8795,netdev=nextnet

``fsdev`` and ``netdev`` are required.  The netdev must name a ``user``
backend with IPv4 enabled.  ``root-path`` defaults to ``/`` and is advertised
as the BOOTP root path; currently it must be ``/``.  The service registers
portmapper v2 on UDP port 111, mountd on UDP port 635, and NFS on UDP port
2049.  It supports mount protocol versions 1 and 3 and NFS versions 2 and 3.
The NetBSD standalone bootstrap uses MOUNT v1 and NFSv2 to load the kernel;
the kernel then uses MOUNT v3 and NFSv3 for its root filesystem.

The server parses ``AUTH_SYS`` credentials for protocol compatibility but
does not trust them as host authorization.  Export a private tree with an
appropriate ``-fsdev`` security model; ``security_model=mapped-xattr`` keeps
guest ownership and special-file metadata in extended attributes rather than
applying them directly to host files.

Exports are read-only by default.  Mutating NFS requests are enabled only when
the object has ``writable=on`` and the referenced fsdev is itself writable.
Live server state is not migratable.  Guest reset clears replay state while
retaining the server file-handle key, so handles remain valid across the
reset.
