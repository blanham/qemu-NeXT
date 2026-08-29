# Repository instructions

## Public project page

`QEMU-NeXT-README.md` is the public showcase and historical record for this
repository. Preserve its author background, debugging history, supported-OS
boot instructions, ROM compatibility matrix, hardware table, and boot gallery.
Do not remove, replace, substantially condense, or relocate that material
without the repository owner's explicit approval.

The GIFs under `docs/boot/` are published evidence, not disposable generated
artifacts. Keep all five README references and their tracked files intact:

- `nextstep.gif`
- `netbsd-network.gif`
- `netbsd-full.gif`
- `plan9-tcp.gif`
- `plan9-il.gif`

Documentation updates should add or correct information in place. Before any
public documentation commit, compare `QEMU-NeXT-README.md` with its parent,
confirm that no existing section or image reference disappeared unintentionally,
and verify that every local `docs/boot/` target referenced by the README exists.

Private lab procedures, unpublished infrastructure, temporary plans, and local
asset paths do not belong in this public repository. Their removal must not be
used as a reason to rewrite or discard the public-facing project narrative.
