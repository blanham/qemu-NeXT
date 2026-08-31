#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later
set -eu

qemu=${QEMU_M68K_SYSTEM:-qemu-system-m68k}
cross=${M68K_CROSS_PREFIX:-m68k-suse-linux-}
assembler=${M68K_AS:-${cross}as}
linker=${M68K_LD:-${cross}ld}
srcdir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
tmpdir=$(mktemp -d)
trap 'rm -rf "$tmpdir"' EXIT HUP INT TERM

"${assembler}" -m68030 -o "$tmpdir/pmmu-fault-restart.o" \
    "$srcdir/pmmu-fault-restart.S"
"${linker}" -T "$srcdir/kernel.ld" \
    -o "$tmpdir/pmmu-fault-restart.elf" \
    "$tmpdir/pmmu-fault-restart.o"

timeout 10s "$qemu" \
    -M virt -cpu m68030 -display none -serial none -monitor none \
    -kernel "$tmpdir/pmmu-fault-restart.elf"
