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

"${assembler}" -m68030 -o "$tmpdir/pmmu-control.o" \
    "$srcdir/pmmu-control.S"
"${linker}" -T "$srcdir/kernel.ld" -o "$tmpdir/pmmu-control.elf" \
    "$tmpdir/pmmu-control.o"

"${assembler}" -m68030 -o "$tmpdir/pmmu-control-privilege.o" \
    "$srcdir/pmmu-control-privilege.S"
"${linker}" -T "$srcdir/kernel.ld" -o "$tmpdir/pmmu-control-privilege.elf" \
    "$tmpdir/pmmu-control-privilege.o"

"${assembler}" -m68030 -o "$tmpdir/movec-caar.o" \
    "$srcdir/movec-caar.S"
"${linker}" -T "$srcdir/kernel.ld" -o "$tmpdir/movec-caar.elf" \
    "$tmpdir/movec-caar.o"

timeout 10s "$qemu" \
    -M virt -cpu m68030 -display none -serial none -monitor none \
    -kernel "$tmpdir/pmmu-control.elf"

timeout 10s "$qemu" \
    -M virt -cpu m68030 -display none -serial none -monitor none \
    -kernel "$tmpdir/pmmu-control-privilege.elf"

timeout 10s "$qemu" \
    -M virt -cpu m68030 -display none -serial none -monitor none \
    -kernel "$tmpdir/movec-caar.elf"
