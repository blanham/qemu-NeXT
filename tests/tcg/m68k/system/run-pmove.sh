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

"${assembler}" -m68030 -o "$tmpdir/pmove.o" "$srcdir/pmove.S"
"${linker}" -T "$srcdir/kernel.ld" -o "$tmpdir/pmove.elf" \
    "$tmpdir/pmove.o"

"${assembler}" -m68030 -o "$tmpdir/pmove-privilege.o" \
    "$srcdir/pmove-privilege.S"
"${linker}" -T "$srcdir/kernel.ld" -o "$tmpdir/pmove-privilege.elf" \
    "$tmpdir/pmove-privilege.o"

"${assembler}" -m68030 -o "$tmpdir/pmove-config.o" \
    "$srcdir/pmove-config.S"
"${linker}" -T "$srcdir/kernel.ld" -o "$tmpdir/pmove-config.elf" \
    "$tmpdir/pmove-config.o"

"${assembler}" -m68030 -o "$tmpdir/pmove-atc.o" \
    "$srcdir/pmove-atc.S"
"${linker}" -T "$srcdir/kernel.ld" -o "$tmpdir/pmove-atc.elf" \
    "$tmpdir/pmove-atc.o"

"${assembler}" -m68030 -o "$tmpdir/pmove-pc.o" \
    "$srcdir/pmove-pc.S"
"${linker}" -T "$srcdir/kernel.ld" -o "$tmpdir/pmove-pc.elf" \
    "$tmpdir/pmove-pc.o"

timeout 10s "$qemu" \
    -M virt -cpu m68030 -display none -serial none -monitor none \
    -kernel "$tmpdir/pmove.elf"

timeout 10s "$qemu" \
    -M virt -cpu m68030 -display none -serial none -monitor none \
    -kernel "$tmpdir/pmove-privilege.elf"

timeout 10s "$qemu" \
    -M virt -cpu m68030 -display none -serial none -monitor none \
    -kernel "$tmpdir/pmove-config.elf"

timeout 10s "$qemu" \
    -M virt -cpu m68030 -display none -serial none -monitor none \
    -kernel "$tmpdir/pmove-atc.elf"

timeout 10s "$qemu" \
    -M virt -cpu m68030 -display none -serial none -monitor none \
    -kernel "$tmpdir/pmove-pc.elf"
