#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later
set -eu

qemu=${QEMU_M68K_SYSTEM:-qemu-system-m68k}
cross=${M68K_CROSS_PREFIX:-m68k-suse-linux-}
srcdir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
tmpdir=$(mktemp -d)
trap 'rm -rf "$tmpdir"' EXIT HUP INT TERM

"${cross}as" -m68040 -o "$tmpdir/packed-fmove.o" \
    "$srcdir/packed-fmove.S"
"${cross}ld" -T "$srcdir/kernel.ld" -o "$tmpdir/packed-fmove.elf" \
    "$tmpdir/packed-fmove.o"

timeout 10s "$qemu" \
    -M virt -cpu m68040 -display none -serial none -monitor none \
    -kernel "$tmpdir/packed-fmove.elf"
