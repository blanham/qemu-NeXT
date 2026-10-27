#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later
# M68040 MMU Modified-bit regression runner (Ninja build, no timeout wrapper;
# guest terminates via virt-ctrl HALT=pass / PANIC=fail with -action).
set -eu
qemu=${QEMU_M68K_SYSTEM:-qemu-system-m68k}
srcdir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
root=$(CDPATH= cd -- "$srcdir/../../../.." && pwd)
ninja -C "$root" -f tests/tcg/m68k/system/mmu-modified.ninja
elf="$root/build/mmu-test/mmu-modified.elf"
exec "$qemu" \
    -M virt -cpu m68040 -display none -serial stdio -monitor none \
    -action panic=exit-failure \
    -kernel "$elf"
