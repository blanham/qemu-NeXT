# QEMU NeXT hardware support

This branch provides QEMU machine models for the non-Turbo NeXTcube and
NeXTstation families. The models use the original v66 NeXT ROM convention,
which must be supplied by the user; firmware and disk images are not included.

## Build

Clone the public repository and build the m68k system emulator:

```sh
git clone --branch metachicken --single-branch https://github.com/blanham/qemu-NeXT.git qemu-NeXT
cd qemu-NeXT
mkdir build
cd build
../configure --target-list=m68k-softmmu
./pyvenv/bin/meson setup --reconfigure --force-fallback-for=slirp . ..
grep -qx '#define CONFIG_SLIRP_PLAN9_BOOTP' config-host.h || {
  echo "bundled Plan 9-capable slirp was not selected" >&2
  exit 1
}
make -j"$(nproc)" qemu-system-m68k qemu-img
```

The v66 ROM is conventionally named `Rev_2.5_v66.bin` and has SHA-1
`b3534796abae238a0111299fc406a9349f7fee24`. Pass it with `-bios`:

```sh
QEMU="$PWD/qemu-system-m68k"
ROM=/path/to/Rev_2.5_v66.bin
DISK=/path/to/next-disk.img

"$QEMU" -M next-cube -m 64M -bios "$ROM" \
  -drive "if=scsi,format=raw,file=$DISK" -display gtk
```

## Plan 9 archive and netboot support

QEMU implements the original 9P1 protocol used by the archived Plan 9 First
and Second Edition NeXT kernels. The NeXT BOOTP vendor data, TFTP kernel
transfer, and a guest-only 9P1 root service are provided through the IPv4
user-mode network backend. The historical release archives are available
from the [OSU Open Source Lab Plan 9 archive](https://ftp.osuosl.org/pub/plan9/history/);
the release hashes and kernel members are:

| Edition | Archive | SHA-256 | Kernel member |
|---|---|---|---|
| First | `plan9-1e.tar.bz2` | `8718e279aa35b10a9391f330976d31c177bad00de40e7f0f67232ffa54fb7d77` | `plan9-1e/68020/9nextstation` |
| Second | `plan9-2e.tar.bz2` | `0bb3c1446deb79b179f73886eb2419ecfac9f9964040683e3d5731f074bc2ce6` | `plan9-2e/68020/9nextstation` |

Verify an archive before extracting it, stage its matching
`68020/9nextstation` member below a TFTP directory, and extract the matching
top-level tree below a writable root directory. Then launch either edition
with the same user-facing command, changing `RELEASE` and `ROOT`:

```sh
QEMU=/path/to/qemu-system-m68k
ROM=/path/to/Rev_2.5_v66.bin
RELEASE=2e
ARCHIVE=/path/to/plan9-$RELEASE.tar.bz2
TFTP=/path/to/tftp
ROOT=/path/to/plan9-2e-root

case "$RELEASE" in
  1e) EXPECTED_SHA256=8718e279aa35b10a9391f330976d31c177bad00de40e7f0f67232ffa54fb7d77 ;;
  2e) EXPECTED_SHA256=0bb3c1446deb79b179f73886eb2419ecfac9f9964040683e3d5731f074bc2ce6 ;;
  *) echo "unsupported Plan 9 release: $RELEASE" >&2; exit 1 ;;
esac
printf '%s  %s\n' "$EXPECTED_SHA256" "$ARCHIVE" | sha256sum -c - || exit 1
mkdir -p "$TFTP/68020" "$ROOT"
tar -xjf "$ARCHIVE" -C "$ROOT" --strip-components=1 "plan9-$RELEASE"
tar -xOf "$ARCHIVE" "plan9-$RELEASE/68020/9nextstation" \
  > "$TFTP/68020/9nextstation"

# First Edition probes for an SCC serial mouse before starting the network.
# QEMU already supplies the native NeXT keyboard/mouse device, so disable only
# this obsolete probe in the writable staged root.
if [ "$RELEASE" = 1e ]; then
  sed -i '/^[[:space:]]*aux\/mouse -dC 1$/s/^/# /' "$ROOT/rc/bin/termrc"
fi

"$QEMU" -M next-station -m 64M -bios "$ROM" \
  -global next-pc.system-timer-frequency=4456448 \
  -display gtk \
  -fsdev "local,id=plan9root,path=$ROOT,security_model=none" \
  -netdev "user,id=nextnet,ipv6=off,tftp=$TFTP,bootfile=68020/9nextstation" \
  -object "plan9-9p1-server,id=plan9fs,fsdev=plan9root,netdev=nextnet,guest-address=10.0.2.100,port=564" \
  -net "nic,model=next-mb8795,netdev=nextnet" \
  -no-reboot
```

At the ROM prompt, enter `ben() 68020/9nextstation`; select `tcp` as the root
source and `none` when the unauthenticated root asks for a user. From the
resulting root shell, the archived profile can be started with:

```rc
home=/usr/tor
. /usr/tor/lib/profile
```

Both editions have been exercised through visible 8½ startup. Second Edition
uses its archived root unchanged. First Edition requires only the staged-root
serial-mouse-probe workaround shown above; its archive and kernel remain
unchanged. The 9P1 service is for these historical kernels and is not a
9P2000 server.

## SCC serial DMA

The NeXT serial controller has one shared, bidirectional SCC DMA engine at
CSR `0x020000c0`. The engine can serve either SCC port, but not both at once;
if both ports request service simultaneously, channel A has priority.

The ESCC WR1 request gate selects the operation: `REQENABLE` (`0x80`) and
`REQFUNC` (`0x40`) must be set, while `REQRX` (`0x20`) selects receive and its
absence selects transmit. The DMA CSR `READ` direction must agree with the
WR1 direction; a mismatch remains pending. PIO interrupt 17 and DMA
interrupt 21 are independent.

The current segment is described by `NEXT`/`LIMIT` at
`0x020040c0`/`0x020040c4`. Setting `SUPDATE` enables a second segment in
`START`/`STOP` at `0x020040c8`/`0x020040cc`. Reaching a segment limit sets
`COMPLETE`; for a promoted segment, `ENABLE` remains set and software must
clear `COMPLETE` before the next segment runs. The final segment sets
`COMPLETE` and clears `ENABLE`. An enabled zero-length segment (`NEXT ==
LIMIT`) completes without a bus error. `NEXT > LIMIT`, or an invalid guest
memory transaction, sets `BUSEXC | COMPLETE`, clears the active state, and
asserts DMA interrupt 21.

Reset cancels pending SCC work and clears the DMA status; migration preserves
the guest-visible pointers, status, and serial state, then reconstructs request
and interrupt lines and resumes a serviceable transfer at the destination. If
a character backend applies backpressure, transmit DMA leaves `NEXT` unchanged
and retries when the backend becomes writable.

## Hardware support

| Hardware | Status | Notes |
|---|---|---|
| `next-cube` | Supported | 25 MHz MC68040 X15 system with up to 64 MiB and a monochrome display. |
| `next-station` | Supported | 25 MHz MC68040 Warp 9 system with up to 64 MiB and a monochrome display. |
| `next-station-color` | Supported | 25 MHz MC68040 Warp 9C system with up to 32 MiB and a 1120 × 832 RGB444 display. |
| MC68040 and FPU | Implemented | Includes the floating-point state needed by NeXT software. |
| DMA and interrupts | Implemented | One controller models the twelve physical NeXT DMA channels. |
| ESP SCSI and 82077 floppy | Implemented | Includes NeXT SCSI DMA staging and floppy DMA transfers. |
| SCC serial and DMA | Implemented | Both PIO channels use IRQ 17; the shared bidirectional DMA engine uses IRQ 21. |
| MB8795 Ethernet | Implemented | DMA transmit/receive, loopback, SLiRP networking, and NeXT BOOTP are available. |
| Video, keyboard, and mouse | Implemented | Monochrome/color output, retrace interrupts, serial keyboard, and absolute-pointer input are modeled. |
| RTC, event counter, and NVRAM | Implemented | NVRAM can be persisted with the `nvram-file` machine option. |
| NextBus | Partial | The Cube NBIC and an empty bus are present; cards are not modeled. |
| Optical formatter, DSP, and printer | Partial | Control registers are present; media, DSP execution, and printer data paths are incomplete. |
| Original MC68030 NeXT Computer/Cube | Not implemented | Requires the original 68030 PMMU and translation-register behavior. |
| Turbo systems | Not implemented | Turbo board variants and their timing are not modeled. |

Plan 9 support is limited to the original 9P1 protocol and IPv4 user-mode
networking. The v66 ROM is external, and this guide makes no claim of a
complete firmware diagnostic pass.
