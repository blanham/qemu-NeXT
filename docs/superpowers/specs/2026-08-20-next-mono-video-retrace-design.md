# NeXT Monochrome Video Retrace Design

## Goal

Make monochrome NeXT machines generate the 68 Hz video-flyback interrupt
independently of video DMA buffer programming.  This must let the archived
Plan 9 NeXT kernels advance their scheduler clock without changing either
kernel.

## Evidence

Plan 9 First and Second Edition both reach `schedinit()` and spin in
`runproc()`.  Their `m->ticks` value cannot advance.  Both kernels enable
interrupt-status bit 5, dispatch that bit from the level-3 handler to
`clock()`, and acknowledge it by resetting the video DMA interrupt latch.
Neither kernel programs a video DMA limit.

QEMU currently starts monochrome retrace only when the video DMA limit is
nonzero.  That makes flyback depend on guest framebuffer DMA setup and leaves
Plan 9 without a clock source.  The separate color display controller does
not have this error.

## Architecture

`next-fb` represents the monochrome display and therefore owns a periodic
68 Hz virtual-clock timer.  Each expiry sends one retrace pulse to the shared
NeXT DMA controller.  The DMA controller latches `COMPLETE` on its video
channel, asserts its existing video interrupt output, and leaves the output
asserted until the guest acknowledges it through the video DMA CSR.

The machine connects this path only when the selected board profile uses the
monochrome framebuffer:

```text
next-fb 68 Hz pulse -> next-dma video latch -> peripheral controller bit 5
                                             -> m68k level-3 interrupt
```

Color machines retain the existing color-controller retrace path and
interrupt-status bit 13.  They do not instantiate `next-fb` and cannot
receive the monochrome interrupt.

The retrace timer and its deadline move out of `next-dma`; the DMA channel
retains only the state it owns: registers, completion latch, and interrupt
output.  Existing DMA reset commands continue to clear completion and lower
the interrupt.  Video DMA limit writes no longer start or stop physical
flyback.

## Device lifecycle and migration

`next-fb` schedules the first retrace one frame after reset and reschedules
after every expiry.  Unrealize deletes the timer.  Its VMState records the
timer deadline so an active monochrome display resumes with the same
remaining interval after migration.

The obsolete DMA-owned timer remains load-compatible for the current DMA
VMState version but is canceled after load and never drives a new machine.
The migrated DMA completion bit remains authoritative for whether interrupt
bit 5 is already asserted.  This prevents duplicate interrupt sources while
retaining compatibility with snapshots made by this development branch.

## Licensing

The moved/reimplemented monochrome hardware is licensed under the
UIUC/NCSA license with Bryce Lanham's existing copyright.  The existing
`next-fb.c` header is updated from its original permissive grant to the
project's standard NCSA text.  Qtests retain their normal QEMU test license.

## Tests

The regression starts a monochrome NeXTstation without touching the video
DMA limit.  Before one frame, status bit 5 must be clear; at one frame it
must be set.  Writing the video DMA reset command must clear it, and the next
frame must set it again.  A color-machine check must prove that advancing the
same virtual time does not set monochrome status bit 5.

Migration coverage verifies that the monochrome timer resumes at the
remaining deadline and that a latched interrupt remains acknowledgeable.
Existing DMA, color-video, machine, MB8795, and timer tests must remain green.

The final acceptance run uses the verified Plan 9 Second Edition kernel in a
visible GTK window.  It succeeds when output advances beyond the memory
statistics and the first user process begins boot initialization.  The VM
will again wait at `NeXT>` until the user confirms they are watching.
