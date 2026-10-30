# NeXTSTEP Sound Startup Handshake Design

## Problem

NeXTSTEP's original sound driver arms sound-out DMA, enables monitor sound
output, and immediately polls for either DMA progress or the monitor underrun
flag. QEMU's virtual-clock pacing waits for the first timer tick before doing
either. The driver therefore reports the otherwise impossible state
`neither dma nor underrun`.

## Design

When monitor sound output transitions from disabled to enabled while sound-out
DMA is armed, QEMU will request exactly one four-byte stereo frame immediately.
All later transfer remains paced by the existing virtual-clock timer. If DMA is
not armed, the existing timer-driven underrun behavior remains unchanged.

This restores the original hardware startup contract without reverting paced
audio, changing steady-state timing, forcing a healthy transfer through the
driver's underrun fallback, or modifying guest sources.

## Verification

A qtest will reproduce the original driver order and assert that the DMA next
register advances by one frame before any virtual-clock step. Existing sound
DMA, clock, backpressure, migration, and NeXT device qtests must continue to
pass. Guest acceptance will then confirm that startup no longer prints the
diagnostic and that the system remains responsive during sound playback.
