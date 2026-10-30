# NeXTSTEP Sound Startup Handshake Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore the immediate sound-DMA startup progress required by the original NeXTSTEP driver while retaining virtual-clock pacing afterward.

**Architecture:** Add one qtest that reproduces the guest driver's register order and observes the DMA next register without advancing virtual time. On the disabled-to-enabled monitor-control transition, consume one frame through the existing `next_sound_consume_dma` path, then leave all subsequent work to the existing timer.

**Tech Stack:** QEMU C device model, libqtest, Meson-generated Ninja build

---

### Task 1: Reproduce the startup contract

**Files:**
- Modify: `tests/qtest/next-dma-test.c`

- [ ] **Step 1: Write the failing test**

Add `test_sound_output_immediate_startup_progress`. Configure a 16-byte aligned
sound DMA buffer, set `DMA_SETENABLE`, write `NEXT_DMAOUT_DMAEN`, send
`MON_SNDOUT_CTRL(SOUT_ENAB)`, and write monitor data. Without calling
`qtest_clock_step`, assert that `next == NEXT_TEST_RAM_BASE + 4`, DMA remains
enabled and incomplete, and underrun is clear. Register the test as
`/next/dma/sound-output-immediate-startup-progress`.

- [ ] **Step 2: Build and verify RED**

Run:

```sh
ninja -C /mnt/build/NeXT/next265-sound-startup-build-20261030 qemu-system-m68k tests/qtest/next-dma-test
/mnt/build/NeXT/next265-sound-startup-build-20261030/tests/qtest/next-dma-test -p /next/dma/sound-output-immediate-startup-progress
```

Expected: the test fails because the next register remains at the buffer start.

### Task 2: Restore immediate first-frame progress

**Files:**
- Modify: `hw/audio/next-sound.c`

- [ ] **Step 1: Implement the minimal transition behavior**

In the monitor sound-control command path, remember whether output was enabled.
After updating `s->control`, when output has just transitioned to enabled and
sound DMA is enabled and incomplete, call:

```c
next_sound_consume_dma(s, 1, false);
```

Keep the existing `next_sound_kick(s)` call so subsequent frames remain timer
paced.

- [ ] **Step 2: Verify GREEN**

Rebuild the same Ninja targets and rerun the focused qtest. Expected: PASS.

- [ ] **Step 3: Run the NeXT DMA/device regression cohort**

Run:

```sh
ninja -C /mnt/build/NeXT/next265-sound-startup-build-20261030 test-qapi test-qtest-m68k
```

Expected: all selected tests pass.

- [ ] **Step 4: Commit and push**

Commit the design, plan, test, and implementation as reviewable commits on
`fix/nextstep-sound-startup-20261030`, then push each commit to `github`.

### Task 3: Guest acceptance

**Files:**
- Create: `/mnt/build/llvm/next265-sound-acceptance-20261030/RESULT.md`

- [ ] **Step 1: Build the candidate binary**

Run the normal Ninja `qemu-system-m68k` target and record the source commit and
binary SHA256.

- [ ] **Step 2: Boot one headless disposable guest**

Use a fresh child disk, no graphical window, and the existing accepted ROM and
disk lineage. Exercise a sound file through the original sound provider.

- [ ] **Step 3: Record acceptance**

Confirm the `neither dma nor underrun` diagnostic is absent, DMA completes,
the guest remains responsive, and the owned QEMU process is stopped. Record
commands, hashes, results, and limitations in `RESULT.md`.

- [ ] **Step 4: Integrate by rebase**

Rebase the feature branch onto current `github/metachicken`, rerun the focused
tests, fast-forward `metachicken`, and push. Do not merge.
