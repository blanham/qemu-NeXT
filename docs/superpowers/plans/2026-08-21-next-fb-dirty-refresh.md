# NeXT Monochrome Framebuffer Dirty Refresh Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make monochrome scanout redraw only changed rows while preserving explicit full invalidation.

**Architecture:** Keep QEMU's generic framebuffer dirty bitmap as the source of truth.  Add trace-backed qtest observability, pass the real invalidation state into the helper, and publish only the returned dirty rectangle.

**Tech Stack:** QEMU C, qtest, QEMU trace events, Meson/Ninja

---

### Task 1: Add the redraw regression

**Files:**
- Modify: `tests/qtest/next-fb-test.c`
- Modify: `hw/display/trace-events`
- Modify: `hw/display/next-fb.c`

- [ ] **Step 1: Write the failing qtest**

Add a private trace/PPM fixture. Start `next-cube` with
`-trace enable=nextfb_update,file=trace->trace_path`, use QMP `screendump` to
request refreshes, write one byte into a middle VRAM row, reset the machine,
and parse the retained `nextfb_update` records. Assert an initial full redraw,
a clean `first=-1` refresh, a partial range containing the written row, and a
post-reset full redraw.

- [ ] **Step 2: Run the focused test and record RED**

Run: `build/pyvenv/bin/meson test -C build --print-errorlogs
'qemu:qtest-m68k/next-fb-test'`

Expected: FAIL because `nextfb_update` is not yet a trace event; after adding
only the event, FAIL because clean refreshes still report all 832 rows.

- [ ] **Step 3: Add trace observability**

Declare:

```text
nextfb_update(int first, int last, bool invalidate) "first=%d last=%d invalidate=%d"
```

Include the generated local trace header and emit the event after
`framebuffer_update_display()`.

- [ ] **Step 4: Implement the minimal dirty-row fix**

Capture `s->invalidate`, initialize the VRAM section only when needed, pass
the captured value rather than literal `1`, conditionally publish
`last - first + 1` rows when `first >= 0`, and clear invalidation afterward.

- [ ] **Step 5: Verify GREEN**

Run the focused qtest, the related color-video qtest, and build
`qemu-system-m68k`. Expected: all commands exit zero.

- [ ] **Step 6: Run static checks and commit**

Run `git diff --check` and QEMU `checkpatch.pl` over the feature commit. Stage
only the two documentation files, `next-fb.c`, `trace-events`, and
`next-fb-test.c`, then commit the implementation.

### Task 2: Integrate and publish

- [ ] **Step 1: Merge the verified feature branch into `metachicken`**

Use a normal fast-forward merge from the canonical QEMU worktree without
touching its unrelated untracked build directories.

- [ ] **Step 2: Verify the merged result**

Re-run the focused qtests and m68k build from the merged commit.

- [ ] **Step 3: Push**

Push `metachicken` to the `github` remote and verify that the remote ref equals
the local commit.
