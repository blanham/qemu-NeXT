# NeXT Plan 9 Timer Frequency Override Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an opt-in NeXT system-timer frequency override and select it only for the historical Plan 9 Second Edition launch profile.

**Architecture:** The `next-pc` QOM device owns a static timer-frequency property whose 1 MHz default preserves hardware behavior. System-timer count/deadline conversions use that property, while the event counter keeps its independent 1 MHz timebase. The lab launcher passes 4,456,448 Hz only for Plan 9 netboot.

**Tech Stack:** QEMU QOM/qdev properties, QEMU virtual timers, qtest, Python `unittest`, Meson/Ninja.

---

### Task 1: Add the `next-pc` system-timer frequency property

**Files:**
- Modify: `tests/qtest/next-cube-timer-test.c`
- Modify: `hw/m68k/next-cube.c`

- [ ] **Step 1: Write failing deadline, event-counter, and validation tests**

Add `NEXT_PLAN9_TIMER_FREQUENCY` and a ceiling-division helper to the timer
qtest. Start a machine with
`-global next-pc.system-timer-frequency=4456448`, arm `0xffff`, and assert that
the interrupt is absent one nanosecond before the scaled deadline and present
at the deadline. In the same machine, advance one millisecond and assert the
event counter advances by exactly 1000. Add subprocess launch tests asserting
that zero and 1,000,000,001 Hz fail with a message naming
`system-timer-frequency`.

```c
#define NEXT_PLAN9_TIMER_FREQUENCY UINT64_C(4456448)

static int64_t ticks_to_ns(uint64_t ticks, uint64_t frequency)
{
    return DIV_ROUND_UP(ticks * NANOSECONDS_PER_SECOND, frequency);
}

static void test_frequency_override(void)
{
    QTestState *qts = next_cube_timer_start_with_args(
        "-global next-pc.system-timer-frequency=4456448");
    int64_t deadline = ticks_to_ns(0xffff,
                                   NEXT_PLAN9_TIMER_FREQUENCY);

    arm_timer(qts, 0xffff);
    qtest_clock_step(qts, deadline - 1);
    g_assert_cmphex(timer_status(qts), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(timer_status(qts), ==, NEXT_INTR_TIMER);
    qtest_quit(qts);
}
```

- [ ] **Step 2: Run the focused test and record RED**

Configure an isolated m68k build and run:

```bash
./configure --target-list=m68k-softmmu --disable-slirp --disable-blkio
ninja -C build tests/qtest/next-cube-timer-test
build/tests/qtest/next-cube-timer-test
```

Expected: the new override test aborts because `next-pc` has no
`system-timer-frequency` property.

- [ ] **Step 3: Implement frequency conversion and validation**

Add `uint32_t system_timer_frequency` to `NeXTPC`, default it through
`DEFINE_PROP_UINT32`, and reject zero or values over one gigahertz in
`next_pc_realize()`:

```c
#define NEXT_SYSTEM_TIMER_DEFAULT_FREQUENCY UINT32_C(1000000)
#define NEXT_TIMER_MAX_FREQUENCY UINT32_C(1000000000)
#define NEXT_EVENTC_TICK_NS INT64_C(1000)

DEFINE_PROP_UINT32("system-timer-frequency", NeXTPC,
                   system_timer_frequency,
                   NEXT_SYSTEM_TIMER_DEFAULT_FREQUENCY),
```

Use overflow-safe integer scaling for both directions. Deadline conversion
rounds up; remaining-count conversion also rounds up and retains the current
full-period cap. Replace only the system timer's uses of the old shared
microsecond constant. Leave event-counter conversion on
`NEXT_EVENTC_TICK_NS`.

- [ ] **Step 4: Run the focused tests and commit**

```bash
ninja -C build tests/qtest/next-cube-timer-test
build/tests/qtest/next-cube-timer-test
git diff --check
git add hw/m68k/next-cube.c tests/qtest/next-cube-timer-test.c
git commit -m "hw/m68k: allow overriding the NeXT system timer frequency"
```

Expected: all NeXT timer cases pass, including the unchanged default, scaled
override, independent event counter, invalid values, and migration coverage.

### Task 2: Select the override in the Plan 9 launch contract

**Files:**
- Modify: `/home/blanham/projects/NeXT/lab/tests/test_plan9_netboot.py`
- Modify: `/home/blanham/projects/NeXT/lab/nextcube_lab/plan9_netboot.py`
- Modify: `QEMU-NeXT-README.md`

- [ ] **Step 1: Write the failing launcher assertion**

Update the exact expected argument list in
`Plan9LaunchTests.test_build_argv_attaches_next_nic_and_keeps_display_visible`
to require this pair immediately after the machine selection:

```python
"-global",
"next-pc.system-timer-frequency=4456448",
```

Also assert the global occurs exactly once.

- [ ] **Step 2: Run the focused Python test and record RED**

```bash
python3 -m unittest tests.test_plan9_netboot.Plan9LaunchTests
```

Expected: the exact argv comparison fails because the launcher omits the
frequency override.

- [ ] **Step 3: Add the Plan 9-only argument**

Add these two entries to `build_plan9_argv()` after `next-station`:

```python
"-global",
"next-pc.system-timer-frequency=4456448",
```

No generic ROM or disk launch function changes.

- [ ] **Step 4: Document the standalone command and commit both repositories**

Add the same `-global` option to the Plan 9 command line in
`QEMU-NeXT-README.md` and explain in one sentence that it compensates for the
historical kernel's `HZ=68`/`0xffff` mismatch while other guests retain the
1 MHz default.

```bash
python3 -m unittest tests.test_plan9_netboot.Plan9LaunchTests
git -C /home/blanham/projects/NeXT/lab diff --check
git -C /home/blanham/projects/NeXT/lab add \
  nextcube_lab/plan9_netboot.py tests/test_plan9_netboot.py
git -C /home/blanham/projects/NeXT/lab commit \
  -m "lab: accelerate the historical Plan 9 system clock"
git add QEMU-NeXT-README.md
git commit -m "docs: document the Plan 9 timer override"
```

Expected: the launcher test passes and the README command matches it.

### Task 3: Verify and integrate

**Files:**
- Verify only; no new production files.

- [ ] **Step 1: Run fresh focused and broader verification**

```bash
ninja -C build tests/qtest/next-cube-timer-test \
  tests/qtest/next-cube-timer-trace-test qemu-system-m68k
build/tests/qtest/next-cube-timer-test
build/tests/qtest/next-cube-timer-trace-test
python3 -m unittest tests.test_plan9_netboot
git diff --check
```

Expected: every command exits zero and the QEMU/lab worktrees contain only
the planned commits.

- [ ] **Step 2: Review the complete changes**

Compare the implementation against the design requirements: accurate 1 MHz
default, opt-in scaling, event-counter independence, invalid-value rejection,
migration behavior, and Plan 9-only launcher selection. Resolve all critical
or important findings and rerun Step 1.

- [ ] **Step 3: Merge locally and push `metachicken`**

After fresh post-merge verification, merge the QEMU feature branch into
`metachicken` and push it to the `github` remote. Merge the lab commit locally;
the lab repository currently has no configured remote, so do not invent one.

- [ ] **Step 4: Run visible acceptance only with the operator present**

Start the GTK Plan 9 profile with its private QMP socket, wait for operator
confirmation at the ROM prompt, and measure login-to-shell latency. Never
signal an unowned process; stop only the exact VM through its private QMP
socket.
