# NeXT Monochrome Video Retrace Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generate a 68 Hz monochrome flyback interrupt without requiring a guest video-DMA limit, so the archived Plan 9 kernels receive scheduler ticks and continue past `schedinit()`.

**Architecture:** `next-fb` owns the physical retrace timer and emits a pulse once per frame. `next-dma` accepts that pulse, latches video-channel `COMPLETE`, and keeps its existing interrupt output asserted until the guest acknowledges the DMA CSR; the NeXT machine connects the two only for monochrome profiles.

**Tech Stack:** QEMU QOM/sysbus devices, virtual-clock `QEMUTimer`, qdev GPIO, VMState migration, m68k qtests, Meson/Ninja.

---

## File map

- Create `tests/qtest/next-fb-test.c`: focused black-box coverage of free-running monochrome flyback, color isolation, DMA acknowledgement, and timer migration.
- Modify `tests/qtest/meson.build`: register the focused m68k qtest.
- Modify `include/hw/dma/next-dma.h`: publish the name of the framebuffer-to-DMA retrace input.
- Modify `hw/dma/next-dma.c`: replace DMA-owned retrace generation with a pulse input while retaining the old timer field only for migration compatibility.
- Modify `hw/display/next-fb.c`: move the file to the project NCSA license and add the 68 Hz timer, output IRQ, reset/unrealize lifecycle, and VMState.
- Modify `hw/m68k/next-cube.c`: instantiate monochrome video explicitly and connect its retrace output to the shared DMA device.

### Task 1: Add the failing hardware contract

**Files:**
- Create: `tests/qtest/next-fb-test.c`
- Modify: `tests/qtest/meson.build`

- [ ] **Step 1: Add the new qtest to the m68k build**

Append `next-fb-test` to the `CONFIG_NEXTCUBE` list next to the other NeXT display tests:

```meson
qtests_m68k = ['boot-serial-test'] + \
  qtests_filter + \
  (config_all_devices.has_key('CONFIG_NEXTCUBE') ?
   ['next-machine-test',
    'next-color-video-test',
    'next-fb-test',
    'next-nbic-test',
```

- [ ] **Step 2: Write the focused test fixture**

Create `tests/qtest/next-fb-test.c` with the normal QEMU test license, a temporary 128 KiB ROM, and helpers that can start either monochrome or color machines:

```c
/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "libqtest.h"

#define NEXT_DMA_BASE           0x02000000
#define NEXT_VIDEO_CSR          (NEXT_DMA_BASE + 0x180)
#define NEXT_VIDEO_LIMIT        (NEXT_DMA_BASE + 0x180 + 0x4004)
#define NEXT_INTR_STATUS        0x02007000
#define NEXT_MONO_VIDEO_IRQ     (1U << 5)
#define NEXT_DMA_RESET          0x00100000
#define NEXT_DMA_COMPLETE       0x08000000
#define NEXT_ROM_SIZE           (128 * KiB)
#define NEXT_RETRACE_NS         (NANOSECONDS_PER_SECOND / 68)

typedef struct TestROM {
    int fd;
    char *path;
} TestROM;

static void cleanup_test_rom(void *opaque)
{
    TestROM *rom = opaque;

    qtest_remove_abrt_handler(rom);
    if (rom->fd >= 0) {
        close(rom->fd);
    }
    if (rom->path) {
        g_unlink(rom->path);
        g_free(rom->path);
    }
    g_free(rom);
}

static QTestState *next_fb_start(const char *machine, const char *extra_args)
{
    TestROM *rom = g_new0(TestROM, 1);
    g_autofree char *quoted_rom = NULL;

    rom->fd = -1;
    qtest_add_abrt_handler(cleanup_test_rom, rom);
    g_test_queue_destroy(cleanup_test_rom, rom);
    rom->fd = g_file_open_tmp("next-fb-rom-XXXXXX", &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;
    quoted_rom = g_shell_quote(rom->path);

    return qtest_initf("-machine %s -m 32M -bios %s %s",
                       machine, quoted_rom, extra_args ?: "");
}

static uint32_t mono_irq_status(QTestState *qts)
{
    return qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_MONO_VIDEO_IRQ;
}
```

- [ ] **Step 3: Add the no-limit and color-isolation regressions**

The monochrome test must never write `NEXT_VIDEO_LIMIT`; it proves physical flyback is independent of guest DMA buffer setup. The color test proves the new source is absent on a color board:

```c
static void test_free_running_retrace(void)
{
    QTestState *qts = next_fb_start("next-cube", NULL);

    g_assert_cmphex(qtest_readl(qts, NEXT_VIDEO_LIMIT), ==, 0);
    g_assert_cmphex(mono_irq_status(qts), ==, 0);
    qtest_clock_step(qts, NEXT_RETRACE_NS - 1);
    g_assert_cmphex(mono_irq_status(qts), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(mono_irq_status(qts), ==, NEXT_MONO_VIDEO_IRQ);
    g_assert_cmphex(qtest_readl(qts, NEXT_VIDEO_CSR) & NEXT_DMA_COMPLETE,
                    ==, NEXT_DMA_COMPLETE);

    qtest_writel(qts, NEXT_VIDEO_CSR, NEXT_DMA_RESET);
    g_assert_cmphex(mono_irq_status(qts), ==, 0);
    qtest_clock_step(qts, NEXT_RETRACE_NS);
    g_assert_cmphex(mono_irq_status(qts), ==, NEXT_MONO_VIDEO_IRQ);

    qtest_quit(qts);
}

static void test_color_has_no_mono_retrace(void)
{
    QTestState *qts = next_fb_start("next-station-color", NULL);

    qtest_clock_step(qts, 2 * NEXT_RETRACE_NS);
    g_assert_cmphex(mono_irq_status(qts), ==, 0);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/next-fb/free-running-retrace",
                   test_free_running_retrace);
    qtest_add_func("/next-fb/color-has-no-mono-retrace",
                   test_color_has_no_mono_retrace);
    return g_test_run();
}
```

- [ ] **Step 4: Build and prove the regression is red for the intended reason**

Run:

```bash
ninja -C build tests/qtest/next-fb-test
build/tests/qtest/next-fb-test -p /next-fb/free-running-retrace
```

Expected: the binary builds, then `free-running-retrace` fails because interrupt-status bit 5 remains clear after one frame when the video DMA limit is zero. The failure must not be a launch, ROM, or QOM error.

- [ ] **Step 5: Commit the red test**

```bash
git add tests/qtest/next-fb-test.c tests/qtest/meson.build
git commit -m "tests/qtest: require free-running NeXT mono retrace"
```

### Task 2: Make the DMA controller a retrace latch

**Files:**
- Modify: `include/hw/dma/next-dma.h`
- Modify: `hw/dma/next-dma.c`

- [ ] **Step 1: Publish the named DMA input**

Add beside `TYPE_NEXT_DMA` in `include/hw/dma/next-dma.h`:

```c
#define NEXT_DMA_VIDEO_RETRACE_GPIO "video-retrace"
```

- [ ] **Step 2: Replace the active DMA timer callback with a GPIO handler**

Delete `NEXT_DMA_VIDEO_RETRACE_HZ`, `NEXT_DMA_VIDEO_RETRACE_NS`, `next_dma_video_retrace_enabled()`, and `next_dma_video_retrace_schedule()`. Replace `next_dma_video_retrace(void *opaque)` with:

```c
static void next_dma_video_retrace_in(void *opaque, int n, int level)
{
    NextDMAState *s = opaque;

    if (!level) {
        return;
    }

    s->channel[NEXT_DMA_VIDEO].csr |= NEXT_DMA_CSR_COMPLETE;
    next_dma_update_irq(s, NEXT_DMA_VIDEO);
}

static void next_dma_legacy_video_retrace(void *opaque)
{
    /* Retained only as the callback for the version-2 migration timer. */
}
```

Keep the `QEMUTimer video_retrace_timer` field and its existing `VMSTATE_TIMER_V(..., 2)` entry so snapshots made by the development branch retain the same wire field. It is now a compatibility-only timer.

- [ ] **Step 3: Remove video-limit coupling and cancel legacy state after load**

Remove this block from `next_dma_write()`:

```c
if (resolved.channel == NEXT_DMA_VIDEO &&
    resolved.reg == NEXT_DMA_REGISTER_LIMIT) {
    next_dma_video_retrace_schedule(s);
}
```

Replace the old post-load scheduling branch with an unconditional cancellation after IRQ reconstruction:

```c
for (channel = 0; channel < NEXT_DMA_CHANNEL_COUNT; channel++) {
    next_dma_update_irq(s, channel);
}
timer_del(&s->video_retrace_timer);
```

The existing reset-time `timer_del()` remains. It safely discards any legacy deadline and must not clear migrated DMA `COMPLETE` outside normal reset.

- [ ] **Step 4: Register the GPIO and no-op compatibility timer**

In `next_dma_init()` use:

```c
timer_init_ns(&s->video_retrace_timer, QEMU_CLOCK_VIRTUAL,
              next_dma_legacy_video_retrace, s);
qdev_init_gpio_in_named(DEVICE(obj), next_dma_video_retrace_in,
                        NEXT_DMA_VIDEO_RETRACE_GPIO, 1);
```

Keep all existing sysbus IRQ outputs unchanged; video remains channel index `NEXT_DMA_VIDEO` and therefore still routes to peripheral-controller bit 5.

- [ ] **Step 5: Build the modified device**

Run:

```bash
ninja -C build qemu-system-m68k tests/qtest/next-fb-test
```

Expected: build succeeds. The focused monochrome test is still red because `next-fb` does not emit the new input yet.

- [ ] **Step 6: Commit the DMA refactor**

```bash
git add include/hw/dma/next-dma.h hw/dma/next-dma.c
git commit -m "next-dma: accept monochrome retrace pulses"
```

### Task 3: Generate flyback in the monochrome framebuffer

**Files:**
- Modify: `hw/display/next-fb.c`
- Modify: `hw/m68k/next-cube.c`

- [ ] **Step 1: Apply the project NCSA header and timer dependencies**

Replace the legacy MIT-style header in `hw/display/next-fb.c` with:

```c
/* SPDX-License-Identifier: NCSA
 *
 * Copyright (c) 2011-2026 Bryce Lanham
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal with the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to
 * whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimers.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimers in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the names of the University of Illinois/NCSA nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * Software without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS WITH THE SOFTWARE.
 */
```

Add:

```c
#include "migration/vmstate.h"
#include "hw/core/irq.h"
#include "qemu/timer.h"
#include "qemu/units.h"
```

- [ ] **Step 2: Add the retrace-owned device state and constants**

Add:

```c
#define NEXT_FB_RETRACE_HZ 68
#define NEXT_FB_RETRACE_NS (NANOSECONDS_PER_SECOND / NEXT_FB_RETRACE_HZ)

struct NeXTFbState {
    SysBusDevice parent_obj;

    MemoryRegion fb_mr;
    MemoryRegionSection fbsection;
    QemuConsole *con;
    qemu_irq retrace_irq;
    QEMUTimer retrace_timer;

    uint32_t cols;
    uint32_t rows;
    int invalidate;
};
```

- [ ] **Step 3: Add periodic pulse, reset, migration, and unrealize lifecycle**

Add these functions before class initialization:

```c
static void nextfb_schedule_retrace(NeXTFbState *s)
{
    timer_mod(&s->retrace_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + NEXT_FB_RETRACE_NS);
}

static void nextfb_retrace(void *opaque)
{
    NeXTFbState *s = opaque;

    qemu_irq_pulse(s->retrace_irq);
    nextfb_schedule_retrace(s);
}

static void nextfb_reset_hold(Object *obj, ResetType type)
{
    NeXTFbState *s = NEXTFB(obj);

    timer_del(&s->retrace_timer);
    s->invalidate = 1;
    nextfb_schedule_retrace(s);
}

static const VMStateDescription vmstate_nextfb = {
    .name = TYPE_NEXTFB,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(retrace_timer, NeXTFbState),
        VMSTATE_END_OF_LIST()
    },
};

static void nextfb_unrealize(DeviceState *dev)
{
    NeXTFbState *s = NEXTFB(dev);

    timer_del(&s->retrace_timer);
    if (s->fbsection.mr) {
        memory_region_set_log(s->fbsection.mr, false, DIRTY_MEMORY_VGA);
        memory_region_unref(s->fbsection.mr);
        s->fbsection.mr = NULL;
    }
    if (s->con) {
        qemu_graphic_console_close(s->con);
        s->con = NULL;
    }
}

static void nextfb_init(Object *obj)
{
    NeXTFbState *s = NEXTFB(obj);

    timer_init_ns(&s->retrace_timer, QEMU_CLOCK_VIRTUAL,
                  nextfb_retrace, s);
}
```

VMState intentionally stores only the physical timer deadline. The DMA device remains authoritative for the latched completion/IRQ state.

- [ ] **Step 4: Expose the retrace output and install lifecycle hooks**

In `nextfb_realize()` add:

```c
sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->retrace_irq);
```

Update class and type initialization:

```c
static void nextfb_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    dc->realize = nextfb_realize;
    dc->unrealize = nextfb_unrealize;
    dc->vmsd = &vmstate_nextfb;
    rc->phases.hold = nextfb_reset_hold;
}

static const TypeInfo nextfb_info = {
    .name = TYPE_NEXTFB,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NeXTFbState),
    .instance_init = nextfb_init,
    .class_init = nextfb_class_init,
};
```

- [ ] **Step 5: Wire only monochrome profiles to DMA**

Replace the `sysbus_create_simple(TYPE_NEXTFB, ...)` call in `next_machine_init()` with an explicit child device and connection:

```c
if (profile->video_kind == NEXT_VIDEO_MONO) {
    DeviceState *fb_dev = qdev_new(TYPE_NEXTFB);
    SysBusDevice *fb_sbd = SYS_BUS_DEVICE(fb_dev);

    object_property_add_child(OBJECT(machine), "next-fb", OBJECT(fb_dev));
    sysbus_realize_and_unref(fb_sbd, &error_fatal);
    sysbus_mmio_map(fb_sbd, 0, 0x0B000000);
    sysbus_connect_irq(
        fb_sbd, 0,
        qdev_get_gpio_in_named(dma_dev, NEXT_DMA_VIDEO_RETRACE_GPIO, 0));
} else {
```

Do not change the separate `next-color-video` timer or its bit-13 interrupt path.

- [ ] **Step 6: Run the focused tests to green**

Run:

```bash
ninja -C build qemu-system-m68k tests/qtest/next-fb-test
build/tests/qtest/next-fb-test
```

Expected: both `free-running-retrace` and `color-has-no-mono-retrace` pass.

- [ ] **Step 7: Commit the device implementation**

```bash
git add hw/display/next-fb.c hw/m68k/next-cube.c
git commit -m "next-fb: generate monochrome video retrace"
```

### Task 4: Prove timer migration and run the NeXT regression suite

**Files:**
- Modify: `tests/qtest/next-fb-test.c`

- [ ] **Step 1: Add migration cleanup and synchronization helpers**

Extend `tests/qtest/next-fb-test.c` with:

```c
typedef struct TestMigration {
    char *tmpdir;
    char *socket_path;
} TestMigration;

static void cleanup_test_migration(void *opaque)
{
    TestMigration *migration = opaque;

    qtest_remove_abrt_handler(migration);
    if (migration->socket_path) {
        g_unlink(migration->socket_path);
    }
    if (migration->tmpdir) {
        g_rmdir(migration->tmpdir);
    }
    g_free(migration->socket_path);
    g_free(migration->tmpdir);
    g_free(migration);
}

static TestMigration *create_test_migration(void)
{
    g_autoptr(GError) error = NULL;
    TestMigration *migration = g_new0(TestMigration, 1);

    qtest_add_abrt_handler(cleanup_test_migration, migration);
    g_test_queue_destroy(cleanup_test_migration, migration);
    migration->tmpdir = g_dir_make_tmp("next-fb-migration-XXXXXX", &error);
    g_assert_no_error(error);
    g_assert_nonnull(migration->tmpdir);
    migration->socket_path =
        g_build_filename(migration->tmpdir, "migration.sock", NULL);
    return migration;
}

static void migrate_wait(QTestState *source, QTestState *destination,
                         const char *uri)
{
    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    qtest_qmp_eventwait(source, "STOP");
    qtest_qmp_eventwait(destination, "RESUME");
}
```

- [ ] **Step 2: Add the active-deadline and latched-state migration test**

Use deliberately different pre-migration clocks so the test cannot pass merely because source and destination happened to schedule identical reset deadlines:

```c
static void test_retrace_migration(void)
{
    TestMigration *migration = create_test_migration();
    g_autofree char *uri =
        g_strdup_printf("unix:%s", migration->socket_path);
    QTestState *destination =
        next_fb_start("next-cube", "-incoming defer");
    QTestState *source = next_fb_start("next-cube", NULL);
    const int64_t source_elapsed = 5 * NEXT_RETRACE_NS +
                                   NEXT_RETRACE_NS / 3;
    const int64_t destination_elapsed = 3 * NEXT_RETRACE_NS +
                                        NEXT_RETRACE_NS / 5;
    int64_t source_clock;

    qtest_qmp_assert_success(
        destination,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);

    qtest_clock_step(destination, destination_elapsed);
    qtest_writel(destination, NEXT_VIDEO_CSR, NEXT_DMA_RESET);
    source_clock = qtest_clock_step(source, source_elapsed);
    qtest_writel(source, NEXT_VIDEO_CSR, NEXT_DMA_RESET);
    g_assert_cmphex(mono_irq_status(source), ==, 0);

    migrate_wait(source, destination, uri);
    qtest_clock_set(destination, source_clock);
    g_assert_cmphex(mono_irq_status(destination), ==, 0);
    qtest_clock_step(destination, NEXT_RETRACE_NS - 1);
    g_assert_cmphex(mono_irq_status(destination), ==, 0);
    qtest_clock_step(destination, 1);
    g_assert_cmphex(mono_irq_status(destination), ==, NEXT_MONO_VIDEO_IRQ);
    qtest_writel(destination, NEXT_VIDEO_CSR, NEXT_DMA_RESET);
    g_assert_cmphex(mono_irq_status(destination), ==, 0);

    qtest_quit(source);
    qtest_quit(destination);
}
```

Register it in `main()`:

```c
qtest_add_func("/next-fb/retrace-migration", test_retrace_migration);
```

- [ ] **Step 3: Run the focused migration test**

Run:

```bash
ninja -C build tests/qtest/next-fb-test
build/tests/qtest/next-fb-test -p /next-fb/retrace-migration
```

Expected: pass, with no immediate stale destination pulse and a new bit-5 interrupt exactly at the migrated remaining deadline.

- [ ] **Step 4: Correct the older DMA retrace contract**

Replace `test_video_retrace_interrupt()` in `tests/qtest/next-dma-test.c` with a limit-independent test. It deliberately writes both nonzero and zero limits after the first frame to prove neither value gates flyback:

```c
static void test_video_retrace_interrupt(void)
{
    QTestState *qts = next_dma_start();
    const TestChannel *video = &channels[9];
    uint64_t csr = NEXT_DMA_BASE + video->csr;
    uint64_t limit = channel_address(video, 0x4004);

    intercept_next_pc_inputs(qts);

    g_assert_cmphex(qtest_readl(qts, limit), ==, 0);
    g_assert_false(qtest_get_irq(qts, dma_board_inputs[9]));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_VIDEO_IRQ,
                    ==, 0);

    qtest_clock_step(qts, NEXT_VIDEO_RETRACE_NS - 1);
    g_assert_false(qtest_get_irq(qts, dma_board_inputs[9]));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_VIDEO_IRQ,
                    ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_true(qtest_get_irq(qts, dma_board_inputs[9]));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_VIDEO_IRQ,
                    ==, NEXT_VIDEO_IRQ);

    qtest_writel(qts, csr, DMA_RESET);
    qtest_writel(qts, limit, NEXT_VIDEO_LIMIT);
    qtest_clock_step(qts, NEXT_VIDEO_RETRACE_NS);
    g_assert_true(qtest_get_irq(qts, dma_board_inputs[9]));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_VIDEO_IRQ,
                    ==, NEXT_VIDEO_IRQ);

    qtest_writel(qts, csr, DMA_RESET);
    qtest_writel(qts, limit, 0);
    qtest_clock_step(qts, NEXT_VIDEO_RETRACE_NS);
    g_assert_true(qtest_get_irq(qts, dma_board_inputs[9]));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_VIDEO_IRQ,
                    ==, NEXT_VIDEO_IRQ);

    qtest_quit(qts);
}
```

- [ ] **Step 5: Run all affected qtests**

Run:

```bash
build/tests/qtest/next-fb-test
build/tests/qtest/next-dma-test
build/tests/qtest/next-color-video-test
build/tests/qtest/next-machine-test
build/tests/qtest/next-mb8795-test
build/tests/qtest/next-cube-timer-test
```

Expected: every test passes, including the limit-independent assertions in both focused retrace tests.

- [ ] **Step 6: Run formatting and diff checks**

Run:

```bash
git clang-format --diff HEAD~3
git diff --check
git status --short
```

Expected: no formatting or whitespace errors; only intended tracked changes are present.

- [ ] **Step 7: Commit migration coverage and the corrected legacy assertion**

```bash
git add tests/qtest/next-fb-test.c tests/qtest/next-dma-test.c
git commit -m "tests/qtest: cover NeXT retrace migration"
```

### Task 5: Visible Plan 9 Second Edition acceptance

**Files:**
- Use: `/home/blanham/projects/NeXT/lab/.worktrees/plan9-next-netboot`
- Use: `/home/blanham/projects/NeXT/archive/plan9/plan9-2e.tar.bz2`
- Create at runtime: a unique directory below `/home/blanham/projects/NeXT/lab/work/plan9-runs/`

- [ ] **Step 1: Perform the completion verification audit**

Read and follow `superpowers:verification-before-completion`. Confirm the branch contains both the SLiRP overlay fix commit `538944022e` and all retrace commits, and confirm both worktrees are otherwise clean.

- [ ] **Step 2: Shut down only the currently controller-owned VM**

Send `quit` to unified execution session `77818`. The controller must issue QMP `quit` to its authenticated child PID. Do not enumerate, signal, or terminate any other QEMU process.

- [ ] **Step 3: Stage a unique visible run**

Use the Plan 9 netboot launcher to create a new run directory, stage the verified Second Edition file as `tftp/68020/9nextstation`, and record its expected SHA-256:

```text
0bb3c1446deb79b179f73886eb2419ecfac9f9964040683e3d5731f074bc2ce6
```

Launch the rebuilt QEMU with GTK and its named QMP socket. The controller must park at `NeXT>` and wait for the user's exact `boot` confirmation.

- [ ] **Step 4: Boot only after the user confirms they are watching**

After the user says `boot`, inject exactly:

```text
ben() 68020/9nextstation
```

Keep the window open. Success requires output to advance beyond the memory-statistics screen into the first user-process boot initialization; visible changing runtime counters alone are supporting evidence, not a substitute for boot progress.

- [ ] **Step 5: Record acceptance evidence and commit final test adjustments**

Record the run directory, QMP transcript, kernel checksum, and visible checkpoint in the lab run evidence. Commit only files belonging to their respective worktrees. Do not merge or push until the user asks for integration.
