/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "libqtest.h"

#define NEXT_DMA_BASE       0x02000000
#define NEXT_VIDEO_CSR      (NEXT_DMA_BASE + 0x180)
#define NEXT_VIDEO_LIMIT    (NEXT_DMA_BASE + 0x180 + 0x4004)
#define NEXT_INTR_STATUS    0x02007000
#define NEXT_MONO_VIDEO_IRQ (1U << 5)
#define NEXT_DMA_RESET      0x00100000
#define NEXT_DMA_COMPLETE   0x08000000
#define NEXT_FB_BASE        0x0b000000
#define NEXT_FB_WIDTH       1120
#define NEXT_FB_HEIGHT      832
#define NEXT_FB_STRIDE      (NEXT_FB_WIDTH / 4 + 8)
#define NEXT_FB_DIRTY_ROW   400
#define NEXT_ROM_SIZE       (128 * KiB)
#define NEXT_RETRACE_NS     (NANOSECONDS_PER_SECOND / 68)

typedef struct TestROM {
    int fd;
    char *path;
} TestROM;

typedef struct TestMigration {
    char *tmpdir;
    char *socket_path;
} TestMigration;

typedef struct TestRefreshTrace {
    char *tmpdir;
    char *trace_path;
    char *ppm_path;
} TestRefreshTrace;

typedef struct RefreshEvent {
    int first;
    int last;
    int invalidate;
} RefreshEvent;

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

static void cleanup_refresh_trace(void *opaque)
{
    TestRefreshTrace *trace = opaque;

    qtest_remove_abrt_handler(trace);
    if (trace->trace_path) {
        g_unlink(trace->trace_path);
    }
    if (trace->ppm_path) {
        g_unlink(trace->ppm_path);
    }
    if (trace->tmpdir) {
        g_rmdir(trace->tmpdir);
    }
    g_free(trace->trace_path);
    g_free(trace->ppm_path);
    g_free(trace->tmpdir);
    g_free(trace);
}

static TestRefreshTrace *create_refresh_trace(void)
{
    g_autoptr(GError) error = NULL;
    TestRefreshTrace *trace = g_new0(TestRefreshTrace, 1);

    qtest_add_abrt_handler(cleanup_refresh_trace, trace);
    g_test_queue_destroy(cleanup_refresh_trace, trace);
    trace->tmpdir = g_dir_make_tmp("next-fb-refresh-XXXXXX", &error);
    g_assert_no_error(error);
    g_assert_nonnull(trace->tmpdir);
    trace->trace_path = g_build_filename(trace->tmpdir, "updates.log", NULL);
    trace->ppm_path = g_build_filename(trace->tmpdir, "screen.ppm", NULL);

    return trace;
}

static QTestState *next_fb_start(const char *machine, const char *extra_args)
{
    TestROM *rom = g_new0(TestROM, 1);
    g_autofree char *quoted_rom_path = NULL;

    rom->fd = -1;
    qtest_add_abrt_handler(cleanup_test_rom, rom);
    g_test_queue_destroy(cleanup_test_rom, rom);

    rom->fd = g_file_open_tmp("next-fb-rom-XXXXXX", &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;

    quoted_rom_path = g_shell_quote(rom->path);
    return qtest_initf("-machine %s -m 32M -bios %s %s",
                       machine, quoted_rom_path, extra_args ?: "");
}

static uint32_t mono_irq_status(QTestState *qts)
{
    return qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_MONO_VIDEO_IRQ;
}

static void take_screendump(QTestState *qts, const char *path)
{
    qtest_qmp_assert_success(
        qts,
        "{ 'execute': 'screendump', 'arguments': { 'filename': %s } }",
        path);
}

static GArray *load_refresh_events(const char *path)
{
    g_autoptr(GError) error = NULL;
    g_autofree char *contents = NULL;
    g_auto(GStrv) lines = NULL;
    gsize length;
    GArray *events = g_array_new(false, false, sizeof(RefreshEvent));

    g_assert_true(g_file_get_contents(path, &contents, &length, &error));
    g_assert_no_error(error);
    lines = g_strsplit(contents, "\n", -1);
    for (char **line = lines; *line; line++) {
        const char *record = strstr(*line, "nextfb_update ");
        RefreshEvent event;

        if (!record) {
            continue;
        }
        if (sscanf(record,
                   "nextfb_update first=%d last=%d invalidate=%d",
                   &event.first, &event.last, &event.invalidate) == 3) {
            g_array_append_val(events, event);
        }
    }

    return events;
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

static void test_retrace_migration(void)
{
    TestMigration *migration = create_test_migration();
    g_autofree char *uri =
        g_strdup_printf("unix:%s", migration->socket_path);
    QTestState *destination = next_fb_start("next-cube", "-incoming defer");
    QTestState *source = next_fb_start("next-cube", NULL);
    const int64_t source_phase = NEXT_RETRACE_NS / 2;
    const int64_t source_fraction = NEXT_RETRACE_NS / 3;
    const int64_t remaining = NEXT_RETRACE_NS - source_fraction;
    const int64_t source_elapsed = 5 * NEXT_RETRACE_NS + source_fraction;
    const int64_t destination_elapsed =
        3 * NEXT_RETRACE_NS + NEXT_RETRACE_NS / 5;
    int64_t source_clock;

    qtest_qmp_assert_success(
        destination,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);

    qtest_clock_step(destination, destination_elapsed);
    qtest_writel(destination, NEXT_VIDEO_CSR, NEXT_DMA_RESET);

    qtest_clock_step(source, source_phase);
    qtest_system_reset(source);
    source_clock = qtest_clock_step(source, source_elapsed);
    qtest_writel(source, NEXT_VIDEO_CSR, NEXT_DMA_RESET);
    g_assert_cmphex(mono_irq_status(source), ==, 0);

    migrate_wait(source, destination, uri);
    g_assert_cmpint(qtest_clock_set(destination, source_clock), ==,
                   source_clock);
    g_assert_cmphex(mono_irq_status(destination), ==, 0);

    qtest_clock_step(destination, remaining - 1);
    g_assert_cmphex(mono_irq_status(destination), ==, 0);
    qtest_clock_step(destination, 1);
    g_assert_cmphex(mono_irq_status(destination), ==, NEXT_MONO_VIDEO_IRQ);
    g_assert_cmphex(qtest_readl(destination, NEXT_VIDEO_CSR) & NEXT_DMA_COMPLETE,
                    ==, NEXT_DMA_COMPLETE);
    qtest_writel(destination, NEXT_VIDEO_CSR, NEXT_DMA_RESET);
    g_assert_cmphex(mono_irq_status(destination), ==, 0);
    g_assert_cmphex(qtest_readl(destination, NEXT_VIDEO_CSR) & NEXT_DMA_COMPLETE,
                    ==, 0);

    qtest_quit(source);
    qtest_quit(destination);
}

static void test_dirty_row_refresh(void)
{
    TestRefreshTrace *trace = create_refresh_trace();
    g_autofree char *quoted_trace_path = g_shell_quote(trace->trace_path);
    g_autofree char *args =
        g_strdup_printf("-trace enable=nextfb_update,file=%s",
                        quoted_trace_path);
    QTestState *qts = next_fb_start("next-cube", args);
    g_autoptr(GArray) events = NULL;
    bool saw_clean = false;
    bool saw_partial = false;
    bool saw_reset_full = false;

    take_screendump(qts, trace->ppm_path);
    take_screendump(qts, trace->ppm_path);
    qtest_writeb(qts, NEXT_FB_BASE + NEXT_FB_DIRTY_ROW * NEXT_FB_STRIDE,
                 0xff);
    take_screendump(qts, trace->ppm_path);
    qtest_system_reset(qts);
    take_screendump(qts, trace->ppm_path);
    qtest_quit(qts);

    events = load_refresh_events(trace->trace_path);
    g_assert_cmpuint(events->len, >=, 4);
    for (guint i = 0; i < events->len; i++) {
        const RefreshEvent *event = &g_array_index(events, RefreshEvent, i);

        if (!saw_clean && event->invalidate == 0 && event->first == -1) {
            saw_clean = true;
            continue;
        }
        if (saw_clean && !saw_partial && event->invalidate == 0 &&
            event->first >= 0 && event->last < NEXT_FB_HEIGHT &&
            event->first <= NEXT_FB_DIRTY_ROW &&
            event->last >= NEXT_FB_DIRTY_ROW) {
            saw_partial = true;
            continue;
        }
        if (saw_partial && event->invalidate == 1 && event->first == 0 &&
            event->last == NEXT_FB_HEIGHT - 1) {
            saw_reset_full = true;
            break;
        }
    }

    g_assert_true(saw_clean);
    g_assert_true(saw_partial);
    g_assert_true(saw_reset_full);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/next-fb/free-running-retrace",
                   test_free_running_retrace);
    qtest_add_func("/next-fb/color-has-no-mono-retrace",
                   test_color_has_no_mono_retrace);
    qtest_add_func("/next-fb/retrace-migration", test_retrace_migration);
    qtest_add_func("/next-fb/dirty-row-refresh", test_dirty_row_refresh);
    return g_test_run();
}
