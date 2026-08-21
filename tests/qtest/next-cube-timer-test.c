/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "libqtest.h"

#define NEXT_INTR_STATUS      0x02007000
#define NEXT_SCR2             0x0200d000
#define NEXT_SYSTEM_TIMER     0x02116000
#define NEXT_TIMER_HIGH       (NEXT_SYSTEM_TIMER + 0)
#define NEXT_TIMER_LOW        (NEXT_SYSTEM_TIMER + 1)
#define NEXT_TIMER_PADDING_2  (NEXT_SYSTEM_TIMER + 2)
#define NEXT_TIMER_PADDING_3  (NEXT_SYSTEM_TIMER + 3)
#define NEXT_TIMER_CSR        (NEXT_SYSTEM_TIMER + 4)
#define NEXT_EVENT_COUNTER    0x0211a000
#define NEXT_EVENT_LATCH      (NEXT_EVENT_COUNTER + 0)
#define NEXT_EVENT_HIGH       (NEXT_EVENT_COUNTER + 1)
#define NEXT_EVENT_MIDDLE     (NEXT_EVENT_COUNTER + 2)
#define NEXT_EVENT_LOW        (NEXT_EVENT_COUNTER + 3)
#define NEXT_TIMER_ENABLE     0x80
#define NEXT_TIMER_UPDATE     0x40
#define NEXT_SCR2_TIMER_IPL7  0x00008000
#define NEXT_SCR2_SOFTINT0    0x01000000
#define NEXT_SCR2_SOFTINT1    0x02000000
#define NEXT_INTR_SOFTINT0    0x00000001
#define NEXT_INTR_SOFTINT1    0x00000002
#define NEXT_INTR_TIMER       0x20000000
#define NEXT_TIMER_TICK_NS    INT64_C(1000)
#define NEXT_NANOSECONDS_PER_SECOND UINT64_C(1000000000)
#define NEXT_PLAN9_TIMER_FREQUENCY UINT64_C(4456448)
#define NEXT_EVENT_MASK       0x000fffff
#define NEXT_ROM_SIZE         (128 * 1024)
#define NEXT_TEST_TIMEOUT     (10 * G_USEC_PER_SEC)

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

static QTestState *next_cube_timer_start_with_args(const char *args)
{
    TestROM *rom = g_new0(TestROM, 1);
    g_autofree char *quoted_rom_path = NULL;
    QTestState *qts;

    rom->fd = -1;
    qtest_add_abrt_handler(cleanup_test_rom, rom);
    g_test_queue_destroy(cleanup_test_rom, rom);

    rom->fd = g_file_open_tmp("next-cube-timer-rom-XXXXXX",
                              &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;

    quoted_rom_path = g_shell_quote(rom->path);
    qts = qtest_initf("-machine next-cube -bios %s %s",
                      quoted_rom_path, args ?: "");
    return qts;
}

static QTestState *next_cube_timer_start(void)
{
    return next_cube_timer_start_with_args(NULL);
}

static void unrealize_next_kbd(QTestState *qts)
{
    g_autoptr(QDict) response = NULL;
    g_autofree char *path = NULL;
    QList *children;
    QListEntry *entry;

    response = qtest_qmp(
        qts, "{ 'execute': 'qom-list', "
        "'arguments': { 'path': '/machine/unattached' } }");
    g_assert_nonnull(response);
    g_assert_true(qdict_haskey(response, "return"));
    children = qdict_get_qlist(response, "return");
    QLIST_FOREACH_ENTRY(children, entry) {
        QDict *child = qobject_to(QDict, qlist_entry_obj(entry));

        if (!strcmp(qdict_get_str(child, "type"), "child<next-kbd>")) {
            g_assert_null(path);
            path = g_strdup_printf("/machine/unattached/%s",
                                   qdict_get_str(child, "name"));
        }
    }
    g_assert_nonnull(path);

    qtest_qmp_assert_success(
        qts,
        "{ 'execute': 'qom-set', 'arguments': { "
        "'path': %s, 'property': 'realized', 'value': false } }", path);
}

static uint32_t timer_status(QTestState *qts)
{
    return qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_TIMER;
}

static void write_timer_latch(QTestState *qts, uint16_t value)
{
    qtest_writeb(qts, NEXT_TIMER_LOW, 0xff);
    qtest_writeb(qts, NEXT_TIMER_HIGH, value >> 8);
    qtest_writeb(qts, NEXT_TIMER_LOW, value);
}

static uint16_t read_timer_count(QTestState *qts)
{
    uint8_t high;
    uint8_t low;

    do {
        high = qtest_readb(qts, NEXT_TIMER_HIGH);
        low = qtest_readb(qts, NEXT_TIMER_LOW);
    } while (high != qtest_readb(qts, NEXT_TIMER_HIGH));

    return ((uint16_t)high << 8) | low;
}

static uint32_t read_event_snapshot(QTestState *qts)
{
    g_assert_cmphex(qtest_readb(qts, NEXT_EVENT_LATCH), ==, 0);

    return ((uint32_t)qtest_readb(qts, NEXT_EVENT_HIGH) << 16) |
           ((uint32_t)qtest_readb(qts, NEXT_EVENT_MIDDLE) << 8) |
           qtest_readb(qts, NEXT_EVENT_LOW);
}

static uint32_t read_event_data(QTestState *qts)
{
    return ((uint32_t)qtest_readb(qts, NEXT_EVENT_HIGH) << 16) |
           ((uint32_t)qtest_readb(qts, NEXT_EVENT_MIDDLE) << 8) |
           qtest_readb(qts, NEXT_EVENT_LOW);
}

static void arm_timer(QTestState *qts, uint16_t value)
{
    qtest_writeb(qts, NEXT_TIMER_CSR, 0);
    write_timer_latch(qts, value);
    qtest_writeb(qts, NEXT_TIMER_CSR,
                 NEXT_TIMER_ENABLE | NEXT_TIMER_UPDATE);
}

static int64_t timer_ticks_to_ns(uint64_t ticks, uint64_t frequency)
{
    return DIV_ROUND_UP(ticks * NEXT_NANOSECONDS_PER_SECOND, frequency);
}

static void test_frequency_override(void)
{
    QTestState *qts = next_cube_timer_start_with_args(
        "-global next-pc.system-timer-frequency=4456448");
    int64_t deadline = timer_ticks_to_ns(0xffff,
                                         NEXT_PLAN9_TIMER_FREQUENCY);
    uint32_t first;
    uint32_t second;

    arm_timer(qts, 0xffff);
    qtest_clock_step(qts, deadline - 1);
    g_assert_cmphex(timer_status(qts), ==, 0);
    g_assert_cmphex(read_timer_count(qts), ==, 1);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(timer_status(qts), ==, NEXT_INTR_TIMER);
    g_assert_cmphex(qtest_readb(qts, NEXT_TIMER_CSR), ==,
                    NEXT_TIMER_ENABLE);
    g_assert_cmphex(timer_status(qts), ==, 0);

    first = read_event_snapshot(qts);
    qtest_clock_step(qts, 1000 * NEXT_TIMER_TICK_NS);
    second = read_event_snapshot(qts);
    g_assert_cmphex((second - first) & NEXT_EVENT_MASK, ==, 1000);

    qtest_quit(qts);
}

static void assert_invalid_frequency(const char *value)
{
    if (g_test_subprocess()) {
        g_autofree char *args = g_strdup_printf(
            "-global next-pc.system-timer-frequency=%s", value);
        QTestState *qts = next_cube_timer_start_with_args(args);

        qtest_quit(qts);
        return;
    }

    g_test_trap_subprocess(NULL, NEXT_TEST_TIMEOUT, 0);
    g_test_trap_assert_failed();
    g_test_trap_assert_stderr("*system-timer-frequency*");
}

static void test_frequency_rejects_zero(void)
{
    assert_invalid_frequency("0");
}

static void test_frequency_rejects_subnanosecond_tick(void)
{
    assert_invalid_frequency("1000000001");
}

static void test_mapping_and_latch(void)
{
    static const uint16_t values[] = { 0x0000, 0x0001, 0x1234, 0xffff };
    QTestState *qts = next_cube_timer_start();
    g_autofree char *flatview = qtest_hmp(qts, "info mtree -f");
    size_t i;

    g_assert_nonnull(strstr(flatview,
        "0000000002116000-0000000002116004 (prio 0, i/o): "
        "next.system-timer"));
    g_assert_nonnull(strstr(flatview,
        "000000000211a000-000000000211a003 (prio 0, i/o): "
        "next.event-counter"));

    for (i = 0; i < ARRAY_SIZE(values); i++) {
        qtest_writeb(qts, NEXT_TIMER_CSR, 0);
        write_timer_latch(qts, values[i]);
        qtest_writeb(qts, NEXT_TIMER_CSR, NEXT_TIMER_UPDATE);
        g_assert_cmphex(read_timer_count(qts), ==, values[i]);
        g_assert_cmphex(timer_status(qts), ==, 0);
        g_assert_cmphex(qtest_readb(qts, NEXT_TIMER_CSR), ==, 0);

        qtest_writeb(qts, NEXT_TIMER_PADDING_2, 0xa5);
        qtest_writeb(qts, NEXT_TIMER_PADDING_3, 0x5a);
        g_assert_cmphex(qtest_readb(qts, NEXT_TIMER_PADDING_2), ==, 0);
        g_assert_cmphex(qtest_readb(qts, NEXT_TIMER_PADDING_3), ==, 0);
        g_assert_cmphex(read_timer_count(qts), ==, values[i]);
    }

    qtest_quit(qts);
}

static void test_deadline_and_ack(void)
{
    QTestState *qts = next_cube_timer_start();
    uint16_t frozen;

    arm_timer(qts, 1000);
    write_timer_latch(qts, 2000);
    qtest_clock_step(qts, 1000 * NEXT_TIMER_TICK_NS - 1);
    g_assert_cmphex(timer_status(qts), ==, 0);
    g_assert_cmphex(read_timer_count(qts), ==, 1);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(timer_status(qts), ==, NEXT_INTR_TIMER);
    g_assert_cmphex(read_timer_count(qts), ==, 2000);
    g_assert_cmphex(qtest_readb(qts, NEXT_TIMER_CSR), ==,
                    NEXT_TIMER_ENABLE);
    g_assert_cmphex(timer_status(qts), ==, 0);
    qtest_clock_step(qts, 2000 * NEXT_TIMER_TICK_NS - 1);
    g_assert_cmphex(timer_status(qts), ==, 0);
    g_assert_cmphex(read_timer_count(qts), ==, 1);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(timer_status(qts), ==, NEXT_INTR_TIMER);
    g_assert_cmphex(read_timer_count(qts), ==, 2000);
    g_assert_cmphex(qtest_readb(qts, NEXT_TIMER_CSR), ==,
                    NEXT_TIMER_ENABLE);
    g_assert_cmphex(timer_status(qts), ==, 0);

    arm_timer(qts, 1000);
    qtest_clock_step(qts, 500 * NEXT_TIMER_TICK_NS);
    write_timer_latch(qts, 2000);
    qtest_clock_step(qts, 500 * NEXT_TIMER_TICK_NS - 1);
    g_assert_cmphex(timer_status(qts), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(timer_status(qts), ==, NEXT_INTR_TIMER);
    g_assert_cmphex(qtest_readb(qts, NEXT_TIMER_CSR), ==,
                    NEXT_TIMER_ENABLE);
    g_assert_cmphex(timer_status(qts), ==, 0);

    arm_timer(qts, 1000);
    qtest_clock_step(qts, 500 * NEXT_TIMER_TICK_NS);
    qtest_writeb(qts, NEXT_TIMER_CSR, 0);
    frozen = read_timer_count(qts);
    g_assert_cmphex(frozen, ==, 500);
    qtest_clock_step(qts, 1000 * NEXT_TIMER_TICK_NS);
    g_assert_cmphex(timer_status(qts), ==, 0);
    g_assert_cmphex(read_timer_count(qts), ==, frozen);

    qtest_quit(qts);
}

static void test_zero_is_full_period(void)
{
    QTestState *qts = next_cube_timer_start();

    arm_timer(qts, 0);
    qtest_clock_step(qts, INT64_C(65536) * NEXT_TIMER_TICK_NS - 1);
    g_assert_cmphex(timer_status(qts), ==, 0);
    g_assert_cmphex(read_timer_count(qts), ==, 1);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(timer_status(qts), ==, NEXT_INTR_TIMER);
    g_assert_cmphex(read_timer_count(qts), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_TIMER_CSR), ==,
                    NEXT_TIMER_ENABLE);
    g_assert_cmphex(timer_status(qts), ==, 0);

    qtest_quit(qts);
}

static void test_event_counter(void)
{
    QTestState *qts = next_cube_timer_start();
    uint8_t high;
    uint8_t middle;
    uint8_t low;
    uint32_t first;
    uint32_t second;
    unsigned int offset;

    g_assert_cmphex(qtest_readb(qts, NEXT_EVENT_LATCH), ==, 0);
    high = qtest_readb(qts, NEXT_EVENT_HIGH);
    middle = qtest_readb(qts, NEXT_EVENT_MIDDLE);
    low = qtest_readb(qts, NEXT_EVENT_LOW);
    first = ((uint32_t)high << 16) | ((uint32_t)middle << 8) | low;

    g_assert_cmphex(qtest_readb(qts, NEXT_EVENT_HIGH), ==, high);
    qtest_clock_step(qts, 1000 * NEXT_TIMER_TICK_NS);
    g_assert_cmphex(qtest_readb(qts, NEXT_EVENT_HIGH), ==, high);
    g_assert_cmphex(qtest_readb(qts, NEXT_EVENT_MIDDLE), ==, middle);
    g_assert_cmphex(qtest_readb(qts, NEXT_EVENT_LOW), ==, low);

    for (offset = 0; offset < 4; offset++) {
        qtest_writeb(qts, NEXT_EVENT_COUNTER + offset, 0xa5 + offset);
    }
    g_assert_cmphex(qtest_readb(qts, NEXT_EVENT_HIGH), ==, high);
    g_assert_cmphex(qtest_readb(qts, NEXT_EVENT_MIDDLE), ==, middle);
    g_assert_cmphex(qtest_readb(qts, NEXT_EVENT_LOW), ==, low);

    second = read_event_snapshot(qts);
    g_assert_cmphex(high & ~0x0f, ==, 0);
    g_assert_cmphex(second & ~NEXT_EVENT_MASK, ==, 0);
    g_assert_cmphex((second - first) & NEXT_EVENT_MASK, ==, 1000);

    qtest_quit(qts);
}

static void test_reset_cancels_deadline(void)
{
    QTestState *qts = next_cube_timer_start();
    uint32_t event_snapshot;

    arm_timer(qts, 1000);
    qtest_clock_step(qts, 500 * NEXT_TIMER_TICK_NS);
    event_snapshot = read_event_snapshot(qts);
    g_assert_cmphex(event_snapshot, !=, 0);
    qtest_system_reset(qts);

    g_assert_cmphex(read_timer_count(qts), ==, 0);
    g_assert_cmphex(read_event_data(qts), ==, 0);
    g_assert_cmphex(timer_status(qts), ==, 0);
    qtest_writeb(qts, NEXT_TIMER_CSR, NEXT_TIMER_UPDATE);
    g_assert_cmphex(read_timer_count(qts), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_TIMER_CSR), ==, 0);
    qtest_clock_step(qts, 1000 * NEXT_TIMER_TICK_NS);
    g_assert_cmphex(timer_status(qts), ==, 0);
    g_assert_cmphex(read_timer_count(qts), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_TIMER_CSR), ==, 0);

    qtest_quit(qts);
}

static void test_scr2_soft_interrupts(void)
{
    QTestState *qts = next_cube_timer_start();
    uint32_t scr2 = qtest_readl(qts, NEXT_SCR2);
    uint32_t softints = NEXT_INTR_SOFTINT0 | NEXT_INTR_SOFTINT1;

    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & softints, ==, 0);

    qtest_writel(qts, NEXT_SCR2, scr2 | NEXT_SCR2_SOFTINT0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & softints, ==,
                    NEXT_INTR_SOFTINT0);

    qtest_writel(qts, NEXT_SCR2,
                 scr2 | NEXT_SCR2_SOFTINT0 | NEXT_SCR2_SOFTINT1);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & softints, ==,
                    NEXT_INTR_SOFTINT0 | NEXT_INTR_SOFTINT1);

    qtest_writel(qts, NEXT_SCR2, scr2 | NEXT_SCR2_SOFTINT1);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & softints, ==,
                    NEXT_INTR_SOFTINT1);

    qtest_writel(qts, NEXT_SCR2, scr2);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & softints, ==, 0);

    qtest_quit(qts);
}

static void run_migration_pending_irq_and_active_timer(const char *extra_args,
                                                       uint32_t frequency)
{
    const int64_t timer_b_ticks = 1000;
    const int64_t elapsed_b_ticks = 400;
    const int64_t first_tick_ns = timer_ticks_to_ns(1, frequency);
    const int64_t elapsed_b_ns = timer_ticks_to_ns(elapsed_b_ticks,
                                                   frequency);
    const int64_t remaining_b_ns = timer_ticks_to_ns(timer_b_ticks,
                                                     frequency) -
                                   elapsed_b_ns;
    g_autoptr(GError) error = NULL;
    g_autofree char *tmpdir = NULL;
    g_autofree char *socket_path = NULL;
    g_autofree char *uri = NULL;
    g_autofree char *quoted_uri = NULL;
    g_autofree char *incoming_args = NULL;
    QTestState *source;
    QTestState *destination;
    int64_t source_clock;

    tmpdir = g_dir_make_tmp("next-cube-timer-migration-XXXXXX", &error);
    g_assert_no_error(error);
    g_assert_nonnull(tmpdir);
    socket_path = g_build_filename(tmpdir, "migration.sock", NULL);
    uri = g_strdup_printf("unix:%s", socket_path);
    quoted_uri = g_shell_quote(uri);
    incoming_args = g_strdup_printf("-incoming %s %s", quoted_uri,
                                    extra_args ?: "");

    destination = next_cube_timer_start_with_args(incoming_args);
    source = next_cube_timer_start_with_args(extra_args);

    /*
     * The fixed next-kbd device has no VMState yet.  It is unrelated to
     * this test, so unrealize it on both ends before migrating next-pc.
     */
    unrealize_next_kbd(source);
    unrealize_next_kbd(destination);

    arm_timer(source, 1);
    qtest_clock_step(source, first_tick_ns);
    g_assert_cmphex(timer_status(source), ==, NEXT_INTR_TIMER);

    write_timer_latch(source, timer_b_ticks);
    qtest_writeb(source, NEXT_TIMER_CSR,
                 NEXT_TIMER_ENABLE | NEXT_TIMER_UPDATE);
    source_clock =
        qtest_clock_step(source, elapsed_b_ns);
    g_assert_cmphex(timer_status(source), ==, NEXT_INTR_TIMER);

    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    qtest_qmp_eventwait(source, "STOP");
    qtest_qmp_eventwait(destination, "RESUME");

    /*
     * The qtest accelerator's manually advanced clock is not VMState.
     * Match the destination test clock to the migrated timer's timebase.
     */
    qtest_clock_set(destination, source_clock);

    g_assert_cmphex(timer_status(destination), ==, NEXT_INTR_TIMER);
    g_assert_cmphex(qtest_readb(destination, NEXT_TIMER_CSR), ==,
                    NEXT_TIMER_ENABLE);
    g_assert_cmphex(timer_status(destination), ==, 0);

    qtest_clock_step(destination, remaining_b_ns - 1);
    g_assert_cmphex(read_timer_count(destination), ==, 1);
    g_assert_cmphex(timer_status(destination), ==, 0);
    qtest_clock_step(destination, 1);
    g_assert_cmphex(timer_status(destination), ==, NEXT_INTR_TIMER);
    g_assert_cmphex(qtest_readb(destination, NEXT_TIMER_CSR), ==,
                    NEXT_TIMER_ENABLE);
    g_assert_cmphex(timer_status(destination), ==, 0);

    qtest_quit(source);
    qtest_quit(destination);
    g_unlink(socket_path);
    g_assert_cmpint(g_rmdir(tmpdir), ==, 0);
}

static void test_migration_pending_irq_and_active_timer(void)
{
    run_migration_pending_irq_and_active_timer(NULL, 1000000);
}

static void test_migration_frequency_override(void)
{
    run_migration_pending_irq_and_active_timer(
        "-global next-pc.system-timer-frequency=4456448",
        NEXT_PLAN9_TIMER_FREQUENCY);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/next-cube/timer/mapping-and-latch",
                   test_mapping_and_latch);
    qtest_add_func("/next-cube/timer/deadline-and-ack",
                   test_deadline_and_ack);
    qtest_add_func("/next-cube/timer/zero-is-full-period",
                   test_zero_is_full_period);
    qtest_add_func("/next-cube/timer/frequency-override",
                   test_frequency_override);
    qtest_add_func("/next-cube/timer/frequency-rejects-zero",
                   test_frequency_rejects_zero);
    qtest_add_func("/next-cube/timer/frequency-rejects-subnanosecond-tick",
                   test_frequency_rejects_subnanosecond_tick);
    qtest_add_func("/next-cube/timer/event-counter", test_event_counter);
    qtest_add_func("/next-cube/timer/reset-cancels-deadline",
                   test_reset_cancels_deadline);
    qtest_add_func("/next-cube/timer/scr2-soft-interrupts",
                   test_scr2_soft_interrupts);
    qtest_add_func("/next-cube/timer/migration-pending-irq-and-active-timer",
                   test_migration_pending_irq_and_active_timer);
    qtest_add_func("/next-cube/timer/migration-frequency-override",
                   test_migration_frequency_override);
    return g_test_run();
}
