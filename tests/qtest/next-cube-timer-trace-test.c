/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "libqtest.h"

#define NEXT_SCR2             0x0200d000
#define NEXT_SYSTEM_TIMER     0x02116000
#define NEXT_TIMER_HIGH       (NEXT_SYSTEM_TIMER + 0)
#define NEXT_TIMER_LOW        (NEXT_SYSTEM_TIMER + 1)
#define NEXT_TIMER_CSR        (NEXT_SYSTEM_TIMER + 4)
#define NEXT_TIMER_ENABLE     0x80
#define NEXT_TIMER_UPDATE     0x40
#define NEXT_SCR2_TIMER_IPL7  0x00008000
#define NEXT_TIMER_TICK_NS    INT64_C(1000)
#define NEXT_ROM_SIZE         (128 * 1024)

#ifndef _WIN32
#define DEV_NULL "/dev/null"
#else
#define DEV_NULL "nul"
#endif

typedef struct TestFiles {
    int rom_fd;
    int ipl6_log_fd;
    int ipl7_log_fd;
    int reset_log_fd;
    char *rom_path;
    char *ipl6_log_path;
    char *ipl7_log_path;
    char *reset_log_path;
} TestFiles;

static TestFiles test_files = {
    .rom_fd = -1,
    .ipl6_log_fd = -1,
    .ipl7_log_fd = -1,
    .reset_log_fd = -1,
};

static void cleanup_temp_file(int *fd, char **path)
{
    if (*fd >= 0) {
        close(*fd);
        *fd = -1;
    }
    if (*path) {
        g_unlink(*path);
        g_clear_pointer(path, g_free);
    }
}

static void cleanup_test_files(void *opaque)
{
    TestFiles *files = opaque;

    qtest_remove_abrt_handler(files);
    cleanup_temp_file(&files->rom_fd, &files->rom_path);
    cleanup_temp_file(&files->ipl6_log_fd, &files->ipl6_log_path);
    cleanup_temp_file(&files->ipl7_log_fd, &files->ipl7_log_path);
    cleanup_temp_file(&files->reset_log_fd, &files->reset_log_path);
}

static void write_timer_latch(QTestState *qts, uint16_t value)
{
    qtest_writeb(qts, NEXT_TIMER_LOW, 0xff);
    qtest_writeb(qts, NEXT_TIMER_HIGH, value >> 8);
    qtest_writeb(qts, NEXT_TIMER_LOW, value);
}

static void arm_timer(QTestState *qts, uint16_t value)
{
    qtest_writeb(qts, NEXT_TIMER_CSR, 0);
    write_timer_latch(qts, value);
    qtest_writeb(qts, NEXT_TIMER_CSR,
                 NEXT_TIMER_ENABLE | NEXT_TIMER_UPDATE);
}

static void arm_one_microsecond(QTestState *qts)
{
    arm_timer(qts, 1);
    qtest_clock_step(qts, NEXT_TIMER_TICK_NS);
}

static void run_route_case(const char *rom_path, const char *log_path,
                           bool ipl7)
{
#ifdef CONFIG_TRACE_SIMPLE
    const char *simple_trace_file = ",file=" DEV_NULL;
#else
    const char *simple_trace_file = "";
#endif
    g_autofree char *quoted_rom_path = g_shell_quote(rom_path);
    g_autofree char *quoted_log_path = g_shell_quote(log_path);
    g_autofree char *trace_arg =
        g_strdup_printf("next_timer_irq*%s", simple_trace_file);
    g_autofree char *quoted_trace_arg = g_shell_quote(trace_arg);
#ifdef CONFIG_TRACE_SIMPLE
    g_autofree char *default_trace_path = NULL;
#endif
    QTestState *qts;
    uint32_t scr2;

    qts = qtest_initf("-machine next-cube -bios %s "
                      "-trace %s -D %s",
                      quoted_rom_path, quoted_trace_arg, quoted_log_path);
#ifdef CONFIG_TRACE_SIMPLE
    default_trace_path = g_strdup_printf(CONFIG_TRACE_FILE "-" FMT_pid,
                                         qtest_pid(qts));
#endif

    scr2 = qtest_readl(qts, NEXT_SCR2);
    if (ipl7) {
        qtest_writel(qts, NEXT_SCR2, scr2 | NEXT_SCR2_TIMER_IPL7);
    } else {
        qtest_writel(qts, NEXT_SCR2, scr2 & ~NEXT_SCR2_TIMER_IPL7);
    }

    arm_one_microsecond(qts);
    if (ipl7) {
        qtest_writel(qts, NEXT_SCR2,
                     qtest_readl(qts, NEXT_SCR2) &
                     ~NEXT_SCR2_TIMER_IPL7);
        qtest_readb(qts, NEXT_TIMER_CSR);
    }
    qtest_quit(qts);

#ifdef CONFIG_TRACE_SIMPLE
    if (default_trace_path) {
        g_assert_false(g_file_test(default_trace_path, G_FILE_TEST_EXISTS));
    }
#endif
}

static void run_reset_case(const char *rom_path, const char *log_path)
{
#ifdef CONFIG_TRACE_SIMPLE
    const char *simple_trace_file = ",file=" DEV_NULL;
#else
    const char *simple_trace_file = "";
#endif
    g_autofree char *quoted_rom_path = g_shell_quote(rom_path);
    g_autofree char *quoted_log_path = g_shell_quote(log_path);
    g_autofree char *trace_arg =
        g_strdup_printf("next_timer_irq*%s", simple_trace_file);
    g_autofree char *quoted_trace_arg = g_shell_quote(trace_arg);
#ifdef CONFIG_TRACE_SIMPLE
    g_autofree char *default_trace_path = NULL;
#endif
    QTestState *qts;

    qts = qtest_initf("-machine next-cube -bios %s "
                      "-trace %s -D %s",
                      quoted_rom_path, quoted_trace_arg, quoted_log_path);
#ifdef CONFIG_TRACE_SIMPLE
    default_trace_path = g_strdup_printf(CONFIG_TRACE_FILE "-" FMT_pid,
                                         qtest_pid(qts));
#endif

    arm_timer(qts, 1000);
    qtest_clock_step(qts, 500 * NEXT_TIMER_TICK_NS);
    qtest_system_reset(qts);
    qtest_clock_step(qts, 1000 * NEXT_TIMER_TICK_NS);
    qtest_quit(qts);

#ifdef CONFIG_TRACE_SIMPLE
    if (default_trace_path) {
        g_assert_false(g_file_test(default_trace_path, G_FILE_TEST_EXISTS));
    }
#endif
}

static void test_irq_routing_trace(void)
{
    static const char ipl7_assert_pattern[] =
        "next_timer_irq pending=1 level=7 vector=31 "
        "status=0x20000000";
    static const char reroute_pattern[] =
        "next_timer_irq pending=1 level=6 vector=30 "
        "status=0x20000000";
    static const char ack_pattern[] =
        "next_timer_irq pending=0 level=6 vector=30 status=0x0";
    TestFiles *files = &test_files;
    g_autofree char *ipl6_log = NULL;
    g_autofree char *ipl7_log = NULL;
    g_autofree char *reset_log = NULL;
    gsize ipl6_log_len;
    gsize ipl7_log_len;
    gsize reset_log_len;
    const char *ipl7_assert;
    const char *reroute;
    const char *ack;

    qtest_add_abrt_handler(cleanup_test_files, files);
    g_test_queue_destroy(cleanup_test_files, files);

    files->rom_fd = g_file_open_tmp("next-cube-timer-rom-XXXXXX",
                                    &files->rom_path, NULL);
    g_assert_cmpint(files->rom_fd, >=, 0);
    g_assert_cmpint(ftruncate(files->rom_fd, NEXT_ROM_SIZE), ==, 0);
    close(files->rom_fd);
    files->rom_fd = -1;

    files->ipl6_log_fd =
        g_file_open_tmp("next-cube-timer-ipl6-XXXXXX",
                        &files->ipl6_log_path, NULL);
    g_assert_cmpint(files->ipl6_log_fd, >=, 0);
    close(files->ipl6_log_fd);
    files->ipl6_log_fd = -1;

    files->ipl7_log_fd =
        g_file_open_tmp("next-cube-timer-ipl7-XXXXXX",
                        &files->ipl7_log_path, NULL);
    g_assert_cmpint(files->ipl7_log_fd, >=, 0);
    close(files->ipl7_log_fd);
    files->ipl7_log_fd = -1;

    files->reset_log_fd =
        g_file_open_tmp("next-cube-timer-reset-XXXXXX",
                        &files->reset_log_path, NULL);
    g_assert_cmpint(files->reset_log_fd, >=, 0);
    close(files->reset_log_fd);
    files->reset_log_fd = -1;

    run_route_case(files->rom_path, files->ipl6_log_path, false);
    run_route_case(files->rom_path, files->ipl7_log_path, true);
    run_reset_case(files->rom_path, files->reset_log_path);

    g_assert_true(g_file_get_contents(files->ipl6_log_path, &ipl6_log,
                                      &ipl6_log_len, NULL));
    g_assert_true(g_file_get_contents(files->ipl7_log_path, &ipl7_log,
                                      &ipl7_log_len, NULL));
    g_assert_true(g_file_get_contents(files->reset_log_path, &reset_log,
                                      &reset_log_len, NULL));

    g_assert_nonnull(g_strstr_len(
        ipl6_log, ipl6_log_len,
        "next_timer_irq pending=1 level=6 vector=30 "
        "status=0x20000000"));

    ipl7_assert = g_strstr_len(ipl7_log, ipl7_log_len, ipl7_assert_pattern);
    g_assert_nonnull(ipl7_assert);
    reroute = strstr(ipl7_assert + strlen(ipl7_assert_pattern),
                     reroute_pattern);
    g_assert_nonnull(reroute);
    ack = strstr(reroute + strlen(reroute_pattern), ack_pattern);
    g_assert_nonnull(ack);

    g_assert_null(g_strstr_len(reset_log, reset_log_len,
                              "next_timer_irq pending=1"));

    cleanup_test_files(files);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/next-cube/timer/irq-routing-trace",
                   test_irq_routing_trace);
    return g_test_run();
}
