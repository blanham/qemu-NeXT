/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "libqtest.h"

#define NEXT_SCSI_CSR1 0x02114020
#define NEXT_SCSI_CSR2 0x02114021
#define NEXT_DMA_CSR   0x02000010
#define NEXT_DMA_NEXT  0x02004010
#define NEXT_ROM_SIZE  (128 * 1024)

#ifndef _WIN32
#define DEV_NULL "/dev/null"
#else
#define DEV_NULL "nul"
#endif

typedef struct AccessResults {
    uint8_t csr1;
    uint8_t csr2;
    uint32_t dma_next;
    uint32_t dma_csr;
} AccessResults;

typedef struct TestFiles {
    int rom_fd;
    int disabled_log_fd;
    int enabled_log_fd;
    char *rom_path;
    char *disabled_log_path;
    char *enabled_log_path;
} TestFiles;

static TestFiles test_files = {
    .rom_fd = -1,
    .disabled_log_fd = -1,
    .enabled_log_fd = -1,
};

static AccessResults run_accesses(const char *rom_path, const char *log_path,
                                  bool enable_tracing)
{
#ifdef CONFIG_TRACE_SIMPLE
    const char *simple_trace_file = ",file=" DEV_NULL;
#else
    const char *simple_trace_file = "";
#endif
    g_autofree char *quoted_rom_path = g_shell_quote(rom_path);
    g_autofree char *quoted_log_path = g_shell_quote(log_path);
    g_autofree char *trace_arg = NULL;
    g_autofree char *quoted_trace_arg = NULL;
#ifdef CONFIG_TRACE_SIMPLE
    g_autofree char *default_trace_path = NULL;
#endif
    QTestState *qts;
    AccessResults results;

    if (enable_tracing) {
        trace_arg = g_strdup_printf("next_scsi_dma_reg_*%s",
                                    simple_trace_file);
        quoted_trace_arg = g_shell_quote(trace_arg);
        qts = qtest_initf("-machine next-cube -bios %s "
                          "-trace 'next_scsi_csr_*' "
                          "-trace %s -D %s",
                          quoted_rom_path, quoted_trace_arg, quoted_log_path);
#ifdef CONFIG_TRACE_SIMPLE
        default_trace_path = g_strdup_printf(CONFIG_TRACE_FILE "-" FMT_pid,
                                             qtest_pid(qts));
#endif
    } else {
        qts = qtest_initf("-machine next-cube -bios %s -D %s",
                          quoted_rom_path, quoted_log_path);
    }

    qtest_writeb(qts, NEXT_SCSI_CSR1, 0x39);
    results.csr1 = qtest_readb(qts, NEXT_SCSI_CSR1);
    g_assert_cmphex(results.csr1, ==, 0x39);

    qtest_writeb(qts, NEXT_SCSI_CSR2, 0x3f);
    results.csr2 = qtest_readb(qts, NEXT_SCSI_CSR2);
    g_assert_cmphex(results.csr2, ==, 0x3f);

    qtest_writel(qts, NEXT_DMA_NEXT, 0x04002000);
    results.dma_next = qtest_readl(qts, NEXT_DMA_NEXT);
    g_assert_cmphex(results.dma_next, ==, 0x04002000);

    results.dma_csr = qtest_readl(qts, NEXT_DMA_CSR);
    qtest_quit(qts);

#ifdef CONFIG_TRACE_SIMPLE
    if (default_trace_path) {
        g_assert_false(g_file_test(default_trace_path, G_FILE_TEST_EXISTS));
    }
#endif

    return results;
}

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
    cleanup_temp_file(&files->disabled_log_fd, &files->disabled_log_path);
    cleanup_temp_file(&files->enabled_log_fd, &files->enabled_log_path);
}

static void test_next_cube_scsi_trace(void)
{
    TestFiles *files = &test_files;
    g_autofree char *disabled_log = NULL;
    g_autofree char *enabled_log = NULL;
    gsize disabled_log_len;
    gsize enabled_log_len;
    AccessResults disabled;
    AccessResults enabled;
    qtest_add_abrt_handler(cleanup_test_files, files);
    g_test_queue_destroy(cleanup_test_files, files);

    files->rom_fd = g_file_open_tmp("next-cube-scsi-rom-XXXXXX",
                                    &files->rom_path, NULL);
    g_assert_cmpint(files->rom_fd, >=, 0);
    g_assert_cmpint(ftruncate(files->rom_fd, NEXT_ROM_SIZE), ==, 0);
    close(files->rom_fd);
    files->rom_fd = -1;

    files->disabled_log_fd =
        g_file_open_tmp("next-cube-scsi-trace-disabled-XXXXXX",
                        &files->disabled_log_path, NULL);
    g_assert_cmpint(files->disabled_log_fd, >=, 0);
    close(files->disabled_log_fd);
    files->disabled_log_fd = -1;

    files->enabled_log_fd =
        g_file_open_tmp("next-cube-scsi-trace-enabled-XXXXXX",
                        &files->enabled_log_path, NULL);
    g_assert_cmpint(files->enabled_log_fd, >=, 0);
    close(files->enabled_log_fd);
    files->enabled_log_fd = -1;

    disabled = run_accesses(files->rom_path, files->disabled_log_path, false);
    enabled = run_accesses(files->rom_path, files->enabled_log_path, true);

    g_assert_cmphex(enabled.csr1, ==, disabled.csr1);
    g_assert_cmphex(enabled.csr2, ==, disabled.csr2);
    g_assert_cmphex(enabled.dma_next, ==, disabled.dma_next);
    g_assert_cmphex(enabled.dma_csr, ==, disabled.dma_csr);

    g_assert_true(g_file_get_contents(files->disabled_log_path, &disabled_log,
                                      &disabled_log_len, NULL));
    g_assert_true(g_file_get_contents(files->enabled_log_path, &enabled_log,
                                      &enabled_log_len, NULL));

    g_assert_null(g_strstr_len(disabled_log, disabled_log_len,
                               "next_scsi_csr_write"));
    g_assert_null(g_strstr_len(disabled_log, disabled_log_len,
                               "next_scsi_csr_read"));
    g_assert_null(g_strstr_len(disabled_log, disabled_log_len,
                               "next_scsi_dma_reg_write"));
    g_assert_null(g_strstr_len(disabled_log, disabled_log_len,
                               "next_scsi_dma_reg_read"));
    g_assert_null(g_strstr_len(disabled_log, disabled_log_len,
                               "next_scsi_dma_transfer"));
    g_assert_null(g_strstr_len(disabled_log, disabled_log_len,
                               "next_scsi_irq"));
    g_assert_null(g_strstr_len(disabled_log, disabled_log_len,
                               "scsi_req_complete"));

    g_assert_nonnull(g_strstr_len(
        enabled_log, enabled_log_len,
        "next_scsi_csr_write addr=0x2114020 old=0x0 value=0x39"));
    g_assert_nonnull(g_strstr_len(
        enabled_log, enabled_log_len,
        "next_scsi_csr_write addr=0x2114021 old=0x0 value=0x3f "
        "enable=0 reset=0 fifofl=0 dmadir=0 cpudma=0 intmask=0"));
    g_assert_nonnull(g_strstr_len(
        enabled_log, enabled_log_len,
        "next_scsi_csr_read addr=0x2114020 value=0x39 repeats=1"));
    g_assert_nonnull(g_strstr_len(
        enabled_log, enabled_log_len,
        "next_scsi_dma_reg_write addr=0x2004010 value=0x4002000 "
        "csr=0x0 next=0x4002000"));
    g_assert_nonnull(g_strstr_len(
        enabled_log, enabled_log_len,
        "next_scsi_dma_reg_read addr=0x2004010 value=0x4002000 "
        "csr=0x0 next=0x4002000"));
    g_assert_nonnull(g_strstr_len(
        enabled_log, enabled_log_len,
        "next_scsi_dma_reg_read addr=0x2000010 value=0x0 csr=0x0"));

    cleanup_test_files(files);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/next-cube/scsi/trace", test_next_cube_scsi_trace);
    return g_test_run();
}
