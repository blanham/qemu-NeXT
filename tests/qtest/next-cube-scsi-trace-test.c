/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "libqtest.h"

#define NEXT_SCSI_CSR1 0x02114020
#define NEXT_SCSI_CSR2 0x02114021
#define NEXT_DMA_CSR   0x02000010
#define NEXT_DMA_NEXT  0x02004010
#define NEXT_DMA_LIMIT 0x02004014
#define NEXT_DMA_SND_OUT_NEXT 0x02004040
#define NEXT_DMA_SND_IN_NEXT  0x02004080
#define NEXT_DMA_SCC_NEXT     0x020040c0
#define NEXT_DMA_R2M_NEXT     0x020041c0
#define NEXT_ESP_TCLO  0x02114000
#define NEXT_ESP_TCMID 0x02114001
#define NEXT_ESP_FIFO  0x02114002
#define NEXT_ESP_CMD   0x02114003
#define NEXT_ESP_BUSID 0x02114004
#define NEXT_ESP_INTR  0x02114005
#define NEXT_ESP_TCHI  0x0211400e
#define NEXT_ROM_SIZE  (128 * 1024)
#define NEXT_DISK_SIZE (512 * 1024)
#define NEXT_DMA_BUFFER 0x04002000

#define ESP_CMD_SEL 0x41
#define ESP_CMD_TI_DMA 0x90
#define ESP_CMD_ICCS 0x11
#define ESP_CMD_MSGACC 0x12

#define SCSI_CSR_DATA_OUT 0xf0
#define SCSI_CSR_DATA_IN  0xf8

#define DMA_SETENABLE 0x00010000
#define DMA_DEV2M     0x00040000
#define DMA_RESET     0x00100000

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
    int disk_fd;
    int disabled_log_fd;
    int enabled_log_fd;
    int completion_log_fd;
    char *rom_path;
    char *disk_path;
    char *disabled_log_path;
    char *enabled_log_path;
    char *completion_log_path;
} TestFiles;

static TestFiles test_files = {
    .rom_fd = -1,
    .disk_fd = -1,
    .disabled_log_fd = -1,
    .enabled_log_fd = -1,
    .completion_log_fd = -1,
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
        trace_arg = g_strdup_printf("next_*dma_reg_*%s",
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

    qtest_writel(qts, NEXT_DMA_SND_OUT_NEXT, 0x11111111);
    qtest_writel(qts, NEXT_DMA_SND_IN_NEXT, 0x22222222);
    qtest_writel(qts, NEXT_DMA_SCC_NEXT, 0x33333333);
    qtest_writel(qts, NEXT_DMA_R2M_NEXT, 0x44444444);
    qtest_quit(qts);

#ifdef CONFIG_TRACE_SIMPLE
    if (default_trace_path) {
        g_assert_false(g_file_test(default_trace_path, G_FILE_TEST_EXISTS));
    }
#endif

    return results;
}

static uint8_t submit_cdb(QTestState *qts, const uint8_t cdb[6])
{
    uint8_t status;
    int i;

    qtest_writeb(qts, NEXT_ESP_BUSID, 0);
    for (i = 0; i < 6; i++) {
        qtest_writeb(qts, NEXT_ESP_FIFO, cdb[i]);
    }
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_SEL);

    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_ICCS);
    status = qtest_readb(qts, NEXT_ESP_FIFO);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_FIFO), ==, 0);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_MSGACC);
    qtest_readb(qts, NEXT_ESP_INTR);

    return status;
}

static void issue_dma_cdb(QTestState *qts, const uint8_t *cdb,
                          size_t cdb_len, uint32_t transfer_len,
                          uint8_t scsi_csr)
{
    size_t i;

    qtest_writeb(qts, NEXT_ESP_BUSID, 0);
    for (i = 0; i < cdb_len; i++) {
        qtest_writeb(qts, NEXT_ESP_FIFO, cdb[i]);
    }
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_SEL);
    qtest_readb(qts, NEXT_ESP_INTR);

    qtest_writeb(qts, NEXT_ESP_TCLO, transfer_len);
    qtest_writeb(qts, NEXT_ESP_TCMID, transfer_len >> 8);
    qtest_writeb(qts, NEXT_ESP_TCHI, transfer_len >> 16);
    qtest_writeb(qts, NEXT_SCSI_CSR1, scsi_csr);
    g_assert_cmphex(qtest_readb(qts, NEXT_SCSI_CSR1), ==, scsi_csr);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_TI_DMA);
}

static void finish_dma_cdb(QTestState *qts)
{
    qtest_readb(qts, NEXT_ESP_INTR);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_ICCS);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_FIFO), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_FIFO), ==, 0);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_MSGACC);
    qtest_readb(qts, NEXT_ESP_INTR);
}

static void run_completion_commands(const char *rom_path,
                                    const char *disk_path,
                                    const char *log_path)
{
#ifdef CONFIG_TRACE_SIMPLE
    const char *simple_trace_file = ",file=" DEV_NULL;
#else
    const char *simple_trace_file = "";
#endif
    static const uint8_t test_unit_ready[6] = { 0 };
    static const uint8_t invalid_opcode[6] = { 0x1f };
    static const uint8_t inquiry[6] = { 0x12, 0, 0, 0, 64, 0 };
    static const uint8_t write_10[10] = {
        0x2a, 0, 0, 0, 0, 1, 0, 0, 1, 0,
    };
    g_autofree char *quoted_rom_path = g_shell_quote(rom_path);
    g_autofree char *quoted_disk_path = g_shell_quote(disk_path);
    g_autofree char *quoted_log_path = g_shell_quote(log_path);
    g_autofree char *trace_arg =
        g_strdup_printf("scsi_req_complete%s", simple_trace_file);
    g_autofree char *quoted_trace_arg = g_shell_quote(trace_arg);
#ifdef CONFIG_TRACE_SIMPLE
    g_autofree char *default_trace_path = NULL;
#endif
    QTestState *qts;

    qts = qtest_initf("-machine next-cube -bios %s "
                      "-drive file=%s,if=scsi,format=raw "
                      "-trace next_scsi_dma_transfer "
                      "-trace next_scsi_dma_read "
                      "-trace next_scsi_irq -trace %s -D %s",
                      quoted_rom_path, quoted_disk_path, quoted_trace_arg,
                      quoted_log_path);
#ifdef CONFIG_TRACE_SIMPLE
    default_trace_path = g_strdup_printf(CONFIG_TRACE_FILE "-" FMT_pid,
                                         qtest_pid(qts));
#endif

    /* Consume power-on unit attention before the two asserted completions. */
    g_assert_cmphex(submit_cdb(qts, test_unit_ready), ==, 0x02);
    g_assert_cmphex(submit_cdb(qts, invalid_opcode), ==, 0x02);
    g_assert_cmphex(submit_cdb(qts, test_unit_ready), ==, 0x00);

    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_DMA_BUFFER + 64);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);
    issue_dma_cdb(qts, inquiry, sizeof(inquiry), 64, SCSI_CSR_DATA_IN);
    finish_dma_cdb(qts);

    qtest_memset(qts, NEXT_DMA_BUFFER, 0, 512);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_DMA_BUFFER + 512);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE);
    issue_dma_cdb(qts, write_10, sizeof(write_10), 512, SCSI_CSR_DATA_OUT);
    finish_dma_cdb(qts);

    qtest_quit(qts);

#ifdef CONFIG_TRACE_SIMPLE
    g_assert_false(g_file_test(default_trace_path, G_FILE_TEST_EXISTS));
#endif
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
    cleanup_temp_file(&files->disk_fd, &files->disk_path);
    cleanup_temp_file(&files->disabled_log_fd, &files->disabled_log_path);
    cleanup_temp_file(&files->enabled_log_fd, &files->enabled_log_path);
    cleanup_temp_file(&files->completion_log_fd,
                      &files->completion_log_path);
}

static void test_next_cube_scsi_trace(void)
{
    TestFiles *files = &test_files;
    g_autofree char *disabled_log = NULL;
    g_autofree char *enabled_log = NULL;
    g_autofree char *completion_log = NULL;
    gsize disabled_log_len;
    gsize enabled_log_len;
    gsize completion_log_len;
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

    files->disk_fd = g_file_open_tmp("next-cube-scsi-disk-XXXXXX",
                                     &files->disk_path, NULL);
    g_assert_cmpint(files->disk_fd, >=, 0);
    g_assert_cmpint(ftruncate(files->disk_fd, NEXT_DISK_SIZE), ==, 0);
    close(files->disk_fd);
    files->disk_fd = -1;

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

    files->completion_log_fd =
        g_file_open_tmp("next-cube-scsi-trace-completion-XXXXXX",
                        &files->completion_log_path, NULL);
    g_assert_cmpint(files->completion_log_fd, >=, 0);
    close(files->completion_log_fd);
    files->completion_log_fd = -1;

    disabled = run_accesses(files->rom_path, files->disabled_log_path, false);
    enabled = run_accesses(files->rom_path, files->enabled_log_path, true);
    run_completion_commands(files->rom_path, files->disk_path,
                            files->completion_log_path);

    g_assert_cmphex(enabled.csr1, ==, disabled.csr1);
    g_assert_cmphex(enabled.csr2, ==, disabled.csr2);
    g_assert_cmphex(enabled.dma_next, ==, disabled.dma_next);
    g_assert_cmphex(enabled.dma_csr, ==, disabled.dma_csr);

    g_assert_true(g_file_get_contents(files->disabled_log_path, &disabled_log,
                                      &disabled_log_len, NULL));
    g_assert_true(g_file_get_contents(files->enabled_log_path, &enabled_log,
                                      &enabled_log_len, NULL));
    g_assert_true(g_file_get_contents(files->completion_log_path,
                                      &completion_log,
                                      &completion_log_len, NULL));

    g_assert_null(g_strstr_len(disabled_log, disabled_log_len,
                               "next_scsi_csr_write"));
    g_assert_null(g_strstr_len(disabled_log, disabled_log_len,
                               "next_scsi_csr_read"));
    g_assert_null(g_strstr_len(disabled_log, disabled_log_len,
                               "next_scsi_dma_reg_write"));
    g_assert_null(g_strstr_len(disabled_log, disabled_log_len,
                               "next_scsi_dma_reg_read"));
    g_assert_null(g_strstr_len(disabled_log, disabled_log_len,
                               "next_dma_reg_write"));
    g_assert_null(g_strstr_len(disabled_log, disabled_log_len,
                               "next_scsi_dma_transfer"));
    g_assert_null(g_strstr_len(disabled_log, disabled_log_len,
                               "next_scsi_dma_read"));
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
    g_assert_nonnull(g_strstr_len(
        enabled_log, enabled_log_len,
        "next_dma_reg_write addr=0x2004040 owner=snd-out/next "
        "value=0x11111111"));
    g_assert_nonnull(g_strstr_len(
        enabled_log, enabled_log_len,
        "next_dma_reg_write addr=0x2004080 owner=snd-in/next "
        "value=0x22222222"));
    g_assert_nonnull(g_strstr_len(
        enabled_log, enabled_log_len,
        "next_dma_reg_write addr=0x20040c0 owner=scc/next "
        "value=0x33333333"));
    g_assert_nonnull(g_strstr_len(
        enabled_log, enabled_log_len,
        "next_dma_reg_write addr=0x20041c0 owner=r2m/next "
        "value=0x44444444"));

    g_assert_nonnull(g_strstr_len(
        completion_log, completion_log_len,
        "scsi_req_complete target=0 lun=0 tag=0x0 status=0x2 residual=0 "
        "sense_len=18 key=0x05 asc=0x20 ascq=0x00"));
    g_assert_nonnull(g_strstr_len(
        completion_log, completion_log_len,
        "scsi_req_complete target=0 lun=0 tag=0x0 status=0x0 residual=0 "
        "sense_len=0 key=0x00 asc=0x00 ascq=0x00"));
    g_assert_nonnull(g_strstr_len(
        completion_log, completion_log_len,
        "next_scsi_dma_transfer stage=complete direction=device-to-memory "
        "requested=64 aligned=64"));
    g_assert_nonnull(g_strstr_len(
        completion_log, completion_log_len,
        "next_scsi_dma_read stage=complete direction=memory-to-device "
        "requested=512 transferred=512"));
    g_assert_nonnull(g_strstr_len(
        completion_log, completion_log_len,
        "next_scsi_irq source=dma level=1"));

    cleanup_test_files(files);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/next-cube/scsi/trace", test_next_cube_scsi_trace);
    return g_test_run();
}
