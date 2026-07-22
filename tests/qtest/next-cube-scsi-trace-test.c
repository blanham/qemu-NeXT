/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "libqtest.h"

#define NEXT_SCSI_CSR1 0x02114020
#define NEXT_SCSI_CSR2 0x02114021
#define NEXT_DMA_CSR   0x02000010
#define NEXT_DMA_NEXT  0x02004010
#define NEXT_ROM_SIZE  (128 * 1024)

typedef struct AccessResults {
    uint8_t csr1;
    uint8_t csr2;
    uint32_t dma_next;
    uint32_t dma_csr;
} AccessResults;

static AccessResults run_accesses(const char *rom_path, const char *log_path,
                                  bool enable_tracing)
{
    g_autofree char *quoted_rom_path = g_shell_quote(rom_path);
    g_autofree char *quoted_log_path = g_shell_quote(log_path);
    QTestState *qts;
    AccessResults results;

    if (enable_tracing) {
        qts = qtest_initf("-machine next-cube -bios %s "
                          "-trace 'next_scsi_csr_*' "
                          "-trace 'next_scsi_dma_reg_*' -D %s",
                          quoted_rom_path, quoted_log_path);
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

    return results;
}

static void unlink_temp_file(void *opaque)
{
    char *path = opaque;

    g_unlink(path);
    g_free(path);
}

static void test_next_cube_scsi_trace(void)
{
    char *rom_path = NULL;
    char *disabled_log_path = NULL;
    char *enabled_log_path = NULL;
    g_autofree char *disabled_log = NULL;
    g_autofree char *enabled_log = NULL;
    gsize disabled_log_len;
    gsize enabled_log_len;
    AccessResults disabled;
    AccessResults enabled;
    int fd;

    fd = g_file_open_tmp("next-cube-scsi-rom-XXXXXX", &rom_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    g_test_queue_destroy(unlink_temp_file, rom_path);
    g_assert_cmpint(ftruncate(fd, NEXT_ROM_SIZE), ==, 0);
    close(fd);

    fd = g_file_open_tmp("next-cube-scsi-trace-disabled-XXXXXX",
                         &disabled_log_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    g_test_queue_destroy(unlink_temp_file, disabled_log_path);
    close(fd);

    fd = g_file_open_tmp("next-cube-scsi-trace-enabled-XXXXXX",
                         &enabled_log_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    g_test_queue_destroy(unlink_temp_file, enabled_log_path);
    close(fd);

    disabled = run_accesses(rom_path, disabled_log_path, false);
    enabled = run_accesses(rom_path, enabled_log_path, true);

    g_assert_cmphex(enabled.csr1, ==, disabled.csr1);
    g_assert_cmphex(enabled.csr2, ==, disabled.csr2);
    g_assert_cmphex(enabled.dma_next, ==, disabled.dma_next);
    g_assert_cmphex(enabled.dma_csr, ==, disabled.dma_csr);

    g_assert_true(g_file_get_contents(disabled_log_path, &disabled_log,
                                      &disabled_log_len, NULL));
    g_assert_true(g_file_get_contents(enabled_log_path, &enabled_log,
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

    g_assert_cmpint(g_unlink(rom_path), ==, 0);
    g_assert_cmpint(g_unlink(disabled_log_path), ==, 0);
    g_assert_cmpint(g_unlink(enabled_log_path), ==, 0);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/next-cube/scsi/trace", test_next_cube_scsi_trace);
    return g_test_run();
}
