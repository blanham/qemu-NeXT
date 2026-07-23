/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "libqtest.h"

#define NEXT_DSP_BASE      0x02108000
#define NEXT_DSP_SIZE      8
#define NEXT_DSP_ICR       (NEXT_DSP_BASE + 0)
#define NEXT_ESP_CMD       0x02114003
#define NEXT_ESP_BUSID     0x02114004
#define NEXT_ESP_STAT      0x02114004
#define NEXT_ESP_SEL       0x02114005
#define NEXT_ESP_INTR      0x02114005
#define NEXT_ESP_SEQ       0x02114006
#define NEXT_ESP_CFG1      0x02114008
#define NEXT_ESP_CCF       0x02114009
#define NEXT_ROM_SIZE      (128 * 1024)

#define ESP_CMD_RESET      0x02
#define ESP_CMD_BUSRESET   0x03
#define ESP_CMD_SELATN     0x42
#define ESP_STAT_INT       0x80
#define ESP_INTR_DC        0x20
#define ESP_CFG1_RESREPT   0x40

#define NEXT_ESP_CLOCK_HZ  20000000
#define NEXT_ESP_SEL_VALUE 0x99
#define NEXT_ESP_CCF_VALUE 4
#define NEXT_ESP_SEL_NS \
    ((int64_t)NEXT_ESP_SEL_VALUE * 8192 * NEXT_ESP_CCF_VALUE * \
     INT64_C(1000000000) / NEXT_ESP_CLOCK_HZ)

G_STATIC_ASSERT(NEXT_ESP_SEL_NS == INT64_C(250675200));

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

static QTestState *next_cube_scsi_start(void)
{
    TestROM *rom = g_new0(TestROM, 1);
    g_autofree char *quoted_rom_path = NULL;
    QTestState *qts;

    rom->fd = -1;
    qtest_add_abrt_handler(cleanup_test_rom, rom);
    g_test_queue_destroy(cleanup_test_rom, rom);

    rom->fd = g_file_open_tmp("next-cube-scsi-rom-XXXXXX",
                              &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;

    quoted_rom_path = g_shell_quote(rom->path);
    qts = qtest_initf("-machine next-cube -bios %s", quoted_rom_path);
    return qts;
}

static void start_missing_target_selection(QTestState *qts, uint8_t target,
                                           uint8_t ccf)
{
    qtest_writeb(qts, NEXT_ESP_CCF, ccf);
    qtest_writeb(qts, NEXT_ESP_SEL, NEXT_ESP_SEL_VALUE);
    qtest_writeb(qts, NEXT_ESP_BUSID, target);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_SELATN);
}

static void assert_esp_quiet(QTestState *qts)
{
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT) & ESP_STAT_INT, ==, 0);
}

static void test_missing_target_selection_timeout(void)
{
    QTestState *qts = next_cube_scsi_start();

    start_missing_target_selection(qts, 1, NEXT_ESP_CCF_VALUE);

    assert_esp_quiet(qts);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_SEQ), ==, 0);

    qtest_clock_step(qts, NEXT_ESP_SEL_NS - 1);
    assert_esp_quiet(qts);

    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT), ==, ESP_STAT_INT);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_SEQ), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_INTR), ==, ESP_INTR_DC);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT) & ESP_STAT_INT, ==, 0);

    qtest_clock_step(qts, NEXT_ESP_SEL_NS);
    assert_esp_quiet(qts);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_INTR), ==, 0);

    qtest_quit(qts);
}

static void test_bus_reset_cancels_selection_timeout(void)
{
    QTestState *qts = next_cube_scsi_start();

    qtest_writeb(qts, NEXT_ESP_CFG1, ESP_CFG1_RESREPT);
    start_missing_target_selection(qts, 1, NEXT_ESP_CCF_VALUE);
    qtest_clock_step(qts, NEXT_ESP_SEL_NS / 2);

    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_BUSRESET);
    assert_esp_quiet(qts);

    qtest_clock_step(qts, NEXT_ESP_SEL_NS);
    assert_esp_quiet(qts);

    qtest_quit(qts);
}

static void test_chip_reset_cancels_selection_timeout(void)
{
    QTestState *qts = next_cube_scsi_start();

    start_missing_target_selection(qts, 1, NEXT_ESP_CCF_VALUE);
    qtest_clock_step(qts, NEXT_ESP_SEL_NS / 2);

    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_RESET);
    assert_esp_quiet(qts);

    qtest_clock_step(qts, NEXT_ESP_SEL_NS);
    assert_esp_quiet(qts);

    qtest_quit(qts);
}

static void test_new_selection_replaces_selection_timeout(void)
{
    QTestState *qts = next_cube_scsi_start();

    start_missing_target_selection(qts, 1, NEXT_ESP_CCF_VALUE);
    qtest_clock_step(qts, NEXT_ESP_SEL_NS / 2);

    start_missing_target_selection(qts, 2, NEXT_ESP_CCF_VALUE);
    qtest_clock_step(qts, NEXT_ESP_SEL_NS / 2);
    assert_esp_quiet(qts);

    qtest_clock_step(qts, NEXT_ESP_SEL_NS / 2 - 1);
    assert_esp_quiet(qts);

    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT), ==, ESP_STAT_INT);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_SEQ), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_INTR), ==, ESP_INTR_DC);
    assert_esp_quiet(qts);

    qtest_clock_step(qts, NEXT_ESP_SEL_NS);
    assert_esp_quiet(qts);

    qtest_quit(qts);
}

static void test_immediate_replacement_cancels_selection_timeout(void)
{
    QTestState *qts = next_cube_scsi_start();

    start_missing_target_selection(qts, 1, NEXT_ESP_CCF_VALUE);
    qtest_clock_step(qts, NEXT_ESP_SEL_NS / 2);

    start_missing_target_selection(qts, 2, 1);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT), ==, ESP_STAT_INT);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_SEQ), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_INTR), ==, ESP_INTR_DC);
    assert_esp_quiet(qts);

    qtest_clock_step(qts, NEXT_ESP_SEL_NS / 2 + 1);
    assert_esp_quiet(qts);

    qtest_quit(qts);
}

static void test_dsp_mmio_mapping(void)
{
    QTestState *qts = next_cube_scsi_start();
    g_autofree char *flatview = qtest_hmp(qts, "info mtree -f");
    unsigned int offset;

    g_assert_nonnull(strstr(flatview,
        "0000000002106000-000000000210601f (prio 0, i/o): next.en"));
    g_assert_nonnull(strstr(flatview,
        "0000000002108000-0000000002108007 (prio 0, i/o): next.dsp"));

    qtest_writeb(qts, NEXT_DSP_ICR, 0x00);
    g_assert_cmphex(qtest_readb(qts, NEXT_DSP_ICR), ==, 0);
    qtest_writeb(qts, NEXT_DSP_ICR, 0xff);
    for (offset = 0; offset < NEXT_DSP_SIZE; offset++) {
        g_assert_cmphex(qtest_readb(qts, NEXT_DSP_BASE + offset), ==, 0);
    }

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/next-cube/scsi/missing-target-selection-timeout",
                   test_missing_target_selection_timeout);
    qtest_add_func("/next-cube/scsi/bus-reset-cancels-selection-timeout",
                   test_bus_reset_cancels_selection_timeout);
    qtest_add_func("/next-cube/scsi/chip-reset-cancels-selection-timeout",
                   test_chip_reset_cancels_selection_timeout);
    qtest_add_func("/next-cube/scsi/new-selection-replaces-selection-timeout",
                   test_new_selection_replaces_selection_timeout);
    qtest_add_func("/next-cube/scsi/immediate-replacement-cancels-timeout",
                   test_immediate_replacement_cancels_selection_timeout);
    qtest_add_func("/next-cube/mmio/dsp-mapping", test_dsp_mmio_mapping);
    return g_test_run();
}
