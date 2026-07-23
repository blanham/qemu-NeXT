/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "libqtest.h"

#define NEXT_ESP_CMD       0x02114003
#define NEXT_ESP_BUSID     0x02114004
#define NEXT_ESP_STAT      0x02114004
#define NEXT_ESP_SEL       0x02114005
#define NEXT_ESP_INTR      0x02114005
#define NEXT_ESP_SEQ       0x02114006
#define NEXT_ESP_CCF       0x02114009
#define NEXT_ROM_SIZE      (128 * 1024)

#define ESP_CMD_SELATN     0x42
#define ESP_STAT_INT       0x80
#define ESP_INTR_DC        0x20

#define NEXT_ESP_CLOCK_HZ  20000000
#define NEXT_ESP_SEL_VALUE 0x99
#define NEXT_ESP_CCF_VALUE 4
#define NEXT_ESP_SEL_NS \
    ((int64_t)NEXT_ESP_SEL_VALUE * 8192 * NEXT_ESP_CCF_VALUE * \
     INT64_C(1000000000) / NEXT_ESP_CLOCK_HZ)

G_STATIC_ASSERT(NEXT_ESP_SEL_NS == INT64_C(250675200));

static void test_missing_target_selection_timeout(void)
{
    g_autofree char *rom_path = NULL;
    g_autofree char *quoted_rom_path = NULL;
    QTestState *qts;
    int rom_fd;

    rom_fd = g_file_open_tmp("next-cube-scsi-rom-XXXXXX",
                             &rom_path, NULL);
    g_assert_cmpint(rom_fd, >=, 0);
    g_assert_cmpint(ftruncate(rom_fd, NEXT_ROM_SIZE), ==, 0);
    close(rom_fd);

    quoted_rom_path = g_shell_quote(rom_path);
    qts = qtest_initf("-machine next-cube -bios %s", quoted_rom_path);

    qtest_writeb(qts, NEXT_ESP_CCF, NEXT_ESP_CCF_VALUE);
    qtest_writeb(qts, NEXT_ESP_SEL, NEXT_ESP_SEL_VALUE);
    qtest_writeb(qts, NEXT_ESP_BUSID, 1);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_SELATN);

    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT) & ESP_STAT_INT, ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_SEQ), ==, 0);

    qtest_clock_step(qts, NEXT_ESP_SEL_NS - 1);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT) & ESP_STAT_INT, ==, 0);

    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT), ==, ESP_STAT_INT);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_SEQ), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_INTR), ==, ESP_INTR_DC);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT) & ESP_STAT_INT, ==, 0);

    qtest_clock_step(qts, NEXT_ESP_SEL_NS);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT) & ESP_STAT_INT, ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_INTR), ==, 0);

    qtest_quit(qts);
    g_assert_cmpint(g_unlink(rom_path), ==, 0);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/next-cube/scsi/missing-target-selection-timeout",
                   test_missing_target_selection_timeout);
    return g_test_run();
}
