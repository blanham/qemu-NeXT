/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "libqtest.h"

#define NEXT_COLOR_VRAM             0x2c000000
#define NEXT_COLOR_VRAM_SIZE        0x00200000
#define NEXT_COLOR_DAC              0x02118100
#define NEXT_COLOR_COMMAND          0x02118180
#define NEXT_COLOR_DRAM_TIMING      0x02118190
#define NEXT_COLOR_VRAM_TIMING      0x02118198
#define NEXT_INT_STATUS             0x02007000
#define NEXT_COLOR_VIDEO_IRQ_STATUS (1U << 13)
#define NEXT_ROM_SIZE               (128 * KiB)

#define NEXT_COLOR_VRAM_MTREE \
    "000000002c000000-000000002c1fffff (prio 0, ram): next-color-vram"
#define NEXT_COLOR_DAC_MTREE \
    "0000000002118100-0000000002118103 (prio 0, i/o): next-color-dac"
#define NEXT_COLOR_COMMAND_MTREE \
    "0000000002118180-0000000002118180 (prio 0, i/o): next-color-command"
#define NEXT_COLOR_DRAM_TIMING_MTREE \
    "0000000002118190-0000000002118190 (prio 0, i/o): next-color-dram-timing"
#define NEXT_COLOR_VRAM_TIMING_MTREE \
    "0000000002118198-0000000002118198 (prio 0, i/o): next-color-vram-timing"
#define NEXT_MONO_VRAM_PREFIX "000000000b000000-"
#define NEXT_ALT_COLOR_VRAM_PREFIX "0000000006000000-"
#define NEXT_COLOR_REGION_MARKER ": next-color-"

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

static TestROM *create_test_rom(void)
{
    TestROM *rom = g_new0(TestROM, 1);

    rom->fd = -1;
    qtest_add_abrt_handler(cleanup_test_rom, rom);
    g_test_queue_destroy(cleanup_test_rom, rom);

    rom->fd = g_file_open_tmp("next-color-video-rom-XXXXXX",
                              &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;

    return rom;
}

static QTestState *next_color_start(void)
{
    TestROM *rom = create_test_rom();
    g_autofree char *quoted_rom_path = g_shell_quote(rom->path);

    return qtest_initf("-machine next-station-color -m 32M -bios %s",
                       quoted_rom_path);
}

static size_t count_occurrences(const char *text, const char *marker)
{
    size_t count = 0;
    size_t marker_len;

    g_assert_nonnull(text);
    g_assert_nonnull(marker);
    marker_len = strlen(marker);
    g_assert_cmpuint(marker_len, >, 0);

    while ((text = strstr(text, marker))) {
        count++;
        text += marker_len;
    }

    return count;
}

static void test_machine_mapping(void)
{
    QTestState *qts = next_color_start();
    g_autofree char *flatview = qtest_hmp(qts, "info mtree -f");

    g_assert_nonnull(strstr(flatview, NEXT_COLOR_VRAM_MTREE));
    g_assert_nonnull(strstr(flatview, NEXT_COLOR_DAC_MTREE));
    g_assert_nonnull(strstr(flatview, NEXT_COLOR_COMMAND_MTREE));
    g_assert_nonnull(strstr(flatview, NEXT_COLOR_DRAM_TIMING_MTREE));
    g_assert_nonnull(strstr(flatview, NEXT_COLOR_VRAM_TIMING_MTREE));
    g_assert_cmpuint(count_occurrences(flatview, NEXT_COLOR_REGION_MARKER),
                     ==, 5);
    g_assert_null(strstr(flatview, NEXT_MONO_VRAM_PREFIX));
    g_assert_null(strstr(flatview, NEXT_ALT_COLOR_VRAM_PREFIX));

    qtest_quit(qts);
}

static void test_vram_endpoints(void)
{
    QTestState *qts = next_color_start();
    const uint64_t last = NEXT_COLOR_VRAM + NEXT_COLOR_VRAM_SIZE - 1;

    qtest_writeb(qts, NEXT_COLOR_VRAM, 0x5a);
    qtest_writeb(qts, last, 0xa5);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_VRAM), ==, 0x5a);
    g_assert_cmphex(qtest_readb(qts, last), ==, 0xa5);

    qtest_quit(qts);
}

static void test_registers_and_reset(void)
{
    QTestState *qts = next_color_start();

    g_assert_cmphex(qtest_readl(qts, NEXT_COLOR_DAC), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_COMMAND), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DRAM_TIMING), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_VRAM_TIMING), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INT_STATUS) &
                    NEXT_COLOR_VIDEO_IRQ_STATUS, ==, 0);

    qtest_writel(qts, NEXT_COLOR_DAC, 0x12345678);
    qtest_writeb(qts, NEXT_COLOR_COMMAND, 0xff);
    qtest_writeb(qts, NEXT_COLOR_DRAM_TIMING, 0x5a);
    qtest_writeb(qts, NEXT_COLOR_VRAM_TIMING, 0xa5);

    g_assert_cmphex(qtest_readl(qts, NEXT_COLOR_DAC), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_COMMAND), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DRAM_TIMING), ==, 0x5a);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_VRAM_TIMING), ==, 0xa5);
    g_assert_cmphex(qtest_readl(qts, NEXT_INT_STATUS) &
                    NEXT_COLOR_VIDEO_IRQ_STATUS, ==, 0);

    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_COMMAND), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DRAM_TIMING), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_VRAM_TIMING), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INT_STATUS) &
                    NEXT_COLOR_VIDEO_IRQ_STATUS, ==, 0);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/next-color-video/machine-mapping",
                   test_machine_mapping);
    qtest_add_func("/next-color-video/vram-endpoints", test_vram_endpoints);
    qtest_add_func("/next-color-video/registers-and-reset",
                   test_registers_and_reset);

    return g_test_run();
}
