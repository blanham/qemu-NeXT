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
#define NEXT_ROM_SIZE       (128 * KiB)
#define NEXT_RETRACE_NS     (NANOSECONDS_PER_SECOND / 68)

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
