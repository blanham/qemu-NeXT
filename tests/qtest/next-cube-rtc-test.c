/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "libqtest.h"

#define NEXT_SCR2          0x0200d000
#define NEXT_SCR2_RTCE     0x00000100
#define NEXT_SCR2_RTCLK    0x00000200
#define NEXT_SCR2_RTDATA   0x00000400
#define NEXT_ROM_SIZE      (128 * 1024)

typedef struct TestROM {
    int fd;
    char *path;
} TestROM;

static const uint8_t initial_nvram[32] = {
    0x94, 0x0f, 0x40, 0x03, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xfb, 0x6d, 0x00, 0x00, 0x4b, 0x00,
    0x41, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x7e,
};

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

static QTestState *next_cube_rtc_start(void)
{
    TestROM *rom = g_new0(TestROM, 1);
    g_autofree char *quoted_rom_path = NULL;
    QTestState *qts;

    rom->fd = -1;
    qtest_add_abrt_handler(cleanup_test_rom, rom);
    g_test_queue_destroy(cleanup_test_rom, rom);

    rom->fd = g_file_open_tmp("next-cube-rtc-rom-XXXXXX",
                              &rom->path, NULL);
    g_assert(rom->fd >= 0);
    g_assert(!ftruncate(rom->fd, NEXT_ROM_SIZE));
    close(rom->fd);
    rom->fd = -1;

    quoted_rom_path = g_shell_quote(rom->path);
    qts = qtest_initf("-machine next-cube -bios %s", quoted_rom_path);
    return qts;
}

static uint32_t rtc_begin(QTestState *qts)
{
    uint32_t scr2 = qtest_readl(qts, NEXT_SCR2);

    scr2 &= ~(NEXT_SCR2_RTCE | NEXT_SCR2_RTCLK | NEXT_SCR2_RTDATA);
    qtest_writel(qts, NEXT_SCR2, scr2);
    scr2 |= NEXT_SCR2_RTCE;
    qtest_writel(qts, NEXT_SCR2, scr2);
    return scr2;
}

static void rtc_end(QTestState *qts, uint32_t scr2)
{
    qtest_writel(qts, NEXT_SCR2,
                 scr2 & ~(NEXT_SCR2_RTCE |
                          NEXT_SCR2_RTCLK |
                          NEXT_SCR2_RTDATA));
}

static void rtc_send_byte(QTestState *qts, uint32_t scr2, uint8_t value)
{
    int bit;

    for (bit = 7; bit >= 0; bit--) {
        uint32_t data = scr2 & ~NEXT_SCR2_RTDATA;

        if (value & (1 << bit)) {
            data |= NEXT_SCR2_RTDATA;
        }
        qtest_writel(qts, NEXT_SCR2, data);
        qtest_writel(qts, NEXT_SCR2, data | NEXT_SCR2_RTCLK);
        qtest_writel(qts, NEXT_SCR2, data);
    }
}

static uint8_t rtc_receive_byte(QTestState *qts, uint32_t scr2)
{
    uint8_t value = 0;
    int bit;

    scr2 &= ~NEXT_SCR2_RTDATA;
    for (bit = 0; bit < 8; bit++) {
        qtest_writel(qts, NEXT_SCR2, scr2 | NEXT_SCR2_RTCLK);
        qtest_writel(qts, NEXT_SCR2, scr2);
        value = (value << 1) |
                !!(qtest_readl(qts, NEXT_SCR2) & NEXT_SCR2_RTDATA);
    }
    return value;
}

static void rtc_block_read(QTestState *qts, uint8_t command,
                           uint8_t *data, size_t len)
{
    uint32_t scr2 = rtc_begin(qts);
    size_t i;

    rtc_send_byte(qts, scr2, command);
    for (i = 0; i < len; i++) {
        data[i] = rtc_receive_byte(qts, scr2);
    }
    rtc_end(qts, scr2);
}

static void rtc_block_write(QTestState *qts, uint8_t command,
                            const uint8_t *data, size_t len)
{
    uint32_t scr2 = rtc_begin(qts);
    size_t i;

    rtc_send_byte(qts, scr2, command);
    for (i = 0; i < len; i++) {
        rtc_send_byte(qts, scr2, data[i]);
    }
    rtc_end(qts, scr2);
}

static void test_nvram_block_transfer(void)
{
    static const uint8_t replacement[32] = {
        0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
        0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
        0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x99, 0x88,
        0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00,
    };
    QTestState *qts = next_cube_rtc_start();
    uint8_t actual[32];

    rtc_block_read(qts, 0x00, actual, sizeof(actual));
    g_assert_cmpmem(actual, sizeof(actual),
                    initial_nvram, sizeof(initial_nvram));

    rtc_block_write(qts, 0x80, replacement, sizeof(replacement));
    rtc_block_read(qts, 0x00, actual, sizeof(actual));
    g_assert_cmpmem(actual, sizeof(actual),
                    replacement, sizeof(replacement));

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/next-cube/rtc/nvram-block-transfer",
                   test_nvram_block_transfer);
    return g_test_run();
}
