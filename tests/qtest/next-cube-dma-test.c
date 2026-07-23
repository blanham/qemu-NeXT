/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "libqtest.h"

#define NEXT_DMA_BASE        0x02000000
#define NEXT_DMA_ENTX_CSR    (NEXT_DMA_BASE + 0x0110)
#define NEXT_DMA_ENRX_CSR    (NEXT_DMA_BASE + 0x0150)
#define NEXT_DMA_SAVED_NEXT  0x3ff0
#define NEXT_DMA_SAVED_LIMIT 0x3ff4
#define NEXT_DMA_SAVED_START 0x3ff8
#define NEXT_DMA_SAVED_STOP  0x3ffc
#define NEXT_DMA_NEXT        0x4000
#define NEXT_DMA_LIMIT       0x4004
#define NEXT_DMA_START       0x4008
#define NEXT_DMA_STOP        0x400c
#define NEXT_DMA_NEXT_INIT   0x4200

#define DMA_SETENABLE        0x00010000
#define DMA_SETSUPDATE       0x00020000
#define DMA_SETREAD          0x00040000
#define DMA_CLRCOMPLETE      0x00080000
#define DMA_RESET            0x00100000
#define DMA_ENABLE           0x01000000
#define DMA_SUPDATE          0x02000000
#define DMA_READ             0x04000000

#define NEXT_ROM_SIZE        (128 * 1024)

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

static QTestState *next_cube_dma_start(void)
{
    TestROM *rom = g_new0(TestROM, 1);
    g_autofree char *quoted_rom_path = NULL;
    QTestState *qts;

    rom->fd = -1;
    qtest_add_abrt_handler(cleanup_test_rom, rom);
    g_test_queue_destroy(cleanup_test_rom, rom);

    rom->fd = g_file_open_tmp("next-cube-dma-rom-XXXXXX",
                              &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;

    quoted_rom_path = g_shell_quote(rom->path);
    qts = qtest_initf("-machine next-cube -bios %s", quoted_rom_path);
    return qts;
}

static void test_dma_register_roundtrip(void)
{
    static const uint32_t channel_bases[] = {
        NEXT_DMA_ENTX_CSR,
        NEXT_DMA_ENRX_CSR,
    };
    static const uint32_t register_offsets[] = {
        NEXT_DMA_SAVED_NEXT,
        NEXT_DMA_SAVED_LIMIT,
        NEXT_DMA_SAVED_START,
        NEXT_DMA_SAVED_STOP,
        NEXT_DMA_NEXT,
        NEXT_DMA_LIMIT,
        NEXT_DMA_START,
        NEXT_DMA_STOP,
        NEXT_DMA_NEXT_INIT,
    };
    QTestState *qts = next_cube_dma_start();
    size_t channel;
    size_t reg;

    for (channel = 0; channel < ARRAY_SIZE(channel_bases); channel++) {
        for (reg = 0; reg < ARRAY_SIZE(register_offsets); reg++) {
            uint32_t value = 0x04001000 + channel * 0x1000 + reg * 0x10;
            uint32_t address = channel_bases[channel] +
                               register_offsets[reg];

            qtest_writel(qts, address, value);
            g_assert_cmphex(qtest_readl(qts, address), ==, value);
        }
    }

    qtest_quit(qts);
}

static void test_dma_csr_commands(void)
{
    static const uint32_t channel_bases[] = {
        NEXT_DMA_ENTX_CSR,
        NEXT_DMA_ENRX_CSR,
    };
    QTestState *qts = next_cube_dma_start();
    size_t channel;

    for (channel = 0; channel < ARRAY_SIZE(channel_bases); channel++) {
        uint32_t csr = channel_bases[channel];

        qtest_writel(qts, csr, DMA_RESET | DMA_SETREAD);
        g_assert_cmphex(qtest_readl(qts, csr), ==, DMA_READ);

        qtest_writel(qts, csr, DMA_SETENABLE | DMA_SETREAD);
        g_assert_cmphex(qtest_readl(qts, csr), ==,
                        DMA_READ | DMA_ENABLE);

        qtest_writel(qts, csr, DMA_SETSUPDATE | DMA_SETREAD);
        g_assert_cmphex(qtest_readl(qts, csr), ==,
                        DMA_READ | DMA_ENABLE | DMA_SUPDATE);

        qtest_writel(qts, csr, DMA_CLRCOMPLETE | DMA_SETREAD);
        g_assert_cmphex(qtest_readl(qts, csr), ==,
                        DMA_READ | DMA_ENABLE | DMA_SUPDATE);

        qtest_writel(qts, csr, DMA_RESET);
        g_assert_cmphex(qtest_readl(qts, csr), ==, 0);
    }

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/next-cube/dma/enet-register-roundtrip",
                   test_dma_register_roundtrip);
    qtest_add_func("/next-cube/dma/enet-csr-commands",
                   test_dma_csr_commands);
    return g_test_run();
}
