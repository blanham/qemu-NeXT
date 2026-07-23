/* SPDX-License-Identifier: NCSA
 *
 * Copyright (c) 2011-2026 Bryce Lanham
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal with the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to
 * whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimers.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimers in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the names of the University of Illinois/NCSA nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * Software without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS WITH THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define NEXT_DMA_BASE       0x02000000
#define NEXT_INTR_STATUS    0x02007000
#define NEXT_ROM_SIZE       (128 * 1024)
#define NEXT_TEST_RAM_BASE  0x04010000

#define DMA_SETENABLE       0x00010000
#define DMA_SETSUPDATE      0x00020000
#define DMA_READ_CMD        0x00040000
#define DMA_CLRCOMPLETE     0x00080000
#define DMA_RESET           0x00100000
#define DMA_INITBUF         0x00200000
#define DMA_ENABLE          0x01000000
#define DMA_SUPDATE         0x02000000
#define DMA_READ            0x04000000
#define DMA_COMPLETE        0x08000000
#define DMA_BUSEXC          0x10000000

typedef struct TestChannel {
    const char *name;
    uint32_t csr;
    int irq_bit;
    unsigned saved_words;
    bool functional;
} TestChannel;

static const TestChannel channels[] = {
    { "scsi",     0x010, 26, 0, true  },
    { "snd-out",  0x040, 23, 0, false },
    { "optical",  0x050, 25, 0, false },
    { "snd-in",   0x080, 22, 0, false },
    { "printer",  0x090, 24, 0, false },
    { "scc",      0x0c0, 21, 0, false },
    { "dsp",      0x0d0, 20, 0, false },
    { "entx",     0x110, 28, 4, true  },
    { "enrx",     0x150, 27, 2, true  },
    { "video",    0x180, -1, 0, false },
    { "r2m",      0x1c0, 18, 0, false },
    { "m2r",      0x1d0, 19, 0, false },
};

static const uint32_t current_offsets[] = {
    0x4000, 0x4004, 0x4008, 0x400c, 0x4200,
};

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

static QTestState *next_dma_start(void)
{
    TestROM *rom = g_new0(TestROM, 1);
    g_autofree char *quoted_rom_path = NULL;

    rom->fd = -1;
    qtest_add_abrt_handler(cleanup_test_rom, rom);
    g_test_queue_destroy(cleanup_test_rom, rom);

    rom->fd = g_file_open_tmp("next-dma-rom-XXXXXX", &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;

    quoted_rom_path = g_shell_quote(rom->path);
    return qtest_initf("-machine next-cube -bios %s", quoted_rom_path);
}

static uint64_t channel_address(const TestChannel *channel, uint32_t offset)
{
    return NEXT_DMA_BASE + channel->csr + offset;
}

static bool is_current_quad_address(uint64_t address)
{
    size_t channel;
    size_t reg;

    for (channel = 0; channel < ARRAY_SIZE(channels); channel++) {
        for (reg = 0; reg < 4; reg++) {
            if (address == channel_address(&channels[channel],
                                           current_offsets[reg])) {
                return true;
            }
        }
    }
    return false;
}

static void test_all_channel_current_registers(void)
{
    QTestState *qts = next_dma_start();
    size_t channel;
    size_t reg;

    for (channel = 0; channel < ARRAY_SIZE(channels); channel++) {
        for (reg = 0; reg < ARRAY_SIZE(current_offsets); reg++) {
            uint32_t value = 0x10000000 | channel << 12 | reg << 4 | 1;
            uint64_t address = channel_address(&channels[channel],
                                               current_offsets[reg]);

            qtest_writel(qts, address, value);
            g_assert_cmphex(qtest_readl(qts, address), ==, value);
        }

        qtest_writel(qts, channel_address(&channels[channel], 0x4200), 0);
        qtest_writel(qts, NEXT_DMA_BASE + channels[channel].csr,
                     DMA_RESET | DMA_SETENABLE | DMA_READ_CMD);
        g_assert_cmphex(qtest_readl(qts,
                                   channel_address(&channels[channel], 0x4200)),
                        ==, 0);
    }

    qtest_quit(qts);
}

static void test_saved_capabilities(void)
{
    static const uint32_t saved_offsets[] = {
        0x3ff0, 0x3ff4, 0x3ff8, 0x3ffc,
    };
    QTestState *qts = next_dma_start();
    size_t channel;
    size_t reg;

    for (channel = 0; channel < ARRAY_SIZE(channels); channel++) {
        for (reg = 0; reg < ARRAY_SIZE(saved_offsets); reg++) {
            uint64_t address = channel_address(&channels[channel],
                                               saved_offsets[reg]);
            uint32_t value = 0x20000000 | channel << 12 | reg << 4 | 2;

            if (reg < channels[channel].saved_words) {
                qtest_writel(qts, address, value);
                g_assert_cmphex(qtest_readl(qts, address), ==, value);
            } else if (!is_current_quad_address(address)) {
                qtest_writel(qts, address, value);
                g_assert_cmphex(qtest_readl(qts, address), ==, 0);
            }
        }
    }

    qtest_quit(qts);
}

static void test_physical_collisions(void)
{
    static const struct {
        uint32_t address;
        size_t upper_channel;
    } collisions[] = {
        { 0x4040, 2 },
        { 0x4080, 4 },
        { 0x40c0, 6 },
        { 0x41c0, 11 },
    };
    QTestState *qts = next_dma_start();
    size_t collision;
    size_t reg;

    for (collision = 0; collision < ARRAY_SIZE(collisions); collision++) {
        uint32_t values[4];

        for (reg = 0; reg < ARRAY_SIZE(values); reg++) {
            uint64_t address = NEXT_DMA_BASE +
                               collisions[collision].address + reg * 4;

            values[reg] = 0x30000000 | collision << 12 | reg << 4 | 3;
            qtest_writel(qts, address, values[reg]);
            g_assert_cmphex(qtest_readl(qts, address), ==, values[reg]);
        }

        qtest_writel(qts,
                     NEXT_DMA_BASE +
                     channels[collisions[collision].upper_channel].csr,
                     DMA_SETENABLE | DMA_READ_CMD);
        for (reg = 0; reg < ARRAY_SIZE(values); reg++) {
            uint64_t address = NEXT_DMA_BASE +
                               collisions[collision].address + reg * 4;

            g_assert_cmphex(qtest_readl(qts, address), ==, values[reg]);
        }
    }

    qtest_quit(qts);
}

static void test_size_holes(void)
{
    QTestState *qts = next_dma_start();
    size_t channel;

    for (channel = 0; channel < ARRAY_SIZE(channels); channel++) {
        uint64_t address = channel_address(&channels[channel], 0x4204);

        qtest_writel(qts, address, 0xdeadbeef);
        g_assert_cmphex(qtest_readl(qts, address), ==, 0);
    }

    qtest_quit(qts);
}

static void test_access_contract(void)
{
    static const uint8_t big_endian_value[] = { 0x12, 0x34, 0x56, 0x78 };
    static const uint8_t setenable_big_endian[] = { 0x00, 0x01, 0x00, 0x00 };
    static const uint8_t second_big_endian_value[] = {
        0x89, 0xab, 0xcd, 0xef,
    };
    uint8_t bytes[sizeof(big_endian_value)];
    QTestState *qts = next_dma_start();
    uint64_t address = channel_address(&channels[1], 0x4000);
    uint64_t csr_address = NEXT_DMA_BASE + channels[1].csr;

    qtest_memwrite(qts, address, big_endian_value, sizeof(big_endian_value));
    g_assert_cmphex(qtest_readl(qts, address), ==, 0x12345678);

    qtest_writel(qts, address, 0x89abcdef);
    qtest_memread(qts, address, bytes, sizeof(bytes));
    g_assert_cmpmem(bytes, sizeof(bytes),
                    second_big_endian_value, sizeof(second_big_endian_value));

    qtest_writeb(qts, address, 0);
    g_assert_cmphex(qtest_readb(qts, address), ==, 0);
    g_assert_cmphex(qtest_readl(qts, address), ==, 0x89abcdef);

    qtest_writew(qts, address, 0);
    g_assert_cmphex(qtest_readw(qts, address), ==, 0);
    g_assert_cmphex(qtest_readl(qts, address), ==, 0x89abcdef);

    qtest_writel(qts, address + 2, 0);
    g_assert_cmphex(qtest_readl(qts, address + 2), ==, 0);
    g_assert_cmphex(qtest_readl(qts, address), ==, 0x89abcdef);

    qtest_memwrite(qts, csr_address, setenable_big_endian,
                   sizeof(setenable_big_endian));
    g_assert_cmphex(qtest_readl(qts, csr_address), ==, DMA_ENABLE);

    qtest_quit(qts);
}

static void test_inert_channels(void)
{
    QTestState *qts = next_dma_start();
    size_t channel;

    qtest_irq_intercept_out_named(qts, "/machine/next-dma", "sysbus-irq");

    for (channel = 0; channel < ARRAY_SIZE(channels); channel++) {
        uint32_t canary = 0x40000000 | channel << 8 | 4;
        uint32_t ram_address = NEXT_TEST_RAM_BASE + channel * 0x100;
        uint32_t csr;

        if (channels[channel].functional) {
            continue;
        }

        qtest_memwrite(qts, ram_address, &canary, sizeof(canary));
        qtest_writel(qts, channel_address(&channels[channel], 0x4000),
                     ram_address);
        qtest_writel(qts, channel_address(&channels[channel], 0x4004),
                     ram_address + sizeof(canary));
        qtest_writel(qts, channel_address(&channels[channel], 0x4008),
                     ram_address + 0x10);
        qtest_writel(qts, channel_address(&channels[channel], 0x400c),
                     ram_address + 0x20);
        qtest_writel(qts, channel_address(&channels[channel], 0x4200),
                     ram_address);

        qtest_writel(qts, NEXT_DMA_BASE + channels[channel].csr,
                     DMA_RESET | DMA_READ_CMD);
        g_assert_cmphex(qtest_readl(qts,
                                   channel_address(&channels[channel], 0x4000)),
                        ==, ram_address);
        qtest_writel(qts, NEXT_DMA_BASE + channels[channel].csr,
                     DMA_SETENABLE | DMA_READ_CMD);
        qtest_writel(qts, NEXT_DMA_BASE + channels[channel].csr,
                     DMA_SETSUPDATE | DMA_READ_CMD);
        qtest_writel(qts, NEXT_DMA_BASE + channels[channel].csr,
                     DMA_INITBUF | DMA_READ_CMD);

        csr = qtest_readl(qts, NEXT_DMA_BASE + channels[channel].csr);
        g_assert_cmphex(csr & (DMA_ENABLE | DMA_SUPDATE | DMA_READ), ==,
                        DMA_ENABLE | DMA_SUPDATE | DMA_READ);
        g_assert_cmphex(csr & (DMA_COMPLETE | DMA_BUSEXC), ==, 0);
        if (channels[channel].irq_bit >= 0) {
            g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                            (1U << channels[channel].irq_bit), ==, 0);
        }
        g_assert_false(qtest_get_irq(qts, channel));
        g_assert_cmphex(qtest_readl(qts,
                                   channel_address(&channels[channel], 0x4200)),
                        ==, ram_address);
        {
            uint32_t actual;

            qtest_memread(qts, ram_address, &actual, sizeof(actual));
            g_assert_cmpmem(&actual, sizeof(actual),
                            &canary, sizeof(canary));
        }
    }

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/next-cube/dma/all-channel-current-registers",
                   test_all_channel_current_registers);
    qtest_add_func("/next-cube/dma/saved-capabilities",
                   test_saved_capabilities);
    qtest_add_func("/next-cube/dma/physical-collisions",
                   test_physical_collisions);
    qtest_add_func("/next-cube/dma/size-holes",
                   test_size_holes);
    qtest_add_func("/next-cube/dma/exactly-32-bit-big-endian",
                   test_access_contract);
    qtest_add_func("/next-cube/dma/inert-never-completes",
                   test_inert_channels);
    return g_test_run();
}
