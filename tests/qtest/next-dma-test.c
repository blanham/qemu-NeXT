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
#define NEXT_MON_CSR        0x0200e000
#define NEXT_MON_DATA       0x0200e004
#define NEXT_ESP_TCLO       0x02114000
#define NEXT_ESP_TCMID      0x02114001
#define NEXT_ESP_FIFO       0x02114002
#define NEXT_ESP_CMD        0x02114003
#define NEXT_ESP_BUSID      0x02114004
#define NEXT_ESP_INTR       0x02114005
#define NEXT_ESP_TCHI       0x0211400e
#define NEXT_SCSI_CSR       0x02114020
#define NEXT_ROM_SIZE       (128 * 1024)
#define NEXT_DISK_SIZE      (512 * 1024)
#define NEXT_TEST_RAM_BASE  0x04010000
#define NEXT_SCSI_DMA_IRQ   (1U << 26)
#define NEXT_SOUND_DMA_IRQ  (1U << 23)
#define NEXT_VIDEO_IRQ      (1U << 5)
#define NEXT_VIDEO_RETRACE_NS (INT64_C(1000000000) / 68)
#define NEXT_VIDEO_LIMIT    0xea
#define NEXT_SOUND_OUT_CHANNEL 1

#define ESP_CMD_SEL         0x41
#define ESP_CMD_TI_DMA      0x90
#define ESP_CMD_ICCS        0x11
#define ESP_CMD_MSGACC      0x12

#define SCSI_CSR_CPUDMA     0x10
#define SCSI_CSR_INTMASK    0x20
#define SCSI_CSR_FIFOFL     0x04
#define SCSI_CSR_DMADIR     0x08

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

#define MON_SNDOUT_CTRL(options) (0x07 | ((options) << 3))
#define SOUT_ENAB                  0x01

typedef struct TestChannel {
    const char *name;
    uint32_t csr;
    int irq_bit;
    unsigned saved_words;
    bool functional;
} TestChannel;

static const TestChannel channels[] = {
    { "scsi",     0x010, 26, 0, true  },
    { "snd-out",  0x040, 23, 0, true  },
    { "optical",  0x050, 25, 0, false },
    { "snd-in",   0x080, 22, 0, false },
    { "printer",  0x090, 24, 0, false },
    { "scc",      0x0c0, 21, 0, false },
    { "dsp",      0x0d0, 20, 0, false },
    { "entx",     0x110, 28, 4, true  },
    { "enrx",     0x150, 27, 2, true  },
    { "video",    0x180,  5, 0, true  },
    { "r2m",      0x1c0, 18, 0, false },
    { "m2r",      0x1d0, 19, 0, false },
};

/* enum next_irqs input indices, kept in stable NextDMAChannel order. */
static const int dma_board_inputs[] = {
    10, 12, 13, 14, 15, 11, 16, 8, 9, 19, 17, 18,
};

G_STATIC_ASSERT(ARRAY_SIZE(dma_board_inputs) == ARRAY_SIZE(channels));

static const uint32_t current_offsets[] = {
    0x4000, 0x4004, 0x4008, 0x400c, 0x4200,
};

typedef struct TestROM {
    int fd;
    char *path;
} TestROM;

typedef struct TestDisk {
    int fd;
    char *path;
} TestDisk;

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

static void cleanup_test_disk(void *opaque)
{
    TestDisk *disk = opaque;

    qtest_remove_abrt_handler(disk);
    if (disk->fd >= 0) {
        close(disk->fd);
    }
    if (disk->path) {
        g_unlink(disk->path);
        g_free(disk->path);
    }
    g_free(disk);
}

static QTestState *next_dma_start_with_args(bool with_scsi_disk,
                                            const char *extra_args)
{
    TestROM *rom = g_new0(TestROM, 1);
    TestDisk *disk = NULL;
    g_autofree char *quoted_rom_path = NULL;
    g_autofree char *quoted_disk_path = NULL;
    g_autofree char *disk_args = NULL;

    rom->fd = -1;
    qtest_add_abrt_handler(cleanup_test_rom, rom);
    g_test_queue_destroy(cleanup_test_rom, rom);

    rom->fd = g_file_open_tmp("next-dma-rom-XXXXXX", &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;

    quoted_rom_path = g_shell_quote(rom->path);
    if (with_scsi_disk) {
        disk = g_new0(TestDisk, 1);
        disk->fd = -1;
        qtest_add_abrt_handler(cleanup_test_disk, disk);
        g_test_queue_destroy(cleanup_test_disk, disk);

        disk->fd = g_file_open_tmp("next-dma-disk-XXXXXX",
                                   &disk->path, NULL);
        g_assert_cmpint(disk->fd, >=, 0);
        g_assert_cmpint(ftruncate(disk->fd, NEXT_DISK_SIZE), ==, 0);
        close(disk->fd);
        disk->fd = -1;
        quoted_disk_path = g_shell_quote(disk->path);
        disk_args = g_strdup_printf("-drive file=%s,if=scsi,format=raw",
                                    quoted_disk_path);
    }

    return qtest_initf("-machine next-cube -bios %s %s %s",
                       quoted_rom_path, disk_args ?: "", extra_args ?: "");
}

static QTestState *next_dma_start(void)
{
    return next_dma_start_with_args(false, NULL);
}

static uint64_t channel_address(const TestChannel *channel, uint32_t offset)
{
    return NEXT_DMA_BASE + channel->csr + offset;
}

static void issue_inquiry_dma(QTestState *qts, uint8_t length)
{
    uint8_t inquiry[6] = { 0x12, 0, 0, 0, length, 0 };
    size_t i;

    qtest_writeb(qts, NEXT_ESP_BUSID, 0);
    for (i = 0; i < sizeof(inquiry); i++) {
        qtest_writeb(qts, NEXT_ESP_FIFO, inquiry[i]);
    }
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_SEL);
    qtest_readb(qts, NEXT_ESP_INTR);

    qtest_writeb(qts, NEXT_ESP_TCLO, length);
    qtest_writeb(qts, NEXT_ESP_TCMID, 0);
    qtest_writeb(qts, NEXT_ESP_TCHI, 0);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_TI_DMA);
}

static void finish_scsi_command(QTestState *qts)
{
    qtest_readb(qts, NEXT_ESP_INTR);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_ICCS);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_FIFO), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_FIFO), ==, 0);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_MSGACC);
    qtest_readb(qts, NEXT_ESP_INTR);
}

static void pulse_scsi_fifo_flush(QTestState *qts)
{
    qtest_writeb(qts, NEXT_SCSI_CSR,
                 SCSI_CSR_INTMASK | SCSI_CSR_CPUDMA |
                 SCSI_CSR_FIFOFL | SCSI_CSR_DMADIR);
    qtest_writeb(qts, NEXT_SCSI_CSR,
                 SCSI_CSR_INTMASK | SCSI_CSR_CPUDMA | SCSI_CSR_DMADIR);
}

static char *find_unattached_device(QTestState *qts, const char *type)
{
    g_autoptr(QDict) response = NULL;
    g_autofree char *child_type = g_strdup_printf("child<%s>", type);
    g_autofree char *path = NULL;
    QList *children;
    QListEntry *entry;

    response = qtest_qmp(
        qts, "{ 'execute': 'qom-list', "
        "'arguments': { 'path': '/machine/unattached' } }");
    g_assert_nonnull(response);
    g_assert_true(qdict_haskey(response, "return"));
    children = qdict_get_qlist(response, "return");
    QLIST_FOREACH_ENTRY(children, entry) {
        QDict *child = qobject_to(QDict, qlist_entry_obj(entry));

        if (!strcmp(qdict_get_str(child, "type"), child_type)) {
            g_assert_null(path);
            path = g_strdup_printf("/machine/unattached/%s",
                                   qdict_get_str(child, "name"));
        }
    }
    g_assert_nonnull(path);

    return g_steal_pointer(&path);
}

static void unrealize_next_kbd(QTestState *qts)
{
    g_autofree char *path = find_unattached_device(qts, "next-kbd");

    qtest_qmp_assert_success(
        qts,
        "{ 'execute': 'qom-set', 'arguments': { "
        "'path': %s, 'property': 'realized', 'value': false } }", path);
}

static void intercept_next_pc_inputs(QTestState *qts)
{
    g_autofree char *path = find_unattached_device(qts, "next-pc");

    qtest_irq_intercept_in(qts, path);
}

static void migrate_wait(QTestState *source, QTestState *destination,
                         const char *uri)
{
    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    qtest_qmp_eventwait(source, "STOP");
    qtest_qmp_eventwait(destination, "RESUME");
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

static void test_video_retrace_interrupt(void)
{
    QTestState *qts = next_dma_start();
    const TestChannel *video = &channels[9];
    uint64_t csr = NEXT_DMA_BASE + video->csr;
    uint64_t limit = channel_address(video, 0x4004);

    intercept_next_pc_inputs(qts);

    g_assert_false(qtest_get_irq(qts, dma_board_inputs[9]));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_VIDEO_IRQ,
                    ==, 0);

    qtest_writel(qts, limit, NEXT_VIDEO_LIMIT);
    qtest_clock_step(qts, NEXT_VIDEO_RETRACE_NS - 1);
    g_assert_false(qtest_get_irq(qts, dma_board_inputs[9]));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_VIDEO_IRQ,
                    ==, 0);

    qtest_clock_step(qts, 1);
    g_assert_true(qtest_get_irq(qts, dma_board_inputs[9]));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_VIDEO_IRQ,
                    ==, NEXT_VIDEO_IRQ);

    qtest_writel(qts, csr, DMA_RESET);
    g_assert_false(qtest_get_irq(qts, dma_board_inputs[9]));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_VIDEO_IRQ,
                    ==, 0);

    qtest_clock_step(qts, NEXT_VIDEO_RETRACE_NS);
    g_assert_true(qtest_get_irq(qts, dma_board_inputs[9]));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_VIDEO_IRQ,
                    ==, NEXT_VIDEO_IRQ);

    qtest_writel(qts, limit, 0);
    g_assert_true(qtest_get_irq(qts, dma_board_inputs[9]));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_VIDEO_IRQ,
                    ==, NEXT_VIDEO_IRQ);
    qtest_writel(qts, csr, DMA_RESET);
    qtest_clock_step(qts, 2 * NEXT_VIDEO_RETRACE_NS);
    g_assert_false(qtest_get_irq(qts, dma_board_inputs[9]));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_VIDEO_IRQ,
                    ==, 0);

    qtest_quit(qts);
}

static void enable_sound_output(QTestState *qts)
{
    qtest_writeb(qts, NEXT_MON_CSR, 0x80);
    qtest_writeb(qts, NEXT_MON_CSR + 3, MON_SNDOUT_CTRL(SOUT_ENAB));
    qtest_writel(qts, NEXT_MON_DATA, 0);
}

static void test_sound_output_final_segment(void)
{
    static const uint8_t samples[16] = {
        0x10, 0x00, 0xf0, 0x00,
        0x20, 0x00, 0xe0, 0x00,
        0x30, 0x00, 0xd0, 0x00,
        0x40, 0x00, 0xc0, 0x00,
    };
    const TestChannel *sound = &channels[NEXT_SOUND_OUT_CHANNEL];
    QTestState *qts = next_dma_start();
    uint64_t csr = NEXT_DMA_BASE + sound->csr;
    uint64_t next = channel_address(sound, 0x4000);
    uint64_t limit = channel_address(sound, 0x4004);

    intercept_next_pc_inputs(qts);
    qtest_memwrite(qts, NEXT_TEST_RAM_BASE, samples, sizeof(samples));
    qtest_writel(qts, next, NEXT_TEST_RAM_BASE);
    qtest_writel(qts, limit, NEXT_TEST_RAM_BASE + sizeof(samples));
    qtest_writel(qts, csr, DMA_SETENABLE);

    enable_sound_output(qts);

    g_assert_cmphex(qtest_readl(qts, next), ==,
                    NEXT_TEST_RAM_BASE + sizeof(samples));
    g_assert_cmphex(qtest_readl(qts, csr) &
                    (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_COMPLETE);
    g_assert_true(qtest_get_irq(qts, dma_board_inputs[NEXT_SOUND_OUT_CHANNEL]));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_SOUND_DMA_IRQ,
                    ==, NEXT_SOUND_DMA_IRQ);

    qtest_writel(qts, csr, DMA_CLRCOMPLETE);
    g_assert_false(qtest_get_irq(qts,
                                 dma_board_inputs[NEXT_SOUND_OUT_CHANNEL]));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_SOUND_DMA_IRQ,
                    ==, 0);

    qtest_quit(qts);
}

static void test_sound_output_chained_segments(void)
{
    static const uint8_t samples[32] = {
        0x10, 0x00, 0xf0, 0x00, 0x20, 0x00, 0xe0, 0x00,
        0x30, 0x00, 0xd0, 0x00, 0x40, 0x00, 0xc0, 0x00,
        0x50, 0x00, 0xb0, 0x00, 0x60, 0x00, 0xa0, 0x00,
        0x70, 0x00, 0x90, 0x00, 0x7f, 0xff, 0x80, 0x00,
    };
    const TestChannel *sound = &channels[NEXT_SOUND_OUT_CHANNEL];
    QTestState *qts = next_dma_start();
    uint64_t csr = NEXT_DMA_BASE + sound->csr;
    uint64_t next = channel_address(sound, 0x4000);
    uint64_t limit = channel_address(sound, 0x4004);
    uint64_t start = channel_address(sound, 0x4008);
    uint64_t stop = channel_address(sound, 0x400c);

    qtest_memwrite(qts, NEXT_TEST_RAM_BASE, samples, sizeof(samples));
    qtest_writel(qts, next, NEXT_TEST_RAM_BASE);
    qtest_writel(qts, limit, NEXT_TEST_RAM_BASE + 16);
    qtest_writel(qts, start, NEXT_TEST_RAM_BASE + 16);
    qtest_writel(qts, stop, NEXT_TEST_RAM_BASE + sizeof(samples));
    qtest_writel(qts, csr, DMA_SETENABLE | DMA_SETSUPDATE);

    enable_sound_output(qts);

    g_assert_cmphex(qtest_readl(qts, next), ==, NEXT_TEST_RAM_BASE + 16);
    g_assert_cmphex(qtest_readl(qts, limit), ==,
                    NEXT_TEST_RAM_BASE + sizeof(samples));
    g_assert_cmphex(qtest_readl(qts, csr) &
                    (DMA_ENABLE | DMA_SUPDATE | DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_ENABLE | DMA_COMPLETE);

    qtest_writel(qts, csr, DMA_CLRCOMPLETE);
    qtest_writeb(qts, NEXT_MON_CSR + 3, MON_SNDOUT_CTRL(SOUT_ENAB));
    qtest_writel(qts, NEXT_MON_DATA, 0);

    g_assert_cmphex(qtest_readl(qts, next), ==,
                    NEXT_TEST_RAM_BASE + sizeof(samples));
    g_assert_cmphex(qtest_readl(qts, csr) &
                    (DMA_ENABLE | DMA_SUPDATE | DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_COMPLETE);

    qtest_quit(qts);
}

static void test_sound_output_range_error_and_reset(void)
{
    const TestChannel *sound = &channels[NEXT_SOUND_OUT_CHANNEL];
    QTestState *qts = next_dma_start();
    uint64_t csr = NEXT_DMA_BASE + sound->csr;

    intercept_next_pc_inputs(qts);
    qtest_writel(qts, channel_address(sound, 0x4000),
                 NEXT_TEST_RAM_BASE + 16);
    qtest_writel(qts, channel_address(sound, 0x4004), NEXT_TEST_RAM_BASE);
    qtest_writel(qts, csr, DMA_SETENABLE);

    enable_sound_output(qts);

    g_assert_cmphex(qtest_readl(qts, csr) &
                    (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_COMPLETE | DMA_BUSEXC);
    g_assert_true(qtest_get_irq(qts, dma_board_inputs[NEXT_SOUND_OUT_CHANNEL]));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_SOUND_DMA_IRQ,
                    ==, NEXT_SOUND_DMA_IRQ);

    qtest_writel(qts, csr, DMA_RESET);
    g_assert_cmphex(qtest_readl(qts, csr), ==, 0);
    g_assert_false(qtest_get_irq(qts,
                                 dma_board_inputs[NEXT_SOUND_OUT_CHANNEL]));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_SOUND_DMA_IRQ,
                    ==, 0);

    qtest_quit(qts);
}

static void test_zero_next_init_valid(void)
{
    enum {
        TRANSFER_LENGTH = 16,
    };
    uint8_t received[TRANSFER_LENGTH];
    QTestState *qts = next_dma_start_with_args(true, NULL);
    uint64_t csr = NEXT_DMA_BASE + channels[0].csr;
    uint64_t next = channel_address(&channels[0], 0x4000);
    uint64_t limit = channel_address(&channels[0], 0x4004);
    uint64_t next_init = channel_address(&channels[0], 0x4200);

    qtest_memset(qts, NEXT_TEST_RAM_BASE, 0xa5, sizeof(received));
    qtest_writel(qts, csr, DMA_RESET | DMA_READ_CMD);
    qtest_writel(qts, next, NEXT_TEST_RAM_BASE);
    qtest_writel(qts, limit, NEXT_TEST_RAM_BASE + TRANSFER_LENGTH);
    qtest_writel(qts, next_init, 0);
    g_assert_cmphex(qtest_readl(qts, next_init), ==, 0);
    qtest_writel(qts, csr, DMA_SETENABLE | DMA_READ_CMD);

    issue_inquiry_dma(qts, TRANSFER_LENGTH);
    finish_scsi_command(qts);

    g_assert_cmphex(qtest_readl(qts, next), ==, 0);
    qtest_memread(qts, NEXT_TEST_RAM_BASE, received, sizeof(received));
    for (size_t i = 0; i < sizeof(received); i++) {
        g_assert_cmphex(received[i], ==, 0xa5);
    }

    /*
     * A zero initial pointer is a value, not "no latch".  The first
     * transfer consumed it, so this second transfer uses live NEXT.
     */
    qtest_writel(qts, next, NEXT_TEST_RAM_BASE);
    qtest_writel(qts, limit, NEXT_TEST_RAM_BASE + TRANSFER_LENGTH);
    qtest_writel(qts, csr, DMA_SETENABLE | DMA_READ_CMD);
    issue_inquiry_dma(qts, TRANSFER_LENGTH);
    finish_scsi_command(qts);

    g_assert_cmphex(qtest_readl(qts, next), ==,
                    NEXT_TEST_RAM_BASE + TRANSFER_LENGTH);
    qtest_memread(qts, NEXT_TEST_RAM_BASE, received, sizeof(received));
    g_assert_cmpmem(&received[8], 4, "QEMU", 4);

    qtest_quit(qts);
}

static uint32_t test_csr_command(size_t channel)
{
    unsigned pattern = channel % 7 + 1;
    uint32_t command = 0;

    if (pattern & 1) {
        command |= DMA_SETENABLE;
    }
    if (pattern & 2) {
        command |= DMA_SETSUPDATE;
    }
    if (pattern & 4) {
        command |= DMA_READ_CMD;
    }
    return command;
}

static uint32_t test_csr_status(size_t channel)
{
    unsigned pattern = channel % 7 + 1;
    uint32_t status = 0;

    if (pattern & 1) {
        status |= DMA_ENABLE;
    }
    if (pattern & 2) {
        status |= DMA_SUPDATE;
    }
    if (pattern & 4) {
        status |= DMA_READ;
    }
    return status;
}

static void assert_all_dma_irqs_low(QTestState *qts)
{
    size_t channel;

    for (channel = 0; channel < ARRAY_SIZE(channels); channel++) {
        g_assert_false(qtest_get_irq(qts, channel));
    }
}

static void assert_all_channel_registers_zero(QTestState *qts)
{
    static const uint32_t saved_offsets[] = {
        0x3ff0, 0x3ff4, 0x3ff8, 0x3ffc,
    };
    size_t channel;
    size_t reg;

    for (channel = 0; channel < ARRAY_SIZE(channels); channel++) {
        g_assert_cmphex(qtest_readl(qts, NEXT_DMA_BASE +
                                    channels[channel].csr), ==, 0);
        for (reg = 0; reg < ARRAY_SIZE(current_offsets); reg++) {
            g_assert_cmphex(qtest_readl(
                                qts, channel_address(&channels[channel],
                                                     current_offsets[reg])),
                            ==, 0);
        }
        for (reg = 0; reg < channels[channel].saved_words; reg++) {
            g_assert_cmphex(qtest_readl(
                                qts, channel_address(&channels[channel],
                                                     saved_offsets[reg])),
                            ==, 0);
        }
    }
}

static void test_device_reset_all_channels(void)
{
    static const uint32_t saved_offsets[] = {
        0x3ff0, 0x3ff4, 0x3ff8, 0x3ffc,
    };
    enum {
        INQUIRY_LENGTH = 66,
        DMA_WINDOW_LENGTH = 96,
        DMA_COMMITTED_LENGTH = 64,
        RESET_BUFFER = NEXT_TEST_RAM_BASE + 0x4000,
    };
    uint8_t received[16];
    QTestState *qts = next_dma_start_with_args(true, NULL);
    size_t channel;
    size_t reg;

    qtest_irq_intercept_out_named(qts, "/machine/next-dma", "sysbus-irq");

    /* Leave two real inquiry bytes in the controller's delayed stage. */
    qtest_writel(qts, NEXT_DMA_BASE + channels[0].csr,
                 DMA_RESET | DMA_READ_CMD);
    qtest_writel(qts, channel_address(&channels[0], 0x4000),
                 NEXT_TEST_RAM_BASE);
    qtest_writel(qts, channel_address(&channels[0], 0x4004),
                 NEXT_TEST_RAM_BASE + DMA_WINDOW_LENGTH);
    qtest_writel(qts, NEXT_DMA_BASE + channels[0].csr,
                 DMA_SETENABLE | DMA_READ_CMD);
    issue_inquiry_dma(qts, INQUIRY_LENGTH);
    g_assert_cmphex(qtest_readl(
                        qts, channel_address(&channels[0], 0x4000)),
                    ==, NEXT_TEST_RAM_BASE + DMA_COMMITTED_LENGTH);

    for (channel = 0; channel < ARRAY_SIZE(channels); channel++) {
        qtest_writel(qts, NEXT_DMA_BASE + channels[channel].csr,
                     test_csr_command(channel));
        for (reg = 0; reg < ARRAY_SIZE(current_offsets); reg++) {
            qtest_writel(qts,
                         channel_address(&channels[channel],
                                         current_offsets[reg]),
                         0x11000000 | channel << 12 | reg << 4 | 1);
        }
        for (reg = 0; reg < channels[channel].saved_words; reg++) {
            qtest_writel(qts,
                         channel_address(&channels[channel],
                                         saved_offsets[reg]),
                         0x22000000 | channel << 12 | reg << 4 | 2);
        }
    }

    qtest_system_reset(qts);
    assert_all_channel_registers_zero(qts);
    assert_all_dma_irqs_low(qts);

    /*
     * Four flush edges after reset must not resurrect the staged tail or
     * write any of its bytes.
     */
    qtest_memset(qts, RESET_BUFFER, 0xa5, sizeof(received));
    qtest_writel(qts, channel_address(&channels[0], 0x4000), RESET_BUFFER);
    qtest_writel(qts, channel_address(&channels[0], 0x4004),
                 RESET_BUFFER + sizeof(received));
    for (reg = 0; reg < 4; reg++) {
        pulse_scsi_fifo_flush(qts);
    }
    g_assert_cmphex(qtest_readl(
                        qts, channel_address(&channels[0], 0x4000)),
                    ==, RESET_BUFFER);
    qtest_memread(qts, RESET_BUFFER, received, sizeof(received));
    for (reg = 0; reg < sizeof(received); reg++) {
        g_assert_cmphex(received[reg], ==, 0xa5);
    }

    /*
     * QOM reset also clears the hidden NEXT_INIT-valid latch.  With no
     * post-reset NEXT_INIT write, functional SCSI must use live NEXT.
     */
    qtest_writel(qts, NEXT_DMA_BASE + channels[0].csr,
                 DMA_SETENABLE | DMA_READ_CMD);
    issue_inquiry_dma(qts, sizeof(received));
    finish_scsi_command(qts);
    g_assert_cmphex(qtest_readl(
                        qts, channel_address(&channels[0], 0x4000)),
                    ==, RESET_BUFFER + sizeof(received));
    g_assert_true(qtest_get_irq(qts, 0));

    qtest_system_reset(qts);
    assert_all_channel_registers_zero(qts);
    assert_all_dma_irqs_low(qts);

    qtest_quit(qts);
}

static void test_functional_irq_invariant(void)
{
    enum {
        TRANSFER_LENGTH = 16,
    };
    QTestState *qts = next_dma_start_with_args(true, NULL);
    size_t functional[] = { 0, 7, 8 };
    size_t i;

    /*
     * ENTX/ENRX transfer entry points are intentionally not implemented
     * yet.  Lock the strongest observable invariant: every supported MMIO
     * command leaves COMPLETE and its output low.
     */
    for (i = 0; i < ARRAY_SIZE(functional); i++) {
        size_t channel = functional[i];
        uint64_t csr = NEXT_DMA_BASE + channels[channel].csr;

        qtest_writel(qts, csr, DMA_RESET | DMA_READ_CMD);
        qtest_writel(qts, channel_address(&channels[channel], 0x4000),
                     NEXT_TEST_RAM_BASE + channel * 0x100);
        qtest_writel(qts, channel_address(&channels[channel], 0x4004),
                     NEXT_TEST_RAM_BASE + channel * 0x100 + 0x40);
        qtest_writel(qts, csr, DMA_SETENABLE | DMA_SETSUPDATE |
                     DMA_INITBUF | DMA_READ_CMD);
        g_assert_cmphex(qtest_readl(qts, csr) & DMA_COMPLETE, ==, 0);
        g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                        (1U << channels[channel].irq_bit), ==, 0);
    }

    qtest_writel(qts, NEXT_DMA_BASE + channels[0].csr,
                 DMA_RESET | DMA_READ_CMD);
    qtest_writel(qts, channel_address(&channels[0], 0x4000),
                 NEXT_TEST_RAM_BASE);
    qtest_writel(qts, channel_address(&channels[0], 0x4004),
                 NEXT_TEST_RAM_BASE + TRANSFER_LENGTH);
    qtest_writel(qts, NEXT_DMA_BASE + channels[0].csr,
                 DMA_SETENABLE | DMA_READ_CMD);
    issue_inquiry_dma(qts, TRANSFER_LENGTH);
    finish_scsi_command(qts);

    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_BASE + channels[0].csr) &
                    DMA_COMPLETE, ==, DMA_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    NEXT_SCSI_DMA_IRQ, ==, NEXT_SCSI_DMA_IRQ);

    qtest_writel(qts, NEXT_DMA_BASE + channels[0].csr,
                 DMA_CLRCOMPLETE | DMA_READ_CMD);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_BASE + channels[0].csr) &
                    DMA_COMPLETE, ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    NEXT_SCSI_DMA_IRQ, ==, 0);

    qtest_quit(qts);
}

static void test_cross_channel_ack_isolation(void)
{
    enum {
        TRANSFER_LENGTH = 16,
    };
    QTestState *qts = next_dma_start_with_args(true, NULL);
    size_t other[] = { 7, 8 };
    size_t i;

    qtest_writel(qts, NEXT_DMA_BASE + channels[0].csr,
                 DMA_RESET | DMA_READ_CMD);
    qtest_writel(qts, channel_address(&channels[0], 0x4000),
                 NEXT_TEST_RAM_BASE);
    qtest_writel(qts, channel_address(&channels[0], 0x4004),
                 NEXT_TEST_RAM_BASE + TRANSFER_LENGTH);
    qtest_writel(qts, NEXT_DMA_BASE + channels[0].csr,
                 DMA_SETENABLE | DMA_READ_CMD);
    issue_inquiry_dma(qts, TRANSFER_LENGTH);
    finish_scsi_command(qts);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    NEXT_SCSI_DMA_IRQ, ==, NEXT_SCSI_DMA_IRQ);

    for (i = 0; i < ARRAY_SIZE(other); i++) {
        size_t channel = other[i];

        qtest_writel(qts, NEXT_DMA_BASE + channels[channel].csr,
                     DMA_CLRCOMPLETE);
        g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                        NEXT_SCSI_DMA_IRQ, ==, NEXT_SCSI_DMA_IRQ);
        qtest_writel(qts, NEXT_DMA_BASE + channels[channel].csr, DMA_RESET);
        g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                        NEXT_SCSI_DMA_IRQ, ==, NEXT_SCSI_DMA_IRQ);
    }

    qtest_writel(qts, NEXT_DMA_BASE + channels[0].csr, DMA_CLRCOMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    NEXT_SCSI_DMA_IRQ, ==, 0);

    qtest_quit(qts);
}

static void test_migration_idle_all_channels(void)
{
    static const uint32_t saved_offsets[] = {
        0x3ff0, 0x3ff4, 0x3ff8, 0x3ffc,
    };
    enum {
        TRANSFER_LENGTH = 16,
        COMPLETE_BUFFER = NEXT_TEST_RAM_BASE + 0x8000,
    };
    g_autoptr(GError) error = NULL;
    g_autofree char *tmpdir = NULL;
    g_autofree char *socket_path = NULL;
    g_autofree char *uri = NULL;
    g_autofree char *quoted_uri = NULL;
    g_autofree char *incoming_args = NULL;
    QTestState *source;
    QTestState *destination;
    size_t channel;
    size_t reg;

    tmpdir = g_dir_make_tmp("next-dma-migration-XXXXXX", &error);
    g_assert_no_error(error);
    g_assert_nonnull(tmpdir);
    socket_path = g_build_filename(tmpdir, "migration.sock", NULL);
    uri = g_strdup_printf("unix:%s", socket_path);
    quoted_uri = g_shell_quote(uri);
    incoming_args = g_strdup_printf("-incoming %s", quoted_uri);

    destination = next_dma_start_with_args(true, incoming_args);
    source = next_dma_start_with_args(true, NULL);
    unrealize_next_kbd(source);
    unrealize_next_kbd(destination);
    intercept_next_pc_inputs(destination);

    /* Create one real, idle COMPLETE state for IRQ reconstruction. */
    qtest_writel(source, NEXT_DMA_BASE + channels[0].csr,
                 DMA_RESET | DMA_READ_CMD);
    qtest_writel(source, channel_address(&channels[0], 0x4000),
                 COMPLETE_BUFFER);
    qtest_writel(source, channel_address(&channels[0], 0x4004),
                 COMPLETE_BUFFER + TRANSFER_LENGTH);
    qtest_writel(source, NEXT_DMA_BASE + channels[0].csr,
                 DMA_SETENABLE | DMA_READ_CMD);
    issue_inquiry_dma(source, TRANSFER_LENGTH);
    finish_scsi_command(source);
    g_assert_cmphex(qtest_readl(source,
                               NEXT_DMA_BASE + channels[0].csr) &
                    DMA_COMPLETE, ==, DMA_COMPLETE);

    for (channel = 0; channel < ARRAY_SIZE(channels); channel++) {
        qtest_writel(source, NEXT_DMA_BASE + channels[channel].csr,
                     test_csr_command(channel));
        for (reg = 0; reg < ARRAY_SIZE(current_offsets); reg++) {
            uint32_t value = 0x04100000 | channel << 12 | reg << 8 | 0x10;

            qtest_writel(source,
                         channel_address(&channels[channel],
                                         current_offsets[reg]),
                         value);
        }
        for (reg = 0; reg < channels[channel].saved_words; reg++) {
            uint32_t value = 0x05100000 | channel << 12 | reg << 8 | 0x20;

            qtest_writel(source,
                         channel_address(&channels[channel],
                                         saved_offsets[reg]),
                         value);
        }
    }
    g_assert_cmphex(qtest_readl(source, NEXT_INTR_STATUS) &
                    NEXT_SCSI_DMA_IRQ, ==, NEXT_SCSI_DMA_IRQ);

    /*
     * The board aggregator is derived state.  Leave the DMA output and
     * COMPLETE asserted, but migrate a stale cleared board status to prove
     * next-dma post-load reconstruction happens after next-pc loads it.
     */
    qtest_writel(source, NEXT_INTR_STATUS, 0);
    g_assert_cmphex(qtest_readl(source,
                               NEXT_DMA_BASE + channels[0].csr) &
                    DMA_COMPLETE, ==, DMA_COMPLETE);
    g_assert_cmphex(qtest_readl(source, NEXT_INTR_STATUS) &
                    NEXT_SCSI_DMA_IRQ, ==, 0);

    migrate_wait(source, destination, uri);

    for (channel = 0; channel < ARRAY_SIZE(channels); channel++) {
        uint32_t expected_csr = test_csr_status(channel);

        if (channel == 0) {
            expected_csr |= DMA_COMPLETE;
        }
        g_assert_cmphex(qtest_readl(destination,
                                   NEXT_DMA_BASE + channels[channel].csr),
                        ==, expected_csr);
        for (reg = 0; reg < ARRAY_SIZE(current_offsets); reg++) {
            uint32_t expected =
                0x04100000 | channel << 12 | reg << 8 | 0x10;

            g_assert_cmphex(qtest_readl(
                                destination,
                                channel_address(&channels[channel],
                                                current_offsets[reg])),
                            ==, expected);
        }
        for (reg = 0; reg < channels[channel].saved_words; reg++) {
            uint32_t expected =
                0x05100000 | channel << 12 | reg << 8 | 0x20;

            g_assert_cmphex(qtest_readl(
                                destination,
                                channel_address(&channels[channel],
                                                saved_offsets[reg])),
                            ==, expected);
        }
        if (dma_board_inputs[channel] >= 0) {
            g_assert_cmpint(qtest_get_irq(destination,
                                         dma_board_inputs[channel]), ==,
                            channel == 0);
        }
    }
    g_assert_cmphex(qtest_readl(destination, NEXT_INTR_STATUS) &
                    NEXT_SCSI_DMA_IRQ, ==, NEXT_SCSI_DMA_IRQ);

    /*
     * The migrated SCSI NEXT_INIT register is nonzero and its hidden valid
     * latch must survive independently of the COMPLETE acknowledgement.
     */
    {
        uint32_t init = 0x04100000 | 4 << 8 | 0x10;
        uint32_t live = init + 0x1000;
        uint8_t received[TRANSFER_LENGTH];
        uint8_t old_init[TRANSFER_LENGTH];
        size_t i;

        qtest_memset(destination, init, 0xa5, sizeof(received));
        qtest_writel(destination, NEXT_DMA_BASE + channels[0].csr,
                     DMA_CLRCOMPLETE | DMA_SETENABLE | DMA_READ_CMD);
        issue_inquiry_dma(destination, TRANSFER_LENGTH);
        finish_scsi_command(destination);
        g_assert_cmphex(qtest_readl(
                            destination,
                            channel_address(&channels[0], 0x4000)),
                        ==, init + TRANSFER_LENGTH);
        qtest_memread(destination, init, received, sizeof(received));
        g_assert_cmpmem(&received[8], 4, "QEMU", 4);
        g_assert_false(qtest_get_irq(destination, dma_board_inputs[0]));

        /*
         * The latch was one-shot.  Poison its old target and a distinct
         * live buffer, then prove the next transfer follows live NEXT
         * without changing the guest-visible NEXT_INIT register.
         */
        qtest_memset(destination, init, 0x5a, sizeof(old_init));
        qtest_memset(destination, live, 0xa5, sizeof(received));
        qtest_writel(destination,
                     channel_address(&channels[0], 0x4000), live);
        qtest_writel(destination,
                     channel_address(&channels[0], 0x4004),
                     live + TRANSFER_LENGTH);
        g_assert_cmphex(qtest_readl(
                            destination,
                            channel_address(&channels[0], 0x4200)),
                        ==, init);

        issue_inquiry_dma(destination, TRANSFER_LENGTH);
        finish_scsi_command(destination);

        g_assert_cmphex(qtest_readl(
                            destination,
                            channel_address(&channels[0], 0x4000)),
                        ==, live + TRANSFER_LENGTH);
        g_assert_cmphex(qtest_readl(
                            destination,
                            channel_address(&channels[0], 0x4200)),
                        ==, init);
        qtest_memread(destination, live, received, sizeof(received));
        g_assert_cmpmem(&received[8], 4, "QEMU", 4);
        qtest_memread(destination, init, old_init, sizeof(old_init));
        for (i = 0; i < sizeof(old_init); i++) {
            g_assert_cmphex(old_init[i], ==, 0x5a);
        }
        g_assert_true(qtest_get_irq(destination, dma_board_inputs[0]));
    }

    qtest_quit(source);
    qtest_quit(destination);
    g_unlink(socket_path);
    g_assert_cmpint(g_rmdir(tmpdir), ==, 0);
}

static void test_migration_partial_scsi_stage(void)
{
    enum {
        INQUIRY_LENGTH = 66,
        DMA_WINDOW_LENGTH = 96,
        DMA_COMMITTED_LENGTH = 64,
        DMA_FLUSHED_LENGTH = 80,
        DMA_BUFFER = NEXT_TEST_RAM_BASE + 0xc000,
    };
    g_autoptr(GError) error = NULL;
    g_autofree char *tmpdir = NULL;
    g_autofree char *socket_path = NULL;
    g_autofree char *uri = NULL;
    g_autofree char *quoted_uri = NULL;
    g_autofree char *incoming_args = NULL;
    uint8_t received[DMA_WINDOW_LENGTH];
    QTestState *source;
    QTestState *destination;
    size_t edge;
    size_t i;

    tmpdir = g_dir_make_tmp("next-dma-stage-migration-XXXXXX", &error);
    g_assert_no_error(error);
    g_assert_nonnull(tmpdir);
    socket_path = g_build_filename(tmpdir, "migration.sock", NULL);
    uri = g_strdup_printf("unix:%s", socket_path);
    quoted_uri = g_shell_quote(uri);
    incoming_args = g_strdup_printf("-incoming %s", quoted_uri);

    destination = next_dma_start_with_args(true, incoming_args);
    source = next_dma_start_with_args(true, NULL);
    unrealize_next_kbd(source);
    unrealize_next_kbd(destination);
    qtest_irq_intercept_out_named(destination, "/machine/next-dma",
                                  "sysbus-irq");

    qtest_memset(source, DMA_BUFFER, 0xa5, sizeof(received));
    qtest_writel(source, NEXT_DMA_BASE + channels[0].csr,
                 DMA_RESET | DMA_READ_CMD);
    qtest_writel(source, channel_address(&channels[0], 0x4000), DMA_BUFFER);
    qtest_writel(source, channel_address(&channels[0], 0x4004),
                 DMA_BUFFER + DMA_WINDOW_LENGTH);
    qtest_writel(source, NEXT_DMA_BASE + channels[0].csr,
                 DMA_SETENABLE | DMA_READ_CMD);
    issue_inquiry_dma(source, INQUIRY_LENGTH);
    g_assert_cmphex(qtest_readl(
                        source, channel_address(&channels[0], 0x4000)),
                    ==, DMA_BUFFER + DMA_COMMITTED_LENGTH);

    migrate_wait(source, destination, uri);

    g_assert_cmphex(qtest_readl(
                        destination,
                        channel_address(&channels[0], 0x4000)),
                    ==, DMA_BUFFER + DMA_COMMITTED_LENGTH);
    for (edge = 0; edge < 3; edge++) {
        pulse_scsi_fifo_flush(destination);
        g_assert_cmphex(qtest_readl(
                            destination,
                            channel_address(&channels[0], 0x4000)),
                        ==, DMA_BUFFER + DMA_COMMITTED_LENGTH);
    }
    pulse_scsi_fifo_flush(destination);
    g_assert_cmphex(qtest_readl(
                        destination,
                        channel_address(&channels[0], 0x4000)),
                    ==, DMA_BUFFER + DMA_FLUSHED_LENGTH);
    g_assert_cmphex(qtest_readl(destination,
                               NEXT_DMA_BASE + channels[0].csr) &
                    (DMA_ENABLE | DMA_SUPDATE | DMA_COMPLETE),
                    ==, DMA_ENABLE);
    g_assert_false(qtest_get_irq(destination, 0));
    g_assert_cmphex(qtest_readl(destination, NEXT_INTR_STATUS) &
                    NEXT_SCSI_DMA_IRQ, ==, 0);

    qtest_memread(destination, DMA_BUFFER, received, sizeof(received));
    g_assert_cmpmem(&received[8], 4, "QEMU", 4);
    g_assert_cmphex(received[64], ==, 0);
    g_assert_cmphex(received[65], ==, 0);
    for (i = 66; i < DMA_FLUSHED_LENGTH; i++) {
        g_assert_cmphex(received[i], ==, 0);
    }
    for (i = DMA_FLUSHED_LENGTH; i < sizeof(received); i++) {
        g_assert_cmphex(received[i], ==, 0xa5);
    }

    qtest_quit(source);
    qtest_quit(destination);
    g_unlink(socket_path);
    g_assert_cmpint(g_rmdir(tmpdir), ==, 0);
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
    qtest_add_func("/next-cube/dma/video-retrace-interrupt",
                   test_video_retrace_interrupt);
    qtest_add_func("/next-cube/dma/sound-output-final-segment",
                   test_sound_output_final_segment);
    qtest_add_func("/next-cube/dma/sound-output-chained-segments",
                   test_sound_output_chained_segments);
    qtest_add_func("/next-cube/dma/sound-output-range-error-and-reset",
                   test_sound_output_range_error_and_reset);
    qtest_add_func("/next-cube/dma/zero-next-init-valid",
                   test_zero_next_init_valid);
    qtest_add_func("/next-cube/dma/device-reset-all-channels",
                   test_device_reset_all_channels);
    qtest_add_func("/next-cube/dma/functional-irq-invariant",
                   test_functional_irq_invariant);
    qtest_add_func("/next-cube/dma/cross-channel-ack-isolation",
                   test_cross_channel_ack_isolation);
    qtest_add_func("/next-cube/dma/migration-idle-all-channels",
                   test_migration_idle_all_channels);
    qtest_add_func("/next-cube/dma/migration-partial-scsi-stage",
                   test_migration_partial_scsi_stage);
    return g_test_run();
}
