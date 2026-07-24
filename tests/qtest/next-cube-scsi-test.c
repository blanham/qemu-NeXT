/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "libqtest.h"

#define NEXT_DSP_BASE      0x02108000
#define NEXT_DSP_SIZE      8
#define NEXT_DSP_ICR       (NEXT_DSP_BASE + 0)
#define NEXT_PRINTER_BASE  0x0200f000
#define NEXT_PRINTER_CSR   (NEXT_PRINTER_BASE + 0)
#define NEXT_PRINTER_CMD   (NEXT_PRINTER_BASE + 3)
#define NEXT_PRINTER_DATA  (NEXT_PRINTER_BASE + 4)
#define NEXT_DMA_CSR       0x02000010
#define NEXT_DMA_NEXT      0x02004010
#define NEXT_DMA_LIMIT     0x02004014
#define NEXT_DMA_START     0x02004018
#define NEXT_DMA_STOP      0x0200401c
#define NEXT_DMA_NEXT_INIT 0x02004210
#define NEXT_INTR_STATUS   0x02007000
#define NEXT_INTR_MASK     0x02007800
#define NEXT_ESP_TCLO      0x02114000
#define NEXT_ESP_TCMID     0x02114001
#define NEXT_ESP_FIFO      0x02114002
#define NEXT_ESP_CMD       0x02114003
#define NEXT_ESP_BUSID     0x02114004
#define NEXT_ESP_STAT      0x02114004
#define NEXT_ESP_SEL       0x02114005
#define NEXT_ESP_INTR      0x02114005
#define NEXT_ESP_SEQ       0x02114006
#define NEXT_ESP_CFG1      0x02114008
#define NEXT_ESP_CCF       0x02114009
#define NEXT_ESP_TCHI      0x0211400e
#define NEXT_SCSI_CSR      0x02114020
#define NEXT_ROM_SIZE      (128 * 1024)
#define NEXT_DISK_SIZE     (512 * 1024)
#define NEXT_DMA_BUFFER    0x04002000
#define NEXT_DMA_BUFFER2   0x04004000
#define NEXT_DMA_BUFFER3   0x04006000
#define NEXT_SECTOR_SIZE   512

#define ESP_CMD_RESET      0x02
#define ESP_CMD_BUSRESET   0x03
#define ESP_CMD_SEL        0x41
#define ESP_CMD_SELATN     0x42
#define ESP_CMD_TI_DMA     0x90
#define ESP_CMD_ICCS       0x11
#define ESP_CMD_MSGACC     0x12
#define ESP_STAT_INT       0x80
#define ESP_INTR_DC        0x20
#define ESP_CFG1_RESREPT   0x40

#define DMA_SETENABLE      0x00010000
#define DMA_SETSUPDATE     0x00020000
#define DMA_DEV2M          0x00040000
#define DMA_CLRCOMPLETE    0x00080000
#define DMA_RESET          0x00100000
#define DMA_INITBUF        0x00200000
#define DMA_ENABLE         0x01000000
#define DMA_SUPDATE        0x02000000
#define DMA_COMPLETE       0x08000000
#define NEXT_SCSI_IRQ      (1U << 12)
#define NEXT_SCSI_DMA_IRQ  (1U << 26)

#define SCSI_CSR_RESET      0x02
#define SCSI_CSR_CPUDMA     0x10
#define SCSI_CSR_INTMASK    0x20
#define SCSI_CSR_FIFOFL     0x04
#define SCSI_CSR_DMADIR     0x08

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

typedef struct TestDisk {
    int rom_fd;
    int disk_fd;
    char *rom_path;
    char *disk_path;
} TestDisk;

static TestDisk test_disk = {
    .rom_fd = -1,
    .disk_fd = -1,
};

static void issue_inquiry_dma(QTestState *qts, uint8_t length);

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

static void cleanup_test_disk(void *opaque)
{
    TestDisk *disk = opaque;

    qtest_remove_abrt_handler(disk);
    if (disk->rom_fd >= 0) {
        close(disk->rom_fd);
        disk->rom_fd = -1;
    }
    if (disk->disk_fd >= 0) {
        close(disk->disk_fd);
        disk->disk_fd = -1;
    }
    if (disk->rom_path) {
        g_unlink(disk->rom_path);
        g_clear_pointer(&disk->rom_path, g_free);
    }
    if (disk->disk_path) {
        g_unlink(disk->disk_path);
        g_clear_pointer(&disk->disk_path, g_free);
    }
}

static QTestState *next_cube_scsi_disk_start(TestDisk *disk)
{
    g_autofree char *quoted_rom_path = NULL;
    g_autofree char *quoted_disk_path = NULL;

    qtest_add_abrt_handler(cleanup_test_disk, disk);
    g_test_queue_destroy(cleanup_test_disk, disk);

    disk->rom_fd = g_file_open_tmp("next-cube-scsi-rom-XXXXXX",
                                   &disk->rom_path, NULL);
    g_assert_cmpint(disk->rom_fd, >=, 0);
    g_assert_cmpint(ftruncate(disk->rom_fd, NEXT_ROM_SIZE), ==, 0);
    close(disk->rom_fd);
    disk->rom_fd = -1;

    disk->disk_fd = g_file_open_tmp("next-cube-scsi-disk-XXXXXX",
                                    &disk->disk_path, NULL);
    g_assert_cmpint(disk->disk_fd, >=, 0);
    g_assert_cmpint(ftruncate(disk->disk_fd, NEXT_DISK_SIZE), ==, 0);
    close(disk->disk_fd);
    disk->disk_fd = -1;

    quoted_rom_path = g_shell_quote(disk->rom_path);
    quoted_disk_path = g_shell_quote(disk->disk_path);
    return qtest_initf("-machine next-cube -bios %s "
                       "-drive file=%s,if=scsi,format=raw",
                       quoted_rom_path, quoted_disk_path);
}

static uint8_t submit_nodata_cdb(QTestState *qts, const uint8_t cdb[6])
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

static QTestState *start_scsi_write_dma(TestDisk *disk,
                                        const uint8_t source[NEXT_SECTOR_SIZE])
{
    static const uint8_t test_unit_ready[6] = { 0 };
    static const uint8_t write_10[10] = {
        0x2a, 0, 0, 0, 0, 1, 0, 0, 1, 0,
    };
    QTestState *qts;
    int i;

    qts = next_cube_scsi_disk_start(disk);

    /* Consume the disk's power-on unit attention. */
    g_assert_cmphex(submit_nodata_cdb(qts, test_unit_ready), ==, 0x02);
    g_assert_cmphex(submit_nodata_cdb(qts, test_unit_ready), ==, 0x00);

    qtest_memwrite(qts, NEXT_DMA_BUFFER, source, NEXT_SECTOR_SIZE);
    qtest_writel(qts, NEXT_INTR_MASK, NEXT_SCSI_DMA_IRQ | NEXT_SCSI_IRQ);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_DMA_BUFFER + NEXT_SECTOR_SIZE);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE);

    qtest_writeb(qts, NEXT_ESP_BUSID, 0);
    for (i = 0; i < sizeof(write_10); i++) {
        qtest_writeb(qts, NEXT_ESP_FIFO, write_10[i]);
    }
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_SEL);
    qtest_readb(qts, NEXT_ESP_INTR);

    qtest_writeb(qts, NEXT_ESP_TCLO, 0);
    qtest_writeb(qts, NEXT_ESP_TCMID, 2);
    qtest_writeb(qts, NEXT_ESP_TCHI, 0);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_TI_DMA);

    return qts;
}

static void assert_scsi_dma_completed(QTestState *qts)
{
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE | DMA_COMPLETE),
                    ==, DMA_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT),
                    ==, NEXT_DMA_BUFFER + NEXT_SECTOR_SIZE);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_LIMIT),
                    ==, NEXT_DMA_BUFFER + NEXT_SECTOR_SIZE);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    (NEXT_SCSI_DMA_IRQ | NEXT_SCSI_IRQ),
                    ==, NEXT_SCSI_DMA_IRQ | NEXT_SCSI_IRQ);
}

static void test_scsi_dma_control_does_not_raise_interrupt(void)
{
    QTestState *qts = next_cube_scsi_start();

    qtest_writeb(qts, NEXT_SCSI_CSR,
                 SCSI_CSR_CPUDMA | SCSI_CSR_INTMASK);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    NEXT_SCSI_DMA_IRQ, ==, 0);

    qtest_quit(qts);
}

static void test_scsi_disabled_dma_does_not_complete(void)
{
    enum {
        INQUIRY_LENGTH = 64,
    };
    uint8_t received[INQUIRY_LENGTH];
    TestDisk *disk = &test_disk;
    QTestState *qts = next_cube_scsi_disk_start(disk);
    int i;

    /*
     * A reset channel retains its programmed pointers, but the ESP must not
     * be able to advance that stale window or post another DMA interrupt
     * until software explicitly enables the channel again.
     */
    qtest_memset(qts, NEXT_DMA_BUFFER, 0xa5, sizeof(received));
    qtest_writel(qts, NEXT_INTR_MASK, NEXT_SCSI_DMA_IRQ | NEXT_SCSI_IRQ);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_DMA_BUFFER + INQUIRY_LENGTH);

    issue_inquiry_dma(qts, INQUIRY_LENGTH);

    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE | DMA_COMPLETE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, NEXT_DMA_BUFFER);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    NEXT_SCSI_DMA_IRQ, ==, 0);
    qtest_memread(qts, NEXT_DMA_BUFFER, received, sizeof(received));
    for (i = 0; i < sizeof(received); i++) {
        g_assert_cmphex(received[i], ==, 0xa5);
    }

    qtest_quit(qts);
    cleanup_test_disk(disk);
}

static void test_scsi_write_dma(void)
{
    uint8_t source[NEXT_SECTOR_SIZE];
    uint8_t stored[NEXT_SECTOR_SIZE];
    TestDisk *disk = &test_disk;
    QTestState *qts;
    ssize_t bytes;
    int fd;
    int i;

    for (i = 0; i < NEXT_SECTOR_SIZE; i++) {
        source[i] = i ^ 0xa5;
    }

    qts = start_scsi_write_dma(disk, source);
    assert_scsi_dma_completed(qts);

    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) & DMA_COMPLETE, ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    (NEXT_SCSI_DMA_IRQ | NEXT_SCSI_IRQ),
                    ==, NEXT_SCSI_IRQ);

    qtest_quit(qts);

    fd = qemu_open(disk->disk_path, O_RDONLY, NULL);
    g_assert_cmpint(fd, >=, 0);
    bytes = pread(fd, stored, sizeof(stored), NEXT_SECTOR_SIZE);
    close(fd);
    g_assert_cmpint(bytes, ==, sizeof(stored));
    g_assert_cmpmem(stored, sizeof(stored), source, sizeof(source));

    cleanup_test_disk(disk);
}

static void test_scsi_dma_irq_level_invariant(void)
{
    uint8_t source[NEXT_SECTOR_SIZE] = { 0 };
    TestDisk *disk = &test_disk;
    QTestState *qts = start_scsi_write_dma(disk, source);

    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) & DMA_COMPLETE,
                    ==, DMA_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_SCSI_DMA_IRQ,
                    ==, NEXT_SCSI_DMA_IRQ);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_CLRCOMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) & DMA_COMPLETE, ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_SCSI_DMA_IRQ,
                    ==, 0);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_SCSI_DMA_IRQ,
                    ==, 0);

    qtest_quit(qts);
    cleanup_test_disk(disk);
}

static void test_scsi_dma_tail_four_fifofl_edges(void)
{
    static const uint8_t inquiry[6] = { 0x12, 0, 0, 0, 66, 0 };
    enum {
        INQUIRY_LENGTH = 66,
        DMA_WINDOW_LENGTH = 96,
        DMA_INITIAL_TRANSFER = 64,
        DMA_FLUSHED_TRANSFER = 80,
    };
    uint8_t received[DMA_WINDOW_LENGTH];
    TestDisk *disk = &test_disk;
    QTestState *qts = next_cube_scsi_disk_start(disk);
    int i;

    qtest_memset(qts, NEXT_DMA_BUFFER, 0xa5, sizeof(received));
    qtest_writel(qts, NEXT_INTR_MASK, NEXT_SCSI_DMA_IRQ | NEXT_SCSI_IRQ);

    /*
     * Mach gives an unaligned device-to-memory transfer a 96-byte tail
     * buffer.  Poison START/STOP so an incorrect single-update transition
     * cannot accidentally satisfy the live NEXT/LIMIT checks below.
     */
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER + DMA_WINDOW_LENGTH);
    qtest_writel(qts, NEXT_DMA_START, NEXT_DMA_BUFFER + 0x1000);
    qtest_writel(qts, NEXT_DMA_STOP, NEXT_DMA_BUFFER + 0x1100);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);

    qtest_writeb(qts, NEXT_ESP_BUSID, 0);
    for (i = 0; i < sizeof(inquiry); i++) {
        qtest_writeb(qts, NEXT_ESP_FIFO, inquiry[i]);
    }
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_SEL);
    qtest_readb(qts, NEXT_ESP_INTR);

    qtest_writeb(qts, NEXT_ESP_TCLO, INQUIRY_LENGTH);
    qtest_writeb(qts, NEXT_ESP_TCMID, 0);
    qtest_writeb(qts, NEXT_ESP_TCHI, 0);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_TI_DMA);

    /*
     * The DMA stage holds the final two bytes.  Only complete 16-byte
     * beats have reached memory, and the programmed window is not done.
     */
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER + DMA_INITIAL_TRANSFER);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_LIMIT), ==,
                    NEXT_DMA_BUFFER + DMA_WINDOW_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE | DMA_COMPLETE),
                    ==, DMA_ENABLE);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    (NEXT_SCSI_DMA_IRQ | NEXT_SCSI_IRQ),
                    ==, NEXT_SCSI_IRQ);

    qtest_memread(qts, NEXT_DMA_BUFFER, received, sizeof(received));
    g_assert_cmpmem(&received[8], 4, "QEMU", 4);
    for (i = DMA_INITIAL_TRANSFER; i < sizeof(received); i++) {
        g_assert_cmphex(received[i], ==, 0xa5);
    }

    /*
     * NeXTMach clocks the four-word DMA staging FIFO after ESP interrupts.
     * The padded final beat advances NEXT, but still does not fill its
     * deliberately oversized tail window or raise a DMA completion IRQ.
     */
    for (i = 0; i < 3; i++) {
        qtest_writeb(qts, NEXT_SCSI_CSR,
                     SCSI_CSR_INTMASK | SCSI_CSR_CPUDMA |
                     SCSI_CSR_FIFOFL | SCSI_CSR_DMADIR);
        g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                        NEXT_DMA_BUFFER + DMA_INITIAL_TRANSFER);
        qtest_writeb(qts, NEXT_SCSI_CSR,
                     SCSI_CSR_INTMASK | SCSI_CSR_CPUDMA |
                     SCSI_CSR_DMADIR);
        g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                        NEXT_DMA_BUFFER + DMA_INITIAL_TRANSFER);
    }
    qtest_writeb(qts, NEXT_SCSI_CSR,
                 SCSI_CSR_INTMASK | SCSI_CSR_CPUDMA |
                 SCSI_CSR_FIFOFL | SCSI_CSR_DMADIR);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER + DMA_FLUSHED_TRANSFER);
    qtest_writeb(qts, NEXT_SCSI_CSR,
                 SCSI_CSR_INTMASK | SCSI_CSR_CPUDMA | SCSI_CSR_DMADIR);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER + DMA_FLUSHED_TRANSFER);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE | DMA_COMPLETE),
                    ==, DMA_ENABLE);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    NEXT_SCSI_DMA_IRQ, ==, 0);
    qtest_memread(qts, NEXT_DMA_BUFFER, received, sizeof(received));
    g_assert_cmphex(received[64], ==, 0);
    g_assert_cmphex(received[65], ==, 0);
    for (i = DMA_FLUSHED_TRANSFER; i < sizeof(received); i++) {
        g_assert_cmphex(received[i], ==, 0xa5);
    }

    qtest_quit(qts);
    cleanup_test_disk(disk);
}

static void test_scsi_reset_clears_staged_tail(void)
{
    enum {
        INQUIRY_LENGTH = 66,
        DMA_WINDOW_LENGTH = 96,
        DMA_INITIAL_TRANSFER = 64,
    };
    uint8_t received[DMA_WINDOW_LENGTH];
    TestDisk *disk = &test_disk;
    QTestState *qts = next_cube_scsi_disk_start(disk);
    int i;

    qtest_memset(qts, NEXT_DMA_BUFFER, 0xa5, sizeof(received));
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER + DMA_WINDOW_LENGTH);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);

    issue_inquiry_dma(qts, INQUIRY_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER + DMA_INITIAL_TRANSFER);

    for (i = 0; i < 3; i++) {
        qtest_writeb(qts, NEXT_SCSI_CSR,
                     SCSI_CSR_FIFOFL | SCSI_CSR_DMADIR);
        g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                        NEXT_DMA_BUFFER + DMA_INITIAL_TRANSFER);
        qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DMADIR);
    }
    qtest_writeb(qts, NEXT_SCSI_CSR,
                 SCSI_CSR_RESET | SCSI_CSR_FIFOFL | SCSI_CSR_DMADIR);

    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER + DMA_INITIAL_TRANSFER);
    qtest_memread(qts, NEXT_DMA_BUFFER, received, sizeof(received));
    for (i = DMA_INITIAL_TRANSFER; i < sizeof(received); i++) {
        g_assert_cmphex(received[i], ==, 0xa5);
    }

    qtest_quit(qts);
    cleanup_test_disk(disk);
}

static void test_scsi_dma_full_beat_short_limit_not_staged(void)
{
    enum {
        TRANSFER_LENGTH = 16,
        DMA_WINDOW_LENGTH = 8,
        POISON_LENGTH = 32,
    };
    uint8_t received[POISON_LENGTH];
    TestDisk *disk = &test_disk;
    QTestState *qts = next_cube_scsi_disk_start(disk);
    int i;

    qtest_memset(qts, NEXT_DMA_BUFFER, 0xa5, sizeof(received));
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER + DMA_WINDOW_LENGTH);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);

    issue_inquiry_dma(qts, TRANSFER_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, NEXT_DMA_BUFFER);

    for (i = 0; i < 4; i++) {
        qtest_writeb(qts, NEXT_SCSI_CSR,
                     SCSI_CSR_FIFOFL | SCSI_CSR_DMADIR);
        qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DMADIR);
    }

    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, NEXT_DMA_BUFFER);
    qtest_memread(qts, NEXT_DMA_BUFFER, received, sizeof(received));
    for (i = 0; i < sizeof(received); i++) {
        g_assert_cmphex(received[i], ==, 0xa5);
    }

    qtest_quit(qts);
    cleanup_test_disk(disk);
}

static void test_scsi_dma_short_tail_window_not_staged(void)
{
    enum {
        TRANSFER_LENGTH = 8,
        SHORT_WINDOW_LENGTH = 8,
        FLUSH_WINDOW_LENGTH = 16,
        POISON_LENGTH = 32,
    };
    uint8_t received[POISON_LENGTH];
    TestDisk *disk = &test_disk;
    QTestState *qts = next_cube_scsi_disk_start(disk);
    int i;

    qtest_memset(qts, NEXT_DMA_BUFFER, 0xa5, sizeof(received));
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER + SHORT_WINDOW_LENGTH);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);

    issue_inquiry_dma(qts, TRANSFER_LENGTH);
    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER + FLUSH_WINDOW_LENGTH);
    for (i = 0; i < 4; i++) {
        qtest_writeb(qts, NEXT_SCSI_CSR,
                     SCSI_CSR_FIFOFL | SCSI_CSR_DMADIR);
        qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DMADIR);
    }

    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, NEXT_DMA_BUFFER);
    qtest_memread(qts, NEXT_DMA_BUFFER, received, sizeof(received));
    for (i = 0; i < sizeof(received); i++) {
        g_assert_cmphex(received[i], ==, 0xa5);
    }

    qtest_quit(qts);
    cleanup_test_disk(disk);
}

static void test_scsi_dma_short_tail_flush_rechecks_limit(void)
{
    enum {
        TRANSFER_LENGTH = 8,
        STAGING_WINDOW_LENGTH = 16,
        SHORT_WINDOW_LENGTH = 8,
        POISON_LENGTH = 32,
    };
    uint8_t received[POISON_LENGTH];
    TestDisk *disk = &test_disk;
    QTestState *qts = next_cube_scsi_disk_start(disk);
    int i;

    qtest_memset(qts, NEXT_DMA_BUFFER, 0xa5, sizeof(received));
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER + STAGING_WINDOW_LENGTH);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);

    issue_inquiry_dma(qts, TRANSFER_LENGTH);
    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER + SHORT_WINDOW_LENGTH);
    for (i = 0; i < 4; i++) {
        qtest_writeb(qts, NEXT_SCSI_CSR,
                     SCSI_CSR_FIFOFL | SCSI_CSR_DMADIR);
        qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DMADIR);
    }

    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, NEXT_DMA_BUFFER);
    qtest_memread(qts, NEXT_DMA_BUFFER, received, sizeof(received));
    for (i = 0; i < sizeof(received); i++) {
        g_assert_cmphex(received[i], ==, 0xa5);
    }

    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER + STAGING_WINDOW_LENGTH);
    qtest_writeb(qts, NEXT_SCSI_CSR,
                 SCSI_CSR_FIFOFL | SCSI_CSR_DMADIR);
    qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DMADIR);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER + STAGING_WINDOW_LENGTH);

    qtest_quit(qts);
    cleanup_test_disk(disk);
}

static void test_scsi_read_dma_chain(void)
{
    static const uint8_t test_unit_ready[6] = { 0 };
    static const uint8_t read_10[10] = {
        0x28, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    };
    enum {
        SEGMENT_LENGTH = NEXT_SECTOR_SIZE / 2,
    };
    uint8_t first[SEGMENT_LENGTH];
    uint8_t second[SEGMENT_LENGTH];
    TestDisk *disk = &test_disk;
    QTestState *qts = next_cube_scsi_disk_start(disk);
    int i;

    /* Consume the disk's power-on unit attention. */
    g_assert_cmphex(submit_nodata_cdb(qts, test_unit_ready), ==, 0x02);
    g_assert_cmphex(submit_nodata_cdb(qts, test_unit_ready), ==, 0x00);

    qtest_memset(qts, NEXT_DMA_BUFFER, 0xa5, sizeof(first));
    qtest_memset(qts, NEXT_DMA_BUFFER2, 0xa5, sizeof(second));
    qtest_writel(qts, NEXT_INTR_MASK, NEXT_SCSI_DMA_IRQ | NEXT_SCSI_IRQ);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_DMA_BUFFER + SEGMENT_LENGTH);
    qtest_writel(qts, NEXT_DMA_START, NEXT_DMA_BUFFER2);
    qtest_writel(qts, NEXT_DMA_STOP, NEXT_DMA_BUFFER2 + SEGMENT_LENGTH);
    qtest_writel(qts, NEXT_DMA_CSR,
                 DMA_SETENABLE | DMA_SETSUPDATE | DMA_DEV2M);

    qtest_writeb(qts, NEXT_ESP_BUSID, 0);
    for (i = 0; i < sizeof(read_10); i++) {
        qtest_writeb(qts, NEXT_ESP_FIFO, read_10[i]);
    }
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_SEL);
    qtest_readb(qts, NEXT_ESP_INTR);

    qtest_writeb(qts, NEXT_ESP_TCLO, 0);
    qtest_writeb(qts, NEXT_ESP_TCMID, 2);
    qtest_writeb(qts, NEXT_ESP_TCHI, 0);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_TI_DMA);

    qtest_memread(qts, NEXT_DMA_BUFFER, first, sizeof(first));
    qtest_memread(qts, NEXT_DMA_BUFFER2, second, sizeof(second));
    for (i = 0; i < SEGMENT_LENGTH; i++) {
        g_assert_cmphex(first[i], ==, 0);
        g_assert_cmphex(second[i], ==, 0);
    }
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER2 + SEGMENT_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_LIMIT), ==,
                    NEXT_DMA_BUFFER2 + SEGMENT_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE | DMA_COMPLETE),
                    ==, DMA_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    (NEXT_SCSI_DMA_IRQ | NEXT_SCSI_IRQ),
                    ==, NEXT_SCSI_DMA_IRQ | NEXT_SCSI_IRQ);

    qtest_quit(qts);
    cleanup_test_disk(disk);
}

static void test_scsi_chained_tail_overflow(void)
{
    static const uint8_t test_unit_ready[6] = { 0 };
    static const uint8_t read_10[10] = {
        0x28, 0, 0, 0, 0, 0, 0, 0, 4, 0,
    };
    enum {
        FIRST_SEGMENT_LENGTH = 384,
        SECOND_SEGMENT_LENGTH = 1648,
        TRANSFER_LENGTH = 4 * NEXT_SECTOR_SIZE,
        STAGED_LENGTH = 16,
        TAIL_BUFFER_LENGTH = 32,
    };
    uint8_t expected[TRANSFER_LENGTH];
    uint8_t first[FIRST_SEGMENT_LENGTH];
    uint8_t second[SECOND_SEGMENT_LENGTH];
    uint8_t tail[TAIL_BUFFER_LENGTH];
    TestDisk *disk = &test_disk;
    QTestState *qts = next_cube_scsi_disk_start(disk);
    ssize_t bytes;
    int fd;
    int i;

    G_STATIC_ASSERT(FIRST_SEGMENT_LENGTH + SECOND_SEGMENT_LENGTH +
                    STAGED_LENGTH == TRANSFER_LENGTH);

    for (i = 0; i < sizeof(expected); i++) {
        unsigned int sector = i / NEXT_SECTOR_SIZE;
        unsigned int offset = i % NEXT_SECTOR_SIZE;

        expected[i] = 0x31 ^ (sector * 0x53) ^ (offset * 0x9d) ^
                      (offset >> 3);
    }
    fd = qemu_open(disk->disk_path, O_WRONLY, NULL);
    g_assert_cmpint(fd, >=, 0);
    bytes = pwrite(fd, expected, sizeof(expected), 0);
    close(fd);
    g_assert_cmpint(bytes, ==, sizeof(expected));

    /* Consume the disk's power-on unit attention. */
    g_assert_cmphex(submit_nodata_cdb(qts, test_unit_ready), ==, 0x02);
    g_assert_cmphex(submit_nodata_cdb(qts, test_unit_ready), ==, 0x00);

    qtest_memset(qts, NEXT_DMA_BUFFER, 0xa5, sizeof(first));
    qtest_memset(qts, NEXT_DMA_BUFFER2, 0xa5, sizeof(second));
    qtest_memset(qts, NEXT_DMA_BUFFER3, 0xa5, sizeof(tail));
    qtest_writel(qts, NEXT_INTR_MASK, NEXT_SCSI_DMA_IRQ | NEXT_SCSI_IRQ);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER + FIRST_SEGMENT_LENGTH);
    qtest_writel(qts, NEXT_DMA_START, NEXT_DMA_BUFFER2);
    qtest_writel(qts, NEXT_DMA_STOP,
                 NEXT_DMA_BUFFER2 + SECOND_SEGMENT_LENGTH);
    qtest_writel(qts, NEXT_DMA_CSR,
                 DMA_SETENABLE | DMA_SETSUPDATE | DMA_DEV2M);

    qtest_writeb(qts, NEXT_ESP_BUSID, 0);
    for (i = 0; i < sizeof(read_10); i++) {
        qtest_writeb(qts, NEXT_ESP_FIFO, read_10[i]);
    }
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_SEL);
    qtest_readb(qts, NEXT_ESP_INTR);

    qtest_writeb(qts, NEXT_ESP_TCLO, TRANSFER_LENGTH & 0xff);
    qtest_writeb(qts, NEXT_ESP_TCMID, TRANSFER_LENGTH >> 8);
    qtest_writeb(qts, NEXT_ESP_TCHI, 0);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_TI_DMA);

    qtest_memread(qts, NEXT_DMA_BUFFER, first, sizeof(first));
    qtest_memread(qts, NEXT_DMA_BUFFER2, second, sizeof(second));
    qtest_memread(qts, NEXT_DMA_BUFFER3, tail, sizeof(tail));
    g_assert_cmpmem(first, sizeof(first), expected, sizeof(first));
    g_assert_cmpmem(second, sizeof(second),
                    expected + FIRST_SEGMENT_LENGTH, sizeof(second));
    for (i = 0; i < sizeof(tail); i++) {
        g_assert_cmphex(tail[i], ==, 0xa5);
    }
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER2 + SECOND_SEGMENT_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE | DMA_COMPLETE),
                    ==, DMA_COMPLETE);

    /*
     * Mach resets and reprograms the DMA window before clocking the final
     * beat out of the SCSI staging FIFO.
     */
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER3);
    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER3 + TAIL_BUFFER_LENGTH);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);

    for (i = 0; i < 3; i++) {
        int j;

        qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DMADIR);
        qtest_writeb(qts, NEXT_SCSI_CSR,
                     SCSI_CSR_FIFOFL | SCSI_CSR_DMADIR);
        g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                        NEXT_DMA_BUFFER3);
        qtest_memread(qts, NEXT_DMA_BUFFER3, tail, sizeof(tail));
        for (j = 0; j < sizeof(tail); j++) {
            g_assert_cmphex(tail[j], ==, 0xa5);
        }
    }

    qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DMADIR);
    qtest_writeb(qts, NEXT_SCSI_CSR,
                 SCSI_CSR_FIFOFL | SCSI_CSR_DMADIR);
    qtest_memread(qts, NEXT_DMA_BUFFER3, tail, sizeof(tail));
    for (i = 0; i < STAGED_LENGTH; i++) {
        g_assert_cmphex(tail[i], ==,
                        expected[FIRST_SEGMENT_LENGTH +
                                 SECOND_SEGMENT_LENGTH + i]);
    }
    for (i = STAGED_LENGTH; i < sizeof(tail); i++) {
        g_assert_cmphex(tail[i], ==, 0xa5);
    }
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER3 + STAGED_LENGTH);

    qtest_quit(qts);
    cleanup_test_disk(disk);
}

static void issue_inquiry_dma(QTestState *qts, uint8_t length)
{
    uint8_t inquiry[6] = { 0x12, 0, 0, 0, length, 0 };
    int i;

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

static void test_scsi_dma_reset_clears_next_init_valid(void)
{
    enum {
        TRANSFER_LENGTH = 16,
    };
    uint8_t current[TRANSFER_LENGTH];
    uint8_t init[TRANSFER_LENGTH];
    TestDisk *disk = &test_disk;
    QTestState *qts = next_cube_scsi_disk_start(disk);
    bool current_changed = false;
    int i;

    qtest_memset(qts, NEXT_DMA_BUFFER, 0xa5, sizeof(current));
    qtest_memset(qts, NEXT_DMA_BUFFER2, 0x5a, sizeof(init));
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER + TRANSFER_LENGTH);
    qtest_writel(qts, NEXT_DMA_NEXT_INIT, NEXT_DMA_BUFFER2);

    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT_INIT), ==,
                    NEXT_DMA_BUFFER2);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);

    issue_inquiry_dma(qts, TRANSFER_LENGTH);
    finish_scsi_command(qts);

    qtest_memread(qts, NEXT_DMA_BUFFER, current, sizeof(current));
    qtest_memread(qts, NEXT_DMA_BUFFER2, init, sizeof(init));
    for (i = 0; i < sizeof(current); i++) {
        current_changed |= current[i] != 0xa5;
        g_assert_cmphex(init[i], ==, 0x5a);
    }
    g_assert_true(current_changed);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER + TRANSFER_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT_INIT), ==,
                    NEXT_DMA_BUFFER2);

    qtest_quit(qts);
    cleanup_test_disk(disk);
}

static void test_scsi_dma_initbuf_preserves_next_init(void)
{
    enum {
        TRANSFER_LENGTH = 16,
        STAGED_LENGTH = 8,
    };
    uint8_t expected[TRANSFER_LENGTH];
    uint8_t actual[TRANSFER_LENGTH];
    uint8_t staged[TRANSFER_LENGTH];
    TestDisk *disk = &test_disk;
    QTestState *qts = next_cube_scsi_disk_start(disk);
    bool reference_changed = false;
    int i;

    qtest_memset(qts, NEXT_DMA_BUFFER3, 0x3c, sizeof(expected));
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER3);
    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER3 + TRANSFER_LENGTH);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);
    issue_inquiry_dma(qts, TRANSFER_LENGTH);
    finish_scsi_command(qts);
    qtest_memread(qts, NEXT_DMA_BUFFER3, expected, sizeof(expected));
    for (i = 0; i < sizeof(expected); i++) {
        reference_changed |= expected[i] != 0x3c;
    }
    g_assert_true(reference_changed);

    qtest_memset(qts, NEXT_DMA_BUFFER, 0xa5, sizeof(staged));
    qtest_memset(qts, NEXT_DMA_BUFFER2, 0x5a, sizeof(actual));
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER + TRANSFER_LENGTH);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);
    issue_inquiry_dma(qts, STAGED_LENGTH);
    finish_scsi_command(qts);
    qtest_memread(qts, NEXT_DMA_BUFFER, staged, sizeof(staged));
    for (i = 0; i < sizeof(staged); i++) {
        g_assert_cmphex(staged[i], ==, 0xa5);
    }

    qtest_writel(qts, NEXT_DMA_NEXT_INIT, NEXT_DMA_BUFFER2);
    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER2 + TRANSFER_LENGTH);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_INITBUF | DMA_DEV2M);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT_INIT), ==,
                    NEXT_DMA_BUFFER2);

    issue_inquiry_dma(qts, TRANSFER_LENGTH);
    finish_scsi_command(qts);
    qtest_memread(qts, NEXT_DMA_BUFFER2, actual, sizeof(actual));
    g_assert_cmpmem(actual, sizeof(actual), expected, sizeof(expected));
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER2 + TRANSFER_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT_INIT), ==,
                    NEXT_DMA_BUFFER2);

    qtest_quit(qts);
    cleanup_test_disk(disk);
}

static void test_scsi_dma_chain_states(void)
{
    enum {
        SEGMENT_LENGTH = 64,
    };
    TestDisk *disk = &test_disk;
    QTestState *qts = next_cube_scsi_disk_start(disk);

    qtest_writel(qts, NEXT_INTR_MASK, NEXT_SCSI_DMA_IRQ | NEXT_SCSI_IRQ);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_DMA_BUFFER + SEGMENT_LENGTH);
    qtest_writel(qts, NEXT_DMA_START, NEXT_DMA_BUFFER2);
    qtest_writel(qts, NEXT_DMA_STOP, NEXT_DMA_BUFFER2 + SEGMENT_LENGTH);
    qtest_writel(qts, NEXT_DMA_CSR,
                 DMA_SETENABLE | DMA_SETSUPDATE | DMA_DEV2M);

    issue_inquiry_dma(qts, SEGMENT_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, NEXT_DMA_BUFFER2);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_LIMIT), ==,
                    NEXT_DMA_BUFFER2 + SEGMENT_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE | DMA_COMPLETE),
                    ==, DMA_ENABLE | DMA_COMPLETE);

    qtest_writel(qts, NEXT_DMA_CSR, DMA_CLRCOMPLETE | DMA_DEV2M);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE | DMA_COMPLETE),
                    ==, DMA_ENABLE);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    (NEXT_SCSI_DMA_IRQ | NEXT_SCSI_IRQ),
                    ==, NEXT_SCSI_IRQ);
    finish_scsi_command(qts);

    issue_inquiry_dma(qts, SEGMENT_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER2 + SEGMENT_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE | DMA_COMPLETE),
                    ==, DMA_COMPLETE);

    qtest_quit(qts);
    cleanup_test_disk(disk);
}

static void test_scsi_write_dma_chain(void)
{
    static const uint8_t test_unit_ready[6] = { 0 };
    static const uint8_t write_10[10] = {
        0x2a, 0, 0, 0, 0, 1, 0, 0, 1, 0,
    };
    enum {
        SEGMENT_LENGTH = NEXT_SECTOR_SIZE / 2,
    };
    uint8_t first[SEGMENT_LENGTH];
    uint8_t second[SEGMENT_LENGTH];
    uint8_t stored[NEXT_SECTOR_SIZE];
    TestDisk *disk = &test_disk;
    QTestState *qts = next_cube_scsi_disk_start(disk);
    ssize_t bytes;
    int fd;
    int i;

    g_assert_cmphex(submit_nodata_cdb(qts, test_unit_ready), ==, 0x02);
    g_assert_cmphex(submit_nodata_cdb(qts, test_unit_ready), ==, 0x00);

    for (i = 0; i < SEGMENT_LENGTH; i++) {
        first[i] = i ^ 0x5a;
        second[i] = i ^ 0xa5;
    }
    qtest_memwrite(qts, NEXT_DMA_BUFFER, first, sizeof(first));
    qtest_memwrite(qts, NEXT_DMA_BUFFER2, second, sizeof(second));
    qtest_writel(qts, NEXT_INTR_MASK, NEXT_SCSI_DMA_IRQ | NEXT_SCSI_IRQ);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_DMA_BUFFER + SEGMENT_LENGTH);
    qtest_writel(qts, NEXT_DMA_START, NEXT_DMA_BUFFER2);
    qtest_writel(qts, NEXT_DMA_STOP, NEXT_DMA_BUFFER2 + SEGMENT_LENGTH);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | DMA_SETSUPDATE);

    qtest_writeb(qts, NEXT_ESP_BUSID, 0);
    for (i = 0; i < sizeof(write_10); i++) {
        qtest_writeb(qts, NEXT_ESP_FIFO, write_10[i]);
    }
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_SEL);
    qtest_readb(qts, NEXT_ESP_INTR);
    qtest_writeb(qts, NEXT_ESP_TCLO, 0);
    qtest_writeb(qts, NEXT_ESP_TCMID, 2);
    qtest_writeb(qts, NEXT_ESP_TCHI, 0);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_TI_DMA);

    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER2 + SEGMENT_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE | DMA_COMPLETE),
                    ==, DMA_COMPLETE);
    qtest_quit(qts);

    fd = qemu_open(disk->disk_path, O_RDONLY, NULL);
    g_assert_cmpint(fd, >=, 0);
    bytes = pread(fd, stored, sizeof(stored), NEXT_SECTOR_SIZE);
    close(fd);
    g_assert_cmpint(bytes, ==, sizeof(stored));
    g_assert_cmpmem(stored, SEGMENT_LENGTH, first, sizeof(first));
    g_assert_cmpmem(stored + SEGMENT_LENGTH, SEGMENT_LENGTH,
                    second, sizeof(second));

    cleanup_test_disk(disk);
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
        "0000000002106000-000000000210600f (prio 0, i/o): next.mb8795"));
    g_assert_null(strstr(flatview,
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

static void test_printer_mmio_mapping(void)
{
    QTestState *qts = next_cube_scsi_start();
    g_autofree char *flatview = qtest_hmp(qts, "info mtree -f");

    g_assert_nonnull(strstr(flatview,
        "0000000002005000-000000000200dfff (prio 0, i/o): next.mmio"));
    g_assert_nonnull(strstr(flatview,
        "000000000200f000-000000000200f007 (prio 0, i/o): next.printer"));

    qtest_writeb(qts, NEXT_PRINTER_CMD, 0xff);
    g_assert_cmphex(qtest_readb(qts, NEXT_PRINTER_CMD), ==, 0);

    qtest_writel(qts, NEXT_PRINTER_CSR, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, NEXT_PRINTER_CSR), ==, 0);
    qtest_writel(qts, NEXT_PRINTER_DATA, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, NEXT_PRINTER_DATA), ==, 0);

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
    qtest_add_func("/next-cube/scsi/dma-control-does-not-raise-interrupt",
                   test_scsi_dma_control_does_not_raise_interrupt);
    qtest_add_func("/next-cube/scsi/disabled-dma-does-not-complete",
                   test_scsi_disabled_dma_does_not_complete);
    qtest_add_func("/next-cube/scsi/write-dma", test_scsi_write_dma);
    qtest_add_func("/next-cube/scsi/dma-irq-level-invariant",
                   test_scsi_dma_irq_level_invariant);
    qtest_add_func("/next-cube/scsi/tail-four-fifofl-edges",
                   test_scsi_dma_tail_four_fifofl_edges);
    qtest_add_func("/next-cube/scsi/reset-clears-staged-tail",
                   test_scsi_reset_clears_staged_tail);
    qtest_add_func("/next-cube/scsi/full-beat-short-limit-not-staged",
                   test_scsi_dma_full_beat_short_limit_not_staged);
    qtest_add_func("/next-cube/scsi/short-tail-window-not-staged",
                   test_scsi_dma_short_tail_window_not_staged);
    qtest_add_func("/next-cube/scsi/short-tail-flush-rechecks-limit",
                   test_scsi_dma_short_tail_flush_rechecks_limit);
    qtest_add_func("/next-cube/scsi/read-dma-chain",
                   test_scsi_read_dma_chain);
    qtest_add_func("/next-cube/scsi/chained-tail-overflow",
                   test_scsi_chained_tail_overflow);
    qtest_add_func("/next-cube/scsi/dma-reset-clears-next-init-valid",
                   test_scsi_dma_reset_clears_next_init_valid);
    qtest_add_func("/next-cube/scsi/dma-initbuf-preserves-next-init",
                   test_scsi_dma_initbuf_preserves_next_init);
    qtest_add_func("/next-cube/scsi/dma-chain-states",
                   test_scsi_dma_chain_states);
    qtest_add_func("/next-cube/scsi/write-dma-chain",
                   test_scsi_write_dma_chain);
    qtest_add_func("/next-cube/mmio/dsp-mapping", test_dsp_mmio_mapping);
    qtest_add_func("/next-cube/mmio/printer-mapping",
                   test_printer_mmio_mapping);
    return g_test_run();
}
