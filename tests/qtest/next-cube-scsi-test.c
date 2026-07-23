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
#define NEXT_ROM_SIZE      (128 * 1024)
#define NEXT_DISK_SIZE     (512 * 1024)
#define NEXT_DMA_BUFFER    0x04002000
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
#define DMA_COMPLETE       0x08000000

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

static void test_scsi_write_dma(void)
{
    static const uint8_t test_unit_ready[6] = { 0 };
    static const uint8_t write_10[10] = {
        0x2a, 0, 0, 0, 0, 1, 0, 0, 1, 0,
    };
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

    qts = next_cube_scsi_disk_start(disk);

    /* Consume the disk's power-on unit attention. */
    g_assert_cmphex(submit_nodata_cdb(qts, test_unit_ready), ==, 0x02);
    g_assert_cmphex(submit_nodata_cdb(qts, test_unit_ready), ==, 0x00);

    qtest_memwrite(qts, NEXT_DMA_BUFFER, source, sizeof(source));
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_DMA_BUFFER + sizeof(source));
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

    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) & DMA_COMPLETE,
                    ==, DMA_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, NEXT_DMA_BUFFER);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_LIMIT),
                    ==, NEXT_DMA_BUFFER + sizeof(source));

    qtest_quit(qts);

    fd = qemu_open(disk->disk_path, O_RDONLY, NULL);
    g_assert_cmpint(fd, >=, 0);
    bytes = pread(fd, stored, sizeof(stored), NEXT_SECTOR_SIZE);
    close(fd);
    g_assert_cmpint(bytes, ==, sizeof(stored));
    g_assert_cmpmem(stored, sizeof(stored), source, sizeof(source));

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
    qtest_add_func("/next-cube/scsi/write-dma", test_scsi_write_dma);
    qtest_add_func("/next-cube/mmio/dsp-mapping", test_dsp_mmio_mapping);
    qtest_add_func("/next-cube/mmio/printer-mapping",
                   test_printer_mmio_mapping);
    return g_test_run();
}
