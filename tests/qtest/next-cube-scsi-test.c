/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "exec/hwaddr.h"
#include "libqtest.h"
#include "qemu/bswap.h"
#include "scsi/constants.h"

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
#define NEXT_CD_SECTOR_SIZE 2048
#define NEXT_CD_SECTORS     4
#define NEXT_CD_SIZE        (NEXT_CD_SECTOR_SIZE * NEXT_CD_SECTORS)

#define ESP_CMD_RESET      0x02
#define ESP_CMD_BUSRESET   0x03
#define ESP_CMD_SEL        0x41
#define ESP_CMD_SELATN     0x42
#define ESP_CMD_SELATN_DMA (ESP_CMD_SELATN | 0x80)
#define ESP_CMD_NOP_DMA    0x80
#define ESP_CMD_TI_DMA     0x90
#define ESP_CMD_ICCS       0x11
#define ESP_CMD_MSGACC     0x12
#define ESP_STAT_INT       0x80
#define ESP_STAT_TC        0x10
#define ESP_STAT_PHASE     0x07
#define ESP_STAT_DI        0x01
#define ESP_INTR_DC        0x20
#define ESP_INTR_IL        0x40
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
#define SCSI_CSR_DATA_OUT   0xf0
#define SCSI_CSR_DATA_IN    0xf8

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

typedef struct TestMedia {
    int rom_fd;
    int disk_fd;
    int cd_fd;
    char *rom_path;
    char *disk_path;
    char *cd_path;
} TestMedia;

static TestDisk test_disk = {
    .rom_fd = -1,
    .disk_fd = -1,
};

static TestMedia test_media = {
    .rom_fd = -1,
    .disk_fd = -1,
    .cd_fd = -1,
};

static TestMedia test_media_destination = {
    .rom_fd = -1,
    .disk_fd = -1,
    .cd_fd = -1,
};

static const uint8_t read_capacity_10[10] = { 0x25 };
static const uint8_t read_10_cd_sector_1[10] = {
    0x28, 0, 0, 0, 0, 1, 0, 0, 1, 0,
};
static const uint8_t netbsd_inquiry_32[6] = {
    0x12, 0, 0, 0, 32, 0,
};
static const uint8_t netbsd_disk_inquiry_prefix[32] = {
    0x00, 0x00, 0x05, 0x12, 0x1f, 0x00, 0x00, 0x10,
    'Q', 'E', 'M', 'U', ' ', ' ', ' ', ' ',
    'Q', 'E', 'M', 'U', ' ', 'H', 'A', 'R',
    'D', 'D', 'I', 'S', 'K', ' ', ' ', ' ',
};
static const uint8_t netbsd_cd_inquiry_prefix[32] = {
    0x05, 0x80, 0x05, 0x12, 0x1f, 0x00, 0x00, 0x10,
    'Q', 'E', 'M', 'U', ' ', ' ', ' ', ' ',
    'Q', 'E', 'M', 'U', ' ', 'C', 'D', '-',
    'R', 'O', 'M', ' ', ' ', ' ', ' ', ' ',
};
static const uint8_t netbsd_disk_capacity[8] = {
    0x00, 0x00, 0x03, 0xff, 0x00, 0x00, 0x02, 0x00,
};
static const uint8_t netbsd_cd_capacity[8] = {
    0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x08, 0x00,
};

static void issue_inquiry_dma(QTestState *qts, uint8_t target,
                              uint8_t length);

static uint8_t next_cube_cd_byte(size_t sector, size_t offset)
{
    return ((offset ^ 0xa5) + sector) & 0xff;
}

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

static void cleanup_test_media(void *opaque)
{
    TestMedia *media = opaque;

    qtest_remove_abrt_handler(media);
    if (media->rom_fd >= 0) {
        close(media->rom_fd);
        media->rom_fd = -1;
    }
    if (media->disk_fd >= 0) {
        close(media->disk_fd);
        media->disk_fd = -1;
    }
    if (media->cd_fd >= 0) {
        close(media->cd_fd);
        media->cd_fd = -1;
    }
    if (media->rom_path) {
        g_unlink(media->rom_path);
        g_clear_pointer(&media->rom_path, g_free);
    }
    if (media->disk_path) {
        g_unlink(media->disk_path);
        g_clear_pointer(&media->disk_path, g_free);
    }
    if (media->cd_path) {
        g_unlink(media->cd_path);
        g_clear_pointer(&media->cd_path, g_free);
    }
}

static QTestState *next_cube_scsi_cdrom_start(TestMedia *media)
{
    g_autofree char *quoted_rom_path = NULL;
    g_autofree char *quoted_cd_path = NULL;

    media->rom_fd = -1;
    media->disk_fd = -1;
    media->cd_fd = -1;
    qtest_add_abrt_handler(cleanup_test_media, media);
    g_test_queue_destroy(cleanup_test_media, media);

    media->rom_fd = g_file_open_tmp("next-cube-scsi-rom-XXXXXX",
                                    &media->rom_path, NULL);
    g_assert_cmpint(media->rom_fd, >=, 0);
    g_assert_cmpint(ftruncate(media->rom_fd, NEXT_ROM_SIZE), ==, 0);
    close(media->rom_fd);
    media->rom_fd = -1;

    media->cd_fd = g_file_open_tmp("next-cube-scsi-cd-XXXXXX",
                                   &media->cd_path, NULL);
    g_assert_cmpint(media->cd_fd, >=, 0);
    g_assert_cmpint(ftruncate(media->cd_fd, NEXT_CD_SIZE), ==, 0);
    close(media->cd_fd);
    media->cd_fd = -1;

    quoted_rom_path = g_shell_quote(media->rom_path);
    quoted_cd_path = g_shell_quote(media->cd_path);
    return qtest_initf("-machine next-cube -bios %s -cdrom %s",
                       quoted_rom_path, quoted_cd_path);
}

static QTestState *next_cube_scsi_media_start_with_args(TestMedia *media,
                                                        const char *extra_args)
{
    uint8_t cd[NEXT_CD_SIZE];
    g_autofree char *quoted_rom_path = NULL;
    g_autofree char *quoted_disk_path = NULL;
    g_autofree char *quoted_cd_path = NULL;
    int i;

    media->rom_fd = -1;
    media->disk_fd = -1;
    media->cd_fd = -1;
    qtest_add_abrt_handler(cleanup_test_media, media);
    g_test_queue_destroy(cleanup_test_media, media);

    media->rom_fd = g_file_open_tmp("next-cube-scsi-rom-XXXXXX",
                                    &media->rom_path, NULL);
    g_assert_cmpint(media->rom_fd, >=, 0);
    g_assert_cmpint(ftruncate(media->rom_fd, NEXT_ROM_SIZE), ==, 0);
    close(media->rom_fd);
    media->rom_fd = -1;

    media->disk_fd = g_file_open_tmp("next-cube-scsi-disk-XXXXXX",
                                     &media->disk_path, NULL);
    g_assert_cmpint(media->disk_fd, >=, 0);
    g_assert_cmpint(ftruncate(media->disk_fd, NEXT_DISK_SIZE), ==, 0);
    close(media->disk_fd);
    media->disk_fd = -1;

    for (i = 0; i < sizeof(cd); i++) {
        cd[i] = next_cube_cd_byte(i / NEXT_CD_SECTOR_SIZE,
                                  i % NEXT_CD_SECTOR_SIZE);
    }
    g_assert_cmpint(memcmp(&cd[0], &cd[NEXT_CD_SECTOR_SIZE],
                           NEXT_CD_SECTOR_SIZE), !=, 0);
    media->cd_fd = g_file_open_tmp("next-cube-scsi-cd-XXXXXX",
                                   &media->cd_path, NULL);
    g_assert_cmpint(media->cd_fd, >=, 0);
    g_assert_cmpint(qemu_write_full(media->cd_fd, cd, sizeof(cd)), ==,
                    sizeof(cd));
    close(media->cd_fd);
    media->cd_fd = -1;

    quoted_rom_path = g_shell_quote(media->rom_path);
    quoted_disk_path = g_shell_quote(media->disk_path);
    quoted_cd_path = g_shell_quote(media->cd_path);
    return qtest_initf(
        "-machine next-cube -bios %s "
        "-drive file=%s,if=none,id=netbsd-disk,format=raw "
        "-device scsi-hd,drive=netbsd-disk,channel=0,scsi-id=0,lun=0,"
        "vendor=QEMU,product='QEMU HARDDISK',ver=2.5+,scsi_version=5,"
        "logical_block_size=512,physical_block_size=512 "
        "-drive file=%s,if=none,id=netbsd-cd,format=raw,readonly=on "
        "-device scsi-cd,drive=netbsd-cd,channel=0,scsi-id=3,lun=0,"
        "vendor=QEMU,product='QEMU CD-ROM',ver=2.5+,scsi_version=5,"
        "logical_block_size=2048,physical_block_size=2048 %s",
        quoted_rom_path, quoted_disk_path, quoted_cd_path, extra_args ?: "");
}

static QTestState *next_cube_scsi_media_start(TestMedia *media)
{
    return next_cube_scsi_media_start_with_args(media, NULL);
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

static void migrate_wait(QTestState *source, QTestState *destination,
                         const char *uri)
{
    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    qtest_qmp_eventwait(source, "STOP");
    qtest_qmp_eventwait(destination, "RESUME");
}

static void test_scsi_cdrom_command_line(void)
{
    TestMedia *media = &test_media;
    QTestState *qts = next_cube_scsi_cdrom_start(media);

    qtest_quit(qts);
    cleanup_test_media(media);
}

static uint8_t submit_nodata_cdb(QTestState *qts, uint8_t target,
                                 const uint8_t cdb[6])
{
    uint8_t status;
    int i;

    qtest_writeb(qts, NEXT_ESP_BUSID, target);
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

static void wait_for_scsi_command_completion(QTestState *qts)
{
    const gint64 deadline =
        g_get_monotonic_time() + 5 * G_TIME_SPAN_SECOND;
    uint8_t status = 0;

    while (g_get_monotonic_time() < deadline) {
        status = qtest_readb(qts, NEXT_ESP_STAT);
        if (status & ESP_STAT_INT) {
            return;
        }
        qtest_clock_step(qts, 1);
    }

    g_error("timed out waiting for SCSI command completion: ESP status "
            "0x%02x, DMA NEXT 0x%08" PRIx32 ", DMA CSR 0x%08" PRIx32,
            status, qtest_readl(qts, NEXT_DMA_NEXT),
            qtest_readl(qts, NEXT_DMA_CSR));
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
    g_assert_cmphex(submit_nodata_cdb(qts, 0, test_unit_ready), ==, 0x02);
    g_assert_cmphex(submit_nodata_cdb(qts, 0, test_unit_ready), ==, 0x00);

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
    qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DATA_OUT);
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

static void assert_scsi_dma_next_read_alias(QTestState *qts,
                                             uint32_t expected)
{
    /* NEXT and NEXT_INIT both read the effective current pointer. */
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, expected);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT_INIT), ==, expected);
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
    qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DATA_IN);
    g_assert_cmphex(qtest_readb(qts, NEXT_SCSI_CSR), ==, SCSI_CSR_DATA_IN);

    issue_inquiry_dma(qts, 0, INQUIRY_LENGTH);

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
    qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DATA_IN);
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

    issue_inquiry_dma(qts, 0, INQUIRY_LENGTH);
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

    issue_inquiry_dma(qts, 0, TRANSFER_LENGTH);
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

    issue_inquiry_dma(qts, 0, TRANSFER_LENGTH);
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

    issue_inquiry_dma(qts, 0, TRANSFER_LENGTH);
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
    g_assert_cmphex(submit_nodata_cdb(qts, 0, test_unit_ready), ==, 0x02);
    g_assert_cmphex(submit_nodata_cdb(qts, 0, test_unit_ready), ==, 0x00);

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
    qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DATA_IN);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_TI_DMA);

    wait_for_scsi_command_completion(qts);
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
    g_assert_cmphex(submit_nodata_cdb(qts, 0, test_unit_ready), ==, 0x02);
    g_assert_cmphex(submit_nodata_cdb(qts, 0, test_unit_ready), ==, 0x00);

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
    qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DATA_IN);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_TI_DMA);

    wait_for_scsi_command_completion(qts);
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

static void issue_inquiry_dma(QTestState *qts, uint8_t target,
                              uint8_t length)
{
    uint8_t inquiry[6] = { 0x12, 0, 0, 0, length, 0 };
    int i;

    qtest_writeb(qts, NEXT_ESP_BUSID, target);
    for (i = 0; i < sizeof(inquiry); i++) {
        qtest_writeb(qts, NEXT_ESP_FIFO, inquiry[i]);
    }
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_SEL);
    qtest_readb(qts, NEXT_ESP_INTR);

    qtest_writeb(qts, NEXT_ESP_TCLO, length);
    qtest_writeb(qts, NEXT_ESP_TCMID, 0);
    qtest_writeb(qts, NEXT_ESP_TCHI, 0);
    qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DATA_IN);
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

static uint32_t read_esp_transfer_count(QTestState *qts)
{
    return qtest_readb(qts, NEXT_ESP_TCLO) |
           qtest_readb(qts, NEXT_ESP_TCMID) << 8 |
           qtest_readb(qts, NEXT_ESP_TCHI) << 16;
}

static void assert_poisoned_scsi_buffer(QTestState *qts, hwaddr guest_buffer,
                                        size_t length, uint8_t poison)
{
    g_autofree uint8_t *received = g_malloc(length);
    size_t i;

    qtest_memread(qts, guest_buffer, received, length);
    for (i = 0; i < length; i++) {
        g_assert_cmphex(received[i], ==, poison);
    }
}

static void assert_netbsd_ti_pending(QTestState *qts, size_t transfer_len,
                                     bool next_dma_enabled)
{
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT) & ESP_STAT_PHASE,
                    ==, ESP_STAT_DI);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT) & ESP_STAT_TC, ==, 0);
    g_assert_cmphex(read_esp_transfer_count(qts), ==, transfer_len);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, NEXT_DMA_BUFFER);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE | DMA_COMPLETE),
                    ==, next_dma_enabled ? DMA_ENABLE : 0);
    assert_poisoned_scsi_buffer(qts, NEXT_DMA_BUFFER, transfer_len, 0xa5);
}

static void consume_power_on_unit_attention(QTestState *qts, uint8_t target)
{
    static const uint8_t test_unit_ready[6] = { 0 };

    g_assert_cmphex(submit_nodata_cdb(qts, target, test_unit_ready), ==, 0x02);
    g_assert_cmphex(submit_nodata_cdb(qts, target, test_unit_ready), ==, 0x00);
}

static void run_netbsd_data_in(QTestState *qts, uint8_t target,
                               const uint8_t *cdb, size_t cdb_len,
                               const uint8_t *expected, size_t transfer_len,
                               bool enable_next_dma_before_ti)
{
    enum {
        DMA_BEAT_LENGTH = 16,
        DMA_FIFOFL_EDGES = 4,
        NETBSD_DCTL_DATA_IN_LOW = 0xe8,
    };
    g_autofree uint8_t *received = g_malloc(transfer_len);
    size_t dma_window_len = QEMU_ALIGN_UP(transfer_len, DMA_BEAT_LENGTH);
    size_t i;

    g_assert_cmpuint(transfer_len, <=, 0xffffff);
    consume_power_on_unit_attention(qts, target);
    qtest_writeb(qts, NEXT_SCSI_CSR, NETBSD_DCTL_DATA_IN_LOW);
    g_assert_cmphex(qtest_readb(qts, NEXT_SCSI_CSR), ==,
                    NETBSD_DCTL_DATA_IN_LOW);

    qtest_memset(qts, NEXT_DMA_BUFFER, 0xa5, dma_window_len);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_DMA_BUFFER + dma_window_len);
    if (enable_next_dma_before_ti) {
        qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);
    }

    qtest_writeb(qts, NEXT_ESP_BUSID, target);
    qtest_writeb(qts, NEXT_ESP_FIFO, 0xc0);
    for (i = 0; i < cdb_len; i++) {
        qtest_writeb(qts, NEXT_ESP_FIFO, cdb[i]);
    }
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_SELATN);
    qtest_readb(qts, NEXT_ESP_INTR);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT) & ESP_STAT_PHASE,
                    ==, ESP_STAT_DI);

    qtest_writeb(qts, NEXT_ESP_TCLO, transfer_len & 0xff);
    qtest_writeb(qts, NEXT_ESP_TCMID, (transfer_len >> 8) & 0xff);
    qtest_writeb(qts, NEXT_ESP_TCHI, (transfer_len >> 16) & 0xff);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_NOP_DMA);
    g_assert_cmphex(read_esp_transfer_count(qts), ==, transfer_len);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_TI_DMA);

    assert_netbsd_ti_pending(qts, transfer_len,
                             enable_next_dma_before_ti);
    if (!enable_next_dma_before_ti) {
        qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);
        assert_netbsd_ti_pending(qts, transfer_len, true);
    }

    qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DATA_IN);
    g_assert_cmphex(read_esp_transfer_count(qts), ==, 0);
    finish_scsi_command(qts);

    if (transfer_len % DMA_BEAT_LENGTH) {
        for (i = 0; i < DMA_FIFOFL_EDGES; i++) {
            qtest_writeb(qts, NEXT_SCSI_CSR,
                         SCSI_CSR_DATA_IN | SCSI_CSR_FIFOFL);
            qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DATA_IN);
        }
    }

    qtest_memread(qts, NEXT_DMA_BUFFER, received, transfer_len);
    g_assert_cmpmem(received, transfer_len, expected, transfer_len);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER + dma_window_len);
}

static void prepare_new_netbsd_inquiry_data(QTestState *qts, uint8_t target)
{
    size_t i;

    qtest_memset(qts, NEXT_DMA_BUFFER, 0xa5,
                 sizeof(netbsd_disk_inquiry_prefix));
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER + sizeof(netbsd_disk_inquiry_prefix));
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);

    qtest_writeb(qts, NEXT_ESP_BUSID, target);
    qtest_writeb(qts, NEXT_ESP_FIFO, 0xc0);
    for (i = 0; i < sizeof(netbsd_inquiry_32); i++) {
        qtest_writeb(qts, NEXT_ESP_FIFO, netbsd_inquiry_32[i]);
    }
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_SELATN);
    qtest_readb(qts, NEXT_ESP_INTR);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT) & ESP_STAT_PHASE,
                    ==, ESP_STAT_DI);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_CMD), ==, ESP_CMD_SELATN);
}

static void issue_new_netbsd_inquiry_ti(QTestState *qts, uint8_t target)
{
    prepare_new_netbsd_inquiry_data(qts, target);

    qtest_writeb(qts, NEXT_ESP_TCLO,
                 sizeof(netbsd_disk_inquiry_prefix));
    qtest_writeb(qts, NEXT_ESP_TCMID, 0);
    qtest_writeb(qts, NEXT_ESP_TCHI, 0);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_NOP_DMA);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_TI_DMA);
}

static void assert_new_netbsd_inquiry_complete(QTestState *qts)
{
    uint8_t received[sizeof(netbsd_disk_inquiry_prefix)];

    g_assert_cmphex(read_esp_transfer_count(qts), ==, 0);
    finish_scsi_command(qts);
    qtest_memread(qts, NEXT_DMA_BUFFER, received, sizeof(received));
    g_assert_cmpmem(received, sizeof(received), netbsd_disk_inquiry_prefix,
                    sizeof(netbsd_disk_inquiry_prefix));
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER + sizeof(received));
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE | DMA_COMPLETE),
                    ==, DMA_COMPLETE);
}

static void wait_scsi_migration_failed(QTestState *qts)
{
    enum {
        MIGRATION_POLL_LIMIT = 10000,
    };
    unsigned int i;

    for (i = 0; i < MIGRATION_POLL_LIMIT; i++) {
        QDict *response = qtest_qmp_assert_success_ref(
            qts, "{ 'execute': 'query-migrate' }");
        const char *status = qdict_get_str(response, "status");

        if (!strcmp(status, "failed")) {
            const char *error_desc =
                qdict_get_try_str(response, "error-desc");

            g_assert_nonnull(error_desc);
            g_assert_nonnull(strstr(error_desc, "NeXT SCSI"));
            g_assert_nonnull(strstr(error_desc, "deferred DMA"));
            g_test_message("outgoing migration failure: %s", error_desc);
            qobject_unref(response);
            return;
        }
        if (!strcmp(status, "completed") || !strcmp(status, "cancelled")) {
            g_error("outgoing migration entered unexpected terminal state "
                    "'%s'", status);
        }
        qobject_unref(response);
        g_usleep(1000);
    }

    g_error("timed out waiting for failed outgoing migration");
}

static void assert_vm_running(QTestState *qts, bool expected)
{
    g_autoptr(QDict) response = qtest_qmp_assert_success_ref(
        qts, "{ 'execute': 'query-status' }");

    g_assert_cmpint(qdict_get_bool(response, "running"), ==, expected);
}

static void test_scsi_netbsd_order_disk_inquiry(void)
{
    TestMedia *media = &test_media;
    QTestState *qts = next_cube_scsi_media_start(media);

    run_netbsd_data_in(qts, 0, netbsd_inquiry_32,
                       sizeof(netbsd_inquiry_32),
                       netbsd_disk_inquiry_prefix,
                       sizeof(netbsd_disk_inquiry_prefix), false);
    qtest_quit(qts);
    cleanup_test_media(media);
}

static void test_scsi_netbsd_order_cd_inquiry(void)
{
    TestMedia *media = &test_media;
    QTestState *qts = next_cube_scsi_media_start(media);

    run_netbsd_data_in(qts, 3, netbsd_inquiry_32,
                       sizeof(netbsd_inquiry_32), netbsd_cd_inquiry_prefix,
                       sizeof(netbsd_cd_inquiry_prefix), false);
    qtest_quit(qts);
    cleanup_test_media(media);
}

static void test_scsi_netbsd_order_disk_read_capacity(void)
{
    TestMedia *media = &test_media;
    QTestState *qts = next_cube_scsi_media_start(media);

    run_netbsd_data_in(qts, 0, read_capacity_10, sizeof(read_capacity_10),
                       netbsd_disk_capacity, sizeof(netbsd_disk_capacity),
                       false);
    qtest_quit(qts);
    cleanup_test_media(media);
}

static void test_scsi_netbsd_order_cd_read_capacity(void)
{
    TestMedia *media = &test_media;
    QTestState *qts = next_cube_scsi_media_start(media);

    run_netbsd_data_in(qts, 3, read_capacity_10, sizeof(read_capacity_10),
                       netbsd_cd_capacity, sizeof(netbsd_cd_capacity), false);
    qtest_quit(qts);
    cleanup_test_media(media);
}

static void test_scsi_migration_rejects_pending_cpudma_low_ti(void)
{
    enum {
        NETBSD_DCTL_DATA_IN_LOW = 0xe8,
    };
    g_autoptr(GError) error = NULL;
    g_autofree char *tmpdir = NULL;
    g_autofree char *socket_path = NULL;
    g_autofree char *uri = NULL;
    g_autofree char *quoted_uri = NULL;
    g_autofree char *incoming_args = NULL;
    TestMedia *source_media = &test_media;
    TestMedia *destination_media = &test_media_destination;
    QTestState *source;
    QTestState *destination;

    tmpdir = g_dir_make_tmp("next-scsi-reject-pending-XXXXXX", &error);
    g_assert_no_error(error);
    g_assert_nonnull(tmpdir);
    socket_path = g_build_filename(tmpdir, "migration.sock", NULL);
    uri = g_strdup_printf("unix:%s", socket_path);
    quoted_uri = g_shell_quote(uri);
    incoming_args = g_strdup_printf("-incoming %s", quoted_uri);

    destination = next_cube_scsi_media_start_with_args(destination_media,
                                                        incoming_args);
    source = next_cube_scsi_media_start(source_media);
    unrealize_next_kbd(source);
    unrealize_next_kbd(destination);
    qtest_set_expected_status(destination, 1);

    consume_power_on_unit_attention(source, 0);
    qtest_writeb(source, NEXT_SCSI_CSR, NETBSD_DCTL_DATA_IN_LOW);
    issue_new_netbsd_inquiry_ti(source, 0);
    assert_netbsd_ti_pending(source,
                             sizeof(netbsd_disk_inquiry_prefix), true);

    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_scsi_migration_failed(source);
    assert_vm_running(source, true);
    qtest_wait_qemu(destination);
    assert_netbsd_ti_pending(source,
                             sizeof(netbsd_disk_inquiry_prefix), true);

    qtest_writeb(source, NEXT_SCSI_CSR, SCSI_CSR_DATA_IN);
    assert_new_netbsd_inquiry_complete(source);

    qtest_quit(destination);
    cleanup_test_media(destination_media);
    g_unlink(socket_path);

    g_clear_pointer(&socket_path, g_free);
    g_clear_pointer(&uri, g_free);
    g_clear_pointer(&quoted_uri, g_free);
    g_clear_pointer(&incoming_args, g_free);
    socket_path = g_build_filename(tmpdir, "retry.sock", NULL);
    uri = g_strdup_printf("unix:%s", socket_path);
    quoted_uri = g_shell_quote(uri);
    incoming_args = g_strdup_printf("-incoming %s", quoted_uri);
    destination = next_cube_scsi_media_start_with_args(destination_media,
                                                        incoming_args);
    unrealize_next_kbd(destination);
    migrate_wait(source, destination, uri);

    qtest_quit(source);
    qtest_quit(destination);
    cleanup_test_media(source_media);
    cleanup_test_media(destination_media);
    g_unlink(socket_path);
    g_assert_cmpint(g_rmdir(tmpdir), ==, 0);
}

static void test_scsi_migration_rejects_pre_ti_async_window(void)
{
    enum {
        NETBSD_DCTL_DATA_IN_LOW = 0xe8,
    };
    g_autoptr(GError) error = NULL;
    g_autofree char *tmpdir = NULL;
    g_autofree char *socket_path = NULL;
    g_autofree char *uri = NULL;
    g_autofree char *quoted_uri = NULL;
    g_autofree char *incoming_args = NULL;
    TestMedia *source_media = &test_media;
    TestMedia *destination_media = &test_media_destination;
    QTestState *source;
    QTestState *destination;

    tmpdir = g_dir_make_tmp("next-scsi-reject-async-XXXXXX", &error);
    g_assert_no_error(error);
    g_assert_nonnull(tmpdir);
    socket_path = g_build_filename(tmpdir, "migration.sock", NULL);
    uri = g_strdup_printf("unix:%s", socket_path);
    quoted_uri = g_shell_quote(uri);
    incoming_args = g_strdup_printf("-incoming %s", quoted_uri);

    destination = next_cube_scsi_media_start_with_args(destination_media,
                                                        incoming_args);
    source = next_cube_scsi_media_start(source_media);
    unrealize_next_kbd(source);
    unrealize_next_kbd(destination);
    qtest_set_expected_status(destination, 1);

    consume_power_on_unit_attention(source, 0);
    qtest_writeb(source, NEXT_SCSI_CSR, NETBSD_DCTL_DATA_IN_LOW);
    prepare_new_netbsd_inquiry_data(source, 0);
    g_assert_cmphex(qtest_readb(source, NEXT_ESP_TCLO), ==, 0);
    g_assert_cmphex(qtest_readb(source, NEXT_ESP_TCMID), ==, 0);
    assert_poisoned_scsi_buffer(source, NEXT_DMA_BUFFER,
                                sizeof(netbsd_disk_inquiry_prefix), 0xa5);

    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_scsi_migration_failed(source);
    assert_vm_running(source, true);
    qtest_wait_qemu(destination);
    g_assert_cmphex(qtest_readb(source, NEXT_ESP_CMD), ==, ESP_CMD_SELATN);
    g_assert_cmphex(qtest_readb(source, NEXT_ESP_STAT) & ESP_STAT_PHASE,
                    ==, ESP_STAT_DI);

    qtest_writeb(source, NEXT_ESP_TCLO,
                 sizeof(netbsd_disk_inquiry_prefix));
    qtest_writeb(source, NEXT_ESP_TCMID, 0);
    qtest_writeb(source, NEXT_ESP_TCHI, 0);
    qtest_writeb(source, NEXT_ESP_CMD, ESP_CMD_NOP_DMA);
    qtest_writeb(source, NEXT_ESP_CMD, ESP_CMD_TI_DMA);
    assert_netbsd_ti_pending(source,
                             sizeof(netbsd_disk_inquiry_prefix), true);
    qtest_writeb(source, NEXT_SCSI_CSR, SCSI_CSR_DATA_IN);
    assert_new_netbsd_inquiry_complete(source);

    qtest_quit(source);
    qtest_quit(destination);
    cleanup_test_media(source_media);
    cleanup_test_media(destination_media);
    g_unlink(socket_path);
    g_assert_cmpint(g_rmdir(tmpdir), ==, 0);
}

static void test_scsi_migration_rejects_deferred_dma_selection(void)
{
    enum {
        NETBSD_DCTL_DATA_IN_LOW = 0xe8,
    };
    uint8_t command_data[1 + sizeof(netbsd_inquiry_32)] = { 0xc0 };
    uint8_t received[sizeof(command_data)];
    g_autoptr(GError) error = NULL;
    g_autofree char *tmpdir = NULL;
    g_autofree char *socket_path = NULL;
    g_autofree char *uri = NULL;
    g_autofree char *quoted_uri = NULL;
    g_autofree char *incoming_args = NULL;
    TestMedia *source_media = &test_media;
    TestMedia *destination_media = &test_media_destination;
    QTestState *source;
    QTestState *destination;

    memcpy(&command_data[1], netbsd_inquiry_32, sizeof(netbsd_inquiry_32));
    tmpdir = g_dir_make_tmp("next-scsi-reject-selection-XXXXXX", &error);
    g_assert_no_error(error);
    g_assert_nonnull(tmpdir);
    socket_path = g_build_filename(tmpdir, "migration.sock", NULL);
    uri = g_strdup_printf("unix:%s", socket_path);
    quoted_uri = g_shell_quote(uri);
    incoming_args = g_strdup_printf("-incoming %s", quoted_uri);

    destination = next_cube_scsi_media_start_with_args(destination_media,
                                                        incoming_args);
    source = next_cube_scsi_media_start(source_media);
    unrealize_next_kbd(source);
    unrealize_next_kbd(destination);
    qtest_set_expected_status(destination, 1);

    consume_power_on_unit_attention(source, 0);
    qtest_writeb(source, NEXT_SCSI_CSR, NETBSD_DCTL_DATA_IN_LOW);
    qtest_memwrite(source, NEXT_DMA_BUFFER, command_data,
                   sizeof(command_data));
    qtest_writel(source, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(source, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(source, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER + sizeof(command_data));
    qtest_writel(source, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);

    qtest_writeb(source, NEXT_ESP_BUSID, 0);
    qtest_writeb(source, NEXT_ESP_TCLO, sizeof(command_data));
    qtest_writeb(source, NEXT_ESP_TCMID, 0);
    qtest_writeb(source, NEXT_ESP_TCHI, 0);
    qtest_writeb(source, NEXT_ESP_CMD, ESP_CMD_SELATN_DMA);

    /*
     * DMA selection is deferred before esp_select(), so no SCSI request has
     * delivered data and async_len remains zero; dma_cb is the sole live
     * transient state here.
     */
    g_assert_cmphex(qtest_readb(source, NEXT_ESP_CMD), ==,
                    ESP_CMD_SELATN_DMA);
    g_assert_cmphex(qtest_readl(source, NEXT_DMA_NEXT), ==, NEXT_DMA_BUFFER);
    qtest_memread(source, NEXT_DMA_BUFFER, received, sizeof(received));
    g_assert_cmpmem(received, sizeof(received), command_data,
                    sizeof(command_data));

    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_scsi_migration_failed(source);
    assert_vm_running(source, true);
    qtest_wait_qemu(destination);
    g_assert_cmphex(qtest_readb(source, NEXT_ESP_CMD), ==,
                    ESP_CMD_SELATN_DMA);
    g_assert_cmphex(qtest_readl(source, NEXT_DMA_NEXT), ==, NEXT_DMA_BUFFER);
    qtest_memread(source, NEXT_DMA_BUFFER, received, sizeof(received));
    g_assert_cmpmem(received, sizeof(received), command_data,
                    sizeof(command_data));

    /*
     * Enabling CPUDMA runs the deferred selection callback and supplies the
     * CDB from the programmed DMA window.
     */
    qtest_writeb(source, NEXT_SCSI_CSR, SCSI_CSR_DATA_IN);
    qtest_writel(source, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(source, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(source, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER + sizeof(netbsd_disk_inquiry_prefix));
    qtest_writel(source, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);
    qtest_writeb(source, NEXT_ESP_TCLO,
                 sizeof(netbsd_disk_inquiry_prefix));
    qtest_writeb(source, NEXT_ESP_TCMID, 0);
    qtest_writeb(source, NEXT_ESP_TCHI, 0);
    qtest_writeb(source, NEXT_ESP_CMD, ESP_CMD_NOP_DMA);
    qtest_writeb(source, NEXT_ESP_CMD, ESP_CMD_TI_DMA);
    assert_new_netbsd_inquiry_complete(source);

    qtest_quit(destination);
    cleanup_test_media(destination_media);
    g_unlink(socket_path);

    g_clear_pointer(&socket_path, g_free);
    g_clear_pointer(&uri, g_free);
    g_clear_pointer(&quoted_uri, g_free);
    g_clear_pointer(&incoming_args, g_free);
    socket_path = g_build_filename(tmpdir, "retry.sock", NULL);
    uri = g_strdup_printf("unix:%s", socket_path);
    quoted_uri = g_shell_quote(uri);
    incoming_args = g_strdup_printf("-incoming %s", quoted_uri);
    destination = next_cube_scsi_media_start_with_args(destination_media,
                                                        incoming_args);
    unrealize_next_kbd(destination);
    migrate_wait(source, destination, uri);

    qtest_quit(source);
    qtest_quit(destination);
    cleanup_test_media(source_media);
    cleanup_test_media(destination_media);
    g_unlink(socket_path);
    g_assert_cmpint(g_rmdir(tmpdir), ==, 0);
}

static void test_scsi_cpudma_low_retains_response(void)
{
    TestMedia *media = &test_media;
    QTestState *qts = next_cube_scsi_media_start(media);

    run_netbsd_data_in(qts, 0, netbsd_inquiry_32,
                       sizeof(netbsd_inquiry_32),
                       netbsd_disk_inquiry_prefix,
                       sizeof(netbsd_disk_inquiry_prefix), true);
    qtest_quit(qts);
    cleanup_test_media(media);
}

static void test_scsi_reset_forces_cpudma_low_for_new_ti(void)
{
    TestMedia *media = &test_media;
    QTestState *qts = next_cube_scsi_media_start(media);

    consume_power_on_unit_attention(qts, 0);
    qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DATA_IN);
    g_assert_cmphex(qtest_readb(qts, NEXT_SCSI_CSR), ==, SCSI_CSR_DATA_IN);

    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readb(qts, NEXT_SCSI_CSR), ==, 0);
    consume_power_on_unit_attention(qts, 0);

    issue_new_netbsd_inquiry_ti(qts, 0);
    assert_netbsd_ti_pending(qts, sizeof(netbsd_disk_inquiry_prefix), true);
    qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DATA_IN);
    assert_new_netbsd_inquiry_complete(qts);

    qtest_quit(qts);
    cleanup_test_media(media);
}

static void run_scsi_migration_owner_test(bool cpudma_high)
{
    enum {
        NETBSD_DCTL_DATA_IN_LOW = 0xe8,
    };
    g_autoptr(GError) error = NULL;
    g_autofree char *tmpdir = NULL;
    g_autofree char *socket_path = NULL;
    g_autofree char *uri = NULL;
    g_autofree char *quoted_uri = NULL;
    g_autofree char *incoming_args = NULL;
    TestMedia *source_media = &test_media;
    TestMedia *destination_media = &test_media_destination;
    QTestState *source;
    QTestState *destination;

    tmpdir = g_dir_make_tmp("next-scsi-owner-migration-XXXXXX", &error);
    g_assert_no_error(error);
    g_assert_nonnull(tmpdir);
    socket_path = g_build_filename(tmpdir, "migration.sock", NULL);
    uri = g_strdup_printf("unix:%s", socket_path);
    quoted_uri = g_shell_quote(uri);
    incoming_args = g_strdup_printf("-incoming %s", quoted_uri);

    destination = next_cube_scsi_media_start_with_args(destination_media,
                                                        incoming_args);
    source = next_cube_scsi_media_start(source_media);
    unrealize_next_kbd(source);
    unrealize_next_kbd(destination);

    consume_power_on_unit_attention(source, 0);
    qtest_writeb(source, NEXT_SCSI_CSR, SCSI_CSR_DATA_IN);
    g_assert_cmphex(qtest_readb(source, NEXT_SCSI_CSR), ==,
                    SCSI_CSR_DATA_IN);
    if (!cpudma_high) {
        qtest_writeb(source, NEXT_SCSI_CSR, NETBSD_DCTL_DATA_IN_LOW);
        g_assert_cmphex(qtest_readb(source, NEXT_SCSI_CSR), ==,
                        NETBSD_DCTL_DATA_IN_LOW);
    }

    migrate_wait(source, destination, uri);
    g_assert_cmphex(qtest_readb(destination, NEXT_SCSI_CSR), ==,
                    cpudma_high ? SCSI_CSR_DATA_IN :
                                  NETBSD_DCTL_DATA_IN_LOW);

    issue_new_netbsd_inquiry_ti(destination, 0);
    if (cpudma_high) {
        assert_new_netbsd_inquiry_complete(destination);
    } else {
        assert_netbsd_ti_pending(destination,
                                 sizeof(netbsd_disk_inquiry_prefix), true);
        qtest_writeb(destination, NEXT_SCSI_CSR, SCSI_CSR_DATA_IN);
        assert_new_netbsd_inquiry_complete(destination);
    }

    qtest_quit(source);
    qtest_quit(destination);
    cleanup_test_media(source_media);
    cleanup_test_media(destination_media);
    g_unlink(socket_path);
    g_assert_cmpint(g_rmdir(tmpdir), ==, 0);
}

static void test_scsi_migration_restores_cpudma_low_for_new_ti(void)
{
    run_scsi_migration_owner_test(false);
}

static void test_scsi_migration_restores_cpudma_high_for_new_ti(void)
{
    run_scsi_migration_owner_test(true);
}

static void read_scsi_dma(QTestState *qts, uint8_t target,
                          const uint8_t *cdb, size_t cdb_len,
                          hwaddr guest_buffer, size_t transfer_len)
{
    enum {
        DMA_BEAT_LENGTH = 16,
        DMA_FIFOFL_EDGES = 4,
    };
    size_t dma_window_len;
    size_t i;

    g_assert_cmpuint(transfer_len, <=, 0xffffff);
    dma_window_len = QEMU_ALIGN_UP(transfer_len, DMA_BEAT_LENGTH);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, guest_buffer);
    qtest_writel(qts, NEXT_DMA_LIMIT, guest_buffer + dma_window_len);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);
    qtest_writeb(qts, NEXT_ESP_BUSID, target);
    for (i = 0; i < cdb_len; i++) {
        qtest_writeb(qts, NEXT_ESP_FIFO, cdb[i]);
    }
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_SEL);
    qtest_readb(qts, NEXT_ESP_INTR);
    qtest_writeb(qts, NEXT_ESP_TCLO, transfer_len & 0xff);
    qtest_writeb(qts, NEXT_ESP_TCMID, (transfer_len >> 8) & 0xff);
    qtest_writeb(qts, NEXT_ESP_TCHI, (transfer_len >> 16) & 0xff);
    qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DATA_IN);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_TI_DMA);
    wait_for_scsi_command_completion(qts);
    finish_scsi_command(qts);

    if (transfer_len % DMA_BEAT_LENGTH) {
        for (i = 0; i < DMA_FIFOFL_EDGES; i++) {
            qtest_writeb(qts, NEXT_SCSI_CSR,
                         SCSI_CSR_INTMASK | SCSI_CSR_CPUDMA |
                         SCSI_CSR_FIFOFL | SCSI_CSR_DMADIR);
            qtest_writeb(qts, NEXT_SCSI_CSR,
                         SCSI_CSR_INTMASK | SCSI_CSR_CPUDMA |
                         SCSI_CSR_DMADIR);
        }
    }
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    guest_buffer + dma_window_len);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE | DMA_COMPLETE),
                    ==, DMA_COMPLETE);
}

static void test_scsi_disk_and_cd_inquiry(void)
{
    static const uint8_t test_unit_ready[6] = { 0 };
    static const uint8_t inquiry_cdb[6] = { 0x12, 0, 0, 0, 36, 0 };
    enum {
        INQUIRY_LENGTH = 36,
        INQUIRY_POISON = 0xcc,
    };
    uint8_t inquiry[INQUIRY_LENGTH];
    TestMedia *media = &test_media;
    QTestState *qts = next_cube_scsi_media_start(media);

    g_assert_cmphex(submit_nodata_cdb(qts, 0, test_unit_ready), ==, 0x02);
    g_assert_cmphex(submit_nodata_cdb(qts, 0, test_unit_ready), ==, 0x00);
    g_assert_cmphex(submit_nodata_cdb(qts, 3, test_unit_ready), ==, 0x02);
    g_assert_cmphex(submit_nodata_cdb(qts, 3, test_unit_ready), ==, 0x00);

    qtest_memset(qts, NEXT_DMA_BUFFER, INQUIRY_POISON, sizeof(inquiry));
    read_scsi_dma(qts, 0, inquiry_cdb, sizeof(inquiry_cdb),
                  NEXT_DMA_BUFFER, sizeof(inquiry));
    qtest_memread(qts, NEXT_DMA_BUFFER, inquiry, sizeof(inquiry));
    g_assert_cmphex(inquiry[0] & 0x1f, ==, 0x00);
    g_assert_cmpmem(&inquiry[8], 4, "QEMU", 4);
    g_assert_cmphex(inquiry[35], !=, INQUIRY_POISON);

    qtest_memset(qts, NEXT_DMA_BUFFER, INQUIRY_POISON, sizeof(inquiry));
    read_scsi_dma(qts, 3, inquiry_cdb, sizeof(inquiry_cdb),
                  NEXT_DMA_BUFFER, sizeof(inquiry));
    qtest_memread(qts, NEXT_DMA_BUFFER, inquiry, sizeof(inquiry));
    g_assert_cmphex(inquiry[0] & 0x1f, ==, 0x05);
    g_assert_cmpmem(&inquiry[8], 4, "QEMU", 4);
    g_assert_cmphex(inquiry[35], !=, INQUIRY_POISON);

    qtest_writeb(qts, NEXT_ESP_CCF, NEXT_ESP_CCF_VALUE);
    qtest_writeb(qts, NEXT_ESP_SEL, NEXT_ESP_SEL_VALUE);
    qtest_writeb(qts, NEXT_ESP_BUSID, 2);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_SELATN);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT) & ESP_STAT_INT, ==, 0);
    qtest_clock_step(qts, NEXT_ESP_SEL_NS - 1);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT) & ESP_STAT_INT, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT), ==, ESP_STAT_INT);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_SEQ), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_INTR), ==, ESP_INTR_DC);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT) & ESP_STAT_INT, ==, 0);

    qtest_quit(qts);
    cleanup_test_media(media);
}

static void test_scsi_legacy_cdb_lun_inquiry(void)
{
    static const uint8_t legacy_lun_inquiry[6] = {
        0x12, 4 << 5, 0, 0, 36, 0,
    };
    uint8_t inquiry[36];
    TestMedia *media = &test_media;
    QTestState *qts = next_cube_scsi_media_start(media);

    qtest_memset(qts, NEXT_DMA_BUFFER, 0xcc, sizeof(inquiry));
    read_scsi_dma(qts, 3, legacy_lun_inquiry, sizeof(legacy_lun_inquiry),
                  NEXT_DMA_BUFFER, sizeof(inquiry));
    qtest_memread(qts, NEXT_DMA_BUFFER, inquiry, sizeof(inquiry));
    g_assert_cmphex(inquiry[0], ==, TYPE_NO_LUN);

    qtest_quit(qts);
    cleanup_test_media(media);
}

static void read_cd_capacity_dma(QTestState *qts)
{
    read_scsi_dma(qts, 3, read_capacity_10, sizeof(read_capacity_10),
                  NEXT_DMA_BUFFER, 8);
}

static void test_scsi_cd_read_capacity(void)
{
    static const uint8_t test_unit_ready[6] = { 0 };
    uint8_t capacity[8];
    TestMedia *media = &test_media;
    QTestState *qts = next_cube_scsi_media_start(media);

    g_assert_cmphex(submit_nodata_cdb(qts, 3, test_unit_ready), ==, 0x02);
    g_assert_cmphex(submit_nodata_cdb(qts, 3, test_unit_ready), ==, 0x00);

    qtest_memset(qts, NEXT_DMA_BUFFER, 0xa5, sizeof(capacity));
    read_cd_capacity_dma(qts);
    qtest_memread(qts, NEXT_DMA_BUFFER, capacity, sizeof(capacity));
    g_assert_cmpuint(ldl_be_p(&capacity[0]), ==, NEXT_CD_SECTORS - 1);
    g_assert_cmpuint(ldl_be_p(&capacity[4]), ==, NEXT_CD_SECTOR_SIZE);

    qtest_quit(qts);
    cleanup_test_media(media);
}

static void read_cd_sector_1_dma(QTestState *qts)
{
    read_scsi_dma(qts, 3, read_10_cd_sector_1,
                  sizeof(read_10_cd_sector_1), NEXT_DMA_BUFFER,
                  NEXT_CD_SECTOR_SIZE);
}

static void test_scsi_cd_read_10(void)
{
    static const uint8_t test_unit_ready[6] = { 0 };
    uint8_t sector[NEXT_CD_SECTOR_SIZE];
    TestMedia *media = &test_media;
    QTestState *qts = next_cube_scsi_media_start(media);
    int i;

    g_assert_cmphex(submit_nodata_cdb(qts, 3, test_unit_ready), ==, 0x02);
    g_assert_cmphex(submit_nodata_cdb(qts, 3, test_unit_ready), ==, 0x00);

    qtest_memset(qts, NEXT_DMA_BUFFER, 0xcc, sizeof(sector));
    read_cd_sector_1_dma(qts);
    qtest_memread(qts, NEXT_DMA_BUFFER, sector, sizeof(sector));
    for (i = 0; i < sizeof(sector); i++) {
        g_assert_cmphex(sector[i], ==, next_cube_cd_byte(1, i));
    }

    qtest_quit(qts);
    cleanup_test_media(media);
}

static void read_cd_sector_1_dma_chain(QTestState *qts)
{
    enum {
        SEGMENT_LENGTH = NEXT_CD_SECTOR_SIZE / 2,
    };
    int i;

    qtest_writel(qts, NEXT_INTR_MASK, NEXT_SCSI_DMA_IRQ | NEXT_SCSI_IRQ);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_DMA_BUFFER + SEGMENT_LENGTH);
    qtest_writel(qts, NEXT_DMA_START, NEXT_DMA_BUFFER2);
    qtest_writel(qts, NEXT_DMA_STOP,
                 NEXT_DMA_BUFFER2 + SEGMENT_LENGTH);
    qtest_writel(qts, NEXT_DMA_CSR,
                 DMA_SETENABLE | DMA_SETSUPDATE | DMA_DEV2M);

    qtest_writeb(qts, NEXT_ESP_BUSID, 3);
    for (i = 0; i < sizeof(read_10_cd_sector_1); i++) {
        qtest_writeb(qts, NEXT_ESP_FIFO, read_10_cd_sector_1[i]);
    }
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_SEL);
    qtest_readb(qts, NEXT_ESP_INTR);
    qtest_writeb(qts, NEXT_ESP_TCLO, NEXT_CD_SECTOR_SIZE & 0xff);
    qtest_writeb(qts, NEXT_ESP_TCMID, NEXT_CD_SECTOR_SIZE >> 8);
    qtest_writeb(qts, NEXT_ESP_TCHI, 0);
    qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DATA_IN);
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_TI_DMA);

    wait_for_scsi_command_completion(qts);
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
    finish_scsi_command(qts);
}

static void test_scsi_cd_read_10_chain(void)
{
    static const uint8_t test_unit_ready[6] = { 0 };
    enum {
        SEGMENT_LENGTH = NEXT_CD_SECTOR_SIZE / 2,
    };
    uint8_t first[SEGMENT_LENGTH];
    uint8_t second[SEGMENT_LENGTH];
    TestMedia *media = &test_media;
    QTestState *qts = next_cube_scsi_media_start(media);
    int i;

    g_assert_cmphex(submit_nodata_cdb(qts, 3, test_unit_ready), ==, 0x02);
    g_assert_cmphex(submit_nodata_cdb(qts, 3, test_unit_ready), ==, 0x00);

    qtest_memset(qts, NEXT_DMA_BUFFER, 0xcc, sizeof(first));
    qtest_memset(qts, NEXT_DMA_BUFFER2, 0xcc, sizeof(second));
    read_cd_sector_1_dma_chain(qts);
    qtest_memread(qts, NEXT_DMA_BUFFER, first, sizeof(first));
    qtest_memread(qts, NEXT_DMA_BUFFER2, second, sizeof(second));
    for (i = 0; i < SEGMENT_LENGTH; i++) {
        g_assert_cmphex(first[i], ==, next_cube_cd_byte(1, i));
        g_assert_cmphex(second[i], ==,
                        next_cube_cd_byte(1, SEGMENT_LENGTH + i));
    }

    qtest_quit(qts);
    cleanup_test_media(media);
}

static void reset_after_staged_cd_inquiry_then_read(QTestState *qts)
{
    enum {
        INQUIRY_LENGTH = 36,
        DMA_INITIAL_TRANSFER = 32,
        DMA_WINDOW_LENGTH = 48,
    };
    uint8_t inquiry[DMA_WINDOW_LENGTH];
    int i;

    qtest_memset(qts, NEXT_DMA_BUFFER, 0xcc, sizeof(inquiry));
    qtest_writel(qts, NEXT_INTR_MASK, NEXT_SCSI_DMA_IRQ | NEXT_SCSI_IRQ);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER + DMA_WINDOW_LENGTH);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);

    issue_inquiry_dma(qts, 3, INQUIRY_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER + DMA_INITIAL_TRANSFER);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_LIMIT), ==,
                    NEXT_DMA_BUFFER + DMA_WINDOW_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE | DMA_COMPLETE),
                    ==, DMA_ENABLE);
    qtest_memread(qts, NEXT_DMA_BUFFER, inquiry, sizeof(inquiry));
    g_assert_cmpmem(&inquiry[8], 4, "QEMU", 4);
    for (i = DMA_INITIAL_TRANSFER; i < sizeof(inquiry); i++) {
        g_assert_cmphex(inquiry[i], ==, 0xcc);
    }

    /* Four inquiry bytes remain staged after the two complete DMA beats. */
    qtest_writeb(qts, NEXT_ESP_CMD, ESP_CMD_RESET);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT) & ESP_STAT_INT, ==, 0);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_DEV2M);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE | DMA_COMPLETE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_SCSI_DMA_IRQ,
                    ==, 0);
    qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_RESET | SCSI_CSR_DMADIR);
    g_assert_cmphex(qtest_readb(qts, NEXT_SCSI_CSR), ==,
                    SCSI_CSR_RESET | SCSI_CSR_DMADIR);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT) & ESP_STAT_INT, ==, 0);

    read_scsi_dma(qts, 3, read_10_cd_sector_1,
                  sizeof(read_10_cd_sector_1), NEXT_DMA_BUFFER2,
                  NEXT_CD_SECTOR_SIZE);
}

static void test_scsi_cd_reset_after_staged_inquiry(void)
{
    static const uint8_t test_unit_ready[6] = { 0 };
    uint8_t sector[NEXT_CD_SECTOR_SIZE];
    TestMedia *media = &test_media;
    QTestState *qts = next_cube_scsi_media_start(media);
    int i;

    g_assert_cmphex(submit_nodata_cdb(qts, 3, test_unit_ready), ==, 0x02);
    g_assert_cmphex(submit_nodata_cdb(qts, 3, test_unit_ready), ==, 0x00);

    qtest_memset(qts, NEXT_DMA_BUFFER2, 0xcc, sizeof(sector));
    reset_after_staged_cd_inquiry_then_read(qts);
    qtest_memread(qts, NEXT_DMA_BUFFER2, sector, sizeof(sector));
    for (i = 0; i < sizeof(sector); i++) {
        g_assert_cmphex(sector[i], ==, next_cube_cd_byte(1, i));
    }

    qtest_quit(qts);
    cleanup_test_media(media);
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
    /* RESET invalidates the latch, so readback falls back to live NEXT. */
    assert_scsi_dma_next_read_alias(qts, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | DMA_DEV2M);

    issue_inquiry_dma(qts, 0, TRANSFER_LENGTH);
    finish_scsi_command(qts);

    qtest_memread(qts, NEXT_DMA_BUFFER, current, sizeof(current));
    qtest_memread(qts, NEXT_DMA_BUFFER2, init, sizeof(init));
    for (i = 0; i < sizeof(current); i++) {
        current_changed |= current[i] != 0xa5;
        g_assert_cmphex(init[i], ==, 0x5a);
    }
    g_assert_true(current_changed);
    assert_scsi_dma_next_read_alias(qts,
                                    NEXT_DMA_BUFFER + TRANSFER_LENGTH);

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
    issue_inquiry_dma(qts, 0, TRANSFER_LENGTH);
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
    issue_inquiry_dma(qts, 0, STAGED_LENGTH);
    finish_scsi_command(qts);
    qtest_memread(qts, NEXT_DMA_BUFFER, staged, sizeof(staged));
    for (i = 0; i < sizeof(staged); i++) {
        g_assert_cmphex(staged[i], ==, 0xa5);
    }

    qtest_writel(qts, NEXT_DMA_NEXT_INIT, NEXT_DMA_BUFFER2);
    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_DMA_BUFFER2 + TRANSFER_LENGTH);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_INITBUF | DMA_DEV2M);
    /* INITBUF preserves the valid latch and its effective readback. */
    assert_scsi_dma_next_read_alias(qts, NEXT_DMA_BUFFER2);

    issue_inquiry_dma(qts, 0, TRANSFER_LENGTH);
    finish_scsi_command(qts);
    qtest_memread(qts, NEXT_DMA_BUFFER2, actual, sizeof(actual));
    g_assert_cmpmem(actual, sizeof(actual), expected, sizeof(expected));
    assert_scsi_dma_next_read_alias(qts,
                                    NEXT_DMA_BUFFER2 + TRANSFER_LENGTH);

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

    issue_inquiry_dma(qts, 0, SEGMENT_LENGTH);
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

    issue_inquiry_dma(qts, 0, SEGMENT_LENGTH);
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

    g_assert_cmphex(submit_nodata_cdb(qts, 0, test_unit_ready), ==, 0x02);
    g_assert_cmphex(submit_nodata_cdb(qts, 0, test_unit_ready), ==, 0x00);

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
    qtest_writeb(qts, NEXT_SCSI_CSR, SCSI_CSR_DATA_OUT);
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

static void test_illegal_command_interrupt(void)
{
    QTestState *qts = next_cube_scsi_start();

    /*
     * This is the illegal-command probe in the NeXT ROM's extended SCSI
     * self-test.  The ESP must report the fault in the interrupt register,
     * not in the status register alongside STAT_INT.
     */
    qtest_readb(qts, NEXT_ESP_INTR);
    qtest_writeb(qts, NEXT_ESP_CMD, 0xff);

    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_STAT), ==, ESP_STAT_INT);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_INTR), ==, ESP_INTR_IL);
    assert_esp_quiet(qts);
    g_assert_cmphex(qtest_readb(qts, NEXT_ESP_INTR), ==, 0);

    qtest_quit(qts);
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
    qtest_add_func("/next-cube/scsi/illegal-command-interrupt",
                   test_illegal_command_interrupt);
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
    qtest_add_func("/next-cube/scsi/netbsd-order-disk-inquiry",
                   test_scsi_netbsd_order_disk_inquiry);
    qtest_add_func("/next-cube/scsi/netbsd-order-cd-inquiry",
                   test_scsi_netbsd_order_cd_inquiry);
    qtest_add_func("/next-cube/scsi/netbsd-order-disk-read-capacity",
                   test_scsi_netbsd_order_disk_read_capacity);
    qtest_add_func("/next-cube/scsi/netbsd-order-cd-read-capacity",
                   test_scsi_netbsd_order_cd_read_capacity);
    qtest_add_func(
        "/next-cube/scsi/migration-rejects-pending-cpudma-low-ti",
        test_scsi_migration_rejects_pending_cpudma_low_ti);
    qtest_add_func(
        "/next-cube/scsi/migration-rejects-pre-ti-async-window",
        test_scsi_migration_rejects_pre_ti_async_window);
    qtest_add_func(
        "/next-cube/scsi/migration-rejects-deferred-dma-selection",
        test_scsi_migration_rejects_deferred_dma_selection);
    qtest_add_func("/next-cube/scsi/cpudma-low-retains-response",
                   test_scsi_cpudma_low_retains_response);
    qtest_add_func("/next-cube/scsi/reset-forces-cpudma-low-for-new-ti",
                   test_scsi_reset_forces_cpudma_low_for_new_ti);
    qtest_add_func(
        "/next-cube/scsi/migration-restores-cpudma-low-for-new-ti",
        test_scsi_migration_restores_cpudma_low_for_new_ti);
    qtest_add_func(
        "/next-cube/scsi/migration-restores-cpudma-high-for-new-ti",
        test_scsi_migration_restores_cpudma_high_for_new_ti);
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
    qtest_add_func("/next-cube/scsi/cdrom-command-line",
                   test_scsi_cdrom_command_line);
    qtest_add_func("/next-cube/scsi/disk-and-cd-inquiry",
                   test_scsi_disk_and_cd_inquiry);
    qtest_add_func("/next-cube/scsi/legacy-cdb-lun-inquiry",
                   test_scsi_legacy_cdb_lun_inquiry);
    qtest_add_func("/next-cube/scsi/cd-read-capacity",
                   test_scsi_cd_read_capacity);
    qtest_add_func("/next-cube/scsi/cd-read-10",
                   test_scsi_cd_read_10);
    qtest_add_func("/next-cube/scsi/cd-read-10-chain",
                   test_scsi_cd_read_10_chain);
    qtest_add_func("/next-cube/scsi/cd-reset-after-staged-inquiry",
                   test_scsi_cd_reset_after_staged_inquiry);
    qtest_add_func("/next-cube/mmio/dsp-mapping", test_dsp_mmio_mapping);
    qtest_add_func("/next-cube/mmio/printer-mapping",
                   test_printer_mmio_mapping);
    return g_test_run();
}
