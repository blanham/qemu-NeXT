/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "libqtest.h"

#define NEXT_OPTICAL_BASE       0x02112000
#define NEXT_OPTICAL_SIZE       0x20
#define NEXT_DISR               (NEXT_OPTICAL_BASE + 0x04)
#define NEXT_DIMR               (NEXT_OPTICAL_BASE + 0x05)
#define NEXT_CONTROL2           (NEXT_OPTICAL_BASE + 0x06)
#define NEXT_CONTROL1           (NEXT_OPTICAL_BASE + 0x07)
#define NEXT_DESR               (NEXT_OPTICAL_BASE + 0x0a)
#define NEXT_ECCCNT             (NEXT_OPTICAL_BASE + 0x0b)

#define NEXT_DMA_BASE           0x02000000
#define NEXT_OPTICAL_DMA_CSR    (NEXT_DMA_BASE + 0x050)
#define NEXT_OPTICAL_DMA_NEXT   (NEXT_DMA_BASE + 0x4050)
#define NEXT_OPTICAL_DMA_LIMIT  (NEXT_DMA_BASE + 0x4054)
#define NEXT_OPTICAL_DMA_INIT   (NEXT_DMA_BASE + 0x4250)
#define NEXT_INTR_STATUS        0x02007000

#define NEXT_RAM_BASE           0x04010000
#define NEXT_INPUT              (NEXT_RAM_BASE + 0x0000)
#define NEXT_CODED              (NEXT_RAM_BASE + 0x1000)
#define NEXT_MUTATED            (NEXT_RAM_BASE + 0x2000)
#define NEXT_RESULT             (NEXT_RAM_BASE + 0x3000)

#define NEXT_ROM_SIZE           (128 * 1024)
#define NEXT_UNCODED_SIZE       1024
#define NEXT_CODED_SIZE         1296

#define OMD_DATA_ERR            0x80
#define OMD_ECC_DONE            0x08
#define OMD_ATTN                0x02
#define OMD_CMD_COMPL           0x01
#define OMD_ERR_ECC             0x01
#define OMD_ECC_SELECT          0x40
#define OMD_ECC_DECODE          0x20
#define OMD_ECC_WRITE           0x40
#define OMD_ECC_READ            0x80

#define DMA_SETENABLE           0x00010000
#define DMA_READ_CMD            0x00040000
#define DMA_CLRCOMPLETE         0x00080000
#define DMA_RESET               0x00100000
#define DMA_INITBUF             0x00200000
#define DMA_ENABLE              0x01000000
#define DMA_READ                0x04000000
#define DMA_COMPLETE            0x08000000
#define DMA_BUSEXC              0x10000000

#define NEXT_OPTICAL_DMA_IRQ    (1U << 25)
#define NEXT_OPTICAL_DEVICE_IRQ (1U << 13)
#define NEXT_OPTICAL_MTREE      \
    "0000000002112000-000000000211201f (prio 0, i/o): next-optical"

typedef struct TestROM {
    int fd;
    char *path;
} TestROM;

typedef struct TestMigration {
    char *tmpdir;
    char *socket_path;
} TestMigration;

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
    rom->fd = g_file_open_tmp("next-optical-rom-XXXXXX",
                              &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;
    return rom;
}

static QTestState *next_machine_start(const char *machine, const char *args)
{
    TestROM *rom = create_test_rom();
    g_autofree char *quoted_rom_path = g_shell_quote(rom->path);

    return qtest_initf("-machine %s -bios %s %s",
                       machine, quoted_rom_path, args ?: "");
}

static void cleanup_test_migration(void *opaque)
{
    TestMigration *migration = opaque;

    qtest_remove_abrt_handler(migration);
    if (migration->socket_path) {
        g_unlink(migration->socket_path);
    }
    if (migration->tmpdir) {
        g_rmdir(migration->tmpdir);
    }
    g_free(migration->socket_path);
    g_free(migration->tmpdir);
    g_free(migration);
}

static TestMigration *create_test_migration(void)
{
    g_autoptr(GError) error = NULL;
    TestMigration *migration = g_new0(TestMigration, 1);

    qtest_add_abrt_handler(cleanup_test_migration, migration);
    g_test_queue_destroy(cleanup_test_migration, migration);
    migration->tmpdir =
        g_dir_make_tmp("next-optical-migration-XXXXXX", &error);
    g_assert_no_error(error);
    g_assert_nonnull(migration->tmpdir);
    migration->socket_path =
        g_build_filename(migration->tmpdir, "migration.sock", NULL);
    return migration;
}

static void make_input(uint8_t input[NEXT_UNCODED_SIZE])
{
    size_t i;

    for (i = 0; i < NEXT_UNCODED_SIZE; i++) {
        input[i] = i;
    }
}

static void make_expected_frame(const uint8_t input[NEXT_UNCODED_SIZE],
                                uint8_t frame[NEXT_CODED_SIZE])
{
    memcpy(frame, input, NEXT_UNCODED_SIZE);
    memset(frame + NEXT_UNCODED_SIZE, 0,
           NEXT_CODED_SIZE - NEXT_UNCODED_SIZE);
}

static void apply_v66_mutations(uint8_t frame[NEXT_CODED_SIZE])
{
    size_t i;

    frame[1] = ~frame[1];
    frame[50] = frame[50] + 1;
    frame[100] = ~frame[100] - 100;
    frame[200] = frame[200] + 255;
    frame[777] = ~frame[777];
    frame[890] = frame[890] + 23;
    frame[1111] = frame[1110] + 39;
    frame[1290] = frame[1290] + 22;
    for (i = 533; i < 565; i++) {
        frame[i] = ((~(int)frame[i] % 47) + 37);
    }
}

static uint32_t dma_command(bool device_to_memory, uint32_t command)
{
    return command | (device_to_memory ? DMA_READ_CMD : 0);
}

static void configure_dma(QTestState *qts, uint32_t address, size_t length,
                          bool device_to_memory, bool use_next_init)
{
    qtest_writel(qts, NEXT_OPTICAL_DMA_CSR,
                 dma_command(device_to_memory, DMA_RESET | DMA_INITBUF));
    qtest_writel(qts, NEXT_OPTICAL_DMA_CSR,
                 dma_command(device_to_memory, 0));
    qtest_writel(qts, NEXT_OPTICAL_DMA_NEXT,
                 use_next_init ? address + 0x100 : address);
    if (use_next_init) {
        qtest_writel(qts, NEXT_OPTICAL_DMA_INIT, address);
    }
    qtest_writel(qts, NEXT_OPTICAL_DMA_LIMIT, address + length);
}

static void enable_dma(QTestState *qts, bool device_to_memory)
{
    qtest_writel(qts, NEXT_OPTICAL_DMA_CSR,
                 dma_command(device_to_memory, DMA_SETENABLE));
}

static void wait_for_formatter(QTestState *qts)
{
    unsigned i;

    for (i = 0; i < 1000; i++) {
        if ((qtest_readb(qts, NEXT_DISR) & OMD_ECC_DONE) &&
            (qtest_readl(qts, NEXT_OPTICAL_DMA_CSR) & DMA_COMPLETE)) {
            return;
        }
        qtest_clock_step(qts, 1);
    }
    g_error("timed out waiting for NeXT optical formatter");
}

static void expect_dma_success(QTestState *qts, uint32_t end)
{
    uint32_t csr = qtest_readl(qts, NEXT_OPTICAL_DMA_CSR);
    uint32_t intr = qtest_readl(qts, NEXT_INTR_STATUS);

    g_assert_cmphex(qtest_readl(qts, NEXT_OPTICAL_DMA_NEXT), ==, end);
    g_assert_cmphex(csr & (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_COMPLETE);
    g_assert_cmphex(intr & NEXT_OPTICAL_DMA_IRQ, ==,
                    NEXT_OPTICAL_DMA_IRQ);
    g_assert_cmphex(intr & NEXT_OPTICAL_DEVICE_IRQ, ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_DISR) &
                    (OMD_DATA_ERR | OMD_ECC_DONE |
                     OMD_ATTN | OMD_CMD_COMPL),
                    ==, OMD_ECC_DONE);
}

static void run_encode(QTestState *qts, const uint8_t *input,
                       bool command_before_enable, bool use_next_init)
{
    qtest_memwrite(qts, NEXT_INPUT, input, NEXT_UNCODED_SIZE);
    configure_dma(qts, NEXT_INPUT, NEXT_UNCODED_SIZE, false,
                  use_next_init);
    qtest_writeb(qts, NEXT_DISR, 0xfc);
    qtest_writeb(qts, NEXT_DIMR, 0);
    qtest_writeb(qts, NEXT_CONTROL2, OMD_ECC_SELECT);
    if (command_before_enable) {
        qtest_writeb(qts, NEXT_CONTROL1, OMD_ECC_WRITE);
    }
    enable_dma(qts, false);
    if (!command_before_enable) {
        qtest_writeb(qts, NEXT_CONTROL1, OMD_ECC_WRITE);
    }
    wait_for_formatter(qts);
    expect_dma_success(qts, NEXT_INPUT + NEXT_UNCODED_SIZE);
}

static void run_read_frame(QTestState *qts, uint8_t *frame)
{
    configure_dma(qts, NEXT_CODED, NEXT_CODED_SIZE, true, false);
    enable_dma(qts, true);
    qtest_writeb(qts, NEXT_DISR, 0xfc);
    qtest_writeb(qts, NEXT_CONTROL1, OMD_ECC_READ);
    wait_for_formatter(qts);
    expect_dma_success(qts, NEXT_CODED + NEXT_CODED_SIZE);
    qtest_memread(qts, NEXT_CODED, frame, NEXT_CODED_SIZE);
}

static void run_decode(QTestState *qts, const uint8_t *mutated)
{
    qtest_memwrite(qts, NEXT_MUTATED, mutated, NEXT_CODED_SIZE);
    configure_dma(qts, NEXT_MUTATED, NEXT_CODED_SIZE, false, false);
    enable_dma(qts, false);
    qtest_writeb(qts, NEXT_DISR, 0xfc);
    qtest_writeb(qts, NEXT_CONTROL2,
                 OMD_ECC_SELECT | OMD_ECC_DECODE);
    qtest_writeb(qts, NEXT_CONTROL1, OMD_ECC_WRITE);
    wait_for_formatter(qts);
    expect_dma_success(qts, NEXT_MUTATED + NEXT_CODED_SIZE);
    g_assert_cmphex(qtest_readb(qts, NEXT_DESR), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_ECCCNT), ==, 36);
}

static void run_read_result(QTestState *qts, uint8_t *result)
{
    configure_dma(qts, NEXT_RESULT, NEXT_UNCODED_SIZE, true, true);
    enable_dma(qts, true);
    qtest_writeb(qts, NEXT_DISR, 0xfc);
    qtest_writeb(qts, NEXT_CONTROL1, OMD_ECC_READ);
    wait_for_formatter(qts);
    expect_dma_success(qts, NEXT_RESULT + NEXT_UNCODED_SIZE);
    qtest_memread(qts, NEXT_RESULT, result, NEXT_UNCODED_SIZE);
}

static void test_machine_mapping(void)
{
    QTestState *cube = next_machine_start("next-cube", NULL);
    g_autofree char *cube_mtree = qtest_hmp(cube, "info mtree -f");
    QTestState *station;
    QTestState *color;
    g_autofree char *station_mtree = NULL;
    g_autofree char *color_mtree = NULL;

    g_assert_nonnull(strstr(cube_mtree, NEXT_OPTICAL_MTREE));
    qtest_quit(cube);

    station = next_machine_start("next-station", NULL);
    station_mtree = qtest_hmp(station, "info mtree -f");
    g_assert_null(strstr(station_mtree, "next-optical"));
    qtest_quit(station);

    color = next_machine_start("next-station-color", NULL);
    color_mtree = qtest_hmp(color, "info mtree -f");
    g_assert_null(strstr(color_mtree, "next-optical"));
    qtest_quit(color);
}

static void test_registers_and_reset(void)
{
    QTestState *qts = next_machine_start("next-cube", NULL);
    size_t i;

    for (i = 0; i < NEXT_OPTICAL_SIZE; i++) {
        g_assert_cmphex(qtest_readb(qts, NEXT_OPTICAL_BASE + i), ==, 0);
    }

    for (i = 0; i < 4; i++) {
        qtest_writeb(qts, NEXT_OPTICAL_BASE + i, 0x10 + i);
        g_assert_cmphex(qtest_readb(qts, NEXT_OPTICAL_BASE + i),
                        ==, 0x10 + i);
    }
    qtest_writeb(qts, NEXT_DIMR, 0xa5);
    qtest_writeb(qts, NEXT_CONTROL2, 0x5a);
    qtest_writeb(qts, NEXT_OPTICAL_BASE + 8, 0x12);
    qtest_writeb(qts, NEXT_OPTICAL_BASE + 9, 0x34);
    g_assert_cmphex(qtest_readb(qts, NEXT_DIMR), ==, 0xa5);
    g_assert_cmphex(qtest_readb(qts, NEXT_CONTROL2), ==, 0x5a);
    g_assert_cmphex(qtest_readb(qts, NEXT_OPTICAL_BASE + 8), ==, 0x12);
    g_assert_cmphex(qtest_readb(qts, NEXT_OPTICAL_BASE + 9), ==, 0x34);

    qtest_writeb(qts, NEXT_DESR, 0xff);
    qtest_writeb(qts, NEXT_ECCCNT, 0xff);
    for (i = 0x0c; i <= 0x0e; i++) {
        qtest_writeb(qts, NEXT_OPTICAL_BASE + i, 0xff);
        g_assert_cmphex(qtest_readb(qts, NEXT_OPTICAL_BASE + i), ==, 0);
    }
    g_assert_cmphex(qtest_readb(qts, NEXT_DESR), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_ECCCNT), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_OPTICAL_BASE + 0x0f), ==, 0);

    for (i = 0x10; i <= 0x16; i++) {
        qtest_writeb(qts, NEXT_OPTICAL_BASE + i, 0x80 + i);
        g_assert_cmphex(qtest_readb(qts, NEXT_OPTICAL_BASE + i),
                        ==, 0x80 + i);
    }

    qtest_writeb(qts, NEXT_DISR, 1);
    for (i = 0; i < NEXT_OPTICAL_SIZE; i++) {
        g_assert_cmphex(qtest_readb(qts, NEXT_OPTICAL_BASE + i), ==, 0);
    }

    qtest_quit(qts);
}

static void test_v66_sequence_both_orderings(void)
{
    uint8_t input[NEXT_UNCODED_SIZE];
    uint8_t expected[NEXT_CODED_SIZE];
    uint8_t frame[NEXT_CODED_SIZE];
    uint8_t result[NEXT_UNCODED_SIZE];
    unsigned ordering;

    make_input(input);
    make_expected_frame(input, expected);
    for (ordering = 0; ordering < 2; ordering++) {
        bool command_first = ordering;
        QTestState *qts = next_machine_start("next-cube", NULL);

        run_encode(qts, input, command_first, command_first);
        run_read_frame(qts, frame);
        g_assert_cmpmem(frame, sizeof(frame), expected, sizeof(expected));
        apply_v66_mutations(frame);
        run_decode(qts, frame);
        run_read_result(qts, result);
        g_assert_cmpmem(result, sizeof(result), input, sizeof(input));
        qtest_quit(qts);
    }
}

static void test_dma_error_is_atomic(void)
{
    uint8_t input[NEXT_UNCODED_SIZE];
    uint8_t canary[NEXT_CODED_SIZE];
    uint8_t actual[NEXT_CODED_SIZE];
    QTestState *qts = next_machine_start("next-cube", NULL);

    make_input(input);
    memset(canary, 0xa5, sizeof(canary));
    run_encode(qts, input, true, false);
    qtest_memwrite(qts, NEXT_CODED, canary, sizeof(canary));

    configure_dma(qts, NEXT_CODED, NEXT_CODED_SIZE - 1, true, false);
    enable_dma(qts, true);
    qtest_writeb(qts, NEXT_DISR, 0xfc);
    qtest_writeb(qts, NEXT_CONTROL1, OMD_ECC_READ);
    wait_for_formatter(qts);

    g_assert_cmphex(qtest_readl(qts, NEXT_OPTICAL_DMA_CSR) &
                    (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_COMPLETE | DMA_BUSEXC);
    g_assert_cmphex(qtest_readb(qts, NEXT_DESR), ==, OMD_ERR_ECC);
    g_assert_cmphex(qtest_readb(qts, NEXT_DISR) &
                    (OMD_DATA_ERR | OMD_ECC_DONE),
                    ==, OMD_DATA_ERR | OMD_ECC_DONE);
    qtest_memread(qts, NEXT_CODED, actual, sizeof(actual));
    g_assert_cmpmem(actual, sizeof(actual), canary, sizeof(canary));

    qtest_quit(qts);
}

static void test_wrong_mutation_is_rejected(void)
{
    uint8_t input[NEXT_UNCODED_SIZE];
    uint8_t frame[NEXT_CODED_SIZE];
    QTestState *qts = next_machine_start("next-cube", NULL);

    make_input(input);
    run_encode(qts, input, true, false);
    run_read_frame(qts, frame);
    apply_v66_mutations(frame);
    frame[42] ^= 1;
    qtest_memwrite(qts, NEXT_MUTATED, frame, sizeof(frame));
    configure_dma(qts, NEXT_MUTATED, sizeof(frame), false, false);
    enable_dma(qts, false);
    qtest_writeb(qts, NEXT_DISR, 0xfc);
    qtest_writeb(qts, NEXT_CONTROL2,
                 OMD_ECC_SELECT | OMD_ECC_DECODE);
    qtest_writeb(qts, NEXT_CONTROL1, OMD_ECC_WRITE);
    wait_for_formatter(qts);

    g_assert_cmphex(qtest_readb(qts, NEXT_DESR), ==, OMD_ERR_ECC);
    g_assert_cmphex(qtest_readb(qts, NEXT_ECCCNT), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_DISR) &
                    (OMD_DATA_ERR | OMD_ECC_DONE),
                    ==, OMD_DATA_ERR | OMD_ECC_DONE);

    qtest_quit(qts);
}

static void expect_dma_controller_error(QTestState *qts)
{
    wait_for_formatter(qts);
    g_assert_cmphex(qtest_readl(qts, NEXT_OPTICAL_DMA_CSR) &
                    (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_COMPLETE | DMA_BUSEXC);
    g_assert_cmphex(qtest_readb(qts, NEXT_DESR), ==, OMD_ERR_ECC);
    g_assert_cmphex(qtest_readb(qts, NEXT_ECCCNT), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_DISR) &
                    (OMD_DATA_ERR | OMD_ECC_DONE),
                    ==, OMD_DATA_ERR | OMD_ECC_DONE);
    g_assert_cmphex(qtest_readb(qts, NEXT_CONTROL1), ==, 0);
}

static void test_wrong_commands_terminate(void)
{
    uint8_t input[NEXT_UNCODED_SIZE];
    QTestState *qts;

    make_input(input);
    qts = next_machine_start("next-cube", NULL);
    qtest_memwrite(qts, NEXT_INPUT, input, sizeof(input));
    configure_dma(qts, NEXT_INPUT, sizeof(input), false, false);
    qtest_writeb(qts, NEXT_CONTROL2, OMD_ECC_SELECT);
    qtest_writeb(qts, NEXT_CONTROL1, 0x20);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, NEXT_OPTICAL_DMA_CSR) &
                    (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_DESR), ==, OMD_ERR_ECC);
    g_assert_cmphex(qtest_readb(qts, NEXT_DISR) &
                    (OMD_DATA_ERR | OMD_ECC_DONE),
                    ==, OMD_DATA_ERR | OMD_ECC_DONE);
    enable_dma(qts, false);
    expect_dma_controller_error(qts);
    qtest_quit(qts);

    qts = next_machine_start("next-cube", NULL);
    qtest_memwrite(qts, NEXT_INPUT, input, sizeof(input));
    configure_dma(qts, NEXT_INPUT, sizeof(input), false, false);
    qtest_writeb(qts, NEXT_CONTROL2, OMD_ECC_SELECT);
    enable_dma(qts, false);
    qtest_writeb(qts, NEXT_CONTROL1, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, NEXT_OPTICAL_DMA_CSR) &
                    (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_ENABLE);
    qtest_writeb(qts, NEXT_CONTROL1, 0x20);
    expect_dma_controller_error(qts);
    qtest_quit(qts);

    qts = next_machine_start("next-cube", NULL);
    run_encode(qts, input, true, false);
    configure_dma(qts, NEXT_CODED, NEXT_CODED_SIZE, true, false);
    enable_dma(qts, true);
    qtest_writeb(qts, NEXT_DISR, 0xfc);
    qtest_writeb(qts, NEXT_CONTROL1, OMD_ECC_WRITE);
    expect_dma_controller_error(qts);
    qtest_quit(qts);
}

static void test_reset_cancels_pending(void)
{
    uint8_t input[NEXT_UNCODED_SIZE];
    QTestState *qts;

    make_input(input);
    qts = next_machine_start("next-cube", NULL);
    qtest_memwrite(qts, NEXT_INPUT, input, sizeof(input));
    configure_dma(qts, NEXT_INPUT, sizeof(input), false, false);
    qtest_writeb(qts, NEXT_CONTROL2, OMD_ECC_SELECT);
    qtest_writeb(qts, NEXT_CONTROL1, OMD_ECC_WRITE);
    qtest_system_reset(qts);
    enable_dma(qts, false);
    qtest_clock_step(qts, 1);

    g_assert_cmphex(qtest_readl(qts, NEXT_OPTICAL_DMA_CSR) &
                    (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_ENABLE);
    g_assert_cmphex(qtest_readb(qts, NEXT_DISR), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_CONTROL1), ==, 0);

    qtest_quit(qts);

    qts = next_machine_start("next-cube", NULL);
    qtest_writeb(qts, NEXT_CONTROL2, OMD_ECC_SELECT);
    qtest_writeb(qts, NEXT_CONTROL1, 0x20);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readb(qts, NEXT_DESR), ==, OMD_ERR_ECC);
    qtest_system_reset(qts);
    configure_dma(qts, NEXT_INPUT, sizeof(input), false, false);
    enable_dma(qts, false);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(qtest_readl(qts, NEXT_OPTICAL_DMA_CSR) &
                    (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_ENABLE);
    g_assert_cmphex(qtest_readb(qts, NEXT_DISR), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_DESR), ==, 0);
    qtest_quit(qts);
}

static void test_migration_preserves_post_state(void)
{
    uint8_t input[NEXT_UNCODED_SIZE];
    uint8_t expected[NEXT_CODED_SIZE];
    uint8_t frame[NEXT_CODED_SIZE];
    uint8_t result[NEXT_UNCODED_SIZE];
    TestMigration *migration = create_test_migration();
    g_autofree char *uri =
        g_strdup_printf("unix:%s", migration->socket_path);
    QTestState *destination =
        next_machine_start("next-cube", "-incoming defer");
    QTestState *source = next_machine_start("next-cube", NULL);

    make_input(input);
    make_expected_frame(input, expected);
    run_encode(source, input, true, true);
    qtest_qmp_assert_success(
        destination,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);
    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    qtest_qmp_eventwait(source, "STOP");
    qtest_qmp_eventwait(destination, "RESUME");

    run_read_frame(destination, frame);
    g_assert_cmpmem(frame, sizeof(frame), expected, sizeof(expected));
    apply_v66_mutations(frame);
    run_decode(destination, frame);
    run_read_result(destination, result);
    g_assert_cmpmem(result, sizeof(result), input, sizeof(input));

    qtest_quit(source);
    qtest_quit(destination);
}

static void test_migration_preserves_pending_command(void)
{
    uint8_t input[NEXT_UNCODED_SIZE];
    uint8_t expected[NEXT_CODED_SIZE];
    uint8_t frame[NEXT_CODED_SIZE];
    TestMigration *migration = create_test_migration();
    g_autofree char *uri =
        g_strdup_printf("unix:%s", migration->socket_path);
    QTestState *destination =
        next_machine_start("next-cube", "-incoming defer");
    QTestState *source = next_machine_start("next-cube", NULL);

    make_input(input);
    make_expected_frame(input, expected);
    qtest_memwrite(source, NEXT_INPUT, input, sizeof(input));
    configure_dma(source, NEXT_INPUT, sizeof(input), false, false);
    qtest_writeb(source, NEXT_DISR, 0xfc);
    qtest_writeb(source, NEXT_CONTROL2, OMD_ECC_SELECT);
    qtest_writeb(source, NEXT_CONTROL1, OMD_ECC_WRITE);
    qtest_clock_step(source, 1);
    g_assert_cmphex(qtest_readl(source, NEXT_OPTICAL_DMA_CSR) &
                    (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC), ==, 0);

    qtest_qmp_assert_success(
        destination,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);
    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    qtest_qmp_eventwait(source, "STOP");
    qtest_qmp_eventwait(destination, "RESUME");

    enable_dma(destination, false);
    wait_for_formatter(destination);
    expect_dma_success(destination,
                       NEXT_INPUT + NEXT_UNCODED_SIZE);
    run_read_frame(destination, frame);
    g_assert_cmpmem(frame, sizeof(frame), expected, sizeof(expected));

    qtest_quit(source);
    qtest_quit(destination);
}

static void test_migration_preserves_pending_abort(void)
{
    uint8_t input[NEXT_UNCODED_SIZE];
    TestMigration *migration = create_test_migration();
    g_autofree char *uri =
        g_strdup_printf("unix:%s", migration->socket_path);
    QTestState *destination =
        next_machine_start("next-cube", "-incoming defer");
    QTestState *source = next_machine_start("next-cube", NULL);

    make_input(input);
    qtest_memwrite(source, NEXT_INPUT, input, sizeof(input));
    configure_dma(source, NEXT_INPUT, sizeof(input), false, false);
    qtest_writeb(source, NEXT_CONTROL2, OMD_ECC_SELECT);
    qtest_writeb(source, NEXT_CONTROL1, 0x20);
    qtest_clock_step(source, 1);
    g_assert_cmphex(qtest_readl(source, NEXT_OPTICAL_DMA_CSR) &
                    (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC), ==, 0);
    g_assert_cmphex(qtest_readb(source, NEXT_DESR), ==, OMD_ERR_ECC);

    qtest_qmp_assert_success(
        destination,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);
    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    qtest_qmp_eventwait(destination, "RESUME");
    qtest_qmp_eventwait(source, "STOP");

    g_assert_cmphex(qtest_readb(destination, NEXT_DESR), ==, OMD_ERR_ECC);
    g_assert_cmphex(qtest_readb(destination, NEXT_DISR) &
                    (OMD_DATA_ERR | OMD_ECC_DONE),
                    ==, OMD_DATA_ERR | OMD_ECC_DONE);
    enable_dma(destination, false);
    expect_dma_controller_error(destination);

    qtest_quit(source);
    qtest_quit(destination);
}

static void test_migration_preserves_decode_output_error(void)
{
    uint8_t input[NEXT_UNCODED_SIZE];
    uint8_t frame[NEXT_CODED_SIZE];
    uint8_t result[NEXT_UNCODED_SIZE];
    TestMigration *migration = create_test_migration();
    g_autofree char *uri =
        g_strdup_printf("unix:%s", migration->socket_path);
    QTestState *destination =
        next_machine_start("next-cube", "-incoming defer");
    QTestState *source = next_machine_start("next-cube", NULL);

    make_input(input);
    run_encode(source, input, true, false);
    run_read_frame(source, frame);
    apply_v66_mutations(frame);
    run_decode(source, frame);

    configure_dma(source, NEXT_RESULT, NEXT_UNCODED_SIZE - 1,
                  true, false);
    enable_dma(source, true);
    qtest_writeb(source, NEXT_DISR, 0xfc);
    qtest_writeb(source, NEXT_CONTROL1, OMD_ECC_READ);
    wait_for_formatter(source);
    g_assert_cmphex(qtest_readl(source, NEXT_OPTICAL_DMA_CSR) &
                    (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_COMPLETE | DMA_BUSEXC);
    g_assert_cmphex(qtest_readb(source, NEXT_DESR), ==, OMD_ERR_ECC);
    g_assert_cmphex(qtest_readb(source, NEXT_ECCCNT), ==, 0);
    g_assert_cmphex(qtest_readb(source, NEXT_DISR) &
                    (OMD_DATA_ERR | OMD_ECC_DONE),
                    ==, OMD_DATA_ERR | OMD_ECC_DONE);

    qtest_qmp_assert_success(
        destination,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);
    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    qtest_qmp_eventwait(destination, "RESUME");
    qtest_qmp_eventwait(source, "STOP");

    g_assert_cmphex(qtest_readl(destination, NEXT_OPTICAL_DMA_CSR) &
                    (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_COMPLETE | DMA_BUSEXC);
    g_assert_cmphex(qtest_readb(destination, NEXT_DESR), ==, OMD_ERR_ECC);
    g_assert_cmphex(qtest_readb(destination, NEXT_ECCCNT), ==, 0);
    g_assert_cmphex(qtest_readb(destination, NEXT_DISR) &
                    (OMD_DATA_ERR | OMD_ECC_DONE),
                    ==, OMD_DATA_ERR | OMD_ECC_DONE);

    qtest_system_reset(destination);
    g_assert_cmphex(qtest_readb(destination, NEXT_DISR), ==, 0);
    g_assert_cmphex(qtest_readb(destination, NEXT_DESR), ==, 0);
    g_assert_cmphex(qtest_readb(destination, NEXT_ECCCNT), ==, 0);
    run_encode(destination, input, true, false);
    run_read_frame(destination, frame);
    apply_v66_mutations(frame);
    run_decode(destination, frame);
    run_read_result(destination, result);
    g_assert_cmpmem(result, sizeof(result), input, sizeof(input));

    qtest_quit(source);
    qtest_quit(destination);
}

static void test_unrealize_cancels_pending_bh(void)
{
    QTestState *qts = next_machine_start("next-cube", NULL);

    qtest_writeb(qts, NEXT_CONTROL2, OMD_ECC_SELECT);
    qtest_writeb(qts, NEXT_CONTROL1, OMD_ECC_WRITE);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/next-optical/machine-mapping", test_machine_mapping);
    qtest_add_func("/next-optical/registers-and-reset",
                   test_registers_and_reset);
    qtest_add_func("/next-optical/v66-sequence-both-orderings",
                   test_v66_sequence_both_orderings);
    qtest_add_func("/next-optical/dma-error-is-atomic",
                   test_dma_error_is_atomic);
    qtest_add_func("/next-optical/wrong-mutation-is-rejected",
                   test_wrong_mutation_is_rejected);
    qtest_add_func("/next-optical/wrong-commands-terminate",
                   test_wrong_commands_terminate);
    qtest_add_func("/next-optical/reset-cancels-pending",
                   test_reset_cancels_pending);
    qtest_add_func("/next-optical/migration-preserves-post-state",
                   test_migration_preserves_post_state);
    qtest_add_func("/next-optical/migration-preserves-pending-command",
                   test_migration_preserves_pending_command);
    qtest_add_func("/next-optical/migration-preserves-pending-abort",
                   test_migration_preserves_pending_abort);
    qtest_add_func("/next-optical/migration-preserves-decode-output-error",
                   test_migration_preserves_decode_output_error);
    qtest_add_func("/next-optical/unrealize-cancels-pending-bh",
                   test_unrealize_cancels_pending_bh);

    return g_test_run();
}
