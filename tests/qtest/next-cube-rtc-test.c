/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/timer.h"

#define NEXT_SCR2          0x0200d000
#define NEXT_SCR2_RTCE     0x00000100
#define NEXT_SCR2_RTCLK    0x00000200
#define NEXT_SCR2_RTDATA   0x00000400
#define NEXT_ROM_SIZE      (128 * 1024)
#define NEXT_RTC_START     0x80
#define NEXT_RTC_NEW_CLOCK 0x80

typedef struct TestROM {
    int fd;
    char *path;
} TestROM;

typedef struct TestMigrationFiles {
    char *tmpdir;
    char *ephemeral_socket;
    char *backed_socket;
    char *destination_nvram;
} TestMigrationFiles;

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

static void cleanup_test_migration_files(void *opaque)
{
    TestMigrationFiles *files = opaque;

    qtest_remove_abrt_handler(files);
    if (files->ephemeral_socket) {
        g_unlink(files->ephemeral_socket);
    }
    if (files->backed_socket) {
        g_unlink(files->backed_socket);
    }
    if (files->destination_nvram) {
        g_unlink(files->destination_nvram);
    }
    if (files->tmpdir) {
        g_rmdir(files->tmpdir);
    }
    g_free(files->ephemeral_socket);
    g_free(files->backed_socket);
    g_free(files->destination_nvram);
    g_free(files->tmpdir);
    g_free(files);
}

static QTestState *next_cube_rtc_start_full(const char *machine_options,
                                            const char *args)
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
    qts = qtest_initf("-machine next-cube%s -bios %s %s",
                      machine_options ?: "", quoted_rom_path, args ?: "");
    return qts;
}

static QTestState *next_cube_rtc_start_with_args(const char *args)
{
    return next_cube_rtc_start_full(NULL, args);
}

static QTestState *next_cube_rtc_start(void)
{
    return next_cube_rtc_start_with_args(NULL);
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

static bool rtc_receive_bit(QTestState *qts, uint32_t scr2)
{
    scr2 &= ~NEXT_SCR2_RTDATA;
    qtest_writel(qts, NEXT_SCR2, scr2 | NEXT_SCR2_RTCLK);
    qtest_writel(qts, NEXT_SCR2, scr2);
    return qtest_readl(qts, NEXT_SCR2) & NEXT_SCR2_RTDATA;
}

static uint8_t rtc_receive_byte(QTestState *qts, uint32_t scr2)
{
    uint8_t value = 0;
    int bit;

    for (bit = 0; bit < 8; bit++) {
        value = (value << 1) | rtc_receive_bit(qts, scr2);
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

static void migrate_wait(QTestState *source, QTestState *destination,
                         const char *uri)
{
    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    qtest_qmp_eventwait(source, "STOP");
    qtest_qmp_eventwait(destination, "RESUME");
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

static void test_nvram_survives_system_reset(void)
{
    static const uint8_t replacement[32] = {
        0x1b, 0xad, 0xc0, 0xde, 0x10, 0x20, 0x30, 0x40,
        0x50, 0x60, 0x70, 0x80, 0x90, 0xa0, 0xb0, 0xc0,
        0xd0, 0xe0, 0xf0, 0x0f, 0x1e, 0x2d, 0x3c, 0x4b,
        0x5a, 0x69, 0x78, 0x87, 0x96, 0xa5, 0xbc, 0xcd,
    };
    QTestState *qts = next_cube_rtc_start();
    uint8_t actual[32];
    uint32_t scr2;
    bool data_out_low;
    size_t i;

    rtc_block_write(qts, 0x80, replacement, sizeof(replacement));

    scr2 = rtc_begin(qts);
    rtc_send_byte(qts, scr2, 0x01);
    g_assert_true(rtc_receive_bit(qts, scr2));

    qtest_system_reset(qts);
    data_out_low = !(qtest_readl(qts, NEXT_SCR2) & NEXT_SCR2_RTDATA);

    /*
     * Start a new transaction without pulsing command reset.  The system
     * reset itself must have discarded the interrupted transaction.
     */
    scr2 = qtest_readl(qts, NEXT_SCR2);
    scr2 &= ~(NEXT_SCR2_RTCLK | NEXT_SCR2_RTDATA);
    scr2 |= NEXT_SCR2_RTCE;
    qtest_writel(qts, NEXT_SCR2, scr2);
    rtc_send_byte(qts, scr2, 0x00);
    for (i = 0; i < sizeof(actual); i++) {
        actual[i] = rtc_receive_byte(qts, scr2);
    }
    rtc_end(qts, scr2);

    g_assert_cmpmem(actual, sizeof(actual),
                    replacement, sizeof(replacement));
    g_assert_true(data_out_low);

    qtest_quit(qts);
}

static void test_nvram_file_relaunch(void)
{
    static const uint8_t replacement[32] = {
        0xf1, 0xe2, 0xd3, 0xc4, 0xb5, 0xa6, 0x97, 0x88,
        0x79, 0x6a, 0x5b, 0x4c, 0x3d, 0x2e, 0x1f, 0x00,
        0x0f, 0x1e, 0x2d, 0x3c, 0x4b, 0x5a, 0x69, 0x78,
        0x87, 0x96, 0xa5, 0xb4, 0xc3, 0xd2, 0xe1, 0xf0,
    };
    g_autofree char *tmpdir = NULL;
    g_autofree char *path = NULL;
    g_autofree char *quoted_path = NULL;
    g_autofree char *machine_options = NULL;
    g_autofree char *contents = NULL;
    g_autoptr(GError) err = NULL;
    gsize length;
    uint8_t actual[32];
    QTestState *qts;

    tmpdir = g_dir_make_tmp("next-nvram-qtest-XXXXXX", &err);
    g_assert_no_error(err);
    g_assert_nonnull(tmpdir);
    path = g_build_filename(tmpdir, "next.nvram", NULL);
    quoted_path = g_shell_quote(path);
    machine_options = g_strdup_printf(",nvram-file=%s", quoted_path);

    qts = next_cube_rtc_start_full(machine_options, NULL);
    rtc_block_write(qts, 0x80, replacement, sizeof(replacement));
    qtest_quit(qts);

    g_assert_true(g_file_get_contents(path, &contents, &length, &err));
    g_assert_no_error(err);
    g_assert_cmpuint(length, ==, sizeof(replacement));
    g_assert_cmpmem(contents, length, replacement, sizeof(replacement));
    g_clear_pointer(&contents, g_free);

    qts = next_cube_rtc_start_full(machine_options, NULL);
    rtc_block_read(qts, 0x00, actual, sizeof(actual));
    g_assert_cmpmem(actual, sizeof(actual),
                    replacement, sizeof(replacement));
    qtest_quit(qts);

    g_unlink(path);
    g_rmdir(tmpdir);
}

static void migrate_nvram_to_destination(const uint8_t replacement[32],
                                         const char *socket_path,
                                         const char *destination_nvram)
{
    g_autofree char *uri = g_strdup_printf("unix:%s", socket_path);
    g_autofree char *quoted_uri = g_shell_quote(uri);
    g_autofree char *incoming_args =
        g_strdup_printf("-incoming %s", quoted_uri);
    g_autofree char *quoted_nvram = destination_nvram ?
        g_shell_quote(destination_nvram) : NULL;
    g_autofree char *machine_options = destination_nvram ?
        g_strdup_printf(",nvram-file=%s", quoted_nvram) : NULL;
    QTestState *destination =
        next_cube_rtc_start_full(machine_options, incoming_args);
    QTestState *source = next_cube_rtc_start();
    g_autoptr(GError) err = NULL;
    g_autofree char *contents = NULL;
    gsize length;
    uint8_t actual[32];

    if (destination_nvram) {
        g_assert_true(g_file_get_contents(destination_nvram, &contents,
                                         &length, &err));
        g_assert_no_error(err);
        g_assert_cmpuint(length, ==, sizeof(initial_nvram));
        g_assert_cmpmem(contents, length,
                        initial_nvram, sizeof(initial_nvram));
    }

    rtc_block_write(source, 0x80, replacement, 32);
    migrate_wait(source, destination, uri);

    rtc_block_read(destination, 0x00, actual, sizeof(actual));
    g_assert_cmpmem(actual, sizeof(actual), replacement, 32);

    if (destination_nvram) {
        g_clear_pointer(&contents, g_free);
        g_assert_true(g_file_get_contents(destination_nvram, &contents,
                                         &length, &err));
        g_assert_no_error(err);
        g_assert_cmpuint(length, ==, 32);
        g_assert_cmpmem(contents, length, replacement, 32);
    }

    qtest_quit(source);
    qtest_quit(destination);
}

static void test_nvram_migration(void)
{
    static const uint8_t replacement[32] = {
        0x5a, 0xc3, 0x19, 0xe7, 0x84, 0x2d, 0xb6, 0x40,
        0xfe, 0x73, 0x08, 0x91, 0x4c, 0xd5, 0x2a, 0xbf,
        0x61, 0x0d, 0xf8, 0x34, 0xa7, 0x52, 0xcb, 0x16,
        0x89, 0xe0, 0x47, 0xbc, 0x25, 0x9e, 0x73, 0x0a,
    };
    g_autoptr(GError) err = NULL;
    TestMigrationFiles *files = g_new0(TestMigrationFiles, 1);

    qtest_add_abrt_handler(cleanup_test_migration_files, files);
    g_test_queue_destroy(cleanup_test_migration_files, files);
    files->tmpdir = g_dir_make_tmp("next-nvram-migration-XXXXXX", &err);
    g_assert_no_error(err);
    g_assert_nonnull(files->tmpdir);
    files->ephemeral_socket =
        g_build_filename(files->tmpdir, "ephemeral.sock", NULL);
    files->backed_socket =
        g_build_filename(files->tmpdir, "backed.sock", NULL);
    files->destination_nvram =
        g_build_filename(files->tmpdir, "destination.nvram", NULL);

    migrate_nvram_to_destination(replacement, files->ephemeral_socket, NULL);
    g_unlink(files->ephemeral_socket);

    migrate_nvram_to_destination(replacement, files->backed_socket,
                                 files->destination_nvram);
    g_unlink(files->backed_socket);
}

static uint32_t rtc_read_counter(QTestState *qts)
{
    uint8_t bytes[4];

    rtc_block_read(qts, 0x20, bytes, sizeof(bytes));
    return ((uint32_t)bytes[0] << 24) |
           ((uint32_t)bytes[1] << 16) |
           ((uint32_t)bytes[2] << 8) |
           bytes[3];
}

static uint32_t rtc_read_counter_with_step(QTestState *qts, int64_t step)
{
    uint32_t scr2 = rtc_begin(qts);
    uint8_t bytes[4];
    int i;

    rtc_send_byte(qts, scr2, 0x20);
    bytes[0] = rtc_receive_byte(qts, scr2);
    qtest_clock_step(qts, step);
    for (i = 1; i < 4; i++) {
        bytes[i] = rtc_receive_byte(qts, scr2);
    }
    rtc_end(qts, scr2);

    return ((uint32_t)bytes[0] << 24) |
           ((uint32_t)bytes[1] << 16) |
           ((uint32_t)bytes[2] << 8) |
           bytes[3];
}

static void test_mcs1850_counter(void)
{
    static const uint8_t replacement[4] = { 0x12, 0x34, 0x56, 0x78 };
    QTestState *qts = next_cube_rtc_start_with_args(
        "-rtc base=2000-01-02T03:04:05,clock=vm");
    uint8_t value;

    rtc_block_read(qts, 0x30, &value, 1);
    g_assert_cmphex(value, ==, NEXT_RTC_NEW_CLOCK);
    rtc_block_read(qts, 0x31, &value, 1);
    g_assert_cmphex(value, ==, NEXT_RTC_START);

    g_assert_cmphex(rtc_read_counter(qts), ==, 946782245);
    g_assert_cmphex(
        rtc_read_counter_with_step(qts, 2 * NANOSECONDS_PER_SECOND),
        ==, 946782245);
    qtest_clock_step(qts, 2 * NANOSECONDS_PER_SECOND);
    g_assert_cmphex(rtc_read_counter(qts), ==, 946782249);

    value = 0;
    rtc_block_write(qts, 0xb1, &value, 1);
    rtc_block_write(qts, 0xa0, replacement, sizeof(replacement));
    qtest_clock_step(qts, 2 * NANOSECONDS_PER_SECOND);
    g_assert_cmphex(rtc_read_counter(qts), ==, 0x12345678);

    value = NEXT_RTC_START;
    rtc_block_write(qts, 0xb1, &value, 1);
    qtest_clock_step(qts, 3 * NANOSECONDS_PER_SECOND);
    g_assert_cmphex(rtc_read_counter(qts), ==, 0x1234567b);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/next-cube/rtc/nvram-block-transfer",
                   test_nvram_block_transfer);
    qtest_add_func("/next-cube/rtc/nvram-survives-system-reset",
                   test_nvram_survives_system_reset);
    qtest_add_func("/next-cube/rtc/nvram-file-relaunch",
                   test_nvram_file_relaunch);
    qtest_add_func("/next-cube/rtc/migration",
                   test_nvram_migration);
    qtest_add_func("/next-cube/rtc/mcs1850-counter",
                   test_mcs1850_counter);
    return g_test_run();
}
