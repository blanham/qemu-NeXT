/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "libqtest.h"

#define NEXT_NBIC_BASE       0x02020000
#define NEXT_NBIC_CR         (NEXT_NBIC_BASE + 0)
#define NEXT_NBIC_IDR        (NEXT_NBIC_BASE + 4)

#define NEXT_NBIC_CR_IGNSID0 0x10000000
#define NEXT_NBIC_CR_STFWD   0x08000000
#define NEXT_NBIC_CR_RMCOL   0x04000000
#define NEXT_NBIC_CR_MASK    (NEXT_NBIC_CR_IGNSID0 | \
                              NEXT_NBIC_CR_STFWD | \
                              NEXT_NBIC_CR_RMCOL)
#define NEXT_NBIC_IDR_VALID  0x80000000
#define NEXT_NBIC_IDR_IDMASK 0x7fff0000
#define NEXT_NBIC_IDR_MASK   (NEXT_NBIC_IDR_VALID | NEXT_NBIC_IDR_IDMASK)

#define NEXT_ROM_SIZE        (128 * 1024)
#define NEXT_NBIC_MTREE      \
    "0000000002020000-0000000002020007 (prio 0, i/o): next-nbic"

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

    rom->fd = g_file_open_tmp("next-nbic-rom-XXXXXX", &rom->path, NULL);
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
    migration->tmpdir = g_dir_make_tmp("next-nbic-migration-XXXXXX",
                                        &error);
    g_assert_no_error(error);
    g_assert_nonnull(migration->tmpdir);
    migration->socket_path =
        g_build_filename(migration->tmpdir, "migration.sock", NULL);

    return migration;
}

static void test_machine_mapping(void)
{
    QTestState *cube = next_machine_start("next-cube", NULL);
    g_autofree char *cube_mtree = qtest_hmp(cube, "info mtree -f");
    QTestState *station;
    g_autofree char *station_mtree = NULL;

    g_assert_nonnull(strstr(cube_mtree, NEXT_NBIC_MTREE));
    qtest_quit(cube);

    station = next_machine_start("next-station", NULL);
    station_mtree = qtest_hmp(station, "info mtree -f");
    g_assert_null(strstr(station_mtree, NEXT_NBIC_MTREE));
    g_assert_null(strstr(station_mtree, "next-nbic"));
    qtest_quit(station);
}

static void test_register_reset_and_lanes(void)
{
    QTestState *qts = next_machine_start("next-cube", NULL);

    g_assert_cmphex(qtest_readl(qts, NEXT_NBIC_CR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_NBIC_IDR), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_NBIC_CR), ==, 0);

    qtest_writel(qts, NEXT_NBIC_CR, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, NEXT_NBIC_CR), ==,
                    NEXT_NBIC_CR_MASK);
    g_assert_cmphex(qtest_readb(qts, NEXT_NBIC_CR), ==, 0x1c);
    qtest_writel(qts, NEXT_NBIC_IDR, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, NEXT_NBIC_IDR), ==,
                    NEXT_NBIC_IDR_MASK);

    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, NEXT_NBIC_CR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_NBIC_IDR), ==, 0);

    qtest_writeb(qts, NEXT_NBIC_CR, 0x0c);
    g_assert_cmphex(qtest_readl(qts, NEXT_NBIC_CR), ==, 0x0c000000);
    qtest_writew(qts, NEXT_NBIC_CR, 0x1800);
    g_assert_cmphex(qtest_readl(qts, NEXT_NBIC_CR), ==, 0x18000000);

    qtest_writeb(qts, NEXT_NBIC_IDR, 0x92);
    qtest_writeb(qts, NEXT_NBIC_IDR + 1, 0x34);
    g_assert_cmphex(qtest_readl(qts, NEXT_NBIC_IDR), ==, 0x92340000);
    qtest_writew(qts, NEXT_NBIC_IDR, 0xabcd);
    g_assert_cmphex(qtest_readl(qts, NEXT_NBIC_IDR), ==, 0xabcd0000);
    g_assert_cmphex(qtest_readw(qts, NEXT_NBIC_IDR), ==, 0xabcd);

    qtest_quit(qts);
}

static void test_migration(void)
{
    TestMigration *migration = create_test_migration();
    g_autofree char *uri =
        g_strdup_printf("unix:%s", migration->socket_path);
    QTestState *destination =
        next_machine_start("next-cube", "-incoming defer");
    QTestState *source = next_machine_start("next-cube", NULL);

    qtest_qmp_assert_success(
        destination,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);

    qtest_writel(source, NEXT_NBIC_CR,
                 NEXT_NBIC_CR_STFWD | NEXT_NBIC_CR_RMCOL);
    qtest_writel(source, NEXT_NBIC_IDR,
                 NEXT_NBIC_IDR_VALID | 0x12340000);

    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    qtest_qmp_eventwait(source, "STOP");
    qtest_qmp_eventwait(destination, "RESUME");

    g_assert_cmphex(qtest_readl(destination, NEXT_NBIC_CR), ==, 0x0c000000);
    g_assert_cmphex(qtest_readl(destination, NEXT_NBIC_IDR), ==, 0x92340000);

    qtest_quit(source);
    qtest_quit(destination);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/next-nbic/machine-mapping", test_machine_mapping);
    qtest_add_func("/next-nbic/register-reset-and-lanes",
                   test_register_reset_and_lanes);
    qtest_add_func("/next-nbic/migration", test_migration);

    return g_test_run();
}
