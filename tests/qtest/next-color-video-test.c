/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "libqtest.h"

#define NEXT_COLOR_VRAM             0x2c000000
#define NEXT_COLOR_VRAM_SIZE        0x00200000
#define NEXT_COLOR_DAC              0x02118100
#define NEXT_COLOR_COMMAND          0x02118180
#define NEXT_COLOR_DRAM_TIMING      0x02118190
#define NEXT_COLOR_VRAM_TIMING      0x02118198
#define NEXT_INT_STATUS             0x02007000
#define NEXT_COLOR_VIDEO_IRQ_STATUS (1U << 13)
#define NEXT_MONO_VIDEO_IRQ_STATUS  (1U << 5)
#define NEXT_ROM_SIZE               (128 * KiB)

#define NEXT_COLOR_COMMAND_CLRINTR   0x01
#define NEXT_COLOR_COMMAND_INTRENA   0x02
#define NEXT_COLOR_COMMAND_UNBLANK   0x04
#define NEXT_COLOR_RETRACE_NS        (NANOSECONDS_PER_SECOND / 68)
#define NEXT_COLOR_WIDTH             1120
#define NEXT_COLOR_HEIGHT            832
#define NEXT_COLOR_STRIDE            2304
#define NEXT_COLOR_PPM_HEADER        "P6\n1120 832\n255\n"
#define NEXT_COLOR_PPM_HEADER_SIZE   (sizeof(NEXT_COLOR_PPM_HEADER) - 1)
#define NEXT_COLOR_PPM_RASTER_SIZE \
    ((size_t)NEXT_COLOR_WIDTH * NEXT_COLOR_HEIGHT * 3)

#define NEXT_COLOR_VRAM_MTREE \
    "000000002c000000-000000002c1fffff (prio 0, ram): next-color-vram"
#define NEXT_COLOR_DAC_MTREE \
    "0000000002118100-0000000002118103 (prio 0, i/o): next-color-dac"
#define NEXT_COLOR_COMMAND_MTREE \
    "0000000002118180-0000000002118180 (prio 0, i/o): next-color-command"
#define NEXT_COLOR_DRAM_TIMING_MTREE \
    "0000000002118190-0000000002118190 (prio 0, i/o): next-color-dram-timing"
#define NEXT_COLOR_VRAM_TIMING_MTREE \
    "0000000002118198-0000000002118198 (prio 0, i/o): next-color-vram-timing"
#define NEXT_MONO_VRAM_PREFIX "000000000b000000-"
#define NEXT_ALT_COLOR_VRAM_PREFIX "0000000006000000-"
#define NEXT_COLOR_REGION_MARKER ": next-color-"

typedef struct TestROM {
    int fd;
    char *path;
} TestROM;

typedef struct TestMigration {
    char *tmpdir;
    char *socket_path;
} TestMigration;

typedef struct TestPPM {
    char *tmpdir;
    char *path;
} TestPPM;

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

    rom->fd = g_file_open_tmp("next-color-video-rom-XXXXXX",
                              &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;

    return rom;
}

static QTestState *next_color_start_with_args(const char *args)
{
    TestROM *rom = create_test_rom();
    g_autofree char *quoted_rom_path = g_shell_quote(rom->path);

    return qtest_initf("-machine next-station-color -m 32M -bios %s %s",
                       quoted_rom_path, args ?: "");
}

static QTestState *next_color_start(void)
{
    return next_color_start_with_args(NULL);
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
        g_dir_make_tmp("next-color-video-migration-XXXXXX", &error);
    g_assert_no_error(error);
    g_assert_nonnull(migration->tmpdir);
    migration->socket_path =
        g_build_filename(migration->tmpdir, "migration.sock", NULL);

    return migration;
}

static void cleanup_test_ppm(void *opaque)
{
    TestPPM *ppm = opaque;

    qtest_remove_abrt_handler(ppm);
    if (ppm->path) {
        g_unlink(ppm->path);
    }
    if (ppm->tmpdir) {
        g_rmdir(ppm->tmpdir);
    }
    g_free(ppm->path);
    g_free(ppm->tmpdir);
    g_free(ppm);
}

static TestPPM *create_test_ppm(void)
{
    g_autoptr(GError) error = NULL;
    TestPPM *ppm = g_new0(TestPPM, 1);

    qtest_add_abrt_handler(cleanup_test_ppm, ppm);
    g_test_queue_destroy(cleanup_test_ppm, ppm);
    ppm->tmpdir = g_dir_make_tmp("next-color-video-ppm-XXXXXX", &error);
    g_assert_no_error(error);
    g_assert_nonnull(ppm->tmpdir);
    ppm->path = g_build_filename(ppm->tmpdir, "screendump.ppm", NULL);

    return ppm;
}

static bool qmp_command_available(QTestState *qts, const char *name)
{
    g_autoptr(QDict) response =
        qtest_qmp(qts, "{ 'execute': 'query-commands' }");
    QList *commands;
    QListEntry *entry;

    g_assert_nonnull(response);
    g_assert_true(qdict_haskey(response, "return"));
    commands = qdict_get_qlist(response, "return");
    g_assert_nonnull(commands);

    QLIST_FOREACH_ENTRY(commands, entry) {
        QDict *command = qobject_to(QDict, qlist_entry_obj(entry));

        g_assert_nonnull(command);
        if (!strcmp(qdict_get_str(command, "name"), name)) {
            return true;
        }
    }

    return false;
}

static bool require_screendump(QTestState *qts)
{
    if (qmp_command_available(qts, "screendump")) {
        return true;
    }

    g_test_skip("QMP screendump is unavailable");
    return false;
}

static void take_screendump(QTestState *qts, const TestPPM *ppm)
{
    qtest_qmp_assert_success(
        qts,
        "{ 'execute': 'screendump', 'arguments': { 'filename': %s } }",
        ppm->path);
}

static char *load_test_ppm(const TestPPM *ppm)
{
    g_autoptr(GError) error = NULL;
    g_autofree char *actual_header = NULL;
    char *contents = NULL;
    const char *cursor;
    const char *end;
    gsize length;

    g_assert_true(g_file_get_contents(ppm->path, &contents, &length, &error));
    g_assert_no_error(error);
    g_assert_cmpuint(length, >=, NEXT_COLOR_PPM_HEADER_SIZE);
    cursor = contents;
    end = contents + length;
    for (unsigned i = 0; i < 3; i++) {
        cursor = memchr(cursor, '\n', end - cursor);
        g_assert_nonnull(cursor);
        cursor++;
    }
    actual_header = g_strndup(contents, cursor - contents);
    g_assert_cmpstr(actual_header, ==, NEXT_COLOR_PPM_HEADER);
    g_assert_cmpuint(length, ==,
                     NEXT_COLOR_PPM_HEADER_SIZE +
                     NEXT_COLOR_PPM_RASTER_SIZE);

    return contents;
}

static void assert_screendump_pixel(QTestState *qts, const TestPPM *ppm,
                                    size_t pixel, const uint8_t expected[3])
{
    g_autofree char *contents = NULL;
    const uint8_t *rgb;

    take_screendump(qts, ppm);
    contents = load_test_ppm(ppm);
    rgb = (const uint8_t *)contents + NEXT_COLOR_PPM_HEADER_SIZE + pixel * 3;
    g_assert_cmpmem(rgb, 3, expected, 3);
}

static void dac_set_address(QTestState *qts, uint16_t address)
{
    qtest_writeb(qts, NEXT_COLOR_DAC, address & 0xff);
    qtest_writeb(qts, NEXT_COLOR_DAC + 1, address >> 8);
}

static void dac_write_triplet(QTestState *qts, unsigned port,
                              const uint8_t value[3])
{
    for (unsigned i = 0; i < 3; i++) {
        qtest_writeb(qts, NEXT_COLOR_DAC + port, value[i]);
    }
}

static void dac_read_triplet(QTestState *qts, unsigned port,
                             uint8_t value[3])
{
    for (unsigned i = 0; i < 3; i++) {
        value[i] = qtest_readb(qts, NEXT_COLOR_DAC + port);
    }
}

static void assert_dac_triplet(QTestState *qts, unsigned port,
                               uint16_t address,
                               const uint8_t expected[3])
{
    uint8_t actual[3];

    dac_set_address(qts, address);
    dac_read_triplet(qts, port, actual);
    for (unsigned i = 0; i < 3; i++) {
        g_assert_cmphex(actual[i], ==, expected[i]);
    }
}

static void test_bt463_mpu_registers(void)
{
    QTestState *qts = next_color_start();
    const uint8_t palette[3] = { 0x12, 0x34, 0x56 };
    const uint8_t cursor[3] = { 0xa1, 0xb2, 0xc3 };
    const uint8_t palette_next[3] = { 0x45, 0x67, 0x89 };
    const uint8_t wtt[3] = { 0xde, 0xad, 0xbe };
    const uint8_t zero[3] = { 0, 0, 0 };

    /* The address register exposes twelve bits and ignores ADDR12-15. */
    dac_set_address(qts, 0x0abc);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC), ==, 0xbc);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 1), ==, 0x0a);
    qtest_writeb(qts, NEXT_COLOR_DAC + 1, 0xff);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC), ==, 0xbc);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 1), ==, 0x0f);

    /* Palette RAM and general-register/cursor space are independent ports. */
    dac_set_address(qts, 0x100);
    dac_write_triplet(qts, 3, palette);
    dac_set_address(qts, 0x100);
    dac_write_triplet(qts, 2, cursor);
    assert_dac_triplet(qts, 3, 0x100, palette);
    assert_dac_triplet(qts, 2, 0x100, cursor);

    /* A complete RGB triplet advances one entry, not one component. */
    dac_set_address(qts, 0x010);
    dac_write_triplet(qts, 3, palette);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC), ==, 0x11);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 1), ==, 0x00);
    dac_write_triplet(qts, 3, palette_next);
    assert_dac_triplet(qts, 3, 0x010, palette);
    assert_dac_triplet(qts, 3, 0x011, palette_next);

    dac_set_address(qts, 0x30f);
    dac_write_triplet(qts, 2, wtt);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC), ==, 0x10);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 1), ==, 0x03);
    assert_dac_triplet(qts, 2, 0x30f, wtt);

    /* The twelve-bit address wraps from its final value to zero. */
    dac_set_address(qts, 0x0fff);
    qtest_writeb(qts, NEXT_COLOR_DAC + 3, 0x11);
    qtest_writeb(qts, NEXT_COLOR_DAC + 3, 0x22);
    qtest_writeb(qts, NEXT_COLOR_DAC + 3, 0x33);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC), ==, 0x00);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 1), ==, 0x00);

    /* Reading either address byte also restarts the component phase. */
    dac_set_address(qts, 0x012);
    qtest_writeb(qts, NEXT_COLOR_DAC + 3, 0xa1);
    qtest_readb(qts, NEXT_COLOR_DAC);
    qtest_writeb(qts, NEXT_COLOR_DAC + 3, 0xb2);
    assert_dac_triplet(qts, 3, 0x012,
                       (const uint8_t[3]) { 0xb2, 0x00, 0x00 });
    dac_set_address(qts, 0x013);
    qtest_writeb(qts, NEXT_COLOR_DAC + 3, 0xc3);
    qtest_readb(qts, NEXT_COLOR_DAC + 1);
    qtest_writeb(qts, NEXT_COLOR_DAC + 3, 0xd4);
    assert_dac_triplet(qts, 3, 0x013,
                       (const uint8_t[3]) { 0xd4, 0x00, 0x00 });

    /* Eight-bit registers advance after each access. */
    dac_set_address(qts, 0x201);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x40);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC), ==, 0x02);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 1), ==, 0x02);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x48);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x80);
    dac_set_address(qts, 0x201);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x40);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x48);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x80);

    dac_set_address(qts, 0x201);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0xff);
    dac_set_address(qts, 0x202);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0xff);
    dac_set_address(qts, 0x203);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0xff);
    dac_set_address(qts, 0x201);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0xcc);
    dac_set_address(qts, 0x202);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x7f);
    dac_set_address(qts, 0x203);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0xc7);

    /* Test and signature locations retain their architected access widths. */
    dac_set_address(qts, 0x20d);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0xa5);
    dac_set_address(qts, 0x20d);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0xa5);
    dac_set_address(qts, 0x20e);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x34);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x12);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0xff);
    dac_set_address(qts, 0x20e);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x34);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x12);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x00);
    dac_set_address(qts, 0x20f);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x11);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x22);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x33);
    dac_set_address(qts, 0x20f);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x11);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x22);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x33);

    /* ID and revision are read-only constants. */
    dac_set_address(qts, 0x200);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x2a);
    dac_set_address(qts, 0x200);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x00);
    dac_set_address(qts, 0x200);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x2a);
    dac_set_address(qts, 0x220);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0xb0);
    dac_set_address(qts, 0x220);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x00);
    dac_set_address(qts, 0x220);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0xb0);

    /* Invalid addresses read as zero and never alias valid storage. */
    dac_set_address(qts, 0x204);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x5a);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x00);
    dac_set_address(qts, 0x204);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x00);
    dac_set_address(qts, 0x400);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x6b);
    dac_set_address(qts, 0x000);
    assert_dac_triplet(qts, 2, 0x000, zero);
    dac_set_address(qts, 0x210);
    dac_write_triplet(qts, 3, wtt);
    assert_dac_triplet(qts, 3, 0x210, zero);

    qtest_quit(qts);
}

static uint32_t video_irq_status(QTestState *qts)
{
    return qtest_readl(qts, NEXT_INT_STATUS) &
        (NEXT_COLOR_VIDEO_IRQ_STATUS | NEXT_MONO_VIDEO_IRQ_STATUS);
}

static size_t count_occurrences(const char *text, const char *marker)
{
    size_t count = 0;
    size_t marker_len;

    g_assert_nonnull(text);
    g_assert_nonnull(marker);
    marker_len = strlen(marker);
    g_assert_cmpuint(marker_len, >, 0);

    while ((text = strstr(text, marker))) {
        count++;
        text += marker_len;
    }

    return count;
}

static void test_machine_mapping(void)
{
    QTestState *qts = next_color_start();
    g_autofree char *flatview = qtest_hmp(qts, "info mtree -f");

    g_assert_nonnull(strstr(flatview, NEXT_COLOR_VRAM_MTREE));
    g_assert_nonnull(strstr(flatview, NEXT_COLOR_DAC_MTREE));
    g_assert_nonnull(strstr(flatview, NEXT_COLOR_COMMAND_MTREE));
    g_assert_nonnull(strstr(flatview, NEXT_COLOR_DRAM_TIMING_MTREE));
    g_assert_nonnull(strstr(flatview, NEXT_COLOR_VRAM_TIMING_MTREE));
    g_assert_cmpuint(count_occurrences(flatview, NEXT_COLOR_REGION_MARKER),
                     ==, 5);
    g_assert_null(strstr(flatview, NEXT_MONO_VRAM_PREFIX));
    g_assert_null(strstr(flatview, NEXT_ALT_COLOR_VRAM_PREFIX));

    qtest_quit(qts);
}

static void test_vram_endpoints(void)
{
    QTestState *qts = next_color_start();
    const uint64_t last = NEXT_COLOR_VRAM + NEXT_COLOR_VRAM_SIZE - 1;

    qtest_writeb(qts, NEXT_COLOR_VRAM, 0x5a);
    qtest_writeb(qts, last, 0xa5);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_VRAM), ==, 0x5a);
    g_assert_cmphex(qtest_readb(qts, last), ==, 0xa5);

    qtest_quit(qts);
}

static void test_bt463_retained_registers(void)
{
    QTestState *qts = next_color_start();
    static const struct {
        uint16_t address;
        uint8_t value[3];
    } palettes[] = {
        { 0x000, { 0x12, 0x34, 0x56 } },
        { 0x100, { 0x78, 0x9a, 0xbc } },
        { 0x20f, { 0xde, 0xf0, 0x11 } },
    }, tags[] = {
        { 0x300, { 0xaa, 0xbb, 0xcc } },
        { 0x30f, { 0x33, 0x66, 0x99 } },
    };
    static const struct {
        uint16_t address;
        uint8_t value;
    } general_bytes[] = {
        { 0x201, 0x40 },
        { 0x202, 0x48 },
        { 0x203, 0xc0 },
        { 0x205, 0xff },
        { 0x206, 0xff },
        { 0x207, 0xff },
        { 0x208, 0xff },
        { 0x209, 0x00 },
        { 0x20a, 0x00 },
        { 0x20b, 0x00 },
        { 0x20c, 0x00 },
    };

    for (unsigned i = 0; i < G_N_ELEMENTS(general_bytes); i++) {
        if (general_bytes[i].value == 0) {
            dac_set_address(qts, general_bytes[i].address);
            qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x5a);
        }
    }
    for (unsigned i = 0; i < G_N_ELEMENTS(palettes); i++) {
        dac_set_address(qts, palettes[i].address);
        g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC), ==,
                        palettes[i].address & 0xff);
        g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 1), ==,
                        palettes[i].address >> 8);
        dac_write_triplet(qts, 3, palettes[i].value);
    }
    for (unsigned i = 0; i < G_N_ELEMENTS(general_bytes); i++) {
        dac_set_address(qts, general_bytes[i].address);
        qtest_writeb(qts, NEXT_COLOR_DAC + 2, general_bytes[i].value);
    }
    for (unsigned i = 0; i < G_N_ELEMENTS(tags); i++) {
        dac_set_address(qts, tags[i].address);
        dac_write_triplet(qts, 2, tags[i].value);
    }

    for (unsigned i = 0; i < G_N_ELEMENTS(palettes); i++) {
        assert_dac_triplet(qts, 3, palettes[i].address, palettes[i].value);
    }
    for (unsigned i = 0; i < G_N_ELEMENTS(general_bytes); i++) {
        dac_set_address(qts, general_bytes[i].address);
        g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==,
                        general_bytes[i].value);
    }
    for (unsigned i = 0; i < G_N_ELEMENTS(tags); i++) {
        assert_dac_triplet(qts, 2, tags[i].address, tags[i].value);
    }

    qtest_quit(qts);
}

static void test_bt463_auto_increment_and_phase_reset(void)
{
    QTestState *qts = next_color_start();
    const uint8_t palette0[3] = { 0x10, 0x20, 0x30 };
    const uint8_t palette1[3] = { 0x40, 0x50, 0x60 };
    const uint8_t tag0[3] = { 0x70, 0x80, 0x90 };
    const uint8_t tag1[3] = { 0xa0, 0xb0, 0xc0 };
    const uint8_t low_reset[3] = { 0x01, 0x02, 0x03 };
    const uint8_t high_reset[3] = { 0x04, 0x05, 0x06 };
    const uint8_t wrap[3] = { 0x12, 0x34, 0x56 };
    uint8_t actual[3];

    dac_set_address(qts, 0x010);
    dac_write_triplet(qts, 3, palette0);
    dac_write_triplet(qts, 3, palette1);
    dac_set_address(qts, 0x010);
    dac_read_triplet(qts, 3, actual);
    g_assert_cmpmem(actual, sizeof(actual), palette0, sizeof(palette0));
    dac_read_triplet(qts, 3, actual);
    g_assert_cmpmem(actual, sizeof(actual), palette1, sizeof(palette1));

    dac_set_address(qts, 0x30e);
    dac_write_triplet(qts, 2, tag0);
    dac_write_triplet(qts, 2, tag1);
    dac_set_address(qts, 0x30e);
    dac_read_triplet(qts, 2, actual);
    g_assert_cmpmem(actual, sizeof(actual), tag0, sizeof(tag0));
    dac_read_triplet(qts, 2, actual);
    g_assert_cmpmem(actual, sizeof(actual), tag1, sizeof(tag1));

    dac_set_address(qts, 0x2a5);
    qtest_writeb(qts, NEXT_COLOR_DAC + 3, 0xee);
    qtest_writeb(qts, NEXT_COLOR_DAC, 0x5a);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC), ==, 0x5a);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 1), ==, 0x02);
    dac_write_triplet(qts, 3, low_reset);
    assert_dac_triplet(qts, 3, 0x25a, (const uint8_t[3]) { 0, 0, 0 });

    dac_set_address(qts, 0x15a);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0xee);
    qtest_writeb(qts, NEXT_COLOR_DAC + 1, 0x03);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC), ==, 0x5b);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 1), ==, 0x03);
    dac_write_triplet(qts, 2, high_reset);
    assert_dac_triplet(qts, 2, 0x35b, (const uint8_t[3]) { 0, 0, 0 });

    dac_set_address(qts, 0x3ff);
    qtest_writeb(qts, NEXT_COLOR_DAC + 3, wrap[0]);
    qtest_writeb(qts, NEXT_COLOR_DAC + 3, wrap[1]);
    qtest_writeb(qts, NEXT_COLOR_DAC + 3, wrap[2]);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC), ==, 0x00);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 1), ==, 0x04);
    assert_dac_triplet(qts, 3, 0x3ff, (const uint8_t[3]) { 0, 0, 0 });

    qtest_quit(qts);
}

static void test_command_and_retrace_irq(void)
{
    QTestState *qts = next_color_start();
    const uint8_t enabled =
        NEXT_COLOR_COMMAND_INTRENA | NEXT_COLOR_COMMAND_UNBLANK;

    g_assert_cmphex(video_irq_status(qts), ==, 0);
    qtest_writeb(qts, NEXT_COLOR_COMMAND, 0xf8);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_COMMAND), ==, 0);
    qtest_clock_step(qts, NEXT_COLOR_RETRACE_NS);
    g_assert_cmphex(video_irq_status(qts), ==, 0);

    qtest_writeb(qts, NEXT_COLOR_COMMAND, enabled);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_COMMAND), ==, enabled);
    qtest_clock_step(qts, NEXT_COLOR_RETRACE_NS - 1);
    g_assert_cmphex(video_irq_status(qts), ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(video_irq_status(qts), ==,
                    NEXT_COLOR_VIDEO_IRQ_STATUS);

    qtest_writeb(qts, NEXT_COLOR_COMMAND,
                 NEXT_COLOR_COMMAND_CLRINTR |
                 NEXT_COLOR_COMMAND_UNBLANK);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_COMMAND), ==,
                    NEXT_COLOR_COMMAND_UNBLANK);
    g_assert_cmphex(video_irq_status(qts), ==, 0);
    qtest_clock_step(qts, 2 * NEXT_COLOR_RETRACE_NS);
    g_assert_cmphex(video_irq_status(qts), ==, 0);

    qtest_writeb(qts, NEXT_COLOR_COMMAND, enabled);
    qtest_clock_step(qts, NEXT_COLOR_RETRACE_NS);
    g_assert_cmphex(video_irq_status(qts), ==,
                    NEXT_COLOR_VIDEO_IRQ_STATUS);

    qtest_quit(qts);
}

static void test_retrace_ack_preserves_phase(void)
{
    QTestState *qts = next_color_start();
    const uint8_t enabled =
        NEXT_COLOR_COMMAND_INTRENA | NEXT_COLOR_COMMAND_UNBLANK;

    /* Two acknowledgements before the deadline must not defer retrace. */
    qtest_writeb(qts, NEXT_COLOR_COMMAND, enabled);
    qtest_clock_step(qts, NEXT_COLOR_RETRACE_NS / 2);
    qtest_writeb(qts, NEXT_COLOR_COMMAND,
                 NEXT_COLOR_COMMAND_CLRINTR |
                 NEXT_COLOR_COMMAND_UNBLANK);
    qtest_clock_step(qts, NEXT_COLOR_RETRACE_NS / 4);
    qtest_writeb(qts, NEXT_COLOR_COMMAND,
                 NEXT_COLOR_COMMAND_CLRINTR |
                 NEXT_COLOR_COMMAND_UNBLANK);
    qtest_writeb(qts, NEXT_COLOR_COMMAND, enabled);
    qtest_clock_step(qts, NEXT_COLOR_RETRACE_NS -
                     NEXT_COLOR_RETRACE_NS / 2 -
                     NEXT_COLOR_RETRACE_NS / 4);
    g_assert_cmphex(video_irq_status(qts), ==,
                    NEXT_COLOR_VIDEO_IRQ_STATUS);

    qtest_quit(qts);
}

static void test_registers_and_reset(void)
{
    QTestState *qts = next_color_start();
    const uint8_t palette[3] = { 0x12, 0x34, 0x56 };
    const uint8_t tag[3] = { 0xaa, 0xbb, 0xcc };
    const uint8_t reset_phase[3] = { 0x11, 0x22, 0x33 };
    const uint8_t zero[3] = { 0, 0, 0 };

    dac_set_address(qts, 0x100);
    dac_write_triplet(qts, 3, palette);
    dac_set_address(qts, 0x300);
    dac_write_triplet(qts, 2, tag);
    qtest_writeb(qts, NEXT_COLOR_DRAM_TIMING, 0x5a);
    qtest_writeb(qts, NEXT_COLOR_VRAM_TIMING, 0xa5);
    qtest_writeb(qts, NEXT_COLOR_COMMAND,
                 NEXT_COLOR_COMMAND_INTRENA |
                 NEXT_COLOR_COMMAND_UNBLANK);
    dac_set_address(qts, 0x222);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0xee);
    qtest_writeb(qts, NEXT_COLOR_VRAM, 0x5a);
    qtest_clock_step(qts, NEXT_COLOR_RETRACE_NS);
    g_assert_cmphex(video_irq_status(qts), ==,
                    NEXT_COLOR_VIDEO_IRQ_STATUS);

    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 1), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_COMMAND), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DRAM_TIMING), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_VRAM_TIMING), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_VRAM), ==, 0x5a);
    dac_set_address(qts, 0x200);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x2a);
    dac_set_address(qts, 0x201);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x00);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x00);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x00);
    for (uint16_t address = 0x205; address <= 0x20c; address++) {
        dac_set_address(qts, address);
        g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x00);
    }
    dac_set_address(qts, 0x220);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0xb0);
    assert_dac_triplet(qts, 3, 0x100, zero);
    assert_dac_triplet(qts, 2, 0x100, zero);
    assert_dac_triplet(qts, 2, 0x300, zero);
    assert_dac_triplet(qts, 2, 0x222, zero);
    dac_set_address(qts, 0x222);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0xee);
    qtest_system_reset(qts);
    dac_set_address(qts, 0x300);
    dac_write_triplet(qts, 2, reset_phase);
    assert_dac_triplet(qts, 2, 0x300, reset_phase);
    g_assert_cmphex(video_irq_status(qts), ==, 0);
    qtest_writeb(qts, NEXT_COLOR_COMMAND,
                 NEXT_COLOR_COMMAND_INTRENA |
                 NEXT_COLOR_COMMAND_UNBLANK);
    qtest_clock_step(qts, NEXT_COLOR_RETRACE_NS - 1);
    qtest_system_reset(qts);
    qtest_clock_step(qts, 2 * NEXT_COLOR_RETRACE_NS);
    g_assert_cmphex(video_irq_status(qts), ==, 0);

    qtest_quit(qts);
}

/*
 * The Warp9C's normal setup is the sequence used by vid_C16_init().  Keep
 * this fixture deliberately small: the palette values are chosen to make the
 * four-bit scanout assertions independent of the driver's gamma table.
 */
static void program_original_warp9c_init(QTestState *qts)
{
    static const uint8_t true_color[3] = { 0x00, 0x01, 0x00 };

    dac_set_address(qts, 0x201);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x40);
    dac_set_address(qts, 0x202);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x00);
    dac_set_address(qts, 0x203);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x80);

    for (uint16_t address = 0x205; address <= 0x208; address++) {
        dac_set_address(qts, address);
        qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0xf0);
    }
    for (uint16_t address = 0x209; address <= 0x20c; address++) {
        dac_set_address(qts, address);
        qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x00);
    }

    for (uint16_t address = 0x300; address <= 0x30f; address++) {
        dac_set_address(qts, address);
        dac_write_triplet(qts, 2, true_color);
    }

    /* Entries used by the four-bit high-nibble inputs. */
    dac_set_address(qts, 0x000);
    dac_write_triplet(qts, 3, (const uint8_t[3]) { 0x00, 0x00, 0x00 });
    dac_set_address(qts, 0x0f0);
    dac_write_triplet(qts, 3, (const uint8_t[3]) { 0xff, 0xff, 0xff });
}

static void program_blink_fixture(QTestState *qts, uint8_t command0)
{
    static const uint8_t red444[] = { 0xf0, 0x00 };

    program_original_warp9c_init(qts);
    dac_set_address(qts, 0x201);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, command0);
    dac_set_address(qts, 0x209);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0xf0);
    qtest_bufwrite(qts, NEXT_COLOR_VRAM, red444, sizeof(red444));
    qtest_writeb(qts, NEXT_COLOR_COMMAND, NEXT_COLOR_COMMAND_UNBLANK);
}

static void test_bt463_blink_rates(void)
{
    static const struct {
        uint8_t rate;
        unsigned on;
        unsigned off;
    } rates[] = {
        { 0x00, 16, 48 },
        { 0x04, 16, 16 },
        { 0x08, 32, 32 },
        { 0x0c, 64, 64 },
    };

    for (unsigned i = 0; i < G_N_ELEMENTS(rates); i++) {
        QTestState *qts = next_color_start();
        TestPPM *ppm;
        static const uint8_t red[] = { 0xff, 0x00, 0x00 };
        static const uint8_t black[] = { 0x00, 0x00, 0x00 };

        if (!require_screendump(qts)) {
            qtest_quit(qts);
            return;
        }
        ppm = create_test_ppm();
        program_blink_fixture(qts, 0x40 | rates[i].rate);

        assert_screendump_pixel(qts, ppm, 0, red);
        qtest_clock_step(qts, (rates[i].on - 1) * NEXT_COLOR_RETRACE_NS);
        assert_screendump_pixel(qts, ppm, 0, red);
        qtest_clock_step(qts, NEXT_COLOR_RETRACE_NS);
        assert_screendump_pixel(qts, ppm, 0, black);
        qtest_clock_step(qts, (rates[i].off - 1) * NEXT_COLOR_RETRACE_NS);
        assert_screendump_pixel(qts, ppm, 0, black);
        qtest_clock_step(qts, NEXT_COLOR_RETRACE_NS);
        assert_screendump_pixel(qts, ppm, 0, red);

        qtest_quit(qts);
    }
}

static void test_bt463_blink_command0_reset(void)
{
    QTestState *qts = next_color_start();
    TestPPM *ppm;
    static const uint8_t red[] = { 0xff, 0x00, 0x00 };
    static const uint8_t black[] = { 0x00, 0x00, 0x00 };

    if (!require_screendump(qts)) {
        qtest_quit(qts);
        return;
    }
    ppm = create_test_ppm();
    program_blink_fixture(qts, 0x44); /* 16 on, 16 off. */
    qtest_clock_step(qts, 16 * NEXT_COLOR_RETRACE_NS);
    assert_screendump_pixel(qts, ppm, 0, black);

    /* Rewriting CR0 resets both the counter and the visible phase. */
    dac_set_address(qts, 0x201);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x44);
    assert_screendump_pixel(qts, ppm, 0, red);
    qtest_clock_step(qts, 16 * NEXT_COLOR_RETRACE_NS);
    assert_screendump_pixel(qts, ppm, 0, black);

    qtest_quit(qts);
}

static void test_rgb444_scanout(void)
{
    QTestState *qts = next_color_start();
    TestPPM *ppm;
    static const uint8_t row0[] = {
        0xf0, 0x00,
        0x0f, 0x00,
        0x00, 0xf0,
        0x00, 0x00,
    };
    static const uint8_t white[] = { 0xff, 0xf0 };
    static const uint8_t expected_row0[] = {
        0xff, 0x00, 0x00,
        0x00, 0xff, 0x00,
        0x00, 0x00, 0xff,
        0x00, 0x00, 0x00,
    };
    static const uint8_t expected_white[] = { 0xff, 0xff, 0xff };
    g_autofree char *contents = NULL;
    const uint8_t *raster;

    if (!require_screendump(qts)) {
        qtest_quit(qts);
        return;
    }
    ppm = create_test_ppm();

    program_original_warp9c_init(qts);
    qtest_bufwrite(qts, NEXT_COLOR_VRAM, row0, sizeof(row0));
    qtest_bufwrite(qts, NEXT_COLOR_VRAM + NEXT_COLOR_STRIDE,
                   white, sizeof(white));
    qtest_writeb(qts, NEXT_COLOR_COMMAND, NEXT_COLOR_COMMAND_UNBLANK);

    take_screendump(qts, ppm);
    contents = load_test_ppm(ppm);
    raster = (const uint8_t *)contents + NEXT_COLOR_PPM_HEADER_SIZE;
    g_assert_cmpmem(raster, sizeof(expected_row0),
                    expected_row0, sizeof(expected_row0));
    g_assert_cmpmem(raster + NEXT_COLOR_WIDTH * 3, sizeof(expected_white),
                    expected_white, sizeof(expected_white));

    qtest_quit(qts);
}

static void test_bt463_lut_invalidation(void)
{
    QTestState *qts = next_color_start();
    TestPPM *before;
    TestPPM *after;
    static const uint8_t pixel[] = { 0x12, 0x30 };

    if (!require_screendump(qts)) {
        qtest_quit(qts);
        return;
    }
    before = create_test_ppm();
    after = create_test_ppm();

    program_original_warp9c_init(qts);
    dac_set_address(qts, 0x010);
    dac_write_triplet(qts, 3, (const uint8_t[3]) { 0x10, 0x00, 0x00 });
    dac_set_address(qts, 0x020);
    dac_write_triplet(qts, 3, (const uint8_t[3]) { 0x00, 0x20, 0x00 });
    dac_set_address(qts, 0x030);
    dac_write_triplet(qts, 3, (const uint8_t[3]) { 0x00, 0x00, 0x30 });
    qtest_bufwrite(qts, NEXT_COLOR_VRAM, pixel, sizeof(pixel));
    qtest_writeb(qts, NEXT_COLOR_COMMAND, NEXT_COLOR_COMMAND_UNBLANK);
    assert_screendump_pixel(qts, before, 0,
                            (const uint8_t[3]) { 0x10, 0x20, 0x30 });

    /* No VRAM write occurs between these two captures. */
    dac_set_address(qts, 0x010);
    dac_write_triplet(qts, 3, (const uint8_t[3]) { 0x90, 0x00, 0x00 });
    assert_screendump_pixel(qts, after, 0,
                            (const uint8_t[3]) { 0x90, 0x20, 0x30 });

    qtest_quit(qts);
}

static void test_bt463_wtt_tags(void)
{
    QTestState *qts = next_color_start();
    TestPPM *ppm;
    static const uint8_t pixels[] = {
        0x10, 0x00, /* window type 0 */
        0x10, 0x01, /* window type 1 */
    };

    if (!require_screendump(qts)) {
        qtest_quit(qts);
        return;
    }
    ppm = create_test_ppm();

    program_original_warp9c_init(qts);
    /* The second tag selects a color map beginning at the next 16-byte row. */
    dac_set_address(qts, 0x301);
    dac_write_triplet(qts, 2, (const uint8_t[3]) { 0x00, 0x01, 0x02 });
    dac_set_address(qts, 0x010);
    dac_write_triplet(qts, 3, (const uint8_t[3]) { 0xa1, 0x00, 0x00 });
    dac_set_address(qts, 0x020);
    dac_write_triplet(qts, 3, (const uint8_t[3]) { 0xb2, 0x00, 0x00 });
    qtest_bufwrite(qts, NEXT_COLOR_VRAM, pixels, sizeof(pixels));
    qtest_writeb(qts, NEXT_COLOR_COMMAND, NEXT_COLOR_COMMAND_UNBLANK);

    assert_screendump_pixel(qts, ppm, 0,
                            (const uint8_t[3]) { 0xa1, 0x00, 0x00 });
    assert_screendump_pixel(qts, ppm, 1,
                            (const uint8_t[3]) { 0xb2, 0x00, 0x00 });

    qtest_quit(qts);
}

static void test_bt463_load_interleave_scanout(void)
{
    QTestState *qts = next_color_start();
    TestPPM *ppm;
    static const uint8_t pixels[] = {
        0xff, 0xf0,
        0xff, 0xf1,
    };
    static const uint8_t lower[] = { 0x10, 0x20, 0x30 };
    static const uint8_t upper[] = { 0xa0, 0xb0, 0xc0 };
    static const uint8_t mode_shift0[] = {
        0x80, 0x08, 0x00, /* 12-plane true color, shift 0 */
    };
    static const uint8_t mode_shift4[] = {
        0x84, 0x08, 0x00, /* 12-plane true color, shift 4 */
    };

    if (!require_screendump(qts)) {
        qtest_quit(qts);
        return;
    }
    ppm = create_test_ppm();

    program_original_warp9c_init(qts);
    dac_set_address(qts, 0x000);
    dac_write_triplet(qts, 3, lower);
    dac_set_address(qts, 0x00f);
    dac_write_triplet(qts, 3, upper);
    dac_set_address(qts, 0x300);
    dac_write_triplet(qts, 2, mode_shift0);
    dac_set_address(qts, 0x301);
    dac_write_triplet(qts, 2, mode_shift4);
    qtest_bufwrite(qts, NEXT_COLOR_VRAM, pixels, sizeof(pixels));
    qtest_writeb(qts, NEXT_COLOR_COMMAND, NEXT_COLOR_COMMAND_UNBLANK);
    assert_screendump_pixel(qts, ppm, 0, lower);
    assert_screendump_pixel(qts, ppm, 1, lower);

    /* Changing only the WTT shift flips the initial phase; VRAM is reused. */
    dac_set_address(qts, 0x300);
    dac_write_triplet(qts, 2, mode_shift4);
    dac_set_address(qts, 0x301);
    dac_write_triplet(qts, 2, mode_shift0);
    assert_screendump_pixel(qts, ppm, 0, upper);
    assert_screendump_pixel(qts, ppm, 1, upper);

    qtest_quit(qts);
}

static void test_blanking(void)
{
    QTestState *qts = next_color_start();
    TestPPM *initial_blank;
    TestPPM *still_blank;
    TestPPM *unblanked;
    TestPPM *reblanked;
    TestPPM *unblanked_again;
    TestPPM *reset_blank;
    static const uint8_t red444[] = { 0xf0, 0x00 };
    static const uint8_t black[] = { 0x00, 0x00, 0x00 };
    static const uint8_t red[] = { 0xff, 0x00, 0x00 };

    if (!require_screendump(qts)) {
        qtest_quit(qts);
        return;
    }
    initial_blank = create_test_ppm();
    still_blank = create_test_ppm();
    unblanked = create_test_ppm();
    reblanked = create_test_ppm();
    unblanked_again = create_test_ppm();
    reset_blank = create_test_ppm();

    qtest_bufwrite(qts, NEXT_COLOR_VRAM, red444, sizeof(red444));
    program_original_warp9c_init(qts);
    assert_screendump_pixel(qts, initial_blank, 0, black);
    assert_screendump_pixel(qts, still_blank, 0, black);

    qtest_writeb(qts, NEXT_COLOR_COMMAND, NEXT_COLOR_COMMAND_UNBLANK);
    assert_screendump_pixel(qts, unblanked, 0, red);

    qtest_writeb(qts, NEXT_COLOR_COMMAND, 0);
    assert_screendump_pixel(qts, reblanked, 0, black);

    qtest_writeb(qts, NEXT_COLOR_COMMAND, NEXT_COLOR_COMMAND_UNBLANK);
    assert_screendump_pixel(qts, unblanked_again, 0, red);

    qtest_system_reset(qts);
    assert_screendump_pixel(qts, reset_blank, 0, black);

    qtest_quit(qts);
}

static void test_migration(void)
{
    TestMigration *migration = create_test_migration();
    g_autofree char *uri =
        g_strdup_printf("unix:%s", migration->socket_path);
    QTestState *destination =
        next_color_start_with_args("-incoming defer");
    QTestState *source = next_color_start();
    const uint8_t palette[3] = { 0x12, 0x34, 0x56 };
    const uint8_t tag[3] = { 0xaa, 0xbb, 0xcc };
    const uint8_t cursor0[3] = { 0xc1, 0xc2, 0xc3 };
    const uint8_t cursor1[3] = { 0xd1, 0xd2, 0xd3 };
    const uint8_t enabled =
        NEXT_COLOR_COMMAND_INTRENA | NEXT_COLOR_COMMAND_UNBLANK;
    int64_t source_clock;

    qtest_qmp_assert_success(
        destination,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);

    dac_set_address(source, 0x100);
    dac_write_triplet(source, 3, palette);
    dac_set_address(source, 0x300);
    dac_write_triplet(source, 2, tag);
    dac_set_address(source, 0x100);
    dac_write_triplet(source, 2, cursor0);
    dac_set_address(source, 0x101);
    dac_write_triplet(source, 2, cursor1);
    dac_set_address(source, 0x201);
    qtest_writeb(source, NEXT_COLOR_DAC + 2, 0x44);
    dac_set_address(source, 0x202);
    qtest_writeb(source, NEXT_COLOR_DAC + 2, 0x02);
    for (uint16_t address = 0x205; address <= 0x20c; address++) {
        dac_set_address(source, address);
        qtest_writeb(source, NEXT_COLOR_DAC + 2, address & 0xff);
    }
    qtest_writeb(source, NEXT_COLOR_DRAM_TIMING, 0x5a);
    qtest_writeb(source, NEXT_COLOR_VRAM_TIMING, 0xa5);
    qtest_writeb(source, NEXT_COLOR_COMMAND, enabled);
    source_clock = qtest_clock_step(source, NEXT_COLOR_RETRACE_NS);
    g_assert_cmphex(video_irq_status(source), ==,
                    NEXT_COLOR_VIDEO_IRQ_STATUS);

    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    qtest_qmp_eventwait(source, "STOP");
    qtest_qmp_eventwait(destination, "RESUME");
    qtest_clock_set(destination, source_clock);

    assert_dac_triplet(destination, 3, 0x100, palette);
    assert_dac_triplet(destination, 2, 0x300, tag);
    assert_dac_triplet(destination, 2, 0x100, cursor0);
    assert_dac_triplet(destination, 2, 0x101, cursor1);
    dac_set_address(destination, 0x201);
    g_assert_cmphex(qtest_readb(destination, NEXT_COLOR_DAC + 2), ==, 0x44);
    g_assert_cmphex(qtest_readb(destination, NEXT_COLOR_DAC + 2), ==, 0x02);
    for (uint16_t address = 0x205; address <= 0x20c; address++) {
        dac_set_address(destination, address);
        g_assert_cmphex(qtest_readb(destination, NEXT_COLOR_DAC + 2), ==,
                        address & 0xff);
    }
    g_assert_cmphex(qtest_readb(destination, NEXT_COLOR_DRAM_TIMING), ==,
                    0x5a);
    g_assert_cmphex(qtest_readb(destination, NEXT_COLOR_VRAM_TIMING), ==,
                    0xa5);
    g_assert_cmphex(qtest_readb(destination, NEXT_COLOR_COMMAND), ==,
                    enabled);
    g_assert_cmphex(video_irq_status(destination), ==,
                    NEXT_COLOR_VIDEO_IRQ_STATUS);

    qtest_writeb(destination, NEXT_COLOR_COMMAND,
                 NEXT_COLOR_COMMAND_CLRINTR |
                 NEXT_COLOR_COMMAND_UNBLANK);
    g_assert_cmphex(video_irq_status(destination), ==, 0);
    qtest_writeb(destination, NEXT_COLOR_COMMAND, enabled);
    qtest_clock_step(destination, NEXT_COLOR_RETRACE_NS - 1);
    g_assert_cmphex(video_irq_status(destination), ==, 0);
    qtest_clock_step(destination, 1);
    g_assert_cmphex(video_irq_status(destination), ==,
                    NEXT_COLOR_VIDEO_IRQ_STATUS);

    qtest_quit(source);
    qtest_quit(destination);
}

static void test_migration_active_timer_and_dac_phase(void)
{
    TestMigration *migration = create_test_migration();
    g_autofree char *uri =
        g_strdup_printf("unix:%s", migration->socket_path);
    QTestState *destination =
        next_color_start_with_args("-incoming defer");
    QTestState *source = next_color_start();
    const uint8_t palette[3] = { 0x12, 0x34, 0x56 };
    const int64_t elapsed = NEXT_COLOR_RETRACE_NS / 2;
    const int64_t remaining = NEXT_COLOR_RETRACE_NS - elapsed;
    int64_t source_clock;

    qtest_qmp_assert_success(
        destination,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);

    dac_set_address(source, 0x100);
    qtest_writeb(source, NEXT_COLOR_DAC + 3, palette[0]);
    qtest_writeb(source, NEXT_COLOR_COMMAND,
                 NEXT_COLOR_COMMAND_INTRENA |
                 NEXT_COLOR_COMMAND_UNBLANK);
    source_clock = qtest_clock_step(source, elapsed);
    g_assert_cmphex(video_irq_status(source), ==, 0);

    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    qtest_qmp_eventwait(source, "STOP");
    qtest_qmp_eventwait(destination, "RESUME");
    qtest_clock_set(destination, source_clock);

    qtest_writeb(destination, NEXT_COLOR_DAC + 3, palette[1]);
    qtest_writeb(destination, NEXT_COLOR_DAC + 3, palette[2]);
    assert_dac_triplet(destination, 3, 0x100, palette);
    qtest_clock_step(destination, remaining - 1);
    g_assert_cmphex(video_irq_status(destination), ==, 0);
    qtest_clock_step(destination, 1);
    g_assert_cmphex(video_irq_status(destination), ==,
                    NEXT_COLOR_VIDEO_IRQ_STATUS);

    qtest_quit(source);
    qtest_quit(destination);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/next-color-video/machine-mapping",
                   test_machine_mapping);
    qtest_add_func("/next-color-video/vram-endpoints", test_vram_endpoints);
    qtest_add_func("/next-color-video/bt463-mpu-registers",
                   test_bt463_mpu_registers);
    qtest_add_func("/next-color-video/bt463-retained-registers",
                   test_bt463_retained_registers);
    qtest_add_func("/next-color-video/bt463-auto-increment-and-phase-reset",
                   test_bt463_auto_increment_and_phase_reset);
    qtest_add_func("/next-color-video/command-and-retrace-irq",
                   test_command_and_retrace_irq);
    qtest_add_func("/next-color-video/retrace-ack-preserves-phase",
                   test_retrace_ack_preserves_phase);
    qtest_add_func("/next-color-video/registers-and-reset",
                   test_registers_and_reset);
    qtest_add_func("/next-color-video/migration", test_migration);
    qtest_add_func("/next-color-video/migration-active-timer-and-dac-phase",
                   test_migration_active_timer_and_dac_phase);
    qtest_add_func("/next-color-video/rgb444-scanout", test_rgb444_scanout);
    qtest_add_func("/next-color-video/bt463-lut-invalidation",
                   test_bt463_lut_invalidation);
    qtest_add_func("/next-color-video/bt463-wtt-tags", test_bt463_wtt_tags);
    qtest_add_func("/next-color-video/bt463-load-interleave",
                   test_bt463_load_interleave_scanout);
    qtest_add_func("/next-color-video/bt463-blink-rates",
                   test_bt463_blink_rates);
    qtest_add_func("/next-color-video/bt463-blink-command0-reset",
                   test_bt463_blink_command0_reset);
    qtest_add_func("/next-color-video/blanking", test_blanking);

    return g_test_run();
}
