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

static QTestState *next_color_start_with_env(const char *binary_env,
                                             const char *args)
{
    TestROM *rom = create_test_rom();
    g_autofree char *quoted_rom_path = g_shell_quote(rom->path);
    g_autofree char *qemu_args = g_strdup_printf(
        "-machine next-station-color -m 32M -bios %s %s",
        quoted_rom_path, args ?: "");

    return qtest_init_ext(binary_env, qemu_args, NULL, true);
}

static QTestState *next_color_start_with_args(const char *args)
{
    return next_color_start_with_env(NULL, args);
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

static void test_bt463_complete_reset(void)
{
    QTestState *qts = next_color_start();
    const uint8_t zero[3] = { 0, 0, 0 };
    TestPPM *ppm = NULL;
    static const uint8_t reset_black[] = { 0, 0, 0 };
    static const uint8_t command_values[] = { 0x44, 0x20, 0xc7 };

    /* Fill every architected DAC array before reset. */
    for (uint16_t address = 0; address < 0x210; address++) {
        const uint8_t value[3] = {
            (uint8_t)(address + 1),
            (uint8_t)(address >> 1),
            (uint8_t)(address ^ 0xa5),
        };

        dac_set_address(qts, address);
        dac_write_triplet(qts, 3, value);
    }
    dac_set_address(qts, 0x100);
    dac_write_triplet(qts, 2, (const uint8_t[3]) { 0xc1, 0xc2, 0xc3 });
    dac_set_address(qts, 0x101);
    dac_write_triplet(qts, 2, (const uint8_t[3]) { 0xd1, 0xd2, 0xd3 });

    for (uint16_t address = 0x201; address <= 0x203; address++) {
        dac_set_address(qts, address);
        qtest_writeb(qts, NEXT_COLOR_DAC + 2,
                     command_values[address - 0x201]);
    }
    for (uint16_t address = 0x205; address <= 0x208; address++) {
        dac_set_address(qts, address);
        qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x10 + address);
    }
    for (uint16_t address = 0x209; address <= 0x20c; address++) {
        dac_set_address(qts, address);
        qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x20 + address);
    }
    dac_set_address(qts, 0x20d);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0xa5);
    dac_set_address(qts, 0x20e);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x34);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x12);
    dac_set_address(qts, 0x20f);
    dac_write_triplet(qts, 2, (const uint8_t[3]) { 0x56, 0x78, 0x9a });

    /* Keep WT0 valid for the pre-reset scanout and vary every other WTT. */
    for (uint16_t address = 0x300; address <= 0x30f; address++) {
        const uint8_t value[3] = {
            address == 0x300 ? 0x00 : (uint8_t)(address - 0x2ff),
            address == 0x300 ? 0x01 : (uint8_t)(address >> 1),
            address == 0x300 ? 0x00 : (uint8_t)(address ^ 0x5a),
        };

        dac_set_address(qts, address);
        dac_write_triplet(qts, 2, value);
    }

    /* Leave the DAC in the middle of a palette triplet and advance blink. */
    dac_set_address(qts, 0x2a0);
    qtest_writeb(qts, NEXT_COLOR_DAC + 3, 0x7f);
    qtest_writeb(qts, NEXT_COLOR_VRAM, 0xf0);
    qtest_writeb(qts, NEXT_COLOR_VRAM + 1, 0x00);
    qtest_writeb(qts, NEXT_COLOR_COMMAND,
                 NEXT_COLOR_COMMAND_INTRENA | NEXT_COLOR_COMMAND_UNBLANK);
    qtest_clock_step(qts, 16 * NEXT_COLOR_RETRACE_NS);
    g_assert_cmphex(video_irq_status(qts), ==, NEXT_COLOR_VIDEO_IRQ_STATUS);

    qtest_system_reset(qts);
    qtest_writeb(qts, NEXT_COLOR_DAC + 3, 0xa5);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 1), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_COMMAND), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DRAM_TIMING), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_VRAM_TIMING), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_VRAM), ==, 0xf0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_VRAM + 1), ==, 0x00);
    g_assert_cmphex(video_irq_status(qts), ==, 0);

    /* The direct post-reset write must have used address 0, component 0. */
    assert_dac_triplet(qts, 3, 0,
                       (const uint8_t[3]) { 0xa5, 0x00, 0x00 });
    dac_set_address(qts, 0);
    dac_write_triplet(qts, 3, zero);

    /* Reset output is deterministic black while the board VRAM survives. */
    qtest_writeb(qts, NEXT_COLOR_COMMAND, NEXT_COLOR_COMMAND_UNBLANK);
    if (require_screendump(qts)) {
        ppm = create_test_ppm();
        assert_screendump_pixel(qts, ppm, 0, reset_black);
    }

    /* All 528 palette entries and both cursor colors are cleared. */
    for (uint16_t address = 0; address < 0x210; address++) {
        assert_dac_triplet(qts, 3, address, zero);
    }
    assert_dac_triplet(qts, 2, 0x100, zero);
    assert_dac_triplet(qts, 2, 0x101, zero);

    for (uint16_t address = 0x201; address <= 0x203; address++) {
        dac_set_address(qts, address);
        g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0);
    }
    for (uint16_t address = 0x205; address <= 0x20c; address++) {
        dac_set_address(qts, address);
        g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0);
    }
    dac_set_address(qts, 0x20d);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0);
    dac_set_address(qts, 0x20e);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0);
    dac_set_address(qts, 0x20f);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0);

    for (uint16_t address = 0x300; address <= 0x30f; address++) {
        assert_dac_triplet(qts, 2, address, zero);
    }

    /* A first palette write starts at component zero after reset. */
    dac_set_address(qts, 0);
    qtest_writeb(qts, NEXT_COLOR_DAC + 3, 0xa5);
    assert_dac_triplet(qts, 3, 0,
                       (const uint8_t[3]) { 0xa5, 0x00, 0x00 });

    qtest_quit(qts);
}

/*
 * Source: NeXTMach mk-108.1, nextdev/video.c, Gamma[] initializer under
 * COLOR_FB, used by vid_C16_init() and vid_C16_SetBrightness().  The digest
 * test below independently checks this exact copy.
 */
#define NEXT_COLOR_BRIGHT_MAX 0x3d

static const uint8_t original_warp9c_gamma[256] = {
    0, 16, 23, 28, 32, 36, 39, 42,
    45, 48, 50, 53, 55, 58, 60, 62,
    64, 66, 68, 70, 71, 73, 75, 77,
    78, 80, 81, 83, 84, 86, 87, 89,
    90, 92, 93, 94, 96, 97, 98, 100,
    101, 102, 103, 105, 106, 107, 108, 109,
    111, 112, 113, 114, 115, 116, 117, 118,
    119, 121, 122, 123, 124, 125, 126, 127,
    128, 129, 130, 131, 132, 133, 134, 135,
    135, 136, 137, 138, 139, 140, 141, 142,
    143, 144, 145, 145, 146, 147, 148, 149,
    150, 151, 151, 152, 153, 154, 155, 156,
    156, 157, 158, 159, 160, 160, 161, 162,
    163, 164, 164, 165, 166, 167, 167, 168,
    169, 170, 170, 171, 172, 173, 173, 174,
    175, 176, 176, 177, 178, 179, 179, 180,
    181, 181, 182, 183, 183, 184, 185, 186,
    186, 187, 188, 188, 189, 190, 190, 191,
    192, 192, 193, 194, 194, 195, 196, 196,
    197, 198, 198, 199, 199, 200, 201, 201,
    202, 203, 203, 204, 204, 205, 206, 206,
    207, 208, 208, 209, 209, 210, 211, 211,
    212, 212, 213, 214, 214, 215, 215, 216,
    217, 217, 218, 218, 219, 220, 220, 221,
    221, 222, 222, 223, 224, 224, 225, 225,
    226, 226, 227, 228, 228, 229, 229, 230,
    230, 231, 231, 232, 233, 233, 234, 234,
    235, 235, 236, 236, 237, 237, 238, 238,
    239, 240, 240, 241, 241, 242, 242, 243,
    243, 244, 244, 245, 245, 246, 246, 247,
    247, 248, 248, 249, 249, 250, 250, 251,
    251, 252, 252, 253, 253, 254, 254, 255,
};

static void test_bt463_original_gamma_digest(void)
{
    g_autofree char *digest = g_compute_checksum_for_data(
        G_CHECKSUM_SHA256, original_warp9c_gamma,
        G_N_ELEMENTS(original_warp9c_gamma));

    g_assert_cmpstr(digest, ==,
                    "b10c349fd56b298262a26a52ea4c6288"
                    "10218deb2ff4a6fd82266e3da41322f6");
}

static uint8_t original_warp9c_brightness_value(unsigned index,
                                                unsigned brightness)
{
    unsigned scale = (brightness * 64) / NEXT_COLOR_BRIGHT_MAX;
    unsigned level = ((original_warp9c_gamma[index] * scale) >> 6) + 0x08;

    return MIN(level, 0xff);
}

static void program_original_warp9c_set_brightness(QTestState *qts,
                                                    unsigned brightness)
{
    unsigned scale = (brightness * 64) / NEXT_COLOR_BRIGHT_MAX;

    /* Keep the DAC write order byte-for-byte with vid_C16_SetBrightness(). */
    for (uint16_t address = 0; address < 0x100; address++) {
        unsigned level = ((original_warp9c_gamma[address] * scale) >> 6) +
            0x08;

        if (level > 0xff) {
            level = 0xff;
        }
        dac_set_address(qts, address);
        qtest_writeb(qts, NEXT_COLOR_DAC + 3, level);
        qtest_writeb(qts, NEXT_COLOR_DAC + 3, level);
        qtest_writeb(qts, NEXT_COLOR_DAC + 3, level);
    }
}

static void original_warp9c_expected_palette(unsigned address,
                                             unsigned brightness,
                                             uint8_t value[3])
{
    if (address < 0x100) {
        value[0] = original_warp9c_brightness_value(address, brightness);
        value[1] = value[0];
        value[2] = value[0];
    } else if (address < 0x200) {
        value[0] = original_warp9c_gamma[address - 0x100];
        value[1] = value[0];
        value[2] = value[0];
    } else if (address < 0x20c) {
        memset(value, 0, 3);
    } else {
        memset(value, 0xff, 3);
    }
}

static void program_original_warp9c_init_at_brightness(QTestState *qts,
                                                        unsigned brightness)
{
    unsigned level;
    /* This is the command/mask/WTT sequence in vid_C16_init(). */
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
        qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x00);
        qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x01);
        qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x00);
    }

    for (uint16_t address = 0x100; address < 0x200; address++) {
        level = original_warp9c_gamma[address - 0x100];
        dac_set_address(qts, address);
        qtest_writeb(qts, NEXT_COLOR_DAC + 3, level);
        qtest_writeb(qts, NEXT_COLOR_DAC + 3, level);
        qtest_writeb(qts, NEXT_COLOR_DAC + 3, level);
    }
    for (uint16_t address = 0x20c; address < 0x210; address++) {
        dac_set_address(qts, address);
        qtest_writeb(qts, NEXT_COLOR_DAC + 3, 0xff);
        qtest_writeb(qts, NEXT_COLOR_DAC + 3, 0xff);
        qtest_writeb(qts, NEXT_COLOR_DAC + 3, 0xff);
    }

    /* This is the final vid_C16_SetBrightness() rewrite of entries 0..255. */
    program_original_warp9c_set_brightness(qts, brightness);
}

static void program_original_warp9c_init(QTestState *qts)
{
    program_original_warp9c_init_at_brightness(qts, NEXT_COLOR_BRIGHT_MAX);
}

static void test_bt463_original_init_sequence(void)
{
    QTestState *qts = next_color_start();
    static const uint8_t zero[3] = { 0, 0, 0 };
    static const uint8_t white[3] = { 0xff, 0xff, 0xff };
    static const uint8_t command[] = { 0x40, 0x00, 0x80 };

    program_original_warp9c_init(qts);

    /* Commands and masks are the individual byte writes from vid_C16_init. */
    for (uint16_t address = 0x201; address <= 0x203; address++) {
        dac_set_address(qts, address);
        g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==,
                        command[address - 0x201]);
    }
    for (uint16_t address = 0x205; address <= 0x208; address++) {
        dac_set_address(qts, address);
        g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0xf0);
    }
    for (uint16_t address = 0x209; address <= 0x20c; address++) {
        dac_set_address(qts, address);
        g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 2), ==, 0x00);
    }
    for (uint16_t address = 0x300; address <= 0x30f; address++) {
        assert_dac_triplet(qts, 2, address,
                           (const uint8_t[3]) { 0x00, 0x01, 0x00 });
    }

    /* vid_C16_init first writes the unscaled copy of Gamma to 0x100..1ff. */
    for (unsigned index = 0; index < 256; index++) {
        const uint8_t gamma[3] = {
            original_warp9c_gamma[index],
            original_warp9c_gamma[index],
            original_warp9c_gamma[index],
        };

        assert_dac_triplet(qts, 3, 0x100 + index, gamma);
    }
    for (uint16_t address = 0x200; address < 0x20c; address++) {
        assert_dac_triplet(qts, 3, address, zero);
    }
    for (uint16_t address = 0x20c; address < 0x210; address++) {
        assert_dac_triplet(qts, 3, address, white);
    }

    /* SetBrightness rewrites all three DAC components of entries 0..255. */
    for (unsigned index = 0; index < 256; index++) {
        const uint8_t brightness[3] = {
            original_warp9c_brightness_value(index, NEXT_COLOR_BRIGHT_MAX),
            original_warp9c_brightness_value(index, NEXT_COLOR_BRIGHT_MAX),
            original_warp9c_brightness_value(index, NEXT_COLOR_BRIGHT_MAX),
        };

        assert_dac_triplet(qts, 3, index, brightness);
    }
    /* Index 0xff is verified by DAC readback, not by the RGB444 scanout. */
    assert_dac_triplet(qts, 3, 0xff, white);

    qtest_quit(qts);
}

static void test_bt463_original_brightness_rewrite(void)
{
    QTestState *qts = next_color_start();
    TestPPM *ppm;
    static const uint8_t pixels[] = {
        0x00, 0x00, /* index 0 on every channel */
        0x22, 0x20, /* index 0x20 on every channel */
        0x88, 0x80, /* index 0x80 on every channel */
        /* 0xfff0 addresses palette index 0xf0 on each channel. */
        /* Its low nibble is WT 0xf. */
        0xff, 0xf0,
        0x20, 0x00, /* red only */
        0x02, 0x00, /* green only */
        0x00, 0x20, /* blue only */
    };
    uint8_t vram_before[sizeof(pixels)];
    const unsigned brightness_levels[] = {
        NEXT_COLOR_BRIGHT_MAX, 0x1f, 0,
    };

    if (!require_screendump(qts)) {
        qtest_quit(qts);
        return;
    }
    ppm = create_test_ppm();

    program_original_warp9c_init(qts);
    qtest_bufwrite(qts, NEXT_COLOR_VRAM, pixels, sizeof(pixels));
    for (unsigned i = 0; i < sizeof(vram_before); i++) {
        vram_before[i] = qtest_readb(qts, NEXT_COLOR_VRAM + i);
    }
    qtest_writeb(qts, NEXT_COLOR_COMMAND, NEXT_COLOR_COMMAND_UNBLANK);

    /* Check the source arithmetic at endpoint and midpoint values. */
    g_assert_cmpuint(original_warp9c_brightness_value(0,
                                                     NEXT_COLOR_BRIGHT_MAX),
                     ==, 0x08);
    g_assert_cmpuint(original_warp9c_brightness_value(0x20,
                                                     NEXT_COLOR_BRIGHT_MAX),
                     ==, 0x62);
    g_assert_cmpuint(original_warp9c_brightness_value(0x80,
                                                     NEXT_COLOR_BRIGHT_MAX),
                     ==, 0xbd);
    g_assert_cmpuint(original_warp9c_brightness_value(0xff,
                                                     NEXT_COLOR_BRIGHT_MAX),
                     ==, 0xff);
    g_assert_cmpuint(original_warp9c_brightness_value(0x20, 0x1f),
                     ==, 0x35);
    g_assert_cmpuint(original_warp9c_brightness_value(0x80, 0x1f),
                     ==, 0x62);
    g_assert_cmpuint(original_warp9c_brightness_value(0xff, 0x1f),
                     ==, 0x87);

    for (unsigned level = 0; level < G_N_ELEMENTS(brightness_levels);
         level++) {
        program_original_warp9c_set_brightness(qts, brightness_levels[level]);

        /* No VRAM write occurs as the LUT-only brightness update runs. */
        for (unsigned i = 0; i < sizeof(vram_before); i++) {
            g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_VRAM + i), ==,
                            vram_before[i]);
        }
        for (unsigned i = 0; i < sizeof(pixels); i += 2) {
            uint16_t pixel = ((uint16_t)pixels[i] << 8) | pixels[i + 1];
            uint8_t expected[3] = {
                original_warp9c_brightness_value(
                    ((pixel >> 12) & 0xf) << 4, brightness_levels[level]),
                original_warp9c_brightness_value(
                    ((pixel >> 8) & 0xf) << 4, brightness_levels[level]),
                original_warp9c_brightness_value(
                    ((pixel >> 4) & 0xf) << 4, brightness_levels[level]),
            };

            assert_screendump_pixel(qts, ppm, i / 2, expected);
        }
    }

    qtest_quit(qts);
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
        static const uint8_t red[] = { 0xff, 0x08, 0x08 };
        static const uint8_t black[] = { 0x08, 0x08, 0x08 };

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
    static const uint8_t red[] = { 0xff, 0x08, 0x08 };
    static const uint8_t black[] = { 0x08, 0x08, 0x08 };

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

static void test_bt463_board_blink_relevance(void)
{
    TestPPM *trace = create_test_ppm();
    g_autofree char *quoted_trace_path = g_shell_quote(trace->path);
    g_autofree char *args =
        g_strdup_printf("-trace enable=next_color_retrace,file=%s",
                        quoted_trace_path);
    QTestState *qts = next_color_start_with_args(args);
    TestPPM *ppm;
    static const uint8_t red[] = { 0xff, 0x08, 0x08 };
    static const uint8_t black[] = { 0x08, 0x08, 0x08 };

    if (!require_screendump(qts)) {
        qtest_quit(qts);
        return;
    }
    ppm = create_test_ppm();
    program_blink_fixture(qts, 0x44); /* 16 on, 16 off. */

    /* Expose inactive low nibbles and byte 3 to the generic helper. */
    for (uint16_t address = 0x205; address <= 0x207; address++) {
        dac_set_address(qts, address);
        qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0xff);
    }
    dac_set_address(qts, 0x208);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x0f);
    dac_set_address(qts, 0x209);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x0f);
    dac_set_address(qts, 0x20a);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x0f);
    dac_set_address(qts, 0x20b);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x0f);
    dac_set_address(qts, 0x20c);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0xff);
    assert_screendump_pixel(qts, ppm, 0, red);

    /* Warp9C P24-P27 are inactive: byte-3 blinking cannot redraw scanout. */
    qtest_clock_step(qts, 16 * NEXT_COLOR_RETRACE_NS);
    assert_screendump_pixel(qts, ppm, 0, red);

    /* An active color byte still invalidates and changes the visible pixel. */
    dac_set_address(qts, 0x20c);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x00);
    dac_set_address(qts, 0x209);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0xf0);
    dac_set_address(qts, 0x201);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0x44); /* Reset phase to on. */
    assert_screendump_pixel(qts, ppm, 0, red);
    qtest_clock_step(qts, 16 * NEXT_COLOR_RETRACE_NS);
    assert_screendump_pixel(qts, ppm, 0, black);

    qtest_quit(qts);

    {
        g_autofree char *events = NULL;
        gsize events_length;
        const char *cursor;
        unsigned invalidations = 0;

        g_assert_true(g_file_get_contents(trace->path, &events,
                                          &events_length, NULL));
        cursor = events;
        while ((cursor = g_strstr_len(cursor, events_length -
                                      (cursor - events),
                                      "invalidate=1"))) {
            invalidations++;
            cursor += strlen("invalidate=1");
        }
        /* The inactive byte-3 phase is absent; the active byte flips once. */
        g_assert_cmpuint(invalidations, ==, 1);
    }
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
        0xff, 0x08, 0x08,
        0x08, 0xff, 0x08,
        0x08, 0x08, 0xff,
        0x08, 0x08, 0x08,
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

static void test_bt463_original_gamma_ramp(void)
{
    QTestState *qts = next_color_start();
    TestPPM *ppm;
    static const uint8_t pixels[] = {
        0x00, 0x00, /* Gamma index 0 on every channel. */
        0x22, 0x20, /* Gamma index 0x20 on every channel. */
        0x88, 0x80, /* Gamma index 0x80 on every channel. */
        /* 0xfff0 addresses palette index 0xf0 on each channel. */
        /* Its low nibble is WT 0xf. */
        0xff, 0xf0,
        0x20, 0x00, /* Red only. */
        0x02, 0x00, /* Green only. */
        0x00, 0x20, /* Blue only. */
    };
    static const uint8_t expected[][3] = {
        { 0x08, 0x08, 0x08 },
        { 0x62, 0x62, 0x62 },
        { 0xbd, 0xbd, 0xbd },
        { 0xff, 0xff, 0xff },
        { 0x62, 0x08, 0x08 },
        { 0x08, 0x62, 0x08 },
        { 0x08, 0x08, 0x62 },
    };

    program_original_warp9c_init(qts);
    /* The true 0xff endpoint is covered by readback; 0xfff0 scans as 0xf0. */
    assert_dac_triplet(qts, 3, 0xff,
                       (const uint8_t[3]) { 0xff, 0xff, 0xff });

    if (!require_screendump(qts)) {
        qtest_quit(qts);
        return;
    }
    ppm = create_test_ppm();

    qtest_bufwrite(qts, NEXT_COLOR_VRAM, pixels, sizeof(pixels));
    qtest_writeb(qts, NEXT_COLOR_COMMAND, NEXT_COLOR_COMMAND_UNBLANK);

    for (unsigned i = 0; i < G_N_ELEMENTS(expected); i++) {
        assert_screendump_pixel(qts, ppm, i, expected[i]);
    }

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
    dac_write_triplet(qts, 2, (const uint8_t[3]) { 0x00, 0x01, 0x04 });
    dac_set_address(qts, 0x010);
    dac_write_triplet(qts, 3, (const uint8_t[3]) { 0xa1, 0x00, 0x00 });
    dac_set_address(qts, 0x020);
    dac_write_triplet(qts, 3, (const uint8_t[3]) { 0xb2, 0x00, 0x00 });
    qtest_bufwrite(qts, NEXT_COLOR_VRAM, pixels, sizeof(pixels));
    qtest_writeb(qts, NEXT_COLOR_COMMAND, NEXT_COLOR_COMMAND_UNBLANK);

    assert_screendump_pixel(qts, ppm, 0,
                            (const uint8_t[3]) { 0xa1, 0x08, 0x08 });
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
    static const uint8_t red[] = { 0xff, 0x08, 0x08 };

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

static void test_migration_outer_v1_fixture(void)
{
    const char *legacy_binary = g_getenv("QTEST_QEMU_BINARY_V1");
    TestMigration *migration;
    g_autofree char *uri = NULL;
    QTestState *destination;
    QTestState *source;
    const uint8_t palette[3] = { 0x12, 0x34, 0x56 };
    const uint8_t wtt[3] = { 0xaa, 0xbb, 0xcc };

    /*
     * QMP cannot select a historical VMState version.  An executable built
     * before the Bt463 extraction is therefore supplied out-of-band when an
     * actual outer-v1 stream is required; the current migration tests cover
     * the normal v3 stream when that fixture is unavailable.
     */
    if (!legacy_binary || !*legacy_binary) {
        g_test_skip("set QTEST_QEMU_BINARY_V1 to a pre-Bt463-extraction "
                    "QEMU binary to generate an outer-v1 stream");
        return;
    }

    migration = create_test_migration();
    uri = g_strdup_printf("unix:%s", migration->socket_path);
    destination = next_color_start_with_args("-incoming defer");
    source = next_color_start_with_env("QTEST_QEMU_BINARY_V1", NULL);

    qtest_qmp_assert_success(
        destination,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);

    /* The v1 device stores the generic 0x400-entry DAC arrays directly. */
    dac_set_address(source, 0x010);
    dac_write_triplet(source, 3, palette);
    dac_set_address(source, 0x300);
    dac_write_triplet(source, 2, wtt);
    dac_set_address(source, 0x3abc);

    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    qtest_qmp_eventwait(source, "STOP");
    qtest_qmp_eventwait(destination, "RESUME");

    /* The imported legacy address is constrained to the 10-bit DAC space. */
    g_assert_cmphex(qtest_readb(destination, NEXT_COLOR_DAC), ==, 0xbc);
    g_assert_cmphex(qtest_readb(destination, NEXT_COLOR_DAC + 1), ==, 0x02);
    assert_dac_triplet(destination, 3, 0x010, palette);
    assert_dac_triplet(destination, 2, 0x300, wtt);

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

static void test_migration_complete_bt463_state(void)
{
    TestMigration *migration = create_test_migration();
    g_autofree char *uri =
        g_strdup_printf("unix:%s", migration->socket_path);
    QTestState *destination =
        next_color_start_with_args("-incoming defer");
    QTestState *source = next_color_start();
    TestPPM *ppm = NULL;
    const unsigned brightness = 0x1f;
    const uint8_t enabled =
        NEXT_COLOR_COMMAND_INTRENA | NEXT_COLOR_COMMAND_UNBLANK;
    static const uint8_t command_values[3] = { 0x44, 0x01, 0x80 };
    static const uint8_t read_mask_values[4] = { 0xf0, 0xe1, 0xd2, 0xc3 };
    static const uint8_t blink_mask_values[4] = { 0xf0, 0x0f, 0xaa, 0x55 };
    const unsigned blink_elapsed = 5;
    const unsigned retraces_to_transition = 15 - blink_elapsed;
    const int64_t elapsed = NEXT_COLOR_RETRACE_NS / 2;
    const int64_t remaining = NEXT_COLOR_RETRACE_NS - elapsed;
    int64_t source_clock;

    qtest_qmp_assert_success(
        destination,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);

    /* Start from the exact driver state, then exercise every writable field. */
    program_original_warp9c_init_at_brightness(source, brightness);
    for (uint16_t address = 0x300; address <= 0x30f; address++) {
        const uint32_t value = 0x000100 + address - 0x300;
        const uint8_t wtt[3] = {
            value,
            value >> 8,
            value >> 16,
        };

        dac_set_address(source, address);
        dac_write_triplet(source, 2, wtt);
    }
    for (uint16_t address = 0x205; address <= 0x208; address++) {
        dac_set_address(source, address);
        qtest_writeb(source, NEXT_COLOR_DAC + 2,
                     read_mask_values[address - 0x205]);
    }
    for (uint16_t address = 0x209; address <= 0x20c; address++) {
        dac_set_address(source, address);
        qtest_writeb(source, NEXT_COLOR_DAC + 2,
                     blink_mask_values[address - 0x209]);
    }
    dac_set_address(source, 0x201);
    qtest_writeb(source, NEXT_COLOR_DAC + 2, 0x44);
    /* CR1=1 is compatible with the fixture: inactive P24-P27 keep OL0 clear. */
    dac_set_address(source, 0x202);
    qtest_writeb(source, NEXT_COLOR_DAC + 2, command_values[1]);
    dac_set_address(source, 0x100);
    dac_write_triplet(source, 2, (const uint8_t[3]) { 0xc1, 0xc2, 0xc3 });
    dac_set_address(source, 0x101);
    dac_write_triplet(source, 2, (const uint8_t[3]) { 0xd1, 0xd2, 0xd3 });
    dac_set_address(source, 0x20d);
    qtest_writeb(source, NEXT_COLOR_DAC + 2, 0xa5);
    dac_set_address(source, 0x20e);
    qtest_writeb(source, NEXT_COLOR_DAC + 2, 0x34);
    qtest_writeb(source, NEXT_COLOR_DAC + 2, 0x12);
    dac_set_address(source, 0x20f);
    dac_write_triplet(source, 2, (const uint8_t[3]) { 0x56, 0x78, 0x9a });

    /* Preserve a real palette component phase across migration. */
    dac_set_address(source, 0x050);
    qtest_writeb(source, NEXT_COLOR_DAC + 3, 0x5a);

    qtest_writeb(source, NEXT_COLOR_VRAM, 0xf0);
    qtest_writeb(source, NEXT_COLOR_VRAM + 1, 0x00);
    qtest_writeb(source, NEXT_COLOR_COMMAND, enabled);
    source_clock = qtest_clock_step(source,
                                    (16 + blink_elapsed) *
                                    NEXT_COLOR_RETRACE_NS + elapsed);
    g_assert_cmphex(video_irq_status(source), ==,
                    NEXT_COLOR_VIDEO_IRQ_STATUS);

    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    qtest_qmp_eventwait(source, "STOP");
    qtest_qmp_eventwait(destination, "RESUME");
    qtest_clock_set(destination, source_clock);

    /* The destination resumes with the saved address/component phase. */
    qtest_writeb(destination, NEXT_COLOR_DAC + 3, 0x6b);
    qtest_writeb(destination, NEXT_COLOR_DAC + 3, 0x7c);
    assert_dac_triplet(destination, 3, 0x050,
                       (const uint8_t[3]) { 0x5a, 0x6b, 0x7c });

    /* Every palette entry, including both driver LUT regions, migrated. */
    for (unsigned address = 0; address < 0x210; address++) {
        uint8_t expected[3];

        original_warp9c_expected_palette(address, brightness, expected);
        if (address == 0x050) {
            expected[0] = 0x5a;
            expected[1] = 0x6b;
            expected[2] = 0x7c;
        }
        assert_dac_triplet(destination, 3, address, expected);
    }
    assert_dac_triplet(destination, 2, 0x100,
                       (const uint8_t[3]) { 0xc1, 0xc2, 0xc3 });
    assert_dac_triplet(destination, 2, 0x101,
                       (const uint8_t[3]) { 0xd1, 0xd2, 0xd3 });

    /* Commands, masks, test/signature bytes, and every WTT entry survive. */
    for (uint16_t address = 0x201; address <= 0x203; address++) {
        dac_set_address(destination, address);
        g_assert_cmphex(qtest_readb(destination, NEXT_COLOR_DAC + 2), ==,
                        command_values[address - 0x201]);
    }
    for (uint16_t address = 0x205; address <= 0x208; address++) {
        dac_set_address(destination, address);
        g_assert_cmphex(qtest_readb(destination, NEXT_COLOR_DAC + 2), ==,
                        read_mask_values[address - 0x205]);
    }
    for (uint16_t address = 0x209; address <= 0x20c; address++) {
        dac_set_address(destination, address);
        g_assert_cmphex(qtest_readb(destination, NEXT_COLOR_DAC + 2), ==,
                        blink_mask_values[address - 0x209]);
    }
    dac_set_address(destination, 0x20d);
    g_assert_cmphex(qtest_readb(destination, NEXT_COLOR_DAC + 2), ==, 0xa5);
    dac_set_address(destination, 0x20e);
    g_assert_cmphex(qtest_readb(destination, NEXT_COLOR_DAC + 2), ==, 0x34);
    g_assert_cmphex(qtest_readb(destination, NEXT_COLOR_DAC + 2), ==, 0x12);
    dac_set_address(destination, 0x20f);
    g_assert_cmphex(qtest_readb(destination, NEXT_COLOR_DAC + 2), ==, 0x56);
    g_assert_cmphex(qtest_readb(destination, NEXT_COLOR_DAC + 2), ==, 0x78);
    g_assert_cmphex(qtest_readb(destination, NEXT_COLOR_DAC + 2), ==, 0x9a);
    for (uint16_t address = 0x300; address <= 0x30f; address++) {
        const uint32_t value = 0x000100 + address - 0x300;

        assert_dac_triplet(destination, 2, address,
                           (const uint8_t[3]) {
                               value,
                               value >> 8,
                               value >> 16,
                           });
    }

    /* IRQ and the half-period timer deadline continue at the destination. */
    g_assert_cmphex(video_irq_status(destination), ==,
                    NEXT_COLOR_VIDEO_IRQ_STATUS);
    qtest_writeb(destination, NEXT_COLOR_COMMAND,
                 NEXT_COLOR_COMMAND_CLRINTR | NEXT_COLOR_COMMAND_UNBLANK);
    qtest_writeb(destination, NEXT_COLOR_COMMAND, enabled);
    qtest_clock_step(destination, remaining - 1);
    g_assert_cmphex(video_irq_status(destination), ==, 0);
    qtest_clock_step(destination, 1);
    g_assert_cmphex(video_irq_status(destination), ==,
                    NEXT_COLOR_VIDEO_IRQ_STATUS);

    /* The saved blink phase changes the same VRAM pixel at the next retrace. */
    if (require_screendump(destination)) {
        ppm = create_test_ppm();
        assert_screendump_pixel(destination, ppm, 0,
                                (const uint8_t[3]) { 0x08, 0x08, 0x08 });
    }

    /*
     * Five retraces had already elapsed in the off phase at migration.  The
     * next ten destination retraces must therefore reach the saved phase
     * transition; a reset/dropped blink counter would need sixteen instead.
     */
    qtest_writeb(destination, NEXT_COLOR_COMMAND,
                 NEXT_COLOR_COMMAND_CLRINTR | NEXT_COLOR_COMMAND_UNBLANK);
    qtest_writeb(destination, NEXT_COLOR_COMMAND, enabled);
    for (unsigned retrace = 0; retrace < retraces_to_transition; retrace++) {
        qtest_clock_step(destination, NEXT_COLOR_RETRACE_NS - 1);
        g_assert_cmphex(video_irq_status(destination), ==, 0);
        qtest_clock_step(destination, 1);
        g_assert_cmphex(video_irq_status(destination), ==,
                        NEXT_COLOR_VIDEO_IRQ_STATUS);
        if (retrace + 1 < retraces_to_transition) {
            if (ppm) {
                assert_screendump_pixel(destination, ppm, 0,
                                        (const uint8_t[3]) {
                                            0x08, 0x08, 0x08,
                                        });
            }
            qtest_writeb(destination, NEXT_COLOR_COMMAND,
                         NEXT_COLOR_COMMAND_CLRINTR |
                         NEXT_COLOR_COMMAND_UNBLANK);
            qtest_writeb(destination, NEXT_COLOR_COMMAND, enabled);
        }
    }
    if (ppm) {
        assert_screendump_pixel(destination, ppm, 0,
                                (const uint8_t[3]) { 0x83, 0x08, 0x08 });
    }

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
    qtest_add_func("/next-color-video/bt463-complete-reset",
                   test_bt463_complete_reset);
    qtest_add_func("/next-color-video/bt463-original-init-sequence",
                   test_bt463_original_init_sequence);
    qtest_add_func("/next-color-video/bt463-original-gamma-digest",
                   test_bt463_original_gamma_digest);
    qtest_add_func("/next-color-video/bt463-original-brightness-rewrite",
                   test_bt463_original_brightness_rewrite);
    qtest_add_func("/next-color-video/migration", test_migration);
    qtest_add_func("/next-color-video/migration-outer-v1-fixture",
                   test_migration_outer_v1_fixture);
    qtest_add_func("/next-color-video/migration-active-timer-and-dac-phase",
                   test_migration_active_timer_and_dac_phase);
    qtest_add_func("/next-color-video/migration-complete-bt463-state",
                   test_migration_complete_bt463_state);
    qtest_add_func("/next-color-video/rgb444-scanout", test_rgb444_scanout);
    qtest_add_func("/next-color-video/bt463-lut-invalidation",
                   test_bt463_lut_invalidation);
    qtest_add_func("/next-color-video/bt463-original-gamma-ramp",
                   test_bt463_original_gamma_ramp);
    qtest_add_func("/next-color-video/bt463-wtt-tags", test_bt463_wtt_tags);
    qtest_add_func("/next-color-video/bt463-load-interleave",
                   test_bt463_load_interleave_scanout);
    qtest_add_func("/next-color-video/bt463-blink-rates",
                   test_bt463_blink_rates);
    qtest_add_func("/next-color-video/bt463-blink-command0-reset",
                   test_bt463_blink_command0_reset);
    qtest_add_func("/next-color-video/bt463-board-blink-relevance",
                   test_bt463_board_blink_relevance);
    qtest_add_func("/next-color-video/blanking", test_blanking);

    return g_test_run();
}
