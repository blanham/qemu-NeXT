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

    dac_set_address(qts, 0x310);
    dac_write_triplet(qts, 2, tag0);
    dac_write_triplet(qts, 2, tag1);
    dac_set_address(qts, 0x310);
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
    assert_dac_triplet(qts, 3, 0x25a, low_reset);

    dac_set_address(qts, 0x15a);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0xee);
    qtest_writeb(qts, NEXT_COLOR_DAC + 1, 0x03);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC), ==, 0x5a);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 1), ==, 0x03);
    dac_write_triplet(qts, 2, high_reset);
    assert_dac_triplet(qts, 2, 0x35a, high_reset);

    dac_set_address(qts, 0x3ff);
    qtest_writeb(qts, NEXT_COLOR_DAC + 3, wrap[0]);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC), ==, 0xff);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 1), ==, 0x03);
    qtest_writeb(qts, NEXT_COLOR_DAC + 3, wrap[1]);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC), ==, 0xff);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 1), ==, 0x03);
    qtest_writeb(qts, NEXT_COLOR_DAC + 3, wrap[2]);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC), ==, 0x00);
    g_assert_cmphex(qtest_readb(qts, NEXT_COLOR_DAC + 1), ==, 0x00);
    assert_dac_triplet(qts, 3, 0x3ff, wrap);

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
    assert_dac_triplet(qts, 3, 0x100, zero);
    assert_dac_triplet(qts, 2, 0x300, zero);
    assert_dac_triplet(qts, 2, 0x222, zero);
    dac_set_address(qts, 0x222);
    qtest_writeb(qts, NEXT_COLOR_DAC + 2, 0xee);
    qtest_system_reset(qts);
    dac_write_triplet(qts, 2, reset_phase);
    assert_dac_triplet(qts, 2, 0x000, reset_phase);
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
    const uint8_t enabled =
        NEXT_COLOR_COMMAND_INTRENA | NEXT_COLOR_COMMAND_UNBLANK;

    qtest_qmp_assert_success(
        destination,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);

    dac_set_address(source, 0x100);
    dac_write_triplet(source, 3, palette);
    dac_set_address(source, 0x300);
    dac_write_triplet(source, 2, tag);
    qtest_writeb(source, NEXT_COLOR_DRAM_TIMING, 0x5a);
    qtest_writeb(source, NEXT_COLOR_VRAM_TIMING, 0xa5);
    qtest_writeb(source, NEXT_COLOR_COMMAND, enabled);
    qtest_clock_step(source, NEXT_COLOR_RETRACE_NS);
    g_assert_cmphex(video_irq_status(source), ==,
                    NEXT_COLOR_VIDEO_IRQ_STATUS);

    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    qtest_qmp_eventwait(source, "STOP");
    qtest_qmp_eventwait(destination, "RESUME");

    assert_dac_triplet(destination, 3, 0x100, palette);
    assert_dac_triplet(destination, 2, 0x300, tag);
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
    qtest_add_func("/next-color-video/bt463-retained-registers",
                   test_bt463_retained_registers);
    qtest_add_func("/next-color-video/bt463-auto-increment-and-phase-reset",
                   test_bt463_auto_increment_and_phase_reset);
    qtest_add_func("/next-color-video/command-and-retrace-irq",
                   test_command_and_retrace_irq);
    qtest_add_func("/next-color-video/registers-and-reset",
                   test_registers_and_reset);
    qtest_add_func("/next-color-video/migration", test_migration);
    qtest_add_func("/next-color-video/migration-active-timer-and-dac-phase",
                   test_migration_active_timer_and_dac_phase);

    return g_test_run();
}
