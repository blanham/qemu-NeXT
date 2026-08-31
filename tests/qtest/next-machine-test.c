/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/units.h"
#include "libqtest.h"

#define NEXT_SCR1             0x0200c000
#define NEXT_RAM_BASE         0x04000000
#define NEXT_ROM_SIZE         (128 * KiB)
#define NEXT_FB_RANGE         \
    "000000000b000000-000000000b1cb0ff"
#define TEST_TIMEOUT          (5 * G_USEC_PER_SEC)

typedef struct TestROM {
    int fd;
    char *path;
} TestROM;

typedef struct ExpectedSCR1 {
    const char *machine;
    uint32_t value;
    uint8_t dma_revision;
    uint8_t machine_type;
    uint8_t board_revision;
    uint8_t cpu_clock;
} ExpectedSCR1;

typedef struct MachineTest {
    const char *machine;
    const char *product_name;
    uint64_t ram_size;
    bool has_mono_framebuffer;
} MachineTest;

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

    rom->fd = g_file_open_tmp("next-machine-rom-XXXXXX", &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;

    return rom;
}

static QTestState *next_machine_start_with_rom(
    TestROM *rom, const char *machine, const char *args)
{
    g_autofree char *quoted_rom_path = g_shell_quote(rom->path);

    return qtest_initf("-machine %s -bios %s %s",
                       machine, quoted_rom_path, args ?: "");
}

static QTestState *next_machine_start(const char *machine, const char *args)
{
    return next_machine_start_with_rom(create_test_rom(), machine, args);
}

static void test_machine_registration(void)
{
    create_test_rom();

    g_assert_true(qtest_has_machine("next-cube"));
    g_assert_true(qtest_has_machine("next-station"));
    g_assert_true(qtest_has_machine("next-station-color"));
    g_assert_true(qtest_has_machine("next-computer"));
}

static void test_scr1(gconstpointer opaque)
{
    const ExpectedSCR1 *expected = opaque;
    QTestState *qts = next_machine_start(expected->machine, NULL);
    uint32_t scr1 = qtest_readl(qts, NEXT_SCR1);

    g_assert_cmphex(scr1, ==, expected->value);
    g_assert_cmphex(extract32(scr1, 16, 8), ==, expected->dma_revision);
    g_assert_cmphex(extract32(scr1, 12, 4), ==, expected->machine_type);
    g_assert_cmphex(extract32(scr1, 8, 4), ==, expected->board_revision);
    g_assert_cmphex(extract32(scr1, 0, 2), ==, expected->cpu_clock);

    qtest_quit(qts);
}

static void test_ram_and_framebuffer(gconstpointer opaque)
{
    const MachineTest *test = opaque;
    g_autofree char *args =
        g_strdup_printf("-m %" PRIu64 "M", test->ram_size / MiB);
    QTestState *qts = next_machine_start(test->machine, args);
    g_autofree char *flatview = qtest_hmp(qts, "info mtree -f");
    const uint64_t last_word =
        NEXT_RAM_BASE + test->ram_size - sizeof(uint32_t);

    qtest_writel(qts, NEXT_RAM_BASE, 0x01234567);
    qtest_writel(qts, last_word, 0x89abcdef);
    g_assert_cmphex(qtest_readl(qts, NEXT_RAM_BASE), ==, 0x01234567);
    g_assert_cmphex(qtest_readl(qts, last_word), ==, 0x89abcdef);
    if (test->has_mono_framebuffer) {
        g_assert_nonnull(strstr(flatview, NEXT_FB_RANGE));
    } else {
        g_assert_null(strstr(flatview, NEXT_FB_RANGE));
    }

    qtest_quit(qts);
}

static void test_ram_rejection(gconstpointer opaque)
{
    const MachineTest *test = opaque;
    TestROM *rom = create_test_rom();

    if (g_test_subprocess()) {
        g_autofree char *args =
            g_strdup_printf("-m %" PRIu64 "M",
                            test->ram_size / MiB + 1);
        QTestState *qts =
            next_machine_start_with_rom(rom, test->machine, args);

        qtest_quit(qts);
        return;
    }

    g_test_trap_subprocess(NULL, TEST_TIMEOUT, 0);
    g_test_trap_assert_failed();
    {
        g_autofree char *pattern =
            g_strdup_printf("*%s supports at most %" PRIu64
                            " MiB of RAM*",
                            test->product_name, test->ram_size / MiB);

        g_test_trap_assert_stderr(pattern);
    }
}

static void test_invalid_cpu(void)
{
    TestROM *rom = create_test_rom();

    if (g_test_subprocess()) {
        QTestState *qts =
            next_machine_start_with_rom(rom, "next-cube", "-cpu m68030");

        qtest_quit(qts);
        return;
    }

    g_test_trap_subprocess(NULL, TEST_TIMEOUT, 0);
    g_test_trap_assert_failed();
    g_test_trap_assert_stderr("*Invalid CPU model*");
}

static void test_valid_cpu(void)
{
    TestROM *rom = create_test_rom();

    if (g_test_subprocess()) {
        QTestState *qts =
            next_machine_start_with_rom(rom, "next-cube", "-cpu m68040");

        qtest_quit(qts);
        return;
    }

    g_test_trap_subprocess(NULL, TEST_TIMEOUT, 0);
    g_test_trap_assert_passed();
}

static void test_default_cpu(void)
{
    QTestState *qts = next_machine_start("next-computer", NULL);
    g_autofree char *cpus = qtest_hmp(qts, "info cpus");

    g_assert_nonnull(strstr(cpus, "model=m68030"));
    qtest_quit(qts);
}

static void test_computer_invalid_cpu(void)
{
    TestROM *rom = create_test_rom();

    if (g_test_subprocess()) {
        QTestState *qts =
            next_machine_start_with_rom(rom, "next-computer", "-cpu m68040");

        qtest_quit(qts);
        return;
    }

    g_test_trap_subprocess(NULL, TEST_TIMEOUT, 0);
    g_test_trap_assert_failed();
    g_test_trap_assert_stderr("*Invalid CPU model*");
}

static void test_missing_firmware(void)
{
    create_test_rom();

    if (g_test_subprocess()) {
        const char *qemu_binary = g_getenv("QTEST_QEMU_BINARY");
        const char *argv[] = {
            qemu_binary,
            "-machine", "next-cube",
            "-display", "none",
            "-audio", "none",
            "-bios", "/definitely/missing/next.rom",
            NULL
        };
        g_autofree char *stderr_data = NULL;
        int wait_status = 0;

        g_assert_nonnull(qemu_binary);
        g_assert_true(g_spawn_sync(NULL, (char **)argv, NULL,
                                  G_SPAWN_STDOUT_TO_DEV_NULL,
                                  NULL, NULL, NULL, &stderr_data,
                                  &wait_status, NULL));
        g_assert_cmpint(wait_status, !=, 0);
        g_assert_nonnull(strstr(stderr_data, "Could not load ROM image"));
        return;
    }

    g_test_trap_subprocess(NULL, TEST_TIMEOUT, 0);
    g_test_trap_assert_passed();
}

int main(int argc, char **argv)
{
    static ExpectedSCR1 scr1_tests[] = {
        {
            .machine = "next-cube",
            .value = 0x00012002,
            .dma_revision = 1,
            .machine_type = 2,
            .board_revision = 0,
            .cpu_clock = 2,
        }, {
            .machine = "next-station",
            .value = 0x00011002,
            .dma_revision = 1,
            .machine_type = 1,
            .board_revision = 0,
            .cpu_clock = 2,
        }, {
            .machine = "next-station-color",
            .value = 0x00013002,
            .dma_revision = 1,
            .machine_type = 3,
            .board_revision = 0,
            .cpu_clock = 2,
        }, {
            .machine = "next-computer",
            .value = 0x00010102,
            .dma_revision = 1,
            .machine_type = 0,
            .board_revision = 1,
            .cpu_clock = 2,
        },
    };
    static MachineTest machine_tests[] = {
        {
            .machine = "next-cube",
            .product_name = "NeXTcube (68040, X15)",
            .ram_size = 64 * MiB,
            .has_mono_framebuffer = true,
        }, {
            .machine = "next-station",
            .product_name = "NeXTstation (Warp 9)",
            .ram_size = 64 * MiB,
            .has_mono_framebuffer = true,
        }, {
            .machine = "next-station-color",
            .product_name = "NeXTstation Color (Warp 9C)",
            .ram_size = 32 * MiB,
            .has_mono_framebuffer = false,
        }, {
            .machine = "next-computer",
            .product_name = "NeXT Computer (68030)",
            .ram_size = 64 * MiB,
            .has_mono_framebuffer = true,
        },
    };
    size_t i;

    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/next-machine/registration", test_machine_registration);
    for (i = 0; i < ARRAY_SIZE(scr1_tests); i++) {
        g_autofree char *path =
            g_strdup_printf("/next-machine/%s/scr1", scr1_tests[i].machine);

        qtest_add_data_func_full(path, &scr1_tests[i], test_scr1, NULL);
    }
    for (i = 0; i < ARRAY_SIZE(machine_tests); i++) {
        g_autofree char *ram_path =
            g_strdup_printf("/next-machine/%s/ram-and-framebuffer",
                            machine_tests[i].machine);
        g_autofree char *reject_path =
            g_strdup_printf("/next-machine/%s/reject-%" PRIu64 "m",
                            machine_tests[i].machine,
                            machine_tests[i].ram_size / MiB + 1);

        qtest_add_data_func_full(ram_path, &machine_tests[i],
                                 test_ram_and_framebuffer, NULL);
        qtest_add_data_func_full(reject_path, &machine_tests[i],
                                 test_ram_rejection, NULL);
    }
    qtest_add_func("/next-machine/next-cube/reject-m68030",
                   test_invalid_cpu);
    qtest_add_func("/next-machine/next-cube/accept-m68040", test_valid_cpu);
    qtest_add_func("/next-machine/next-computer/default-m68030",
                   test_default_cpu);
    qtest_add_func("/next-machine/next-computer/reject-m68040",
                   test_computer_invalid_cpu);
    qtest_add_func("/next-machine/next-cube/missing-firmware",
                   test_missing_firmware);

    return g_test_run();
}
