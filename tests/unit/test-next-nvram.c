/*
 * NeXT NVRAM schema tests
 *
 * Copyright (c) 2026 Bryce Lanham
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/nvram/next-nvram.h"

static const uint8_t expected_nvram[NEXT_NVRAM_SIZE] = {
    0x94, 0x0f, 0x40, 0x03, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xfb, 0x6d, 0x00, 0x00, 0x4b, 0x00,
    0x41, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x7e,
};

typedef struct NVRAMTempFile {
    char *dir;
    char *path;
} NVRAMTempFile;

static NVRAMTempFile nvram_temp_file_new(void)
{
    NVRAMTempFile temp = { 0 };
    g_autoptr(GError) err = NULL;

    temp.dir = g_dir_make_tmp("next-nvram-test-XXXXXX", &err);
    g_assert_no_error(err);
    g_assert_nonnull(temp.dir);
    temp.path = g_build_filename(temp.dir, "next.nvram", NULL);
    return temp;
}

static void nvram_temp_file_clear(NVRAMTempFile *temp)
{
    g_unlink(temp->path);
    g_rmdir(temp->dir);
    g_free(temp->path);
    g_free(temp->dir);
}

static void test_defaults(void)
{
    NextNVRAMState nvram;
    NextNVRAMSettings settings;

    next_nvram_init(&nvram);

    g_assert_cmpmem(nvram.data, NEXT_NVRAM_SIZE,
                    expected_nvram, sizeof(expected_nvram));
    g_assert_cmphex(next_nvram_get_stored_checksum(&nvram), ==, 0x847e);
    g_assert_cmphex(next_nvram_compute_checksum(&nvram), ==, 0x847e);
    g_assert_true(next_nvram_checksum_is_valid(&nvram));

    next_nvram_decode_settings(&nvram, &settings);
    g_assert_cmpuint(settings.reset, ==, 9);
    g_assert_false(settings.alt_console);
    g_assert_true(settings.allow_eject);
    g_assert_cmpuint(settings.volume_right, ==, 0);
    g_assert_cmpuint(settings.brightness, ==, 61);
    g_assert_cmpuint(settings.hw_password, ==, 0);
    g_assert_cmpuint(settings.volume_left, ==, 0);
    g_assert_false(settings.speaker);
    g_assert_false(settings.lowpass);
    g_assert_true(settings.boot_any);
    g_assert_true(settings.any_command);
}

static void test_settings_round_trip(void)
{
    NextNVRAMState nvram;
    const NextNVRAMSettings expected = {
        .reset = 5,
        .alt_console = true,
        .allow_eject = false,
        .volume_right = 17,
        .brightness = 42,
        .hw_password = 11,
        .volume_left = 53,
        .speaker = true,
        .lowpass = true,
        .boot_any = false,
        .any_command = false,
    };
    NextNVRAMSettings actual;

    next_nvram_init(&nvram);
    next_nvram_encode_settings(&nvram, &expected);
    next_nvram_decode_settings(&nvram, &actual);

    g_assert_cmpuint(actual.reset, ==, expected.reset);
    g_assert_cmpint(actual.alt_console, ==, expected.alt_console);
    g_assert_cmpint(actual.allow_eject, ==, expected.allow_eject);
    g_assert_cmpuint(actual.volume_right, ==, expected.volume_right);
    g_assert_cmpuint(actual.brightness, ==, expected.brightness);
    g_assert_cmpuint(actual.hw_password, ==, expected.hw_password);
    g_assert_cmpuint(actual.volume_left, ==, expected.volume_left);
    g_assert_cmpint(actual.speaker, ==, expected.speaker);
    g_assert_cmpint(actual.lowpass, ==, expected.lowpass);
    g_assert_cmpint(actual.boot_any, ==, expected.boot_any);
    g_assert_cmpint(actual.any_command, ==, expected.any_command);
}

static void test_settings_decode_layout(void)
{
    NextNVRAMState nvram;
    NextNVRAMSettings settings;

    next_nvram_init(&nvram);
    /*
     * 0xa95aaf5a:
     * reset=0xa, alt=1, eject=0, vol_r=0x15, brightness=0x2a,
     * password=0xb, vol_l=0x35, speaker=1, lowpass=0, boot_any=1, any=0.
     */
    nvram.data[NEXT_NVRAM_SETTINGS] = 0xa9;
    nvram.data[NEXT_NVRAM_SETTINGS + 1] = 0x5a;
    nvram.data[NEXT_NVRAM_SETTINGS + 2] = 0xaf;
    nvram.data[NEXT_NVRAM_SETTINGS + 3] = 0x5a;

    next_nvram_decode_settings(&nvram, &settings);

    g_assert_cmpuint(settings.reset, ==, 0xa);
    g_assert_true(settings.alt_console);
    g_assert_false(settings.allow_eject);
    g_assert_cmpuint(settings.volume_right, ==, 0x15);
    g_assert_cmpuint(settings.brightness, ==, 0x2a);
    g_assert_cmpuint(settings.hw_password, ==, 0xb);
    g_assert_cmpuint(settings.volume_left, ==, 0x35);
    g_assert_true(settings.speaker);
    g_assert_false(settings.lowpass);
    g_assert_true(settings.boot_any);
    g_assert_false(settings.any_command);
}

static void test_settings_encode_layout(void)
{
    NextNVRAMState nvram;
    const NextNVRAMSettings settings = {
        .reset = 0x3,
        .alt_console = false,
        .allow_eject = true,
        .volume_right = 0x2d,
        .brightness = 0x13,
        .hw_password = 0xe,
        .volume_left = 0x26,
        .speaker = false,
        .lowpass = true,
        .boot_any = false,
        .any_command = true,
    };
    static const uint8_t expected[] = { 0x36, 0xd4, 0xfa, 0x65 };

    next_nvram_init(&nvram);
    next_nvram_encode_settings(&nvram, &settings);

    g_assert_cmpmem(&nvram.data[NEXT_NVRAM_SETTINGS], sizeof(expected),
                    expected, sizeof(expected));
}

static void test_clock_config_decode_layout(void)
{
    static const struct {
        uint8_t raw;
        NextNVRAMClockConfig expected;
    } cases[] = {
        { 0x80, { .new_clock_chip = true } },
        { 0x40, { .auto_poweron = true } },
        { 0x20, { .use_console_slot = true } },
        { 0x08, { .console_slot = 1 } },
        { 0x10, { .console_slot = 2 } },
        { 0x07, { 0 } },
    };
    unsigned i;

    for (i = 0; i < ARRAY_SIZE(cases); i++) {
        NextNVRAMState nvram;
        NextNVRAMClockConfig config;

        next_nvram_init(&nvram);
        nvram.data[NEXT_NVRAM_CLOCK_CONFIG] = cases[i].raw;
        next_nvram_decode_clock_config(&nvram, &config);

        g_assert_cmpint(config.new_clock_chip, ==,
                        cases[i].expected.new_clock_chip);
        g_assert_cmpint(config.auto_poweron, ==,
                        cases[i].expected.auto_poweron);
        g_assert_cmpint(config.use_console_slot, ==,
                        cases[i].expected.use_console_slot);
        g_assert_cmpuint(config.console_slot, ==,
                         cases[i].expected.console_slot);
    }
}

static void test_clock_config_encode_layout(void)
{
    static const struct {
        NextNVRAMClockConfig config;
        uint8_t expected;
    } cases[] = {
        { { .new_clock_chip = true }, 0x87 },
        { { .auto_poweron = true }, 0x47 },
        { { .use_console_slot = true }, 0x27 },
        { { .console_slot = 1 }, 0x0f },
        { { .console_slot = 2 }, 0x17 },
        { { 0 }, 0x07 },
    };
    unsigned i;

    for (i = 0; i < ARRAY_SIZE(cases); i++) {
        NextNVRAMState nvram;

        next_nvram_init(&nvram);
        nvram.data[NEXT_NVRAM_CLOCK_CONFIG] = 0x07;
        next_nvram_encode_clock_config(&nvram, &cases[i].config);

        g_assert_cmphex(nvram.data[NEXT_NVRAM_CLOCK_CONFIG], ==,
                        cases[i].expected);
    }
}

static void test_clock_config_round_trip(void)
{
    NextNVRAMState nvram;
    const NextNVRAMClockConfig expected = {
        .new_clock_chip = true,
        .auto_poweron = true,
        .use_console_slot = true,
        .console_slot = 2,
    };
    NextNVRAMClockConfig actual;

    next_nvram_init(&nvram);
    nvram.data[NEXT_NVRAM_CLOCK_CONFIG] = 0x07;
    next_nvram_encode_clock_config(&nvram, &expected);

    g_assert_cmphex(nvram.data[NEXT_NVRAM_CLOCK_CONFIG], ==, 0xf7);
    g_assert_cmphex(nvram.data[NEXT_NVRAM_CLOCK_CONFIG] & 0x07, ==, 0x07);

    next_nvram_decode_clock_config(&nvram, &actual);
    g_assert_cmpint(actual.new_clock_chip, ==, expected.new_clock_chip);
    g_assert_cmpint(actual.auto_poweron, ==, expected.auto_poweron);
    g_assert_cmpint(actual.use_console_slot, ==, expected.use_console_slot);
    g_assert_cmpuint(actual.console_slot, ==, expected.console_slot);
}

static void test_simm_big_endian(void)
{
    NextNVRAMState nvram;

    next_nvram_init(&nvram);
    g_assert_cmphex(next_nvram_get_simm(&nvram), ==, 0xfb6d);

    next_nvram_set_simm(&nvram, 0x1234);
    g_assert_cmphex(nvram.data[NEXT_NVRAM_SIMM], ==, 0x12);
    g_assert_cmphex(nvram.data[NEXT_NVRAM_SIMM + 1], ==, 0x34);
    g_assert_cmphex(next_nvram_get_simm(&nvram), ==, 0x1234);
}

static void test_explicit_checksum_update(void)
{
    NextNVRAMState nvram;
    uint16_t stored;
    uint8_t checksum_high;
    uint8_t checksum_low;

    next_nvram_init(&nvram);
    stored = next_nvram_get_stored_checksum(&nvram);
    checksum_high = nvram.data[NEXT_NVRAM_CHECKSUM];
    checksum_low = nvram.data[NEXT_NVRAM_CHECKSUM + 1];

    next_nvram_write(&nvram, NEXT_NVRAM_EP, 0xa5);
    g_assert_cmphex(next_nvram_read(&nvram, NEXT_NVRAM_EP), ==, 0xa5);
    g_assert_cmphex(next_nvram_get_stored_checksum(&nvram), ==, stored);
    g_assert_cmphex(nvram.data[NEXT_NVRAM_CHECKSUM], ==, checksum_high);
    g_assert_cmphex(nvram.data[NEXT_NVRAM_CHECKSUM + 1], ==, checksum_low);
    g_assert_false(next_nvram_checksum_is_valid(&nvram));

    next_nvram_update_checksum(&nvram);
    g_assert_true(next_nvram_checksum_is_valid(&nvram));
    g_assert_cmphex(next_nvram_get_stored_checksum(&nvram), ==,
                    next_nvram_compute_checksum(&nvram));
}

static void test_raw_out_of_range(void)
{
    NextNVRAMState nvram;
    NextNVRAMState before;

    next_nvram_init(&nvram);
    nvram.filename = (char *)(uintptr_t)UINTPTR_MAX;
    nvram.fd = 0x12345678;
    nvram.dirty = true;
    nvram.write_error_reported = true;
    memcpy(&before, &nvram, sizeof(before));

    g_assert_cmphex(next_nvram_read(&nvram, NEXT_NVRAM_SIZE), ==, 0);
    g_assert_cmphex(next_nvram_read(&nvram, UINT_MAX), ==, 0);
    next_nvram_write(&nvram, NEXT_NVRAM_SIZE, 0x00);
    next_nvram_write(&nvram, UINT_MAX, 0x5a);
    g_assert_cmpmem(&nvram, sizeof(nvram), &before, sizeof(before));
}

static void test_file_create_defaults(void)
{
    NVRAMTempFile temp = nvram_temp_file_new();
    NextNVRAMState nvram;
    g_autofree char *contents = NULL;
    g_autoptr(GError) err = NULL;
    gsize length;
    struct stat st;

    next_nvram_init(&nvram);
    nvram.filename = temp.path;
    g_assert_true(next_nvram_realize(&nvram, &error_abort));
    g_assert_cmpint(nvram.fd, >=, 0);
    g_assert_cmpint(fstat(nvram.fd, &st), ==, 0);
    g_assert_cmpuint(st.st_mode & 0777, ==, 0600);
    g_assert_cmpint(st.st_size, ==, NEXT_NVRAM_SIZE);
    g_assert_true(g_file_get_contents(temp.path, &contents, &length, &err));
    g_assert_no_error(err);
    g_assert_cmpuint(length, ==, NEXT_NVRAM_SIZE);
    g_assert_cmpmem(contents, length, expected_nvram,
                    sizeof(expected_nvram));

    next_nvram_write(&nvram, NEXT_NVRAM_EP, 0xa5);
    g_clear_pointer(&contents, g_free);
    g_assert_true(g_file_get_contents(temp.path, &contents, &length, &err));
    g_assert_no_error(err);
    g_assert_cmphex((uint8_t)contents[NEXT_NVRAM_EP], ==, 0xa5);

    next_nvram_unrealize(&nvram);
    nvram_temp_file_clear(&temp);
}

static void test_file_ephemeral(void)
{
    NextNVRAMState nvram;

    next_nvram_init(&nvram);
    g_assert_true(next_nvram_realize(&nvram, &error_abort));
    g_assert_cmpint(nvram.fd, ==, -1);

    next_nvram_write(&nvram, NEXT_NVRAM_EP, 0xa5);
    g_assert_cmphex(nvram.data[NEXT_NVRAM_EP], ==, 0xa5);
    g_assert_true(next_nvram_flush(&nvram, &error_abort));
    next_nvram_unrealize(&nvram);
    g_assert_cmpint(nvram.fd, ==, -1);
}

static void test_file_load_raw_bad_checksum(void)
{
    NVRAMTempFile temp = nvram_temp_file_new();
    NextNVRAMState nvram;
    uint8_t raw[NEXT_NVRAM_SIZE];
    g_autoptr(GError) err = NULL;

    memcpy(raw, expected_nvram, sizeof(raw));
    raw[NEXT_NVRAM_EP] ^= 0xa5;
    g_assert_true(g_file_set_contents(temp.path, (char *)raw, sizeof(raw),
                                     &err));
    g_assert_no_error(err);

    next_nvram_init(&nvram);
    nvram.filename = temp.path;
    g_assert_true(next_nvram_realize(&nvram, &error_abort));
    g_assert_cmpmem(nvram.data, sizeof(nvram.data), raw, sizeof(raw));
    g_assert_false(next_nvram_checksum_is_valid(&nvram));

    next_nvram_unrealize(&nvram);
    nvram_temp_file_clear(&temp);
}

static void test_file_size_rejected(gconstpointer opaque)
{
    const size_t size = GPOINTER_TO_SIZE(opaque);
    NVRAMTempFile temp = nvram_temp_file_new();
    NextNVRAMState nvram;
    g_autofree uint8_t *raw = g_malloc0(size);
    Error *err = NULL;
    g_autoptr(GError) gerr = NULL;

    g_assert_true(g_file_set_contents(temp.path, (char *)raw, size, &gerr));
    g_assert_no_error(gerr);
    next_nvram_init(&nvram);
    nvram.filename = temp.path;

    g_assert_false(next_nvram_realize(&nvram, &err));
    g_assert_nonnull(err);
    g_assert_cmpint(nvram.fd, ==, -1);
    g_assert_true(g_file_test(temp.path, G_FILE_TEST_IS_REGULAR));
    error_free(err);

    nvram_temp_file_clear(&temp);
}

static void test_file_lock_contention(void)
{
    NVRAMTempFile temp = nvram_temp_file_new();
    NextNVRAMState first;
    NextNVRAMState second;
    Error *err = NULL;

#ifdef _WIN32
    /*
     * LockFileEx locks are tied to handles, so independent opens in this
     * process are sufficient to test contention.
     */
#else
    if (!qemu_has_ofd_lock()) {
        g_test_skip("same-process lock contention requires OFD locks");
        nvram_temp_file_clear(&temp);
        return;
    }
#endif

    next_nvram_init(&first);
    first.filename = temp.path;
    g_assert_true(next_nvram_realize(&first, &error_abort));

    next_nvram_init(&second);
    second.filename = temp.path;
    g_assert_false(next_nvram_realize(&second, &err));
    g_assert_nonnull(err);
    g_assert_cmpint(second.fd, ==, -1);
    error_free(err);

    next_nvram_unrealize(&first);
    nvram_temp_file_clear(&temp);
}

static void test_file_dirty_recovery(void)
{
    static const unsigned first_address = NEXT_NVRAM_EP;
    static const unsigned second_address = NEXT_NVRAM_EP + 1;
    NVRAMTempFile temp = nvram_temp_file_new();
    NextNVRAMState nvram;
    g_autofree char *contents = NULL;
    g_autoptr(GError) err = NULL;
    gsize length;
    int recovery_fd;
    int stale_fd;

    next_nvram_init(&nvram);
    nvram.filename = temp.path;
    g_assert_true(next_nvram_realize(&nvram, &error_abort));

    recovery_fd = dup(nvram.fd);
    g_assert_cmpint(recovery_fd, >=, 0);
    stale_fd = nvram.fd;
    g_assert_cmpint(close(stale_fd), ==, 0);
    nvram.fd = stale_fd;

    next_nvram_write(&nvram, first_address, 0xa5);
    g_assert_true(nvram.dirty);
    g_assert_true(nvram.write_error_reported);

    nvram.fd = recovery_fd;
    next_nvram_write(&nvram, second_address, 0x5a);
    g_assert_false(nvram.dirty);
    g_assert_false(nvram.write_error_reported);

    g_assert_true(g_file_get_contents(temp.path, &contents, &length, &err));
    g_assert_no_error(err);
    g_assert_cmpuint(length, ==, NEXT_NVRAM_SIZE);
    g_assert_cmphex((uint8_t)contents[first_address], ==, 0xa5);
    g_assert_cmphex((uint8_t)contents[second_address], ==, 0x5a);
    g_assert_cmpmem(contents, first_address, expected_nvram, first_address);
    g_assert_cmpmem(contents + second_address + 1,
                    NEXT_NVRAM_SIZE - second_address - 1,
                    expected_nvram + second_address + 1,
                    NEXT_NVRAM_SIZE - second_address - 1);

    next_nvram_unrealize(&nvram);
    nvram_temp_file_clear(&temp);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    g_test_add_func("/next-nvram/defaults", test_defaults);
    g_test_add_func("/next-nvram/settings-round-trip",
                    test_settings_round_trip);
    g_test_add_func("/next-nvram/settings-decode-layout",
                    test_settings_decode_layout);
    g_test_add_func("/next-nvram/settings-encode-layout",
                    test_settings_encode_layout);
    g_test_add_func("/next-nvram/clock-config-decode-layout",
                    test_clock_config_decode_layout);
    g_test_add_func("/next-nvram/clock-config-encode-layout",
                    test_clock_config_encode_layout);
    g_test_add_func("/next-nvram/clock-config-round-trip",
                    test_clock_config_round_trip);
    g_test_add_func("/next-nvram/simm-big-endian", test_simm_big_endian);
    g_test_add_func("/next-nvram/explicit-checksum-update",
                    test_explicit_checksum_update);
    g_test_add_func("/next-nvram/raw-out-of-range",
                    test_raw_out_of_range);
    g_test_add_func("/next-nvram/file/create-defaults",
                    test_file_create_defaults);
    g_test_add_func("/next-nvram/file/ephemeral",
                    test_file_ephemeral);
    g_test_add_func("/next-nvram/file/load-raw-bad-checksum",
                    test_file_load_raw_bad_checksum);
    g_test_add_data_func("/next-nvram/file/reject-size-31",
                         GSIZE_TO_POINTER(NEXT_NVRAM_SIZE - 1),
                         test_file_size_rejected);
    g_test_add_data_func("/next-nvram/file/reject-size-33",
                         GSIZE_TO_POINTER(NEXT_NVRAM_SIZE + 1),
                         test_file_size_rejected);
    g_test_add_func("/next-nvram/file/lock-contention",
                    test_file_lock_contention);
    g_test_add_func("/next-nvram/file/dirty-recovery",
                    test_file_dirty_recovery);

    return g_test_run();
}
