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
    NextNVRAMState nvram;
    NextNVRAMClockConfig config;

    next_nvram_init(&nvram);
    nvram.data[NEXT_NVRAM_CLOCK_CONFIG] = 0xab;

    next_nvram_decode_clock_config(&nvram, &config);

    g_assert_true(config.new_clock_chip);
    g_assert_false(config.auto_poweron);
    g_assert_true(config.use_console_slot);
    g_assert_cmpuint(config.console_slot, ==, 1);
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
    next_nvram_write(&nvram, NEXT_NVRAM_SIZE, 0xff);
    next_nvram_write(&nvram, UINT_MAX, 0xff);
    g_assert_cmpmem(&nvram, sizeof(nvram), &before, sizeof(before));
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
    g_test_add_func("/next-nvram/clock-config-round-trip",
                    test_clock_config_round_trip);
    g_test_add_func("/next-nvram/simm-big-endian", test_simm_big_endian);
    g_test_add_func("/next-nvram/explicit-checksum-update",
                    test_explicit_checksum_update);
    g_test_add_func("/next-nvram/raw-out-of-range",
                    test_raw_out_of_range);

    return g_test_run();
}
