/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026 Bryce Lanham */
/* Bt463 register model tests. */

#include "qemu/osdep.h"
#include "hw/display/bt463.h"
#include "io/channel-buffer.h"
#include "migration/qemu-file.h"
#include "qemu/module.h"

static void test_bt463_import_legacy(void)
{
    Bt463State state;
    Bt463LegacyState legacy = { 0 };

    legacy.dac_address = 0x3abc;
    legacy.dac_component = 5;
    legacy.palette[0x000][0] = 0x12;
    legacy.palette[0x20f][2] = 0x34;
    legacy.general[0x100][0] = 0x21;
    legacy.general[0x100][1] = 0x32;
    legacy.general[0x100][2] = 0x43;
    legacy.general[0x101][0] = 0x54;
    legacy.general[0x101][1] = 0x65;
    legacy.general[0x101][2] = 0x76;
    legacy.general[0x201][0] = 0xff;
    legacy.general[0x202][0] = 0xff;
    legacy.general[0x203][0] = 0xff;
    for (unsigned i = 0; i < 4; i++) {
        legacy.general[0x205 + i][0] = 0x10 + i;
        legacy.general[0x209 + i][0] = 0x20 + i;
    }
    legacy.general[0x20d][0] = 0x87;
    legacy.general[0x20e][0] = 0x34;
    legacy.general[0x20e][1] = 0x12;
    legacy.general[0x20f][0] = 0xa1;
    legacy.general[0x20f][1] = 0xb2;
    legacy.general[0x20f][2] = 0xc3;
    legacy.general[0x220][0] = 0xff;
    legacy.general[0x300][0] = 0x11;
    legacy.general[0x300][1] = 0x22;
    legacy.general[0x300][2] = 0x33;
    legacy.general[0x30f][0] = 0xaa;
    legacy.general[0x30f][1] = 0xbb;
    legacy.general[0x30f][2] = 0xcc;

    bt463_init(&state);
    bt463_import_legacy(&state, &legacy);

    g_assert_cmpuint(state.address, ==, 0x02bc);
    g_assert_cmpuint(state.component, ==, 2);
    g_assert_cmpuint(state.palette[0x000][0], ==, 0x12);
    g_assert_cmpuint(state.palette[0x20f][2], ==, 0x34);
    g_assert_cmpmem(state.cursor[0], sizeof(state.cursor[0]),
                    legacy.general[0x100], 3);
    g_assert_cmpmem(state.cursor[1], sizeof(state.cursor[1]),
                    legacy.general[0x101], 3);
    g_assert_cmpuint(state.command[0], ==, 0xcc);
    g_assert_cmpuint(state.command[1], ==, 0x7f);
    g_assert_cmpuint(state.command[2], ==, 0xc7);
    for (unsigned i = 0; i < 4; i++) {
        g_assert_cmpuint(state.read_mask[i], ==, 0x10 + i);
        g_assert_cmpuint(state.blink_mask[i], ==, 0x20 + i);
    }
    g_assert_cmpuint(state.test_register, ==, 0x87);
    g_assert_cmpuint(state.input_signature, ==, 0x1234);
    g_assert_cmpmem(state.output_signature, sizeof(state.output_signature),
                    legacy.general[0x20f], 3);
    g_assert_cmphex(state.wtt[0], ==, 0x332211);
    g_assert_cmphex(state.wtt[15], ==, 0xccbbaa);
    g_assert_cmpuint(vmstate_bt463_legacy.version_id, ==, 1);
}

static void test_bt463_import_legacy_wtt_latches(void)
{
    Bt463State read_state;
    Bt463State write_state;
    Bt463LegacyState read_legacy = { 0 };
    Bt463LegacyState write_legacy = { 0 };

    read_legacy.dac_address = 0x0301;
    read_legacy.dac_component = 1;
    read_legacy.general[0x301][0] = 0xa1;
    read_legacy.general[0x301][1] = 0xb2;
    read_legacy.general[0x301][2] = 0xc3;
    bt463_init(&read_state);
    bt463_import_legacy(&read_state, &read_legacy);

    /* A partial legacy read resumes from the imported WTT entry. */
    g_assert_cmphex(bt463_general_read(&read_state), ==, 0xb2);
    g_assert_cmphex(bt463_general_read(&read_state), ==, 0xc3);
    g_assert_cmpuint(read_state.address, ==, 0x302);
    g_assert_cmpuint(read_state.component, ==, 0);

    write_legacy.dac_address = 0x0302;
    write_legacy.dac_component = 1;
    write_legacy.general[0x302][0] = 0x11;
    write_legacy.general[0x302][1] = 0x22;
    write_legacy.general[0x302][2] = 0x33;
    bt463_init(&write_state);
    bt463_import_legacy(&write_state, &write_legacy);

    /* A partial legacy write preserves the untouched red component. */
    bt463_general_write(&write_state, 0xaa);
    bt463_general_write(&write_state, 0xbb);
    bt463_address_write(&write_state, false, 0x02);
    bt463_address_write(&write_state, true, 0x03);
    g_assert_cmphex(bt463_general_read(&write_state), ==, 0x11);
    g_assert_cmphex(bt463_general_read(&write_state), ==, 0xaa);
    g_assert_cmphex(bt463_general_read(&write_state), ==, 0xbb);
}

static void test_bt463_legacy_vmstate(void)
{
    Bt463LegacyState source = { 0 };
    Bt463LegacyState destination = { 0 };
    QIOChannelBuffer *save_channel = qio_channel_buffer_new(0);
    QIOChannelBuffer *load_channel;
    QEMUFile *file;
    g_autofree uint8_t *wire = NULL;
    Error *local_err = NULL;
    size_t wire_size;

    source.dac_address = 0x0123;
    source.dac_component = 1;
    source.palette[0x20f][2] = 0x5a;
    source.general[0x201][0] = 0xcc;
    source.general[0x300][0] = 0x11;
    source.general[0x300][1] = 0x22;
    source.general[0x300][2] = 0x33;

    file = qemu_file_new_output(QIO_CHANNEL(save_channel));
    g_assert_cmpint(vmstate_save_state(file, &vmstate_bt463_legacy, &source,
                                       NULL, &local_err), ==, 0);
    g_assert_null(local_err);
    g_assert_cmpint(qemu_fflush(file), ==, 0);
    wire_size = save_channel->usage;
    g_assert_cmpuint(wire_size, ==, 2 + 1 + (0x400 * 3) + (0x400 * 3));
    wire = g_memdup2(save_channel->data, wire_size);
    qemu_fclose(file);

    load_channel = qio_channel_buffer_new(wire_size);
    memcpy(load_channel->data, wire, wire_size);
    load_channel->usage = wire_size;
    load_channel->offset = 0;
    file = qemu_file_new_input(QIO_CHANNEL(load_channel));
    g_assert_cmpint(vmstate_load_state(file, &vmstate_bt463_legacy,
                                       &destination, 1, &local_err), ==, 0);
    g_assert_null(local_err);
    qemu_fclose(file);
    object_unref(OBJECT(save_channel));
    object_unref(OBJECT(load_channel));

    g_assert_cmpuint(destination.dac_address, ==, source.dac_address);
    g_assert_cmpuint(destination.dac_component, ==, source.dac_component);
    g_assert_cmpuint(destination.palette[0x20f][2], ==, 0x5a);
    g_assert_cmpuint(destination.general[0x201][0], ==, 0xcc);
    g_assert_cmpuint(destination.general[0x300][2], ==, 0x33);
}

static void test_bt463_vmstate(void)
{
    Bt463State source;
    Bt463State destination;
    QIOChannelBuffer *save_channel = qio_channel_buffer_new(0);
    QIOChannelBuffer *load_channel;
    QEMUFile *file;
    g_autofree uint8_t *wire = NULL;
    Error *local_err = NULL;
    size_t wire_size;
    size_t old_wire_size;

    bt463_init(&source);
    source.address = 0x0305;
    source.component = 2;
    source.palette[0x20f][2] = 0x5a;
    source.cursor[0][0] = 0x11;
    source.cursor[1][2] = 0x22;
    source.command[0] = 0x44;
    source.command[1] = 0x07;
    source.read_mask[3] = 0x0f;
    source.blink_mask[3] = 0x03;
    source.wtt[0x05] = 0x332211;
    source.wtt_write_latch = 0x665544;
    source.wtt_read_latch = 0x998877;
    source.blink_counter = 7;
    source.blink_phase = false;

    file = qemu_file_new_output(QIO_CHANNEL(save_channel));
    g_assert_cmpint(vmstate_save_state(file, &vmstate_bt463, &source,
                                       NULL, &local_err), ==, 0);
    g_assert_null(local_err);
    g_assert_cmpint(qemu_fflush(file), ==, 0);
    wire_size = save_channel->usage;
    g_assert_cmpuint(vmstate_bt463.version_id, ==, 2);
    g_assert_cmpuint(wire_size, >, 2);
    wire = g_memdup2(save_channel->data, wire_size);
    qemu_fclose(file);

    /* Current v2 nested state preserves every newly added blink field. */
    load_channel = qio_channel_buffer_new(wire_size);
    memcpy(load_channel->data, wire, wire_size);
    load_channel->usage = wire_size;
    load_channel->offset = 0;
    bt463_init(&destination);
    file = qemu_file_new_input(QIO_CHANNEL(load_channel));
    g_assert_cmpint(vmstate_load_state(file, &vmstate_bt463, &destination, 2,
                                       &local_err), ==, 0);
    g_assert_null(local_err);
    qemu_fclose(file);
    object_unref(OBJECT(load_channel));

    g_assert_cmpuint(destination.address, ==, source.address);
    g_assert_cmpuint(destination.component, ==, source.component);
    g_assert_cmpuint(destination.palette[0x20f][2], ==, 0x5a);
    g_assert_cmpuint(destination.cursor[0][0], ==, 0x11);
    g_assert_cmpuint(destination.cursor[1][2], ==, 0x22);
    g_assert_cmpuint(destination.command[1], ==, source.command[1]);
    g_assert_cmphex(destination.wtt[0x05], ==, 0x332211);
    g_assert_cmphex(destination.wtt_write_latch, ==, 0x665544);
    g_assert_cmphex(destination.wtt_read_latch, ==, 0x998877);
    g_assert_cmpuint(destination.blink_counter, ==, 7);
    g_assert_false(destination.blink_phase);

    /* A v1 nested stream has the same prefix but no blink fields. */
    old_wire_size = wire_size - 2;
    load_channel = qio_channel_buffer_new(old_wire_size);
    memcpy(load_channel->data, wire, old_wire_size);
    load_channel->usage = old_wire_size;
    load_channel->offset = 0;
    destination.blink_counter = 99;
    destination.blink_phase = false;
    local_err = NULL;
    file = qemu_file_new_input(QIO_CHANNEL(load_channel));
    g_assert_cmpint(vmstate_load_state(file, &vmstate_bt463, &destination, 1,
                                       &local_err), ==, 0);
    g_assert_null(local_err);
    qemu_fclose(file);
    object_unref(OBJECT(load_channel));
    object_unref(OBJECT(save_channel));

    g_assert_cmphex(destination.wtt[0x05], ==, 0x332211);
    g_assert_cmpuint(destination.blink_counter, ==, 0);
    g_assert_true(destination.blink_phase);
}

static void test_bt463_reset_every_state(void)
{
    Bt463State state;
    const uint8_t zero_rgb[3] = { 0, 0, 0 };
    const uint8_t zero_bytes[4] = { 0, 0, 0, 0 };

    bt463_init(&state);
    state.address = 0xabc;
    state.component = 2;
    for (unsigned address = 0; address < BT463_PALETTE_ENTRIES; address++) {
        for (unsigned component = 0; component < 3; component++) {
            state.palette[address][component] =
                (address + component + 1) & 0xff;
        }
    }
    for (unsigned cursor = 0; cursor < BT463_CURSOR_COLORS; cursor++) {
        for (unsigned component = 0; component < 3; component++) {
            state.cursor[cursor][component] =
                0x10 * (cursor + 1) + component;
        }
    }
    state.command[0] = 0x40;
    state.command[1] = 0x20;
    state.command[2] = 0x80;
    for (unsigned i = 0; i < 4; i++) {
        state.read_mask[i] = 0x10 + i;
        state.blink_mask[i] = 0x20 + i;
    }
    state.test_register = 0xa5;
    state.input_signature = 0x1234;
    state.output_signature[0] = 0x56;
    state.output_signature[1] = 0x78;
    state.output_signature[2] = 0x9a;
    for (unsigned i = 0; i < BT463_WTT_ENTRIES; i++) {
        state.wtt[i] = 0x10000 + i;
    }
    state.wtt_write_latch = 0xabcdef;
    state.wtt_read_latch = 0x123456;
    state.blink_counter = 31;
    state.blink_phase = false;

    bt463_reset(&state);

    g_assert_cmpuint(state.address, ==, 0);
    g_assert_cmpuint(state.component, ==, 0);
    for (unsigned address = 0; address < BT463_PALETTE_ENTRIES; address++) {
        g_assert_cmpmem(state.palette[address], sizeof(zero_rgb),
                        zero_rgb, sizeof(zero_rgb));
    }
    for (unsigned cursor = 0; cursor < BT463_CURSOR_COLORS; cursor++) {
        g_assert_cmpmem(state.cursor[cursor], sizeof(zero_rgb),
                        zero_rgb, sizeof(zero_rgb));
    }
    g_assert_cmpmem(state.command, sizeof(state.command), zero_rgb,
                    sizeof(zero_rgb));
    g_assert_cmpmem(state.read_mask, sizeof(state.read_mask), zero_bytes,
                    sizeof(zero_bytes));
    g_assert_cmpmem(state.blink_mask, sizeof(state.blink_mask), zero_bytes,
                    sizeof(zero_bytes));
    g_assert_cmpuint(state.test_register, ==, 0);
    g_assert_cmpuint(state.input_signature, ==, 0);
    g_assert_cmpmem(state.output_signature, sizeof(state.output_signature),
                    zero_rgb, sizeof(zero_rgb));
    for (unsigned i = 0; i < BT463_WTT_ENTRIES; i++) {
        g_assert_cmpuint(state.wtt[i], ==, 0);
    }
    g_assert_cmpuint(state.wtt_write_latch, ==, 0);
    g_assert_cmpuint(state.wtt_read_latch, ==, 0);
    g_assert_cmpuint(state.blink_counter, ==, 0);
    g_assert_true(state.blink_phase);
}

static uint32_t test_bt463_wtt(unsigned shift, unsigned planes,
                               unsigned mode, unsigned overlay_location,
                               unsigned overlay_mask, unsigned start,
                               bool bypass)
{
    return (shift & 0x1f) |
        ((planes & 0xf) << 5) |
        ((mode & 0x7) << 9) |
        ((overlay_location & 1) << 12) |
        ((overlay_mask & 0xf) << 13) |
        ((start & 0x3f) << 17) |
        ((bypass & 1) << 23);
}

static uint32_t test_bt463_warp9c_pins(uint8_t red, uint8_t green,
                                       uint8_t blue, uint8_t overlay)
{
    return ((uint32_t)(red & 0xf) << 4) |
        ((uint32_t)(green & 0xf) << 12) |
        ((uint32_t)(blue & 0xf) << 20) |
        (overlay & 0xf) << 24;
}

static void test_bt463_lookup_true_color(void)
{
    Bt463State state;
    uint32_t compact_pixel;

    bt463_init(&state);
    state.read_mask[0] = 0xf0;
    state.read_mask[1] = 0xf0;
    state.read_mask[2] = 0xf0;
    state.wtt[0] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                  0, 0, 0, false);
    state.palette[0xa0][0] = 0x12;
    state.palette[0xb0][1] = 0x34;
    state.palette[0xc0][2] = 0x56;

    /* Read masks select P4-P7, P12-P15 and P20-P23 before WTT decoding. */
    g_assert_cmphex(bt463_lookup_rgb(&state,
                                    test_bt463_warp9c_pins(0xa, 0xb, 0xc,
                                                           0),
                                    0, BT463_LOAD_LOWER),
                    ==, 0x123456);

    /*
     * A shift of four compacts those same physical nibbles into each
     * channel's least-significant four bits.
     */
    state.wtt[0] = test_bt463_wtt(4, 8, BT463_WTT_TRUE_COLOR,
                                  0, 0, 0, false);
    state.palette[0x0a][0] = 0x21;
    state.palette[0x0b][1] = 0x43;
    state.palette[0x0c][2] = 0x65;
    g_assert_cmphex(bt463_lookup_rgb(&state,
                                    test_bt463_warp9c_pins(0xa, 0xb, 0xc,
                                                           0),
                                    0, BT463_LOAD_LOWER),
                    ==, 0x214365);

    /* Reduced true color keeps each channel in its fixed input octet. */
    state.read_mask[0] = 0xff;
    state.read_mask[1] = 0xff;
    state.read_mask[2] = 0xff;
    state.wtt[0] = test_bt463_wtt(0, 4, BT463_WTT_TRUE_COLOR,
                                  0, 0, 0, false);
    compact_pixel = 0x00c5b6a7;
    state.palette[0x07][0] = 0x31;
    state.palette[0x06][1] = 0x42;
    state.palette[0x05][2] = 0x53;
    g_assert_cmphex(bt463_lookup_rgb(&state, compact_pixel, 0,
                                    BT463_LOAD_LOWER),
                    ==, 0x314253);
}

static void test_bt463_lookup_contiguous_planes(void)
{
    Bt463State state;
    uint32_t pixel;

    bt463_init(&state);
    memset(state.read_mask, 0xff, sizeof(state.read_mask));
    state.command[1] = 0x20; /* CR15: 12/16-plane contiguous input. */

    /* CR15 true color is ordered R=P3:0, B=P7:4, G=P11:8. */
    pixel = (1U << 0) | (2U << 4) | (3U << 8);
    state.wtt[0] = test_bt463_wtt(0, 4, BT463_WTT_TRUE_COLOR,
                                  0, 0, 0, false);
    state.palette[1][0] = 0x11;
    state.palette[2][2] = 0x33;
    state.palette[3][1] = 0x22;
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0x112233);

    /* A nonzero shift is not legal for contiguous true color. */
    state.palette[2][0] = 0x90;
    state.palette[0][1] = 0x91;
    state.palette[0][2] = 0x92;
    state.wtt[0] = test_bt463_wtt(4, 4, BT463_WTT_TRUE_COLOR,
                                  0, 0, 0, false);
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0);

    /* CR15 leaves pseudo-color contiguous, including its legal high shift. */
    pixel = 0x1abU << 11;
    state.wtt[0] = test_bt463_wtt(11, 9, BT463_WTT_PSEUDO_COLOR,
                                  0, 0, 0, false);
    state.palette[0x1ab][0] = 0x41;
    state.palette[0x1ab][1] = 0x52;
    state.palette[0x1ab][2] = 0x63;
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0x415263);
    pixel = 0x1abU << 15;
    state.wtt[0] = test_bt463_wtt(15, 9, BT463_WTT_PSEUDO_COLOR,
                                  0, 0, 0, false);
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0x415263);
    pixel = 0x1abU << 16;
    state.wtt[0] = test_bt463_wtt(16, 9, BT463_WTT_PSEUDO_COLOR,
                                  0, 0, 0, false);
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0);

    /* In the 16-plane overlay wiring, normal overlays are P15:P12. */
    state.wtt[0] = test_bt463_wtt(0, 4, BT463_WTT_TRUE_COLOR,
                                  0, 0xf, 0, false);
    state.palette[0x20f][0] = 0xa1;
    state.palette[0x20f][1] = 0xa2;
    state.palette[0x20f][2] = 0xa3;
    pixel = (0xfU << 12) | (1U << 24);
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0xa1a2a3);

    /* Alternate true-color overlays use P5, P0, P8, P4 as OL3:OL0. */
    state.wtt[0] = test_bt463_wtt(0, 4, BT463_WTT_TRUE_COLOR,
                                  1, 0xf, 0, false);
    pixel = (1U << 0) | (1U << 4) | (1U << 5) | (1U << 8);
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0xa1a2a3);
}

static void test_bt463_lookup_modes(void)
{
    Bt463State state;
    uint32_t pixel;

    bt463_init(&state);
    state.read_mask[0] = 0xff;
    state.read_mask[1] = 0xff;
    state.read_mask[2] = 0xff;
    state.read_mask[3] = 0x0f;
    pixel = 0x00000000 | (0x0b << 8) | 0x0a | (0x0c << 16) |
        (1U << 24);

    /* Pseudo color addresses all three DACs with one compact pixel index. */
    state.wtt[0] = test_bt463_wtt(0, 4, BT463_WTT_PSEUDO_COLOR,
                                  0, 0, 2, false);
    state.palette[0x1a][0] = 0x11;
    state.palette[0x1a][1] = 0x22;
    state.palette[0x1a][2] = 0x33;
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0x112233);

    /* Bank select prepends the selected overlay planes to pixel data. */
    state.wtt[0] = test_bt463_wtt(0, 4, BT463_WTT_BANK_SELECT,
                                  0, 1, 0, false);
    state.palette[0x1a][0] = 0x44;
    state.palette[0x1a][1] = 0x55;
    state.palette[0x1a][2] = 0x66;
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0x445566);

    /* Overlay pins remain fixed at P24-P27 when pixel data is shifted. */
    state.wtt[0] = test_bt463_wtt(4, 4, BT463_WTT_BANK_SELECT,
                                  0, 1, 0, false);
    g_assert_cmphex(bt463_lookup_rgb(&state,
                                    test_bt463_warp9c_pins(0xa, 0, 0, 1),
                                    0, BT463_LOAD_LOWER),
                    ==, 0x445566);

    /*
     * The fixed overlay source is independent of the shifted pixel-plane
     * budget, so all four planes may begin at P24.
     */
    state.wtt[0] = test_bt463_wtt(24, 4, BT463_WTT_BANK_SELECT,
                                  0, 1, 0, false);
    state.palette[0x11][0] = 0x47;
    state.palette[0x11][1] = 0x58;
    state.palette[0x11][2] = 0x69;
    g_assert_cmphex(bt463_lookup_rgb(&state, 1U << 24, 0,
                                    BT463_LOAD_LOWER),
                    ==, 0x475869);

    /* Lookup bypass in true-color mode drives each channel directly. */
    state.wtt[0] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                  0, 0, 0, true);
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0x0a0b0c);

    /* Lookup bypass in pseudo-color mode drives one value to all DACs. */
    state.wtt[0] = test_bt463_wtt(0, 8, BT463_WTT_PSEUDO_COLOR,
                                  0, 0, 0, true);
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0x0a0a0a);
}

static void test_bt463_lookup_load_interleave(void)
{
    Bt463State state;
    uint32_t pixel = 0x00000000 | (0x2a << 0) | (0x3b << 8) |
        (0x4c << 16);

    bt463_init(&state);
    state.read_mask[0] = 0xff;
    state.read_mask[1] = 0xff;
    state.read_mask[2] = 0xff;

    state.wtt[0] = test_bt463_wtt(0, 4, BT463_TRUE_COLOR_LOAD_INTERLEAVE,
                                  0, 0, 0, false);
    state.palette[0x0a][0] = 0x10;
    state.palette[0x0b][1] = 0x20;
    state.palette[0x0c][2] = 0x30;
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0x102030);

    state.palette[0x02][0] = 0x40;
    state.palette[0x03][1] = 0x50;
    state.palette[0x04][2] = 0x60;
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_UPPER),
                    ==, 0x405060);

    state.wtt[0] = test_bt463_wtt(0, 8, BT463_PSEUDO_COLOR_LOAD_INTERLEAVE,
                                  0, 0, 0, false);
    state.palette[0xba][0] = 0x71;
    state.palette[0xba][1] = 0x72;
    state.palette[0xba][2] = 0x73;
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0x717273);
    state.palette[0x32][0] = 0x81;
    state.palette[0x32][1] = 0x82;
    state.palette[0x32][2] = 0x83;
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_UPPER),
                    ==, 0x818283);

    /* Shift four selects the upper nibble first but leaves octets fixed. */
    state.wtt[0] = test_bt463_wtt(4, 4, BT463_TRUE_COLOR_LOAD_INTERLEAVE,
                                  0, 0, 0, false);
    state.command[0] = 0x80;
    g_assert_cmpuint(bt463_load_phase_seed(&state, 0), ==, BT463_LOAD_UPPER);
    g_assert_cmpuint(bt463_load_phase_at(&state, BT463_LOAD_UPPER, 0), ==,
                     BT463_LOAD_UPPER);
    g_assert_cmpuint(bt463_load_phase_at(&state, BT463_LOAD_UPPER, 1), ==,
                     BT463_LOAD_LOWER);
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_UPPER),
                    ==, 0x405060);
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0x102030);

    state.wtt[0] = test_bt463_wtt(0, 4, BT463_TRUE_COLOR_LOAD_INTERLEAVE,
                                  0, 0, 0, false);
    state.command[0] = 0x40;
    g_assert_cmpuint(bt463_load_phase_seed(&state, 0), ==, BT463_LOAD_LOWER);
    g_assert_cmpuint(bt463_load_phase_at(&state, BT463_LOAD_LOWER, 0), ==,
                     BT463_LOAD_LOWER);
    g_assert_cmpuint(bt463_load_phase_at(&state, BT463_LOAD_LOWER, 1), ==,
                     BT463_LOAD_LOWER);
    g_assert_cmpuint(bt463_load_phase_at(&state, BT463_LOAD_LOWER, 2), ==,
                     BT463_LOAD_LOWER);

    /* The first tag seeds the line; later tags cannot re-seed its phase. */
    state.command[0] = 0x80;
    state.wtt[0] = test_bt463_wtt(0, 4, BT463_TRUE_COLOR_LOAD_INTERLEAVE,
                                  0, 0, 0, false);
    state.wtt[1] = test_bt463_wtt(4, 4, BT463_TRUE_COLOR_LOAD_INTERLEAVE,
                                  0, 0, 0, false);
    g_assert_cmpuint(bt463_load_phase_at(&state,
                                         bt463_load_phase_seed(&state, 0), 0),
                     ==, BT463_LOAD_LOWER);
    g_assert_cmpuint(bt463_load_phase_at(&state,
                                         bt463_load_phase_seed(&state, 0), 1),
                     ==, BT463_LOAD_UPPER);
    state.wtt[0] = test_bt463_wtt(4, 4, BT463_TRUE_COLOR_LOAD_INTERLEAVE,
                                  0, 0, 0, false);
    state.wtt[1] = test_bt463_wtt(0, 4, BT463_TRUE_COLOR_LOAD_INTERLEAVE,
                                  0, 0, 0, false);
    g_assert_cmpuint(bt463_load_phase_at(&state,
                                         bt463_load_phase_seed(&state, 0), 0),
                     ==, BT463_LOAD_UPPER);
    g_assert_cmpuint(bt463_load_phase_at(&state,
                                         bt463_load_phase_seed(&state, 0), 1),
                     ==, BT463_LOAD_LOWER);

    /* Multiplexing changes phase only after a complete load cycle. */
    state.command[0] = 0x40; /* 4:1, four displayed pixels per load. */
    for (unsigned i = 0; i < 4; i++) {
        g_assert_cmpuint(bt463_load_phase_at(&state, BT463_LOAD_LOWER, i),
                         ==, BT463_LOAD_LOWER);
    }
    g_assert_cmpuint(bt463_load_phase_at(&state, BT463_LOAD_LOWER, 4), ==,
                     BT463_LOAD_UPPER);
    g_assert_cmpuint(bt463_load_phase_at(&state, BT463_LOAD_LOWER, 7), ==,
                     BT463_LOAD_UPPER);
    g_assert_cmpuint(bt463_load_phase_at(&state, BT463_LOAD_LOWER, 8), ==,
                     BT463_LOAD_LOWER);

    state.command[0] = 0x80; /* 1:1, one displayed pixel per load. */
    g_assert_cmpuint(bt463_load_phase_at(&state, BT463_LOAD_LOWER, 0), ==,
                     BT463_LOAD_LOWER);
    g_assert_cmpuint(bt463_load_phase_at(&state, BT463_LOAD_LOWER, 1), ==,
                     BT463_LOAD_UPPER);
    g_assert_cmpuint(bt463_load_phase_at(&state, BT463_LOAD_LOWER, 2), ==,
                     BT463_LOAD_LOWER);

    state.command[0] = 0xc0; /* 2:1, two displayed pixels per load. */
    g_assert_cmpuint(bt463_load_phase_at(&state, BT463_LOAD_LOWER, 0), ==,
                     BT463_LOAD_LOWER);
    g_assert_cmpuint(bt463_load_phase_at(&state, BT463_LOAD_LOWER, 1), ==,
                     BT463_LOAD_LOWER);
    g_assert_cmpuint(bt463_load_phase_at(&state, BT463_LOAD_LOWER, 2), ==,
                     BT463_LOAD_UPPER);
    g_assert_cmpuint(bt463_load_phase_at(&state, BT463_LOAD_LOWER, 3), ==,
                     BT463_LOAD_UPPER);

    state.command[0] = 0x00; /* Reserved multiplexing selection. */
    g_assert_cmpuint(bt463_load_phase_at(&state, BT463_LOAD_LOWER, 0), ==,
                     BT463_LOAD_INVALID);
}

static void test_bt463_lookup_overlay_and_cursor(void)
{
    Bt463State state;
    uint32_t pixel;

    bt463_init(&state);
    memset(state.read_mask, 0xff, sizeof(state.read_mask));
    state.wtt[0] = test_bt463_wtt(0, 4, BT463_WTT_TRUE_COLOR,
                                  0, 1, 4, false);
    pixel = test_bt463_warp9c_pins(1, 2, 3, 1);
    state.palette[0x21][0] = 0x11;
    state.palette[0x22][1] = 0x22;
    state.palette[0x23][2] = 0x33;
    state.palette[0x11][0] = 0xa1;
    state.palette[0x11][1] = 0xa2;
    state.palette[0x11][2] = 0xa3;

    /* Fixed P24-P27 overlay maps relative to the WTT start row. */
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0xa1a2a3);

    /* The overlay mask compacts OL0 and OL2 into a two-bit value. */
    state.wtt[0] = test_bt463_wtt(0, 4, BT463_WTT_TRUE_COLOR,
                                  0, 5, 4, false);
    pixel = test_bt463_warp9c_pins(1, 2, 3, 5);
    state.palette[0x13][0] = 0x91;
    state.palette[0x13][1] = 0x92;
    state.palette[0x13][2] = 0x93;
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0x919293);

    /* Alternate true-color overlay location uses shifted P16 as OL0. */
    state.wtt[0] = test_bt463_wtt(0, 4, BT463_WTT_TRUE_COLOR,
                                  1, 1, 4, false);
    pixel = test_bt463_warp9c_pins(1, 2, 3, 0) | (1U << 16);
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0xa1a2a3);

    /* CR16 selects the common overlay palette at 0x201-0x20f. */
    state.command[1] = 0x40;
    state.palette[0x201][0] = 0xb1;
    state.palette[0x201][1] = 0xb2;
    state.palette[0x201][2] = 0xb3;
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0xb1b2b3);

    /* A low per-window start also falls back to the common overlay palette. */
    state.command[1] = 0;
    state.wtt[0] = test_bt463_wtt(0, 4, BT463_WTT_TRUE_COLOR,
                                  1, 1, 0, false);
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0xb1b2b3);

    /* A selected overlay outside the 528-entry RAM is invalid. */
    state.command[1] = 0;
    state.wtt[0] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                  1, 1, 0x3f, true);
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0);

    /* CR13 plus dual-cursor mode maps WT E/F to direct cursor colors. */
    state.command[1] = 0x0a;
    /* Table 12 replaces E/F; their WTT storage may contain garbage. */
    state.wtt[0x0e] = test_bt463_wtt(31, 0, BT463_WTT_RESERVED_7,
                                     0, 0, 0, false);
    state.wtt[0x0f] = test_bt463_wtt(31, 0, BT463_WTT_RESERVED_7,
                                     0, 0, 0, false);
    state.cursor[0][0] = 0xc1;
    state.cursor[0][1] = 0xc2;
    state.cursor[0][2] = 0xc3;
    state.cursor[1][0] = 0xd1;
    state.cursor[1][1] = 0xd2;
    state.cursor[1][2] = 0xd3;
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0x0e, BT463_LOAD_LOWER),
                    ==, 0xc1c2c3);
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0x0f, BT463_LOAD_LOWER),
                    ==, 0xd1d2d3);

    /* Table 12 does not alias WT E/F without dual cursor and CR13. */
    state.wtt[0x0e] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                     0, 0, 0, false);
    state.wtt[0x0f] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                     0, 0, 0, false);
    state.palette[0][0] = 0x11;
    state.palette[0][1] = 0x22;
    state.palette[0][2] = 0x33;
    state.command[1] = 0x08; /* CR13, no cursor planes. */
    g_assert_cmphex(bt463_lookup_rgb(&state,
                                    test_bt463_warp9c_pins(0, 0, 0, 0),
                                    0x0e, BT463_LOAD_LOWER), ==, 0x112233);
    state.command[1] = 0x09; /* CR13 plus one cursor plane. */
    g_assert_cmphex(bt463_lookup_rgb(&state,
                                    test_bt463_warp9c_pins(0, 0, 0, 0),
                                    0x0e, BT463_LOAD_LOWER), ==, 0x112233);
    state.command[1] = 0x02; /* Dual cursor without CR13. */
    g_assert_cmphex(bt463_lookup_rgb(&state,
                                    test_bt463_warp9c_pins(0, 0, 0, 0),
                                    0x0e, BT463_LOAD_LOWER), ==, 0x112233);
    state.palette[0x0e0][0] = 0x71;
    state.palette[0x0e0][1] = 0x72;
    state.palette[0x0e0][2] = 0x73;
    state.wtt[0x0e] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                     0, 0, 0x20, false);
    state.wtt[0x0f] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                     0, 0, 0x20, false);
    state.command[1] = 0x1a; /* CR14 repurposes WT0-WT3. */
    g_assert_cmphex(bt463_lookup_rgb(&state,
                                    test_bt463_warp9c_pins(0, 0, 0, 0),
                                    0x0e, BT463_LOAD_LOWER), ==, 0x717273);
}

static uint32_t test_bt463_palette_value(unsigned address)
{
    return ((uint32_t)((address ^ 0x31) & 0xff) << 16) |
        ((uint32_t)((address ^ 0x52) & 0xff) << 8) |
        ((address ^ 0x73) & 0xff);
}

static void test_bt463_fill_palette(Bt463State *state)
{
    for (unsigned address = 0; address < BT463_PALETTE_ENTRIES; address++) {
        uint32_t value = test_bt463_palette_value(address);

        state->palette[address][0] = value >> 16;
        state->palette[address][1] = value >> 8;
        state->palette[address][2] = value;
    }
}

static void test_bt463_lookup_standard_overlay_truth_table(void)
{
    Bt463State state;
    const unsigned start = 4;
    const uint32_t pixel_data = test_bt463_warp9c_pins(0, 0, 0, 0);

    bt463_init(&state);
    memset(state.read_mask, 0xff, sizeof(state.read_mask));
    test_bt463_fill_palette(&state);
    state.wtt[0] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                  0, 0xf, start, false);

    /* Table 9: OL=0 is pixel data; overlays are start-$10+$OL. */
    for (unsigned overlay = 0; overlay < 16; overlay++) {
        uint32_t pixel = pixel_data | (overlay << 24);
        unsigned address = overlay ? 0x10 + overlay : start << 3;

        g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0,
                                         BT463_LOAD_LOWER), ==,
                        test_bt463_palette_value(address));
    }

    /* CR12 splits OL3=1 into overlay and OL3=0 into underlay. */
    state.command[1] = 0x04;
    for (unsigned overlay = 0; overlay < 16; overlay++) {
        uint32_t pixel = pixel_data | (overlay << 24);
        unsigned address = overlay ? 0x10 + overlay : start << 3;

        g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0,
                                         BT463_LOAD_LOWER), ==,
                        test_bt463_palette_value(address));
    }

    /* Underlay is selected only for a zero manipulated pixel value. */
    state.palette[0x30][0] = 0x31;
    state.palette[0x20][1] = 0x22;
    state.palette[0x20][2] = 0x23;
    g_assert_cmphex(bt463_lookup_rgb(&state,
                                    test_bt463_warp9c_pins(1, 0, 0, 1),
                                    0, BT463_LOAD_LOWER), ==, 0x312223);

    /* CR16 maps every selected overlay to the common 0x201-0x20f palette. */
    state.command[1] = 0x40;
    for (unsigned overlay = 1; overlay < 16; overlay++) {
        uint32_t pixel = pixel_data | (overlay << 24);

        g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0,
                                         BT463_LOAD_LOWER), ==,
                        test_bt463_palette_value(0x200 + overlay));
    }

    /* A 16-plane, noncontiguous input uses P15:P12 for the overlay port. */
    state.command[1] = 0x20;
    state.wtt[0] = test_bt463_wtt(0, 4, BT463_WTT_TRUE_COLOR,
                                  0, 0xf, start, false);
    g_assert_cmphex(bt463_lookup_rgb(&state, 0xaU << 12, 0,
                                     BT463_LOAD_LOWER), ==,
                    test_bt463_palette_value(0x1a));
}

static void test_bt463_lookup_single_cursor_truth_table(void)
{
    Bt463State state;
    const unsigned start = 4;

    bt463_init(&state);
    memset(state.read_mask, 0xff, sizeof(state.read_mask));
    test_bt463_fill_palette(&state);
    state.wtt[0] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                  0, 0xf, start, false);
    state.cursor[0][0] = 0xc1;
    state.cursor[0][1] = 0xc2;
    state.cursor[0][2] = 0xc3;
    state.command[1] = 0x01;

    /* Table 10: every OL0=1 value selects cursor color 0. */
    for (unsigned overlay = 1; overlay < 16; overlay += 2) {
        g_assert_cmphex(bt463_lookup_rgb(&state, overlay << 24, 0,
                                         BT463_LOAD_LOWER), ==, 0xc1c2c3);
    }

    /* With underlays disabled all nonzero even values are overlays. */
    for (unsigned overlay = 2; overlay < 16; overlay += 2) {
        g_assert_cmphex(bt463_lookup_rgb(&state, overlay << 24, 0,
                                         BT463_LOAD_LOWER), ==,
                        test_bt463_palette_value(0x10 + overlay));
    }

    /* Routing sees the compacted mask result, not a masked-out raw plane. */
    state.wtt[0] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                  0, 0xe, start, false);
    g_assert_cmphex(bt463_lookup_rgb(&state, 1U << 24, 0,
                                     BT463_LOAD_LOWER), ==,
                    test_bt463_palette_value(start << 3));
    g_assert_cmphex(bt463_lookup_rgb(&state, 2U << 24, 0,
                                     BT463_LOAD_LOWER), ==, 0xc1c2c3);

    /* With underlays enabled, OL3 selects overlay versus underlay. */
    state.wtt[0] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                  0, 0xf, start, false);
    state.command[1] = 0x05;
    for (unsigned overlay = 2; overlay < 8; overlay += 2) {
        g_assert_cmphex(bt463_lookup_rgb(&state, overlay << 24, 0,
                                         BT463_LOAD_LOWER), ==,
                        test_bt463_palette_value(0x10 + overlay));
    }
    for (unsigned overlay = 8; overlay < 16; overlay += 2) {
        g_assert_cmphex(bt463_lookup_rgb(&state, overlay << 24, 0,
                                         BT463_LOAD_LOWER), ==,
                        test_bt463_palette_value(0x10 + overlay));
    }

    /* A nonzero pixel still wins over an underlay-selected overlay word. */
    state.palette[0x30][0] = 0x31;
    state.palette[0x20][1] = 0x22;
    state.palette[0x20][2] = 0x23;
    g_assert_cmphex(bt463_lookup_rgb(&state,
                                    test_bt463_warp9c_pins(1, 0, 0, 2),
                                    0, BT463_LOAD_LOWER), ==, 0x312223);

    /* Alternate true-color source and CR15 contiguous source are routed too. */
    state.command[1] = 0x01;
    state.wtt[0] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                  1, 0xf, start, false);
    g_assert_cmphex(bt463_lookup_rgb(&state, (1U << 16) | (1U << 24), 0,
                                     BT463_LOAD_LOWER), ==, 0xc1c2c3);
    state.command[1] = 0x21;
    state.wtt[0] = test_bt463_wtt(0, 4, BT463_WTT_TRUE_COLOR,
                                  0, 0xf, start, false);
    g_assert_cmphex(bt463_lookup_rgb(&state, 1U << 13, 0,
                                     BT463_LOAD_LOWER), ==,
                    test_bt463_palette_value(0x12));
}

static void test_bt463_lookup_dual_cursor_truth_table(void)
{
    Bt463State state;
    const unsigned start = 4;
    const uint32_t cursor0 = 0xd1d2d3;
    const uint32_t cursor1 = 0xe1e2e3;

    bt463_init(&state);
    memset(state.read_mask, 0xff, sizeof(state.read_mask));
    test_bt463_fill_palette(&state);
    state.wtt[0] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                  0, 0xf, start, false);
    state.cursor[0][0] = (cursor0 >> 16) & 0xff;
    state.cursor[0][1] = (cursor0 >> 8) & 0xff;
    state.cursor[0][2] = cursor0 & 0xff;
    state.cursor[1][0] = (cursor1 >> 16) & 0xff;
    state.cursor[1][1] = (cursor1 >> 8) & 0xff;
    state.cursor[1][2] = cursor1 & 0xff;
    state.command[1] = 0x02;

    /* Table 11: OL1:OL0=01 selects cursor 0; OL1=1 selects cursor 1. */
    for (unsigned overlay = 1; overlay < 16; overlay += 4) {
        g_assert_cmphex(bt463_lookup_rgb(&state, overlay << 24, 0,
                                         BT463_LOAD_LOWER), ==, cursor0);
    }
    for (unsigned overlay = 2; overlay < 16; overlay++) {
        if (!(overlay & 2)) {
            continue;
        }
        g_assert_cmphex(bt463_lookup_rgb(&state, overlay << 24, 0,
                                         BT463_LOAD_LOWER), ==, cursor1);
    }

    /* The remaining 00 values route through overlay/underlay selection. */
    for (unsigned overlay = 4; overlay <= 12; overlay += 4) {
        g_assert_cmphex(bt463_lookup_rgb(&state, overlay << 24, 0,
                                         BT463_LOAD_LOWER), ==,
                        test_bt463_palette_value(0x10 + overlay));
    }
    state.command[1] = 0x06;
    g_assert_cmphex(bt463_lookup_rgb(&state, 4U << 24, 0,
                                     BT463_LOAD_LOWER), ==,
                    test_bt463_palette_value(0x14));
    g_assert_cmphex(bt463_lookup_rgb(&state, 8U << 24, 0,
                                     BT463_LOAD_LOWER), ==,
                    test_bt463_palette_value(0x18));
    g_assert_cmphex(bt463_lookup_rgb(&state, 12U << 24, 0,
                                     BT463_LOAD_LOWER), ==,
                    test_bt463_palette_value(0x1c));

    /* Dual-cursor underlay also yields to nonzero pixel data. */
    state.palette[0x30][0] = 0x31;
    state.palette[0x20][1] = 0x22;
    state.palette[0x20][2] = 0x23;
    g_assert_cmphex(bt463_lookup_rgb(&state,
                                    test_bt463_warp9c_pins(1, 0, 0, 4),
                                    0, BT463_LOAD_LOWER), ==, 0x312223);
}

static void test_bt463_lookup_eight_overlay_planes(void)
{
    Bt463State state;
    const uint32_t overlay_color = 0xa1a2a3;
    const uint32_t pixel_color = 0xb1b2b3;

    bt463_init(&state);
    memset(state.read_mask, 0xff, sizeof(state.read_mask));
    state.command[1] = 0x50; /* CR16 plus CR14: fixed eight-plane overlay. */
    test_bt463_fill_palette(&state);
    state.wtt[0] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                  0, 0xf, 0x20, false);
    state.wtt[0xa] = state.wtt[0];
    state.palette[0xa5][0] = (overlay_color >> 16) & 0xff;
    state.palette[0xa5][1] = (overlay_color >> 8) & 0xff;
    state.palette[0xa5][2] = overlay_color & 0xff;
    state.palette[0x110][0] = (pixel_color >> 16) & 0xff;
    state.palette[0x110][1] = (pixel_color >> 8) & 0xff;
    state.palette[0x110][2] = pixel_color & 0xff;
    state.palette[0x100][0] = 0;
    state.palette[0x100][1] = (pixel_color >> 8) & 0xff;
    state.palette[0x100][2] = pixel_color & 0xff;

    /* WT0-WT3 supply OL4-OL7; P24-P27 supply OL0-OL3. */
    g_assert_cmphex(bt463_lookup_rgb(&state,
                                    test_bt463_warp9c_pins(0, 0, 0, 5),
                                    0xa, BT463_LOAD_LOWER), ==,
                    overlay_color);

    /* CR14 ignores CR16 and puts the pixel palette at start=$100. */
    g_assert_cmphex(bt463_lookup_rgb(&state,
                                    test_bt463_warp9c_pins(1, 0, 0, 0),
                                    0, BT463_LOAD_LOWER), ==,
                    pixel_color);

    /* Pseudo-color is also valid, while window operations are unavailable. */
    state.wtt[0] = test_bt463_wtt(0, 8, BT463_WTT_PSEUDO_COLOR,
                                  0, 0xf, 0x20, false);
    state.palette[0x1ab][0] = 0x41;
    state.palette[0x1ab][1] = 0x52;
    state.palette[0x1ab][2] = 0x63;
    g_assert_cmphex(bt463_lookup_rgb(&state, 0x1ab, 0,
                                    BT463_LOAD_LOWER), ==, 0x415263);

    /* CR12 makes an upper-only CR14 overlay word an underlay candidate. */
    state.wtt[1] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                  0, 0xf, 0x20, false);
    state.palette[0x010][0] = 0x91;
    state.palette[0x010][1] = 0x92;
    state.palette[0x010][2] = 0x93;
    state.palette[0x100][0] = 0x11;
    state.palette[0x100][1] = 0x22;
    state.palette[0x100][2] = 0x33;
    state.palette[0x110][0] = 0xa1;
    for (unsigned config = 0; config < 3; config++) {
        state.command[1] = 0x14 | config; /* CR14 + CR12 + cursor mode. */
        g_assert_cmphex(bt463_lookup_rgb(&state,
                                        test_bt463_warp9c_pins(0, 0, 0, 0),
                                        1, BT463_LOAD_LOWER), ==, 0x919293);
        g_assert_cmphex(bt463_lookup_rgb(&state,
                                        test_bt463_warp9c_pins(1, 0, 0, 0),
                                        1, BT463_LOAD_LOWER), ==, 0xa12233);
    }
}

static void test_bt463_lookup_invalid(void)
{
    Bt463State state;
    uint32_t pixel = test_bt463_warp9c_pins(0xf, 0xf, 0xf, 0xf);
    const uint32_t overlay_pixel = test_bt463_warp9c_pins(0, 0, 0, 1);

    bt463_init(&state);
    memset(state.read_mask, 0xff, sizeof(state.read_mask));
    state.wtt[0] = test_bt463_wtt(0, 8, 3, 0, 0, 0, false);
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0);

    state.wtt[0] = test_bt463_wtt(5, 8, BT463_WTT_TRUE_COLOR,
                                  0, 0, 0, false);
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0);

    state.wtt[0] = test_bt463_wtt(0, 8, BT463_WTT_PSEUDO_COLOR,
                                  0, 0, 0x3f, false);
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0);

    /* An overlay cannot make a reserved WTT encoding valid. */
    state.command[1] = 0x40;
    state.wtt[0] = test_bt463_wtt(0, 8, BT463_WTT_RESERVED_3,
                                  0, 0xf, 0, false);
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0);
    state.wtt[0] = test_bt463_wtt(0, 8,
                                  BT463_PSEUDO_COLOR_LOAD_INTERLEAVE,
                                  1, 0xf, 0, false);
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER),
                    ==, 0);

    /* Malformed bypass modes cannot be rescued by overlay routing. */
    state.palette[0x201][0] = 0xa1;
    state.palette[0x201][1] = 0xa2;
    state.palette[0x201][2] = 0xa3;
    state.cursor[0][0] = 0xb1;
    state.cursor[0][1] = 0xb2;
    state.cursor[0][2] = 0xb3;
    state.wtt[0] = test_bt463_wtt(0, 4, BT463_WTT_TRUE_COLOR,
                                  0, 0xf, 0, true);
    state.command[1] = 0x40; /* Common overlay palette. */
    g_assert_cmphex(bt463_lookup_rgb(&state, overlay_pixel, 0,
                                    BT463_LOAD_LOWER), ==, 0);
    state.command[1] = 0x01; /* One cursor plane. */
    g_assert_cmphex(bt463_lookup_rgb(&state, overlay_pixel, 0,
                                    BT463_LOAD_LOWER), ==, 0);

    state.wtt[0] = test_bt463_wtt(0, 4, BT463_WTT_PSEUDO_COLOR,
                                  0, 0xf, 0, true);
    state.command[1] = 0x40;
    g_assert_cmphex(bt463_lookup_rgb(&state, overlay_pixel, 0,
                                    BT463_LOAD_LOWER), ==, 0);
    state.command[1] = 0x01;
    g_assert_cmphex(bt463_lookup_rgb(&state, overlay_pixel, 0,
                                    BT463_LOAD_LOWER), ==, 0);
}

static void test_bt463_lookup_invalid_start(void)
{
    Bt463State state;
    const uint32_t overlay_pixel = test_bt463_warp9c_pins(0, 0, 0, 1);

    bt463_init(&state);
    memset(state.read_mask, 0xff, sizeof(state.read_mask));
    state.palette[0x201][0] = 0xa1;
    state.palette[0x201][1] = 0xa2;
    state.palette[0x201][2] = 0xa3;
    state.cursor[0][0] = 0xb1;
    state.cursor[0][1] = 0xb2;
    state.cursor[0][2] = 0xb3;

    /* The largest six-bit start field maps to the legal 0x1f8 row. */
    state.palette[0x1f8][0] = 0x11;
    state.palette[0x1f8][1] = 0x22;
    state.palette[0x1f8][2] = 0x33;
    state.wtt[0] = test_bt463_wtt(0, 0, BT463_WTT_TRUE_COLOR,
                                  0, 0, 0x3f, false);
    g_assert_cmphex(bt463_lookup_rgb(&state, 0, 0, BT463_LOAD_LOWER), ==,
                    0x112233);

    /* CR14 accepts only the physical start row at 0x100. */
    state.command[1] = 0x10;
    state.wtt[0] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                  0, 0xf, 0x10, false);
    g_assert_cmphex(bt463_lookup_rgb(&state, overlay_pixel, 0,
                                    BT463_LOAD_LOWER), ==, 0);

    /* Start validation also precedes bypass, even if the pixel is direct. */
    state.wtt[0] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                  0, 0, 0x10, true);
    g_assert_cmphex(bt463_lookup_rgb(&state, overlay_pixel, 0,
                                    BT463_LOAD_LOWER), ==, 0);
}

static void test_bt463_blink_step(void)
{
    static const struct {
        uint8_t rate;
        unsigned on;
        unsigned off;
    } rates[] = {
        { 0, 16, 48 },
        { 1, 16, 16 },
        { 2, 32, 32 },
        { 3, 64, 64 },
    };
    Bt463State state;

    for (unsigned rate = 0; rate < G_N_ELEMENTS(rates); rate++) {
        bt463_init(&state);
        memset(state.read_mask, 0xff, sizeof(state.read_mask));
        state.blink_mask[0] = 0xf0;
        state.command[0] = rates[rate].rate << 2;

        g_assert_true(state.blink_phase);
        g_assert_cmpuint(state.blink_counter, ==, 0);
        for (unsigned i = 1; i < rates[rate].on; i++) {
            g_assert_false(bt463_retrace_step(&state));
            g_assert_true(state.blink_phase);
            g_assert_cmpuint(state.blink_counter, ==, i);
        }
        g_assert_true(bt463_retrace_step(&state));
        g_assert_false(state.blink_phase);
        g_assert_cmpuint(state.blink_counter, ==, 0);
        for (unsigned i = 1; i < rates[rate].off; i++) {
            g_assert_false(bt463_retrace_step(&state));
            g_assert_false(state.blink_phase);
            g_assert_cmpuint(state.blink_counter, ==, i);
        }
        g_assert_true(bt463_retrace_step(&state));
        g_assert_true(state.blink_phase);
        g_assert_cmpuint(state.blink_counter, ==, 0);
    }

    /* Each mask byte can make a phase change visible independently. */
    bt463_init(&state);
    memset(state.read_mask, 0xff, sizeof(state.read_mask));
    for (unsigned i = 0; i < G_N_ELEMENTS(state.blink_mask); i++) {
        memset(state.blink_mask, 0, sizeof(state.blink_mask));
        state.blink_mask[i] = i == 3 ? 0x0f : 0xf0;
        state.command[0] = 0x04;
        for (unsigned retrace = 0; retrace < 15; retrace++) {
            g_assert_false(bt463_retrace_step(&state));
        }
        g_assert_true(bt463_retrace_step(&state));
        state.blink_phase = true;
        state.blink_counter = 0;
    }

    /* The high nibble of the fourth byte is outside the physical overlay. */
    memset(state.blink_mask, 0, sizeof(state.blink_mask));
    state.blink_mask[3] = 0xf0;
    state.command[0] = 0x04;
    for (unsigned retrace = 0; retrace < 15; retrace++) {
        g_assert_false(bt463_retrace_step(&state));
    }
    g_assert_false(bt463_retrace_step(&state));
    g_assert_false(state.blink_phase);

    /* A board may exclude an otherwise modeled mask byte from scanout. */
    bt463_reset(&state);
    memset(state.read_mask, 0xff, sizeof(state.read_mask));
    state.blink_mask[0] = 0x0f;
    state.blink_mask[1] = 0x0f;
    state.blink_mask[2] = 0x0f;
    state.blink_mask[3] = 0xff;
    state.command[0] = 0x04;
    for (unsigned retrace = 0; retrace < 15; retrace++) {
        g_assert_false(bt463_retrace_step_visible(&state, 0x00f0f0f0U));
    }
    g_assert_false(bt463_retrace_step_visible(&state, 0x00f0f0f0U));
    g_assert_false(state.blink_phase);
    state.blink_phase = true;
    state.blink_counter = 15;
    state.blink_mask[0] = 0xf0;
    g_assert_true(bt463_retrace_step_visible(&state, 0x00f0f0f0U));

    /* Read masks independently suppress the same blink byte's visibility. */
    bt463_reset(&state);
    memset(state.blink_mask, 0, sizeof(state.blink_mask));
    memset(state.read_mask, 0, sizeof(state.read_mask));
    state.blink_mask[2] = 0xff;
    state.read_mask[2] = 0;
    state.command[0] = 0x04;
    for (unsigned retrace = 0; retrace < 15; retrace++) {
        g_assert_false(bt463_retrace_step(&state));
    }
    g_assert_false(bt463_retrace_step(&state));
    g_assert_false(state.blink_phase);

    /* A CR0 write resets both cadence state and phase. */
    bt463_reset(&state);
    state.command[0] = 0x04;
    memset(state.read_mask, 0xff, sizeof(state.read_mask));
    state.blink_mask[0] = 0xf0;
    for (unsigned i = 0; i < 16; i++) {
        bt463_retrace_step(&state);
    }
    g_assert_false(state.blink_phase);
    bt463_address_write(&state, false, 0x01);
    bt463_address_write(&state, true, 0x02);
    g_assert_true(bt463_general_write(&state, 0x04));
    g_assert_true(state.blink_phase);
    g_assert_cmpuint(state.blink_counter, ==, 0);
}

static void test_bt463_mask_bytes(void)
{
    Bt463State state;
    const uint32_t pixel = test_bt463_warp9c_pins(0xf, 0xf, 0xf, 0);

    bt463_init(&state);
    memset(state.read_mask, 0xff, sizeof(state.read_mask));
    state.wtt[0] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                  0, 0, 0, false);
    state.palette[0xf0][0] = 0x10;
    state.palette[0xf0][1] = 0x20;
    state.palette[0xf0][2] = 0x30;
    g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0, BT463_LOAD_LOWER), ==,
                    0x102030);

    /* Every read-mask byte is applied before WTT channel extraction. */
    for (unsigned i = 0; i < 3; i++) {
        memset(state.read_mask, 0xff, sizeof(state.read_mask));
        state.read_mask[i] = 0;
        g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0,
                                         BT463_LOAD_LOWER), ==,
                        i == 0 ? 0x002030 :
                        i == 1 ? 0x100030 : 0x102000);
    }

    /* Blink masks use the same four physical input bytes. */
    memset(state.read_mask, 0xff, sizeof(state.read_mask));
    state.blink_phase = false;
    for (unsigned i = 0; i < 3; i++) {
        memset(state.blink_mask, 0, sizeof(state.blink_mask));
        state.blink_mask[i] = 0xf0;
        g_assert_cmphex(bt463_lookup_rgb(&state, pixel, 0,
                                         BT463_LOAD_LOWER), ==,
                        i == 0 ? 0x002030 :
                        i == 1 ? 0x100030 : 0x102000);
    }

    /* The fourth byte carries the physical overlay port's low nibble. */
    memset(state.blink_mask, 0, sizeof(state.blink_mask));
    state.blink_phase = true;
    state.command[1] = 0x40; /* common overlay palette */
    state.wtt[0] = test_bt463_wtt(0, 8, BT463_WTT_TRUE_COLOR,
                                  0, 0xf, 0, false);
    state.palette[0x201][0] = 0xa1;
    state.palette[0x201][1] = 0xa2;
    state.palette[0x201][2] = 0xa3;
    g_assert_cmphex(bt463_lookup_rgb(&state,
                                     test_bt463_warp9c_pins(0, 0, 0, 1),
                                     0, BT463_LOAD_LOWER), ==, 0xa1a2a3);

    state.blink_phase = false;
    state.blink_mask[3] = 0x01;
    g_assert_cmphex(bt463_lookup_rgb(&state,
                                     test_bt463_warp9c_pins(0, 0, 0, 1),
                                     0, BT463_LOAD_LOWER), ==, 0);
    state.blink_phase = true;
    state.blink_mask[3] = 0;
    state.read_mask[3] = 0;
    g_assert_cmphex(bt463_lookup_rgb(&state,
                                     test_bt463_warp9c_pins(0, 0, 0, 1),
                                     0, BT463_LOAD_LOWER), ==, 0);
}

int main(int argc, char **argv)
{
    module_call_init(MODULE_INIT_QOM);
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/bt463/import-legacy", test_bt463_import_legacy);
    g_test_add_func("/bt463/import-legacy-wtt-latches",
                    test_bt463_import_legacy_wtt_latches);
    g_test_add_func("/bt463/legacy-vmstate", test_bt463_legacy_vmstate);
    g_test_add_func("/bt463/vmstate", test_bt463_vmstate);
    g_test_add_func("/bt463/reset-every-state",
                    test_bt463_reset_every_state);
    g_test_add_func("/bt463/lookup-true-color",
                    test_bt463_lookup_true_color);
    g_test_add_func("/bt463/lookup-contiguous-planes",
                    test_bt463_lookup_contiguous_planes);
    g_test_add_func("/bt463/lookup-modes", test_bt463_lookup_modes);
    g_test_add_func("/bt463/lookup-load-interleave",
                    test_bt463_lookup_load_interleave);
    g_test_add_func("/bt463/lookup-overlay-and-cursor",
                    test_bt463_lookup_overlay_and_cursor);
    g_test_add_func("/bt463/lookup-standard-overlay-truth-table",
                    test_bt463_lookup_standard_overlay_truth_table);
    g_test_add_func("/bt463/lookup-single-cursor-truth-table",
                    test_bt463_lookup_single_cursor_truth_table);
    g_test_add_func("/bt463/lookup-dual-cursor-truth-table",
                    test_bt463_lookup_dual_cursor_truth_table);
    g_test_add_func("/bt463/lookup-eight-overlay-planes",
                    test_bt463_lookup_eight_overlay_planes);
    g_test_add_func("/bt463/lookup-invalid", test_bt463_lookup_invalid);
    g_test_add_func("/bt463/lookup-invalid-start",
                    test_bt463_lookup_invalid_start);
    g_test_add_func("/bt463/blink-step", test_bt463_blink_step);
    g_test_add_func("/bt463/mask-bytes", test_bt463_mask_bytes);
    return g_test_run();
}
