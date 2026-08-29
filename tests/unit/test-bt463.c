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

    g_assert_cmpuint(state.address, ==, 0x0abc);
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

    /*
     * A four-plane true-color entry packs the three channel fields
     * contiguously after the shift (P0-P3, P4-P7, P8-P11).
     */
    state.read_mask[0] = 0xff;
    state.read_mask[1] = 0xff;
    state.read_mask[2] = 0xff;
    state.wtt[0] = test_bt463_wtt(0, 4, BT463_WTT_TRUE_COLOR,
                                  0, 0, 0, false);
    compact_pixel = 0x00000cba;
    state.palette[0x0a][0] = 0x31;
    state.palette[0x0b][1] = 0x42;
    state.palette[0x0c][2] = 0x53;
    g_assert_cmphex(bt463_lookup_rgb(&state, compact_pixel, 0,
                                    BT463_LOAD_LOWER),
                    ==, 0x314253);
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
                                  0, 0, 1, false);
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
}

static void test_bt463_lookup_invalid(void)
{
    Bt463State state;
    uint32_t pixel = test_bt463_warp9c_pins(0xf, 0xf, 0xf, 0xf);

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
}

int main(int argc, char **argv)
{
    module_call_init(MODULE_INIT_QOM);
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/bt463/import-legacy", test_bt463_import_legacy);
    g_test_add_func("/bt463/import-legacy-wtt-latches",
                    test_bt463_import_legacy_wtt_latches);
    g_test_add_func("/bt463/legacy-vmstate", test_bt463_legacy_vmstate);
    g_test_add_func("/bt463/lookup-true-color",
                    test_bt463_lookup_true_color);
    g_test_add_func("/bt463/lookup-modes", test_bt463_lookup_modes);
    g_test_add_func("/bt463/lookup-load-interleave",
                    test_bt463_lookup_load_interleave);
    g_test_add_func("/bt463/lookup-invalid", test_bt463_lookup_invalid);
    return g_test_run();
}
