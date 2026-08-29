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
    g_assert_cmpuint(state.wtt_write_latch, ==, 0);
    g_assert_cmpuint(state.wtt_read_latch, ==, 0);
    g_assert_cmpuint(vmstate_bt463_legacy.version_id, ==, 1);
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

int main(int argc, char **argv)
{
    module_call_init(MODULE_INIT_QOM);
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/bt463/import-legacy", test_bt463_import_legacy);
    g_test_add_func("/bt463/legacy-vmstate", test_bt463_legacy_vmstate);
    return g_test_run();
}
