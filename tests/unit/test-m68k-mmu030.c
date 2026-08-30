/*
 * MC68030 PMMU architectural state tests.
 *
 * Copyright (c) 2026 Bryce Lanham
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "target/m68k/mmu030.h"
#include "io/channel-buffer.h"
#include "migration/qemu-file.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

static void test_mmu030_reset(void)
{
    M68KMMU030State state;

    memset(&state, 0xff, sizeof(state));
    m68k_mmu030_reset(&state);

    g_assert_cmpuint(state.crp, ==, 0);
    g_assert_cmpuint(state.srp, ==, 0);
    g_assert_cmpuint(state.tc, ==, 0);
    g_assert_cmpuint(state.tt[0], ==, 0);
    g_assert_cmpuint(state.tt[1], ==, 0);
    g_assert_cmpuint(state.mmusr, ==, 0);
    g_assert_cmpuint(state.atc_next, ==, 0);
    g_assert_false(state.fault_pending);
    g_assert_cmpuint(state.fault_address, ==, 0);
    g_assert_cmpuint(state.fault_pc, ==, 0);
    g_assert_cmpuint(state.fault_ssw, ==, 0);
    g_assert_cmpuint(state.fault_status, ==, 0);

    for (unsigned i = 0; i < M68K_MMU030_ATC_ENTRIES; i++) {
        g_assert_cmpuint(state.atc[i].logical, ==, 0);
        g_assert_cmpuint(state.atc[i].physical, ==, 0);
        g_assert_cmpuint(state.atc[i].status, ==, 0);
    }
}

static void test_mmu030_vmstate(void)
{
    M68KMMU030State source;
    M68KMMU030State destination;
    QIOChannelBuffer *save_channel = qio_channel_buffer_new(0);
    QIOChannelBuffer *load_channel;
    QEMUFile *file;
    Error *local_err = NULL;
    g_autofree uint8_t *wire = NULL;
    size_t wire_size;

    memset(&source, 0, sizeof(source));
    source.crp = UINT64_C(0x123456789abcdef0);
    source.srp = UINT64_C(0x0fedcba987654321);
    source.tc = UINT32_C(0x87654321);
    source.tt[0] = UINT32_C(0x10203040);
    source.tt[1] = UINT32_C(0x50607080);
    source.mmusr = UINT16_C(0xa55a);
    source.atc_next = 17;
    source.fault_pending = true;
    source.fault_address = UINT32_C(0xfeedcafe);
    source.fault_pc = UINT32_C(0x00123456);
    source.fault_ssw = UINT16_C(0x5aa5);
    source.fault_status = UINT32_C(0xdeadbeef);
    for (unsigned i = 0; i < M68K_MMU030_ATC_ENTRIES; i++) {
        source.atc[i].logical = UINT32_C(0x10000000) + i;
        source.atc[i].physical = UINT32_C(0x20000000) + i * 0x1000;
        source.atc[i].status = UINT32_C(0x30000000) + i;
    }

    file = qemu_file_new_output(QIO_CHANNEL(save_channel));
    g_assert_cmpint(vmstate_save_state(file, &vmstate_mmu030_state,
                                       &source, NULL, &local_err), ==, 0);
    g_assert_null(local_err);
    g_assert_cmpint(qemu_fflush(file), ==, 0);
    wire_size = save_channel->usage;
    wire = g_memdup2(save_channel->data, wire_size);
    qemu_fclose(file);

    memset(&destination, 0, sizeof(destination));
    load_channel = qio_channel_buffer_new(wire_size);
    memcpy(load_channel->data, wire, wire_size);
    load_channel->usage = wire_size;
    load_channel->offset = 0;
    file = qemu_file_new_input(QIO_CHANNEL(load_channel));
    g_assert_cmpint(vmstate_load_state(file, &vmstate_mmu030_state,
                                       &destination, 1, &local_err), ==, 0);
    g_assert_null(local_err);
    qemu_fclose(file);

    g_assert_cmpuint(destination.crp, ==, source.crp);
    g_assert_cmpuint(destination.srp, ==, source.srp);
    g_assert_cmpuint(destination.tc, ==, source.tc);
    g_assert_cmpuint(destination.tt[0], ==, source.tt[0]);
    g_assert_cmpuint(destination.tt[1], ==, source.tt[1]);
    g_assert_cmpuint(destination.mmusr, ==, source.mmusr);
    g_assert_cmpuint(destination.atc_next, ==, source.atc_next);
    g_assert_true(destination.fault_pending);
    g_assert_cmpuint(destination.fault_address, ==, source.fault_address);
    g_assert_cmpuint(destination.fault_pc, ==, source.fault_pc);
    g_assert_cmpuint(destination.fault_ssw, ==, source.fault_ssw);
    g_assert_cmpuint(destination.fault_status, ==, source.fault_status);
    for (unsigned i = 0; i < M68K_MMU030_ATC_ENTRIES; i++) {
        g_assert_cmpuint(destination.atc[i].logical, ==,
                         source.atc[i].logical);
        g_assert_cmpuint(destination.atc[i].physical, ==,
                         source.atc[i].physical);
        g_assert_cmpuint(destination.atc[i].status, ==,
                         source.atc[i].status);
    }

    object_unref(OBJECT(save_channel));
    object_unref(OBJECT(load_channel));
}

int main(int argc, char **argv)
{
    module_call_init(MODULE_INIT_QOM);
    g_test_init(&argc, &argv, NULL);

    g_test_add_func("/m68k/mmu030/reset", test_mmu030_reset);
    g_test_add_func("/m68k/mmu030/vmstate", test_mmu030_vmstate);

    return g_test_run();
}
