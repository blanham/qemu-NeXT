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
#include "migration/savevm.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "../qtest/libqtest.h"

#define M68K_MMU030_TC_E        (UINT32_C(1) << 31)
#define M68K_MMU030_TC_PS_SHIFT 20
#define M68K_MMU030_TC_IS_SHIFT 16
#define M68K_MMU030_TC_TIA_SHIFT 12
#define M68K_MMU030_TC_TIB_SHIFT 8
#define M68K_MMU030_TC_TIC_SHIFT 4

#define M68K_MMU030_TT_E        (UINT32_C(1) << 15)
#define M68K_MMU030_TT_CI       (UINT32_C(1) << 10)
#define M68K_MMU030_TT_RW       (UINT32_C(1) << 9)
#define M68K_MMU030_TT_RWM      (UINT32_C(1) << 8)

#define M68K_MMU030_WIRE_SIZE \
    (sizeof(uint64_t) + sizeof(uint64_t) + sizeof(uint32_t) + \
     2 * sizeof(uint32_t) + sizeof(uint16_t) + \
     M68K_MMU030_ATC_ENTRIES * 3 * sizeof(uint32_t) + \
     sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint32_t) + \
     sizeof(uint32_t) + sizeof(uint16_t) + sizeof(uint32_t))

typedef struct MigrationSubsection {
    uint8_t *data;
    size_t size;
    size_t payload_offset;
    size_t payload_size;
} MigrationSubsection;

static void wire_put_u16(uint8_t **cursor, uint16_t value)
{
    *(*cursor)++ = value >> 8;
    *(*cursor)++ = value;
}

static void wire_put_u32(uint8_t **cursor, uint32_t value)
{
    *(*cursor)++ = value >> 24;
    *(*cursor)++ = value >> 16;
    *(*cursor)++ = value >> 8;
    *(*cursor)++ = value;
}

static void wire_put_u64(uint8_t **cursor, uint64_t value)
{
    wire_put_u32(cursor, value >> 32);
    wire_put_u32(cursor, value);
}

static void encode_mmu030_wire(uint8_t *wire, const M68KMMU030State *state)
{
    uint8_t *cursor = wire;

    wire_put_u64(&cursor, state->crp);
    wire_put_u64(&cursor, state->srp);
    wire_put_u32(&cursor, state->tc);
    wire_put_u32(&cursor, state->tt[0]);
    wire_put_u32(&cursor, state->tt[1]);
    wire_put_u16(&cursor, state->mmusr);
    for (unsigned i = 0; i < M68K_MMU030_ATC_ENTRIES; i++) {
        wire_put_u32(&cursor, state->atc[i].logical);
        wire_put_u32(&cursor, state->atc[i].physical);
        wire_put_u32(&cursor, state->atc[i].status);
    }
    *cursor++ = state->atc_next;
    *cursor++ = state->fault_pending;
    wire_put_u32(&cursor, state->fault_address);
    wire_put_u32(&cursor, state->fault_pc);
    wire_put_u16(&cursor, state->fault_ssw);
    wire_put_u32(&cursor, state->fault_status);
    g_assert_cmpuint(cursor - wire, ==, M68K_MMU030_WIRE_SIZE);
}

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

static uint32_t mmu030_tc(unsigned ps, unsigned is, unsigned tia,
                          unsigned tib, unsigned tic, unsigned tid)
{
    return M68K_MMU030_TC_E |
           (ps << M68K_MMU030_TC_PS_SHIFT) |
           (is << M68K_MMU030_TC_IS_SHIFT) |
           (tia << M68K_MMU030_TC_TIA_SHIFT) |
           (tib << M68K_MMU030_TC_TIB_SHIFT) |
           (tic << M68K_MMU030_TC_TIC_SHIFT) |
           tid;
}

static uint32_t mmu030_tt(uint8_t address_base, uint8_t address_mask,
                          bool cache_inhibit, bool write, bool rw_mask,
                          uint8_t function_code_base,
                          uint8_t function_code_mask)
{
    return ((uint32_t)address_base << 24) |
           ((uint32_t)address_mask << 16) |
           M68K_MMU030_TT_E |
           (cache_inhibit ? M68K_MMU030_TT_CI : 0) |
           (write ? 0 : M68K_MMU030_TT_RW) |
           (rw_mask ? M68K_MMU030_TT_RWM : 0) |
           ((function_code_base & 7) << 4) |
           (function_code_mask & 7);
}

static void test_mmu030_tc_validation(void)
{
    g_assert_true(m68k_mmu030_validate_tc(0));
    g_assert_true(m68k_mmu030_validate_tc(UINT32_C(0x7fffffff)));

    /* 4 KiB pages with two ten-bit table indexes. */
    g_assert_true(m68k_mmu030_validate_tc(mmu030_tc(12, 0, 10, 10,
                                                     0, 0)));
    /* NeXT's 8 KiB layout: 13 + 7 + 7 + 5 = 32. */
    g_assert_true(m68k_mmu030_validate_tc(mmu030_tc(13, 0, 7, 7, 5,
                                                     0)));
    /* A generic 8 KiB layout may use a different table decomposition. */
    g_assert_true(m68k_mmu030_validate_tc(mmu030_tc(13, 0, 9, 10,
                                                     0, 0)));

    /* Page size encoding 7 is below the MC68030 minimum of 8. */
    g_assert_false(m68k_mmu030_validate_tc(mmu030_tc(7, 0, 10, 10,
                                                      0, 0)));
    /* The enabled widths must account for all 32 address bits. */
    g_assert_false(m68k_mmu030_validate_tc(mmu030_tc(12, 0, 10, 9,
                                                      0, 0)));
    /* TI fields after the first zero are ignored by the consistency check. */
    g_assert_true(m68k_mmu030_validate_tc(mmu030_tc(13, 9, 10, 0, 12,
                                                     0)));
}

static void test_mmu030_tt_matching(void)
{
    M68KMMU030TTResult result;
    uint32_t tt;

    tt = mmu030_tt(0x12, 0x00, false, false, true, 5, 0);
    result = m68k_mmu030_tt_match(tt, UINT32_C(0x12abcdef), 5, false);
    g_assert_true(result.matched);
    g_assert_false(result.cache_inhibit);
    result = m68k_mmu030_tt_match(tt, UINT32_C(0x13abcdef), 5, false);
    g_assert_false(result.matched);

    tt = mmu030_tt(0x12, 0xff, true, false, true, 5, 7);
    result = m68k_mmu030_tt_match(tt, UINT32_C(0x12abcdef), 2, false);
    g_assert_true(result.matched);
    g_assert_true(result.cache_inhibit);
    result = m68k_mmu030_tt_match(tt, UINT32_C(0x12abcdef), 7, true);
    g_assert_true(result.matched);
    result = m68k_mmu030_tt_match(tt, UINT32_C(0x13abcdef), 2, false);
    g_assert_true(result.matched);

    tt = mmu030_tt(0x12, 0x0f, false, false, true, 5, 7);
    result = m68k_mmu030_tt_match(tt, UINT32_C(0x1abcdef0), 5, false);
    g_assert_true(result.matched);
    result = m68k_mmu030_tt_match(tt, UINT32_C(0x22abcdef), 5, false);
    g_assert_false(result.matched);

    tt = mmu030_tt(0x12, 0x00, false, false, false, 5, 0);
    result = m68k_mmu030_tt_match(tt, UINT32_C(0x12000000), 5, false);
    g_assert_true(result.matched);
    result = m68k_mmu030_tt_match(tt, UINT32_C(0x12000000), 5, true);
    g_assert_false(result.matched);

    tt = mmu030_tt(0x12, 0x00, false, true, false, 5, 0);
    result = m68k_mmu030_tt_match(tt, UINT32_C(0x12000000), 5, true);
    g_assert_true(result.matched);
    result = m68k_mmu030_tt_match(tt, UINT32_C(0x12000000), 5, false);
    g_assert_false(result.matched);

    tt = mmu030_tt(0x12, 0x00, false, false, true, 5, 0);
    result = m68k_mmu030_tt_match(tt, UINT32_C(0x12000000), 4, false);
    g_assert_false(result.matched);

    tt = mmu030_tt(0x12, 0x00, true, false, true, 5, 0) &
         ~M68K_MMU030_TT_E;
    result = m68k_mmu030_tt_match(tt, UINT32_C(0x12000000), 5, false);
    g_assert_false(result.matched);
}

static size_t migration_find_subsection(const uint8_t *wire, size_t wire_size,
                                        const char *name,
                                        size_t *payload_offset)
{
    size_t name_size = strlen(name);
    size_t count = 0;

    g_assert_cmpuint(name_size, <=, UINT8_MAX);
    for (size_t i = 0; i + 2 + name_size + sizeof(uint32_t) <= wire_size;
         i++) {
        if (wire[i] != QEMU_VM_SUBSECTION ||
            wire[i + 1] != name_size ||
            memcmp(wire + i + 2, name, name_size) != 0) {
            continue;
        }
        if (count++ == 0) {
            *payload_offset = i + 2 + name_size + sizeof(uint32_t);
        }
    }
    return count;
}

static MigrationSubsection migration_read_subsection(const char *path,
                                                     const char *name)
{
    MigrationSubsection subsection = { 0 };
    g_autoptr(GError) error = NULL;
    gchar *contents = NULL;
    gsize contents_size;

    g_assert_true(g_file_get_contents(path, &contents, &contents_size,
                                      &error));
    g_assert_no_error(error);
    subsection.data = (uint8_t *)contents;
    subsection.size = contents_size;
    g_assert_cmpuint(migration_find_subsection(subsection.data,
                                               subsection.size, name,
                                               &subsection.payload_offset),
                     ==, 1);
    subsection.payload_size = M68K_MMU030_WIRE_SIZE;
    g_assert_cmpuint(subsection.payload_offset + subsection.payload_size,
                     <=, subsection.size);
    return subsection;
}

static char *migration_path_new(char **tmpdir, const char *prefix)
{
    g_autoptr(GError) error = NULL;
    char *path;

    *tmpdir = g_dir_make_tmp(prefix, &error);
    g_assert_no_error(error);
    g_assert_nonnull(*tmpdir);
    path = g_build_filename(*tmpdir, "migration.state", NULL);
    return path;
}

static void migration_path_cleanup(char *tmpdir, char *path)
{
    g_assert_cmpint(g_unlink(path), ==, 0);
    g_assert_cmpint(g_rmdir(tmpdir), ==, 0);
}

static QTestState *m68k_qtest_start(const char *model, const char *extra)
{
    return qtest_initf("-machine virt -cpu %s -m 8M -nodefaults %s",
                       model, extra ?: "");
}

static void migrate_to_file(QTestState *source, const char *path)
{
    g_autofree char *uri = g_strdup_printf("file:%s", path);

    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    for (unsigned i = 0; i < 200; i++) {
        QDict *response = qtest_qmp(source,
                                    "{ 'execute': 'query-migrate' }");
        QDict *return_value = qdict_get_qdict(response, "return");
        const char *status = qdict_get_try_str(return_value, "status");
        bool done = status && !strcmp(status, "completed");
        bool failed = status && (!strcmp(status, "failed") ||
                                 !strcmp(status, "cancelled"));

        qobject_unref(response);
        g_assert_false(failed);
        if (done) {
            return;
        }
        g_usleep(10 * 1000);
    }
    g_error("timed out waiting for migration to %s", path);
}

static void migrate_from_file(QTestState *destination, const char *path)
{
    g_autofree char *uri = g_strdup_printf("file:%s", path);

    qtest_qmp_assert_success(
        destination,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        uri);
    qtest_qmp_eventwait(destination, "RESUME");
}

static M68KMMU030State migration_pattern(void)
{
    M68KMMU030State state = { 0 };

    state.crp = UINT64_C(0x123456789abcdef0);
    state.srp = UINT64_C(0x0fedcba987654321);
    state.tc = UINT32_C(0x87654321);
    state.tt[0] = UINT32_C(0x10203040);
    state.tt[1] = UINT32_C(0x50607080);
    state.mmusr = UINT16_C(0xa55a);
    state.atc_next = 17;
    state.fault_pending = true;
    state.fault_address = UINT32_C(0xfeedcafe);
    state.fault_pc = UINT32_C(0x00123456);
    state.fault_ssw = UINT16_C(0x5aa5);
    state.fault_status = UINT32_C(0xdeadbeef);
    for (unsigned i = 0; i < M68K_MMU030_ATC_ENTRIES; i++) {
        state.atc[i].logical = UINT32_C(0x10000000) + i;
        state.atc[i].physical = UINT32_C(0x20000000) + i * 0x1000;
        state.atc[i].status = UINT32_C(0x30000000) + i;
    }
    return state;
}

static void test_cpu_vmstate_gating(void)
{
    static const struct {
        const char *model;
        unsigned expected_mmu030_subsections;
        unsigned expected_mmu040_subsections;
    } cases[] = {
        { "m68030", 1, 0 },
        { "m68040", 0, 1 },
    };

    for (unsigned i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *tmpdir = NULL;
        g_autofree char *path = migration_path_new(&tmpdir,
                                                   "m68k-mmu030-gating-XXXXXX");
        QTestState *source = m68k_qtest_start(cases[i].model, NULL);
        g_autofree gchar *wire = NULL;
        gsize wire_size;
        size_t payload_offset;
        size_t subsection_count;
        size_t mmu040_count;

        migrate_to_file(source, path);
        g_assert_true(g_file_get_contents(path, &wire, &wire_size, NULL));
        subsection_count = migration_find_subsection(
            (const uint8_t *)wire, wire_size, "cpu/68030_mmu",
            &payload_offset);
        g_assert_cmpuint(cases[i].expected_mmu030_subsections, ==,
                         subsection_count);
        mmu040_count = migration_find_subsection(
            (const uint8_t *)wire, wire_size, "cpu/68040_mmu",
            &payload_offset);
        g_assert_cmpuint(cases[i].expected_mmu040_subsections, ==,
                         mmu040_count);
        qtest_quit(source);
        migration_path_cleanup(g_steal_pointer(&tmpdir),
                                g_steal_pointer(&path));
    }
}

static void test_cpu_migration_stream(void)
{
    g_autofree char *source_tmpdir = NULL;
    g_autofree char *destination_tmpdir = NULL;
    g_autofree char *source_path = migration_path_new(
        &source_tmpdir, "m68k-mmu030-source-XXXXXX");
    g_autofree char *destination_path = migration_path_new(
        &destination_tmpdir, "m68k-mmu030-destination-XXXXXX");
    QTestState *source = m68k_qtest_start("m68030", NULL);
    QTestState *destination = m68k_qtest_start("m68030", "-incoming defer");
    M68KMMU030State expected = migration_pattern();
    uint8_t expected_wire[M68K_MMU030_WIRE_SIZE];
    MigrationSubsection subsection;
    MigrationSubsection roundtrip;

    encode_mmu030_wire(expected_wire, &expected);
    migrate_to_file(source, source_path);
    subsection = migration_read_subsection(source_path, "cpu/68030_mmu");
    memcpy(subsection.data + subsection.payload_offset, expected_wire,
           sizeof(expected_wire));
    g_assert_true(g_file_set_contents(source_path,
                                      (const gchar *)subsection.data,
                                      subsection.size, NULL));
    g_free(subsection.data);

    /* The incoming CPU subsection invokes its post-load TLB flush hook. */
    migrate_from_file(destination, source_path);
    /* A successful re-save proves the loaded CPU state remains usable. */
    migrate_to_file(destination, destination_path);
    roundtrip = migration_read_subsection(destination_path,
                                          "cpu/68030_mmu");
    g_assert_cmpmem(roundtrip.data + roundtrip.payload_offset,
                    roundtrip.payload_size, expected_wire,
                    sizeof(expected_wire));
    g_free(roundtrip.data);

    qtest_quit(source);
    qtest_quit(destination);
    migration_path_cleanup(g_steal_pointer(&source_tmpdir),
                            g_steal_pointer(&source_path));
    migration_path_cleanup(g_steal_pointer(&destination_tmpdir),
                            g_steal_pointer(&destination_path));
}

int main(int argc, char **argv)
{
    module_call_init(MODULE_INIT_QOM);
    g_test_init(&argc, &argv, NULL);

    g_test_add_func("/m68k/mmu030/reset", test_mmu030_reset);
    g_test_add_func("/m68k/mmu030/vmstate", test_mmu030_vmstate);
    g_test_add_func("/m68k/mmu030/tc-validation",
                    test_mmu030_tc_validation);
    g_test_add_func("/m68k/mmu030/tt-matching",
                    test_mmu030_tt_matching);
    if (g_getenv("QTEST_QEMU_BINARY")) {
        g_test_add_func("/m68k/mmu030/cpu-vmstate-gating",
                        test_cpu_vmstate_gating);
        g_test_add_func("/m68k/mmu030/cpu-migration-stream",
                        test_cpu_migration_stream);
    }

    return g_test_run();
}
