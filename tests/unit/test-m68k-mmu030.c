/*
 * MC68030 PMMU architectural state tests.
 *
 * Copyright (c) 2026 Bryce Lanham
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "exec/page-protection.h"
#include "target/m68k/mmu030.h"
#include "io/channel-buffer.h"
#include "migration/qemu-file.h"
#include "migration/savevm.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "../qtest/libqtest.h"

typedef struct M68KMMU030TestMemory {
    uint8_t data[0x10000];
    bool fail_read;
    bool fail_write;
    unsigned read_count;
    unsigned write_count;
} M68KMMU030TestMemory;

static bool mmu030_test_readl(void *opaque, uint32_t address, uint32_t *value)
{
    M68KMMU030TestMemory *memory = opaque;

    memory->read_count++;
    if (memory->fail_read || address > sizeof(memory->data) - 4) {
        return false;
    }

    *value = ((uint32_t)memory->data[address] << 24) |
             ((uint32_t)memory->data[address + 1] << 16) |
             ((uint32_t)memory->data[address + 2] << 8) |
             memory->data[address + 3];
    return true;
}

static bool mmu030_test_writel(void *opaque, uint32_t address, uint32_t value)
{
    M68KMMU030TestMemory *memory = opaque;

    memory->write_count++;
    if (memory->fail_write || address > sizeof(memory->data) - 4) {
        return false;
    }

    memory->data[address] = value >> 24;
    memory->data[address + 1] = value >> 16;
    memory->data[address + 2] = value >> 8;
    memory->data[address + 3] = value;
    return true;
}

static void mmu030_test_putl(M68KMMU030TestMemory *memory, uint32_t address,
                              uint32_t value)
{
    g_assert_true(mmu030_test_writel(memory, address, value));
    memory->write_count--;
}

static M68KMMU030MemoryOps mmu030_test_ops(M68KMMU030TestMemory *memory)
{
    return (M68KMMU030MemoryOps) {
        .readl = mmu030_test_readl,
        .writel = mmu030_test_writel,
        .opaque = memory,
    };
}

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
        source.atc[i].status = M68K_MMU030_ATC_VALID |
                               ((i & 7) << M68K_MMU030_ATC_FC_SHIFT) |
                               (12 << M68K_MMU030_ATC_PAGE_BITS_SHIFT) |
                               (i & 0x3f);
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

enum {
    MMU030_TEST_ACCESS_SUPER = 0x01,
    MMU030_TEST_ACCESS_STORE = 0x02,
    MMU030_TEST_ACCESS_PTEST = 0x08,
    MMU030_TEST_ACCESS_CODE = 0x10,
    MMU030_TEST_ACCESS_DATA = 0x20,
};

static uint32_t mmu030_test_getl(M68KMMU030TestMemory *memory,
                                  uint32_t address)
{
    uint32_t value = 0;

    g_assert_true(mmu030_test_readl(memory, address, &value));
    memory->read_count--;
    return value;
}

static void test_mmu030_descriptor_walker(void)
{
    M68KMMU030State state = { 0 };
    M68KMMU030TestMemory memory = { 0 };
    M68KMMU030MemoryOps ops = mmu030_test_ops(&memory);
    M68KMMU030TranslateResult result;
    const uint32_t logical = UINT32_C(0x12345678);
    const unsigned root_index = (logical >> 22) & 0x3ff;
    const unsigned page_index = (logical >> 12) & 0x3ff;
    uint32_t root_entry = UINT32_C(0x1000) + root_index * 4;
    uint32_t page_entry = UINT32_C(0x2000) + page_index * 4;

    /* A 4 KiB tree using short descriptors at both levels. */
    state.tc = mmu030_tc(12, 0, 10, 10, 0, 0);
    state.crp = (UINT64_C(0x7fff0002) << 32) | UINT32_C(0x1000);
    mmu030_test_putl(&memory, root_entry, UINT32_C(0x2002));
    mmu030_test_putl(&memory, page_entry, UINT32_C(0x00abc001));

    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA, 1, false,
                                     &result), ==, 0);
    g_assert_cmpuint(result.physical, ==, UINT32_C(0x00abc678));
    g_assert_cmpuint(result.page_size, ==, UINT32_C(4096));
    g_assert_cmpint(result.prot, ==, PAGE_READ | PAGE_WRITE);
    g_assert_false(result.cache_inhibit);
    g_assert_cmpuint(mmu030_test_getl(&memory, root_entry), ==,
                     UINT32_C(0x200a));
    g_assert_cmpuint(mmu030_test_getl(&memory, page_entry), ==,
                     UINT32_C(0x00abc009));
}

static void test_mmu030_descriptor_roots_and_fcl(void)
{
    M68KMMU030State state = { 0 };
    M68KMMU030TestMemory memory = { 0 };
    M68KMMU030MemoryOps ops = mmu030_test_ops(&memory);
    M68KMMU030TranslateResult result;
    const uint32_t logical = UINT32_C(0x12345678);
    const unsigned tia_index = (logical >> 22) & 0x3ff;
    const unsigned tib_index = (logical >> 12) & 0x3ff;

    state.tc = mmu030_tc(12, 0, 10, 10, 0, 0) | M68K_MMU030_TC_SRE;
    state.crp = (UINT64_C(0x7fff0002) << 32) | UINT32_C(0x1000);
    state.srp = (UINT64_C(0x7fff0002) << 32) | UINT32_C(0x3000);

    mmu030_test_putl(&memory, 0x1000 + tia_index * 4, 0x2002);
    mmu030_test_putl(&memory, 0x2000 + tib_index * 4, 0x00a00001);
    mmu030_test_putl(&memory, 0x3000 + tia_index * 4, 0x4002);
    mmu030_test_putl(&memory, 0x4000 + tib_index * 4, 0x00b00001);

    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA, 1, false,
                                     &result), ==, 0);
    g_assert_cmpuint(result.physical, ==, UINT32_C(0x00a00678));
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA | \
                                     MMU030_TEST_ACCESS_SUPER, 5, false,
                                     &result), ==, 0);
    g_assert_cmpuint(result.physical, ==, UINT32_C(0x00b00678));

    /* With FCL set, FC selects the root entry and the root limit is ignored. */
    memset(&memory, 0, sizeof(memory));
    state.tc = mmu030_tc(12, 0, 10, 10, 0, 0) | M68K_MMU030_TC_FCL;
    state.crp = (UINT64_C(0x00010002) << 32) | UINT32_C(0x1000);
    mmu030_test_putl(&memory, 0x1000 + 5 * 4, 0x5002);
    mmu030_test_putl(&memory, 0x5000 + tia_index * 4, 0x6002);
    mmu030_test_putl(&memory, 0x6000 + tib_index * 4, 0x00c00001);
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA, 5, false,
                                     &result), ==, 0);
    g_assert_cmpuint(result.physical, ==, UINT32_C(0x00c00678));
}

static void test_mmu030_descriptor_long_direct_and_early(void)
{
    M68KMMU030State state = { 0 };
    M68KMMU030TestMemory memory = { 0 };
    M68KMMU030MemoryOps ops = mmu030_test_ops(&memory);
    M68KMMU030TranslateResult result;
    const uint32_t logical = UINT32_C(0x12345678);
    const unsigned tia_index = (logical >> 22) & 0x3ff;
    const unsigned tib_index = (logical >> 12) & 0x3ff;
    uint32_t root_entry = 0x1000 + tia_index * 8;
    uint32_t page_entry = 0x2000 + tib_index * 8;

    state.tc = mmu030_tc(12, 0, 10, 10, 0, 0);
    state.crp = (UINT64_C(0x7fff0003) << 32) | UINT32_C(0x1000);
    mmu030_test_putl(&memory, root_entry, UINT32_C(0x7fff0003));
    mmu030_test_putl(&memory, root_entry + 4, UINT32_C(0x2010));
    page_entry = UINT32_C(0x2010) + tib_index * 8;
    mmu030_test_putl(&memory, page_entry, M68K_MMU030_DESC_PAGE |
                     M68K_MMU030_DESC_CI | M68K_MMU030_DESC_M);
    mmu030_test_putl(&memory, page_entry + 4, UINT32_C(0x00def000));
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA | \
                                     MMU030_TEST_ACCESS_SUPER, 5, false,
                                     &result), ==, 0);
    g_assert_cmpuint(result.physical, ==, UINT32_C(0x00def678));
    g_assert_cmpuint(result.page_size, ==, UINT32_C(4096));
    g_assert_cmpint(result.prot, ==, PAGE_READ | PAGE_WRITE);
    g_assert_true(result.cache_inhibit);

    /* A DT=1 root pointer maps the entire logical address with an offset. */
    memset(&memory, 0, sizeof(memory));
    state.crp = (UINT64_C(0x7fff0001) << 32) | UINT32_C(0x00100000);
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA, 1, false,
                                     &result), ==, 0);
    g_assert_cmpuint(result.physical, ==, UINT32_C(0x12445678));
    g_assert_cmpuint(memory.read_count, ==, 0);

    /* FCL selects the root by function code, so its LIMIT is ignored. */
    state.tc = mmu030_tc(12, 0, 10, 10, 0, 0) |
               M68K_MMU030_TC_FCL;
    state.crp = (UINT64_C(0x00000001) << 32) | UINT32_C(0x00100000);
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA, 1, false,
                                     &result), ==, 0);
    g_assert_cmpuint(result.physical, ==, UINT32_C(0x12445678));
    g_assert_false(result.limit_violation);

    /* DT=1 in a pointer table maps all remaining index bits contiguously. */
    memset(&memory, 0, sizeof(memory));
    state.tc = mmu030_tc(12, 0, 10, 10, 0, 0);
    state.crp = (UINT64_C(0x7fff0002) << 32) | UINT32_C(0x1000);
    mmu030_test_putl(&memory, 0x1000 + tia_index * 4, 0x00600001);
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA, 1, false,
                                     &result), ==, 0);
    g_assert_cmpuint(result.physical, ==,
                     UINT32_C(0x00600000) + (logical & UINT32_C(0x003fffff)));
}

static void test_mmu030_descriptor_indirect_and_8k(void)
{
    M68KMMU030State state = { 0 };
    M68KMMU030TestMemory memory = { 0 };
    M68KMMU030MemoryOps ops = mmu030_test_ops(&memory);
    M68KMMU030TranslateResult result;
    const uint32_t logical = UINT32_C(0x12345678);
    const unsigned tia_index = (logical >> 22) & 0x3ff;
    const unsigned tib_index = (logical >> 12) & 0x3ff;

    state.tc = mmu030_tc(12, 0, 10, 10, 0, 0);
    state.crp = (UINT64_C(0x7fff0002) << 32) | UINT32_C(0x1000);
    mmu030_test_putl(&memory, 0x1000 + tia_index * 4, 0x2002);
    mmu030_test_putl(&memory, 0x2000 + tib_index * 4, 0x7002);
    mmu030_test_putl(&memory, 0x7000, 0x00900001);
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA, 1, false,
                                     &result), ==, 0);
    g_assert_cmpuint(result.physical, ==, UINT32_C(0x00900678));
    g_assert_cmpuint(mmu030_test_getl(&memory, 0x7000), ==,
                     UINT32_C(0x00900009));

    memset(&memory, 0, sizeof(memory));
    mmu030_test_putl(&memory, 0x1000 + tia_index * 4, UINT32_C(0x2002));
    mmu030_test_putl(&memory, 0x2000 + tib_index * 4, UINT32_C(0x7002));
    mmu030_test_putl(&memory, 0x7000, M68K_MMU030_DESC_PAGE);
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA |
                                     MMU030_TEST_ACCESS_STORE, 1, false,
                                     &result), ==, 0);
    g_assert_cmpuint(mmu030_test_getl(&memory, 0x7000), ==,
                     M68K_MMU030_DESC_PAGE | M68K_MMU030_DESC_U |
                     M68K_MMU030_DESC_M);

    /* Long indirection uses the descriptor's second word as its address. */
    memset(&memory, 0, sizeof(memory));
    state.tc = mmu030_tc(12, 0, 10, 10, 0, 0);
    state.crp = (UINT64_C(0x7fff0003) << 32) | UINT32_C(0x1000);
    mmu030_test_putl(&memory, 0x1000 + tia_index * 8,
                     UINT32_C(0x7fff0003));
    mmu030_test_putl(&memory, 0x1000 + tia_index * 8 + 4,
                     UINT32_C(0x2000));
    mmu030_test_putl(&memory, 0x2000 + tib_index * 8,
                     M68K_MMU030_DESC_VALID8);
    mmu030_test_putl(&memory, 0x2000 + tib_index * 8 + 4,
                     UINT32_C(0x7000));
    mmu030_test_putl(&memory, 0x7000, M68K_MMU030_DESC_PAGE);
    mmu030_test_putl(&memory, 0x7004, UINT32_C(0x00900000));
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA |
                                     MMU030_TEST_ACCESS_STORE, 1, false,
                                     &result), ==, 0);
    g_assert_cmpuint(result.physical, ==, UINT32_C(0x00900678));
    g_assert_cmpuint(mmu030_test_getl(&memory, 0x7000), ==,
                     M68K_MMU030_DESC_PAGE | M68K_MMU030_DESC_U |
                     M68K_MMU030_DESC_M);

    /* NeXT's 8 KiB layout is 13 + 7 + 7 + 5 address bits. */
    memset(&memory, 0, sizeof(memory));
    state.tc = mmu030_tc(13, 0, 7, 7, 5, 0);
    state.crp = (UINT64_C(0x7fff0002) << 32) | UINT32_C(0x1000);
    const unsigned a = (logical >> 25) & 0x7f;
    const unsigned b = (logical >> 18) & 0x7f;
    const unsigned c = (logical >> 13) & 0x1f;
    mmu030_test_putl(&memory, 0x1000 + a * 4, 0x2002);
    mmu030_test_putl(&memory, 0x2000 + b * 4, 0x3002);
    mmu030_test_putl(&memory, 0x3000 + c * 4, 0x00e00001);
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA, 1, false,
                                     &result), ==, 0);
    g_assert_cmpuint(result.physical, ==, UINT32_C(0x00e01678));
    g_assert_cmpuint(result.page_size, ==, UINT32_C(8192));
}

static void test_mmu030_descriptor_status_and_errors(void)
{
    M68KMMU030State state = { 0 };
    M68KMMU030TestMemory memory = { 0 };
    M68KMMU030MemoryOps ops = mmu030_test_ops(&memory);
    M68KMMU030TranslateResult result;
    const uint32_t logical = UINT32_C(0x12345678);
    const unsigned tia_index = (logical >> 22) & 0x3ff;
    const unsigned tib_index = (logical >> 12) & 0x3ff;
    const uint32_t root_entry = 0x1000 + tia_index * 8;

    state.tc = mmu030_tc(12, 0, 10, 10, 0, 0);
    state.crp = (UINT64_C(0x7fff0003) << 32) | UINT32_C(0x1000);
    mmu030_test_putl(&memory, root_entry, UINT32_C(0x7fff0003) |
                     M68K_MMU030_DESC_S | M68K_MMU030_DESC_WP);
    mmu030_test_putl(&memory, root_entry + 4, UINT32_C(0x2000));
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA |
                                     MMU030_TEST_ACCESS_PTEST, 1, true,
                                     &result), ==, 0);
    g_assert_false(result.fault);
    g_assert_cmpuint(result.mmusr, ==,
                     M68K_MMU030_MMUSR_S | M68K_MMU030_MMUSR_WP | 1);
    g_assert_cmpuint(state.mmusr, ==, result.mmusr);
    g_assert_cmpuint(mmu030_test_getl(&memory, root_entry), ==,
                     UINT32_C(0x7fff0003) | M68K_MMU030_DESC_S |
                     M68K_MMU030_DESC_WP);

    /* Normal user access still faults on the same supervisor-only table. */
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA, 1, false,
                                     &result), ==, -1);
    g_assert_true(result.fault);

    /* An indirect long page accumulates S and M for a user PTEST. */
    memset(&memory, 0, sizeof(memory));
    state.crp = (UINT64_C(0x7fff0003) << 32) | UINT32_C(0x1000);
    mmu030_test_putl(&memory, root_entry, UINT32_C(0x7fff0003));
    mmu030_test_putl(&memory, root_entry + 4, UINT32_C(0x2000));
    const uint32_t indirect_entry = 0x2000 + tib_index * 8;
    mmu030_test_putl(&memory, indirect_entry, M68K_MMU030_DESC_VALID8);
    mmu030_test_putl(&memory, indirect_entry + 4, UINT32_C(0x7000));
    mmu030_test_putl(&memory, 0x7000,
                     M68K_MMU030_DESC_PAGE | M68K_MMU030_DESC_S |
                     M68K_MMU030_DESC_M);
    mmu030_test_putl(&memory, 0x7004, UINT32_C(0x00900000));
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA |
                                     MMU030_TEST_ACCESS_PTEST, 1, true,
                                     &result), ==, 0);
    g_assert_false(result.fault);
    g_assert_cmpuint(result.mmusr, ==,
                     M68K_MMU030_MMUSR_S | M68K_MMU030_MMUSR_M | 2);
    g_assert_cmpuint(state.mmusr, ==, result.mmusr);
    g_assert_cmpuint(mmu030_test_getl(&memory, root_entry), ==,
                     UINT32_C(0x7fff0003));
    g_assert_cmpuint(mmu030_test_getl(&memory, indirect_entry), ==,
                     M68K_MMU030_DESC_VALID8);
    g_assert_cmpuint(mmu030_test_getl(&memory, 0x7000), ==,
                     M68K_MMU030_DESC_PAGE | M68K_MMU030_DESC_S |
                     M68K_MMU030_DESC_M);

    /* PTESTW reports WP status without taking the write-protect fault. */
    memset(&memory, 0, sizeof(memory));
    state.crp = (UINT64_C(0x7fff0002) << 32) | UINT32_C(0x1000);
    mmu030_test_putl(&memory, 0x1000 + tia_index * 4, 0x2002);
    mmu030_test_putl(&memory, 0x2000 + tib_index * 4,
                     M68K_MMU030_DESC_PAGE | M68K_MMU030_DESC_WP);
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA |
                                     MMU030_TEST_ACCESS_STORE |
                                     MMU030_TEST_ACCESS_PTEST, 1, true,
                                     &result), ==, 0);
    g_assert_false(result.fault);
    g_assert_cmpuint(result.physical, ==, UINT32_C(0x00000678));
    g_assert_cmpint(result.prot, ==, PAGE_READ);
    g_assert_cmpuint(result.mmusr, ==, M68K_MMU030_MMUSR_WP | 2);
    g_assert_cmpuint(state.mmusr, ==, result.mmusr);
    g_assert_cmpuint(mmu030_test_getl(&memory, 0x2000 + tib_index * 4), ==,
                     M68K_MMU030_DESC_PAGE | M68K_MMU030_DESC_WP);

    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA |
                                     MMU030_TEST_ACCESS_PTEST, 1, true,
                                     &result), ==, 0);
    g_assert_cmpuint(result.physical, ==, UINT32_C(0x00000678));
    g_assert_cmpint(result.prot, ==, PAGE_READ);
    g_assert_cmpuint(result.mmusr, ==, M68K_MMU030_MMUSR_WP | 2);

    /* Probe history is read-only: a clear U/M descriptor remains untouched. */
    g_assert_cmpuint(mmu030_test_getl(&memory, 0x2000 + tib_index * 4), ==,
                     M68K_MMU030_DESC_PAGE | M68K_MMU030_DESC_WP);

    /* An ancestor WP is accumulated through the walk before the retry. */
    memset(&memory, 0, sizeof(memory));
    state.crp = (UINT64_C(0x7fff0002) << 32) | UINT32_C(0x1000);
    mmu030_test_putl(&memory, 0x1000 + tia_index * 4,
                     UINT32_C(0x2000) | M68K_MMU030_DESC_VALID4 |
                     M68K_MMU030_DESC_WP);
    mmu030_test_putl(&memory, 0x2000 + tib_index * 4,
                     M68K_MMU030_DESC_PAGE);
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA |
                                     MMU030_TEST_ACCESS_STORE, 1, false,
                                     &result), ==, -1);
    g_assert_true(result.fault);
    g_assert_false(result.bus_error);
    g_assert_cmpuint(result.physical, ==, UINT32_C(0x00000678));
    g_assert_cmpint(result.prot, ==, PAGE_READ);
    g_assert_cmpuint(result.mmusr, ==, M68K_MMU030_MMUSR_WP | 2);
    g_assert_cmpuint(mmu030_test_getl(&memory, 0x1000 + tia_index * 4), ==,
                     UINT32_C(0x2000) | M68K_MMU030_DESC_VALID4 |
                     M68K_MMU030_DESC_WP |
                     M68K_MMU030_DESC_U);
    g_assert_cmpuint(mmu030_test_getl(&memory, 0x2000 + tib_index * 4), ==,
                     M68K_MMU030_DESC_PAGE | M68K_MMU030_DESC_U);

    /* The CRP upper limit rejects the first logical index without a read. */
    memset(&memory, 0, sizeof(memory));
    state.crp = (UINT64_C(0x00000002) << 32) | UINT32_C(0x1000);
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA |
                                     MMU030_TEST_ACCESS_PTEST, 1, true,
                                     &result), ==, -1);
    g_assert_true(result.limit_violation);
    g_assert_cmpuint(result.mmusr, ==,
                     M68K_MMU030_MMUSR_L | M68K_MMU030_MMUSR_I);
    g_assert_cmpuint(memory.read_count, ==, 0);

    /* A long pointer descriptor limits its next logical index. */
    memset(&memory, 0, sizeof(memory));
    state.crp = (UINT64_C(0x7fff0003) << 32) | UINT32_C(0x1000);
    mmu030_test_putl(&memory, root_entry, UINT32_C(0x00000003) |
                     (UINT32_C(0x20) << 16));
    mmu030_test_putl(&memory, root_entry + 4, UINT32_C(0x2000));
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA |
                                     MMU030_TEST_ACCESS_PTEST, 1, true,
                                     &result), ==, -1);
    g_assert_true(result.limit_violation);
    g_assert_cmpuint(result.mmusr, ==,
                     M68K_MMU030_MMUSR_L | M68K_MMU030_MMUSR_I | 1);

    /* A successful write sets U and M; PTEST above did not set either. */
    memset(&memory, 0, sizeof(memory));
    state.crp = (UINT64_C(0x7fff0002) << 32) | UINT32_C(0x1000);
    mmu030_test_putl(&memory, 0x1000 + tia_index * 4, UINT32_C(0x2002));
    mmu030_test_putl(&memory, 0x2000 + tib_index * 4,
                     M68K_MMU030_DESC_PAGE);
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA |
                                     MMU030_TEST_ACCESS_STORE, 1, false,
                                     &result), ==, 0);
    g_assert_cmpuint(mmu030_test_getl(&memory, 0x1000 + tia_index * 4), ==,
                     UINT32_C(0x200a));
    g_assert_cmpuint(mmu030_test_getl(&memory, 0x2000 + tib_index * 4), ==,
                     M68K_MMU030_DESC_PAGE | M68K_MMU030_DESC_U |
                     M68K_MMU030_DESC_M);

    /* A failed physical write while setting U is a table bus error. */
    memset(&memory, 0, sizeof(memory));
    mmu030_test_putl(&memory, 0x1000 + tia_index * 4, UINT32_C(0x2002));
    memory.fail_write = true;
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA, 1, false,
                                     &result), ==, -1);
    g_assert_true(result.bus_error);
    g_assert_cmpuint(result.mmusr, ==,
                     M68K_MMU030_MMUSR_B | M68K_MMU030_MMUSR_I | 1);

    /* Invalid descriptors and table bus errors are reported distinctly. */
    memset(&memory, 0, sizeof(memory));
    mmu030_test_putl(&memory, 0x1000 + tia_index * 4, 0);
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA |
                                     MMU030_TEST_ACCESS_PTEST, 1, true,
                                     &result), ==, -1);
    g_assert_cmpuint(result.mmusr, ==, M68K_MMU030_MMUSR_I | 1);
    g_assert_cmpuint(mmu030_test_getl(&memory, 0x1000 + tia_index * 4), ==,
                     0);
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA, 1, false,
                                     &result), ==, -1);
    g_assert_cmpuint(mmu030_test_getl(&memory, 0x1000 + tia_index * 4), ==,
                     0);

    memset(&memory, 0, sizeof(memory));
    memory.fail_read = true;
    g_assert_cmpint(m68k_mmu030_walk(&state, &ops, logical,
                                     MMU030_TEST_ACCESS_DATA |
                                     MMU030_TEST_ACCESS_PTEST, 1, true,
                                     &result), ==, -1);
    g_assert_true(result.bus_error);
    g_assert_cmpuint(result.mmusr, ==,
                     M68K_MMU030_MMUSR_B | M68K_MMU030_MMUSR_I | 1);
}

static void mmu030_assert_short_access_frame(
    M68KMMU030State *state, M68KMMU030MemoryOps *ops,
    uint32_t logical_address, int access_type, uint8_t function_code,
    unsigned size, uint16_t expected_status, uint16_t saved_sr,
    uint32_t fault_pc)
{
    M68KMMU030TranslateResult result;
    uint8_t frame[M68K_MMU030_ACCESS_FRAME_SIZE];

    g_assert_cmpint(m68k_mmu030_walk(state, ops, logical_address,
                                     access_type, function_code, false,
                                     &result), ==, -1);
    g_assert_true(result.fault);
    g_assert_cmpuint(result.mmusr, ==, expected_status);

    state->fault_pending = true;
    state->fault_address = logical_address;
    state->fault_pc = fault_pc;
    state->fault_ssw = m68k_mmu030_make_ssw(size,
                                            access_type &
                                            MMU030_TEST_ACCESS_STORE,
                                            access_type &
                                            MMU030_TEST_ACCESS_CODE,
                                            function_code);
    state->fault_status = result.mmusr;
    g_assert_true(m68k_mmu030_build_short_access_frame(state, saved_sr, 8,
                                                       frame));
    g_assert_false(state->fault_pending);

    /* Format $A is the MC68030 short bus-cycle fault frame. */
    g_assert_cmpuint(lduw_be_p(frame + 0x00), ==, saved_sr);
    g_assert_cmpuint(ldl_be_p(frame + 0x02), ==, fault_pc);
    g_assert_cmpuint(lduw_be_p(frame + 0x06), ==, UINT16_C(0xa008));
    g_assert_cmpuint(lduw_be_p(frame + 0x08), ==, 0);
    g_assert_cmpuint(lduw_be_p(frame + 0x0a), ==, state->fault_ssw);
    g_assert_cmpuint(lduw_be_p(frame + 0x0c), ==, 0);
    g_assert_cmpuint(lduw_be_p(frame + 0x0e), ==, 0);
    g_assert_cmpuint(ldl_be_p(frame + 0x10), ==, logical_address);
    g_assert_cmpuint(lduw_be_p(frame + 0x14), ==, 0);
    g_assert_cmpuint(lduw_be_p(frame + 0x16), ==, 0);
    g_assert_cmpuint(ldl_be_p(frame + 0x18), ==, 0);
    g_assert_cmpuint(lduw_be_p(frame + 0x1c), ==, 0);
    g_assert_cmpuint(lduw_be_p(frame + 0x1e), ==, 0);
}

static void test_mmu030_access_error_frame(void)
{
    M68KMMU030State state = { 0 };
    M68KMMU030TestMemory memory = { 0 };
    M68KMMU030MemoryOps ops = mmu030_test_ops(&memory);
    const uint32_t logical = UINT32_C(0x12345678);
    const unsigned tia_index = (logical >> 22) & 0x3ff;
    const unsigned tib_index = (logical >> 12) & 0x3ff;
    const uint32_t root_entry = UINT32_C(0x1000) + tia_index * 4;
    const uint32_t page_entry = UINT32_C(0x2000) + tib_index * 4;
    const uint16_t saved_sr = UINT16_C(0x2015);

    state.tc = mmu030_tc(12, 0, 10, 10, 0, 0);
    state.crp = (UINT64_C(0x7fff0002) << 32) | UINT32_C(0x1000);

    /* The real walker supplies the invalid-descriptor status. */
    mmu030_test_putl(&memory, root_entry, 0);
    mmu030_assert_short_access_frame(
        &state, &ops, logical, MMU030_TEST_ACCESS_DATA, 1, 4,
        M68K_MMU030_MMUSR_I | 1, saved_sr, UINT32_C(0x00123456));

    /* Write protection is a format-A access fault, not a probe result. */
    memset(&memory, 0, sizeof(memory));
    mmu030_test_putl(&memory, root_entry, UINT32_C(0x2002));
    mmu030_test_putl(&memory, page_entry,
                     M68K_MMU030_DESC_PAGE | M68K_MMU030_DESC_WP);
    mmu030_assert_short_access_frame(
        &state, &ops, logical,
        MMU030_TEST_ACCESS_DATA | MMU030_TEST_ACCESS_STORE, 1, 2,
        M68K_MMU030_MMUSR_WP | 2, saved_sr, UINT32_C(0x00123458));

    /* A user access to a long supervisor-only descriptor reports S. */
    memset(&memory, 0, sizeof(memory));
    state.crp = (UINT64_C(0x7fff0003) << 32) | UINT32_C(0x1000);
    mmu030_test_putl(&memory, UINT32_C(0x1000) + tia_index * 8,
                     M68K_MMU030_DESC_VALID8 | M68K_MMU030_DESC_S);
    mmu030_test_putl(&memory, UINT32_C(0x1000) + tia_index * 8 + 4,
                     UINT32_C(0x2000));
    mmu030_assert_short_access_frame(
        &state, &ops, logical, MMU030_TEST_ACCESS_DATA, 1, 1,
        M68K_MMU030_MMUSR_S | 1, saved_sr, UINT32_C(0x0012345a));

    /* A table read bus error is reported with B and the table level. */
    memset(&memory, 0, sizeof(memory));
    state.crp = (UINT64_C(0x7fff0002) << 32) | UINT32_C(0x1000);
    memory.fail_read = true;
    mmu030_assert_short_access_frame(
        &state, &ops, logical, MMU030_TEST_ACCESS_DATA, 5, 4,
        M68K_MMU030_MMUSR_B | M68K_MMU030_MMUSR_I | 1, saved_sr,
        UINT32_C(0x0012345c));

    g_assert_cmpuint(m68k_mmu030_make_ssw(1, false, false, 1), ==,
                     UINT16_C(0x0151));
    g_assert_cmpuint(m68k_mmu030_make_ssw(2, true, false, 5), ==,
                     UINT16_C(0x0125));
    g_assert_cmpuint(m68k_mmu030_make_ssw(4, false, false, 5), ==,
                     UINT16_C(0x0145));
    g_assert_cmpuint(m68k_mmu030_make_ssw(3, true, false, 5), ==,
                     UINT16_C(0x0135));
    /* Instruction-pipe faults do not claim a data-cycle rerun. */
    g_assert_cmpuint(m68k_mmu030_make_ssw(2, false, true, 2), ==, 0);
}

static void test_mmu030_access_error_rte_roundtrip(void)
{
    M68KMMU030State state = {
        .fault_pending = true,
        .fault_address = UINT32_C(0x00abcdef),
        .fault_pc = UINT32_C(0x00123456),
        .fault_ssw = UINT16_C(0x0141),
    };
    uint8_t frame[M68K_MMU030_ACCESS_FRAME_SIZE];
    const uint32_t stack_top = UINT32_C(0x00001000);
    const uint32_t frame_sp = stack_top - M68K_MMU030_ACCESS_FRAME_SIZE;
    uint32_t rte_sp = frame_sp + 8;
    uint16_t format;

    g_assert_true(m68k_mmu030_build_short_access_frame(
        &state, UINT16_C(0x2015), 8, frame));
    g_assert_false(state.fault_pending);

    /* Model m68k_rte's common SR/PC/format reads and format-$A tail skip. */
    g_assert_cmpuint(lduw_be_p(frame + 0x00), ==, UINT16_C(0x2015));
    g_assert_cmpuint(ldl_be_p(frame + 0x02), ==, UINT32_C(0x00123456));
    format = lduw_be_p(frame + 0x06);
    g_assert_cmpuint(format, ==, UINT16_C(0xa008));
    rte_sp += m68k_mmu030_rte_frame_tail_size(format);
    g_assert_cmpuint(rte_sp, ==, stack_top);
    g_assert_cmpuint(rte_sp - frame_sp, ==, M68K_MMU030_ACCESS_FRAME_SIZE);
    g_assert_cmpuint(m68k_mmu030_rte_frame_tail_size(UINT16_C(0x7008)), ==,
                     0);
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
        state.atc[i].status = M68K_MMU030_ATC_VALID |
                              ((i & 7) << M68K_MMU030_ATC_FC_SHIFT) |
                              (12 << M68K_MMU030_ATC_PAGE_BITS_SHIFT) |
                              (i & 0x3f);
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

static unsigned mmu030_test_atc_valid_count(const M68KMMU030State *state)
{
    unsigned count = 0;

    for (unsigned i = 0; i < M68K_MMU030_ATC_ENTRIES; i++) {
        count += (state->atc[i].status & M68K_MMU030_ATC_VALID) != 0;
    }
    return count;
}

static M68KMMU030TranslateResult mmu030_test_atc_result(
    uint32_t physical, uint32_t page_size, int prot)
{
    return (M68KMMU030TranslateResult) {
        .physical = physical,
        .page_size = page_size,
        .prot = prot,
    };
}

static void test_mmu030_atc_entries_and_matching(void)
{
    M68KMMU030State state = { 0 };
    M68KMMU030TranslateResult result;
    M68KMMU030TranslateResult mapping;

    mapping = mmu030_test_atc_result(UINT32_C(0x80012000), 4096,
                                     PAGE_READ | PAGE_WRITE);
    m68k_mmu030_atc_fill(&state, UINT32_C(0x12345678), 1, &mapping);
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==, 1);
    g_assert_cmpuint(state.atc[0].logical, ==, UINT32_C(0x12345000));

    g_assert_true(m68k_mmu030_atc_lookup(&state, UINT32_C(0x1234567c),
                                         MMU030_TEST_ACCESS_DATA, 1,
                                         &result));
    g_assert_cmpuint(result.physical, ==, UINT32_C(0x8001267c));
    g_assert_cmpuint(result.page_size, ==, 4096);
    g_assert_cmpint(result.prot, ==, PAGE_READ | PAGE_WRITE);

    /* The three function-code bits are part of the tag, not only FC2. */
    g_assert_false(m68k_mmu030_atc_lookup(&state, UINT32_C(0x1234567c),
                                          MMU030_TEST_ACCESS_DATA, 5,
                                          &result));

    mapping = mmu030_test_atc_result(UINT32_C(0x90012000), 4096,
                                     PAGE_READ | PAGE_WRITE);
    m68k_mmu030_atc_fill(&state, UINT32_C(0x12345678), 5, &mapping);
    g_assert_true(m68k_mmu030_atc_lookup(&state, UINT32_C(0x1234567c),
                                         MMU030_TEST_ACCESS_DATA, 5,
                                         &result));
    g_assert_cmpuint(result.physical, ==, UINT32_C(0x9001267c));

    /* Page-size bits in the tag determine which low address bits are free. */
    mapping = mmu030_test_atc_result(UINT32_C(0xa0010000), 8192,
                                     PAGE_READ | PAGE_WRITE);
    m68k_mmu030_atc_fill(&state, UINT32_C(0x01012345), 2, &mapping);
    g_assert_true(m68k_mmu030_atc_lookup(&state, UINT32_C(0x01013fff),
                                         MMU030_TEST_ACCESS_DATA, 2,
                                         &result));
    g_assert_cmpuint(result.physical, ==, UINT32_C(0xa0011fff));
    g_assert_false(m68k_mmu030_atc_lookup(&state, UINT32_C(0x01014000),
                                          MMU030_TEST_ACCESS_DATA, 2,
                                          &result));

    /* FC2 is also checked independently of the address tag. */
    g_assert_false(m68k_mmu030_atc_lookup(&state, UINT32_C(0x1234567c),
                                          MMU030_TEST_ACCESS_DATA, 3,
                                          &result));
    g_assert_true(m68k_mmu030_atc_lookup(&state, UINT32_C(0x1234567c),
                                         MMU030_TEST_ACCESS_SUPER |
                                         MMU030_TEST_ACCESS_DATA, 5,
                                         &result));
}

static void test_mmu030_atc_permissions_and_replacement(void)
{
    M68KMMU030State state = { 0 };
    M68KMMU030TranslateResult result;
    M68KMMU030TranslateResult mapping;

    mapping = mmu030_test_atc_result(UINT32_C(0x10000000), 4096,
                                     PAGE_READ);
    m68k_mmu030_atc_fill(&state, UINT32_C(0x00401234), 1, &mapping);
    g_assert_true(m68k_mmu030_atc_lookup(&state, UINT32_C(0x00401238),
                                         MMU030_TEST_ACCESS_DATA, 1,
                                         &result));
    g_assert_false(result.fault);
    g_assert_cmpint(result.prot, ==, PAGE_READ);
    g_assert_true(m68k_mmu030_atc_lookup(&state, UINT32_C(0x00401238),
                                         MMU030_TEST_ACCESS_DATA |
                                         MMU030_TEST_ACCESS_STORE, 1,
                                         &result));
    g_assert_true(result.fault);
    g_assert_cmpuint(result.mmusr, ==, M68K_MMU030_MMUSR_WP);

    /* A full ATC uses a deterministic round-robin replacement cursor. */
    m68k_mmu030_reset(&state);
    mapping = mmu030_test_atc_result(UINT32_C(0x20000000), 4096,
                                     PAGE_READ | PAGE_WRITE);
    for (unsigned i = 0; i < M68K_MMU030_ATC_ENTRIES; i++) {
        m68k_mmu030_atc_fill(&state, UINT32_C(0x10000000) + i * 0x1000,
                             1, &mapping);
    }
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==,
                     M68K_MMU030_ATC_ENTRIES);
    g_assert_cmpuint(state.atc_next, ==, 0);

    m68k_mmu030_atc_fill(&state, UINT32_C(0x20000000), 1, &mapping);
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==,
                     M68K_MMU030_ATC_ENTRIES);
    g_assert_cmpuint(state.atc[0].logical, ==, UINT32_C(0x20000000));
    g_assert_cmpuint(state.atc_next, ==, 1);
}

static void test_mmu030_atc_descriptor_attributes_and_errors(void)
{
    M68KMMU030State state = { 0 };
    M68KMMU030TranslateResult result;
    M68KMMU030TranslateResult mapping;

    mapping = mmu030_test_atc_result(UINT32_C(0x40000000), 4096,
                                     PAGE_READ | PAGE_WRITE);
    mapping.write_protect = true;
    mapping.supervisor_only = true;
    mapping.modified = true;
    mapping.cache_inhibit = true;
    m68k_mmu030_atc_fill(&state, UINT32_C(0x00604000), 5, &mapping);
    g_assert_true(m68k_mmu030_atc_lookup(&state, UINT32_C(0x00604004),
                                         MMU030_TEST_ACCESS_DATA, 5,
                                         &result));
    g_assert_false(result.fault);
    g_assert_true(result.write_protect);
    g_assert_true(result.supervisor_only);
    g_assert_true(result.modified);
    g_assert_true(result.cache_inhibit);
    g_assert_cmpint(result.prot, ==, PAGE_READ);

    /* Invalid/limited/bus-error walks leave a B entry for level-zero PTEST. */
    m68k_mmu030_reset(&state);
    mapping = mmu030_test_atc_result(0, 4096, PAGE_READ | PAGE_WRITE);
    mapping.fault = true;
    mapping.atc_error = true;
    mapping.mmusr = M68K_MMU030_MMUSR_I;
    m68k_mmu030_atc_fill(&state, UINT32_C(0x00708000), 1, &mapping);
    g_assert_true(m68k_mmu030_atc_lookup(&state, UINT32_C(0x00708004),
                                         MMU030_TEST_ACCESS_DATA, 1,
                                         &result));
    g_assert_true(result.fault);
    g_assert_cmpuint(result.mmusr, ==,
                     M68K_MMU030_MMUSR_B | M68K_MMU030_MMUSR_I);
}

static void test_mmu030_atc_preload_and_level_zero_ptest(void)
{
    M68KMMU030State state = { 0 };
    M68KMMU030TestMemory memory = { 0 };
    M68KMMU030MemoryOps ops = mmu030_test_ops(&memory);
    M68KMMU030TranslateResult result;
    const uint32_t logical = UINT32_C(0x12345678);
    const unsigned root_index = (logical >> 22) & 0x3ff;
    const unsigned page_index = (logical >> 12) & 0x3ff;

    state.tc = mmu030_tc(12, 0, 10, 10, 0, 0);
    state.crp = (UINT64_C(0x7fff0002) << 32) | UINT32_C(0x1000);
    mmu030_test_putl(&memory, UINT32_C(0x1000) + root_index * 4,
                     UINT32_C(0x2002));
    mmu030_test_putl(&memory, UINT32_C(0x2000) + page_index * 4,
                     UINT32_C(0x00abc001));

    /* PLOAD must leave the pre-existing MMUSR untouched, even on a probe. */
    state.mmusr = UINT16_C(0x55aa);
    g_assert_cmpint(m68k_mmu030_atc_preload(
                        &state, &ops, logical,
                        MMU030_TEST_ACCESS_DATA | MMU030_TEST_ACCESS_PTEST, 1,
                        &result), ==, 0);
    g_assert_cmpuint(state.mmusr, ==, UINT16_C(0x55aa));
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==, 1);

    memory.read_count = 0;
    g_assert_true(m68k_mmu030_atc_ptest(&state, logical, false, 1,
                                       &result));
    g_assert_cmpuint(result.physical, ==, UINT32_C(0x00abc678));
    g_assert_cmpuint(result.mmusr, ==, 0);
    g_assert_cmpuint(memory.read_count, ==, 0);

    /* The runtime translation entry point must use the ATC-only PTEST path. */
    memory.read_count = 0;
    memory.write_count = 0;
    g_assert_cmpint(m68k_mmu030_translate_state(
                        &state, &ops, logical, MMU030_TEST_ACCESS_PTEST, 1,
                        true, &result), ==, 0);
    g_assert_cmpuint(result.physical, ==, UINT32_C(0x00abc678));
    g_assert_cmpuint(result.mmusr, ==, 0);
    g_assert_cmpuint(memory.read_count, ==, 0);
    g_assert_cmpuint(memory.write_count, ==, 0);

    /* Level zero never falls through to a table walk on an ATC miss. */
    g_assert_true(m68k_mmu030_atc_ptest(&state, UINT32_C(0x76543210),
                                       false, 1, &result));
    g_assert_true(result.fault);
    g_assert_cmpuint(result.mmusr, ==, M68K_MMU030_MMUSR_I);
    g_assert_cmpuint(memory.read_count, ==, 0);

    memory.read_count = 0;
    memory.write_count = 0;
    g_assert_cmpint(m68k_mmu030_translate_state(
                        &state, &ops, UINT32_C(0x76543210),
                        MMU030_TEST_ACCESS_PTEST, 1, true, &result), ==, 0);
    g_assert_true(result.fault);
    g_assert_cmpuint(result.mmusr, ==, M68K_MMU030_MMUSR_I);
    g_assert_cmpuint(memory.read_count, ==, 0);
    g_assert_cmpuint(memory.write_count, ==, 0);

    state.mmusr = UINT16_C(0xaa55);
    memory.fail_read = true;
    g_assert_cmpint(m68k_mmu030_atc_preload(
                        &state, &ops, logical,
                        MMU030_TEST_ACCESS_DATA | MMU030_TEST_ACCESS_PTEST, 1,
                        &result), ==, -1);
    g_assert_cmpuint(state.mmusr, ==, UINT16_C(0xaa55));
    memory.fail_read = false;

    /* PLOAD is allowed to populate the ATC while TC.E is clear. */
    state.tc &= ~M68K_MMU030_TC_ENABLE;
    m68k_mmu030_atc_flush_all(&state);
    g_assert_cmpint(m68k_mmu030_atc_preload(
                        &state, &ops, logical, MMU030_TEST_ACCESS_DATA, 1,
                        &result), ==, 0);
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==, 1);
}

typedef struct M68KMMU030FlushTrace {
    unsigned all_count;
    unsigned range_count;
    uint32_t range_address;
    uint32_t range_size;
    const M68KMMU030State *state;
    uint32_t observed_tc;
    uint64_t observed_crp;
} M68KMMU030FlushTrace;

static void mmu030_test_flush_all(void *opaque)
{
    M68KMMU030FlushTrace *trace = opaque;

    trace->all_count++;
    if (trace->state) {
        trace->observed_tc = trace->state->tc;
        trace->observed_crp = trace->state->crp;
    }
}

static void mmu030_test_flush_range(void *opaque, uint32_t address,
                                    uint32_t size)
{
    M68KMMU030FlushTrace *trace = opaque;

    trace->range_count++;
    trace->range_address = address;
    trace->range_size = size;
}

static void test_mmu030_atc_coherent_flush_wrappers(void)
{
    M68KMMU030State state = { 0 };
    M68KMMU030FlushTrace trace = { 0 };
    M68KMMU030TranslateResult mapping;

    mapping = mmu030_test_atc_result(UINT32_C(0x30000000), 4096,
                                     PAGE_READ | PAGE_WRITE);
    m68k_mmu030_atc_fill(&state, UINT32_C(0x00401000), 1, &mapping);
    mapping.physical = UINT32_C(0x30002000);
    mapping.page_size = 8192;
    m68k_mmu030_atc_fill(&state, UINT32_C(0x00402000), 5, &mapping);
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==, 2);

    state.tc = mmu030_tc(12, 0, 10, 10, 0, 0);
    m68k_mmu030_atc_flush_page_coherent(
        &state, UINT32_C(0x00402004), 5, 7, mmu030_test_flush_range, &trace);
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==, 1);
    g_assert_cmpuint(trace.range_count, ==, 1);
    g_assert_cmpuint(trace.range_address, ==, UINT32_C(0x00402000));
    g_assert_cmpuint(trace.range_size, ==, 8192);

    /* The derived TLB scope is flushed even when the ATC has no tag. */
    state.tc = mmu030_tc(13, 0, 10, 9, 0, 0);
    m68k_mmu030_atc_flush_page_coherent(
        &state, UINT32_C(0x00500004), 5, 7, mmu030_test_flush_range, &trace);
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==, 1);
    g_assert_cmpuint(trace.range_count, ==, 2);
    g_assert_cmpuint(trace.range_address, ==, UINT32_C(0x00500000));
    g_assert_cmpuint(trace.range_size, ==, 8192);

    m68k_mmu030_atc_flush_fc_coherent(
        &state, 1, 7, mmu030_test_flush_all, &trace);
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==, 0);
    g_assert_cmpuint(trace.all_count, ==, 1);

    m68k_mmu030_atc_fill(&state, UINT32_C(0x00403000), 1, &mapping);
    m68k_mmu030_atc_flush_all_coherent(&state, mmu030_test_flush_all,
                                       &trace);
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==, 0);
    g_assert_cmpuint(trace.all_count, ==, 2);
}

static void test_mmu030_control_reconfigure(void)
{
    M68KMMU030State state = { 0 };
    M68KMMU030ControlState control;
    M68KMMU030FlushTrace trace = { .state = &state };
    M68KMMU030TranslateResult mapping;
    uint64_t previous_crp;
    const uint32_t old_tc = mmu030_tc(12, 0, 10, 10, 0, 0);
    const uint32_t new_tc = mmu030_tc(13, 0, 10, 9, 0, 0);

    state.crp = (UINT64_C(0x7fff0002) << 32) | UINT32_C(0x1000);
    state.srp = (UINT64_C(0x7fff0002) << 32) | UINT32_C(0x2000);
    state.tc = old_tc;
    state.tt[0] = UINT32_C(0x12348000);
    state.tt[1] = UINT32_C(0x56788000);
    mapping = mmu030_test_atc_result(UINT32_C(0x30000000), 4096,
                                     PAGE_READ | PAGE_WRITE);
    m68k_mmu030_atc_fill(&state, UINT32_C(0x00401000), 1, &mapping);
    state.atc_next = 17;

    control = (M68KMMU030ControlState) {
        .crp = state.crp,
        .srp = state.srp,
        .tc = new_tc,
        .tt = { state.tt[0], state.tt[1] },
    };
    g_assert_true(m68k_mmu030_reconfigure(
        &state, &control, true, mmu030_test_flush_all, &trace));
    g_assert_cmpuint(trace.all_count, ==, 1);
    g_assert_cmpuint(trace.observed_tc, ==, old_tc);
    g_assert_cmpuint(state.tc, ==, new_tc);
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==, 0);
    g_assert_cmpuint(state.atc_next, ==, 17);

    previous_crp = state.crp;
    control.crp += UINT64_C(0x1000);
    control.tt[0] ^= UINT32_C(0x10000);
    g_assert_true(m68k_mmu030_reconfigure(
        &state, &control, true, mmu030_test_flush_all, &trace));
    g_assert_cmpuint(trace.all_count, ==, 2);
    g_assert_cmpuint(trace.observed_crp, ==, previous_crp);
    g_assert_cmpuint(state.crp, ==, control.crp);
    g_assert_cmpuint(state.tt[0], ==, control.tt[0]);

    /* An invalid TC is rejected without flushing or changing controls. */
    control.tc = mmu030_tc(7, 0, 10, 11, 0, 0);
    g_assert_false(m68k_mmu030_reconfigure(
        &state, &control, true, mmu030_test_flush_all, &trace));
    g_assert_cmpuint(trace.all_count, ==, 2);
    g_assert_cmpuint(state.tc, ==, new_tc);

    /* PMOVEFD can suppress the flush while still changing the controls. */
    control.tc = old_tc;
    m68k_mmu030_atc_fill(&state, UINT32_C(0x00402000), 1, &mapping);
    g_assert_true(m68k_mmu030_reconfigure(
        &state, &control, false, mmu030_test_flush_all, &trace));
    g_assert_cmpuint(trace.all_count, ==, 2);
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==, 1);
    g_assert_cmpuint(state.tc, ==, old_tc);

    /* FD=0 flushes even when a register write leaves controls unchanged. */
    state.atc_next = 9;
    g_assert_true(m68k_mmu030_reconfigure(
        &state, &control, true, mmu030_test_flush_all, &trace));
    g_assert_cmpuint(trace.all_count, ==, 3);
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==, 0);
    g_assert_cmpuint(state.atc_next, ==, 9);

    /* FD=1 leaves an unchanged control image and its ATC untouched. */
    m68k_mmu030_atc_fill(&state, UINT32_C(0x00403000), 1, &mapping);
    state.atc_next = 11;
    g_assert_true(m68k_mmu030_reconfigure(
        &state, &control, false, mmu030_test_flush_all, &trace));
    g_assert_cmpuint(trace.all_count, ==, 3);
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==, 1);
    g_assert_cmpuint(state.atc_next, ==, 11);
}

static void test_mmu030_atc_flush_scopes(void)
{
    M68KMMU030State state = { 0 };
    M68KMMU030TranslateResult mapping;

    mapping = mmu030_test_atc_result(UINT32_C(0x30000000), 4096,
                                     PAGE_READ | PAGE_WRITE);
    m68k_mmu030_atc_fill(&state, UINT32_C(0x00401000), 1, &mapping);
    mapping.physical = UINT32_C(0x30001000);
    m68k_mmu030_atc_fill(&state, UINT32_C(0x00401000), 5, &mapping);
    mapping.physical = UINT32_C(0x30002000);
    m68k_mmu030_atc_fill(&state, UINT32_C(0x00402000), 1, &mapping);
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==, 3);

    m68k_mmu030_atc_flush_page(&state, UINT32_C(0x00401004), 1, 7);
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==, 2);
    g_assert_false(m68k_mmu030_atc_lookup(&state, UINT32_C(0x00401004),
                                          MMU030_TEST_ACCESS_DATA, 1,
                                          &(M68KMMU030TranslateResult) { 0 }));
    g_assert_true(m68k_mmu030_atc_lookup(&state, UINT32_C(0x00401004),
                                         MMU030_TEST_ACCESS_DATA, 5,
                                         &(M68KMMU030TranslateResult) { 0 }));

    m68k_mmu030_atc_flush_fc(&state, 5, 7);
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==, 1);
    m68k_mmu030_atc_flush_fc(&state, 0, 4);
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==, 0);

    /* Flush-all invalidates entries but does not perturb replacement state. */
    state.atc_next = 19;
    m68k_mmu030_atc_flush_all(&state);
    g_assert_cmpuint(mmu030_test_atc_valid_count(&state), ==, 0);
    g_assert_cmpuint(state.atc_next, ==, 19);
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
    g_test_add_func("/m68k/mmu030/descriptor-walker",
                    test_mmu030_descriptor_walker);
    g_test_add_func("/m68k/mmu030/descriptor-roots-fcl",
                    test_mmu030_descriptor_roots_and_fcl);
    g_test_add_func("/m68k/mmu030/descriptor-long-direct-early",
                    test_mmu030_descriptor_long_direct_and_early);
    g_test_add_func("/m68k/mmu030/descriptor-indirect-8k",
                    test_mmu030_descriptor_indirect_and_8k);
    g_test_add_func("/m68k/mmu030/descriptor-status-errors",
                    test_mmu030_descriptor_status_and_errors);
    g_test_add_func("/m68k/mmu030/access-error-frame",
                    test_mmu030_access_error_frame);
    g_test_add_func("/m68k/mmu030/access-error-rte-roundtrip",
                    test_mmu030_access_error_rte_roundtrip);
    g_test_add_func("/m68k/mmu030/atc-entries-matching",
                    test_mmu030_atc_entries_and_matching);
    g_test_add_func("/m68k/mmu030/atc-permissions-replacement",
                    test_mmu030_atc_permissions_and_replacement);
    g_test_add_func("/m68k/mmu030/atc-descriptor-attributes-errors",
                    test_mmu030_atc_descriptor_attributes_and_errors);
    g_test_add_func("/m68k/mmu030/atc-preload-ptest",
                    test_mmu030_atc_preload_and_level_zero_ptest);
    g_test_add_func("/m68k/mmu030/atc-coherent-flush-wrappers",
                    test_mmu030_atc_coherent_flush_wrappers);
    g_test_add_func("/m68k/mmu030/control-reconfigure",
                    test_mmu030_control_reconfigure);
    g_test_add_func("/m68k/mmu030/atc-flush-scopes",
                    test_mmu030_atc_flush_scopes);
    if (g_getenv("QTEST_QEMU_BINARY")) {
        g_test_add_func("/m68k/mmu030/cpu-vmstate-gating",
                        test_cpu_vmstate_gating);
        g_test_add_func("/m68k/mmu030/cpu-migration-stream",
                        test_cpu_migration_stream);
    }

    return g_test_run();
}
