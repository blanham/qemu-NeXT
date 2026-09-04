/*
 * MC68030 PMMU architectural state.
 *
 * Copyright (c) 2026 Bryce Lanham
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "target/m68k/mmu030.h"
#include "exec/page-protection.h"
#include "qemu/bswap.h"

/*
 * Keep the walker independent of cpu.h so that it can be unit-tested in a
 * target-independent build.  These values are the m68k access_type bits.
 */
#define M68K_MMU030_ACCESS_STORE UINT32_C(0x02)
#define M68K_MMU030_ACCESS_PTEST UINT32_C(0x08)
#define M68K_MMU030_ACCESS_CODE UINT32_C(0x10)

#define M68K_MMU030_DEFAULT_PAGE_BITS 12

#define M68K_MMU030_TT_ENABLE        (UINT32_C(1) << 15)
#define M68K_MMU030_TT_CACHE_INHIBIT (UINT32_C(1) << 10)
#define M68K_MMU030_TT_RW            (UINT32_C(1) << 9)
#define M68K_MMU030_TT_RW_MASK       (UINT32_C(1) << 8)

bool m68k_mmu030_pmove_decode(uint16_t extension, unsigned *reg,
                              unsigned *size, bool *direction, bool *fd)
{
    unsigned top = (extension >> 13) & 7;
    unsigned p = (extension >> 10) & 7;
    bool is_memory_to_register = (extension & UINT16_C(0x0200)) == 0;
    bool is_fd = (extension & UINT16_C(0x0100)) != 0;
    unsigned register_id;
    unsigned transfer_size;

    /* Bits 7:0 are reserved in all PMOVE forms. */
    if ((extension & UINT16_C(0x00ff)) != 0) {
        return false;
    }

    switch (top) {
    case 0:
        /* P=010 and P=011 select TT0 and TT1 respectively. */
        if (p == 2) {
            register_id = M68K_MMU030_PMOVE_TT0;
        } else if (p == 3) {
            register_id = M68K_MMU030_PMOVE_TT1;
        } else {
            return false;
        }
        transfer_size = sizeof(uint32_t);
        break;
    case 2:
        /* P=000, P=010, and P=011 select TC, SRP, and CRP. */
        if (p == 0) {
            register_id = M68K_MMU030_PMOVE_TC;
            transfer_size = sizeof(uint32_t);
        } else if (p == 2) {
            register_id = M68K_MMU030_PMOVE_SRP;
            transfer_size = sizeof(uint64_t);
        } else if (p == 3) {
            register_id = M68K_MMU030_PMOVE_CRP;
            transfer_size = sizeof(uint64_t);
        } else {
            return false;
        }
        break;
    case 3:
        /* The MMUSR form has no FD bit and is a word transfer. */
        if (p != 0 || is_fd) {
            return false;
        }
        register_id = M68K_MMU030_PMOVE_MMUSR;
        transfer_size = sizeof(uint16_t);
        break;
    default:
        return false;
    }

    /* PMOVEFD is defined only for memory-to-register transfers. */
    if (is_fd && !is_memory_to_register) {
        return false;
    }

    if (reg) {
        *reg = register_id;
    }
    if (size) {
        *size = transfer_size;
    }
    if (direction) {
        *direction = !is_memory_to_register;
    }
    if (fd) {
        *fd = is_fd;
    }
    return true;
}

bool m68k_mmu030_control_decode(uint16_t extension,
                                M68KMMU030ControlDecode *decode)
{
    unsigned format = (extension >> 13) & 7;
    unsigned mode = (extension >> 10) & 7;
    unsigned fc_field = extension & 0x1f;

    if (!decode) {
        return false;
    }
    memset(decode, 0, sizeof(*decode));
    decode->mode = mode;

    /* The function-code field is shared by all three operations. */
    switch (fc_field & 0x18) {
    case 0x10:
        decode->function_code_source = M68K_MMU030_FC_IMMEDIATE;
        decode->function_code_value = fc_field & 7;
        break;
    case 0x08:
        decode->function_code_source = M68K_MMU030_FC_DREG;
        decode->function_code_value = fc_field & 7;
        break;
    case 0:
        if (fc_field == 0) {
            decode->function_code_source = M68K_MMU030_FC_SFC;
        } else if (fc_field == 1) {
            decode->function_code_source = M68K_MMU030_FC_DFC;
        } else {
            return false;
        }
        break;
    default:
        return false;
    }

    switch (format) {
    case 1: /* PLOAD and PFLUSH share the 030 PMMU format. */
        if (mode == 0) {
            /* PLOAD: bit 8 and bits 7:5 are reserved. */
            if (extension & UINT16_C(0x01e0)) {
                return false;
            }
            decode->operation = M68K_MMU030_CONTROL_PLOAD;
            decode->is_write = (extension & UINT16_C(0x0200)) == 0;
            return true;
        }
        if (mode != 1 && mode != 4 && mode != 6) {
            return false;
        }
        /* PFLUSH has no R/W or A field. */
        if (extension & UINT16_C(0x0300)) {
            return false;
        }
        decode->operation = M68K_MMU030_CONTROL_PFLUSH;
        decode->mask = (extension >> 5) & 7;
        if (mode == 1 && (decode->mask != 0 || fc_field != 0)) {
            return false;
        }
        return true;

    case 4: /* PTEST. */
        decode->operation = M68K_MMU030_CONTROL_PTEST;
        decode->level = mode;
        decode->is_write = (extension & UINT16_C(0x0200)) == 0;
        decode->has_address_register = (extension & UINT16_C(0x0100)) != 0;
        decode->address_register = (extension >> 5) & 7;
        /*
         * The register field is reserved when the A bit is clear.  Level
         * zero additionally forbids the optional descriptor-address result.
         */
        if (!decode->has_address_register && decode->address_register != 0) {
            return false;
        }
        if (decode->level == 0 && decode->has_address_register) {
            return false;
        }
        return true;

    default:
        return false;
    }
}

static unsigned m68k_mmu030_atc_page_bits(uint32_t status)
{
    unsigned page_bits = (status & M68K_MMU030_ATC_PAGE_BITS_MASK) >>
                          M68K_MMU030_ATC_PAGE_BITS_SHIFT;

    return page_bits >= 8 && page_bits <= 15 ? page_bits : 0;
}

static uint32_t m68k_mmu030_atc_page_size(uint32_t status)
{
    unsigned page_bits = m68k_mmu030_atc_page_bits(status);

    return page_bits ? UINT32_C(1) << page_bits : 0;
}

static uint32_t m68k_mmu030_atc_active_page_size(
    const M68KMMU030State *state)
{
    unsigned page_bits = (state->tc >> M68K_MMU030_TC_PS_SHIFT) & 0xf;

    if (page_bits < 8 || page_bits > 15) {
        page_bits = M68K_MMU030_DEFAULT_PAGE_BITS;
    }
    return UINT32_C(1) << page_bits;
}

static bool m68k_mmu030_atc_entry_valid(const M68KMMU030ATCEntry *entry)
{
    return (entry->status & M68K_MMU030_ATC_VALID) != 0 &&
           m68k_mmu030_atc_page_bits(entry->status) != 0;
}

static uint8_t m68k_mmu030_atc_entry_fc(const M68KMMU030ATCEntry *entry)
{
    return (entry->status & M68K_MMU030_ATC_FC_MASK) >>
           M68K_MMU030_ATC_FC_SHIFT;
}

static bool m68k_mmu030_atc_fc_match(const M68KMMU030ATCEntry *entry,
                                     uint8_t function_code,
                                     uint8_t function_code_mask)
{
    return (((m68k_mmu030_atc_entry_fc(entry) ^ function_code) &
             function_code_mask & 7) == 0);
}

static bool m68k_mmu030_atc_address_match(
    const M68KMMU030ATCEntry *entry, uint32_t logical_address)
{
    uint32_t page_size = m68k_mmu030_atc_page_size(entry->status);

    return page_size != 0 &&
           (entry->logical == (logical_address & ~(page_size - 1)));
}

static void m68k_mmu030_atc_result_from_entry(
    const M68KMMU030ATCEntry *entry, uint32_t logical_address,
    M68KMMU030TranslateResult *result)
{
    uint32_t page_size = m68k_mmu030_atc_page_size(entry->status);
    uint32_t page_mask = ~(page_size - 1);

    memset(result, 0, sizeof(*result));
    result->physical = (entry->physical & page_mask) +
                       (logical_address & ~page_mask);
    result->page_size = page_size;
    result->prot = PAGE_READ;
    result->write_protect =
        (entry->status & M68K_MMU030_ATC_WRITE_PROTECT) != 0;
    result->supervisor_only =
        (entry->status & M68K_MMU030_ATC_SUPERVISOR) != 0;
    result->modified =
        (entry->status & M68K_MMU030_ATC_MODIFIED) != 0;
    result->cache_inhibit =
        (entry->status & M68K_MMU030_ATC_CACHE_INHIBIT) != 0;
    if (!result->write_protect) {
        result->prot |= PAGE_WRITE;
    }
}

bool m68k_mmu030_atc_lookup(M68KMMU030State *state,
                            uint32_t logical_address, int access_type,
                            uint8_t function_code,
                            M68KMMU030TranslateResult *result)
{
    bool is_write = (access_type & M68K_MMU030_ACCESS_STORE) != 0;

    if (!state || !result) {
        return false;
    }

    memset(result, 0, sizeof(*result));
    for (unsigned i = 0; i < M68K_MMU030_ATC_ENTRIES; i++) {
        M68KMMU030ATCEntry *entry = &state->atc[i];
        uint16_t mmusr = 0;

        if (!m68k_mmu030_atc_entry_valid(entry) ||
            m68k_mmu030_atc_entry_fc(entry) != (function_code & 7) ||
            !m68k_mmu030_atc_address_match(entry, logical_address)) {
            continue;
        }

        m68k_mmu030_atc_result_from_entry(entry, logical_address, result);
        if (access_type & M68K_MMU030_ACCESS_CODE) {
            result->prot |= PAGE_EXEC;
        }
        if (entry->status & M68K_MMU030_ATC_BUS_ERROR) {
            result->bus_error = true;
            mmusr = M68K_MMU030_MMUSR_B | M68K_MMU030_MMUSR_I;
        } else if (result->supervisor_only && !(function_code & 4)) {
            mmusr = M68K_MMU030_MMUSR_S;
        } else if (is_write && result->write_protect) {
            mmusr = M68K_MMU030_MMUSR_WP;
        }
        if (mmusr) {
            result->mmusr = mmusr;
            result->fault = true;
        }
        return true;
    }
    return false;
}

void m68k_mmu030_atc_fill(M68KMMU030State *state, uint32_t logical_address,
                          uint8_t function_code,
                          const M68KMMU030TranslateResult *result)
{
    uint32_t page_size;
    uint32_t page_mask;
    unsigned page_bits;
    unsigned slot = M68K_MMU030_ATC_ENTRIES;
    bool replace = true;

    if (!state || !result ||
        (result->fault && !result->atc_error && !result->write_protect) ||
        (result->mmusr & M68K_MMU030_MMUSR_T)) {
        return;
    }

    page_size = result->page_size;
    if (page_size < 256 || page_size > 32768 ||
        (page_size & (page_size - 1)) != 0) {
        return;
    }
    page_bits = 0;
    while ((UINT32_C(1) << page_bits) < page_size) {
        page_bits++;
    }
    if (page_bits < 8 || page_bits > 15) {
        return;
    }
    page_mask = ~(page_size - 1);

    /* A fill for an existing tag overwrites that tag in place. */
    for (unsigned i = 0; i < M68K_MMU030_ATC_ENTRIES; i++) {
        M68KMMU030ATCEntry *entry = &state->atc[i];

        if (m68k_mmu030_atc_entry_valid(entry) &&
            m68k_mmu030_atc_entry_fc(entry) == (function_code & 7) &&
            m68k_mmu030_atc_address_match(entry, logical_address)) {
            /* PLOAD/refill replaces a tag despite an older page size. */
            if (slot == M68K_MMU030_ATC_ENTRIES) {
                slot = i;
            } else {
                entry->status &= ~M68K_MMU030_ATC_VALID;
            }
            replace = false;
        }
    }

    if (replace) {
        /* Prefer an invalid entry, scanning from the deterministic cursor. */
        for (unsigned offset = 0; offset < M68K_MMU030_ATC_ENTRIES;
             offset++) {
            unsigned candidate = (state->atc_next + offset) %
                                  M68K_MMU030_ATC_ENTRIES;

            if (!m68k_mmu030_atc_entry_valid(&state->atc[candidate])) {
                slot = candidate;
                break;
            }
        }
        if (slot == M68K_MMU030_ATC_ENTRIES) {
            slot = state->atc_next % M68K_MMU030_ATC_ENTRIES;
        }
    }

    uint32_t status = M68K_MMU030_ATC_VALID |
                      ((uint32_t)(function_code & 7) <<
                       M68K_MMU030_ATC_FC_SHIFT) |
                      ((uint32_t)page_bits << M68K_MMU030_ATC_PAGE_BITS_SHIFT);

    if (result->atc_error) {
        status |= M68K_MMU030_ATC_BUS_ERROR;
    }
    if (result->write_protect || !(result->prot & PAGE_WRITE)) {
        status |= M68K_MMU030_ATC_WRITE_PROTECT;
    }
    if (result->supervisor_only) {
        status |= M68K_MMU030_ATC_SUPERVISOR;
    }
    if (result->modified) {
        status |= M68K_MMU030_ATC_MODIFIED;
    }
    if (result->cache_inhibit) {
        status |= M68K_MMU030_ATC_CACHE_INHIBIT;
    }

    state->atc[slot].logical = logical_address & page_mask;
    state->atc[slot].physical = result->physical & page_mask;
    state->atc[slot].status = status;
    if (replace) {
        state->atc_next = (slot + 1) % M68K_MMU030_ATC_ENTRIES;
    }
}

int m68k_mmu030_atc_preload(M68KMMU030State *state,
                            const M68KMMU030MemoryOps *ops,
                            uint32_t logical_address, int access_type,
                            uint8_t function_code,
                            M68KMMU030TranslateResult *result)
{
    uint32_t tc;
    uint16_t mmusr;
    int ret;

    if (!state || !result) {
        return -1;
    }

    /* PLOAD performs a table search even when ordinary translation is off. */
    tc = state->tc;
    mmusr = state->mmusr;
    state->tc |= M68K_MMU030_TC_ENABLE;
    ret = m68k_mmu030_walk(state, ops, logical_address, access_type,
                           function_code, false, result);
    state->tc = tc;
    state->mmusr = mmusr;
    if (ret == 0 || result->atc_error || result->write_protect) {
        m68k_mmu030_atc_fill(state, logical_address, function_code, result);
    }
    return ret;
}

int m68k_mmu030_atc_ptest(M68KMMU030State *state,
                          uint32_t logical_address, bool is_write,
                          uint8_t function_code,
                          M68KMMU030TranslateResult *result)
{
    bool matched = false;

    if (!state || !result) {
        return -1;
    }

    memset(result, 0, sizeof(*result));
    for (unsigned i = 0; i < ARRAY_SIZE(state->tt); i++) {
        M68KMMU030TTResult tt = m68k_mmu030_tt_match(
            state->tt[i], logical_address, function_code, is_write);

        if (tt.matched) {
            result->physical = logical_address;
            result->page_size = UINT32_C(1) << M68K_MMU030_DEFAULT_PAGE_BITS;
            result->prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
            result->cache_inhibit = tt.cache_inhibit;
            result->mmusr = M68K_MMU030_MMUSR_T;
            state->mmusr = result->mmusr;
            return 1;
        }
    }

    for (unsigned i = 0; i < M68K_MMU030_ATC_ENTRIES; i++) {
        M68KMMU030ATCEntry *entry = &state->atc[i];

        if (!m68k_mmu030_atc_entry_valid(entry) ||
            m68k_mmu030_atc_entry_fc(entry) != (function_code & 7) ||
            !m68k_mmu030_atc_address_match(entry, logical_address)) {
            continue;
        }

        m68k_mmu030_atc_result_from_entry(entry, logical_address, result);
        if (entry->status & M68K_MMU030_ATC_BUS_ERROR) {
            result->bus_error = true;
            result->mmusr |= M68K_MMU030_MMUSR_B | M68K_MMU030_MMUSR_I;
        } else {
            if (entry->status & M68K_MMU030_ATC_WRITE_PROTECT) {
                result->mmusr |= M68K_MMU030_MMUSR_WP;
            }
            if (entry->status & M68K_MMU030_ATC_MODIFIED) {
                result->mmusr |= M68K_MMU030_MMUSR_M;
            }
        }
        state->mmusr = result->mmusr;
        matched = true;
        break;
    }

    if (!matched) {
        result->page_size = UINT32_C(1) << M68K_MMU030_DEFAULT_PAGE_BITS;
        result->mmusr = M68K_MMU030_MMUSR_I;
        result->fault = true;
        state->mmusr = result->mmusr;
    }
    return 1;
}

void m68k_mmu030_atc_flush_all(M68KMMU030State *state)
{
    if (!state) {
        return;
    }
    for (unsigned i = 0; i < M68K_MMU030_ATC_ENTRIES; i++) {
        state->atc[i].status &= ~M68K_MMU030_ATC_VALID;
    }
}

void m68k_mmu030_atc_flush_fc(M68KMMU030State *state,
                              uint8_t function_code,
                              uint8_t function_code_mask)
{
    if (!state) {
        return;
    }
    for (unsigned i = 0; i < M68K_MMU030_ATC_ENTRIES; i++) {
        M68KMMU030ATCEntry *entry = &state->atc[i];

        if (m68k_mmu030_atc_entry_valid(entry) &&
            m68k_mmu030_atc_fc_match(entry, function_code,
                                     function_code_mask)) {
            entry->status &= ~M68K_MMU030_ATC_VALID;
        }
    }
}

void m68k_mmu030_atc_flush_page(M68KMMU030State *state,
                                uint32_t logical_address,
                                uint8_t function_code,
                                uint8_t function_code_mask)
{
    if (!state) {
        return;
    }
    for (unsigned i = 0; i < M68K_MMU030_ATC_ENTRIES; i++) {
        M68KMMU030ATCEntry *entry = &state->atc[i];

        if (m68k_mmu030_atc_entry_valid(entry) &&
            m68k_mmu030_atc_fc_match(entry, function_code,
                                     function_code_mask) &&
            m68k_mmu030_atc_address_match(entry, logical_address)) {
            entry->status &= ~M68K_MMU030_ATC_VALID;
        }
    }
}

void m68k_mmu030_atc_flush_all_coherent(
    M68KMMU030State *state, M68KMMU030ATCFlushAllFn flush_all, void *opaque)
{
    m68k_mmu030_atc_flush_all(state);
    if (flush_all) {
        flush_all(opaque);
    }
}

void m68k_mmu030_atc_flush_fc_coherent(
    M68KMMU030State *state, uint8_t function_code,
    uint8_t function_code_mask, M68KMMU030ATCFlushAllFn flush_all,
    void *opaque)
{
    m68k_mmu030_atc_flush_fc(state, function_code, function_code_mask);
    /* QEMU's TLB has no FC tag, so an FC scope requires a full flush. */
    if (flush_all) {
        flush_all(opaque);
    }
}

void m68k_mmu030_atc_flush_page_coherent(
    M68KMMU030State *state, uint32_t logical_address,
    uint8_t function_code, uint8_t function_code_mask,
    M68KMMU030ATCFlushRangeFn flush_range, void *opaque)
{
    bool matched = false;

    if (!state) {
        return;
    }

    for (unsigned i = 0; i < M68K_MMU030_ATC_ENTRIES; i++) {
        M68KMMU030ATCEntry *entry = &state->atc[i];
        uint32_t page_size;
        uint32_t page_address;

        if (!m68k_mmu030_atc_entry_valid(entry) ||
            !m68k_mmu030_atc_fc_match(entry, function_code,
                                      function_code_mask) ||
            !m68k_mmu030_atc_address_match(entry, logical_address)) {
            continue;
        }

        matched = true;
        /* A matching entry retains its effective early-termination size. */
        page_size = m68k_mmu030_atc_page_size(entry->status);
        page_address = entry->logical & ~(page_size - 1);
        entry->status &= ~M68K_MMU030_ATC_VALID;
        if (flush_range) {
            flush_range(opaque, page_address, page_size);
        }
    }

    if (!matched && flush_range) {
        uint32_t page_size = m68k_mmu030_atc_active_page_size(state);
        uint32_t page_address = logical_address & ~(page_size - 1);

        flush_range(opaque, page_address, page_size);
    }
}

bool m68k_mmu030_write_tt(M68KMMU030State *state, unsigned index,
                          uint32_t value,
                          M68KMMU030ATCFlushAllFn flush_derived,
                          void *opaque)
{
    if (!state || index >= ARRAY_SIZE(state->tt)) {
        return false;
    }
    if (state->tt[index] == value) {
        return true;
    }

    /* TT has priority over the ATC; invalidate only derived translations. */
    if (flush_derived) {
        flush_derived(opaque);
    }
    state->tt[index] = value;
    return true;
}

bool m68k_mmu030_reconfigure(
    M68KMMU030State *state, const M68KMMU030ControlState *control,
    bool flush, M68KMMU030ATCFlushAllFn flush_all, void *opaque)
{
    bool changed;

    if (!state || !control || !m68k_mmu030_validate_tc(control->tc)) {
        return false;
    }

    changed = state->crp != control->crp || state->srp != control->srp ||
              state->tc != control->tc || state->tt[0] != control->tt[0] ||
              state->tt[1] != control->tt[1];
    if (!changed && !flush) {
        return true;
    }

    /* Publish new translation controls only after invalidating old entries. */
    if (flush) {
        m68k_mmu030_atc_flush_all_coherent(state, flush_all, opaque);
    }
    if (!changed) {
        return true;
    }
    state->crp = control->crp;
    state->srp = control->srp;
    state->tc = control->tc;
    state->tt[0] = control->tt[0];
    state->tt[1] = control->tt[1];
    return true;
}

int m68k_mmu030_translate_state(
    M68KMMU030State *state, const M68KMMU030MemoryOps *ops,
    uint32_t logical_address, int access_type, uint8_t function_code,
    bool probe, M68KMMU030TranslateResult *result)
{
    int ret;

    if (!state || !result) {
        return -1;
    }

    /* The existing 030 helper path is the level-zero PTEST operation. */
    if (access_type & M68K_MMU030_ACCESS_PTEST) {
        ret = m68k_mmu030_atc_ptest(
            state, logical_address,
            (access_type & M68K_MMU030_ACCESS_STORE) != 0,
            function_code, result);
        return ret < 0 ? ret : 0;
    }

    /* Transparent translation has priority over an architectural ATC hit. */
    if (state->tc & M68K_MMU030_TC_ENABLE) {
        bool transparent = false;

        for (unsigned i = 0; i < ARRAY_SIZE(state->tt); i++) {
            if (m68k_mmu030_tt_match(state->tt[i], logical_address,
                                     function_code,
                                     access_type & M68K_MMU030_ACCESS_STORE).
                    matched) {
                transparent = true;
                break;
            }
        }

        if (!transparent &&
            m68k_mmu030_atc_lookup(state, logical_address, access_type,
                                   function_code, result)) {
            /* A clear M bit requires a table search before the first write. */
            if ((access_type & M68K_MMU030_ACCESS_STORE) && !result->fault &&
                !result->write_protect && !result->modified) {
                goto table_walk;
            }
            return result->fault ? -1 : 0;
        }
    }

table_walk:
    ret = m68k_mmu030_walk(state, ops, logical_address, access_type,
                           function_code, probe, result);
    if (!probe && (state->tc & M68K_MMU030_TC_ENABLE)) {
        m68k_mmu030_atc_fill(state, logical_address, function_code, result);
    }
    return ret;
}

bool m68k_mmu030_validate_tc(uint32_t tc)
{
    unsigned page_size;
    unsigned initial_shift;
    unsigned address_bits;

    if ((tc & M68K_MMU030_TC_ENABLE) == 0) {
        return true;
    }

    page_size = (tc >> M68K_MMU030_TC_PS_SHIFT) & 0xf;
    initial_shift = (tc >> M68K_MMU030_TC_IS_SHIFT) & 0xf;
    if (page_size < 8 || page_size > 15) {
        return false;
    }

    address_bits = page_size + initial_shift;
    for (unsigned shift = 12; ; shift -= 4) {
        unsigned index = (tc >> shift) & 0xf;

        if (index == 0) {
            break;
        }
        address_bits += index;

        if (shift == 0) {
            break;
        }
    }

    return address_bits == 32;
}

M68KMMU030TTResult m68k_mmu030_tt_match(uint32_t tt,
                                         uint32_t logical_address,
                                         uint8_t function_code,
                                         bool is_write)
{
    M68KMMU030TTResult result = { 0 };
    uint8_t address_base;
    uint8_t address_mask;
    uint8_t function_code_base;
    uint8_t function_code_mask;

    if ((tt & M68K_MMU030_TT_ENABLE) == 0) {
        return result;
    }

    address_base = tt >> 24;
    address_mask = tt >> 16;
    if ((((logical_address >> 24) ^ address_base) & ~address_mask) != 0) {
        return result;
    }

    function_code_base = (tt >> 4) & 7;
    function_code_mask = tt & 7;
    if ((((function_code & 7) ^ function_code_base) &
         ~function_code_mask) != 0) {
        return result;
    }

    if ((tt & M68K_MMU030_TT_RW_MASK) == 0 &&
        (is_write == ((tt & M68K_MMU030_TT_RW) != 0))) {
        return result;
    }

    result.matched = true;
    result.cache_inhibit = (tt & M68K_MMU030_TT_CACHE_INHIBIT) != 0;
    return result;
}

void m68k_mmu030_reset(M68KMMU030State *state)
{
    memset(state, 0, sizeof(*state));
}

static void m68k_mmu030_save_fault_context(
    M68KMMU030FaultContext *context, const M68KMMU030State *state,
    uint32_t frame_start, uint32_t frame_size)
{
    context->frame_start = frame_start;
    context->frame_end = frame_start + frame_size;
    context->fault_pending = state->fault_pending;
    context->fault_address = state->fault_address;
    context->fault_pc = state->fault_pc;
    context->fault_ssw = state->fault_ssw;
    context->fault_status = state->fault_status;
    context->fault_format = state->fault_format;
    context->fault_size = state->fault_size;
    context->fault_function_code = state->fault_function_code;
    context->fault_table_level = state->fault_table_level;
    context->fault_stage_c = state->fault_stage_c;
    context->fault_stage_b = state->fault_stage_b;
    context->fault_stage_b_address = state->fault_stage_b_address;
    context->fault_data_output = state->fault_data_output;
    context->fault_data_input = state->fault_data_input;
    context->fault_data_input_address = state->fault_data_input_address;
    context->fault_descriptor_address = state->fault_descriptor_address;
    context->fault_instruction_address = state->fault_instruction_address;
    context->fault_resume_pc = state->fault_resume_pc;
    context->fault_data_complete = state->fault_data_complete;
    context->fault_data_input_valid = state->fault_data_input_valid;
    context->fault_data_write = state->fault_data_write;
    context->fault_frame_version = state->fault_frame_version;
    context->restart_pending = state->restart_pending;
    context->fault_rmw = state->fault_rmw;
    context->fault_fetch_active = state->fault_fetch_active;
    context->fault_code_fetch = state->fault_code_fetch;
    context->fault_pipe_accept = state->fault_pipe_accept;
    context->fault_rmw_phase = state->fault_rmw_phase;
    context->fault_rmw_data1 = state->fault_rmw_data1;
    context->fault_rmw_data2 = state->fault_rmw_data2;
    context->fault_rmw_data_valid = state->fault_rmw_data_valid;
    context->fault_special_kind = state->fault_special_kind;
    context->fault_special_phase = state->fault_special_phase;
    context->fault_special_valid = state->fault_special_valid;
    context->fault_special_pc = state->fault_special_pc;
    memcpy(context->fault_special_data, state->fault_special_data,
           sizeof(context->fault_special_data));
}

static void m68k_mmu030_restore_fault_context(
    M68KMMU030State *state, const M68KMMU030FaultContext *context)
{
    state->fault_pending = context->fault_pending;
    state->fault_address = context->fault_address;
    state->fault_pc = context->fault_pc;
    state->fault_ssw = context->fault_ssw;
    state->fault_status = context->fault_status;
    state->fault_format = context->fault_format;
    state->fault_size = context->fault_size;
    state->fault_function_code = context->fault_function_code;
    state->fault_table_level = context->fault_table_level;
    state->fault_stage_c = context->fault_stage_c;
    state->fault_stage_b = context->fault_stage_b;
    state->fault_stage_b_address = context->fault_stage_b_address;
    state->fault_data_output = context->fault_data_output;
    state->fault_data_input = context->fault_data_input;
    state->fault_data_input_address = context->fault_data_input_address;
    state->fault_descriptor_address = context->fault_descriptor_address;
    state->fault_instruction_address = context->fault_instruction_address;
    state->fault_resume_pc = context->fault_resume_pc;
    state->fault_data_complete = context->fault_data_complete;
    state->fault_data_input_valid = context->fault_data_input_valid;
    state->fault_data_write = context->fault_data_write;
    state->fault_frame_version = context->fault_frame_version;
    state->restart_pending = context->restart_pending;
    state->fault_rmw = context->fault_rmw;
    state->fault_fetch_active = context->fault_fetch_active;
    state->fault_code_fetch = context->fault_code_fetch;
    state->fault_pipe_accept = context->fault_pipe_accept;
    state->fault_rmw_phase = context->fault_rmw_phase;
    state->fault_rmw_data1 = context->fault_rmw_data1;
    state->fault_rmw_data2 = context->fault_rmw_data2;
    state->fault_rmw_data_valid = context->fault_rmw_data_valid;
    state->fault_special_kind = context->fault_special_kind;
    state->fault_special_phase = context->fault_special_phase;
    state->fault_special_valid = context->fault_special_valid;
    state->fault_special_pc = context->fault_special_pc;
    memcpy(state->fault_special_data, context->fault_special_data,
           sizeof(state->fault_special_data));
}

static void m68k_mmu030_clear_fault_scratch(M68KMMU030State *state)
{
    memset((uint8_t *)state + offsetof(M68KMMU030State, fault_pending), 0,
           offsetof(M68KMMU030State, fault_frames) -
           offsetof(M68KMMU030State, fault_pending));
}

static bool m68k_mmu030_fault_frames_active(
    const M68KMMU030State *state)
{
    return state && (state->fault_frame_depth != 0 ||
                     state->fault_legacy_frame_active);
}

bool m68k_mmu030_legacy_fault_frame_active(
    const M68KMMU030State *state)
{
    return state && state->fault_legacy_frame_active;
}

void m68k_mmu030_clear_legacy_fault_frame(M68KMMU030State *state)
{
    if (!state) {
        return;
    }
    state->fault_legacy_frame_active = false;
    state->fault_frame_active = m68k_mmu030_fault_frames_active(state);
}

void m68k_mmu030_reset_fault_scratch(M68KMMU030State *state)
{
    bool exception_processing;
    uint8_t prefetch_words;

    if (!state) {
        return;
    }
    exception_processing = state->fault_exception_processing;
    prefetch_words = state->fault_exception_prefetch_words;
    m68k_mmu030_clear_fault_scratch(state);
    state->fault_frame_active = m68k_mmu030_fault_frames_active(state);
    state->fault_exception_processing = exception_processing;
    state->fault_exception_prefetch_words = prefetch_words;
}

M68KMMU030FaultFramePushResult m68k_mmu030_push_fault_frame(
    M68KMMU030State *state, uint32_t frame_start, uint32_t frame_size)
{
    uint32_t frame_end;

    if (!state || !frame_size || state->fault_legacy_frame_active ||
        state->fault_frame_depth > M68K_MMU030_MAX_FAULT_FRAMES ||
        frame_start + frame_size < frame_start) {
        return M68K_MMU030_FAULT_FRAME_PUSH_CONFLICT;
    }

    frame_end = frame_start + frame_size;
    for (unsigned i = 0; i < state->fault_frame_depth; i++) {
        const M68KMMU030FaultContext *context = &state->fault_frames[i];

        /* Exact ranges must identify one live frame. */
        if (context->frame_start == frame_start &&
            context->frame_end == frame_end) {
            return M68K_MMU030_FAULT_FRAME_PUSH_CONFLICT;
        }
        /* A format-0 RTE names a frame by its trailing eight-byte base
         * frame.  Do not admit two live contexts with the same collapse
         * address, even when their full ranges differ. */
        if (frame_size >= 8 && context->frame_end >= context->frame_start &&
            context->frame_end - context->frame_start >= 8 &&
            context->frame_end == frame_end) {
            return M68K_MMU030_FAULT_FRAME_PUSH_CONFLICT;
        }
    }

    if (state->fault_frame_depth >= M68K_MMU030_MAX_FAULT_FRAMES) {
        return M68K_MMU030_FAULT_FRAME_PUSH_CAPACITY;
    }

    m68k_mmu030_save_fault_context(
        &state->fault_frames[state->fault_frame_depth], state,
        frame_start, frame_size);
    state->fault_frame_depth++;
    state->fault_frame_active = m68k_mmu030_fault_frames_active(state);
    return M68K_MMU030_FAULT_FRAME_PUSHED;
}

static bool m68k_mmu030_find_fault_frame(
    const M68KMMU030State *state, uint32_t frame_start, uint32_t frame_size,
    unsigned *index)
{
    uint32_t frame_end;
    unsigned found = 0;

    if (!state || !state->fault_frame_active || !frame_size ||
        state->fault_frame_depth > M68K_MMU030_MAX_FAULT_FRAMES ||
        frame_start + frame_size < frame_start) {
        return false;
    }

    frame_end = frame_start + frame_size;
    for (unsigned i = 0; i < state->fault_frame_depth; i++) {
        const M68KMMU030FaultContext *context = &state->fault_frames[i];

        if (context->frame_start == frame_start &&
            context->frame_end == frame_end) {
            if (found++) {
                /* A malformed/migrated duplicate is ambiguous. */
                return false;
            }
            if (index) {
                *index = i;
            }
        }
    }
    return found == 1;
}

bool m68k_mmu030_fault_frame_matches(const M68KMMU030State *state,
                                     uint32_t frame_start,
                                     uint32_t frame_size)
{
    return m68k_mmu030_find_fault_frame(state, frame_start, frame_size,
                                        NULL);
}

bool m68k_mmu030_fault_frame_collapsed_matches(
    const M68KMMU030State *state, uint32_t frame_start)
{
    unsigned found = 0;

    if (!state || !state->fault_frame_active ||
        (state->fault_frame_depth == 0 &&
         !state->fault_legacy_frame_active) ||
        state->fault_frame_depth > M68K_MMU030_MAX_FAULT_FRAMES) {
        return false;
    }

    for (unsigned i = 0; i < state->fault_frame_depth; i++) {
        const M68KMMU030FaultContext *context = &state->fault_frames[i];

        /* Mach's format-0 collapse points RTE at the final eight-byte base
         * frame, not at the original access-frame start. */
        if (context->frame_end >= context->frame_start &&
            context->frame_end - context->frame_start >= 8 &&
            frame_start == context->frame_end - 8) {
            found++;
        }
    }
    return found == 1;
}

bool m68k_mmu030_restore_fault_frame_context(M68KMMU030State *state,
                                             uint32_t frame_start,
                                             uint32_t frame_size)
{
    unsigned index;

    if (!m68k_mmu030_find_fault_frame(state, frame_start, frame_size,
                                      &index)) {
        return false;
    }

    m68k_mmu030_restore_fault_context(
        state, &state->fault_frames[index]);
    state->fault_frame_active = m68k_mmu030_fault_frames_active(state);
    return true;
}

void m68k_mmu030_discard_fault_frames(M68KMMU030State *state)
{
    if (!state) {
        return;
    }

    m68k_mmu030_clear_fault_scratch(state);
    memset(state->fault_frames, 0, sizeof(state->fault_frames));
    state->fault_frame_depth = 0;
    state->fault_legacy_frame_active = false;
    state->fault_frame_active = false;
}

static bool m68k_mmu030_pop_fault_frame_internal(M68KMMU030State *state,
                                                  unsigned index, bool discard)
{
    unsigned last = state->fault_frame_depth - 1;

    if (index < last) {
        memmove(&state->fault_frames[index], &state->fault_frames[index + 1],
                (last - index) * sizeof(state->fault_frames[0]));
    }
    memset(&state->fault_frames[last], 0, sizeof(state->fault_frames[0]));
    state->fault_frame_depth--;
    state->fault_exception_processing = false;
    state->fault_exception_prefetch_words = 0;
    state->fault_frame_active = m68k_mmu030_fault_frames_active(state);

    if (!state->fault_frame_depth) {
        if (discard) {
            m68k_mmu030_discard_fault_frames(state);
        }
    } else if (discard) {
        /* The repaired inner instruction remains the current scratch
         * context; the outer snapshot is restored when its own RTE reads
         * the matching frame. */
        m68k_mmu030_reset_fault_scratch(state);
        state->fault_frame_active = m68k_mmu030_fault_frames_active(state);
    }
    return true;
}

bool m68k_mmu030_pop_fault_frame(M68KMMU030State *state,
                                 uint32_t frame_start,
                                 uint32_t frame_size,
                                 bool discard)
{
    unsigned index;

    if (!m68k_mmu030_find_fault_frame(state, frame_start, frame_size,
                                      &index)) {
        return false;
    }
    return m68k_mmu030_pop_fault_frame_internal(state, index, discard);
}

bool m68k_mmu030_pop_collapsed_fault_frame(M68KMMU030State *state,
                                           uint32_t frame_start,
                                           bool discard)
{
    unsigned index = 0;
    unsigned found = 0;

    if (!state || !state->fault_frame_active ||
        (state->fault_frame_depth == 0 &&
         !state->fault_legacy_frame_active) ||
        state->fault_frame_depth > M68K_MMU030_MAX_FAULT_FRAMES) {
        return false;
    }

    for (unsigned i = 0; i < state->fault_frame_depth; i++) {
        const M68KMMU030FaultContext *context = &state->fault_frames[i];

        if (context->frame_end >= context->frame_start &&
            context->frame_end - context->frame_start >= 8 &&
            frame_start == context->frame_end - 8) {
            index = i;
            found++;
        }
    }
    if (found == 1) {
        return m68k_mmu030_pop_fault_frame_internal(state, index, discard);
    }

    /* Pre-v7 streams have no frame identity.  A format-0 RTE is the only
     * architectural completion available for that scalar continuation, so
     * consume it when no compact identity can be matched. */
    if (!found && state->fault_legacy_frame_active &&
        !state->fault_frame_depth) {
        m68k_mmu030_clear_legacy_fault_frame(state);
        if (discard) {
            m68k_mmu030_reset_fault_scratch(state);
        }
        return true;
    }
    return false;
}

static bool m68k_mmu030_special_matches(const M68KMMU030State *state,
                                        unsigned kind, uint32_t pc)
{
    return state && state->fault_special_valid &&
           state->fault_special_kind == kind && state->fault_special_pc == pc;
}

void m68k_mmu030_special_clear(M68KMMU030State *state)
{
    if (!state) {
        return;
    }
    state->fault_special_kind = M68K_MMU030_SPECIAL_NONE;
    state->fault_special_phase = 0;
    state->fault_special_valid = false;
    state->fault_special_pc = 0;
    memset(state->fault_special_data, 0, sizeof(state->fault_special_data));
}

bool m68k_mmu030_special_cycle(M68KMMU030State *state, unsigned kind,
                               unsigned cycle, uint32_t pc,
                               uint32_t data, bool is_write)
{
    if (!state || cycle >= M68K_MMU030_SPECIAL_MAX_CYCLES ||
        kind == M68K_MMU030_SPECIAL_NONE) {
        return false;
    }

    if (!m68k_mmu030_special_matches(state, kind, pc)) {
        m68k_mmu030_special_clear(state);
        state->fault_special_kind = kind;
        state->fault_special_pc = pc;
        state->fault_special_valid = true;
    }

    if (state->fault_special_phase > cycle) {
        return true;
    }

    /* The ordinary access-frame builder consumes these values if this bus
     * cycle faults.  Keep the output right-justified, as for gen_store(). */
    state->fault_data_output = is_write ? data : 0;
    state->fault_data_input_valid = false;
    return false;
}

void m68k_mmu030_special_record(M68KMMU030State *state, unsigned kind,
                                unsigned cycle, uint32_t pc, uint32_t data,
                                bool is_load)
{
    if (!state || cycle >= M68K_MMU030_SPECIAL_MAX_CYCLES ||
        !m68k_mmu030_special_matches(state, kind, pc)) {
        return;
    }

    if (is_load) {
        state->fault_special_data[cycle] = data;
    }
    if (state->fault_special_phase <= cycle) {
        state->fault_special_phase = cycle + 1;
    }
}

void m68k_mmu030_special_finish(M68KMMU030State *state, unsigned kind,
                                unsigned cycles, uint32_t pc)
{
    if (m68k_mmu030_special_matches(state, kind, pc) &&
        state->fault_special_phase >= cycles) {
        m68k_mmu030_special_clear(state);
    }
}

void m68k_mmu030_special_complete(M68KMMU030State *state, unsigned kind,
                                  uint32_t pc)
{
    if (m68k_mmu030_special_matches(state, kind, pc)) {
        unsigned failed_cycle = state->fault_special_phase;

        /*
         * For a read cycle, DF-clear means the handler supplied the failed
         * bus word in DIB.  Install it in the failed fragment/word slot
         * before advancing past the failed cycle.
         */
        if (state->fault_data_input_valid &&
            failed_cycle < M68K_MMU030_SPECIAL_MAX_CYCLES) {
            state->fault_special_data[failed_cycle] =
                state->fault_data_input;
        }
        /*
         * A handler which clears DF has completed the failed cycle in
         * software.  Leave later cycles pending so the retried instruction
         * can issue them on the repaired mapping.
         */
        if (failed_cycle < M68K_MMU030_SPECIAL_MAX_CYCLES) {
            state->fault_special_phase = failed_cycle + 1;
        }
    }
}

static bool m68k_mmu030_fmovem_prepare(M68KMMU030State *state,
                                       uint32_t pc)
{
    if (!state) {
        return false;
    }

    if (!m68k_mmu030_special_matches(state, M68K_MMU030_SPECIAL_FMOVEM,
                                     pc)) {
        m68k_mmu030_special_clear(state);
        state->fault_special_kind = M68K_MMU030_SPECIAL_FMOVEM;
        state->fault_special_pc = pc;
        state->fault_special_valid = true;
    }
    return true;
}

bool m68k_mmu030_fmovem_cycle(M68KMMU030State *state, unsigned reg,
                              unsigned cycle, uint32_t pc, uint32_t data,
                              bool is_write)
{
    if (cycle >= 3 || reg >= 8 || !m68k_mmu030_fmovem_prepare(state, pc)) {
        return false;
    }

    /* Word three is the number of complete operands.  Older operands have
     * already committed all of their device cycles and must not be touched
     * again when the instruction is retried. */
    if (state->fault_special_data[3] > reg ||
        (state->fault_special_data[3] == reg &&
         state->fault_special_phase > cycle)) {
        return true;
    }

    state->fault_data_output = is_write ? data : 0;
    state->fault_data_input_valid = false;
    return false;
}

void m68k_mmu030_fmovem_record(M68KMMU030State *state, unsigned reg,
                               unsigned cycle, uint32_t pc, uint32_t data,
                               bool is_load)
{
    if (cycle >= 3 || reg >= 8 ||
        !m68k_mmu030_fmovem_prepare(state, pc) ||
        state->fault_special_data[3] != reg) {
        return;
    }

    if (is_load) {
        state->fault_special_data[cycle] = data;
    }
    if (state->fault_special_phase <= cycle) {
        state->fault_special_phase = cycle + 1;
    }
}

void m68k_mmu030_fmovem_finish_register(M68KMMU030State *state,
                                        unsigned reg, unsigned cycles,
                                        uint32_t pc)
{
    if (reg >= 8 || cycles > 3 ||
        !m68k_mmu030_special_matches(state, M68K_MMU030_SPECIAL_FMOVEM,
                                     pc) ||
        state->fault_special_data[3] != reg ||
        state->fault_special_phase < cycles) {
        return;
    }

    state->fault_special_data[3] = reg + 1;
    state->fault_special_phase = 0;
    memset(state->fault_special_data, 0,
           3 * sizeof(state->fault_special_data[0]));
}

void m68k_mmu030_fmovem_finish(M68KMMU030State *state, unsigned regs,
                               uint32_t pc)
{
    if (regs <= 8 && m68k_mmu030_special_matches(
            state, M68K_MMU030_SPECIAL_FMOVEM, pc) &&
        state->fault_special_data[3] >= regs) {
        m68k_mmu030_special_clear(state);
    }
}

void m68k_mmu030_begin_instruction_fetch(M68KMMU030State *state,
                                         uint32_t instruction_pc)
{
    bool accepted;

    if (!state) {
        return;
    }

    accepted = state->fault_pipe_accept && state->fault_code_fetch &&
               state->fault_pc == instruction_pc;
    state->fault_fetch_active = true;
    state->fault_code_fetch = true;
    state->fault_instruction_address = instruction_pc;
    if (!accepted) {
        state->fault_stage_c = 0;
        state->fault_stage_b = 0;
        state->fault_stage_b_address = instruction_pc + 4;
    } else if (!state->fault_stage_b_address) {
        state->fault_stage_b_address = instruction_pc + 4;
    }
}

void m68k_mmu030_record_instruction_fetch(M68KMMU030State *state,
                                          uint32_t instruction_pc,
                                          uint32_t fetch_address,
                                          uint16_t word)
{
    uint32_t offset;

    if (!state || !state->fault_fetch_active ||
        state->fault_instruction_address != instruction_pc) {
        return;
    }

    offset = fetch_address - instruction_pc;
    if (offset == 2) {
        state->fault_stage_c = word;
    } else if (offset == 4) {
        state->fault_stage_b = word;
        state->fault_stage_b_address = fetch_address;
    }
    if (state->fault_exception_processing &&
        state->fault_exception_prefetch_words < 3) {
        state->fault_exception_prefetch_words++;
        if (state->fault_exception_prefetch_words == 3) {
            state->fault_exception_processing = false;
        }
    }
}

void m68k_mmu030_end_instruction_fetch(M68KMMU030State *state)
{
    if (!state) {
        return;
    }

    state->fault_fetch_active = false;
    if (!state->fault_pending) {
        state->fault_instruction_address = 0;
        state->fault_stage_c = 0;
        state->fault_stage_b = 0;
        state->fault_stage_b_address = 0;
        state->fault_code_fetch = false;
        state->fault_pipe_accept = false;
    }
}

uint16_t m68k_mmu030_make_ssw(unsigned size, bool is_write, bool is_code,
                              uint8_t function_code)
{
    unsigned size_code;

    /*
     * The generic helper predates the pipeline-aware fault latch and is
     * retained for callers which only have the access tuple.  Instruction
     * faults are completed by m68k_mmu030_capture_fault(), which adds the
     * appropriate FC/FB/RC/RB bits once the pipe stage is known.
     */
    if (is_code) {
        return 0;
    }

    switch (size) {
    case 1:
        size_code = 1;
        break;
    case 2:
        size_code = 2;
        break;
    case 3:
        size_code = 3;
        break;
    case 4:
        size_code = 0;
        break;
    default:
        /*
         * QEMU reports larger accesses as multiple bus cycles; the SSW
         * describes each long-word cycle.
         */
        size_code = 0;
        break;
    }

    return M68K_MMU030_SSW_DF |
           (is_write ? 0 : M68K_MMU030_SSW_RW) |
           (size_code << M68K_MMU030_SSW_SIZE_SHIFT) |
           (function_code & M68K_MMU030_SSW_FC_MASK);
}

void m68k_mmu030_capture_fault(
    M68KMMU030State *state, uint32_t logical_address, uint32_t fault_pc,
    unsigned size, bool is_write, bool is_code, uint8_t function_code,
    const M68KMMU030TranslateResult *result)
{
    uint32_t offset;
    uint16_t stage_c;
    uint16_t stage_b;
    uint32_t stage_b_address;
    uint32_t data_output;
    uint32_t data_input;
    uint32_t resume_pc;
    bool data_input_valid;
    uint32_t data_input_address;
    bool pipeline_valid;
    bool rmw;

    /* A fault while constructing, reading, or vectoring an access frame is a
     * double bus fault.  Once exception entry has completed, a handler may
     * take another access fault; its frame is stacked below the live outer
     * frame and the outer frame remains authoritative for its restart state. */
    if (!state || state->fault_exception_processing ||
        state->fault_legacy_frame_active) {
        return;
    }

    state->fault_pending = true;
    state->fault_address = logical_address;
    state->fault_pc = fault_pc;
    state->fault_size = size > UINT8_MAX ? UINT8_MAX : size;
    state->fault_function_code = function_code & 7;
    state->fault_table_level = result ?
                               result->mmusr & M68K_MMU030_MMUSR_N_MASK : 0;
    state->fault_status = result ? result->mmusr : 0;
    state->fault_descriptor_address = result ? result->descriptor_address : 0;
    /* The translator keeps the most recently decoded pipe image in these
     * fields.  Preserve it in the frame when it belongs to this instruction;
     * a prefetch fault can arrive before the new instruction emits its image,
     * in which case stale words from the preceding instruction are invalid. */
    /*
     * RMC helpers may fault after TCG has restored the TB boundary rather
     * than the exact CAS/CAS2 instruction PC.  Their explicit RMW latch
     * still carries the correct restart boundary and data image.
     */
    pipeline_valid = state->fault_instruction_address == fault_pc ||
                     ((state->fault_rmw || state->fault_special_valid) &&
                      state->fault_resume_pc != 0);
    stage_c = pipeline_valid ? state->fault_stage_c : 0;
    stage_b = pipeline_valid ? state->fault_stage_b : 0;
    stage_b_address = pipeline_valid ? state->fault_stage_b_address : 0;
    data_output = state->fault_data_output;
    data_input = state->fault_data_input;
    data_input_address = state->fault_data_input_address;
    rmw = state->fault_rmw;
    data_input_valid = !is_code && pipeline_valid &&
                       state->fault_data_input_valid;
    /* An explicit TAS read is latched before its write cycle.  If that
     * write faults, expose the completed byte as DIB so a handler which
     * clears DF can preserve the original condition-code input. */
    if (!data_input_valid && !is_code && rmw && size == 1 &&
        state->fault_rmw_data_valid && state->fault_rmw_phase >= 1) {
        data_input = state->fault_rmw_data1 & UINT32_C(0xff);
        data_input_address = logical_address;
        data_input_valid = true;
    }
    /* Only data cycles can be completed by accepting the stacked buffers;
     * instruction-pipe faults always resume through their PC/rerun bits. */
    resume_pc = !is_code && pipeline_valid ? state->fault_resume_pc : 0;
    state->fault_stage_c = 0;
    state->fault_stage_b = 0;
    state->fault_stage_b_address = stage_b_address ? stage_b_address :
                                   fault_pc + 4;
    state->fault_data_output = 0;
    state->fault_data_input = 0;
    state->fault_data_input_address = 0;
    state->fault_data_input_valid = false;
    state->fault_data_complete = false;
    state->fault_data_write = is_write;
    state->fault_rmw = false;
    state->fault_instruction_address = fault_pc;
    state->fault_resume_pc = resume_pc;
    state->fault_frame_active = m68k_mmu030_fault_frames_active(state);
    state->fault_fetch_active = false;
    state->fault_code_fetch = is_code;
    state->fault_pipe_accept = false;
    state->fault_frame_version = M68K_MMU030_FRAME_VERSION;
    state->restart_pending = false;

    state->fault_ssw = m68k_mmu030_make_ssw(
        size, is_write, is_code, function_code);
    if (rmw) {
        state->fault_ssw |= M68K_MMU030_SSW_RM;
    }
    if (is_code) {
        /*
         * The pipe words are addressed as C=PC+2 and B=PC+4.  A TLB fill
         * normally reports the first word (C), while an already executing
         * translation can report the next prefetched word (B).  Keep the
         * distinction in the SSW so the exception handler can repair only
         * the invalid stage.
         */
        offset = logical_address - fault_pc;
        if (offset == 4) {
            state->fault_ssw |= M68K_MMU030_SSW_FB |
                                M68K_MMU030_SSW_RB;
            /* Stage B belongs to an instruction already in flight. */
            state->fault_format = M68K_MMU030_FAULT_FORMAT_B;
        } else {
            state->fault_ssw |= M68K_MMU030_SSW_FC |
                                M68K_MMU030_SSW_RC;
            /* A stage-C fetch at the boundary is representable in format A. */
            state->fault_format = M68K_MMU030_FAULT_FORMAT_A;
        }
        state->fault_stage_c = stage_c;
        state->fault_stage_b = stage_b;
    } else {
        /* Data faults occur while an instruction is active. */
        state->fault_format = M68K_MMU030_FAULT_FORMAT_B;
        state->fault_stage_c = stage_c;
        state->fault_stage_b = stage_b;
        if (data_input_valid) {
            state->fault_data_input = data_input;
            state->fault_data_input_address = data_input_address;
            state->fault_data_input_valid = true;
        }
        if (is_write) {
            switch (size) {
            case 1:
                data_output &= UINT32_C(0xff);
                break;
            case 2:
                data_output &= UINT32_C(0xffff);
                break;
            case 3:
                data_output &= UINT32_C(0xffffff);
                break;
            default:
                break;
            }
            state->fault_data_output = data_output;
        }
    }
}

uint32_t m68k_mmu030_rte_frame_tail_size(uint16_t format)
{
    switch (format >> 12) {
    case M68K_MMU030_FAULT_FORMAT_9:
        /* Format 9 is the 10-word coprocessor mid-instruction frame. */
        return M68K_MMU030_COPROCESSOR_FRAME_SIZE - 8;
    case M68K_MMU030_FAULT_FORMAT_A:
        /* RTE has already consumed the status, PC, and format words. */
        return M68K_MMU030_ACCESS_FRAME_SIZE_SHORT - 8;
    case M68K_MMU030_FAULT_FORMAT_B:
        return M68K_MMU030_ACCESS_FRAME_SIZE_LONG - 8;
    default:
        return 0;
    }
}

static uint8_t m68k_mmu030_fault_format(const M68KMMU030State *state)
{
    uint16_t ssw = state->fault_ssw;

    if (state->fault_format == M68K_MMU030_FAULT_FORMAT_A ||
        state->fault_format == M68K_MMU030_FAULT_FORMAT_B ||
        state->fault_format == M68K_MMU030_FAULT_FORMAT_9) {
        return state->fault_format;
    }

    /*
     * A read fault needs the long frame's data input buffer.  Pipeline
     * faults likewise need the stage-B address and the long restart state.
     * Writes at an instruction boundary can use the compact format A frame.
     */
    if ((ssw & (M68K_MMU030_SSW_FC | M68K_MMU030_SSW_FB |
                M68K_MMU030_SSW_RC | M68K_MMU030_SSW_RB)) ||
        ((ssw & (M68K_MMU030_SSW_DF | M68K_MMU030_SSW_RM)) &&
         (ssw & M68K_MMU030_SSW_RW))) {
        return M68K_MMU030_FAULT_FORMAT_B;
    }
    return M68K_MMU030_FAULT_FORMAT_A;
}

uint32_t m68k_mmu030_access_frame_size(const M68KMMU030State *state)
{
    uint8_t format;

    if (!state) {
        return 0;
    }
    format = m68k_mmu030_fault_format(state);
    switch (format) {
    case M68K_MMU030_FAULT_FORMAT_A:
        return M68K_MMU030_ACCESS_FRAME_SIZE_SHORT;
    case M68K_MMU030_FAULT_FORMAT_B:
        return M68K_MMU030_ACCESS_FRAME_SIZE_LONG;
    default:
        /* Format 9 is intentionally not built by the ordinary fault path. */
        return 0;
    }
}

bool m68k_mmu030_build_access_frame(
    M68KMMU030State *state, uint16_t saved_sr, uint16_t vector_offset,
    uint8_t *frame, size_t frame_size)
{
    uint8_t format;
    uint32_t frame_length;
    uint16_t version;

    if (!state || !state->fault_pending || !frame) {
        return false;
    }

    format = m68k_mmu030_fault_format(state);
    frame_length = m68k_mmu030_access_frame_size(state);
    if (!frame_length) {
        return false;
    }
    if (frame_size != frame_length) {
        return false;
    }

    memset(frame, 0, frame_length);
    stw_be_p(frame + 0x00, saved_sr);
    stl_be_p(frame + 0x02, state->fault_pc);
    stw_be_p(frame + 0x06,
             ((uint16_t)format << 12) | (vector_offset & 0x0fff));
    stw_be_p(frame + 0x0a, state->fault_ssw);
    stw_be_p(frame + 0x0c, state->fault_stage_c);
    stw_be_p(frame + 0x0e, state->fault_stage_b);
    stl_be_p(frame + 0x10, state->fault_address);
    stl_be_p(frame + 0x18, state->fault_data_output);
    if (format == M68K_MMU030_FAULT_FORMAT_B) {
        uint32_t stage_b_address = state->fault_stage_b_address;

        if (!stage_b_address &&
            (state->fault_ssw & M68K_MMU030_SSW_RB)) {
            stage_b_address = state->fault_pc + 4;
        }
        stl_be_p(frame + 0x24, stage_b_address);
        stl_be_p(frame + 0x2c, state->fault_data_input);
        version = state->fault_frame_version ?
                  state->fault_frame_version : M68K_MMU030_FRAME_VERSION;
        stw_be_p(frame + 0x36, (version & 0xf) << 12);
    }

    /* The frame has consumed the pending MMU fault context. */
    state->fault_pending = false;
    state->fault_format = format;
    state->fault_frame_active = m68k_mmu030_fault_frames_active(state);
    return true;
}

bool m68k_mmu030_restore_access_frame(
    M68KMMU030State *state, const uint8_t *frame, size_t frame_size,
    uint16_t *saved_sr, uint32_t *resume_pc)
{
    uint16_t format_word;
    uint16_t ssw;
    uint8_t format;
    uint32_t expected_size;
    uint32_t restored_pc;
    bool data_input_valid;
    uint32_t data_input_address;
    bool code_fetch;
    bool special_valid;

    if (!state || !frame || frame_size < M68K_MMU030_ACCESS_FRAME_SIZE_SHORT) {
        return false;
    }

    format_word = lduw_be_p(frame + 0x06);
    format = format_word >> 12;
    switch (format) {
    case M68K_MMU030_FAULT_FORMAT_A:
        expected_size = M68K_MMU030_ACCESS_FRAME_SIZE_SHORT;
        break;
    case M68K_MMU030_FAULT_FORMAT_B:
        expected_size = M68K_MMU030_ACCESS_FRAME_SIZE_LONG;
        break;
    default:
        return false;
    }
    if (frame_size != expected_size) {
        return false;
    }
    if (format == M68K_MMU030_FAULT_FORMAT_B &&
        (lduw_be_p(frame + 0x36) >> 12) != M68K_MMU030_FRAME_VERSION) {
        return false;
    }

    ssw = lduw_be_p(frame + 0x0a);
    if (saved_sr) {
        *saved_sr = lduw_be_p(frame + 0x00);
    }
    if (resume_pc) {
        *resume_pc = ldl_be_p(frame + 0x02);
    }
    restored_pc = ldl_be_p(frame + 0x02);

    /* A completed read may precede a non-RMC write in the same translated
     * instruction.  Preserve that internal validity bit across RTE; the
     * architectural SSW RM bit is reserved for indivisible RMC operations
     * (CAS/CAS2/TAS), not every ordinary read-then-write instruction. */
    data_input_valid = state->fault_data_input_valid;
    data_input_address = state->fault_data_input_address;
    code_fetch = state->fault_code_fetch;
    /* The special-cycle key is the translated instruction's PC.  QEMU may
     * restore a faulting helper to the preceding TCG instruction boundary,
     * so it need not equal the architectural frame PC. */
    special_valid = state->fault_special_valid;

    state->fault_pending = false;
    state->fault_format = format;
    state->fault_pc = restored_pc;
    state->fault_instruction_address = state->fault_pc;
    state->fault_ssw = ssw;
    state->fault_size = 0;
    switch (ssw & M68K_MMU030_SSW_SIZE_MASK) {
    case M68K_MMU030_SSW_SIZE_BYTE:
        state->fault_size = 1;
        break;
    case M68K_MMU030_SSW_SIZE_WORD:
        state->fault_size = 2;
        break;
    case M68K_MMU030_SSW_SIZE_LONG:
        state->fault_size = 4;
        break;
    default:
        state->fault_size = 3;
        break;
    }
    state->fault_function_code = ssw & M68K_MMU030_SSW_FC_MASK;
    state->fault_stage_c = lduw_be_p(frame + 0x0c);
    state->fault_stage_b = lduw_be_p(frame + 0x0e);
    state->fault_address = ldl_be_p(frame + 0x10);
    state->fault_data_output = ldl_be_p(frame + 0x18);
    state->fault_descriptor_address = 0;
    state->fault_table_level = 0;
    state->fault_data_input = 0;
    state->fault_data_input_address = 0;
    state->fault_stage_b_address = 0;
    state->fault_data_input_valid = false;
    state->fault_data_complete = false;
    state->fault_data_write = false;
    state->fault_frame_version = 0;
    state->fault_fetch_active = false;
    state->fault_code_fetch = code_fetch;
    state->fault_pipe_accept = false;
    if (format == M68K_MMU030_FAULT_FORMAT_B) {
        state->fault_stage_b_address = ldl_be_p(frame + 0x24);
        state->fault_data_input = ldl_be_p(frame + 0x2c);
        state->fault_frame_version = lduw_be_p(frame + 0x36) >> 12;
        if (ssw & M68K_MMU030_SSW_RW) {
            /* A handler which clears DF supplies the read data in DIB. */
            state->fault_data_input_address = state->fault_address;
            state->fault_data_input_valid = true;
        } else if (data_input_valid) {
            /* Preserve the completed read half of an ordinary translated
             * read/write instruction. */
            state->fault_data_input_address = data_input_address;
            state->fault_data_input_valid = true;
        }
    }
    state->fault_data_write = !(ssw & M68K_MMU030_SSW_RW);
    state->fault_rmw = !!(ssw & M68K_MMU030_SSW_RM);
    if (!state->fault_rmw) {
        state->fault_rmw_phase = 0;
        state->fault_rmw_data1 = 0;
        state->fault_rmw_data2 = 0;
        state->fault_rmw_data_valid = false;
    }
    if (!special_valid) {
        m68k_mmu030_special_clear(state);
    }

    /* RTE owns the next cycle only when the frame says it is still pending. */
    state->restart_pending = (ssw & (M68K_MMU030_SSW_DF |
                                     M68K_MMU030_SSW_RC |
                                     M68K_MMU030_SSW_RB)) != 0;
    return true;
}

bool m68k_mmu030_restore_coprocessor_frame(
    M68KMMU030State *state, const uint8_t *frame, size_t frame_size,
    uint16_t *saved_sr, uint32_t *resume_pc)
{
    if (!state || !frame || frame_size != M68K_MMU030_COPROCESSOR_FRAME_SIZE ||
        (lduw_be_p(frame + 0x06) >> 12) != M68K_MMU030_FAULT_FORMAT_9) {
        return false;
    }

    if (saved_sr) {
        *saved_sr = lduw_be_p(frame + 0x00);
    }
    if (resume_pc) {
        *resume_pc = ldl_be_p(frame + 0x02);
    }

    /* The four internal words are consumed by the coprocessor path.  The
     * integer PMMU has no corresponding architectural registers, but the
     * instruction address is part of the common restart image. */
    state->fault_pending = false;
    state->fault_format = M68K_MMU030_FAULT_FORMAT_9;
    state->fault_address = 0;
    state->fault_ssw = 0;
    state->fault_status = 0;
    state->fault_stage_c = 0;
    state->fault_stage_b = 0;
    state->fault_stage_b_address = 0;
    state->fault_data_output = 0;
    state->fault_data_input = 0;
    state->fault_data_input_address = 0;
    state->fault_descriptor_address = 0;
    state->fault_table_level = 0;
    state->fault_size = 0;
    state->fault_function_code = 0;
    state->fault_frame_version = 0;
    state->fault_pc = ldl_be_p(frame + 0x02);
    state->fault_instruction_address = ldl_be_p(frame + 0x08);
    state->restart_pending = false;
    state->fault_resume_pc = 0;
    state->fault_frame_active = m68k_mmu030_fault_frames_active(state);
    state->fault_data_complete = false;
    state->fault_data_input_valid = false;
    state->fault_data_write = false;
    state->fault_rmw = false;
    state->fault_rmw_phase = 0;
    state->fault_rmw_data1 = 0;
    state->fault_rmw_data2 = 0;
    state->fault_rmw_data_valid = false;
    state->fault_fetch_active = false;
    state->fault_code_fetch = false;
    state->fault_pipe_accept = false;
    return true;
}

bool m68k_mmu030_consume_restart(M68KMMU030State *state)
{
    bool pending;

    if (!state) {
        return false;
    }
    pending = state->restart_pending;
    state->restart_pending = false;
    return pending;
}

bool m68k_mmu030_build_short_access_frame(
    M68KMMU030State *state, uint16_t saved_sr, uint16_t vector_offset,
    uint8_t frame[M68K_MMU030_ACCESS_FRAME_SIZE])
{
    uint8_t format;

    if (!state || !state->fault_pending || !frame) {
        return false;
    }

    format = state->fault_format;
    if (format == 0) {
        state->fault_format = M68K_MMU030_FAULT_FORMAT_A;
    }
    if (state->fault_format != M68K_MMU030_FAULT_FORMAT_A) {
        state->fault_format = format;
        return false;
    }
    bool result = m68k_mmu030_build_access_frame(
        state, saved_sr, vector_offset, frame,
        M68K_MMU030_ACCESS_FRAME_SIZE_SHORT);
    if (!result && format == 0) {
        state->fault_format = 0;
    }
    return result;
}

static int m68k_mmu030_walk_level(M68KMMU030State *state,
                                  const M68KMMU030MemoryOps *ops,
                                  uint32_t logical_address, int access_type,
                                  uint8_t function_code, bool probe,
                                  unsigned ptest_level,
                                  M68KMMU030TranslateResult *result)
{
    bool ptest = (access_type & M68K_MMU030_ACCESS_PTEST) != 0;
    bool bounded_ptest = ptest && ptest_level != 0;
    bool no_history = probe || ptest;
    bool is_write = (access_type & M68K_MMU030_ACCESS_STORE) != 0;
    bool is_super = (function_code & 4) != 0;
    unsigned page_bits;
    unsigned initial_shift;
    unsigned ti_width[4];
    unsigned ti_count = 0;
    unsigned logical_bits = 0;
    unsigned table_count = 0;
    uint16_t mmusr = 0;
    uint32_t prot = PAGE_READ | PAGE_WRITE;
    uint32_t root_high;
    uint32_t root_address;
    uint32_t table_address;
    uint32_t logical_shift;
    bool wp_seen = false;
    bool supervisor_seen = false;
    bool cache_inhibit = false;

    memset(result, 0, sizeof(*result));
    if (ptest) {
        state->mmusr = 0;
    }
    result->page_size = UINT32_C(1) << M68K_MMU030_DEFAULT_PAGE_BITS;
    if (access_type & M68K_MMU030_ACCESS_CODE) {
        prot |= PAGE_EXEC;
    }
    result->prot = prot;

    /*
     * Transparent translation applies to level zero only.  A table PTEST
     * explicitly bypasses both the TTs and the ATC.
     */
    if (!bounded_ptest) {
        for (unsigned i = 0; i < ARRAY_SIZE(state->tt); i++) {
            M68KMMU030TTResult tt = m68k_mmu030_tt_match(
                state->tt[i], logical_address, function_code, is_write);

            if (tt.matched) {
                mmusr = M68K_MMU030_MMUSR_T;
                result->physical = logical_address;
                result->cache_inhibit = tt.cache_inhibit;
                result->mmusr = mmusr;
                if (ptest) {
                    state->mmusr = mmusr;
                }
                return 0;
            }
        }
    }

    if (!bounded_ptest && (state->tc & M68K_MMU030_TC_ENABLE) == 0) {
        result->physical = logical_address;
        result->cache_inhibit = false;
        result->mmusr = 0;
        result->prot = prot;
        if (ptest) {
            state->mmusr = result->mmusr;
        }
        return 0;
    }

    page_bits = (state->tc >> M68K_MMU030_TC_PS_SHIFT) & 0xf;
    initial_shift = (state->tc >> M68K_MMU030_TC_IS_SHIFT) & 0xf;
    if (page_bits < 8 || page_bits > 15 ||
        !m68k_mmu030_validate_tc(state->tc)) {
        result->fault = true;
        result->mmusr = M68K_MMU030_MMUSR_I;
        if (ptest) {
            state->mmusr = result->mmusr;
        }
        return -1;
    }
    result->page_size = UINT32_C(1) << page_bits;

    ti_width[0] = (state->tc >> M68K_MMU030_TC_TIA_SHIFT) & 0xf;
    ti_width[1] = (state->tc >> M68K_MMU030_TC_TIB_SHIFT) & 0xf;
    ti_width[2] = (state->tc >> M68K_MMU030_TC_TIC_SHIFT) & 0xf;
    ti_width[3] = (state->tc >> M68K_MMU030_TC_TID_SHIFT) & 0xf;
    while (ti_count < ARRAY_SIZE(ti_width) && ti_width[ti_count]) {
        ti_count++;
    }
    if ((state->tc & M68K_MMU030_TC_SRE) && (function_code & 4)) {
        root_high = state->srp >> 32;
        root_address = state->srp;
    } else {
        root_high = state->crp >> 32;
        root_address = state->crp;
    }
    root_address &= ~UINT32_C(0xf);

    /* The root pointer is a descriptor, not a memory-resident table entry. */
    unsigned root_dt = root_high & M68K_MMU030_DESC_DT_MASK;
    if (root_dt == 0) {
        result->fault = true;
        result->mmusr = M68K_MMU030_MMUSR_I;
        if (ptest) {
            state->mmusr = result->mmusr;
        }
        return -1;
    }

    /* A root page maps directly; its limit applies only when FCL is clear. */
    logical_shift = 32 - initial_shift;
    unsigned first_index = 0;
    if (ti_count) {
        unsigned width = ti_width[0];
        first_index = (logical_address >> (logical_shift - width)) &
                      ((UINT32_C(1) << width) - 1);
    }
    if (root_dt == M68K_MMU030_DESC_PAGE) {
        uint32_t limit = (root_high & M68K_MMU030_DESC_LIMIT_MASK) >> 16;
        bool lower = (root_high & M68K_MMU030_DESC_LU) != 0;
        bool root_wp = (root_high & M68K_MMU030_DESC_WP) != 0;
        bool root_supervisor = (root_high & M68K_MMU030_DESC_S) != 0;
        bool root_modified = (root_high & M68K_MMU030_DESC_M) != 0;
        bool limit_bad = (lower && limit != 0 && first_index < limit) ||
                         (!lower && limit != UINT32_C(0x7fff) &&
                          first_index > limit);

        if ((state->tc & M68K_MMU030_TC_FCL) == 0 && limit_bad) {
            result->fault = true;
            result->limit_violation = true;
            result->atc_error = true;
            mmusr = M68K_MMU030_MMUSR_L | M68K_MMU030_MMUSR_I;
            result->mmusr = mmusr;
            if (ptest) {
                state->mmusr = mmusr;
            }
            return -1;
        }

        result->physical = root_address + logical_address;
        result->cache_inhibit = false;
        result->write_protect = root_wp;
        result->supervisor_only = root_supervisor;
        result->modified = root_modified;
        if (root_supervisor && !is_super) {
            result->mmusr = M68K_MMU030_MMUSR_S;
            result->atc_error = true;
            if (ptest) {
                state->mmusr = result->mmusr;
                return 0;
            }
            result->fault = true;
            return -1;
        }
        result->mmusr = ptest ?
            ((root_modified ? M68K_MMU030_MMUSR_M : 0) |
             (root_wp ? M68K_MMU030_MMUSR_WP : 0)) : 0;
        if (root_wp) {
            result->prot &= ~PAGE_WRITE;
        }
        if (is_write && root_wp && !ptest) {
            result->fault = true;
            result->mmusr = M68K_MMU030_MMUSR_WP;
            return -1;
        }
        if (ptest) {
            state->mmusr = result->mmusr;
        }
        return 0;
    }

    if ((state->tc & M68K_MMU030_TC_FCL) == 0) {
        uint32_t limit = (root_high & M68K_MMU030_DESC_LIMIT_MASK) >> 16;
        bool lower = (root_high & M68K_MMU030_DESC_LU) != 0;
        bool limit_bad = (lower && limit != 0 && first_index < limit) ||
                         (!lower && limit != UINT32_C(0x7fff) &&
                          first_index > limit);

        if (limit_bad) {
            result->fault = true;
            result->limit_violation = true;
            result->atc_error = true;
            mmusr = M68K_MMU030_MMUSR_L | M68K_MMU030_MMUSR_I;
            result->mmusr = mmusr;
            if (ptest) {
                state->mmusr = mmusr;
            }
            return -1;
        }
    }

    table_address = root_address;
    logical_shift = 32 - initial_shift;
    unsigned ti_index = 0;
    bool use_fc = (state->tc & M68K_MMU030_TC_FCL) != 0;
    unsigned index = use_fc ? function_code & 7 : 0;
    bool long_format = root_dt == M68K_MMU030_DESC_VALID8;
    bool have_index = use_fc;
    uint32_t descriptor_address = 0;

    while (true) {
        uint32_t first;
        uint32_t second = 0;
        uint32_t descriptor;
        unsigned dt;
        bool source_long = long_format;
        bool source_page = false;
        bool source_indirect = false;
        bool descriptor_wp;
        bool descriptor_s;
        bool descriptor_ci;
        uint32_t next_table;

        if (!have_index) {
            if (ti_index >= ti_count) {
                result->fault = true;
                result->atc_error = true;
                result->mmusr = M68K_MMU030_MMUSR_I | table_count;
                if (ptest) {
                    state->mmusr = result->mmusr;
                }
                return -1;
            }
            unsigned width = ti_width[ti_index];
            logical_shift -= width;
            index = (logical_address >> logical_shift) &
                    ((UINT32_C(1) << width) - 1);
            logical_bits += width;
            ti_index++;
        }
        have_index = false;

        table_count++;
        descriptor_address = table_address + (index << (source_long ? 3 : 2));
        /* Preserve the descriptor causing a table-read fault as well as the
         * descriptor of a successful walk.  This is consumed by the access
         * error latch when the memory operation below fails. */
        result->descriptor_address = descriptor_address;
        if (!ops || !ops->readl ||
            !ops->readl(ops->opaque, descriptor_address, &first)) {
            result->fault = true;
            result->bus_error = true;
            result->atc_error = true;
            mmusr = M68K_MMU030_MMUSR_B | M68K_MMU030_MMUSR_I |
                    (table_count & M68K_MMU030_MMUSR_N_MASK);
            result->mmusr = mmusr;
            if (ptest) {
                state->mmusr = mmusr;
            }
            return -1;
        }
        if (source_long && (!ops->readl ||
                            !ops->readl(ops->opaque, descriptor_address + 4,
                                        &second))) {
            result->fault = true;
            result->bus_error = true;
            result->atc_error = true;
            mmusr = M68K_MMU030_MMUSR_B | M68K_MMU030_MMUSR_I |
                    (table_count & M68K_MMU030_MMUSR_N_MASK);
            result->mmusr = mmusr;
            if (ptest) {
                state->mmusr = mmusr;
            }
            return -1;
        }

        /* A long descriptor is successful only after both words have been
         * fetched; descriptor_address remains the base of both words. */
        result->descriptor_address = descriptor_address;

        descriptor = first;
        dt = descriptor & M68K_MMU030_DESC_DT_MASK;

        /* DT=0 is invalid; no other descriptor fields are interpreted. */
        if (dt == 0) {
            result->fault = true;
            result->atc_error = true;
            mmusr = M68K_MMU030_MMUSR_I |
                    (table_count & M68K_MMU030_MMUSR_N_MASK);
            result->mmusr = mmusr;
            if (ptest) {
                state->mmusr = mmusr;
            }
            return -1;
        }

        source_page = dt == M68K_MMU030_DESC_PAGE;
        source_indirect = (dt == M68K_MMU030_DESC_VALID4 ||
                           dt == M68K_MMU030_DESC_VALID8) &&
                          ti_index == ti_count;
        descriptor_wp = !source_indirect &&
                        (descriptor & M68K_MMU030_DESC_WP) != 0;
        descriptor_s = source_long && !source_indirect &&
                       (descriptor & M68K_MMU030_DESC_S) != 0;
        descriptor_ci = source_page &&
                        (descriptor & M68K_MMU030_DESC_CI) != 0;

        if (descriptor_wp) {
            wp_seen = true;
        }
        if (descriptor_s) {
            supervisor_seen = true;
        }

        /* Supervisor violations stop the search before the offending U bit. */
        if (descriptor_s && !is_super) {
            mmusr = M68K_MMU030_MMUSR_S |
                    (source_page && (descriptor & M68K_MMU030_DESC_M) ?
                     M68K_MMU030_MMUSR_M : 0) |
                    (wp_seen ? M68K_MMU030_MMUSR_WP : 0) |
                    (table_count & M68K_MMU030_MMUSR_N_MASK);
            result->mmusr = mmusr;
            result->atc_error = true;
            if (ptest) {
                state->mmusr = mmusr;
                return 0;
            }
            result->fault = true;
            return -1;
        }

        /*
         * A bounded table PTEST stops after the requested descriptor.  A
         * page descriptor is still processed to report its attributes, while
         * an indirect descriptor is followed only when the requested level
         * extends past it.
         */
        if (bounded_ptest && table_count >= ptest_level && !source_page) {
            result->mmusr = (wp_seen ? M68K_MMU030_MMUSR_WP : 0) |
                            (table_count & M68K_MMU030_MMUSR_N_MASK);
            state->mmusr = result->mmusr;
            return 0;
        }

        /* U is meaningful in table descriptors and page descriptors. */
        if (!no_history && !(descriptor & M68K_MMU030_DESC_U) &&
            !source_indirect) {
            if (!ops->writel ||
                !ops->writel(ops->opaque, descriptor_address,
                             first | M68K_MMU030_DESC_U)) {
                result->fault = true;
                result->bus_error = true;
                result->atc_error = true;
                mmusr = M68K_MMU030_MMUSR_B | M68K_MMU030_MMUSR_I |
                        (table_count & M68K_MMU030_MMUSR_N_MASK);
                result->mmusr = mmusr;
                if (ptest) {
                    state->mmusr = mmusr;
                }
                return -1;
            }
            first |= M68K_MMU030_DESC_U;
        }

        /* Long descriptor limits apply to the next logical table index. */
        if (source_long && ti_index < ti_count) {
            unsigned width = ti_width[ti_index];
            unsigned next_index = (logical_address >>
                                   (logical_shift - width)) &
                                  ((UINT32_C(1) << width) - 1);
            uint32_t limit = (descriptor & M68K_MMU030_DESC_LIMIT_MASK) >> 16;
            bool lower = (descriptor & M68K_MMU030_DESC_LU) != 0;
            bool limit_bad = (lower && limit != 0 && next_index < limit) ||
                             (!lower && limit != UINT32_C(0x7fff) &&
                              next_index > limit);

            if (limit_bad) {
                result->fault = true;
                result->limit_violation = true;
                result->atc_error = true;
                mmusr = M68K_MMU030_MMUSR_L | M68K_MMU030_MMUSR_I |
                        (table_count & M68K_MMU030_MMUSR_N_MASK);
                result->mmusr = mmusr;
                if (ptest) {
                    state->mmusr = mmusr;
                }
                return -1;
            }
        }

        if (descriptor_ci) {
            cache_inhibit = true;
        }
        if (source_page) {
            uint32_t page_address = source_long ? second : first;
            uint32_t page_mask = ~((UINT32_C(1) << page_bits) - 1);

            if (!no_history && is_write && !wp_seen &&
                !(first & M68K_MMU030_DESC_M)) {
                first |= M68K_MMU030_DESC_M;
                if (!ops->writel ||
                    !ops->writel(ops->opaque, descriptor_address, first)) {
                    result->fault = true;
                    result->bus_error = true;
                    result->atc_error = true;
                    mmusr = M68K_MMU030_MMUSR_B | M68K_MMU030_MMUSR_I |
                            (table_count & M68K_MMU030_MMUSR_N_MASK);
                    result->mmusr = mmusr;
                    if (ptest) {
                        state->mmusr = mmusr;
                    }
                    return -1;
                }
            }

            result->physical = (page_address & page_mask) +
                               (logical_address &
                                ((UINT32_C(1) <<
                                  (32 - initial_shift - logical_bits)) - 1));
            result->cache_inhibit = cache_inhibit;
            result->write_protect = wp_seen;
            result->supervisor_only = supervisor_seen;
            result->modified =
                (first & M68K_MMU030_DESC_M) != 0;
            result->mmusr = ptest ?
                 ((first & M68K_MMU030_DESC_M ?
                  M68K_MMU030_MMUSR_M : 0) |
                 (wp_seen ? M68K_MMU030_MMUSR_WP : 0) |
                 (table_count & M68K_MMU030_MMUSR_N_MASK)) : 0;
            if (wp_seen) {
                result->prot &= ~PAGE_WRITE;
            }
            if (is_write && wp_seen && !ptest) {
                result->fault = true;
                result->mmusr = M68K_MMU030_MMUSR_WP |
                                (table_count & M68K_MMU030_MMUSR_N_MASK);
                return -1;
            }
            if (ptest) {
                state->mmusr = result->mmusr;
            }
            return 0;
        }

        if (source_indirect) {
            uint32_t indirect_address = source_long ? second : first;
            uint32_t indirect_first;
            uint32_t indirect_second = 0;
            bool indirect_long = dt == M68K_MMU030_DESC_VALID8;
            uint32_t page_address;

            if (bounded_ptest) {
                /*
                 * The indirect page descriptor is an additional table level
                 * for PTEST's MMUSR.N accounting.
                 */
                table_count++;
            }
            indirect_address &= ~UINT32_C(3);
            /* An indirect descriptor is a second table access.  Publish its
             * address before either long/short fetch so a bus error points at
             * the actual failing table word. */
            result->descriptor_address = indirect_address;
            if (!ops || !ops->readl ||
                !ops->readl(ops->opaque, indirect_address,
                            &indirect_first) ||
                (indirect_long &&
                 (!ops->readl(ops->opaque, indirect_address + 4,
                              &indirect_second)))) {
                result->fault = true;
                result->bus_error = true;
                result->atc_error = true;
                mmusr = M68K_MMU030_MMUSR_B | M68K_MMU030_MMUSR_I |
                        (table_count & M68K_MMU030_MMUSR_N_MASK);
                result->mmusr = mmusr;
                if (ptest) {
                    state->mmusr = mmusr;
                }
                return -1;
            }
            result->descriptor_address = indirect_address;
            if ((indirect_first & M68K_MMU030_DESC_DT_MASK) !=
                M68K_MMU030_DESC_PAGE) {
                result->fault = true;
                result->atc_error = true;
                mmusr = M68K_MMU030_MMUSR_I |
                        (table_count & M68K_MMU030_MMUSR_N_MASK);
                result->mmusr = mmusr;
                if (ptest) {
                    state->mmusr = mmusr;
                }
                return -1;
            }

            bool indirect_s = indirect_long &&
                               (indirect_first & M68K_MMU030_DESC_S);
            bool indirect_wp = (indirect_first & M68K_MMU030_DESC_WP) != 0;
            bool indirect_ci = (indirect_first & M68K_MMU030_DESC_CI) != 0;
            bool indirect_m =
                (indirect_first & M68K_MMU030_DESC_M) != 0;
            wp_seen |= indirect_wp;
            supervisor_seen |= indirect_s;
            if (indirect_s && !is_super) {
                mmusr = M68K_MMU030_MMUSR_S |
                        (indirect_m ? M68K_MMU030_MMUSR_M : 0) |
                        (wp_seen ? M68K_MMU030_MMUSR_WP : 0) |
                        (table_count & M68K_MMU030_MMUSR_N_MASK);
                result->mmusr = mmusr;
                result->atc_error = true;
                if (ptest) {
                    state->mmusr = mmusr;
                    return 0;
                }
                result->fault = true;
                return -1;
            }
            if (!no_history && !(indirect_first & M68K_MMU030_DESC_U)) {
                uint32_t updated = indirect_first | M68K_MMU030_DESC_U;

                if (!ops->writel ||
                    !ops->writel(ops->opaque, indirect_address, updated)) {
                    result->fault = true;
                    result->bus_error = true;
                    result->atc_error = true;
                    mmusr = M68K_MMU030_MMUSR_B | M68K_MMU030_MMUSR_I |
                            (table_count & M68K_MMU030_MMUSR_N_MASK);
                    result->mmusr = mmusr;
                    if (ptest) {
                        state->mmusr = mmusr;
                    }
                    return -1;
                }
                indirect_first = updated;
            }
            if (!no_history && is_write && !wp_seen &&
                !(indirect_first & M68K_MMU030_DESC_M)) {
                indirect_first |= M68K_MMU030_DESC_M;
                if (!ops->writel ||
                    !ops->writel(ops->opaque, indirect_address,
                                 indirect_first)) {
                    result->fault = true;
                    result->bus_error = true;
                    result->atc_error = true;
                    mmusr = M68K_MMU030_MMUSR_B | M68K_MMU030_MMUSR_I |
                            (table_count & M68K_MMU030_MMUSR_N_MASK);
                    result->mmusr = mmusr;
                    if (ptest) {
                        state->mmusr = result->mmusr;
                    }
                    return -1;
                }
            }
            page_address = indirect_long ? indirect_second : indirect_first;
            page_address &= ~((UINT32_C(1) << page_bits) - 1);
            cache_inhibit |= indirect_ci;
            wp_seen |= indirect_wp;
            result->physical = page_address +
                               (logical_address &
                                ((UINT32_C(1) <<
                                  (32 - initial_shift - logical_bits)) - 1));
            result->cache_inhibit = cache_inhibit;
            result->write_protect = wp_seen;
            result->supervisor_only = supervisor_seen;
            result->modified =
                (indirect_first & M68K_MMU030_DESC_M) != 0;
            result->mmusr = ptest ?
                ((indirect_m ? M68K_MMU030_MMUSR_M : 0) |
                 (wp_seen ? M68K_MMU030_MMUSR_WP : 0) |
                 (table_count & M68K_MMU030_MMUSR_N_MASK)) : 0;
            if (wp_seen) {
                result->prot &= ~PAGE_WRITE;
            }
            if (is_write && wp_seen && !ptest) {
                result->fault = true;
                result->mmusr = M68K_MMU030_MMUSR_WP |
                                (table_count & M68K_MMU030_MMUSR_N_MASK);
                return -1;
            }
            if (ptest) {
                state->mmusr = result->mmusr;
            }
            return 0;
        }

        next_table = source_long ? (second & ~UINT32_C(0xf)) :
                                    (first & ~UINT32_C(0xf));
        table_address = next_table;
        long_format = dt == M68K_MMU030_DESC_VALID8;
    }
}

int m68k_mmu030_walk(M68KMMU030State *state,
                     const M68KMMU030MemoryOps *ops,
                     uint32_t logical_address, int access_type,
                     uint8_t function_code, bool probe,
                     M68KMMU030TranslateResult *result)
{
    /*
     * Preserve the historical walker contract: a direct PTEST-marked walk
     * remains unbounded.  Architectural PTEST levels use the API below.
     */
    return m68k_mmu030_walk_level(state, ops, logical_address, access_type,
                                  function_code, probe, 0, result);
}

int m68k_mmu030_ptest(M68KMMU030State *state,
                      const M68KMMU030MemoryOps *ops,
                      uint32_t logical_address, bool is_write,
                      uint8_t function_code, unsigned level,
                      M68KMMU030TranslateResult *result)
{
    int access_type = M68K_MMU030_ACCESS_PTEST;

    if (!state || !result || level == 0 || level > 7) {
        return -1;
    }
    if (is_write) {
        access_type |= M68K_MMU030_ACCESS_STORE;
    }

    /*
     * PTEST reports architectural status rather than taking the walk's
     * translation fault as a helper error.
     */
    m68k_mmu030_walk_level(state, ops, logical_address, access_type,
                           function_code, true, level, result);
    return 0;
}
