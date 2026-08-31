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

uint16_t m68k_mmu030_make_ssw(unsigned size, bool is_write, bool is_code,
                              uint8_t function_code)
{
    unsigned size_code;

    /* Instruction faults use the pipe fault/rerun bits, deferred to Task 7. */
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

    return M68K_MMU030_SSW_OF |
           (is_write ? 0 : M68K_MMU030_SSW_RW) |
           (size_code << M68K_MMU030_SSW_SIZE_SHIFT) |
           (function_code & M68K_MMU030_SSW_FC_MASK);
}

uint32_t m68k_mmu030_rte_frame_tail_size(uint16_t format)
{
    if ((format >> 12) == 0xa) {
        /* RTE has already consumed the status, PC, and format words. */
        return M68K_MMU030_ACCESS_FRAME_SIZE - 8;
    }
    return 0;
}

bool m68k_mmu030_build_short_access_frame(
    M68KMMU030State *state, uint16_t saved_sr, uint16_t vector_offset,
    uint8_t frame[M68K_MMU030_ACCESS_FRAME_SIZE])
{
    if (!state || !state->fault_pending || !frame) {
        return false;
    }

    /*
     * Section 8.4, format $A: ordinary faults taken at an instruction
     * boundary use this 16-word frame.  The pipeline and rerun state needed
     * by long format $B (and format $9 coprocessor frames) is deferred to the
     * later exception-recovery work; keep those internal words zero here.
     */
    memset(frame, 0, M68K_MMU030_ACCESS_FRAME_SIZE);
    stw_be_p(frame + 0x00, saved_sr);
    stl_be_p(frame + 0x02, state->fault_pc);
    stw_be_p(frame + 0x06, UINT16_C(0xa000) | (vector_offset & 0x0fff));
    stw_be_p(frame + 0x0a, state->fault_ssw);
    stl_be_p(frame + 0x10, state->fault_address);

    /* The frame has consumed the pending MMU fault context. */
    state->fault_pending = false;
    return true;
}

int m68k_mmu030_walk(M68KMMU030State *state,
                     const M68KMMU030MemoryOps *ops,
                     uint32_t logical_address, int access_type,
                     uint8_t function_code, bool probe,
                     M68KMMU030TranslateResult *result)
{
    bool ptest = (access_type & M68K_MMU030_ACCESS_PTEST) != 0;
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

    /* Transparent translation is independent of TC.E. */
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

    if ((state->tc & M68K_MMU030_TC_ENABLE) == 0) {
        result->physical = logical_address;
        result->cache_inhibit = false;
        result->mmusr = 0;
        result->prot = prot;
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
    if (ti_count == 0) {
        result->fault = true;
        result->mmusr = M68K_MMU030_MMUSR_I;
        if (ptest) {
            state->mmusr = result->mmusr;
        }
        return -1;
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

    /* A root page descriptor is direct mapping, but still checks its limit. */
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
        bool limit_bad = (lower && limit != 0 && first_index < limit) ||
                         (!lower && limit != UINT32_C(0x7fff) &&
                          first_index > limit);

        if (limit_bad) {
            result->fault = true;
            result->limit_violation = true;
            mmusr = M68K_MMU030_MMUSR_L | M68K_MMU030_MMUSR_I;
            result->mmusr = mmusr;
            if (ptest) {
                state->mmusr = mmusr;
            }
            return -1;
        }

        result->physical = root_address + logical_address;
        result->cache_inhibit = false;
        result->mmusr = 0;
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
        if (!ops || !ops->readl ||
            !ops->readl(ops->opaque, descriptor_address, &first)) {
            result->fault = true;
            result->bus_error = true;
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
            mmusr = M68K_MMU030_MMUSR_B | M68K_MMU030_MMUSR_I |
                    (table_count & M68K_MMU030_MMUSR_N_MASK);
            result->mmusr = mmusr;
            if (ptest) {
                state->mmusr = mmusr;
            }
            return -1;
        }

        descriptor = first;
        dt = descriptor & M68K_MMU030_DESC_DT_MASK;

        /* DT=0 is invalid; no other descriptor fields are interpreted. */
        if (dt == 0) {
            result->fault = true;
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

        /* Supervisor violations stop the search before the offending U bit. */
        if (descriptor_s && !is_super) {
            mmusr = M68K_MMU030_MMUSR_S |
                    (source_page && (descriptor & M68K_MMU030_DESC_M) ?
                     M68K_MMU030_MMUSR_M : 0) |
                    (wp_seen ? M68K_MMU030_MMUSR_WP : 0) |
                    (table_count & M68K_MMU030_MMUSR_N_MASK);
            result->mmusr = mmusr;
            if (ptest) {
                state->mmusr = mmusr;
                return 0;
            }
            result->fault = true;
            return -1;
        }

        /* U is meaningful in table descriptors and page descriptors. */
        if (!no_history && !(descriptor & M68K_MMU030_DESC_U) &&
            !source_indirect) {
            if (!ops->writel ||
                !ops->writel(ops->opaque, descriptor_address,
                             first | M68K_MMU030_DESC_U)) {
                result->fault = true;
                result->bus_error = true;
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

            indirect_address &= ~UINT32_C(3);
            if (!ops || !ops->readl ||
                !ops->readl(ops->opaque, indirect_address,
                            &indirect_first) ||
                (indirect_long &&
                 (!ops->readl(ops->opaque, indirect_address + 4,
                              &indirect_second)))) {
                result->fault = true;
                result->bus_error = true;
                mmusr = M68K_MMU030_MMUSR_B | M68K_MMU030_MMUSR_I |
                        (table_count & M68K_MMU030_MMUSR_N_MASK);
                result->mmusr = mmusr;
                if (ptest) {
                    state->mmusr = mmusr;
                }
                return -1;
            }
            if ((indirect_first & M68K_MMU030_DESC_DT_MASK) !=
                M68K_MMU030_DESC_PAGE) {
                result->fault = true;
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
            if (indirect_s && !is_super) {
                mmusr = M68K_MMU030_MMUSR_S |
                        (indirect_m ? M68K_MMU030_MMUSR_M : 0) |
                        (wp_seen ? M68K_MMU030_MMUSR_WP : 0) |
                        (table_count & M68K_MMU030_MMUSR_N_MASK);
                result->mmusr = mmusr;
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
