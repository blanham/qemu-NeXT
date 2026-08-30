/*
 * MC68030 PMMU architectural state.
 *
 * Copyright (c) 2026 Bryce Lanham
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "target/m68k/mmu030.h"

#define M68K_MMU030_TC_ENABLE        (UINT32_C(1) << 31)
#define M68K_MMU030_TC_PS_SHIFT      20
#define M68K_MMU030_TC_IS_SHIFT      16

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
