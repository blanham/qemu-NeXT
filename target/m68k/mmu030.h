/*
 * MC68030 PMMU state.
 *
 * This header is private to the m68k target.  The MC68030 PMMU is separate
 * from the MC68040 MMU state because the two processors expose different
 * control registers and translation-cache formats.
 *
 * Copyright (c) 2026 Bryce Lanham
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef M68K_MMU030_H
#define M68K_MMU030_H

#include <stdbool.h>
#include <stdint.h>

#include "qemu/typedefs.h"

#define M68K_MMU030_ATC_ENTRIES 22

typedef struct M68KMMU030ATCEntry {
    uint32_t logical;
    uint32_t physical;
    uint32_t status;
} M68KMMU030ATCEntry;

/* Result of matching one logical access against an MC68030 TT register. */
typedef struct M68KMMU030TTResult {
    bool matched;
    bool cache_inhibit;
} M68KMMU030TTResult;

typedef struct M68KMMU030State {
    uint64_t crp;
    uint64_t srp;
    uint32_t tc;
    uint32_t tt[2];
    uint16_t mmusr;

    M68KMMU030ATCEntry atc[M68K_MMU030_ATC_ENTRIES];
    uint8_t atc_next;

    /* State needed to construct a later MC68030 access-error frame. */
    bool fault_pending;
    uint32_t fault_address;
    uint32_t fault_pc;
    uint16_t fault_ssw;
    uint32_t fault_status;
} M68KMMU030State;

void m68k_mmu030_reset(M68KMMU030State *state);

bool m68k_mmu030_validate_tc(uint32_t tc);
/* is_write is true for a write access and false for a read access. */
M68KMMU030TTResult m68k_mmu030_tt_match(uint32_t tt,
                                         uint32_t logical_address,
                                         uint8_t function_code,
                                         bool is_write);

extern const VMStateDescription vmstate_mmu030_state;

#endif /* M68K_MMU030_H */
