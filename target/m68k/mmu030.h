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

/* Translation control register fields. */
#define M68K_MMU030_TC_ENABLE       (UINT32_C(1) << 31)
#define M68K_MMU030_TC_SRE          (UINT32_C(1) << 25)
#define M68K_MMU030_TC_FCL          (UINT32_C(1) << 24)
#define M68K_MMU030_TC_PS_SHIFT     20
#define M68K_MMU030_TC_IS_SHIFT     16
#define M68K_MMU030_TC_TIA_SHIFT    12
#define M68K_MMU030_TC_TIB_SHIFT    8
#define M68K_MMU030_TC_TIC_SHIFT    4
#define M68K_MMU030_TC_TID_SHIFT    0

/* MC68030 MMUSR fields (Table 9-3 of the User's Manual). */
#define M68K_MMU030_MMUSR_B         UINT16_C(0x8000)
#define M68K_MMU030_MMUSR_L         UINT16_C(0x4000)
#define M68K_MMU030_MMUSR_S         UINT16_C(0x2000)
#define M68K_MMU030_MMUSR_WP        UINT16_C(0x0800)
#define M68K_MMU030_MMUSR_I         UINT16_C(0x0400)
#define M68K_MMU030_MMUSR_M         UINT16_C(0x0200)
#define M68K_MMU030_MMUSR_T         UINT16_C(0x0040)
#define M68K_MMU030_MMUSR_N_MASK    UINT16_C(0x0007)

/* MC68030 short bus-cycle fault frame and SSW fields (Section 8.4). */
#define M68K_MMU030_ACCESS_FRAME_SIZE UINT32_C(0x20)
#define M68K_MMU030_SSW_OF           UINT16_C(0x0100)
#define M68K_MMU030_SSW_RW           UINT16_C(0x0040)
#define M68K_MMU030_SSW_SIZE_SHIFT   4
#define M68K_MMU030_SSW_FC_MASK      UINT16_C(0x0007)

/* Descriptor fields common to short and long descriptors. */
#define M68K_MMU030_DESC_DT_MASK    UINT32_C(0x00000003)
#define M68K_MMU030_DESC_PAGE       UINT32_C(0x00000001)
#define M68K_MMU030_DESC_VALID4     UINT32_C(0x00000002)
#define M68K_MMU030_DESC_VALID8     UINT32_C(0x00000003)
#define M68K_MMU030_DESC_WP         UINT32_C(0x00000004)
#define M68K_MMU030_DESC_U          UINT32_C(0x00000008)
#define M68K_MMU030_DESC_M          UINT32_C(0x00000010)
#define M68K_MMU030_DESC_CI         UINT32_C(0x00000040)
#define M68K_MMU030_DESC_S          UINT32_C(0x00000080)
#define M68K_MMU030_DESC_LU         UINT32_C(0x80000000)
#define M68K_MMU030_DESC_LIMIT_MASK UINT32_C(0x7fff0000)

typedef struct M68KMMU030MemoryOps {
    bool (*readl)(void *opaque, uint32_t address, uint32_t *value);
    bool (*writel)(void *opaque, uint32_t address, uint32_t value);
    void *opaque;
} M68KMMU030MemoryOps;

typedef struct M68KMMU030TranslateResult {
    uint32_t physical;
    uint32_t page_size;
    int prot;
    uint16_t mmusr;
    bool cache_inhibit;
    bool fault;
    bool bus_error;
    bool limit_violation;
} M68KMMU030TranslateResult;

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

    /* Pending state used to construct an MC68030 access-error frame. */
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

int m68k_mmu030_walk(M68KMMU030State *state,
                     const M68KMMU030MemoryOps *ops,
                     uint32_t logical_address, int access_type,
                     uint8_t function_code, bool probe,
                     M68KMMU030TranslateResult *result);

int m68k_mmu030_translate(CPUArchState *env, uint32_t logical_address,
                          int access_type, uint8_t function_code, bool probe,
                          M68KMMU030TranslateResult *result);

uint16_t m68k_mmu030_make_ssw(unsigned size, bool is_write, bool is_code,
                              uint8_t function_code);

uint32_t m68k_mmu030_rte_frame_tail_size(uint16_t format);

bool m68k_mmu030_build_short_access_frame(
    M68KMMU030State *state, uint16_t saved_sr, uint16_t vector_offset,
    uint8_t frame[M68K_MMU030_ACCESS_FRAME_SIZE]);

extern const VMStateDescription vmstate_mmu030_state;

#endif /* M68K_MMU030_H */
