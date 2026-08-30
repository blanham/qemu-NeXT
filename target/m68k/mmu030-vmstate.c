/*
 * MC68030 PMMU migration state.
 *
 * Copyright (c) 2026 Bryce Lanham
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "target/m68k/mmu030.h"
#include "migration/vmstate.h"

static const VMStateDescription vmstate_mmu030_atc_entry = {
    .name = "m68k_mmu030_atc_entry",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(logical, M68KMMU030ATCEntry),
        VMSTATE_UINT32(physical, M68KMMU030ATCEntry),
        VMSTATE_UINT32(status, M68KMMU030ATCEntry),
        VMSTATE_END_OF_LIST()
    }
};

const VMStateDescription vmstate_mmu030_state = {
    .name = "cpu/68030_mmu_state",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(crp, M68KMMU030State),
        VMSTATE_UINT64(srp, M68KMMU030State),
        VMSTATE_UINT32(tc, M68KMMU030State),
        VMSTATE_UINT32_ARRAY(tt, M68KMMU030State, 2),
        VMSTATE_UINT16(mmusr, M68KMMU030State),
        VMSTATE_STRUCT_ARRAY(atc, M68KMMU030State,
                             M68K_MMU030_ATC_ENTRIES, 0,
                             vmstate_mmu030_atc_entry,
                             M68KMMU030ATCEntry),
        VMSTATE_UINT8(atc_next, M68KMMU030State),
        VMSTATE_BOOL(fault_pending, M68KMMU030State),
        VMSTATE_UINT32(fault_address, M68KMMU030State),
        VMSTATE_UINT32(fault_pc, M68KMMU030State),
        VMSTATE_UINT16(fault_ssw, M68KMMU030State),
        VMSTATE_UINT32(fault_status, M68KMMU030State),
        VMSTATE_END_OF_LIST()
    }
};
