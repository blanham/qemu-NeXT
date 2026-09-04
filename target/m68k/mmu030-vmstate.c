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

static const VMStateDescription vmstate_mmu030_fault_context = {
    .name = "m68k_mmu030_fault_context",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(frame_start, M68KMMU030FaultContext),
        VMSTATE_UINT32(frame_end, M68KMMU030FaultContext),
        VMSTATE_BOOL(fault_pending, M68KMMU030FaultContext),
        VMSTATE_UINT32(fault_address, M68KMMU030FaultContext),
        VMSTATE_UINT32(fault_pc, M68KMMU030FaultContext),
        VMSTATE_UINT16(fault_ssw, M68KMMU030FaultContext),
        VMSTATE_UINT32(fault_status, M68KMMU030FaultContext),
        VMSTATE_UINT8(fault_format, M68KMMU030FaultContext),
        VMSTATE_UINT8(fault_size, M68KMMU030FaultContext),
        VMSTATE_UINT8(fault_function_code, M68KMMU030FaultContext),
        VMSTATE_UINT8(fault_table_level, M68KMMU030FaultContext),
        VMSTATE_UINT16(fault_stage_c, M68KMMU030FaultContext),
        VMSTATE_UINT16(fault_stage_b, M68KMMU030FaultContext),
        VMSTATE_UINT32(fault_stage_b_address, M68KMMU030FaultContext),
        VMSTATE_UINT32(fault_data_output, M68KMMU030FaultContext),
        VMSTATE_UINT32(fault_data_input, M68KMMU030FaultContext),
        VMSTATE_UINT32(fault_data_input_address, M68KMMU030FaultContext),
        VMSTATE_UINT32(fault_descriptor_address, M68KMMU030FaultContext),
        VMSTATE_UINT32(fault_instruction_address, M68KMMU030FaultContext),
        VMSTATE_UINT32(fault_resume_pc, M68KMMU030FaultContext),
        VMSTATE_BOOL(fault_data_complete, M68KMMU030FaultContext),
        VMSTATE_BOOL(fault_data_input_valid, M68KMMU030FaultContext),
        VMSTATE_BOOL(fault_data_write, M68KMMU030FaultContext),
        VMSTATE_UINT8(fault_frame_version, M68KMMU030FaultContext),
        VMSTATE_BOOL(restart_pending, M68KMMU030FaultContext),
        VMSTATE_BOOL(fault_rmw, M68KMMU030FaultContext),
        VMSTATE_BOOL(fault_fetch_active, M68KMMU030FaultContext),
        VMSTATE_BOOL(fault_code_fetch, M68KMMU030FaultContext),
        VMSTATE_BOOL(fault_pipe_accept, M68KMMU030FaultContext),
        VMSTATE_UINT8(fault_rmw_phase, M68KMMU030FaultContext),
        VMSTATE_UINT32(fault_rmw_data1, M68KMMU030FaultContext),
        VMSTATE_UINT32(fault_rmw_data2, M68KMMU030FaultContext),
        VMSTATE_BOOL(fault_rmw_data_valid, M68KMMU030FaultContext),
        VMSTATE_UINT8(fault_special_kind, M68KMMU030FaultContext),
        VMSTATE_UINT8(fault_special_phase, M68KMMU030FaultContext),
        VMSTATE_BOOL(fault_special_valid, M68KMMU030FaultContext),
        VMSTATE_UINT32(fault_special_pc, M68KMMU030FaultContext),
        VMSTATE_UINT32_ARRAY(fault_special_data, M68KMMU030FaultContext,
                             M68K_MMU030_SPECIAL_MAX_CYCLES),
        VMSTATE_END_OF_LIST()
    }
};

static int vmstate_mmu030_state_post_load(void *opaque, int version_id)
{
    M68KMMU030State *state = opaque;

    /* Versions 3 through 6 had one scalar live-frame marker but no frame
     * identity.  Keep that conservative ownership only for an actually old
     * stream; a current compact stream with zero entries must not acquire a
     * phantom legacy frame. */
    if (version_id < 7) {
        state->fault_legacy_frame_active = state->fault_frame_active &&
                                          state->fault_frame_depth == 0;
    } else if (version_id < 8) {
        /* v7 introduced compact frame identities but had no way to carry a
         * pre-v7 scalar-only frame through another migration hop. */
        state->fault_legacy_frame_active = false;
    } else {
        state->fault_legacy_frame_active =
            state->fault_legacy_frame_active &&
            state->fault_frame_depth == 0;
    }
    if (version_id < 7) {
        /* The depth field did not exist in the old wire format. */
        state->fault_frame_depth = 0;
    }
    state->fault_frame_active = state->fault_frame_depth != 0 ||
                                state->fault_legacy_frame_active;
    return 0;
}

const VMStateDescription vmstate_mmu030_state = {
    .name = "cpu/68030_mmu_state",
    .version_id = 8,
    .minimum_version_id = 1,
    .post_load = vmstate_mmu030_state_post_load,
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
        /* Access-error restart state was added after the initial PMMU
         * migration stream.  Keep the old stream loadable and default the
         * new fields to zero when loading version 1. */
        VMSTATE_UINT8_V(fault_format, M68KMMU030State, 2),
        VMSTATE_UINT8_V(fault_size, M68KMMU030State, 2),
        VMSTATE_UINT8_V(fault_function_code, M68KMMU030State, 2),
        VMSTATE_UINT8_V(fault_table_level, M68KMMU030State, 2),
        VMSTATE_UINT16_V(fault_stage_c, M68KMMU030State, 2),
        VMSTATE_UINT16_V(fault_stage_b, M68KMMU030State, 2),
        VMSTATE_UINT32_V(fault_stage_b_address, M68KMMU030State, 2),
        VMSTATE_UINT32_V(fault_data_output, M68KMMU030State, 2),
        VMSTATE_UINT32_V(fault_data_input, M68KMMU030State, 2),
        VMSTATE_UINT32_V(fault_descriptor_address, M68KMMU030State, 2),
        VMSTATE_UINT32_V(fault_instruction_address, M68KMMU030State, 2),
        VMSTATE_UINT8_V(fault_frame_version, M68KMMU030State, 2),
        VMSTATE_BOOL_V(restart_pending, M68KMMU030State, 2),
        /* Keep version-3 state after the complete version-2 prefix so
         * readers of the earlier stream retain the old field offsets. */
        VMSTATE_UINT32_V(fault_resume_pc, M68KMMU030State, 3),
        VMSTATE_BOOL_V(fault_frame_active, M68KMMU030State, 3),
        VMSTATE_BOOL_V(fault_data_complete, M68KMMU030State, 3),
        VMSTATE_BOOL_V(fault_data_input_valid, M68KMMU030State, 3),
        VMSTATE_BOOL_V(fault_data_write, M68KMMU030State, 3),
        VMSTATE_UINT32_V(fault_data_input_address, M68KMMU030State, 3),
        VMSTATE_BOOL_V(fault_fetch_active, M68KMMU030State, 4),
        VMSTATE_BOOL_V(fault_code_fetch, M68KMMU030State, 4),
        VMSTATE_BOOL_V(fault_pipe_accept, M68KMMU030State, 4),
        /* CAS2 phase/data state must survive migration while its handler
         * repairs a mapping between independent bus cycles. */
        VMSTATE_UINT8_V(fault_rmw_phase, M68KMMU030State, 5),
        VMSTATE_UINT32_V(fault_rmw_data1, M68KMMU030State, 5),
        VMSTATE_UINT32_V(fault_rmw_data2, M68KMMU030State, 5),
        VMSTATE_BOOL_V(fault_rmw_data_valid, M68KMMU030State, 5),
        VMSTATE_UINT8_V(fault_special_kind, M68KMMU030State, 6),
        VMSTATE_UINT8_V(fault_special_phase, M68KMMU030State, 6),
        VMSTATE_BOOL_V(fault_special_valid, M68KMMU030State, 6),
        VMSTATE_UINT32_V(fault_special_pc, M68KMMU030State, 6),
        VMSTATE_UINT32_ARRAY_V(fault_special_data, M68KMMU030State,
                               M68K_MMU030_SPECIAL_MAX_CYCLES, 6),
        /* Nested handler restart contexts and the entry guard were appended
         * together so all post-v6 state is versioned as one extension. */
        VMSTATE_BOOL_V(fault_exception_processing, M68KMMU030State, 7),
        VMSTATE_UINT8_V(fault_exception_prefetch_words,
                        M68KMMU030State, 7),
        VMSTATE_STRUCT_ARRAY(fault_frames, M68KMMU030State,
                             M68K_MMU030_MAX_FAULT_FRAMES, 7,
                             vmstate_mmu030_fault_context,
                             M68KMMU030FaultContext),
        VMSTATE_UINT8_V(fault_frame_depth, M68KMMU030State, 7),
        /* Preserve a scalar-only live frame when a pre-v7 state is migrated
         * again after it has been loaded. */
        VMSTATE_BOOL_V(fault_legacy_frame_active, M68KMMU030State, 8),
        VMSTATE_END_OF_LIST()
    }
};
