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
#include <stddef.h>
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

/*
 * MC68030 bus-cycle fault frames and SSW fields (Sections 8.2 and 8.4).
 *
 * The short frame is 16 words.  A long frame contains the same first 16
 * words followed by 30 words of restart state.  Keep the old short-frame
 * name as an alias: it is part of the small unit-test API introduced with
 * the initial PMMU implementation.
 */
#define M68K_MMU030_ACCESS_FRAME_SIZE_SHORT UINT32_C(0x20)
#define M68K_MMU030_ACCESS_FRAME_SIZE_LONG  UINT32_C(0x5c)
#define M68K_MMU030_ACCESS_FRAME_SIZE       M68K_MMU030_ACCESS_FRAME_SIZE_SHORT
#define M68K_MMU030_COPROCESSOR_FRAME_SIZE  UINT32_C(0x14)
#define M68K_MMU030_FRAME_VERSION           UINT8_C(1)

/* Format codes stored in the high nibble of the vector-offset word. */
#define M68K_MMU030_FAULT_FORMAT_9         UINT8_C(0x9)
#define M68K_MMU030_FAULT_FORMAT_A         UINT8_C(0xa)
#define M68K_MMU030_FAULT_FORMAT_B         UINT8_C(0xb)

/* MC68030 special status word (Figure 8-9). */
#define M68K_MMU030_SSW_FC           UINT16_C(0x8000)
#define M68K_MMU030_SSW_FB           UINT16_C(0x4000)
#define M68K_MMU030_SSW_RC           UINT16_C(0x2000)
#define M68K_MMU030_SSW_RB           UINT16_C(0x1000)
#define M68K_MMU030_SSW_DF           UINT16_C(0x0100)
#define M68K_MMU030_SSW_RM           UINT16_C(0x0080)
#define M68K_MMU030_SSW_RW           UINT16_C(0x0040)
#define M68K_MMU030_SSW_SIZE_SHIFT   4
#define M68K_MMU030_SSW_SIZE_MASK    UINT16_C(0x0030)
#define M68K_MMU030_SSW_SIZE_BYTE    UINT16_C(0x0010)
#define M68K_MMU030_SSW_SIZE_WORD    UINT16_C(0x0020)
#define M68K_MMU030_SSW_SIZE_LONG    UINT16_C(0x0000)
#define M68K_MMU030_SSW_FC_MASK      UINT16_C(0x0007)

/* Multi-cycle instructions which do not use the CAS/CAS2 RM protocol still
 * need a restart latch.  The latch records completed bus cycles and, for
 * reads, their values so a handler cannot cause an earlier device cycle to
 * be issued again when RTE retries the instruction. */
enum {
    M68K_MMU030_SPECIAL_NONE = 0,
    M68K_MMU030_SPECIAL_MOVEP,
    M68K_MMU030_SPECIAL_PMOVE,
    M68K_MMU030_SPECIAL_FPU,
    M68K_MMU030_SPECIAL_SPLIT,
    M68K_MMU030_SPECIAL_FMOVEM,
};
#define M68K_MMU030_SPECIAL_MAX_CYCLES 4

/* Compatibility spelling used by the Task3 short-frame API. */
#define M68K_MMU030_SSW_OF M68K_MMU030_SSW_DF

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

/*
 * The architectural ATC is represented as a fully associative array.  The
 * real part/tag split is kept in the compact status word below so that the
 * existing migration wire format remains stable while still retaining the
 * page-size and function-code portions of a tag.
 */
#define M68K_MMU030_ATC_VALID           UINT32_C(0x80000000)
#define M68K_MMU030_ATC_BUS_ERROR       UINT32_C(0x40000000)
#define M68K_MMU030_ATC_WRITE_PROTECT   UINT32_C(0x20000000)
#define M68K_MMU030_ATC_SUPERVISOR      UINT32_C(0x10000000)
#define M68K_MMU030_ATC_MODIFIED        UINT32_C(0x08000000)
#define M68K_MMU030_ATC_CACHE_INHIBIT   UINT32_C(0x04000000)
#define M68K_MMU030_ATC_FC_SHIFT        23
#define M68K_MMU030_ATC_FC_MASK         UINT32_C(0x03800000)
#define M68K_MMU030_ATC_PAGE_BITS_SHIFT 19
#define M68K_MMU030_ATC_PAGE_BITS_MASK  UINT32_C(0x00780000)

typedef struct M68KMMU030MemoryOps {
    bool (*readl)(void *opaque, uint32_t address, uint32_t *value);
    bool (*writel)(void *opaque, uint32_t address, uint32_t value);
    void *opaque;
} M68KMMU030MemoryOps;

typedef struct M68KMMU030TranslateResult {
    uint32_t physical;
    uint32_t page_size;
    /* Physical address of the last descriptor fetched by a table PTEST. */
    uint32_t descriptor_address;
    int prot;
    uint16_t mmusr;
    bool cache_inhibit;
    bool fault;
    bool bus_error;
    bool limit_violation;
    /* A failed table search creates a resident ATC entry with B set. */
    bool atc_error;
    bool write_protect;
    bool supervisor_only;
    bool modified;
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

    /*
     * Restart information for the 68030 bus-error frames.  These values are
     * deliberately kept as scalar fields instead of a host-layout struct:
     * the frame is a big-endian architectural object and the fields are also
     * migrated independently of host alignment and bit-field rules.
     */
    uint8_t fault_format;
    uint8_t fault_size;
    uint8_t fault_function_code;
    uint8_t fault_table_level;
    uint16_t fault_stage_c;
    uint16_t fault_stage_b;
    uint32_t fault_stage_b_address;
    uint32_t fault_data_output;
    uint32_t fault_data_input;
    uint32_t fault_data_input_address;
    uint32_t fault_descriptor_address;
    uint32_t fault_instruction_address;
    uint32_t fault_resume_pc;
    /* An access frame is live while its handler is running. */
    bool fault_frame_active;
    /* A cleared DF lets the translated instruction consume the stacked data
     * buffer without issuing the completed bus cycle again. */
    bool fault_data_complete;
    bool fault_data_input_valid;
    bool fault_data_write;
    uint8_t fault_frame_version;
    /* RTE has restored a bus-cycle frame and still owes its rerun cycle. */
    bool restart_pending;
    /* A translated CAS/CAS2/TAS has an indivisible read-modify-write cycle.
     * This marker is consumed by capture_fault and is intentionally live
     * only during execution of that translated instruction. */
    bool fault_rmw;
    /* Translation-time code fetch metadata is live only while decoding. */
    bool fault_fetch_active;
    /* The suspended access was an instruction-pipeline fetch. */
    bool fault_code_fetch;
    /* RTE accepted handler-supplied C/B words for the next translation. */
    bool fault_pipe_accept;
    /* CAS2 may fault between its independent bus cycles.  Retain the
     * completed phase and comparison values so RTE resumes at the failed
     * cycle instead of repeating an observable read/write. */
    uint8_t fault_rmw_phase;
    uint32_t fault_rmw_data1;
    uint32_t fault_rmw_data2;
    bool fault_rmw_data_valid;
    /* Restart state for MOVEP, PMOVE, and FPU multi-cycle accesses. */
    uint8_t fault_special_kind;
    uint8_t fault_special_phase;
    bool fault_special_valid;
    uint32_t fault_special_pc;
    uint32_t fault_special_data[M68K_MMU030_SPECIAL_MAX_CYCLES];
} M68KMMU030State;

typedef struct M68KMMU030ControlState {
    uint64_t crp;
    uint64_t srp;
    uint32_t tc;
    uint32_t tt[2];
} M68KMMU030ControlState;

/* PMOVE extension register identifiers used by the MC68030 decoder. */
typedef enum M68KMMU030PMOVERegister {
    M68K_MMU030_PMOVE_TC,
    M68K_MMU030_PMOVE_TT0,
    M68K_MMU030_PMOVE_TT1,
    M68K_MMU030_PMOVE_SRP,
    M68K_MMU030_PMOVE_CRP,
    M68K_MMU030_PMOVE_MMUSR,
} M68KMMU030PMOVERegister;

/* MC68030 PMMU control-instruction extension fields. */
typedef enum M68KMMU030ControlOperation {
    M68K_MMU030_CONTROL_PLOAD,
    M68K_MMU030_CONTROL_PFLUSH,
    M68K_MMU030_CONTROL_PTEST,
} M68KMMU030ControlOperation;

typedef enum M68KMMU030FunctionCodeSource {
    M68K_MMU030_FC_SFC,
    M68K_MMU030_FC_DFC,
    M68K_MMU030_FC_DREG,
    M68K_MMU030_FC_IMMEDIATE,
} M68KMMU030FunctionCodeSource;

typedef struct M68KMMU030ControlDecode {
    M68KMMU030ControlOperation operation;
    M68KMMU030FunctionCodeSource function_code_source;
    unsigned function_code_value;
    unsigned mode;
    unsigned mask;
    unsigned level;
    unsigned address_register;
    bool is_write;
    bool has_address_register;
} M68KMMU030ControlDecode;

/* Decode the PMOVE extension word described by MC68030 Section 9.3. */
bool m68k_mmu030_pmove_decode(uint16_t extension, unsigned *reg,
                              unsigned *size, bool *direction, bool *fd);

/* Decode an MC68030 PLOAD/PFLUSH/PTEST extension word. */
bool m68k_mmu030_control_decode(uint16_t extension,
                                M68KMMU030ControlDecode *decode);

void m68k_mmu030_reset(M68KMMU030State *state);

/* Helpers shared by translated and C-helper multi-cycle instructions. */
bool m68k_mmu030_special_cycle(M68KMMU030State *state, unsigned kind,
                               unsigned cycle, uint32_t pc,
                               uint32_t data, bool is_write);
void m68k_mmu030_special_record(M68KMMU030State *state, unsigned kind,
                                unsigned cycle, uint32_t pc, uint32_t data,
                                bool is_load);
void m68k_mmu030_special_finish(M68KMMU030State *state, unsigned kind,
                                unsigned cycles, uint32_t pc);
void m68k_mmu030_special_complete(M68KMMU030State *state, unsigned kind,
                                  uint32_t pc);
void m68k_mmu030_special_clear(M68KMMU030State *state);

/* FMOVEM has up to eight operands, each of which is transferred as two or
 * three longwords.  Reuse the four-word special-data image for the current
 * operand and keep the completed-operand count in word three; this keeps the
 * version-6 migration prefix unchanged while retaining every bus phase. */
bool m68k_mmu030_fmovem_cycle(M68KMMU030State *state, unsigned reg,
                              unsigned cycle, uint32_t pc, uint32_t data,
                              bool is_write);
void m68k_mmu030_fmovem_record(M68KMMU030State *state, unsigned reg,
                               unsigned cycle, uint32_t pc, uint32_t data,
                               bool is_load);
void m68k_mmu030_fmovem_finish_register(M68KMMU030State *state,
                                        unsigned reg, unsigned cycles,
                                        uint32_t pc);
void m68k_mmu030_fmovem_finish(M68KMMU030State *state, unsigned regs,
                               uint32_t pc);

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

/* Shared translation path used by the 030 runtime and unit tests. */
int m68k_mmu030_translate_state(
    M68KMMU030State *state, const M68KMMU030MemoryOps *ops,
    uint32_t logical_address, int access_type, uint8_t function_code,
    bool probe, M68KMMU030TranslateResult *result);

int m68k_mmu030_translate(CPUArchState *env, uint32_t logical_address,
                          int access_type, uint8_t function_code, bool probe,
                          M68KMMU030TranslateResult *result);

/* Architectural address-translation-cache operations. */
bool m68k_mmu030_atc_lookup(M68KMMU030State *state,
                            uint32_t logical_address, int access_type,
                            uint8_t function_code,
                            M68KMMU030TranslateResult *result);
void m68k_mmu030_atc_fill(M68KMMU030State *state, uint32_t logical_address,
                          uint8_t function_code,
                          const M68KMMU030TranslateResult *result);
int m68k_mmu030_atc_preload(M68KMMU030State *state,
                            const M68KMMU030MemoryOps *ops,
                            uint32_t logical_address, int access_type,
                            uint8_t function_code,
                            M68KMMU030TranslateResult *result);
int m68k_mmu030_atc_ptest(M68KMMU030State *state,
                          uint32_t logical_address, bool is_write,
                          uint8_t function_code,
                          M68KMMU030TranslateResult *result);

/*
 * Search translation tables for PTEST levels 1..7 without ATC/history side
 * effects.  Architectural status is returned in result and state->mmusr.
 */
int m68k_mmu030_ptest(M68KMMU030State *state,
                      const M68KMMU030MemoryOps *ops,
                      uint32_t logical_address, bool is_write,
                      uint8_t function_code, unsigned level,
                      M68KMMU030TranslateResult *result);
void m68k_mmu030_atc_flush_all(M68KMMU030State *state);
void m68k_mmu030_atc_flush_fc(M68KMMU030State *state,
                              uint8_t function_code,
                              uint8_t function_code_mask);
void m68k_mmu030_atc_flush_page(M68KMMU030State *state,
                                uint32_t logical_address,
                                uint8_t function_code,
                                uint8_t function_code_mask);

typedef void (*M68KMMU030ATCFlushAllFn)(void *opaque);
typedef void (*M68KMMU030ATCFlushRangeFn)(void *opaque, uint32_t address,
                                         uint32_t size);

/* Update a TT without touching architectural ATC entries. */
bool m68k_mmu030_write_tt(M68KMMU030State *state, unsigned index,
                          uint32_t value,
                          M68KMMU030ATCFlushAllFn flush_derived,
                          void *opaque);

/* Flush an architectural scope and its derived QEMU TLB through callbacks. */
void m68k_mmu030_atc_flush_all_coherent(
    M68KMMU030State *state, M68KMMU030ATCFlushAllFn flush_all,
    void *opaque);
void m68k_mmu030_atc_flush_fc_coherent(
    M68KMMU030State *state, uint8_t function_code,
    uint8_t function_code_mask, M68KMMU030ATCFlushAllFn flush_all,
    void *opaque);
void m68k_mmu030_atc_flush_page_coherent(
    M68KMMU030State *state, uint32_t logical_address,
    uint8_t function_code, uint8_t function_code_mask,
    M68KMMU030ATCFlushRangeFn flush_range, void *opaque);
bool m68k_mmu030_reconfigure(
    M68KMMU030State *state, const M68KMMU030ControlState *control,
    bool flush, M68KMMU030ATCFlushAllFn flush_all, void *opaque);

uint16_t m68k_mmu030_make_ssw(unsigned size, bool is_write, bool is_code,
                              uint8_t function_code);

uint32_t m68k_mmu030_rte_frame_tail_size(uint16_t format);

/*
 * Track code words fetched by the translator without issuing speculative
 * reads beyond the instruction currently being decoded.
 */
void m68k_mmu030_begin_instruction_fetch(M68KMMU030State *state,
                                         uint32_t instruction_pc);
void m68k_mmu030_record_instruction_fetch(M68KMMU030State *state,
                                          uint32_t instruction_pc,
                                          uint32_t fetch_address,
                                          uint16_t word);
void m68k_mmu030_end_instruction_fetch(M68KMMU030State *state);

/* Latch one failed CPU access for exception entry. */
void m68k_mmu030_capture_fault(
    M68KMMU030State *state, uint32_t logical_address, uint32_t fault_pc,
    unsigned size, bool is_write, bool is_code, uint8_t function_code,
    const M68KMMU030TranslateResult *result);

/* Return the exact ordinary access-error frame size selected by the latch. */
uint32_t m68k_mmu030_access_frame_size(const M68KMMU030State *state);

/*
 * Build an architectural format A or B frame.  The caller supplies storage
 * for exactly the selected frame size.  Building a frame consumes the
 * pending fault context, just as exception entry consumes the core's fault
 * latch on a physical MC68030.
 */
bool m68k_mmu030_build_access_frame(
    M68KMMU030State *state, uint16_t saved_sr, uint16_t vector_offset,
    uint8_t *frame, size_t frame_size);

/* Restore the internal restart image from a format A/B frame during RTE. */
bool m68k_mmu030_restore_access_frame(
    M68KMMU030State *state, const uint8_t *frame, size_t frame_size,
    uint16_t *saved_sr, uint32_t *resume_pc);

/* Restore the common state from a format-$9 coprocessor frame. */
bool m68k_mmu030_restore_coprocessor_frame(
    M68KMMU030State *state, const uint8_t *frame, size_t frame_size,
    uint16_t *saved_sr, uint32_t *resume_pc);

/* Consume the one rerun decision restored by RTE. */
bool m68k_mmu030_consume_restart(M68KMMU030State *state);

bool m68k_mmu030_build_short_access_frame(
    M68KMMU030State *state, uint16_t saved_sr, uint16_t vector_offset,
    uint8_t frame[M68K_MMU030_ACCESS_FRAME_SIZE]);

extern const VMStateDescription vmstate_mmu030_state;

#endif /* M68K_MMU030_H */
