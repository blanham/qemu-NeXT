/*
 *  M68K helper routines
 *
 *  Copyright (c) 2007 CodeSourcery
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "exec/cputlb.h"
#include "exec/target_page.h"
#include "accel/tcg/cpu-ldst.h"
#include "accel/tcg/cpu-loop.h"
#include "qemu/bswap.h"
#include "semihosting/semihost.h"
#include "qemu/plugin.h"

#if !defined(CONFIG_USER_ONLY)

static G_NORETURN void raise_exception_ra(CPUM68KState *env, int tt,
                                           uintptr_t raddr);

/* A second bus fault while the 68030's access frame is live cannot be
 * represented by another exception frame.  The physical processor halts
 * with the original frame state intact. */
static G_NORETURN void m68k_mmu030_double_fault(CPUState *cs)
{
    qemu_log_mask(CPU_LOG_INT, "MC68030 double access fault; CPU halted\n");
    cs->halted = 1;
    cs->exception_index = EXCP_HLT;
    cpu_loop_exit(cs);
}

static void m68k_mmu030_consume_data_input(M68KMMU030State *state)
{
    state->fault_data_input = 0;
    state->fault_data_input_address = 0;
    state->fault_data_input_valid = false;
    state->fault_data_complete = false;
}

static G_NORETURN void m68k_rte_format_error(CPUM68KState *env,
                                             uint32_t rte_pc)
{
    /* The attempted frame must remain on the stack for the format handler. */
    env->pc = rte_pc;
    raise_exception_ra(env, EXCP_FORMAT, 0);
}

static void m68k_rte_complete_access(CPUM68KState *env,
                                     uint32_t fault_pc,
                                     uint32_t resume_pc,
                                     uint32_t restored_pc)
{
    uint16_t ssw = env->mmu030.fault_ssw;
    bool rerun;
    bool special_valid = env->mmu030.fault_special_valid;
    uint32_t special_pc = env->mmu030.fault_special_pc;

    /* A set DF/RC/RB bit transfers ownership of the next bus cycle to RTE.
     * Consume that decision here: an already repaired frame must not leave a
     * stale restart request for a later exception. */
    rerun = m68k_mmu030_consume_restart(&env->mmu030);
    if (rerun) {
        tlb_flush_page(env_cpu(env), env->mmu030.fault_address);
        env->mmu030.fault_data_complete = false;
        env->mmu030.fault_data_input_valid = false;
        env->mmu030.fault_data_write = false;
        env->mmu030.fault_rmw = false;
        /* RM cycles may have completed earlier CAS2 phases.  Preserve their
         * phase/data latch across RTE so the retry starts at the failed bus
         * cycle rather than repeating reads or the first write. */
        if (!(ssw & M68K_MMU030_SSW_RM) &&
            !env->mmu030.fault_special_valid) {
            env->mmu030.fault_rmw_phase = 0;
            env->mmu030.fault_rmw_data1 = 0;
            env->mmu030.fault_rmw_data2 = 0;
            env->mmu030.fault_rmw_data_valid = false;
        }
        env->mmu030.fault_data_input_address = 0;
        env->mmu030.fault_resume_pc = 0;
        env->mmu030.fault_pipe_accept = false;
    } else if (env->mmu030.fault_code_fetch &&
               restored_pc == fault_pc &&
               !(ssw & (M68K_MMU030_SSW_RC | M68K_MMU030_SSW_RB))) {
        /*
         * RC/RB cleared means the handler accepted the saved instruction
         * pipeline.  Keep the edited C/B words for the translator's next
         * decode; re-fetching them from memory would discard the frame's
         * repaired image.
         */
        env->mmu030.fault_pipe_accept = true;
        env->mmu030.fault_data_complete = false;
        env->mmu030.fault_data_input_valid = false;
        env->mmu030.fault_data_write = false;
        env->mmu030.fault_rmw = false;
        env->mmu030.fault_rmw_phase = 0;
        env->mmu030.fault_rmw_data1 = 0;
        env->mmu030.fault_rmw_data2 = 0;
        env->mmu030.fault_rmw_data_valid = false;
        m68k_mmu030_special_clear(&env->mmu030);
        env->mmu030.fault_data_input_address = 0;
        env->mmu030.fault_resume_pc = 0;
    } else if (resume_pc && restored_pc == fault_pc &&
               !(ssw & (M68K_MMU030_SSW_DF |
                        M68K_MMU030_SSW_RC |
                        M68K_MMU030_SSW_RB))) {
        /* DF was cleared by the handler.  Re-enter the faulting instruction
         * at its saved PC, but let its translated load consume DIB and its
         * store consume DOB without repeating an already completed bus
         * cycle.  This preserves the instruction's arithmetic and flags. */
        env->mmu030.fault_data_complete = true;
        env->mmu030.fault_data_write = !(ssw & M68K_MMU030_SSW_RW);
        if (special_valid &&
            (env->mmu030.fault_special_kind == M68K_MMU030_SPECIAL_SPLIT ||
             env->mmu030.fault_special_kind == M68K_MMU030_SPECIAL_FMOVEM)) {
            /*
             * The special-cycle helper owns individual fragments.  Mark the
             * failed fragment consumed so a DF-cleared frame cannot replay a
             * committed prefix when the original EA is retried.
             */
            m68k_mmu030_special_complete(
                &env->mmu030, env->mmu030.fault_special_kind, special_pc);
        } else {
            m68k_mmu030_special_clear(&env->mmu030);
        }
    } else {
        /* QEMU does not retain the MC68030's B/C prefetch registers.  When
         * RC/RB are cleared, restore_access_frame has consumed the handler's
         * stack images and the next TB lookup re-fetches from env->pc; this
         * is the equivalent accepted-pipeline state for the translator. */
        env->mmu030.fault_data_complete = false;
        env->mmu030.fault_data_input_valid = false;
        env->mmu030.fault_data_write = false;
        env->mmu030.fault_rmw = false;
        env->mmu030.fault_rmw_phase = 0;
        env->mmu030.fault_rmw_data1 = 0;
        env->mmu030.fault_rmw_data2 = 0;
        env->mmu030.fault_rmw_data_valid = false;
        m68k_mmu030_special_clear(&env->mmu030);
        env->mmu030.fault_data_input_address = 0;
        env->mmu030.fault_resume_pc = 0;
    }
}

static void cf_rte(CPUM68KState *env)
{
    uint32_t sp;
    uint32_t fmt;

    sp = env->aregs[7];
    fmt = cpu_ldl_be_mmuidx_ra(env, sp, MMU_KERNEL_IDX, 0);
    env->pc = cpu_ldl_be_mmuidx_ra(env, sp + 4, MMU_KERNEL_IDX, 0);
    sp |= (fmt >> 28) & 3;
    env->aregs[7] = sp + 8;

    cpu_m68k_set_sr(env, fmt);
}

static void m68k_rte(CPUM68KState *env)
{
    uint32_t sp;
    uint32_t frame_start;
    uint32_t rte_pc;
    uint16_t fmt;
    uint16_t sr;

    sp = env->aregs[7];
    rte_pc = env->pc;
throwaway:
    frame_start = sp;
    sr = cpu_lduw_be_mmuidx_ra(env, sp, MMU_KERNEL_IDX, 0);
    sp += 2;
    env->pc = cpu_ldl_be_mmuidx_ra(env, sp, MMU_KERNEL_IDX, 0);
    sp += 4;
    if (m68k_feature(env, M68K_FEATURE_EXCEPTION_FORMAT_VEC)) {
        /*  all except 68000 */
        fmt = cpu_lduw_be_mmuidx_ra(env, sp, MMU_KERNEL_IDX, 0);
        sp += 2;
        switch (fmt >> 12) {
        case 0:
            if (m68k_feature(env, M68K_FEATURE_M68030)) {
                m68k_mmu030_pop_collapsed_fault_frame(&env->mmu030,
                                                       frame_start, true);
            }
            break;
        case 1:
            env->aregs[7] = sp;
            cpu_m68k_set_sr(env, sr);
            goto throwaway;
        case 2:
        case 3:
            sp += 4;
            break;
        case 4:
            sp += 8;
            break;
        case 7:
            sp += 52;
            break;
        case 0xa:
            if (m68k_feature(env, M68K_FEATURE_M68030)) {
                uint8_t frame[M68K_MMU030_ACCESS_FRAME_SIZE_LONG];
                uint32_t frame_size =
                    M68K_MMU030_ACCESS_FRAME_SIZE_SHORT;
                uint16_t restored_sr;
                uint32_t restored_pc;
                uint32_t fault_pc = 0;
                uint32_t resume_pc = 0;
                bool frame_owned =
                    m68k_mmu030_restore_fault_frame_context(
                        &env->mmu030, frame_start, frame_size);
                bool legacy_owned =
                    !frame_owned &&
                    m68k_mmu030_legacy_fault_frame_active(&env->mmu030);

                if (frame_owned || legacy_owned) {
                    /* Save the original continuation before reading the
                     * handler-editable frame image.  An owned frame's
                     * architectural PC may have been deliberately changed
                     * by the handler. */
                    fault_pc = env->mmu030.fault_pc;
                    resume_pc = env->mmu030.fault_resume_pc;
                }

                for (uint32_t offset = 0; offset < frame_size; offset += 2) {
                    stw_be_p(frame + offset,
                             cpu_lduw_be_mmuidx_ra(env, frame_start + offset,
                                                   MMU_KERNEL_IDX, 0));
                }
                if (!frame_owned && !legacy_owned) {
                    fault_pc = ldl_be_p(frame + 0x02);
                }
                if (!m68k_mmu030_restore_access_frame(
                        &env->mmu030, frame, frame_size,
                        &restored_sr, &restored_pc)) {
                    m68k_rte_format_error(env, rte_pc);
                }
                sr = restored_sr;
                env->pc = restored_pc;
                m68k_rte_complete_access(env, fault_pc, resume_pc,
                                         restored_pc);
                if (frame_owned) {
                    m68k_mmu030_pop_fault_frame(&env->mmu030,
                                                frame_start, frame_size,
                                                false);
                } else if (legacy_owned) {
                    m68k_mmu030_clear_legacy_fault_frame(&env->mmu030);
                }
                sp = frame_start + frame_size;
            }
            break;
        case 0xb:
            if (m68k_feature(env, M68K_FEATURE_M68030)) {
                uint8_t frame[M68K_MMU030_ACCESS_FRAME_SIZE_LONG];
                uint32_t frame_size =
                    M68K_MMU030_ACCESS_FRAME_SIZE_LONG;
                uint16_t restored_sr;
                uint32_t restored_pc;
                uint32_t fault_pc = 0;
                uint32_t resume_pc = 0;
                bool frame_owned =
                    m68k_mmu030_restore_fault_frame_context(
                        &env->mmu030, frame_start, frame_size);
                bool legacy_owned =
                    !frame_owned &&
                    m68k_mmu030_legacy_fault_frame_active(&env->mmu030);

                if (frame_owned || legacy_owned) {
                    /* Preserve the original scalar continuation before the
                     * handler can edit the frame's saved PC. */
                    fault_pc = env->mmu030.fault_pc;
                    resume_pc = env->mmu030.fault_resume_pc;
                }

                for (uint32_t offset = 0; offset < frame_size; offset += 2) {
                    stw_be_p(frame + offset,
                             cpu_lduw_be_mmuidx_ra(env, frame_start + offset,
                                                   MMU_KERNEL_IDX, 0));
                }
                if (!frame_owned && !legacy_owned) {
                    fault_pc = ldl_be_p(frame + 0x02);
                }
                if (!m68k_mmu030_restore_access_frame(
                        &env->mmu030, frame, frame_size,
                        &restored_sr, &restored_pc)) {
                    m68k_rte_format_error(env, rte_pc);
                }
                sr = restored_sr;
                env->pc = restored_pc;
                m68k_rte_complete_access(env, fault_pc, resume_pc,
                                         restored_pc);
                if (frame_owned) {
                    m68k_mmu030_pop_fault_frame(&env->mmu030,
                                                frame_start, frame_size,
                                                false);
                } else if (legacy_owned) {
                    m68k_mmu030_clear_legacy_fault_frame(&env->mmu030);
                }
                sp = frame_start + frame_size;
            }
            break;
        case 9:
            if (m68k_feature(env, M68K_FEATURE_M68030)) {
                uint8_t frame[M68K_MMU030_COPROCESSOR_FRAME_SIZE];
                uint32_t frame_size = M68K_MMU030_COPROCESSOR_FRAME_SIZE;
                uint16_t restored_sr;
                uint32_t restored_pc;
                bool frame_owned =
                    m68k_mmu030_restore_fault_frame_context(
                        &env->mmu030, frame_start, frame_size);
                for (uint32_t offset = 0; offset < frame_size; offset += 2) {
                    stw_be_p(frame + offset,
                             cpu_lduw_be_mmuidx_ra(env, frame_start + offset,
                                                   MMU_KERNEL_IDX, 0));
                }
                if (!m68k_mmu030_restore_coprocessor_frame(
                        &env->mmu030, frame, frame_size,
                        &restored_sr, &restored_pc)) {
                    m68k_rte_format_error(env, rte_pc);
                }
                sr = restored_sr;
                env->pc = restored_pc;
                if (frame_owned) {
                    m68k_mmu030_pop_fault_frame(&env->mmu030,
                                                frame_start, frame_size,
                                                false);
                }
                sp = frame_start + frame_size;
            }
            break;
        default:
            if (m68k_feature(env, M68K_FEATURE_M68030)) {
                m68k_rte_format_error(env, rte_pc);
            }
            break;
        }
    }
    env->aregs[7] = sp;
    cpu_m68k_set_sr(env, sr);
}

static const char *m68k_exception_name(int index)
{
    switch (index) {
    case EXCP_ACCESS:
        return "Access Fault";
    case EXCP_ADDRESS:
        return "Address Error";
    case EXCP_ILLEGAL:
        return "Illegal Instruction";
    case EXCP_DIV0:
        return "Divide by Zero";
    case EXCP_CHK:
        return "CHK/CHK2";
    case EXCP_TRAPCC:
        return "FTRAPcc, TRAPcc, TRAPV";
    case EXCP_PRIVILEGE:
        return "Privilege Violation";
    case EXCP_TRACE:
        return "Trace";
    case EXCP_LINEA:
        return "A-Line";
    case EXCP_LINEF:
        return "F-Line";
    case EXCP_DEBEGBP: /* 68020/030 only */
        return "Copro Protocol Violation";
    case EXCP_FORMAT:
        return "Format Error";
    case EXCP_UNINITIALIZED:
        return "Uninitialized Interrupt";
    case EXCP_SPURIOUS:
        return "Spurious Interrupt";
    case EXCP_INT_LEVEL_1:
        return "Level 1 Interrupt";
    case EXCP_INT_LEVEL_1 + 1:
        return "Level 2 Interrupt";
    case EXCP_INT_LEVEL_1 + 2:
        return "Level 3 Interrupt";
    case EXCP_INT_LEVEL_1 + 3:
        return "Level 4 Interrupt";
    case EXCP_INT_LEVEL_1 + 4:
        return "Level 5 Interrupt";
    case EXCP_INT_LEVEL_1 + 5:
        return "Level 6 Interrupt";
    case EXCP_INT_LEVEL_1 + 6:
        return "Level 7 Interrupt";
    case EXCP_TRAP0:
        return "TRAP #0";
    case EXCP_TRAP0 + 1:
        return "TRAP #1";
    case EXCP_TRAP0 + 2:
        return "TRAP #2";
    case EXCP_TRAP0 + 3:
        return "TRAP #3";
    case EXCP_TRAP0 + 4:
        return "TRAP #4";
    case EXCP_TRAP0 + 5:
        return "TRAP #5";
    case EXCP_TRAP0 + 6:
        return "TRAP #6";
    case EXCP_TRAP0 + 7:
        return "TRAP #7";
    case EXCP_TRAP0 + 8:
        return "TRAP #8";
    case EXCP_TRAP0 + 9:
        return "TRAP #9";
    case EXCP_TRAP0 + 10:
        return "TRAP #10";
    case EXCP_TRAP0 + 11:
        return "TRAP #11";
    case EXCP_TRAP0 + 12:
        return "TRAP #12";
    case EXCP_TRAP0 + 13:
        return "TRAP #13";
    case EXCP_TRAP0 + 14:
        return "TRAP #14";
    case EXCP_TRAP0 + 15:
        return "TRAP #15";
    case EXCP_FP_BSUN:
        return "FP Branch/Set on unordered condition";
    case EXCP_FP_INEX:
        return "FP Inexact Result";
    case EXCP_FP_DZ:
        return "FP Divide by Zero";
    case EXCP_FP_UNFL:
        return "FP Underflow";
    case EXCP_FP_OPERR:
        return "FP Operand Error";
    case EXCP_FP_OVFL:
        return "FP Overflow";
    case EXCP_FP_SNAN:
        return "FP Signaling NAN";
    case EXCP_FP_UNIMP:
        return "FP Unimplemented Data Type";
    case EXCP_MMU_CONF: /* 68030/68851 only */
        return "MMU Configuration Error";
    case EXCP_MMU_ILLEGAL: /* 68851 only */
        return "MMU Illegal Operation";
    case EXCP_MMU_ACCESS: /* 68851 only */
        return "MMU Access Level Violation";
    case 64 ... 255:
        return "User Defined Vector";
    }
    return "Unassigned";
}

static void do_plugin_vcpu_interrupt_cb(CPUState *cs, uint64_t from)
{
    switch (cs->exception_index) {
    case EXCP_SPURIOUS ... EXCP_INT_LEVEL_7:
        qemu_plugin_vcpu_interrupt_cb(cs, from);
        break;
    case EXCP_SEMIHOSTING:
        qemu_plugin_vcpu_hostcall_cb(cs, from);
        break;
    default:
        qemu_plugin_vcpu_exception_cb(cs, from);
        break;
    }
}

static void cf_interrupt_all(CPUM68KState *env, int is_hw)
{
    CPUState *cs = env_cpu(env);
    uint32_t sp;
    uint32_t sr;
    uint32_t fmt;
    uint32_t retaddr;
    uint32_t vector;

    fmt = 0;
    retaddr = env->pc;

    if (!is_hw) {
        switch (cs->exception_index) {
        case EXCP_RTE:
            /* Return from an exception.  */
            cf_rte(env);
            return;
        case EXCP_SEMIHOSTING:
            do_m68k_semihosting(env, env->dregs[0]);
            qemu_plugin_vcpu_hostcall_cb(cs, retaddr);
            return;
        }
    }

    vector = cs->exception_index << 2;

    sr = env->sr | cpu_m68k_get_ccr(env);
    if (qemu_loglevel_mask(CPU_LOG_INT)) {
        static int count;
        qemu_log("INT %6d: %s(%#x) pc=%08x sp=%08x sr=%04x\n",
                 ++count, m68k_exception_name(cs->exception_index),
                 vector, env->pc, env->aregs[7], sr);
    }

    fmt |= 0x40000000;
    fmt |= vector << 16;
    fmt |= sr;

    env->sr |= SR_S;
    if (is_hw) {
        env->sr = (env->sr & ~SR_I) | (env->pending_level << SR_I_SHIFT);
        env->sr &= ~SR_M;
    }
    m68k_switch_sp(env);
    sp = env->aregs[7];
    fmt |= (sp & 3) << 28;

    /* ??? This could cause MMU faults.  */
    sp &= ~3;
    sp -= 4;
    cpu_stl_be_mmuidx_ra(env, sp, retaddr, MMU_KERNEL_IDX, 0);
    sp -= 4;
    cpu_stl_be_mmuidx_ra(env, sp, fmt, MMU_KERNEL_IDX, 0);
    env->aregs[7] = sp;
    /* Jump to vector.  */
    env->pc = cpu_ldl_be_mmuidx_ra(env, env->vbr + vector, MMU_KERNEL_IDX, 0);

    do_plugin_vcpu_interrupt_cb(cs, retaddr);
}

static inline void do_stack_frame(CPUM68KState *env, uint32_t *sp,
                                  uint16_t format, uint16_t sr,
                                  uint32_t addr, uint32_t retaddr)
{
    if (m68k_feature(env, M68K_FEATURE_EXCEPTION_FORMAT_VEC)) {
        /*  all except 68000 */
        CPUState *cs = env_cpu(env);
        switch (format) {
        case 4:
            *sp -= 4;
            cpu_stl_be_mmuidx_ra(env, *sp, env->pc, MMU_KERNEL_IDX, 0);
            *sp -= 4;
            cpu_stl_be_mmuidx_ra(env, *sp, addr, MMU_KERNEL_IDX, 0);
            break;
        case 3:
        case 2:
            *sp -= 4;
            cpu_stl_be_mmuidx_ra(env, *sp, addr, MMU_KERNEL_IDX, 0);
            break;
        }
        *sp -= 2;
        cpu_stw_be_mmuidx_ra(env, *sp,
                             (format << 12) + (cs->exception_index << 2),
                             MMU_KERNEL_IDX, 0);
    }
    *sp -= 4;
    cpu_stl_be_mmuidx_ra(env, *sp, retaddr, MMU_KERNEL_IDX, 0);
    *sp -= 2;
    cpu_stw_be_mmuidx_ra(env, *sp, sr, MMU_KERNEL_IDX, 0);
}

static void m68k_mmu030_access_error_frame(CPUM68KState *env, uint32_t *sp,
                                           uint16_t saved_sr,
                                           uint32_t vector)
{
    uint8_t frame[M68K_MMU030_ACCESS_FRAME_SIZE_LONG];
    uint32_t frame_size;
    M68KMMU030FaultFramePushResult push_result;

    if (env->mmu030.fault_exception_processing ||
        m68k_mmu030_legacy_fault_frame_active(&env->mmu030)) {
        m68k_mmu030_double_fault(env_cpu(env));
    }

    env->mmu030.fault_exception_processing = true;

    /*
     * The 68030 has two ordinary bus-cycle fault frames.  The latched fault
     * selects format A at an instruction boundary and format B while an
     * instruction/data cycle is in progress; the frame builder also handles
     * legacy callers which only populated the Task 3 fields.
     */
    frame_size = m68k_mmu030_access_frame_size(&env->mmu030);
    if (!frame_size) {
        cpu_abort(env_cpu(env),
                  "unsupported MC68030 access fault frame format %u\n",
                  env->mmu030.fault_format);
    }
    if (!m68k_mmu030_build_access_frame(&env->mmu030, saved_sr, vector,
                                        frame, frame_size)) {
        cpu_abort(env_cpu(env),
                  "MC68030 access fault without pending MMU context\n");
    }

    *sp -= frame_size;
    for (uint32_t offset = 0; offset < frame_size; offset += 2) {
        cpu_stw_be_mmuidx_ra(env, *sp + offset, lduw_be_p(frame + offset),
                             MMU_KERNEL_IDX, 0);
    }
    push_result = m68k_mmu030_push_fault_frame(&env->mmu030, *sp,
                                               frame_size);
    if (push_result == M68K_MMU030_FAULT_FRAME_PUSH_CAPACITY) {
        /* The architectural frame is already complete.  A full bounded
         * sidecar only loses hidden restart fidelity; it is not another CPU
         * bus fault and must not halt the processor. */
        m68k_mmu030_reset_fault_scratch(&env->mmu030);
    } else if (push_result == M68K_MMU030_FAULT_FRAME_PUSH_CONFLICT) {
        /* Reusing an exact or collapsed identity would let this frame steal
         * an older frame's hidden restart image on RTE. */
        m68k_mmu030_double_fault(env_cpu(env));
    }
}

static void m68k_interrupt_all(CPUM68KState *env, int is_hw)
{
    CPUState *cs = env_cpu(env);
    uint32_t sp;
    uint32_t vector;
    uint16_t sr, oldsr;
    uint64_t last_pc = env->pc;

    if (m68k_feature(env, M68K_FEATURE_M68030) &&
        (env->mmu030.fault_exception_processing ||
         m68k_mmu030_legacy_fault_frame_active(&env->mmu030)) &&
        (cs->exception_index == EXCP_ACCESS ||
         cs->exception_index == EXCP_ADDRESS)) {
        m68k_mmu030_double_fault(cs);
    }

    if (!is_hw) {
        switch (cs->exception_index) {
        case EXCP_RTE:
            /* Return from an exception.  */
            m68k_rte(env);
            return;
        }
    }

    vector = cs->exception_index << 2;

    sr = env->sr | cpu_m68k_get_ccr(env);
    if (qemu_loglevel_mask(CPU_LOG_INT)) {
        static int count;
        qemu_log("INT %6d: %s(%#x) pc=%08x sp=%08x sr=%04x\n",
                 ++count, m68k_exception_name(cs->exception_index),
                 vector, env->pc, env->aregs[7], sr);
    }

    /*
     * MC68040UM/AD,  chapter 9.3.10
     */

    /* "the processor first make an internal copy" */
    oldsr = sr;
    /* "set the mode to supervisor" */
    sr |= SR_S;
    /* "suppress tracing" */
    sr &= ~SR_T;
    /* "sets the processor interrupt mask" */
    if (is_hw) {
        sr |= (env->sr & ~SR_I) | (env->pending_level << SR_I_SHIFT);
    }
    cpu_m68k_set_sr(env, sr);
    sp = env->aregs[7];

    if (!m68k_feature(env, M68K_FEATURE_UNALIGNED_DATA)) {
        sp &= ~1;
    }

    switch (cs->exception_index) {
    case EXCP_ACCESS:
        if (m68k_feature(env, M68K_FEATURE_M68030)) {
            m68k_mmu030_access_error_frame(env, &sp, oldsr, vector);
            break;
        }
        if (env->mmu.fault) {
            cpu_abort(cs, "DOUBLE MMU FAULT\n");
        }
        env->mmu.fault = true;
        /* push data 3 */
        sp -= 4;
        cpu_stl_be_mmuidx_ra(env, sp, 0, MMU_KERNEL_IDX, 0);
        /* push data 2 */
        sp -= 4;
        cpu_stl_be_mmuidx_ra(env, sp, 0, MMU_KERNEL_IDX, 0);
        /* push data 1 */
        sp -= 4;
        cpu_stl_be_mmuidx_ra(env, sp, 0, MMU_KERNEL_IDX, 0);
        /* write back 1 / push data 0 */
        sp -= 4;
        cpu_stl_be_mmuidx_ra(env, sp, 0, MMU_KERNEL_IDX, 0);
        /* write back 1 address */
        sp -= 4;
        cpu_stl_be_mmuidx_ra(env, sp, 0, MMU_KERNEL_IDX, 0);
        /* write back 2 data */
        sp -= 4;
        cpu_stl_be_mmuidx_ra(env, sp, 0, MMU_KERNEL_IDX, 0);
        /* write back 2 address */
        sp -= 4;
        cpu_stl_be_mmuidx_ra(env, sp, 0, MMU_KERNEL_IDX, 0);
        /* write back 3 data */
        sp -= 4;
        cpu_stl_be_mmuidx_ra(env, sp, 0, MMU_KERNEL_IDX, 0);
        /* write back 3 address */
        sp -= 4;
        cpu_stl_be_mmuidx_ra(env, sp, env->mmu.ar, MMU_KERNEL_IDX, 0);
        /* fault address */
        sp -= 4;
        cpu_stl_be_mmuidx_ra(env, sp, env->mmu.ar, MMU_KERNEL_IDX, 0);
        /* write back 1 status */
        sp -= 2;
        cpu_stw_be_mmuidx_ra(env, sp, 0, MMU_KERNEL_IDX, 0);
        /* write back 2 status */
        sp -= 2;
        cpu_stw_be_mmuidx_ra(env, sp, 0, MMU_KERNEL_IDX, 0);
        /* write back 3 status */
        sp -= 2;
        cpu_stw_be_mmuidx_ra(env, sp, 0, MMU_KERNEL_IDX, 0);
        /* special status word */
        sp -= 2;
        cpu_stw_be_mmuidx_ra(env, sp, env->mmu.ssw, MMU_KERNEL_IDX, 0);
        /* effective address */
        sp -= 4;
        cpu_stl_be_mmuidx_ra(env, sp, env->mmu.ar, MMU_KERNEL_IDX, 0);

        do_stack_frame(env, &sp, 7, oldsr, 0, env->pc);
        env->mmu.fault = false;
        if (qemu_loglevel_mask(CPU_LOG_INT)) {
            qemu_log("            "
                     "ssw:  %08x ea:   %08x sfc:  %d    dfc: %d\n",
                     env->mmu.ssw, env->mmu.ar, env->sfc, env->dfc);
        }
        break;

    case EXCP_ILLEGAL:
        do_stack_frame(env, &sp, 0, oldsr, 0, env->pc);
        break;

    case EXCP_ADDRESS:
        do_stack_frame(env, &sp, 2, oldsr, 0, env->pc);
        break;

    case EXCP_CHK:
    case EXCP_DIV0:
    case EXCP_TRACE:
    case EXCP_TRAPCC:
        do_stack_frame(env, &sp, 2, oldsr, env->mmu.ar, env->pc);
        break;

    case EXCP_SPURIOUS ... EXCP_INT_LEVEL_7:
        if (is_hw && (oldsr & SR_M)) {
            do_stack_frame(env, &sp, 0, oldsr, 0, env->pc);
            oldsr = sr;
            env->aregs[7] = sp;
            cpu_m68k_set_sr(env, sr & ~SR_M);
            sp = env->aregs[7];
            if (!m68k_feature(env, M68K_FEATURE_UNALIGNED_DATA)) {
                sp &= ~1;
            }
            do_stack_frame(env, &sp, 1, oldsr, 0, env->pc);
            break;
        }
        /* fall through */

    default:
        do_stack_frame(env, &sp, 0, oldsr, 0, env->pc);
        break;
    }

    env->aregs[7] = sp;
    /* Jump to vector.  */
    env->pc = cpu_ldl_be_mmuidx_ra(env, env->vbr + vector, MMU_KERNEL_IDX, 0);
    if (m68k_feature(env, M68K_FEATURE_M68030) &&
        env->mmu030.fault_exception_processing) {
        env->mmu030.fault_exception_prefetch_words = 0;
        m68k_mmu030_reset_fault_scratch(&env->mmu030);
        /*
         * Exception processing ends only after the processor has fetched
         * the first three words at the handler PC.  Do this synchronously:
         * a previously translated handler TB would otherwise bypass the
         * translator-time fetch bookkeeping and leave the double-fault
         * guard asserted indefinitely.
         */
        for (unsigned int i = 0; i < 3; i++) {
            cpu_lduw_be_mmuidx_ra(env, env->pc + i * 2,
                                  MMU_KERNEL_IDX, 0);
            env->mmu030.fault_exception_prefetch_words++;
        }
        env->mmu030.fault_exception_processing = false;
    }

    do_plugin_vcpu_interrupt_cb(cs, last_pc);
}

static void do_interrupt_all(CPUM68KState *env, int is_hw)
{
    if (m68k_feature(env, M68K_FEATURE_M68K)) {
        m68k_interrupt_all(env, is_hw);
        return;
    }
    cf_interrupt_all(env, is_hw);
}

void m68k_cpu_do_interrupt(CPUState *cs)
{
    do_interrupt_all(cpu_env(cs), 0);
}

static inline void do_interrupt_m68k_hardirq(CPUM68KState *env)
{
    do_interrupt_all(env, 1);
}

void m68k_cpu_transaction_failed(CPUState *cs, hwaddr physaddr, vaddr addr,
                                 unsigned size, MMUAccessType access_type,
                                 int mmu_idx, MemTxAttrs attrs,
                                 MemTxResult response, uintptr_t retaddr)
{
    CPUM68KState *env = cpu_env(cs);

    if (m68k_feature(env, M68K_FEATURE_M68030) &&
        (env->mmu030.fault_exception_processing ||
         m68k_mmu030_legacy_fault_frame_active(&env->mmu030))) {
        m68k_mmu030_double_fault(cs);
    }

    cpu_restore_state(cs, retaddr);

    if (m68k_feature(env, M68K_FEATURE_M68040)) {
        env->mmu.mmusr = 0;

        /*
         * According to the MC68040 users manual the ATC bit of the SSW is
         * used to distinguish between ATC faults and physical bus errors.
         * In the case of a bus error e.g. during nubus read from an empty
         * slot this bit should not be set
         */
        if (response != MEMTX_DECODE_ERROR) {
            env->mmu.ssw |= M68K_ATC_040;
        }

        /* FIXME: manage MMU table access error */
        env->mmu.ssw &= ~M68K_TM_040;
        if (env->sr & SR_S) { /* SUPERVISOR */
            env->mmu.ssw |= M68K_TM_040_SUPER;
        }
        if (access_type == MMU_INST_FETCH) { /* instruction or data */
            env->mmu.ssw |= M68K_TM_040_CODE;
        } else {
            env->mmu.ssw |= M68K_TM_040_DATA;
        }
        env->mmu.ssw &= ~M68K_BA_SIZE_MASK;
        switch (size) {
        case 1:
            env->mmu.ssw |= M68K_BA_SIZE_BYTE;
            break;
        case 2:
            env->mmu.ssw |= M68K_BA_SIZE_WORD;
            break;
        case 4:
            env->mmu.ssw |= M68K_BA_SIZE_LONG;
            break;
        }

        if (access_type != MMU_DATA_STORE) {
            env->mmu.ssw |= M68K_RW_040;
        }

        env->mmu.ar = addr;

        cs->exception_index = EXCP_ACCESS;
        cpu_loop_exit(cs);
    }

    if (m68k_feature(env, M68K_FEATURE_M68030)) {
        M68KMMU030TranslateResult result = {
            .fault = true,
            .bus_error = true,
        };
        bool is_code = access_type == MMU_INST_FETCH;
        bool is_write = access_type == MMU_DATA_STORE;
        uint8_t function_code = (mmu_idx != MMU_USER_IDX ? 4 : 0) |
                                (is_code ? 2 : 1);

        /*
         * A transaction failure after a successful MMU translation is the
         * physical data/instruction bus cycle itself.  Table-walk failures
         * are reported by m68k_mmu030_translate() with their descriptor
         * address and never reach this callback (the walker uses direct
         * AddressSpace accesses), so do not manufacture MMUSR table bits
         * here.
         */
        m68k_mmu030_capture_fault(
            &env->mmu030, addr, env->pc, size, is_write, is_code,
            function_code, &result);
        cs->exception_index = EXCP_ACCESS;
        cpu_loop_exit(cs);
    }
}

bool m68k_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    CPUM68KState *env = cpu_env(cs);

    if (interrupt_request & CPU_INTERRUPT_HARD
        && ((env->sr & SR_I) >> SR_I_SHIFT) < env->pending_level) {
        /*
         * Real hardware gets the interrupt vector via an IACK cycle
         * at this point.  Current emulated hardware doesn't rely on
         * this, so we provide/save the vector when the interrupt is
         * first signalled.
         */
        cs->exception_index = env->pending_vector;
        do_interrupt_m68k_hardirq(env);
        return true;
    }
    return false;
}

#endif /* !CONFIG_USER_ONLY */

G_NORETURN static void
raise_exception_ra(CPUM68KState *env, int tt, uintptr_t raddr)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = tt;
    cpu_loop_exit_restore(cs, raddr);
}

G_NORETURN static void raise_exception(CPUM68KState *env, int tt)
{
    raise_exception_ra(env, tt, 0);
}

uint32_t HELPER(m68k_mmu030_special_cycle)(CPUM68KState *env,
                                           uint32_t kind, uint32_t cycle,
                                           uint32_t pc, uint32_t data,
                                           uint32_t is_write)
{
    return m68k_mmu030_special_cycle(&env->mmu030, kind, cycle, pc, data,
                                     is_write != 0);
}

void HELPER(m68k_mmu030_special_record)(CPUM68KState *env, uint32_t kind,
                                        uint32_t cycle, uint32_t pc,
                                        uint32_t data, uint32_t is_load)
{
    m68k_mmu030_special_record(&env->mmu030, kind, cycle, pc, data,
                               is_load != 0);
}

void HELPER(m68k_mmu030_special_finish)(CPUM68KState *env, uint32_t kind,
                                        uint32_t cycles, uint32_t pc)
{
    m68k_mmu030_special_finish(&env->mmu030, kind, cycles, pc);
}

static unsigned m68k_mmu030_split_chunk(uint32_t address, unsigned remaining)
{
    unsigned page_remaining = TARGET_PAGE_SIZE -
                               (address & (TARGET_PAGE_SIZE - 1));

    /* A crossing word/long is decomposed into the largest aligned pieces
     * which remain in the current page.  The byte fallback is required for
     * an odd address and preserves the big-endian byte lane order. */
    remaining = MIN(remaining, page_remaining);
    if (remaining >= 2 && !(address & 1)) {
        return 2;
    }
    return 1;
}

uint32_t HELPER(m68k_mmu030_split_load)(CPUM68KState *env,
                                        uint32_t address, uint32_t bytes,
                                        uint32_t mmu_idx, uint32_t pc)
{
    M68KMMU030State *state = &env->mmu030;
    uintptr_t ra = GETPC();
    uint32_t value = 0;

    g_assert(bytes == 2 || bytes == 4);
    if (((address ^ (address + bytes - 1)) & TARGET_PAGE_MASK) == 0) {
        return bytes == 2 ? cpu_lduw_be_mmuidx_ra(env, address, mmu_idx, ra) :
                            cpu_ldl_be_mmuidx_ra(env, address, mmu_idx, ra);
    }

    unsigned offset = 0;
    unsigned cycle = 0;
    while (offset < bytes) {
        uint32_t cycle_address = address + offset;
        unsigned chunk = m68k_mmu030_split_chunk(cycle_address,
                                                 bytes - offset);
        uint32_t piece;
        bool skip = m68k_mmu030_special_cycle(
            state, M68K_MMU030_SPECIAL_SPLIT, cycle, pc, 0, false);

        if (skip) {
            piece = state->fault_special_data[cycle];
        } else if (chunk == 2) {
            piece = cpu_lduw_be_mmuidx_ra(env, cycle_address, mmu_idx, ra);
            m68k_mmu030_special_record(
                state, M68K_MMU030_SPECIAL_SPLIT, cycle, pc, piece, true);
        } else {
            piece = cpu_ldub_mmuidx_ra(env, cycle_address, mmu_idx, ra);
            m68k_mmu030_special_record(
                state, M68K_MMU030_SPECIAL_SPLIT, cycle, pc, piece, true);
        }

        value = (value << (chunk * 8)) | piece;
        offset += chunk;
        cycle++;
    }
    m68k_mmu030_special_finish(state, M68K_MMU030_SPECIAL_SPLIT, cycle, pc);
    return value;
}

void HELPER(m68k_mmu030_split_store)(CPUM68KState *env, uint32_t address,
                                     uint32_t value, uint32_t bytes,
                                     uint32_t mmu_idx, uint32_t pc)
{
    M68KMMU030State *state = &env->mmu030;
    uintptr_t ra = GETPC();

    g_assert(bytes == 2 || bytes == 4);
    if (((address ^ (address + bytes - 1)) & TARGET_PAGE_MASK) == 0) {
        if (bytes == 2) {
            cpu_stw_be_mmuidx_ra(env, address, value, mmu_idx, ra);
        } else {
            cpu_stl_be_mmuidx_ra(env, address, value, mmu_idx, ra);
        }
        return;
    }

    unsigned offset = 0;
    unsigned cycle = 0;
    while (offset < bytes) {
        uint32_t cycle_address = address + offset;
        unsigned chunk = m68k_mmu030_split_chunk(cycle_address,
                                                 bytes - offset);
        unsigned shift = (bytes - offset - chunk) * 8;
        uint32_t piece = (value >> shift) &
                         (chunk == 2 ? UINT32_C(0xffff) : UINT32_C(0xff));
        bool skip = m68k_mmu030_special_cycle(
            state, M68K_MMU030_SPECIAL_SPLIT, cycle, pc, piece, true);

        if (!skip) {
            if (chunk == 2) {
                cpu_stw_be_mmuidx_ra(env, cycle_address, piece, mmu_idx, ra);
            } else {
                cpu_stb_mmuidx_ra(env, cycle_address, piece, mmu_idx, ra);
            }
            m68k_mmu030_special_record(
                state, M68K_MMU030_SPECIAL_SPLIT, cycle, pc, piece, false);
        }

        offset += chunk;
        cycle++;
    }
    m68k_mmu030_special_finish(state, M68K_MMU030_SPECIAL_SPLIT, cycle, pc);
}

/* TAS is one indivisible 030 read-modify-write protocol, but unlike CAS its
 * result is the value returned by the read.  Keep the read value and cycle
 * phase explicitly so a fault on the write can resume without repeating an
 * observable device read, and so a DF-cleared frame can complete in software
 * while preserving TAS's condition-code input. */
uint32_t HELPER(m68k_mmu030_tas)(CPUM68KState *env, uint32_t address,
                                 uint32_t mmu_idx, uint32_t pc)
{
    M68KMMU030State *state = &env->mmu030;
    uintptr_t ra = GETPC();
    uint32_t old;
    bool completed = state->fault_data_complete && state->fault_rmw &&
                     state->fault_address == address;
    bool write_completed = completed && state->fault_data_write;

    /* The live fault state is unique to the current translated instruction;
     * retain the PC in the helper ABI alongside the other restart helpers. */
    (void)pc;

    if (completed) {
        /* DIB supplied by a handler takes precedence over the saved read. */
        if (state->fault_data_input_valid &&
            state->fault_data_input_address == address) {
            old = state->fault_data_input & 0xff;
        } else {
            old = state->fault_rmw_data1 & 0xff;
        }
        if (!write_completed) {
            /*
             * A DF-cleared RM|RW frame supplies only the completed read.
             * Consume DIB, then perform TAS's write half without rereading.
             */
            m68k_mmu030_consume_data_input(state);
            state->fault_data_write = false;
            state->fault_data_output = UINT32_C(0x80);
            cpu_stb_mmuidx_ra(env, address, UINT32_C(0x80), mmu_idx, ra);
        }
    } else {
        if (!state->fault_rmw_data_valid) {
            state->fault_rmw_phase = 0;
        }
        if (state->fault_rmw_phase < 1) {
            old = cpu_ldub_mmuidx_ra(env, address, mmu_idx, ra);
            state->fault_rmw_data1 = old;
            state->fault_rmw_data_valid = true;
            state->fault_rmw_phase = 1;
        } else {
            old = state->fault_rmw_data1;
        }
        if (state->fault_rmw_phase < 2) {
            state->fault_data_output = UINT32_C(0x80);
            cpu_stb_mmuidx_ra(env, address, UINT32_C(0x80), mmu_idx, ra);
            state->fault_rmw_phase = 2;
        }
    }

    state->fault_data_complete = false;
    state->fault_data_input = 0;
    state->fault_data_input_valid = false;
    state->fault_data_write = false;
    state->fault_rmw = false;
    state->fault_rmw_phase = 0;
    state->fault_rmw_data1 = 0;
    state->fault_rmw_data2 = 0;
    state->fault_rmw_data_valid = false;
    return old;
}

/*
 * CAS is a single 030 read-modify-write protocol.  Keep the successful read
 * in the RMW latch while the write is attempted, and consume a handler's DIB
 * when RTE returns with RM|RW and DF clear.
 */
uint32_t HELPER(m68k_mmu030_cas)(CPUM68KState *env, uint32_t address,
                                 uint32_t compare, uint32_t update,
                                 uint32_t bytes, uint32_t mmu_idx)
{
    M68KMMU030State *state = &env->mmu030;
    uintptr_t ra = GETPC();
    uint32_t mask;
    uint32_t old;
    bool completed = state->fault_data_complete && state->fault_rmw &&
                     state->fault_address == address;
    bool write_completed = completed && state->fault_data_write;

    switch (bytes) {
    case 1:
        mask = 0xff;
        break;
    case 2:
        mask = 0xffff;
        break;
    case 4:
        mask = UINT32_MAX;
        break;
    default:
        g_assert_not_reached();
    }
    compare &= mask;
    update &= mask;

    if (completed) {
        if (state->fault_data_input_valid &&
            state->fault_data_input_address == address) {
            old = state->fault_data_input & mask;
        } else {
            old = state->fault_rmw_data1 & mask;
        }
        if (!write_completed && old == compare) {
            /*
             * The read is represented by DIB; only the conditional write
             * remains to be issued on the repaired mapping.
             */
            m68k_mmu030_consume_data_input(state);
            state->fault_data_write = false;
            state->fault_data_output = update;
            switch (bytes) {
            case 1:
                cpu_stb_mmuidx_ra(env, address, update, mmu_idx, ra);
                break;
            case 2:
                cpu_stw_be_mmuidx_ra(env, address, update, mmu_idx, ra);
                break;
            case 4:
                cpu_stl_be_mmuidx_ra(env, address, update, mmu_idx, ra);
                break;
            }
        }
    } else {
        if (!state->fault_rmw_data_valid) {
            state->fault_rmw_phase = 0;
        }
        if (state->fault_rmw_phase < 1) {
            switch (bytes) {
            case 1:
                old = cpu_ldub_mmuidx_ra(env, address, mmu_idx, ra);
                break;
            case 2:
                old = cpu_lduw_be_mmuidx_ra(env, address, mmu_idx, ra);
                break;
            case 4:
                old = cpu_ldl_be_mmuidx_ra(env, address, mmu_idx, ra);
                break;
            default:
                g_assert_not_reached();
            }
            state->fault_rmw_data1 = old & mask;
            state->fault_rmw_data_valid = true;
            state->fault_rmw_phase = 1;
        } else {
            old = state->fault_rmw_data1 & mask;
        }
        if (old == compare && state->fault_rmw_phase < 2) {
            state->fault_data_output = update;
            switch (bytes) {
            case 1:
                cpu_stb_mmuidx_ra(env, address, update, mmu_idx, ra);
                break;
            case 2:
                cpu_stw_be_mmuidx_ra(env, address, update, mmu_idx, ra);
                break;
            case 4:
                cpu_stl_be_mmuidx_ra(env, address, update, mmu_idx, ra);
                break;
            default:
                g_assert_not_reached();
            }
            state->fault_rmw_phase = 2;
        }
    }

    state->fault_data_complete = false;
    state->fault_data_input = 0;
    state->fault_data_input_valid = false;
    state->fault_data_write = false;
    state->fault_data_input_address = 0;
    state->fault_rmw = false;
    state->fault_rmw_phase = 0;
    state->fault_rmw_data1 = 0;
    state->fault_rmw_data2 = 0;
    state->fault_rmw_data_valid = false;
    return old;
}

uint32_t HELPER(m68k_movep)(CPUM68KState *env, uint32_t address,
                            uint32_t data, uint32_t is_load,
                            uint32_t bytes, uint32_t pc)
{
    M68KMMU030State *state = &env->mmu030;
    uintptr_t ra = GETPC();
    uint32_t result = data;
    bool mmu030 = m68k_feature(env, M68K_FEATURE_M68030);

    for (unsigned cycle = 0; cycle < bytes; cycle++) {
        uint32_t cycle_address = address + cycle * 2;
        uint32_t value;
        bool skip = mmu030 && m68k_mmu030_special_cycle(
            state, M68K_MMU030_SPECIAL_MOVEP, cycle, pc,
            (data >> ((bytes - cycle - 1) * 8)) & 0xff,
            !is_load);

        if (skip) {
            value = state->fault_special_data[cycle] & 0xff;
        } else if (is_load) {
            value = cpu_ldub_data_ra(env, cycle_address, ra);
            if (mmu030) {
                m68k_mmu030_special_record(
                    state, M68K_MMU030_SPECIAL_MOVEP, cycle, pc, value, true);
            }
        } else {
            value = (data >> ((bytes - cycle - 1) * 8)) & 0xff;
            cpu_stb_data_ra(env, cycle_address, value, ra);
            if (mmu030) {
                m68k_mmu030_special_record(
                    state, M68K_MMU030_SPECIAL_MOVEP, cycle, pc, value, false);
            }
        }

        if (is_load) {
            unsigned shift = (bytes - cycle - 1) * 8;
            result = deposit32(result, shift, 8, value);
        }
    }
    if (mmu030) {
        m68k_mmu030_special_finish(state, M68K_MMU030_SPECIAL_MOVEP,
                                   bytes, pc);
    }
    return result;
}

void HELPER(raise_exception)(CPUM68KState *env, uint32_t tt)
{
    raise_exception(env, tt);
}

G_NORETURN static void
raise_exception_format2(CPUM68KState *env, int tt, int ilen, uintptr_t raddr)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = tt;

    /* Recover PC and CC_OP for the beginning of the insn.  */
    cpu_restore_state(cs, raddr);

    /* Flags are current in env->cc_*, or are undefined. */
    env->cc_op = CC_OP_FLAGS;

    /*
     * Remember original pc in mmu.ar, for the Format 2 stack frame.
     * Adjust PC to end of the insn.
     */
    env->mmu.ar = env->pc;
    env->pc += ilen;

    cpu_loop_exit(cs);
}

void HELPER(divuw)(CPUM68KState *env, int destr, uint32_t den, int ilen)
{
    uint32_t num = env->dregs[destr];
    uint32_t quot, rem;

    env->cc_c = 0; /* always cleared, even if div0 */

    if (den == 0) {
        raise_exception_format2(env, EXCP_DIV0, ilen, GETPC());
    }
    quot = num / den;
    rem = num % den;

    if (quot > 0xffff) {
        env->cc_v = -1;
        /*
         * real 68040 keeps N and unset Z on overflow,
         * whereas documentation says "undefined"
         */
        env->cc_z = 1;
        return;
    }
    env->dregs[destr] = deposit32(quot, 16, 16, rem);
    env->cc_z = (int16_t)quot;
    env->cc_n = (int16_t)quot;
    env->cc_v = 0;
}

void HELPER(divsw)(CPUM68KState *env, int destr, int32_t den, int ilen)
{
    int32_t num = env->dregs[destr];
    uint32_t quot, rem;

    env->cc_c = 0; /* always cleared, even if overflow/div0 */

    if (den == 0) {
        raise_exception_format2(env, EXCP_DIV0, ilen, GETPC());
    }
    quot = num / den;
    rem = num % den;

    if (quot != (int16_t)quot) {
        env->cc_v = -1;
        /* nothing else is modified */
        /*
         * real 68040 keeps N and unset Z on overflow,
         * whereas documentation says "undefined"
         */
        env->cc_z = 1;
        return;
    }
    env->dregs[destr] = deposit32(quot, 16, 16, rem);
    env->cc_z = (int16_t)quot;
    env->cc_n = (int16_t)quot;
    env->cc_v = 0;
}

void HELPER(divul)(CPUM68KState *env, int numr, int regr,
                   uint32_t den, int ilen)
{
    uint32_t num = env->dregs[numr];
    uint32_t quot, rem;

    env->cc_c = 0; /* always cleared, even if div0 */

    if (den == 0) {
        raise_exception_format2(env, EXCP_DIV0, ilen, GETPC());
    }
    quot = num / den;
    rem = num % den;

    env->cc_z = quot;
    env->cc_n = quot;
    env->cc_v = 0;

    if (m68k_feature(env, M68K_FEATURE_CF_ISA_A)) {
        if (numr == regr) {
            env->dregs[numr] = quot;
        } else {
            env->dregs[regr] = rem;
        }
    } else {
        env->dregs[regr] = rem;
        env->dregs[numr] = quot;
    }
}

void HELPER(divsl)(CPUM68KState *env, int numr, int regr,
                   int32_t den, int ilen)
{
    int32_t num = env->dregs[numr];
    int32_t quot, rem;

    env->cc_c = 0; /* always cleared, even if overflow/div0 */

    if (den == 0) {
        raise_exception_format2(env, EXCP_DIV0, ilen, GETPC());
    }
    quot = num / den;
    rem = num % den;

    env->cc_z = quot;
    env->cc_n = quot;
    env->cc_v = 0;

    if (m68k_feature(env, M68K_FEATURE_CF_ISA_A)) {
        if (numr == regr) {
            env->dregs[numr] = quot;
        } else {
            env->dregs[regr] = rem;
        }
    } else {
        env->dregs[regr] = rem;
        env->dregs[numr] = quot;
    }
}

void HELPER(divull)(CPUM68KState *env, int numr, int regr,
                    uint32_t den, int ilen)
{
    uint64_t num = deposit64(env->dregs[numr], 32, 32, env->dregs[regr]);
    uint64_t quot;
    uint32_t rem;

    env->cc_c = 0; /* always cleared, even if overflow/div0 */

    if (den == 0) {
        raise_exception_format2(env, EXCP_DIV0, ilen, GETPC());
    }
    quot = num / den;
    rem = num % den;

    if (quot > 0xffffffffULL) {
        env->cc_v = -1;
        /*
         * real 68040 keeps N and unset Z on overflow,
         * whereas documentation says "undefined"
         */
        env->cc_z = 1;
        return;
    }
    env->cc_z = quot;
    env->cc_n = quot;
    env->cc_v = 0;

    /*
     * If Dq and Dr are the same, the quotient is returned.
     * therefore we set Dq last.
     */

    env->dregs[regr] = rem;
    env->dregs[numr] = quot;
}

void HELPER(divsll)(CPUM68KState *env, int numr, int regr,
                    int32_t den, int ilen)
{
    int64_t num = deposit64(env->dregs[numr], 32, 32, env->dregs[regr]);
    int64_t quot;
    int32_t rem;

    env->cc_c = 0; /* always cleared, even if overflow/div0 */

    if (den == 0) {
        raise_exception_format2(env, EXCP_DIV0, ilen, GETPC());
    }
    quot = num / den;
    rem = num % den;

    if (quot != (int32_t)quot) {
        env->cc_v = -1;
        /*
         * real 68040 keeps N and unset Z on overflow,
         * whereas documentation says "undefined"
         */
        env->cc_z = 1;
        return;
    }
    env->cc_z = quot;
    env->cc_n = quot;
    env->cc_v = 0;

    /*
     * If Dq and Dr are the same, the quotient is returned.
     * therefore we set Dq last.
     */

    env->dregs[regr] = rem;
    env->dregs[numr] = quot;
}

/* We're executing in a serial context -- no need to be atomic.  */
void HELPER(cas2w)(CPUM68KState *env, uint32_t regs, uint32_t a1, uint32_t a2)
{
    M68KMMU030State *state = &env->mmu030;
    uint32_t Dc1 = extract32(regs, 9, 3);
    uint32_t Dc2 = extract32(regs, 6, 3);
    uint32_t Du1 = extract32(regs, 3, 3);
    uint32_t Du2 = extract32(regs, 0, 3);
    int16_t c1 = env->dregs[Dc1];
    int16_t c2 = env->dregs[Dc2];
    int16_t u1 = env->dregs[Du1];
    int16_t u2 = env->dregs[Du2];
    int16_t l1, l2;
    uintptr_t ra = GETPC();
    bool mmu030 = m68k_feature(env, M68K_FEATURE_M68030);
    bool completed = mmu030 && state->fault_data_complete &&
                     state->fault_data_write && state->fault_rmw;
    bool read_completed = mmu030 && state->fault_data_complete &&
                          !state->fault_data_write && state->fault_rmw;
    unsigned read_phase = state->fault_rmw_phase;
    /* CAS2 is a sequence of independently restartable bus cycles in the
     * serial execution path.  Keep completed reads in the internal latch so
     * a fault on the second read or either write does not repeat a device
     * access after RTE. */
    if (completed) {
        /* RM-only with DF clear means the handler supplied the complete
         * operation.  Preserve the successful comparison result without
         * issuing either read or write bus cycle a second time. */
        l1 = c1;
        l2 = c2;
    } else if (!mmu030) {
        l1 = cpu_lduw_be_data_ra(env, a1, ra);
        l2 = cpu_lduw_be_data_ra(env, a2, ra);
    } else {
        if (!state->fault_rmw_data_valid) {
            state->fault_rmw_phase = 0;
        }
        if (state->fault_rmw_phase < 1) {
            if (read_completed && read_phase == 0 &&
                state->fault_data_input_valid) {
                l1 = state->fault_data_input;
                m68k_mmu030_consume_data_input(state);
            } else {
                l1 = cpu_lduw_be_data_ra(env, a1, ra);
            }
            state->fault_rmw_data1 = (uint16_t)l1;
            state->fault_rmw_data_valid = true;
            state->fault_rmw_phase = 1;
        } else {
            l1 = state->fault_rmw_data1;
        }
        if (state->fault_rmw_phase < 2) {
            if (read_completed && read_phase == 1 &&
                state->fault_data_input_valid) {
                l2 = state->fault_data_input;
                m68k_mmu030_consume_data_input(state);
            } else {
                l2 = cpu_lduw_be_data_ra(env, a2, ra);
            }
            state->fault_rmw_data2 = (uint16_t)l2;
            state->fault_rmw_phase = 2;
        } else {
            l2 = state->fault_rmw_data2;
        }
    }
    if (!completed && l1 == c1 && l2 == c2) {
        if (!mmu030) {
            cpu_stw_be_data_ra(env, a1, u1, ra);
            cpu_stw_be_data_ra(env, a2, u2, ra);
        } else {
            if (state->fault_rmw_phase < 3) {
                cpu_stw_be_data_ra(env, a1, u1, ra);
                state->fault_rmw_phase = 3;
            }
            if (state->fault_rmw_phase < 4) {
                cpu_stw_be_data_ra(env, a2, u2, ra);
                state->fault_rmw_phase = 4;
            }
        }
    }

    if (c1 != l1) {
        env->cc_n = l1;
        env->cc_v = c1;
    } else {
        env->cc_n = l2;
        env->cc_v = c2;
    }
    env->cc_op = CC_OP_CMPW;
    env->dregs[Dc2] = deposit32(env->dregs[Dc2], 0, 16, l2);
    env->dregs[Dc1] = deposit32(env->dregs[Dc1], 0, 16, l1);
    if (mmu030) {
        state->fault_data_complete = false;
        state->fault_data_input = 0;
        state->fault_data_input_address = 0;
        state->fault_data_input_valid = false;
        state->fault_data_write = false;
        state->fault_rmw = false;
        state->fault_rmw_phase = 0;
        state->fault_rmw_data_valid = false;
    }
}

static void do_cas2l(CPUM68KState *env, uint32_t regs, uint32_t a1, uint32_t a2,
                     bool parallel)
{
    M68KMMU030State *state = &env->mmu030;
    uint32_t Dc1 = extract32(regs, 9, 3);
    uint32_t Dc2 = extract32(regs, 6, 3);
    uint32_t Du1 = extract32(regs, 3, 3);
    uint32_t Du2 = extract32(regs, 0, 3);
    uint32_t c1 = env->dregs[Dc1];
    uint32_t c2 = env->dregs[Dc2];
    uint32_t u1 = env->dregs[Du1];
    uint32_t u2 = env->dregs[Du2];
    uint32_t l1, l2;
    uintptr_t ra = GETPC();
    int mmu_idx = cpu_mmu_index(env_cpu(env), 0);
    MemOpIdx oi = make_memop_idx(MO_BEUQ, mmu_idx);
    bool mmu030 = m68k_feature(env, M68K_FEATURE_M68030);
    bool completed = mmu030 && !parallel && state->fault_data_complete &&
                     state->fault_data_write && state->fault_rmw;
    bool read_completed = mmu030 && !parallel && state->fault_data_complete &&
                          !state->fault_data_write && state->fault_rmw;
    unsigned read_phase = state->fault_rmw_phase;
    if (parallel) {
        /* We're executing in a parallel context -- must be atomic.  */
        uint64_t c, u, l;
        if ((a1 & 7) == 0 && a2 == a1 + 4) {
            c = deposit64(c2, 32, 32, c1);
            u = deposit64(u2, 32, 32, u1);
            l = cpu_atomic_cmpxchgq_be_mmu(env, a1, c, u, oi, ra);
            l1 = l >> 32;
            l2 = l;
        } else if ((a2 & 7) == 0 && a1 == a2 + 4) {
            c = deposit64(c1, 32, 32, c2);
            u = deposit64(u1, 32, 32, u2);
            l = cpu_atomic_cmpxchgq_be_mmu(env, a2, c, u, oi, ra);
            l2 = l >> 32;
            l1 = l;
        } else {
            /* Tell the main loop we need to serialize this insn.  */
            cpu_loop_exit_atomic(env_cpu(env), ra);
        }
    } else {
        /* We're executing in a serial context -- no need to be atomic.  */
        if (completed) {
            l1 = c1;
            l2 = c2;
        } else if (!mmu030) {
            l1 = cpu_ldl_be_data_ra(env, a1, ra);
            l2 = cpu_ldl_be_data_ra(env, a2, ra);
        } else {
            if (!state->fault_rmw_data_valid) {
                state->fault_rmw_phase = 0;
            }
            if (state->fault_rmw_phase < 1) {
                if (read_completed && read_phase == 0 &&
                    state->fault_data_input_valid) {
                    l1 = state->fault_data_input;
                    m68k_mmu030_consume_data_input(state);
                } else {
                    l1 = cpu_ldl_be_data_ra(env, a1, ra);
                }
                state->fault_rmw_data1 = l1;
                state->fault_rmw_data_valid = true;
                state->fault_rmw_phase = 1;
            } else {
                l1 = state->fault_rmw_data1;
            }
            if (state->fault_rmw_phase < 2) {
                if (read_completed && read_phase == 1 &&
                    state->fault_data_input_valid) {
                    l2 = state->fault_data_input;
                    m68k_mmu030_consume_data_input(state);
                } else {
                    l2 = cpu_ldl_be_data_ra(env, a2, ra);
                }
                state->fault_rmw_data2 = l2;
                state->fault_rmw_phase = 2;
            } else {
                l2 = state->fault_rmw_data2;
            }
        }
        if (!completed && l1 == c1 && l2 == c2) {
            if (!mmu030) {
                cpu_stl_be_data_ra(env, a1, u1, ra);
                cpu_stl_be_data_ra(env, a2, u2, ra);
            } else {
                if (state->fault_rmw_phase < 3) {
                    cpu_stl_be_data_ra(env, a1, u1, ra);
                    state->fault_rmw_phase = 3;
                }
                if (state->fault_rmw_phase < 4) {
                    cpu_stl_be_data_ra(env, a2, u2, ra);
                    state->fault_rmw_phase = 4;
                }
            }
        }
    }

    if (c1 != l1) {
        env->cc_n = l1;
        env->cc_v = c1;
    } else {
        env->cc_n = l2;
        env->cc_v = c2;
    }
    env->cc_op = CC_OP_CMPL;
    env->dregs[Dc2] = l2;
    env->dregs[Dc1] = l1;
    if (mmu030) {
        state->fault_data_complete = false;
        state->fault_data_input = 0;
        state->fault_data_input_address = 0;
        state->fault_data_input_valid = false;
        state->fault_data_write = false;
        state->fault_rmw = false;
        state->fault_rmw_phase = 0;
        state->fault_rmw_data_valid = false;
    }
}

void HELPER(cas2l)(CPUM68KState *env, uint32_t regs, uint32_t a1, uint32_t a2)
{
    do_cas2l(env, regs, a1, a2, false);
}

void HELPER(cas2l_parallel)(CPUM68KState *env, uint32_t regs, uint32_t a1,
                            uint32_t a2)
{
    do_cas2l(env, regs, a1, a2, true);
}

struct bf_data {
    uint32_t addr;
    uint32_t bofs;
    uint32_t blen;
    uint32_t len;
};

static struct bf_data bf_prep(uint32_t addr, int32_t ofs, uint32_t len)
{
    int bofs, blen;

    /* Bound length; map 0 to 32.  */
    len = ((len - 1) & 31) + 1;

    /* Note that ofs is signed.  */
    addr += ofs / 8;
    bofs = ofs % 8;
    if (bofs < 0) {
        bofs += 8;
        addr -= 1;
    }

    /*
     * Compute the number of bytes required (minus one) to
     * satisfy the bitfield.
     */
    blen = (bofs + len - 1) / 8;

    /*
     * Canonicalize the bit offset for data loaded into a 64-bit big-endian
     * word.  For the cases where BLEN is not a power of 2, adjust ADDR so
     * that we can use the next power of two sized load without crossing a
     * page boundary, unless the field itself crosses the boundary.
     */
    switch (blen) {
    case 0:
        bofs += 56;
        break;
    case 1:
        bofs += 48;
        break;
    case 2:
        if (addr & 1) {
            bofs += 8;
            addr -= 1;
        }
        /* fallthru */
    case 3:
        bofs += 32;
        break;
    case 4:
        if (addr & 3) {
            bofs += 8 * (addr & 3);
            addr &= -4;
        }
        break;
    default:
        g_assert_not_reached();
    }

    return (struct bf_data){
        .addr = addr,
        .bofs = bofs,
        .blen = blen,
        .len = len,
    };
}

static uint64_t bf_load(CPUM68KState *env, uint32_t addr, int blen,
                        uintptr_t ra)
{
    switch (blen) {
    case 0:
        return cpu_ldub_data_ra(env, addr, ra);
    case 1:
        return cpu_lduw_be_data_ra(env, addr, ra);
    case 2:
    case 3:
        return cpu_ldl_be_data_ra(env, addr, ra);
    case 4:
        return cpu_ldq_be_data_ra(env, addr, ra);
    default:
        g_assert_not_reached();
    }
}

static void bf_store(CPUM68KState *env, uint32_t addr, int blen,
                     uint64_t data, uintptr_t ra)
{
    switch (blen) {
    case 0:
        cpu_stb_data_ra(env, addr, data, ra);
        break;
    case 1:
        cpu_stw_be_data_ra(env, addr, data, ra);
        break;
    case 2:
    case 3:
        cpu_stl_be_data_ra(env, addr, data, ra);
        break;
    case 4:
        cpu_stq_be_data_ra(env, addr, data, ra);
        break;
    default:
        g_assert_not_reached();
    }
}

uint32_t HELPER(bfexts_mem)(CPUM68KState *env, uint32_t addr,
                            int32_t ofs, uint32_t len)
{
    uintptr_t ra = GETPC();
    struct bf_data d = bf_prep(addr, ofs, len);
    uint64_t data = bf_load(env, d.addr, d.blen, ra);

    return (int64_t)(data << d.bofs) >> (64 - d.len);
}

uint64_t HELPER(bfextu_mem)(CPUM68KState *env, uint32_t addr,
                            int32_t ofs, uint32_t len)
{
    uintptr_t ra = GETPC();
    struct bf_data d = bf_prep(addr, ofs, len);
    uint64_t data = bf_load(env, d.addr, d.blen, ra);

    /*
     * Put CC_N at the top of the high word; put the zero-extended value
     * at the bottom of the low word.
     */
    data <<= d.bofs;
    data >>= 64 - d.len;
    data |= data << (64 - d.len);

    return data;
}

uint32_t HELPER(bfins_mem)(CPUM68KState *env, uint32_t addr, uint32_t val,
                           int32_t ofs, uint32_t len)
{
    uintptr_t ra = GETPC();
    struct bf_data d = bf_prep(addr, ofs, len);
    uint64_t data = bf_load(env, d.addr, d.blen, ra);
    uint64_t mask = -1ull << (64 - d.len) >> d.bofs;

    data = (data & ~mask) | (((uint64_t)val << (64 - d.len)) >> d.bofs);

    bf_store(env, d.addr, d.blen, data, ra);

    /* The field at the top of the word is also CC_N for CC_OP_LOGIC.  */
    return val << (32 - d.len);
}

uint32_t HELPER(bfchg_mem)(CPUM68KState *env, uint32_t addr,
                           int32_t ofs, uint32_t len)
{
    uintptr_t ra = GETPC();
    struct bf_data d = bf_prep(addr, ofs, len);
    uint64_t data = bf_load(env, d.addr, d.blen, ra);
    uint64_t mask = -1ull << (64 - d.len) >> d.bofs;

    bf_store(env, d.addr, d.blen, data ^ mask, ra);

    return ((data & mask) << d.bofs) >> 32;
}

uint32_t HELPER(bfclr_mem)(CPUM68KState *env, uint32_t addr,
                           int32_t ofs, uint32_t len)
{
    uintptr_t ra = GETPC();
    struct bf_data d = bf_prep(addr, ofs, len);
    uint64_t data = bf_load(env, d.addr, d.blen, ra);
    uint64_t mask = -1ull << (64 - d.len) >> d.bofs;

    bf_store(env, d.addr, d.blen, data & ~mask, ra);

    return ((data & mask) << d.bofs) >> 32;
}

uint32_t HELPER(bfset_mem)(CPUM68KState *env, uint32_t addr,
                           int32_t ofs, uint32_t len)
{
    uintptr_t ra = GETPC();
    struct bf_data d = bf_prep(addr, ofs, len);
    uint64_t data = bf_load(env, d.addr, d.blen, ra);
    uint64_t mask = -1ull << (64 - d.len) >> d.bofs;

    bf_store(env, d.addr, d.blen, data | mask, ra);

    return ((data & mask) << d.bofs) >> 32;
}

uint32_t HELPER(bfffo_reg)(uint32_t n, uint32_t ofs, uint32_t len)
{
    return (n ? clz32(n) : len) + ofs;
}

uint64_t HELPER(bfffo_mem)(CPUM68KState *env, uint32_t addr,
                           int32_t ofs, uint32_t len)
{
    uintptr_t ra = GETPC();
    struct bf_data d = bf_prep(addr, ofs, len);
    uint64_t data = bf_load(env, d.addr, d.blen, ra);
    uint64_t mask = -1ull << (64 - d.len) >> d.bofs;
    uint64_t n = (data & mask) << d.bofs;
    uint32_t ffo = helper_bfffo_reg(n >> 32, ofs, d.len);

    /*
     * Return FFO in the low word and N in the high word.
     * Note that because of MASK and the shift, the low word
     * is already zero.
     */
    return n | ffo;
}

void HELPER(chk)(CPUM68KState *env, int32_t val, int32_t ub, int ilen)
{
    /*
     * From the specs:
     *   X: Not affected, C,V,Z: Undefined,
     *   N: Set if val < 0; cleared if val > ub, undefined otherwise
     * We implement here values found from a real MC68040:
     *   X,V,Z: Not affected
     *   N: Set if val < 0; cleared if val >= 0
     *   C: if 0 <= ub: set if val < 0 or val > ub, cleared otherwise
     *      if 0 > ub: set if val > ub and val < 0, cleared otherwise
     */
    env->cc_n = val;
    env->cc_c = 0 <= ub ? val < 0 || val > ub : val > ub && val < 0;

    if (val < 0 || val > ub) {
        raise_exception_format2(env, EXCP_CHK, ilen, GETPC());
    }
}

void HELPER(chk2)(CPUM68KState *env, int32_t val, int32_t lb, int32_t ub,
                  int ilen)
{
    /*
     * From the specs:
     *   X: Not affected, N,V: Undefined,
     *   Z: Set if val is equal to lb or ub
     *   C: Set if val < lb or val > ub, cleared otherwise
     * We implement here values found from a real MC68040:
     *   X,N,V: Not affected
     *   Z: Set if val is equal to lb or ub
     *   C: if lb <= ub: set if val < lb or val > ub, cleared otherwise
     *      if lb > ub: set if val > ub and val < lb, cleared otherwise
     */
    env->cc_z = val != lb && val != ub;
    env->cc_c = lb <= ub ? val < lb || val > ub : val > ub && val < lb;

    if (env->cc_c) {
        raise_exception_format2(env, EXCP_CHK, ilen, GETPC());
    }
}

void HELPER(cmp2)(CPUM68KState *env, int32_t val, int32_t lb, int32_t ub)
{
    /* Identical to CHK2 (above) but doesn't raise an exception */
    env->cc_z = val != lb && val != ub;
    env->cc_c = lb <= ub ? val < lb || val > ub : val > ub && val < lb;
}
