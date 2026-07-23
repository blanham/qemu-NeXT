/*
 *  m68k FPU helpers
 *
 *  Copyright (c) 2006-2007 CodeSourcery
 *  Written by Paul Brook
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
#include "cpu.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-ldst.h"
#include "accel/tcg/cpu-loop.h"
#include "softfloat.h"

/*
 * Undefined offsets may be different on various FPU.
 * On 68040 they return 0.0 (floatx80_zero)
 */

static const floatx80 fpu_rom[128] = {
    [0x00] = make_floatx80_init(0x4000, 0xc90fdaa22168c235ULL),  /* Pi       */
    [0x0b] = make_floatx80_init(0x3ffd, 0x9a209a84fbcff798ULL),  /* Log10(2) */
    [0x0c] = make_floatx80_init(0x4000, 0xadf85458a2bb4a9aULL),  /* e        */
    [0x0d] = make_floatx80_init(0x3fff, 0xb8aa3b295c17f0bcULL),  /* Log2(e)  */
    [0x0e] = make_floatx80_init(0x3ffd, 0xde5bd8a937287195ULL),  /* Log10(e) */
    [0x0f] = make_floatx80_init(0x0000, 0x0000000000000000ULL),  /* Zero     */
    [0x30] = make_floatx80_init(0x3ffe, 0xb17217f7d1cf79acULL),  /* ln(2)    */
    [0x31] = make_floatx80_init(0x4000, 0x935d8dddaaa8ac17ULL),  /* ln(10)   */
    [0x32] = make_floatx80_init(0x3fff, 0x8000000000000000ULL),  /* 10^0     */
    [0x33] = make_floatx80_init(0x4002, 0xa000000000000000ULL),  /* 10^1     */
    [0x34] = make_floatx80_init(0x4005, 0xc800000000000000ULL),  /* 10^2     */
    [0x35] = make_floatx80_init(0x400c, 0x9c40000000000000ULL),  /* 10^4     */
    [0x36] = make_floatx80_init(0x4019, 0xbebc200000000000ULL),  /* 10^8     */
    [0x37] = make_floatx80_init(0x4034, 0x8e1bc9bf04000000ULL),  /* 10^16    */
    [0x38] = make_floatx80_init(0x4069, 0x9dc5ada82b70b59eULL),  /* 10^32    */
    [0x39] = make_floatx80_init(0x40d3, 0xc2781f49ffcfa6d5ULL),  /* 10^64    */
    [0x3a] = make_floatx80_init(0x41a8, 0x93ba47c980e98ce0ULL),  /* 10^128   */
    [0x3b] = make_floatx80_init(0x4351, 0xaa7eebfb9df9de8eULL),  /* 10^256   */
    [0x3c] = make_floatx80_init(0x46a3, 0xe319a0aea60e91c7ULL),  /* 10^512   */
    [0x3d] = make_floatx80_init(0x4d48, 0xc976758681750c17ULL),  /* 10^1024  */
    [0x3e] = make_floatx80_init(0x5a92, 0x9e8b3b5dc53d5de5ULL),  /* 10^2048  */
    [0x3f] = make_floatx80_init(0x7525, 0xc46052028a20979bULL),  /* 10^4096  */
};

int32_t HELPER(reds32)(CPUM68KState *env, FPReg *val)
{
    return floatx80_to_int32(val->d, &env->fp_status);
}

float32 HELPER(redf32)(CPUM68KState *env, FPReg *val)
{
    return floatx80_to_float32(val->d, &env->fp_status);
}

void HELPER(exts32)(CPUM68KState *env, FPReg *res, int32_t val)
{
    res->d = int32_to_floatx80(val, &env->fp_status);
}

void HELPER(extf32)(CPUM68KState *env, FPReg *res, float32 val)
{
    res->d = float32_to_floatx80(val, &env->fp_status);
}

void HELPER(extf64)(CPUM68KState *env, FPReg *res, float64 val)
{
    res->d = float64_to_floatx80(val, &env->fp_status);
}

/*
 * Convert the 68881/68882 96-bit packed-decimal format to extended
 * precision.  The finite format contains a 17-digit BCD significand and a
 * signed three-digit decimal exponent:
 *
 *     value = significand * 10 ** (exponent - 16)
 *
 * The 68040 implements this conversion through its FPSP exception path.
 * QEMU already executes the other software-assisted 68040 operations
 * directly, so doing the conversion here avoids requiring a guest FPSP while
 * preserving the architectural result.
 */
void HELPER(extp96)(CPUM68KState *env, FPReg *res,
                    uint32_t word0, uint32_t word1, uint32_t word2)
{
    const bool sign = word0 >> 31;
    const bool exp_sign = extract32(word0, 30, 1);
    uint64_t significand = word0 & 0xf;
    float128 value, ten, scale;
    int exponent, decimal_scale;
    int shift;

    /*
     * Infinity and NaN use SE=y=1 and an all-ones exponent.  A zero
     * significand selects infinity; a nonzero significand selects NaN.
     */
    if ((word0 & 0x7fff0000) == 0x7fff0000) {
        if ((word0 & 0xf) == 0 && word1 == 0 && word2 == 0) {
            res->d = packFloatx80(sign, 0x7fff, 0);
        } else {
            res->d = floatx80_default_nan(&env->fp_status);
            res->d.high = deposit32(res->d.high, 15, 1, sign);
        }
        return;
    }

    exponent = extract32(word0, 24, 4) * 100 +
               extract32(word0, 20, 4) * 10 +
               extract32(word0, 16, 4);
    if ((word0 & 0xf) > 9 ||
        extract32(word0, 24, 4) > 9 ||
        extract32(word0, 20, 4) > 9 ||
        extract32(word0, 16, 4) > 9) {
        float_raise(float_flag_invalid, &env->fp_status);
        res->d = floatx80_default_nan(&env->fp_status);
        return;
    }

    for (shift = 28; shift >= 0; shift -= 4) {
        unsigned digit = extract32(word1, shift, 4);

        if (digit > 9) {
            float_raise(float_flag_invalid, &env->fp_status);
            res->d = floatx80_default_nan(&env->fp_status);
            return;
        }
        significand = significand * 10 + digit;
    }
    for (shift = 28; shift >= 0; shift -= 4) {
        unsigned digit = extract32(word2, shift, 4);

        if (digit > 9) {
            float_raise(float_flag_invalid, &env->fp_status);
            res->d = floatx80_default_nan(&env->fp_status);
            return;
        }
        significand = significand * 10 + digit;
    }

    if (significand == 0) {
        res->d = packFloatx80(sign, 0, 0);
        return;
    }

    decimal_scale = (exp_sign ? -exponent : exponent) - 16;
    value = uint64_to_float128(significand, &env->fp_status);
    ten = int64_to_float128(10, &env->fp_status);
    scale = int64_to_float128(1, &env->fp_status);

    for (unsigned power = decimal_scale < 0 ? -decimal_scale : decimal_scale;
         power != 0; power >>= 1) {
        if (power & 1) {
            scale = float128_mul(scale, ten, &env->fp_status);
        }
        ten = float128_mul(ten, ten, &env->fp_status);
    }
    if (decimal_scale < 0) {
        value = float128_div(value, scale, &env->fp_status);
    } else {
        value = float128_mul(value, scale, &env->fp_status);
    }

    res->d = float128_to_floatx80(value, &env->fp_status);
    res->d.high = deposit32(res->d.high, 15, 1, sign);
}

float64 HELPER(redf64)(CPUM68KState *env, FPReg *val)
{
    return floatx80_to_float64(val->d, &env->fp_status);
}

static bool valid_fp_state_frame(uint8_t version, unsigned size)
{
    if (version == 0) {
        return true; /* null */
    }
    if ((version & 0xf0) != 0x40) {
        return false;
    }

    switch (size) {
    case 4:   /* idle */
    case 44:  /* original-revision unimplemented instruction */
    case 52:  /* revised unimplemented instruction */
    case 100: /* busy */
        return true;
    default:
        return false;
    }
}

uint32_t HELPER(fsave)(CPUM68KState *env, uint32_t addr, uint32_t mode)
{
    uintptr_t ra = GETPC();
    unsigned size = env->fp_state_size ? env->fp_state_size : 4;
    unsigned i;

    if (mode == 4) {
        addr -= size;
    }

    if (env->fp_state_size) {
        for (i = 0; i < size; i++) {
            cpu_stb_data_ra(env, addr + i, env->fp_state[i], ra);
        }
    } else {
        cpu_stl_be_data_ra(env, addr, 0x41000000, ra);
    }

    /* FSAVE leaves the floating-point unit in the idle state. */
    env->fp_state_size = 0;
    env->fp_pending_vector = 0;

    return mode == 3 ? addr + size : addr;
}

uint32_t HELPER(frestore)(CPUM68KState *env, uint32_t addr)
{
    uintptr_t ra = GETPC();
    CPUState *cs = env_cpu(env);
    uint8_t version = cpu_ldub_data_ra(env, addr, ra);
    unsigned size = version == 0 ? 4 :
                    cpu_ldub_data_ra(env, addr + 1, ra) + 4;
    unsigned i;

    if (!valid_fp_state_frame(version, size)) {
        cs->exception_index = EXCP_FORMAT;
        cpu_loop_exit_restore(cs, ra);
    }

    if (version == 0) {
        floatx80 nan = floatx80_default_nan(&env->fp_status);

        for (i = 0; i < ARRAY_SIZE(env->fregs); i++) {
            env->fregs[i].d = nan;
        }
        cpu_m68k_set_fpcr(env, 0);
        env->fpsr = 0;
        env->fpiar = 0;
        env->fp_pending_vector = 0;
        env->fp_pending_pc = 0;
        set_float_exception_flags(0, &env->fp_status);
        memset(env->fp_state, 0, 4);
        env->fp_state_size = 4;
    } else if (size == 4) {
        env->fp_state_size = 0;
    } else {
        for (i = 0; i < size; i++) {
            env->fp_state[i] = cpu_ldub_data_ra(env, addr + i, ra);
        }
        env->fp_state_size = size;
    }

    return size;
}

void HELPER(firound)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_round_to_int(val->d, &env->fp_status);
}

static void m68k_restore_precision_mode(CPUM68KState *env)
{
    switch (env->fpcr & FPCR_PREC_MASK) {
    case FPCR_PREC_X: /* extended */
        set_floatx80_rounding_precision(floatx80_precision_x, &env->fp_status);
        break;
    case FPCR_PREC_S: /* single */
        set_floatx80_rounding_precision(floatx80_precision_s, &env->fp_status);
        break;
    case FPCR_PREC_D: /* double */
        set_floatx80_rounding_precision(floatx80_precision_d, &env->fp_status);
        break;
    case FPCR_PREC_U: /* undefined */
    default:
        break;
    }
}

static void cf_restore_precision_mode(CPUM68KState *env)
{
    if (env->fpcr & FPCR_PREC_S) { /* single */
        set_floatx80_rounding_precision(floatx80_precision_s, &env->fp_status);
    } else { /* double */
        set_floatx80_rounding_precision(floatx80_precision_d, &env->fp_status);
    }
}

static void restore_rounding_mode(CPUM68KState *env)
{
    switch (env->fpcr & FPCR_RND_MASK) {
    case FPCR_RND_N: /* round to nearest */
        set_float_rounding_mode(float_round_nearest_even, &env->fp_status);
        break;
    case FPCR_RND_Z: /* round to zero */
        set_float_rounding_mode(float_round_to_zero, &env->fp_status);
        break;
    case FPCR_RND_M: /* round toward minus infinity */
        set_float_rounding_mode(float_round_down, &env->fp_status);
        break;
    case FPCR_RND_P: /* round toward positive infinity */
        set_float_rounding_mode(float_round_up, &env->fp_status);
        break;
    }
}

void cpu_m68k_restore_fp_status(CPUM68KState *env)
{
    if (m68k_feature(env, M68K_FEATURE_CF_FPU)) {
        cf_restore_precision_mode(env);
    } else {
        m68k_restore_precision_mode(env);
    }
    restore_rounding_mode(env);
}

void cpu_m68k_set_fpcr(CPUM68KState *env, uint32_t val)
{
    env->fpcr = val & 0xffff;
    cpu_m68k_restore_fp_status(env);
}

void HELPER(fitrunc)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    FloatRoundMode rounding_mode = get_float_rounding_mode(&env->fp_status);
    set_float_rounding_mode(float_round_to_zero, &env->fp_status);
    res->d = floatx80_round_to_int(val->d, &env->fp_status);
    set_float_rounding_mode(rounding_mode, &env->fp_status);
}

void HELPER(set_fpcr)(CPUM68KState *env, uint32_t val)
{
    cpu_m68k_set_fpcr(env, val);
}

/* Convert host exception flags to cpu_m68k form.  */
static int cpu_m68k_exceptbits_from_host(int host_bits)
{
    int target_bits = 0;

    if (host_bits & float_flag_invalid) {
        target_bits |= 0x80;
    }
    if (host_bits & float_flag_overflow) {
        target_bits |= 0x40;
    }
    if (host_bits & (float_flag_underflow | float_flag_output_denormal_flushed)) {
        target_bits |= 0x20;
    }
    if (host_bits & float_flag_divbyzero) {
        target_bits |= 0x10;
    }
    if (host_bits & float_flag_inexact) {
        target_bits |= 0x08;
    }
    return target_bits;
}

/* Convert cpu_m68k exception flags to target form.  */
static int cpu_m68k_exceptbits_to_host(int target_bits)
{
    int host_bits = 0;

    if (target_bits & 0x80) {
        host_bits |= float_flag_invalid;
    }
    if (target_bits & 0x40) {
        host_bits |= float_flag_overflow;
    }
    if (target_bits & 0x20) {
        host_bits |= float_flag_underflow;
    }
    if (target_bits & 0x10) {
        host_bits |= float_flag_divbyzero;
    }
    if (target_bits & 0x08) {
        host_bits |= float_flag_inexact;
    }
    return host_bits;
}

uint32_t cpu_m68k_get_fpsr(CPUM68KState *env)
{
    int host_flags = get_float_exception_flags(&env->fp_status);
    int target_flags = cpu_m68k_exceptbits_from_host(host_flags);

    /*
     * Some architectural accrued exceptions, notably BSUN, have no
     * softfloat equivalent and live only in env->fpsr.
     */
    return env->fpsr | target_flags;
}

uint32_t HELPER(get_fpsr)(CPUM68KState *env)
{
    return cpu_m68k_get_fpsr(env);
}

static void make_busy_fp_state(CPUM68KState *env, uint32_t pc)
{
    memset(env->fp_state, 0, sizeof(env->fp_state));
    env->fp_state[0] = 0x40;
    env->fp_state[1] = 0x60;
    stl_be_p(&env->fp_state[36], cpu_m68k_get_fpsr(env));
    stl_be_p(&env->fp_state[40], pc);
    env->fp_state_size = 100;
}

static void deliver_pending_fp_exception(CPUM68KState *env, uintptr_t ra)
{
    CPUState *cs;
    unsigned vector = env->fp_pending_vector;

    if (!vector) {
        return;
    }

    env->fp_pending_vector = 0;
    cs = env_cpu(env);
    cs->exception_index = vector;
    cpu_loop_exit_restore(cs, ra);
}

void HELPER(fpu_check_pending)(CPUM68KState *env, uint32_t pc)
{
    (void)pc;
    deliver_pending_fp_exception(env, GETPC());
}

void HELPER(fpu_begin)(CPUM68KState *env, uint32_t pc)
{
    if (env->fp_state_size == 4) {
        env->fp_state_size = 0;
    }
    env->fpiar = pc;
    env->fpsr &= ~FPSR_EXC_MASK;
    set_float_exception_flags(0, &env->fp_status);
}

static uint32_t fp_exc_from_softfloat(int flags)
{
    uint32_t exc = 0;

    if (flags & float_flag_invalid_snan) {
        exc |= FPSR_EXC_SNAN;
    } else if (flags & float_flag_invalid) {
        exc |= FPSR_EXC_OPERR;
    }
    if (flags & float_flag_overflow) {
        exc |= FPSR_EXC_OVFL;
    }
    if (flags & float_flag_underflow) {
        exc |= FPSR_EXC_UNFL;
    }
    if (flags & float_flag_divbyzero) {
        exc |= FPSR_EXC_DZ;
    }
    if (flags & float_flag_inexact) {
        exc |= FPSR_EXC_INEX2;
    }
    return exc;
}

static uint32_t fp_aexc_from_exc(uint32_t exc)
{
    uint32_t aexc = 0;

    if (exc & (FPSR_EXC_SNAN | FPSR_EXC_OPERR)) {
        aexc |= FPSR_AEXC_IOP;
    }
    if (exc & FPSR_EXC_OVFL) {
        aexc |= FPSR_AEXC_OVFL;
    }
    if ((exc & (FPSR_EXC_UNFL | FPSR_EXC_INEX2)) ==
        (FPSR_EXC_UNFL | FPSR_EXC_INEX2)) {
        aexc |= FPSR_AEXC_UNFL;
    }
    if (exc & FPSR_EXC_DZ) {
        aexc |= FPSR_AEXC_DZ;
    }
    if (exc & (FPSR_EXC_INEX1 | FPSR_EXC_INEX2)) {
        aexc |= FPSR_AEXC_INEX;
    }
    return aexc;
}

static unsigned enabled_fp_exception(uint32_t enabled)
{
    if (enabled & FPSR_EXC_SNAN) {
        return EXCP_FP_SNAN;
    }
    if (enabled & FPSR_EXC_OPERR) {
        return EXCP_FP_OPERR;
    }
    if (enabled & FPSR_EXC_OVFL) {
        return EXCP_FP_OVFL;
    }
    if (enabled & FPSR_EXC_UNFL) {
        return EXCP_FP_UNFL;
    }
    if (enabled & FPSR_EXC_DZ) {
        return EXCP_FP_DZ;
    }
    if (enabled & (FPSR_EXC_INEX1 | FPSR_EXC_INEX2)) {
        return EXCP_FP_INEX;
    }
    return 0;
}

void HELPER(fpu_finish)(CPUM68KState *env, uint32_t pc)
{
    int flags = get_float_exception_flags(&env->fp_status);
    uint32_t exc = fp_exc_from_softfloat(flags);
    unsigned vector;

    env->fpsr |= exc | fp_aexc_from_exc(exc);
    vector = enabled_fp_exception(exc & env->fpcr);
    if (vector) {
        env->fp_pending_vector = vector;
        env->fp_pending_pc = pc;
        make_busy_fp_state(env, pc);
    }
}

void HELPER(fcc_check)(CPUM68KState *env, uint32_t cond, uint32_t pc)
{
    CPUState *cs;

    deliver_pending_fp_exception(env, GETPC());

    if (!(cond & 0x10) || !(env->fpsr & FPSR_CC_A)) {
        return;
    }

    env->fpsr |= FPSR_EXC_BSUN | FPSR_AEXC_IOP;
    if (!(env->fpcr & FPSR_EXC_BSUN)) {
        return;
    }

    make_busy_fp_state(env, pc);
    cs = env_cpu(env);
    cs->exception_index = EXCP_FP_BSUN;
    cpu_loop_exit_restore(cs, GETPC());
}

void cpu_m68k_set_fpsr(CPUM68KState *env, uint32_t val)
{
    env->fpsr = val;

    int host_flags = cpu_m68k_exceptbits_to_host((int) env->fpsr);
    set_float_exception_flags(host_flags, &env->fp_status);
}

void HELPER(set_fpsr)(CPUM68KState *env, uint32_t val)
{
    cpu_m68k_set_fpsr(env, val);
}

#define PREC_BEGIN(prec)                                        \
    do {                                                        \
        FloatX80RoundPrec old =                                 \
            get_floatx80_rounding_precision(&env->fp_status);   \
        set_floatx80_rounding_precision(prec, &env->fp_status)  \

#define PREC_END()                                              \
        set_floatx80_rounding_precision(old, &env->fp_status);  \
    } while (0)

void HELPER(fsround)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    PREC_BEGIN(floatx80_precision_s);
    res->d = floatx80_round(val->d, &env->fp_status);
    PREC_END();
}

void HELPER(fdround)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    PREC_BEGIN(floatx80_precision_d);
    res->d = floatx80_round(val->d, &env->fp_status);
    PREC_END();
}

void HELPER(fsqrt)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_sqrt(val->d, &env->fp_status);
}

void HELPER(fssqrt)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    PREC_BEGIN(floatx80_precision_s);
    res->d = floatx80_sqrt(val->d, &env->fp_status);
    PREC_END();
}

void HELPER(fdsqrt)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    PREC_BEGIN(floatx80_precision_d);
    res->d = floatx80_sqrt(val->d, &env->fp_status);
    PREC_END();
}

void HELPER(fabs)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_round(floatx80_abs(val->d), &env->fp_status);
}

void HELPER(fsabs)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    PREC_BEGIN(floatx80_precision_s);
    res->d = floatx80_round(floatx80_abs(val->d), &env->fp_status);
    PREC_END();
}

void HELPER(fdabs)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    PREC_BEGIN(floatx80_precision_d);
    res->d = floatx80_round(floatx80_abs(val->d), &env->fp_status);
    PREC_END();
}

void HELPER(fneg)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_round(floatx80_chs(val->d), &env->fp_status);
}

void HELPER(fsneg)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    PREC_BEGIN(floatx80_precision_s);
    res->d = floatx80_round(floatx80_chs(val->d), &env->fp_status);
    PREC_END();
}

void HELPER(fdneg)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    PREC_BEGIN(floatx80_precision_d);
    res->d = floatx80_round(floatx80_chs(val->d), &env->fp_status);
    PREC_END();
}

void HELPER(fadd)(CPUM68KState *env, FPReg *res, FPReg *val0, FPReg *val1)
{
    res->d = floatx80_add(val0->d, val1->d, &env->fp_status);
}

void HELPER(fsadd)(CPUM68KState *env, FPReg *res, FPReg *val0, FPReg *val1)
{
    PREC_BEGIN(floatx80_precision_s);
    res->d = floatx80_add(val0->d, val1->d, &env->fp_status);
    PREC_END();
}

void HELPER(fdadd)(CPUM68KState *env, FPReg *res, FPReg *val0, FPReg *val1)
{
    PREC_BEGIN(floatx80_precision_d);
    res->d = floatx80_add(val0->d, val1->d, &env->fp_status);
    PREC_END();
}

void HELPER(fsub)(CPUM68KState *env, FPReg *res, FPReg *val0, FPReg *val1)
{
    res->d = floatx80_sub(val1->d, val0->d, &env->fp_status);
}

void HELPER(fssub)(CPUM68KState *env, FPReg *res, FPReg *val0, FPReg *val1)
{
    PREC_BEGIN(floatx80_precision_s);
    res->d = floatx80_sub(val1->d, val0->d, &env->fp_status);
    PREC_END();
}

void HELPER(fdsub)(CPUM68KState *env, FPReg *res, FPReg *val0, FPReg *val1)
{
    PREC_BEGIN(floatx80_precision_d);
    res->d = floatx80_sub(val1->d, val0->d, &env->fp_status);
    PREC_END();
}

void HELPER(fmul)(CPUM68KState *env, FPReg *res, FPReg *val0, FPReg *val1)
{
    res->d = floatx80_mul(val0->d, val1->d, &env->fp_status);
}

void HELPER(fsmul)(CPUM68KState *env, FPReg *res, FPReg *val0, FPReg *val1)
{
    PREC_BEGIN(floatx80_precision_s);
    res->d = floatx80_mul(val0->d, val1->d, &env->fp_status);
    PREC_END();
}

void HELPER(fdmul)(CPUM68KState *env, FPReg *res, FPReg *val0, FPReg *val1)
{
    PREC_BEGIN(floatx80_precision_d);
    res->d = floatx80_mul(val0->d, val1->d, &env->fp_status);
    PREC_END();
}

void HELPER(fsglmul)(CPUM68KState *env, FPReg *res, FPReg *val0, FPReg *val1)
{
    FloatRoundMode rounding_mode = get_float_rounding_mode(&env->fp_status);
    floatx80 a, b;

    PREC_BEGIN(floatx80_precision_s);
    set_float_rounding_mode(float_round_to_zero, &env->fp_status);
    a = floatx80_round(val0->d, &env->fp_status);
    b = floatx80_round(val1->d, &env->fp_status);
    set_float_rounding_mode(rounding_mode, &env->fp_status);
    res->d = floatx80_mul(a, b, &env->fp_status);
    PREC_END();
}

void HELPER(fdiv)(CPUM68KState *env, FPReg *res, FPReg *val0, FPReg *val1)
{
    res->d = floatx80_div(val1->d, val0->d, &env->fp_status);
}

void HELPER(fsdiv)(CPUM68KState *env, FPReg *res, FPReg *val0, FPReg *val1)
{
    PREC_BEGIN(floatx80_precision_s);
    res->d = floatx80_div(val1->d, val0->d, &env->fp_status);
    PREC_END();
}

void HELPER(fddiv)(CPUM68KState *env, FPReg *res, FPReg *val0, FPReg *val1)
{
    PREC_BEGIN(floatx80_precision_d);
    res->d = floatx80_div(val1->d, val0->d, &env->fp_status);
    PREC_END();
}

void HELPER(fsgldiv)(CPUM68KState *env, FPReg *res, FPReg *val0, FPReg *val1)
{
    FloatRoundMode rounding_mode = get_float_rounding_mode(&env->fp_status);
    floatx80 a, b;

    PREC_BEGIN(floatx80_precision_s);
    set_float_rounding_mode(float_round_to_zero, &env->fp_status);
    a = floatx80_round(val1->d, &env->fp_status);
    b = floatx80_round(val0->d, &env->fp_status);
    set_float_rounding_mode(rounding_mode, &env->fp_status);
    res->d = floatx80_div(a, b, &env->fp_status);
    PREC_END();
}

static int float_comp_to_cc(FloatRelation float_compare)
{
    switch (float_compare) {
    case float_relation_equal:
        return FPSR_CC_Z;
    case float_relation_less:
        return FPSR_CC_N;
    case float_relation_unordered:
        return FPSR_CC_A;
    case float_relation_greater:
        return 0;
    default:
        g_assert_not_reached();
    }
}

void HELPER(fcmp)(CPUM68KState *env, FPReg *val0, FPReg *val1)
{
    FloatRelation float_compare;

    float_compare = floatx80_compare_quiet(val1->d, val0->d,
                                           &env->fp_status);
    env->fpsr = (env->fpsr & ~FPSR_CC_MASK) | float_comp_to_cc(float_compare);
}

void HELPER(ftst)(CPUM68KState *env, FPReg *val)
{
    uint32_t cc = 0;

    if (floatx80_is_neg(val->d)) {
        cc |= FPSR_CC_N;
    }

    if (floatx80_is_any_nan(val->d)) {
        cc |= FPSR_CC_A;
    } else if (floatx80_is_infinity(val->d, &env->fp_status)) {
        cc |= FPSR_CC_I;
    } else if (floatx80_is_zero(val->d)) {
        cc |= FPSR_CC_Z;
    }
    env->fpsr = (env->fpsr & ~FPSR_CC_MASK) | cc;
}

void HELPER(fconst)(CPUM68KState *env, FPReg *val, uint32_t offset)
{
    val->d = fpu_rom[offset];
}

typedef int (*float_access)(CPUM68KState *env, uint32_t addr, FPReg *fp,
                            uintptr_t ra);

static uint32_t fmovem_predec(CPUM68KState *env, uint32_t addr, uint32_t mask,
                              float_access access_fn)
{
    uintptr_t ra = GETPC();
    int i, size;

    for (i = 7; i >= 0; i--, mask <<= 1) {
        if (mask & 0x80) {
            size = access_fn(env, addr, &env->fregs[i], ra);
            if ((mask & 0xff) != 0x80) {
                addr -= size;
            }
        }
    }

    return addr;
}

static uint32_t fmovem_postinc(CPUM68KState *env, uint32_t addr, uint32_t mask,
                               float_access access_fn)
{
    uintptr_t ra = GETPC();
    int i, size;

    for (i = 0; i < 8; i++, mask <<= 1) {
        if (mask & 0x80) {
            size = access_fn(env, addr, &env->fregs[i], ra);
            addr += size;
        }
    }

    return addr;
}

static int cpu_ld_floatx80_ra(CPUM68KState *env, uint32_t addr, FPReg *fp,
                              uintptr_t ra)
{
    uint32_t high;
    uint64_t low;

    high = cpu_ldl_be_data_ra(env, addr, ra);
    low = cpu_ldq_be_data_ra(env, addr + 4, ra);

    fp->l.upper = high >> 16;
    fp->l.lower = low;

    return 12;
}

static int cpu_st_floatx80_ra(CPUM68KState *env, uint32_t addr, FPReg *fp,
                               uintptr_t ra)
{
    cpu_stl_be_data_ra(env, addr, fp->l.upper << 16, ra);
    cpu_stq_be_data_ra(env, addr + 4, fp->l.lower, ra);

    return 12;
}

static int cpu_ld_float64_ra(CPUM68KState *env, uint32_t addr, FPReg *fp,
                             uintptr_t ra)
{
    uint64_t val;

    val = cpu_ldq_be_data_ra(env, addr, ra);
    fp->d = float64_to_floatx80(*(float64 *)&val, &env->fp_status);

    return 8;
}

static int cpu_st_float64_ra(CPUM68KState *env, uint32_t addr, FPReg *fp,
                             uintptr_t ra)
{
    float64 val;

    val = floatx80_to_float64(fp->d, &env->fp_status);
    cpu_stq_be_data_ra(env, addr, *(uint64_t *)&val, ra);

    return 8;
}

uint32_t HELPER(fmovemx_st_predec)(CPUM68KState *env, uint32_t addr,
                                   uint32_t mask)
{
    return fmovem_predec(env, addr, mask, cpu_st_floatx80_ra);
}

uint32_t HELPER(fmovemx_st_postinc)(CPUM68KState *env, uint32_t addr,
                                    uint32_t mask)
{
    return fmovem_postinc(env, addr, mask, cpu_st_floatx80_ra);
}

uint32_t HELPER(fmovemx_ld_postinc)(CPUM68KState *env, uint32_t addr,
                                    uint32_t mask)
{
    return fmovem_postinc(env, addr, mask, cpu_ld_floatx80_ra);
}

uint32_t HELPER(fmovemd_st_predec)(CPUM68KState *env, uint32_t addr,
                                   uint32_t mask)
{
    return fmovem_predec(env, addr, mask, cpu_st_float64_ra);
}

uint32_t HELPER(fmovemd_st_postinc)(CPUM68KState *env, uint32_t addr,
                                    uint32_t mask)
{
    return fmovem_postinc(env, addr, mask, cpu_st_float64_ra);
}

uint32_t HELPER(fmovemd_ld_postinc)(CPUM68KState *env, uint32_t addr,
                                    uint32_t mask)
{
    return fmovem_postinc(env, addr, mask, cpu_ld_float64_ra);
}

static void make_quotient(CPUM68KState *env, int sign, uint32_t quotient)
{
    quotient = (sign << 7) | (quotient & 0x7f);
    env->fpsr = (env->fpsr & ~FPSR_QT_MASK) | (quotient << FPSR_QT_SHIFT);
}

void HELPER(fmod)(CPUM68KState *env, FPReg *res, FPReg *val0, FPReg *val1)
{
    uint64_t quotient;
    int sign = extractFloatx80Sign(val1->d) ^ extractFloatx80Sign(val0->d);

    res->d = floatx80_modrem(val1->d, val0->d, true, &quotient,
                             &env->fp_status);

    if (floatx80_is_any_nan(res->d)) {
        return;
    }

    make_quotient(env, sign, quotient);
}

void HELPER(frem)(CPUM68KState *env, FPReg *res, FPReg *val0, FPReg *val1)
{
    FPReg fp_quot;
    floatx80 fp_rem;

    fp_rem = floatx80_rem(val1->d, val0->d, &env->fp_status);
    if (!floatx80_is_any_nan(fp_rem)) {
        /* Use local temporary fp_status to set different rounding mode */
        float_status fp_status = env->fp_status;
        uint32_t quotient;
        int sign;

        /* Calculate quotient directly using round to nearest mode */
        set_float_rounding_mode(float_round_nearest_even, &fp_status);
        fp_quot.d = floatx80_div(val1->d, val0->d, &fp_status);

        sign = extractFloatx80Sign(fp_quot.d);
        quotient = floatx80_to_int32(floatx80_abs(fp_quot.d), &env->fp_status);
        make_quotient(env, sign, quotient);
    }

    res->d = fp_rem;
}

void HELPER(fgetexp)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_getexp(val->d, &env->fp_status);
}

void HELPER(fgetman)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_getman(val->d, &env->fp_status);
}

void HELPER(fscale)(CPUM68KState *env, FPReg *res, FPReg *val0, FPReg *val1)
{
    res->d = floatx80_scale(val1->d, val0->d, &env->fp_status);
}

void HELPER(flognp1)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_lognp1(val->d, &env->fp_status);
}

void HELPER(flogn)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_logn(val->d, &env->fp_status);
}

void HELPER(flog10)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_log10(val->d, &env->fp_status);
}

void HELPER(flog2)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_log2(val->d, &env->fp_status);
}

void HELPER(fetox)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_etox(val->d, &env->fp_status);
}

void HELPER(ftwotox)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_twotox(val->d, &env->fp_status);
}

void HELPER(ftentox)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_tentox(val->d, &env->fp_status);
}

void HELPER(ftan)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_tan(val->d, &env->fp_status);
}

void HELPER(fsin)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_sin(val->d, &env->fp_status);
}

void HELPER(fcos)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_cos(val->d, &env->fp_status);
}

void HELPER(fsincos)(CPUM68KState *env, FPReg *res0, FPReg *res1, FPReg *val)
{
    floatx80 a = val->d;
    /*
     * If res0 and res1 specify the same floating-point data register,
     * the sine result is stored in the register, and the cosine
     * result is discarded.
     */
    res1->d = floatx80_cos(a, &env->fp_status);
    res0->d = floatx80_sin(a, &env->fp_status);
}

void HELPER(fatan)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_atan(val->d, &env->fp_status);
}

void HELPER(fasin)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_asin(val->d, &env->fp_status);
}

void HELPER(facos)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_acos(val->d, &env->fp_status);
}

void HELPER(fatanh)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_atanh(val->d, &env->fp_status);
}

void HELPER(fetoxm1)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_etoxm1(val->d, &env->fp_status);
}

void HELPER(ftanh)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_tanh(val->d, &env->fp_status);
}

void HELPER(fsinh)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_sinh(val->d, &env->fp_status);
}

void HELPER(fcosh)(CPUM68KState *env, FPReg *res, FPReg *val)
{
    res->d = floatx80_cosh(val->d, &env->fp_status);
}
