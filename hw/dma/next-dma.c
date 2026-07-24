/* SPDX-License-Identifier: NCSA
 *
 * Copyright (c) 2011-2026 Bryce Lanham
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal with the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to
 * whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimers.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimers in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the names of the University of Illinois/NCSA nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * Software without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS WITH THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "hw/dma/next-dma.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/isa/isa.h"
#include "migration/vmstate.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "trace.h"

#define NEXT_DMA_MMIO_BASE        0x02000000
#define NEXT_DMA_MMIO_SIZE        0x5000

#define NEXT_DMA_REG_CSR          0x0000
#define NEXT_DMA_REG_SAVED_NEXT   0x3ff0
#define NEXT_DMA_REG_SAVED_LIMIT  0x3ff4
#define NEXT_DMA_REG_SAVED_START  0x3ff8
#define NEXT_DMA_REG_SAVED_STOP   0x3ffc
#define NEXT_DMA_REG_NEXT         0x4000
#define NEXT_DMA_REG_LIMIT        0x4004
#define NEXT_DMA_REG_START        0x4008
#define NEXT_DMA_REG_STOP         0x400c
#define NEXT_DMA_REG_NEXT_INIT    0x4200

#define NEXT_DMA_CMD_SETENABLE    0x00010000
#define NEXT_DMA_CMD_SETSUPDATE   0x00020000
#define NEXT_DMA_CMD_READ         0x00040000
#define NEXT_DMA_CMD_CLRCOMPLETE  0x00080000
#define NEXT_DMA_CMD_RESET        0x00100000
#define NEXT_DMA_CMD_INITBUF      0x00200000

#define NEXT_DMA_CSR_ENABLE       0x01000000
#define NEXT_DMA_CSR_SUPDATE      0x02000000
#define NEXT_DMA_CSR_READ         0x04000000
#define NEXT_DMA_CSR_COMPLETE     0x08000000
#define NEXT_DMA_CSR_BUSEXC       0x10000000

#define NEXT_DMA_SCSI_BEAT        16
#define NEXT_DMA_SCSI_FLUSH_EDGES 4
#define NEXT_DMA_SCSI_DMAMODE     0x10
#define NEXT_DMA_SCSI_DMAREAD     0x08

#define NEXT_DMA_ENET_ADDR_MASK   0x0fffffff
#define NEXT_DMA_ENTX_EOP         0x80000000
#define NEXT_DMA_ENTX_END_BIAS    15
#define NEXT_DMA_ENTX_MIN_FRAME   60
#define NEXT_DMA_ENTX_MAX_FRAME   1514
#define NEXT_DMA_ENRX_BOP         0x40000000
#define NEXT_DMA_ENRX_EOP         0x80000000
#define NEXT_DMA_ENRX_MAX_FRAME   1518

#define NEXT_DMA_VIDEO_RETRACE_HZ 68
#define NEXT_DMA_VIDEO_RETRACE_NS \
    (NANOSECONDS_PER_SECOND / NEXT_DMA_VIDEO_RETRACE_HZ)

typedef enum NextDMASavedCapability {
    NEXT_DMA_SAVED_NONE,
    NEXT_DMA_SAVED_TWO,
    NEXT_DMA_SAVED_FOUR,
} NextDMASavedCapability;

typedef enum NextDMATransferPolicy {
    NEXT_DMA_TRANSFER_INERT,
    NEXT_DMA_TRANSFER_SCSI,
    NEXT_DMA_TRANSFER_SOUND_OUT,
    NEXT_DMA_TRANSFER_ENTX,
    NEXT_DMA_TRANSFER_ENRX,
} NextDMATransferPolicy;

typedef struct NextDMAChannelDesc {
    const char *name;
    hwaddr csr;
    int irq_bit;
    NextDMASavedCapability saved;
    NextDMATransferPolicy transfer;
} NextDMAChannelDesc;

static const NextDMAChannelDesc next_dma_channels[NEXT_DMA_CHANNEL_COUNT] = {
    /* The physical channel is shared by the SCSI and floppy controllers. */
    [NEXT_DMA_SCSI] = {
        "scsi", 0x010, 26, NEXT_DMA_SAVED_NONE, NEXT_DMA_TRANSFER_SCSI,
    },
    [NEXT_DMA_SOUND_OUT] = {
        "snd-out", 0x040, 23, NEXT_DMA_SAVED_NONE,
        NEXT_DMA_TRANSFER_SOUND_OUT,
    },
    [NEXT_DMA_OPTICAL] = {
        "optical", 0x050, 25, NEXT_DMA_SAVED_NONE, NEXT_DMA_TRANSFER_INERT,
    },
    [NEXT_DMA_SOUND_IN] = {
        "snd-in", 0x080, 22, NEXT_DMA_SAVED_NONE, NEXT_DMA_TRANSFER_INERT,
    },
    [NEXT_DMA_PRINTER] = {
        "printer", 0x090, 24, NEXT_DMA_SAVED_NONE, NEXT_DMA_TRANSFER_INERT,
    },
    [NEXT_DMA_SCC] = {
        "scc", 0x0c0, 21, NEXT_DMA_SAVED_NONE, NEXT_DMA_TRANSFER_INERT,
    },
    [NEXT_DMA_DSP] = {
        "dsp", 0x0d0, 20, NEXT_DMA_SAVED_NONE, NEXT_DMA_TRANSFER_INERT,
    },
    [NEXT_DMA_ENTX] = {
        "entx", 0x110, 28, NEXT_DMA_SAVED_FOUR, NEXT_DMA_TRANSFER_ENTX,
    },
    [NEXT_DMA_ENRX] = {
        "enrx", 0x150, 27, NEXT_DMA_SAVED_TWO, NEXT_DMA_TRANSFER_ENRX,
    },
    [NEXT_DMA_VIDEO] = {
        "video", 0x180, 5, NEXT_DMA_SAVED_NONE, NEXT_DMA_TRANSFER_INERT,
    },
    [NEXT_DMA_R2M] = {
        "r2m", 0x1c0, 18, NEXT_DMA_SAVED_NONE, NEXT_DMA_TRANSFER_INERT,
    },
    [NEXT_DMA_M2R] = {
        "m2r", 0x1d0, 19, NEXT_DMA_SAVED_NONE, NEXT_DMA_TRANSFER_INERT,
    },
};

typedef struct NextDMAChannelState {
    uint32_t csr;
    uint32_t saved_next;
    uint32_t saved_limit;
    uint32_t saved_start;
    uint32_t saved_stop;
    uint32_t next;
    uint32_t limit;
    uint32_t start;
    uint32_t stop;
    uint32_t next_initbuf;
    bool next_initbuf_valid;
    uint8_t scsi_stage[NEXT_DMA_SCSI_BEAT];
    uint8_t scsi_stage_len;
    uint8_t scsi_stage_flushes;
} NextDMAChannelState;

typedef struct NextDMATraceReadSampler {
    hwaddr addr;
    uint64_t value;
    uint64_t repeats;
} NextDMATraceReadSampler;

struct NextDMAState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    AddressSpace *as;
    NextDMAChannelState channel[NEXT_DMA_CHANNEL_COUNT];
    qemu_irq irq[NEXT_DMA_CHANNEL_COUNT];
    const NextDMAEthernetNotify *enet_notify;
    void *enet_opaque;
    bool rx_ready;
    bool rx_keep_enabled;
    QEMUTimer video_retrace_timer;
    NextDMATraceReadSampler trace_scsi_dma_read;
    IsaDmaTransferHandler floppy_transfer_handler;
    void *floppy_transfer_opaque;
    QEMUBH *floppy_bh;
    int32_t floppy_dma_position;
    int32_t floppy_callback_position;
    uint32_t floppy_callback_address;
    uint32_t floppy_callback_limit;
    uint8_t scsi_control;
    bool floppy_selected;
    bool floppy_dreq;
    bool floppy_in_callback;
    bool floppy_running;
    bool floppy_reschedule;
};

static void next_dma_floppy_schedule_request(NextDMAState *s);

static int next_dma_trace_int(size_t value)
{
    return value > INT_MAX ? INT_MAX : value;
}

static bool next_dma_trace_read_now(NextDMATraceReadSampler *sampler,
                                    hwaddr addr, uint64_t value)
{
    if (!sampler->repeats || sampler->addr != addr ||
        sampler->value != value) {
        sampler->addr = addr;
        sampler->value = value;
        sampler->repeats = 1;
    } else if (sampler->repeats < UINT64_MAX) {
        sampler->repeats++;
    }

    return (sampler->repeats & (sampler->repeats - 1)) == 0;
}

static void next_dma_update_irq(NextDMAState *s, NextDMAChannel channel)
{
    if (next_dma_channels[channel].irq_bit >= 0) {
        qemu_set_irq(s->irq[channel],
                     !!(s->channel[channel].csr &
                        NEXT_DMA_CSR_COMPLETE));
    }
}

static bool next_dma_video_retrace_enabled(const NextDMAState *s)
{
    return s->channel[NEXT_DMA_VIDEO].limit != 0;
}

static void next_dma_video_retrace_schedule(NextDMAState *s)
{
    if (!next_dma_video_retrace_enabled(s)) {
        timer_del(&s->video_retrace_timer);
        return;
    }
    if (!timer_pending(&s->video_retrace_timer)) {
        timer_mod(&s->video_retrace_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  NEXT_DMA_VIDEO_RETRACE_NS);
    }
}

static void next_dma_video_retrace(void *opaque)
{
    NextDMAState *s = opaque;
    NextDMAChannelState *video = &s->channel[NEXT_DMA_VIDEO];

    if (!next_dma_video_retrace_enabled(s)) {
        return;
    }

    video->csr |= NEXT_DMA_CSR_COMPLETE;
    next_dma_update_irq(s, NEXT_DMA_VIDEO);
    timer_mod(&s->video_retrace_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NEXT_DMA_VIDEO_RETRACE_NS);
}

typedef struct NextDMAEnetRxRange {
    uint32_t first_start;
    uint32_t first_limit;
    uint32_t second_start;
    uint32_t second_limit;
    size_t first_capacity;
    size_t second_capacity;
    bool chained;
    bool consume_next_initbuf;
} NextDMAEnetRxRange;

static bool next_dma_enrx_address_valid(uint32_t value)
{
    return !(value & ~NEXT_DMA_ENET_ADDR_MASK);
}

static bool next_dma_enrx_decode(const NextDMAChannelState *c,
                                 NextDMAEnetRxRange *range)
{
    memset(range, 0, sizeof(*range));
    range->first_start = c->next_initbuf_valid
                       ? c->next_initbuf : c->next;
    range->first_limit = c->limit;
    range->consume_next_initbuf = c->next_initbuf_valid;

    if (!next_dma_enrx_address_valid(range->first_start) ||
        !next_dma_enrx_address_valid(range->first_limit) ||
        range->first_limit < range->first_start) {
        return false;
    }
    range->first_capacity = range->first_limit - range->first_start;

    if (c->csr & NEXT_DMA_CSR_SUPDATE) {
        range->second_start = c->start;
        range->second_limit = c->stop;
        range->chained = true;
        if (!next_dma_enrx_address_valid(range->second_start) ||
            !next_dma_enrx_address_valid(range->second_limit) ||
            range->second_limit < range->second_start) {
            return false;
        }
        range->second_capacity =
            range->second_limit - range->second_start;
    }

    return range->first_capacity || range->second_capacity;
}

static bool next_dma_enrx_ready_state(NextDMAState *s)
{
    NextDMAChannelState *c = &s->channel[NEXT_DMA_ENRX];
    NextDMAEnetRxRange range;

    if (!(c->csr & NEXT_DMA_CSR_ENABLE) ||
        (c->csr & NEXT_DMA_CSR_COMPLETE) ||
        !next_dma_enrx_decode(c, &range)) {
        return false;
    }

    return (!range.first_capacity ||
            address_space_access_valid(s->as, range.first_start,
                                       range.first_capacity, true,
                                       MEMTXATTRS_UNSPECIFIED)) &&
           (!range.second_capacity ||
            address_space_access_valid(s->as, range.second_start,
                                       range.second_capacity, true,
                                       MEMTXATTRS_UNSPECIFIED));
}

static void next_dma_recompute_rx_ready(NextDMAState *s)
{
    bool ready = next_dma_enrx_ready_state(s);

    if (ready == s->rx_ready) {
        return;
    }
    s->rx_ready = ready;
    if (s->enet_notify && s->enet_notify->rx_ready_changed) {
        s->enet_notify->rx_ready_changed(s->enet_opaque, ready);
    }
}

static bool next_dma_complete_segment(NextDMAState *s,
                                      NextDMAChannel channel)
{
    NextDMAChannelState *c = &s->channel[channel];
    bool promote = c->csr & NEXT_DMA_CSR_SUPDATE;

    c->csr |= NEXT_DMA_CSR_COMPLETE;
    if (promote) {
        c->csr &= ~NEXT_DMA_CSR_SUPDATE;
        c->next = c->start;
        c->limit = c->stop;
    } else {
        c->csr &= ~NEXT_DMA_CSR_ENABLE;
    }
    next_dma_update_irq(s, channel);

    return promote;
}

static bool next_dma_advance(NextDMAState *s, NextDMAChannel channel,
                             uint32_t amount)
{
    NextDMAChannelState *c = &s->channel[channel];

    c->next += amount;
    if (c->next == c->limit) {
        return next_dma_complete_segment(s, channel);
    }
    return true;
}

typedef enum NextDMARegister {
    NEXT_DMA_REGISTER_CSR,
    NEXT_DMA_REGISTER_SAVED_NEXT,
    NEXT_DMA_REGISTER_SAVED_LIMIT,
    NEXT_DMA_REGISTER_SAVED_START,
    NEXT_DMA_REGISTER_SAVED_STOP,
    NEXT_DMA_REGISTER_NEXT,
    NEXT_DMA_REGISTER_LIMIT,
    NEXT_DMA_REGISTER_START,
    NEXT_DMA_REGISTER_STOP,
    NEXT_DMA_REGISTER_NEXT_INIT,
} NextDMARegister;

typedef struct NextDMAResolvedRegister {
    NextDMAChannel channel;
    NextDMARegister reg;
    uint32_t *value;
} NextDMAResolvedRegister;

static uint32_t *next_dma_channel_register(NextDMAChannelState *c,
                                           NextDMARegister reg)
{
    switch (reg) {
    case NEXT_DMA_REGISTER_CSR:
        return &c->csr;
    case NEXT_DMA_REGISTER_SAVED_NEXT:
        return &c->saved_next;
    case NEXT_DMA_REGISTER_SAVED_LIMIT:
        return &c->saved_limit;
    case NEXT_DMA_REGISTER_SAVED_START:
        return &c->saved_start;
    case NEXT_DMA_REGISTER_SAVED_STOP:
        return &c->saved_stop;
    case NEXT_DMA_REGISTER_NEXT:
        return &c->next;
    case NEXT_DMA_REGISTER_LIMIT:
        return &c->limit;
    case NEXT_DMA_REGISTER_START:
        return &c->start;
    case NEXT_DMA_REGISTER_STOP:
        return &c->stop;
    case NEXT_DMA_REGISTER_NEXT_INIT:
        return &c->next_initbuf;
    default:
        g_assert_not_reached();
    }
}

static const char *next_dma_register_name(NextDMARegister reg)
{
    switch (reg) {
    case NEXT_DMA_REGISTER_CSR:
        return "csr";
    case NEXT_DMA_REGISTER_SAVED_NEXT:
        return "saved-next";
    case NEXT_DMA_REGISTER_SAVED_LIMIT:
        return "saved-limit";
    case NEXT_DMA_REGISTER_SAVED_START:
        return "saved-start";
    case NEXT_DMA_REGISTER_SAVED_STOP:
        return "saved-stop";
    case NEXT_DMA_REGISTER_NEXT:
        return "next";
    case NEXT_DMA_REGISTER_LIMIT:
        return "limit";
    case NEXT_DMA_REGISTER_START:
        return "start";
    case NEXT_DMA_REGISTER_STOP:
        return "stop";
    case NEXT_DMA_REGISTER_NEXT_INIT:
        return "next-init";
    default:
        g_assert_not_reached();
    }
}

static void next_dma_resolve(NextDMAState *s, NextDMAChannel channel,
                             NextDMARegister reg,
                             NextDMAResolvedRegister *resolved)
{
    resolved->channel = channel;
    resolved->reg = reg;
    resolved->value = next_dma_channel_register(&s->channel[channel], reg);
}

static unsigned next_dma_saved_word_count(NextDMASavedCapability capability)
{
    switch (capability) {
    case NEXT_DMA_SAVED_NONE:
        return 0;
    case NEXT_DMA_SAVED_TWO:
        return 2;
    case NEXT_DMA_SAVED_FOUR:
        return 4;
    default:
        g_assert_not_reached();
    }
}

static bool next_dma_resolve_register(NextDMAState *s, hwaddr addr,
                                      NextDMAResolvedRegister *resolved)
{
    static const struct {
        hwaddr offset;
        NextDMARegister reg;
    } current[] = {
        { NEXT_DMA_REG_NEXT, NEXT_DMA_REGISTER_NEXT },
        { NEXT_DMA_REG_LIMIT, NEXT_DMA_REGISTER_LIMIT },
        { NEXT_DMA_REG_START, NEXT_DMA_REGISTER_START },
        { NEXT_DMA_REG_STOP, NEXT_DMA_REGISTER_STOP },
    };
    static const struct {
        hwaddr offset;
        NextDMARegister reg;
    } saved[] = {
        { NEXT_DMA_REG_SAVED_NEXT, NEXT_DMA_REGISTER_SAVED_NEXT },
        { NEXT_DMA_REG_SAVED_LIMIT, NEXT_DMA_REGISTER_SAVED_LIMIT },
        { NEXT_DMA_REG_SAVED_START, NEXT_DMA_REGISTER_SAVED_START },
        { NEXT_DMA_REG_SAVED_STOP, NEXT_DMA_REGISTER_SAVED_STOP },
    };
    NextDMAChannel channel;
    size_t reg;

    /* Exact CSR words take precedence over every bank. */
    for (channel = 0; channel < NEXT_DMA_CHANNEL_COUNT; channel++) {
        if (addr == next_dma_channels[channel].csr + NEXT_DMA_REG_CSR) {
            next_dma_resolve(s, channel, NEXT_DMA_REGISTER_CSR, resolved);
            return true;
        }
    }

    /* Current registers are physical words and win saved-bank collisions. */
    for (channel = 0; channel < NEXT_DMA_CHANNEL_COUNT; channel++) {
        for (reg = 0; reg < ARRAY_SIZE(current); reg++) {
            if (addr == next_dma_channels[channel].csr +
                        current[reg].offset) {
                next_dma_resolve(s, channel, current[reg].reg, resolved);
                return true;
            }
        }
    }

    /* NEXT_INIT is a single physical word, not the start of a pair. */
    for (channel = 0; channel < NEXT_DMA_CHANNEL_COUNT; channel++) {
        if (addr == next_dma_channels[channel].csr +
                    NEXT_DMA_REG_NEXT_INIT) {
            next_dma_resolve(s, channel, NEXT_DMA_REGISTER_NEXT_INIT,
                             resolved);
            return true;
        }
    }

    /* Only ENTX and ENRX expose the saved capabilities in the descriptor. */
    for (channel = 0; channel < NEXT_DMA_CHANNEL_COUNT; channel++) {
        unsigned words =
            next_dma_saved_word_count(next_dma_channels[channel].saved);

        for (reg = 0; reg < words; reg++) {
            if (addr == next_dma_channels[channel].csr +
                        saved[reg].offset) {
                next_dma_resolve(s, channel, saved[reg].reg, resolved);
                return true;
            }
        }
    }

    return false;
}

static void next_dma_clear_staging(NextDMAChannelState *c)
{
    c->scsi_stage_len = 0;
    c->scsi_stage_flushes = 0;
}

void next_dma_scsi_fifo_reset(NextDMAState *s)
{
    next_dma_clear_staging(&s->channel[NEXT_DMA_SCSI]);
}

static void next_dma_write_csr(NextDMAState *s, NextDMAChannel channel,
                               uint32_t value)
{
    NextDMAChannelState *c = &s->channel[channel];

    if (value & NEXT_DMA_CMD_RESET) {
        c->csr &= ~(NEXT_DMA_CSR_ENABLE | NEXT_DMA_CSR_SUPDATE |
                    NEXT_DMA_CSR_COMPLETE | NEXT_DMA_CSR_BUSEXC);
        c->next_initbuf_valid = false;
    }
    if (value & NEXT_DMA_CMD_INITBUF) {
        next_dma_clear_staging(c);
    }
    if (value & NEXT_DMA_CMD_SETENABLE) {
        c->csr |= NEXT_DMA_CSR_ENABLE;
    }
    if (value & NEXT_DMA_CMD_SETSUPDATE) {
        c->csr |= NEXT_DMA_CSR_SUPDATE;
    }
    if (value & NEXT_DMA_CMD_CLRCOMPLETE) {
        c->csr &= ~NEXT_DMA_CSR_COMPLETE;
    }

    c->csr &= ~NEXT_DMA_CSR_READ;
    if (value & NEXT_DMA_CMD_READ) {
        c->csr |= NEXT_DMA_CSR_READ;
    }
    next_dma_update_irq(s, channel);
    if (channel == NEXT_DMA_ENRX) {
        next_dma_recompute_rx_ready(s);
    }

    if (channel == NEXT_DMA_ENTX &&
        (value & NEXT_DMA_CMD_SETENABLE) &&
        (c->csr & NEXT_DMA_CSR_ENABLE) &&
        s->enet_notify && s->enet_notify->tx_kick) {
        s->enet_notify->tx_kick(s->enet_opaque);
    }
}

static void next_dma_trace_scsi_register_write(NextDMAState *s, hwaddr addr,
                                               uint64_t value)
{
    NextDMAChannelState *scsi = &s->channel[NEXT_DMA_SCSI];

    trace_next_scsi_dma_reg_write(
        NEXT_DMA_MMIO_BASE + addr, value, scsi->csr, scsi->next,
        scsi->next_initbuf, scsi->limit, scsi->start, scsi->stop);
}

static void next_dma_write(void *opaque, hwaddr addr, uint64_t value,
                           unsigned int size)
{
    NextDMAState *s = NEXT_DMA(opaque);
    NextDMAResolvedRegister resolved;

    if (!next_dma_resolve_register(s, addr, &resolved)) {
        return;
    }

    if (resolved.reg == NEXT_DMA_REGISTER_CSR) {
        next_dma_write_csr(s, resolved.channel, value);
    } else {
        *resolved.value = value;
        if (resolved.reg == NEXT_DMA_REGISTER_NEXT_INIT) {
            s->channel[resolved.channel].next_initbuf_valid = true;
        }
        if (resolved.channel == NEXT_DMA_VIDEO &&
            resolved.reg == NEXT_DMA_REGISTER_LIMIT) {
            next_dma_video_retrace_schedule(s);
        }
    }

    if (resolved.channel == NEXT_DMA_SCSI) {
        next_dma_trace_scsi_register_write(s, addr, value);
        next_dma_floppy_schedule_request(s);
    }
    if (resolved.channel == NEXT_DMA_ENRX &&
        resolved.reg != NEXT_DMA_REGISTER_CSR) {
        next_dma_recompute_rx_ready(s);
    }
    trace_next_dma_reg_write(NEXT_DMA_MMIO_BASE + addr,
                             next_dma_channels[resolved.channel].name,
                             next_dma_register_name(resolved.reg), value);
}

static uint64_t next_dma_read(void *opaque, hwaddr addr, unsigned int size)
{
    NextDMAState *s = NEXT_DMA(opaque);
    NextDMAResolvedRegister resolved;
    uint64_t value;

    if (!next_dma_resolve_register(s, addr, &resolved)) {
        return 0;
    }
    value = *resolved.value;

    if (resolved.channel == NEXT_DMA_SCSI &&
        trace_event_get_state_backends(TRACE_NEXT_SCSI_DMA_REG_READ)) {
        NextDMAChannelState *scsi = &s->channel[NEXT_DMA_SCSI];
        hwaddr trace_addr = NEXT_DMA_MMIO_BASE + addr;

        if (next_dma_trace_read_now(&s->trace_scsi_dma_read,
                                    trace_addr, value)) {
            trace_next_scsi_dma_reg_read(
                trace_addr, value, scsi->csr, scsi->next,
                scsi->next_initbuf, scsi->limit, scsi->start, scsi->stop,
                s->trace_scsi_dma_read.repeats);
        }
    }

    return value;
}

static const MemoryRegionOps next_dma_ops = {
    .read = next_dma_read,
    .write = next_dma_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint32_t next_dma_begin_scsi_transfer(NextDMAChannelState *c)
{
    if (c->next_initbuf_valid) {
        c->next = c->next_initbuf;
        c->next_initbuf_valid = false;
    }
    return c->next;
}

static void next_dma_floppy_error(NextDMAState *s)
{
    NextDMAChannelState *c = &s->channel[NEXT_DMA_SCSI];

    c->csr |= NEXT_DMA_CSR_BUSEXC | NEXT_DMA_CSR_COMPLETE;
    c->csr &= ~(NEXT_DMA_CSR_ENABLE | NEXT_DMA_CSR_SUPDATE);
    next_dma_update_irq(s, NEXT_DMA_SCSI);
}

static bool next_dma_floppy_gates_open(const NextDMAState *s)
{
    const NextDMAChannelState *c = &s->channel[NEXT_DMA_SCSI];
    bool dma_read = c->csr & NEXT_DMA_CSR_READ;

    return s->floppy_dreq &&
           s->floppy_selected &&
           (s->scsi_control & NEXT_DMA_SCSI_DMAMODE) &&
           (!!(s->scsi_control & NEXT_DMA_SCSI_DMAREAD) == dma_read) &&
           (c->csr & NEXT_DMA_CSR_ENABLE) &&
           !(c->csr & NEXT_DMA_CSR_COMPLETE);
}

static void next_dma_floppy_schedule_request(NextDMAState *s)
{
    if (!s->floppy_dreq || !s->floppy_bh) {
        return;
    }
    if (s->floppy_running) {
        s->floppy_reschedule = true;
        return;
    }
    qemu_bh_schedule(s->floppy_bh);
}

void next_dma_set_scsi_control(NextDMAState *s, uint8_t control)
{
    s->scsi_control = control;
    next_dma_floppy_schedule_request(s);
}

void next_dma_set_floppy_selected(NextDMAState *s, bool selected)
{
    s->floppy_selected = selected;
    next_dma_floppy_schedule_request(s);
}

static bool next_dma_floppy_channel_valid(int nchan)
{
    return nchan == NEXT_DMA_SCSI;
}

static bool next_dma_floppy_has_autoinitialization(IsaDma *obj, int nchan)
{
    g_assert(next_dma_floppy_channel_valid(nchan));
    return false;
}

static int next_dma_floppy_memory(IsaDma *obj, int nchan, void *buf, int pos,
                                  int len, bool write)
{
    NextDMAState *s = NEXT_DMA(obj);
    NextDMAChannelState *c = &s->channel[NEXT_DMA_SCSI];
    uint64_t relative;
    uint64_t address;
    bool dma_read = c->csr & NEXT_DMA_CSR_READ;
    MemTxResult result;

    g_assert(next_dma_floppy_channel_valid(nchan));
    if (!s->floppy_in_callback || !buf || pos < s->floppy_callback_position ||
        len < 0 || write != dma_read) {
        next_dma_floppy_error(s);
        return 0;
    }

    relative = (uint32_t)(pos - s->floppy_callback_position);
    address = (uint64_t)s->floppy_callback_address + relative;
    if (relative > s->floppy_callback_limit - s->floppy_callback_address ||
        (uint64_t)len >
        s->floppy_callback_limit - s->floppy_callback_address - relative ||
        address > UINT32_MAX ||
        !address_space_access_valid(s->as, address, len, write,
                                    MEMTXATTRS_UNSPECIFIED)) {
        next_dma_floppy_error(s);
        return 0;
    }

    if (write) {
        result = address_space_write(s->as, address, MEMTXATTRS_UNSPECIFIED,
                                     buf, len);
    } else {
        result = address_space_read(s->as, address, MEMTXATTRS_UNSPECIFIED,
                                    buf, len);
    }
    if (result != MEMTX_OK) {
        next_dma_floppy_error(s);
        return 0;
    }
    return len;
}

static int next_dma_floppy_read_memory(IsaDma *obj, int nchan, void *buf,
                                       int pos, int len)
{
    return next_dma_floppy_memory(obj, nchan, buf, pos, len, false);
}

static int next_dma_floppy_write_memory(IsaDma *obj, int nchan, void *buf,
                                        int pos, int len)
{
    return next_dma_floppy_memory(obj, nchan, buf, pos, len, true);
}

static void next_dma_floppy_hold_dreq(IsaDma *obj, int nchan)
{
    NextDMAState *s = NEXT_DMA(obj);

    g_assert(next_dma_floppy_channel_valid(nchan));
    if (!s->floppy_dreq) {
        s->floppy_dma_position = 0;
        s->floppy_dreq = true;
    }
    next_dma_floppy_schedule_request(s);
}

static void next_dma_floppy_release_dreq(IsaDma *obj, int nchan)
{
    NextDMAState *s = NEXT_DMA(obj);

    g_assert(next_dma_floppy_channel_valid(nchan));
    s->floppy_dreq = false;
    s->floppy_dma_position = 0;
}

static void next_dma_floppy_schedule(IsaDma *obj)
{
    next_dma_floppy_schedule_request(NEXT_DMA(obj));
}

static void next_dma_floppy_register_channel(
    IsaDma *obj, int nchan, IsaDmaTransferHandler transfer_handler,
    void *opaque)
{
    NextDMAState *s = NEXT_DMA(obj);

    g_assert(next_dma_floppy_channel_valid(nchan));
    s->floppy_transfer_handler = transfer_handler;
    s->floppy_transfer_opaque = opaque;
}

static void next_dma_floppy_run(void *opaque)
{
    NextDMAState *s = opaque;
    NextDMAChannelState *c = &s->channel[NEXT_DMA_SCSI];
    uint32_t descriptor_start;
    uint32_t descriptor_length;
    int callback_start;
    int callback_end;
    int new_position;
    uint32_t moved;

    if (s->floppy_running) {
        s->floppy_reschedule = true;
        return;
    }
    s->floppy_running = true;
    s->floppy_reschedule = false;

    if (!next_dma_floppy_gates_open(s)) {
        goto out;
    }

    descriptor_start = c->next_initbuf_valid ? c->next_initbuf : c->next;
    if ((descriptor_start & 3) || (c->limit & 15) || (c->stop & 15) ||
        descriptor_start >= c->limit) {
        next_dma_floppy_error(s);
        goto out;
    }
    descriptor_length = c->limit - descriptor_start;
    callback_start = s->floppy_dma_position;
    if (!s->floppy_transfer_handler || callback_start < 0 ||
        descriptor_length > INT_MAX ||
        callback_start > INT_MAX - (int)descriptor_length ||
        !address_space_access_valid(s->as, descriptor_start,
                                    descriptor_length,
                                    c->csr & NEXT_DMA_CSR_READ,
                                    MEMTXATTRS_UNSPECIFIED)) {
        next_dma_floppy_error(s);
        goto out;
    }
    callback_end = callback_start + descriptor_length;

    c->next = descriptor_start;
    c->next_initbuf_valid = false;
    s->floppy_callback_position = callback_start;
    s->floppy_callback_address = descriptor_start;
    s->floppy_callback_limit = c->limit;
    s->floppy_in_callback = true;
    new_position = s->floppy_transfer_handler(s->floppy_transfer_opaque,
                                               NEXT_DMA_SCSI,
                                               callback_start,
                                               callback_end);
    s->floppy_in_callback = false;

    if (c->csr & NEXT_DMA_CSR_BUSEXC) {
        goto out;
    }
    if (new_position < callback_start || new_position > callback_end) {
        next_dma_floppy_error(s);
        goto out;
    }

    moved = new_position - callback_start;
    c->next += moved;
    if (s->floppy_dreq) {
        s->floppy_dma_position = new_position;
    }
    if (c->next == c->limit) {
        next_dma_complete_segment(s, NEXT_DMA_SCSI);
    }

out:
    s->floppy_in_callback = false;
    s->floppy_running = false;
    if (s->floppy_reschedule) {
        s->floppy_reschedule = false;
        next_dma_floppy_schedule_request(s);
    }
}

static bool next_dma_scsi_beat_fits(const NextDMAChannelState *c)
{
    return c->limit <= c->next ||
           c->limit - c->next >= NEXT_DMA_SCSI_BEAT;
}

void next_dma_scsi_write(NextDMAState *s, const uint8_t *buf, size_t len)
{
    NextDMAChannelState *c = &s->channel[NEXT_DMA_SCSI];
    size_t remaining = len;
    size_t committed = 0;
    uint32_t base;
    bool access_error = false;

    if (!(c->csr & NEXT_DMA_CSR_ENABLE)) {
        trace_next_scsi_dma_transfer(
            "disabled", next_dma_trace_int(len), 0, c->next, c->csr,
            c->next, c->next_initbuf, c->limit, c->saved_next,
            c->saved_limit);
        return;
    }

    base = next_dma_begin_scsi_transfer(c);
    trace_next_scsi_dma_transfer(
        "entry", next_dma_trace_int(len), 0, base, c->csr, c->next,
        c->next_initbuf, c->limit, c->saved_next, c->saved_limit);

    while (remaining) {
        size_t room;
        size_t copied;
        bool continue_segment;

        if (!next_dma_scsi_beat_fits(c)) {
            break;
        }
        room = NEXT_DMA_SCSI_BEAT - c->scsi_stage_len;
        copied = MIN(remaining, room);
        memcpy(c->scsi_stage + c->scsi_stage_len, buf, copied);
        c->scsi_stage_len += copied;
        c->scsi_stage_flushes = NEXT_DMA_SCSI_FLUSH_EDGES;
        buf += copied;
        remaining -= copied;

        if (c->scsi_stage_len != NEXT_DMA_SCSI_BEAT) {
            break;
        }
        if (address_space_write(s->as, c->next, MEMTXATTRS_UNSPECIFIED,
                                c->scsi_stage,
                                NEXT_DMA_SCSI_BEAT) != MEMTX_OK) {
            access_error = true;
            break;
        }

        c->scsi_stage_len = 0;
        c->scsi_stage_flushes = 0;
        committed += NEXT_DMA_SCSI_BEAT;
        continue_segment = next_dma_advance(s, NEXT_DMA_SCSI,
                                            NEXT_DMA_SCSI_BEAT);
        if (!continue_segment && remaining) {
            if (!c->scsi_stage_len &&
                remaining <= NEXT_DMA_SCSI_BEAT) {
                memcpy(c->scsi_stage, buf, remaining);
                c->scsi_stage_len = remaining;
                c->scsi_stage_flushes = NEXT_DMA_SCSI_FLUSH_EDGES;
                buf += remaining;
                remaining = 0;
            }
            break;
        }
    }

    trace_next_scsi_dma_transfer(
        access_error ? "error" : (c->scsi_stage_len ? "staged" : "complete"),
        next_dma_trace_int(len), next_dma_trace_int(committed), base, c->csr,
        c->next, c->next_initbuf, c->limit, c->saved_next, c->saved_limit);
}

void next_dma_scsi_read(NextDMAState *s, uint8_t *buf, size_t len)
{
    NextDMAChannelState *c = &s->channel[NEXT_DMA_SCSI];
    size_t remaining = len;
    size_t transferred = 0;
    uint32_t base;
    bool access_error = false;

    if (!(c->csr & NEXT_DMA_CSR_ENABLE)) {
        trace_next_scsi_dma_read(
            "disabled", next_dma_trace_int(len), 0, c->next, c->csr,
            c->next, c->next_initbuf, c->limit, c->saved_next,
            c->saved_limit);
        return;
    }

    base = next_dma_begin_scsi_transfer(c);
    trace_next_scsi_dma_read(
        "entry", next_dma_trace_int(len), 0, base, c->csr, c->next,
        c->next_initbuf, c->limit, c->saved_next, c->saved_limit);

    while (remaining) {
        size_t chunk = MIN(remaining, (size_t)NEXT_DMA_SCSI_BEAT);
        bool continue_segment;

        if (c->limit > c->next) {
            chunk = MIN(chunk, (size_t)(c->limit - c->next));
        }
        if (!chunk) {
            break;
        }
        if (address_space_read(s->as, c->next, MEMTXATTRS_UNSPECIFIED,
                               buf, chunk) != MEMTX_OK) {
            access_error = true;
            break;
        }

        buf += chunk;
        remaining -= chunk;
        transferred += chunk;
        continue_segment = next_dma_advance(s, NEXT_DMA_SCSI, chunk);
        if (!continue_segment && remaining) {
            break;
        }
    }

    trace_next_scsi_dma_read(
        access_error ? "error" : "complete", next_dma_trace_int(len),
        next_dma_trace_int(transferred), base, c->csr, c->next,
        c->next_initbuf, c->limit, c->saved_next, c->saved_limit);
}

void next_dma_scsi_fifo_flush(NextDMAState *s)
{
    NextDMAChannelState *c = &s->channel[NEXT_DMA_SCSI];
    uint8_t beat[NEXT_DMA_SCSI_BEAT] = { 0 };
    uint32_t base;
    uint8_t staged;

    if (!c->scsi_stage_len || !c->scsi_stage_flushes) {
        return;
    }
    if (c->scsi_stage_flushes > 1) {
        c->scsi_stage_flushes--;
        return;
    }
    if (!next_dma_scsi_beat_fits(c)) {
        return;
    }

    base = c->next;
    staged = c->scsi_stage_len;
    memcpy(beat, c->scsi_stage, staged);
    if (address_space_write(s->as, base, MEMTXATTRS_UNSPECIFIED,
                            beat, sizeof(beat)) != MEMTX_OK) {
        trace_next_scsi_dma_transfer(
            "error", staged, 0, base, c->csr, c->next, c->next_initbuf,
            c->limit, c->saved_next, c->saved_limit);
        return;
    }

    c->scsi_stage_len = 0;
    c->scsi_stage_flushes = 0;
    next_dma_advance(s, NEXT_DMA_SCSI, sizeof(beat));
    trace_next_scsi_dma_transfer(
        "flush", staged, sizeof(beat), base, c->csr, c->next,
        c->next_initbuf, c->limit, c->saved_next, c->saved_limit);
}

static NextDMAResult next_dma_sound_out_error(NextDMAState *s)
{
    NextDMAChannelState *c = &s->channel[NEXT_DMA_SOUND_OUT];

    c->csr |= NEXT_DMA_CSR_BUSEXC | NEXT_DMA_CSR_COMPLETE;
    c->csr &= ~(NEXT_DMA_CSR_ENABLE | NEXT_DMA_CSR_SUPDATE);
    next_dma_update_irq(s, NEXT_DMA_SOUND_OUT);
    return NEXT_DMA_RANGE_ERROR;
}

NextDMAResult next_dma_sound_out_read(NextDMAState *s, uint8_t *samples,
                                      size_t capacity, size_t *length)
{
    NextDMAChannelState *c = &s->channel[NEXT_DMA_SOUND_OUT];
    uint32_t start;
    size_t available;
    size_t chunk;

    *length = 0;
    if (!(c->csr & NEXT_DMA_CSR_ENABLE) ||
        (c->csr & NEXT_DMA_CSR_COMPLETE)) {
        return NEXT_DMA_NOT_READY;
    }
    if ((c->csr & NEXT_DMA_CSR_READ) || !samples || capacity < 4) {
        return next_dma_sound_out_error(s);
    }

    start = c->next_initbuf_valid ? c->next_initbuf : c->next;
    if ((start & 3) || (c->limit & 15) || c->limit <= start) {
        return next_dma_sound_out_error(s);
    }

    available = c->limit - start;
    chunk = MIN(available, capacity) & ~(size_t)3;
    if (!chunk ||
        !address_space_access_valid(s->as, start, chunk, false,
                                    MEMTXATTRS_UNSPECIFIED) ||
        address_space_read(s->as, start, MEMTXATTRS_UNSPECIFIED,
                           samples, chunk) != MEMTX_OK) {
        return next_dma_sound_out_error(s);
    }

    if (c->next_initbuf_valid) {
        c->next = start;
        c->next_initbuf_valid = false;
    }
    next_dma_advance(s, NEXT_DMA_SOUND_OUT, chunk);
    *length = chunk;
    return NEXT_DMA_OK;
}

typedef struct NextDMAEnetTxRange {
    uint32_t first_start;
    uint32_t first_end;
    uint32_t second_start;
    uint32_t second_end;
    uint32_t final_limit;
    size_t first_length;
    size_t second_length;
    size_t total_length;
    bool consume_next_initbuf;
} NextDMAEnetTxRange;

static bool next_dma_entx_eop_address_valid(uint32_t value)
{
    return (value & NEXT_DMA_ENTX_EOP) &&
           !(value & ~(NEXT_DMA_ENTX_EOP | NEXT_DMA_ENET_ADDR_MASK));
}

static bool next_dma_entx_decode(NextDMAChannelState *c,
                                 NextDMAEnetTxRange *range)
{
    uint32_t final_encoded;

    memset(range, 0, sizeof(*range));
    if (c->limit & NEXT_DMA_ENTX_EOP) {
        if (!next_dma_entx_eop_address_valid(c->limit)) {
            return false;
        }
        final_encoded = c->limit & NEXT_DMA_ENET_ADDR_MASK;
        if (final_encoded < NEXT_DMA_ENTX_END_BIAS) {
            return false;
        }

        range->first_start = c->next_initbuf_valid
                           ? c->next_initbuf
                           : c->next & NEXT_DMA_ENET_ADDR_MASK;
        if (range->first_start > NEXT_DMA_ENET_ADDR_MASK) {
            return false;
        }
        range->first_end = final_encoded - NEXT_DMA_ENTX_END_BIAS;
        if (range->first_end < range->first_start) {
            return false;
        }
        range->first_length = range->first_end - range->first_start;
        range->final_limit = c->limit;
        range->consume_next_initbuf = c->next_initbuf_valid;
    } else {
        if (c->limit & ~NEXT_DMA_ENET_ADDR_MASK ||
            !next_dma_entx_eop_address_valid(c->stop)) {
            return false;
        }
        final_encoded = c->stop & NEXT_DMA_ENET_ADDR_MASK;
        if (final_encoded < NEXT_DMA_ENTX_END_BIAS) {
            return false;
        }

        range->first_start = c->next & NEXT_DMA_ENET_ADDR_MASK;
        range->first_end = c->limit;
        range->second_start = c->start & NEXT_DMA_ENET_ADDR_MASK;
        range->second_end = final_encoded - NEXT_DMA_ENTX_END_BIAS;
        if (range->first_end < range->first_start ||
            range->second_end < range->second_start) {
            return false;
        }
        range->first_length = range->first_end - range->first_start;
        range->second_length = range->second_end - range->second_start;
        range->final_limit = c->stop;
    }

    if (range->first_length > SIZE_MAX - range->second_length) {
        return false;
    }
    range->total_length = range->first_length + range->second_length;
    return range->total_length >= NEXT_DMA_ENTX_MIN_FRAME &&
           range->total_length <= NEXT_DMA_ENTX_MAX_FRAME;
}

NextDMAResult next_dma_enet_tx_read(NextDMAState *s, uint8_t *frame,
                                    size_t capacity, size_t *length)
{
    NextDMAChannelState *c = &s->channel[NEXT_DMA_ENTX];
    NextDMAEnetTxRange range;

    *length = 0;
    if (!(c->csr & NEXT_DMA_CSR_ENABLE)) {
        return NEXT_DMA_RANGE_ERROR;
    }
    if (!next_dma_entx_decode(c, &range)) {
        return NEXT_DMA_RANGE_ERROR;
    }
    if (range.total_length > capacity) {
        return NEXT_DMA_NO_SPACE;
    }

    if (address_space_read(s->as, range.first_start,
                           MEMTXATTRS_UNSPECIFIED, frame,
                           range.first_length) != MEMTX_OK) {
        return NEXT_DMA_RANGE_ERROR;
    }
    if (range.second_length &&
        address_space_read(s->as, range.second_start,
                           MEMTXATTRS_UNSPECIFIED,
                           frame + range.first_length,
                           range.second_length) != MEMTX_OK) {
        return NEXT_DMA_RANGE_ERROR;
    }

    c->next = range.second_length ? range.second_end : range.first_end;
    c->limit = range.final_limit;
    if (range.consume_next_initbuf) {
        c->next_initbuf_valid = false;
    }
    *length = range.total_length;
    return NEXT_DMA_OK;
}

void next_dma_enet_tx_complete(NextDMAState *s, NextDMAResult result)
{
    NextDMAChannelState *c = &s->channel[NEXT_DMA_ENTX];

    if (result == NEXT_DMA_OK) {
        c->csr |= NEXT_DMA_CSR_COMPLETE;
        c->csr &= ~(NEXT_DMA_CSR_ENABLE | NEXT_DMA_CSR_SUPDATE);
        next_dma_update_irq(s, NEXT_DMA_ENTX);
        return;
    }

    c->csr |= NEXT_DMA_CSR_BUSEXC | NEXT_DMA_CSR_COMPLETE;
    c->csr &= ~(NEXT_DMA_CSR_ENABLE | NEXT_DMA_CSR_SUPDATE);
    next_dma_update_irq(s, NEXT_DMA_ENTX);
}

bool next_dma_enet_rx_ready(NextDMAState *s)
{
    return s->rx_ready;
}

NextDMAResult next_dma_enet_rx_write(NextDMAState *s,
                                     const uint8_t *frame_fcs,
                                     size_t length)
{
    NextDMAChannelState *c = &s->channel[NEXT_DMA_ENRX];
    NextDMAEnetRxRange range;
    size_t first_length;
    size_t second_length;
    uint32_t first_end;
    uint32_t final_end;

    s->rx_keep_enabled = false;
    if (!(c->csr & NEXT_DMA_CSR_ENABLE) ||
        (c->csr & NEXT_DMA_CSR_COMPLETE) ||
        !frame_fcs || !length || length > NEXT_DMA_ENRX_MAX_FRAME ||
        !next_dma_enrx_decode(c, &range)) {
        return NEXT_DMA_RANGE_ERROR;
    }

    first_length = MIN(length, range.first_capacity);
    second_length = length - first_length;
    if (second_length > range.second_capacity) {
        return NEXT_DMA_NO_SPACE;
    }
    first_end = range.first_start + first_length;
    final_end = second_length
              ? range.second_start + second_length : first_end;

    /*
     * Validate both guest ranges before the first write.  This preserves
     * the all-or-nothing receive contract for malformed DMA programming.
     */
    if ((first_length &&
         !address_space_access_valid(s->as, range.first_start,
                                     first_length, true,
                                     MEMTXATTRS_UNSPECIFIED)) ||
        (second_length &&
         !address_space_access_valid(s->as, range.second_start,
                                     second_length, true,
                                     MEMTXATTRS_UNSPECIFIED))) {
        return NEXT_DMA_RANGE_ERROR;
    }
    if ((first_length &&
         address_space_write(s->as, range.first_start,
                             MEMTXATTRS_UNSPECIFIED, frame_fcs,
                             first_length) != MEMTX_OK) ||
        (second_length &&
         address_space_write(s->as, range.second_start,
                             MEMTXATTRS_UNSPECIFIED,
                             frame_fcs + first_length,
                             second_length) != MEMTX_OK)) {
        return NEXT_DMA_RANGE_ERROR;
    }

    c->saved_next = range.first_start | NEXT_DMA_ENRX_BOP;
    if (second_length) {
        c->saved_limit = range.first_limit;
        c->next = final_end | NEXT_DMA_ENRX_EOP;
        c->limit = range.second_limit;
        c->csr &= ~NEXT_DMA_CSR_SUPDATE;
    } else {
        c->saved_limit = first_end | NEXT_DMA_ENRX_EOP;
        if (range.chained) {
            c->next = range.second_start;
            c->limit = range.second_limit;
            c->csr &= ~NEXT_DMA_CSR_SUPDATE;
            s->rx_keep_enabled = true;
        } else {
            c->next = first_end | NEXT_DMA_ENRX_EOP;
        }
    }
    if (range.consume_next_initbuf) {
        c->next_initbuf_valid = false;
    }

    return NEXT_DMA_OK;
}

void next_dma_enet_rx_complete(NextDMAState *s, NextDMAResult result)
{
    NextDMAChannelState *c = &s->channel[NEXT_DMA_ENRX];

    if (result == NEXT_DMA_OK) {
        c->csr |= NEXT_DMA_CSR_COMPLETE;
        c->csr &= ~NEXT_DMA_CSR_SUPDATE;
        if (!s->rx_keep_enabled) {
            c->csr &= ~NEXT_DMA_CSR_ENABLE;
        }
    } else {
        c->csr |= NEXT_DMA_CSR_BUSEXC | NEXT_DMA_CSR_COMPLETE;
        c->csr &= ~(NEXT_DMA_CSR_ENABLE | NEXT_DMA_CSR_SUPDATE);
    }
    s->rx_keep_enabled = false;
    next_dma_update_irq(s, NEXT_DMA_ENRX);
    next_dma_recompute_rx_ready(s);
}

void next_dma_set_ethernet_notify(NextDMAState *s,
                                  const NextDMAEthernetNotify *notify,
                                  void *opaque)
{
    s->enet_notify = notify;
    s->enet_opaque = opaque;
    if (notify) {
        next_dma_recompute_rx_ready(s);
    }
}

static void next_dma_reset_hold(Object *obj, ResetType type)
{
    NextDMAState *s = NEXT_DMA(obj);
    int channel;

    qemu_bh_cancel(s->floppy_bh);
    memset(s->channel, 0, sizeof(s->channel));
    memset(&s->trace_scsi_dma_read, 0, sizeof(s->trace_scsi_dma_read));
    s->rx_keep_enabled = false;
    s->floppy_dma_position = 0;
    s->floppy_callback_position = 0;
    s->floppy_callback_address = 0;
    s->floppy_callback_limit = 0;
    s->scsi_control = 0;
    s->floppy_selected = false;
    s->floppy_dreq = false;
    s->floppy_in_callback = false;
    s->floppy_running = false;
    s->floppy_reschedule = false;
    timer_del(&s->video_retrace_timer);

    for (channel = 0; channel < NEXT_DMA_CHANNEL_COUNT; channel++) {
        qemu_irq_lower(s->irq[channel]);
    }
    next_dma_recompute_rx_ready(s);
}

static int next_dma_post_load(void *opaque, int version_id)
{
    NextDMAState *s = opaque;
    int channel;

    if (s->floppy_dma_position < 0) {
        return -EINVAL;
    }
    for (channel = 0; channel < NEXT_DMA_CHANNEL_COUNT; channel++) {
        NextDMAChannelState *c = &s->channel[channel];

        if (c->scsi_stage_len > NEXT_DMA_SCSI_BEAT ||
            c->scsi_stage_flushes > NEXT_DMA_SCSI_FLUSH_EDGES ||
            (!!c->scsi_stage_len != !!c->scsi_stage_flushes)) {
            return -EINVAL;
        }
    }

    memset(&s->trace_scsi_dma_read, 0, sizeof(s->trace_scsi_dma_read));
    for (channel = 0; channel < NEXT_DMA_CHANNEL_COUNT; channel++) {
        next_dma_update_irq(s, channel);
    }
    if (next_dma_video_retrace_enabled(s) &&
        !timer_pending(&s->video_retrace_timer)) {
        next_dma_video_retrace_schedule(s);
    } else if (!next_dma_video_retrace_enabled(s)) {
        timer_del(&s->video_retrace_timer);
    }

    /* Host callbacks and their opaque are deliberately not VMState. */
    s->rx_ready = false;
    s->rx_keep_enabled = false;
    next_dma_recompute_rx_ready(s);
    s->floppy_callback_position = 0;
    s->floppy_callback_address = 0;
    s->floppy_callback_limit = 0;
    s->floppy_in_callback = false;
    s->floppy_running = false;
    s->floppy_reschedule = false;
    if (s->floppy_dreq && next_dma_floppy_gates_open(s)) {
        next_dma_floppy_schedule_request(s);
    }

    return 0;
}

static const VMStateDescription vmstate_next_dma_channel = {
    .name = "next-dma-channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(csr, NextDMAChannelState),
        VMSTATE_UINT32(saved_next, NextDMAChannelState),
        VMSTATE_UINT32(saved_limit, NextDMAChannelState),
        VMSTATE_UINT32(saved_start, NextDMAChannelState),
        VMSTATE_UINT32(saved_stop, NextDMAChannelState),
        VMSTATE_UINT32(next, NextDMAChannelState),
        VMSTATE_UINT32(limit, NextDMAChannelState),
        VMSTATE_UINT32(start, NextDMAChannelState),
        VMSTATE_UINT32(stop, NextDMAChannelState),
        VMSTATE_UINT32(next_initbuf, NextDMAChannelState),
        VMSTATE_BOOL(next_initbuf_valid, NextDMAChannelState),
        VMSTATE_UINT8_ARRAY(scsi_stage, NextDMAChannelState,
                            NEXT_DMA_SCSI_BEAT),
        VMSTATE_UINT8(scsi_stage_len, NextDMAChannelState),
        VMSTATE_UINT8(scsi_stage_flushes, NextDMAChannelState),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_next_dma = {
    .name = "next-dma",
    .priority = MIG_PRI_LOW,
    .version_id = 3,
    .minimum_version_id = 1,
    .post_load = next_dma_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(channel, NextDMAState, NEXT_DMA_CHANNEL_COUNT, 1,
                             vmstate_next_dma_channel,
                             NextDMAChannelState),
        VMSTATE_TIMER_V(video_retrace_timer, NextDMAState, 2),
        VMSTATE_INT32_V(floppy_dma_position, NextDMAState, 3),
        VMSTATE_UINT8_V(scsi_control, NextDMAState, 3),
        VMSTATE_BOOL_V(floppy_selected, NextDMAState, 3),
        VMSTATE_BOOL_V(floppy_dreq, NextDMAState, 3),
        VMSTATE_END_OF_LIST()
    },
};

static void next_dma_init(Object *obj)
{
    NextDMAState *s = NEXT_DMA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;

    s->as = &address_space_memory;
    memory_region_init_io(&s->mmio, obj, &next_dma_ops, s,
                          "next.dma", NEXT_DMA_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->mmio);
    timer_init_ns(&s->video_retrace_timer, QEMU_CLOCK_VIRTUAL,
                  next_dma_video_retrace, s);
    s->floppy_bh = qemu_bh_new(next_dma_floppy_run, s);

    for (i = 0; i < NEXT_DMA_CHANNEL_COUNT; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
}

static void next_dma_finalize(Object *obj)
{
    NextDMAState *s = NEXT_DMA(obj);

    qemu_bh_delete(s->floppy_bh);
}

static void next_dma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    IsaDmaClass *idc = ISADMA_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->vmsd = &vmstate_next_dma;
    rc->phases.hold = next_dma_reset_hold;
    idc->has_autoinitialization =
        next_dma_floppy_has_autoinitialization;
    idc->read_memory = next_dma_floppy_read_memory;
    idc->write_memory = next_dma_floppy_write_memory;
    idc->hold_DREQ = next_dma_floppy_hold_dreq;
    idc->release_DREQ = next_dma_floppy_release_dreq;
    idc->schedule = next_dma_floppy_schedule;
    idc->register_channel = next_dma_floppy_register_channel;
}

static const TypeInfo next_dma_info = {
    .name = TYPE_NEXT_DMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NextDMAState),
    .instance_init = next_dma_init,
    .instance_finalize = next_dma_finalize,
    .class_init = next_dma_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { TYPE_ISADMA },
        { }
    },
};

static void next_dma_register_types(void)
{
    type_register_static(&next_dma_info);
}

type_init(next_dma_register_types)
