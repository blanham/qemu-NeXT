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
#include "migration/vmstate.h"
#include "qemu/module.h"
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

typedef enum NextDMASavedCapability {
    NEXT_DMA_SAVED_NONE,
    NEXT_DMA_SAVED_TWO,
    NEXT_DMA_SAVED_FOUR,
} NextDMASavedCapability;

typedef enum NextDMATransferPolicy {
    NEXT_DMA_TRANSFER_INERT,
    NEXT_DMA_TRANSFER_SCSI,
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
    [NEXT_DMA_SCSI] = {
        "scsi", 0x010, 26, NEXT_DMA_SAVED_NONE, NEXT_DMA_TRANSFER_SCSI,
    },
    [NEXT_DMA_SOUND_OUT] = {
        "snd-out", 0x040, 23, NEXT_DMA_SAVED_NONE, NEXT_DMA_TRANSFER_INERT,
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
        "video", 0x180, -1, NEXT_DMA_SAVED_NONE, NEXT_DMA_TRANSFER_INERT,
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
    NextDMATraceReadSampler trace_scsi_dma_read;
};

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

static void next_dma_write_csr(NextDMAState *s, NextDMAChannel channel,
                               uint32_t value)
{
    NextDMAChannelState *c = &s->channel[channel];

    if (value & NEXT_DMA_CMD_RESET) {
        c->csr &= ~(NEXT_DMA_CSR_ENABLE | NEXT_DMA_CSR_SUPDATE |
                    NEXT_DMA_CSR_COMPLETE | NEXT_DMA_CSR_BUSEXC);
        next_dma_clear_staging(c);
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
    }

    if (resolved.channel == NEXT_DMA_SCSI) {
        next_dma_trace_scsi_register_write(s, addr, value);
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

void next_dma_set_ethernet_notify(NextDMAState *s,
                                  const NextDMAEthernetNotify *notify,
                                  void *opaque)
{
    s->enet_notify = notify;
    s->enet_opaque = opaque;
}

static void next_dma_reset_hold(Object *obj, ResetType type)
{
    NextDMAState *s = NEXT_DMA(obj);
    int channel;

    memset(s->channel, 0, sizeof(s->channel));
    memset(&s->trace_scsi_dma_read, 0, sizeof(s->trace_scsi_dma_read));
    s->rx_ready = false;

    for (channel = 0; channel < NEXT_DMA_CHANNEL_COUNT; channel++) {
        qemu_irq_lower(s->irq[channel]);
    }
}

static int next_dma_post_load(void *opaque, int version_id)
{
    NextDMAState *s = opaque;
    int channel;

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

    /*
     * Ethernet transfer support lands with the MB8795.  Until then there
     * is no receive-ready DMA state, so the only valid cached value is
     * false.  Host callbacks and their opaque are deliberately not VMState.
     */
    s->rx_ready = false;
    if (s->enet_notify && s->enet_notify->rx_ready_changed) {
        s->enet_notify->rx_ready_changed(s->enet_opaque, s->rx_ready);
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
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = next_dma_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(channel, NextDMAState, NEXT_DMA_CHANNEL_COUNT, 1,
                             vmstate_next_dma_channel,
                             NextDMAChannelState),
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

    for (i = 0; i < NEXT_DMA_CHANNEL_COUNT; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
}

static void next_dma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->vmsd = &vmstate_next_dma;
    rc->phases.hold = next_dma_reset_hold;
}

static const TypeInfo next_dma_info = {
    .name = TYPE_NEXT_DMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NextDMAState),
    .instance_init = next_dma_init,
    .class_init = next_dma_class_init,
};

static void next_dma_register_types(void)
{
    type_register_static(&next_dma_info);
}

type_init(next_dma_register_types)
