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
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/dma/next-dma.h"
#include "hw/misc/next-optical.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/cutils.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"

#define NEXT_OPTICAL_MMIO_SIZE       0x20
#define NEXT_OPTICAL_REGISTER_COUNT  0x17
#define NEXT_OPTICAL_UNCODED_SIZE    1024
#define NEXT_OPTICAL_CODED_SIZE      1296

#define NEXT_OPTICAL_DISR            0x04
#define NEXT_OPTICAL_DIMR            0x05
#define NEXT_OPTICAL_CONTROL2        0x06
#define NEXT_OPTICAL_CONTROL1        0x07
#define NEXT_OPTICAL_DESR            0x0a
#define NEXT_OPTICAL_ECCCNT          0x0b
#define NEXT_OPTICAL_INITR           0x0c
#define NEXT_OPTICAL_DMARK           0x0e
#define NEXT_OPTICAL_FLAG_FIRST      0x10
#define NEXT_OPTICAL_FLAG_LAST       0x16

#define NEXT_OPTICAL_DATA_ERR        0x80
#define NEXT_OPTICAL_ECC_DONE        0x08
#define NEXT_OPTICAL_RESET           0x01
#define NEXT_OPTICAL_ERR_ECC         0x01
#define NEXT_OPTICAL_ECC_SELECT      0x40
#define NEXT_OPTICAL_ECC_DECODE      0x20
#define NEXT_OPTICAL_ECC_WRITE       0x40
#define NEXT_OPTICAL_ECC_READ        0x80

typedef enum NextOpticalPhase {
    NEXT_OPTICAL_ENCODE_INPUT,
    NEXT_OPTICAL_ENCODE_OUTPUT,
    NEXT_OPTICAL_DECODE_INPUT,
    NEXT_OPTICAL_DECODE_OUTPUT,
    NEXT_OPTICAL_COMPLETE,
} NextOpticalPhase;

struct NextOpticalState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    NextDMAState *dma;
    uint8_t registers[NEXT_OPTICAL_MMIO_SIZE];
    uint8_t original[NEXT_OPTICAL_UNCODED_SIZE];
    uint8_t frame[NEXT_OPTICAL_CODED_SIZE];
    uint8_t phase;
    bool pending_dma_abort;
    QEMUBH *bh;
};

static void next_optical_schedule(NextOpticalState *s)
{
    if (s->bh) {
        qemu_bh_schedule(s->bh);
    }
}

static void next_optical_reset_state(NextOpticalState *s)
{
    memset(s->registers, 0, sizeof(s->registers));
    memset(s->original, 0, sizeof(s->original));
    memset(s->frame, 0, sizeof(s->frame));
    s->phase = NEXT_OPTICAL_ENCODE_INPUT;
    s->pending_dma_abort = false;
}

static void next_optical_finish(NextOpticalState *s)
{
    s->registers[NEXT_OPTICAL_CONTROL1] = 0;
    s->registers[NEXT_OPTICAL_DESR] = 0;
    s->registers[NEXT_OPTICAL_DISR] |= NEXT_OPTICAL_ECC_DONE;
}

static void next_optical_fail(NextOpticalState *s)
{
    s->registers[NEXT_OPTICAL_CONTROL1] = 0;
    s->registers[NEXT_OPTICAL_DESR] = NEXT_OPTICAL_ERR_ECC;
    s->registers[NEXT_OPTICAL_ECCCNT] = 0;
    s->registers[NEXT_OPTICAL_DISR] |=
        NEXT_OPTICAL_DATA_ERR | NEXT_OPTICAL_ECC_DONE;
}

static void next_optical_command_fail(NextOpticalState *s)
{
    s->pending_dma_abort =
        next_dma_optical_abort(s->dma) == NEXT_DMA_NOT_READY;
    next_optical_fail(s);
}

static void next_optical_apply_v66_mutations(
    uint8_t frame[NEXT_OPTICAL_CODED_SIZE])
{
    size_t index;

    frame[1] = ~frame[1];
    frame[50] = frame[50] + 1;
    frame[100] = ~frame[100] - 100;
    frame[200] = frame[200] + 255;
    frame[777] = ~frame[777];
    frame[890] = frame[890] + 23;
    frame[1111] = frame[1110] + 39;
    frame[1290] = frame[1290] + 22;
    for (index = 533; index < 565; index++) {
        int promoted = ~(int)frame[index];

        frame[index] = (promoted % 47) + 37;
    }
}

static bool next_optical_valid_v66_frame(
    const NextOpticalState *s,
    const uint8_t candidate[NEXT_OPTICAL_CODED_SIZE])
{
    uint8_t expected[NEXT_OPTICAL_CODED_SIZE];

    memcpy(expected, s->frame, sizeof(expected));
    next_optical_apply_v66_mutations(expected);
    return memcmp(candidate, expected, sizeof(expected)) == 0;
}

static void next_optical_run(void *opaque)
{
    NextOpticalState *s = opaque;
    uint8_t candidate[NEXT_OPTICAL_CODED_SIZE];
    uint8_t command = s->registers[NEXT_OPTICAL_CONTROL1];
    uint8_t mode = s->registers[NEXT_OPTICAL_CONTROL2] &
                   (NEXT_OPTICAL_ECC_SELECT | NEXT_OPTICAL_ECC_DECODE);
    NextDMAResult result;

    if (s->pending_dma_abort) {
        result = next_dma_optical_abort(s->dma);
        if (result == NEXT_DMA_NOT_READY) {
            return;
        }
        s->pending_dma_abort = false;
        next_optical_fail(s);
        return;
    }

    switch (s->phase) {
    case NEXT_OPTICAL_ENCODE_INPUT:
        if (!command) {
            return;
        }
        if (command != NEXT_OPTICAL_ECC_WRITE) {
            next_optical_command_fail(s);
            return;
        }
        if (mode != NEXT_OPTICAL_ECC_SELECT) {
            next_optical_command_fail(s);
            return;
        }
        result = next_dma_optical_read(s->dma, candidate,
                                       NEXT_OPTICAL_UNCODED_SIZE);
        if (result == NEXT_DMA_NOT_READY) {
            return;
        }
        if (result != NEXT_DMA_OK) {
            next_optical_fail(s);
            return;
        }
        memcpy(s->original, candidate, sizeof(s->original));
        memcpy(s->frame, s->original, sizeof(s->original));
        memset(s->frame + sizeof(s->original), 0,
               sizeof(s->frame) - sizeof(s->original));
        s->phase = NEXT_OPTICAL_ENCODE_OUTPUT;
        next_optical_finish(s);
        return;

    case NEXT_OPTICAL_ENCODE_OUTPUT:
        if (!command) {
            return;
        }
        if (command != NEXT_OPTICAL_ECC_READ) {
            next_optical_command_fail(s);
            return;
        }
        result = next_dma_optical_write(s->dma, s->frame,
                                        sizeof(s->frame));
        if (result == NEXT_DMA_NOT_READY) {
            return;
        }
        if (result != NEXT_DMA_OK) {
            next_optical_fail(s);
            return;
        }
        s->phase = NEXT_OPTICAL_DECODE_INPUT;
        next_optical_finish(s);
        return;

    case NEXT_OPTICAL_DECODE_INPUT:
        if (!command) {
            return;
        }
        if (command != NEXT_OPTICAL_ECC_WRITE) {
            next_optical_command_fail(s);
            return;
        }
        if (mode != (NEXT_OPTICAL_ECC_SELECT |
                     NEXT_OPTICAL_ECC_DECODE)) {
            next_optical_command_fail(s);
            return;
        }
        result = next_dma_optical_read(s->dma, candidate,
                                       sizeof(candidate));
        if (result == NEXT_DMA_NOT_READY) {
            return;
        }
        if (result != NEXT_DMA_OK ||
            !next_optical_valid_v66_frame(s, candidate)) {
            next_optical_fail(s);
            return;
        }
        s->registers[NEXT_OPTICAL_ECCCNT] = 36;
        s->phase = NEXT_OPTICAL_DECODE_OUTPUT;
        next_optical_finish(s);
        return;

    case NEXT_OPTICAL_DECODE_OUTPUT:
        if (!command) {
            return;
        }
        if (command != NEXT_OPTICAL_ECC_READ) {
            next_optical_command_fail(s);
            return;
        }
        result = next_dma_optical_write(s->dma, s->original,
                                        sizeof(s->original));
        if (result == NEXT_DMA_NOT_READY) {
            return;
        }
        if (result != NEXT_DMA_OK) {
            next_optical_fail(s);
            return;
        }
        s->phase = NEXT_OPTICAL_COMPLETE;
        s->registers[NEXT_OPTICAL_ECCCNT] = 36;
        next_optical_finish(s);
        return;

    case NEXT_OPTICAL_COMPLETE:
        /*
         * The X15 ROM probes the formatter after POST.  With no optical
         * medium attached, leave drive commands pending and report no drive.
         */
        return;

    default:
        g_assert_not_reached();
    }
}

static void next_optical_dma_enabled(void *opaque)
{
    next_optical_schedule(opaque);
}

static const NextDMAOpticalNotify next_optical_dma_notify = {
    .enabled = next_optical_dma_enabled,
};

static uint64_t next_optical_read(void *opaque, hwaddr addr, unsigned size)
{
    NextOpticalState *s = NEXT_OPTICAL(opaque);

    if (addr >= NEXT_OPTICAL_REGISTER_COUNT ||
        (addr >= NEXT_OPTICAL_INITR && addr <= NEXT_OPTICAL_DMARK) ||
        addr == 0x0f) {
        return 0;
    }
    return s->registers[addr];
}

static void next_optical_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    NextOpticalState *s = NEXT_OPTICAL(opaque);
    uint8_t byte = value;

    switch (addr) {
    case 0x00 ... 0x03:
    case NEXT_OPTICAL_DIMR:
    case NEXT_OPTICAL_CONTROL2:
    case 0x08 ... 0x09:
    case NEXT_OPTICAL_INITR ... NEXT_OPTICAL_DMARK:
    case NEXT_OPTICAL_FLAG_FIRST ... NEXT_OPTICAL_FLAG_LAST:
        s->registers[addr] = byte;
        return;

    case NEXT_OPTICAL_DISR:
        if (byte & NEXT_OPTICAL_RESET) {
            qemu_bh_cancel(s->bh);
            next_optical_reset_state(s);
        } else {
            s->registers[NEXT_OPTICAL_DISR] &= ~(byte & 0xfc);
        }
        return;

    case NEXT_OPTICAL_CONTROL1:
        s->registers[addr] = byte;
        if (byte) {
            next_optical_schedule(s);
        }
        return;

    default:
        return;
    }
}

static const MemoryRegionOps next_optical_ops = {
    .read = next_optical_read,
    .write = next_optical_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 1,
        .unaligned = false,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static void next_optical_reset_hold(Object *obj, ResetType type)
{
    NextOpticalState *s = NEXT_OPTICAL(obj);

    qemu_bh_cancel(s->bh);
    next_optical_reset_state(s);
}

static bool next_optical_migration_state_valid(const NextOpticalState *s)
{
    uint8_t disr = s->registers[NEXT_OPTICAL_DISR];
    uint8_t desr = s->registers[NEXT_OPTICAL_DESR];
    uint8_t ecccnt = s->registers[NEXT_OPTICAL_ECCCNT];
    bool have_frame;

    if (s->phase > NEXT_OPTICAL_COMPLETE ||
        (disr & ~(NEXT_OPTICAL_DATA_ERR | NEXT_OPTICAL_ECC_DONE)) ||
        desr > NEXT_OPTICAL_ERR_ECC ||
        ((disr & NEXT_OPTICAL_DATA_ERR) &&
         desr != NEXT_OPTICAL_ERR_ECC)) {
        return false;
    }
    if (s->pending_dma_abort &&
        (s->phase >= NEXT_OPTICAL_COMPLETE ||
         desr != NEXT_OPTICAL_ERR_ECC || ecccnt != 0)) {
        return false;
    }

    have_frame = s->phase >= NEXT_OPTICAL_ENCODE_OUTPUT;
    if (have_frame) {
        if (memcmp(s->frame, s->original, sizeof(s->original)) ||
            !buffer_is_zero(s->frame + sizeof(s->original),
                            sizeof(s->frame) - sizeof(s->original))) {
            return false;
        }
    } else if (!buffer_is_zero(s->original, sizeof(s->original)) ||
               !buffer_is_zero(s->frame, sizeof(s->frame))) {
        return false;
    }

    switch (s->phase) {
    case NEXT_OPTICAL_ENCODE_INPUT:
    case NEXT_OPTICAL_ENCODE_OUTPUT:
    case NEXT_OPTICAL_DECODE_INPUT:
        return ecccnt == 0;

    case NEXT_OPTICAL_DECODE_OUTPUT:
        return desr == NEXT_OPTICAL_ERR_ECC
             ? ecccnt == 0 : ecccnt == 36;

    case NEXT_OPTICAL_COMPLETE:
        return desr == 0 && ecccnt == 36;

    default:
        return false;
    }
}

static int next_optical_post_load(void *opaque, int version_id)
{
    NextOpticalState *s = opaque;

    if (!next_optical_migration_state_valid(s)) {
        return -EINVAL;
    }
    if (s->phase < NEXT_OPTICAL_COMPLETE &&
        (s->pending_dma_abort ||
         s->registers[NEXT_OPTICAL_CONTROL1])) {
        next_optical_schedule(s);
    }
    return 0;
}

static const VMStateDescription vmstate_next_optical = {
    .name = TYPE_NEXT_OPTICAL,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = next_optical_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(registers, NextOpticalState,
                            NEXT_OPTICAL_MMIO_SIZE),
        VMSTATE_UINT8_ARRAY(original, NextOpticalState,
                            NEXT_OPTICAL_UNCODED_SIZE),
        VMSTATE_UINT8_ARRAY(frame, NextOpticalState,
                            NEXT_OPTICAL_CODED_SIZE),
        VMSTATE_UINT8(phase, NextOpticalState),
        VMSTATE_BOOL(pending_dma_abort, NextOpticalState),
        VMSTATE_END_OF_LIST()
    },
};

static void next_optical_realize(DeviceState *dev, Error **errp)
{
    NextOpticalState *s = NEXT_OPTICAL(dev);

    if (!s->dma) {
        error_setg(errp, "NeXT optical formatter requires a DMA controller");
        return;
    }
    s->bh = qemu_bh_new(next_optical_run, s);
    next_dma_set_optical_notify(s->dma, &next_optical_dma_notify, s);
}

static void next_optical_unrealize(DeviceState *dev)
{
    NextOpticalState *s = NEXT_OPTICAL(dev);

    next_dma_set_optical_notify(s->dma, NULL, NULL);
    qemu_bh_delete(s->bh);
    s->bh = NULL;
}

static const Property next_optical_properties[] = {
    DEFINE_PROP_LINK("dma", NextOpticalState, dma,
                     TYPE_NEXT_DMA, NextDMAState *),
};

static void next_optical_init(Object *obj)
{
    NextOpticalState *s = NEXT_OPTICAL(obj);

    memory_region_init_io(&s->mmio, obj, &next_optical_ops, s,
                          TYPE_NEXT_OPTICAL, NEXT_OPTICAL_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void next_optical_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "NeXT magneto-optical formatter";
    dc->realize = next_optical_realize;
    dc->unrealize = next_optical_unrealize;
    dc->vmsd = &vmstate_next_optical;
    device_class_set_props(dc, next_optical_properties);
    rc->phases.hold = next_optical_reset_hold;
}

static const TypeInfo next_optical_info = {
    .name = TYPE_NEXT_OPTICAL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NextOpticalState),
    .instance_init = next_optical_init,
    .class_init = next_optical_class_init,
};

static void next_optical_register_types(void)
{
    type_register_static(&next_optical_info);
}

type_init(next_optical_register_types)
