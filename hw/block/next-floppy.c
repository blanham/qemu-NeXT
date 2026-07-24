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
#include "hw/block/fdc.h"
#include "hw/block/next-floppy.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/dma/next-dma.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"

/*
 * NeXT floppy control register, nextdev/fd_reg.h.
 *
 * EJECT and 82077_SEL are writable latches.  DRIVEID is true low and the
 * two MID bits report the inserted medium's capacity class.
 */
#define NEXT_FLC_EJECT       0x80
#define NEXT_FLC_82077_SEL   0x40
#define NEXT_FLC_DRIVEID     0x04
#define NEXT_FLC_MID_MASK    0x03
#define NEXT_FLC_WRITABLE    (NEXT_FLC_EJECT | NEXT_FLC_82077_SEL)

#define NEXT_FLOPPY_1MB_SIZE  737280
#define NEXT_FLOPPY_2MB_SIZE 1474560

struct NextFloppyCtrlState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    DeviceState *fdc;
    NextDMAState *dma;
    uint8_t control;
};

static uint8_t next_floppy_media_id(int64_t size)
{
    if (size <= NEXT_FLOPPY_1MB_SIZE) {
        return 3 & NEXT_FLC_MID_MASK;
    }
    if (size <= NEXT_FLOPPY_2MB_SIZE) {
        return 2 & NEXT_FLC_MID_MASK;
    }
    return 1 & NEXT_FLC_MID_MASK;
}

static uint64_t next_floppy_read(void *opaque, hwaddr addr, unsigned size)
{
    NextFloppyCtrlState *s = opaque;
    bool drive_present = false;
    int64_t media_size = 0;
    uint8_t value = s->control;

    if (!sysbus_fdc_get_media_info(s->fdc, 0, &drive_present, &media_size)) {
        media_size = 0;
    } else {
        value |= next_floppy_media_id(media_size);
    }

    if (!drive_present) {
        value |= NEXT_FLC_DRIVEID;
    }

    return value;
}

static void next_floppy_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned size)
{
    NextFloppyCtrlState *s = opaque;

    /*
     * Real media removal requires a coordinated removable-frontend operation,
     * which the public block API does not provide to this companion device.
     * Keep EJECT as a faithful latch; the guest normally pulses it high then
     * low, and media remains attached.
     */
    s->control = value & NEXT_FLC_WRITABLE;
    next_dma_set_floppy_selected(s->dma,
                                 !!(s->control & NEXT_FLC_82077_SEL));
}

static const MemoryRegionOps next_floppy_ops = {
    .read = next_floppy_read,
    .write = next_floppy_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static void next_floppy_reset(DeviceState *dev)
{
    NextFloppyCtrlState *s = NEXT_FLOPPY_CTRL(dev);

    s->control = NEXT_FLC_82077_SEL;
    next_dma_set_floppy_selected(s->dma, true);
}

static int next_floppy_post_load(void *opaque, int version_id)
{
    NextFloppyCtrlState *s = opaque;

    s->control &= NEXT_FLC_WRITABLE;
    next_dma_set_floppy_selected(s->dma,
                                 !!(s->control & NEXT_FLC_82077_SEL));
    return 0;
}

static const VMStateDescription vmstate_next_floppy = {
    .name = TYPE_NEXT_FLOPPY_CTRL,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = next_floppy_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(control, NextFloppyCtrlState),
        VMSTATE_END_OF_LIST()
    },
};

static void next_floppy_realize(DeviceState *dev, Error **errp)
{
    NextFloppyCtrlState *s = NEXT_FLOPPY_CTRL(dev);

    if (!s->fdc) {
        error_setg(errp, TYPE_NEXT_FLOPPY_CTRL
                   " requires an 'fdc' link");
        return;
    }
    if (!s->dma) {
        error_setg(errp, TYPE_NEXT_FLOPPY_CTRL
                   " requires a 'dma' link");
    }
}

static void next_floppy_init(Object *obj)
{
    NextFloppyCtrlState *s = NEXT_FLOPPY_CTRL(obj);

    memory_region_init_io(&s->mmio, obj, &next_floppy_ops, s,
                          TYPE_NEXT_FLOPPY_CTRL, 1);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static const Property next_floppy_properties[] = {
    DEFINE_PROP_LINK("fdc", NextFloppyCtrlState, fdc,
                     TYPE_SYSBUS_FDC, DeviceState *),
    DEFINE_PROP_LINK("dma", NextFloppyCtrlState, dma,
                     TYPE_NEXT_DMA, NextDMAState *),
};

static void next_floppy_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = next_floppy_realize;
    device_class_set_legacy_reset(dc, next_floppy_reset);
    dc->vmsd = &vmstate_next_floppy;
    device_class_set_props(dc, next_floppy_properties);
}

static const TypeInfo next_floppy_type_info = {
    .name = TYPE_NEXT_FLOPPY_CTRL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NextFloppyCtrlState),
    .instance_init = next_floppy_init,
    .class_init = next_floppy_class_init,
};

static void next_floppy_register_types(void)
{
    type_register_static(&next_floppy_type_info);
}

type_init(next_floppy_register_types)
