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
#include "hw/display/next-color-video.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "ui/console.h"

#define NEXT_C16_VRAM_SIZE (2 * MiB)
#define NEXT_C16_WIDTH 1120
#define NEXT_C16_HEIGHT 832
#define NEXT_C16_STRIDE (1152 * 2)

struct NextColorVideoState {
    SysBusDevice parent_obj;

    MemoryRegion vram;
    MemoryRegion dac_mmio;
    MemoryRegion command_mmio;
    MemoryRegion dram_timing_mmio;
    MemoryRegion vram_timing_mmio;
    MemoryRegionSection vram_section;
    QemuConsole *console;
    qemu_irq irq;
    QEMUTimer retrace_timer;
    uint8_t command;
    uint8_t dram_timing;
    uint8_t vram_timing;
    bool irq_level;
    bool invalidate;
};

static uint64_t next_color_dac_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static void next_color_dac_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
}

static uint64_t next_color_command_read(void *opaque, hwaddr addr,
                                        unsigned size)
{
    return 0;
}

static void next_color_command_write(void *opaque, hwaddr addr,
                                     uint64_t value, unsigned size)
{
}

static uint64_t next_color_dram_timing_read(void *opaque, hwaddr addr,
                                            unsigned size)
{
    NextColorVideoState *s = opaque;

    return s->dram_timing;
}

static void next_color_dram_timing_write(void *opaque, hwaddr addr,
                                         uint64_t value, unsigned size)
{
    NextColorVideoState *s = opaque;

    s->dram_timing = value;
}

static uint64_t next_color_vram_timing_read(void *opaque, hwaddr addr,
                                            unsigned size)
{
    NextColorVideoState *s = opaque;

    return s->vram_timing;
}

static void next_color_vram_timing_write(void *opaque, hwaddr addr,
                                         uint64_t value, unsigned size)
{
    NextColorVideoState *s = opaque;

    s->vram_timing = value;
}

static const MemoryRegionOps next_color_dac_ops = {
    .read = next_color_dac_read,
    .write = next_color_dac_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static const MemoryRegionOps next_color_command_ops = {
    .read = next_color_command_read,
    .write = next_color_command_write,
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

static const MemoryRegionOps next_color_dram_timing_ops = {
    .read = next_color_dram_timing_read,
    .write = next_color_dram_timing_write,
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

static const MemoryRegionOps next_color_vram_timing_ops = {
    .read = next_color_vram_timing_read,
    .write = next_color_vram_timing_write,
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

static void next_color_video_reset_hold(Object *obj, ResetType type)
{
    NextColorVideoState *s = NEXT_COLOR_VIDEO(obj);

    s->command = 0;
    s->dram_timing = 0;
    s->vram_timing = 0;
    s->irq_level = false;
    s->invalidate = false;
    qemu_set_irq(s->irq, 0);
}

static void next_color_video_realize(DeviceState *dev, Error **errp)
{
    NextColorVideoState *s = NEXT_COLOR_VIDEO(dev);
    Object *obj = OBJECT(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    if (!memory_region_init_ram(&s->vram, obj, "next-color-vram",
                                NEXT_C16_VRAM_SIZE, errp)) {
        return;
    }
    sysbus_init_mmio(sbd, &s->vram);

    memory_region_init_io(&s->dac_mmio, obj, &next_color_dac_ops, s,
                          "next-color-dac", 4);
    sysbus_init_mmio(sbd, &s->dac_mmio);
    memory_region_init_io(&s->command_mmio, obj, &next_color_command_ops, s,
                          "next-color-command", 1);
    sysbus_init_mmio(sbd, &s->command_mmio);
    memory_region_init_io(&s->dram_timing_mmio, obj,
                          &next_color_dram_timing_ops, s,
                          "next-color-dram-timing", 1);
    sysbus_init_mmio(sbd, &s->dram_timing_mmio);
    memory_region_init_io(&s->vram_timing_mmio, obj,
                          &next_color_vram_timing_ops, s,
                          "next-color-vram-timing", 1);
    sysbus_init_mmio(sbd, &s->vram_timing_mmio);

    sysbus_init_irq(sbd, &s->irq);
}

static void next_color_video_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    dc->realize = next_color_video_realize;
    rc->phases.hold = next_color_video_reset_hold;
}

static const TypeInfo next_color_video_info = {
    .name = TYPE_NEXT_COLOR_VIDEO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NextColorVideoState),
    .class_init = next_color_video_class_init,
};

static void next_color_video_register_types(void)
{
    type_register_static(&next_color_video_info);
}

type_init(next_color_video_register_types)
