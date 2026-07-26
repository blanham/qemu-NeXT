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
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "ui/console.h"

#define NEXT_C16_VRAM_SIZE (2 * MiB)
#define NEXT_C16_WIDTH 1120
#define NEXT_C16_HEIGHT 832
#define NEXT_C16_STRIDE (1152 * 2)
#define NEXT_BT463_ENTRIES 0x400
#define NEXT_COLOR_RETRACE_NS (NANOSECONDS_PER_SECOND / 68)

#define NEXT_COLOR_COMMAND_CLRINTR 0x01
#define NEXT_COLOR_COMMAND_INTRENA 0x02
#define NEXT_COLOR_COMMAND_UNBLANK 0x04

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
    uint16_t dac_address;
    uint8_t dac_component;
    uint8_t palette[NEXT_BT463_ENTRIES][3];
    uint8_t general[NEXT_BT463_ENTRIES][3];
    uint8_t command;
    uint8_t dram_timing;
    uint8_t vram_timing;
    bool irq_level;
    bool invalidate;
};

static uint8_t next_color_dac_data_read(NextColorVideoState *s,
                                        uint8_t table[][3])
{
    uint8_t value = table[s->dac_address & 0x3ff][s->dac_component];

    s->dac_component++;
    if (s->dac_component == 3) {
        s->dac_component = 0;
        s->dac_address = (s->dac_address + 1) & 0x3ff;
    }

    return value;
}

static void next_color_dac_data_write(NextColorVideoState *s,
                                      uint8_t table[][3], uint8_t value)
{
    table[s->dac_address & 0x3ff][s->dac_component] = value;

    s->dac_component++;
    if (s->dac_component == 3) {
        s->dac_component = 0;
        s->dac_address = (s->dac_address + 1) & 0x3ff;
    }
}

static uint64_t next_color_dac_read(void *opaque, hwaddr addr, unsigned size)
{
    NextColorVideoState *s = opaque;

    switch (addr) {
    case 0:
        return s->dac_address & 0xff;
    case 1:
        return s->dac_address >> 8;
    case 2:
        return next_color_dac_data_read(s, s->general);
    case 3:
        return next_color_dac_data_read(s, s->palette);
    default:
        return 0;
    }
}

static void next_color_dac_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
    NextColorVideoState *s = opaque;

    switch (addr) {
    case 0:
        s->dac_address = (s->dac_address & 0xff00) | (value & 0xff);
        s->dac_component = 0;
        break;
    case 1:
        s->dac_address = (s->dac_address & 0x00ff) |
            ((value & 0xff) << 8);
        s->dac_component = 0;
        break;
    case 2:
        next_color_dac_data_write(s, s->general, value);
        break;
    case 3:
        next_color_dac_data_write(s, s->palette, value);
        break;
    }
}

static uint64_t next_color_command_read(void *opaque, hwaddr addr,
                                        unsigned size)
{
    NextColorVideoState *s = opaque;

    return s->command;
}

static void next_color_set_irq(NextColorVideoState *s, bool level)
{
    s->irq_level = level;
    qemu_set_irq(s->irq, level);
}

static void next_color_retrace(void *opaque)
{
    NextColorVideoState *s = opaque;

    if (s->command & NEXT_COLOR_COMMAND_INTRENA) {
        next_color_set_irq(s, true);
    }
}

static void next_color_schedule_retrace(NextColorVideoState *s)
{
    timer_mod(&s->retrace_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NEXT_COLOR_RETRACE_NS);
}

static void next_color_command_write(void *opaque, hwaddr addr,
                                     uint64_t value, unsigned size)
{
    NextColorVideoState *s = opaque;

    if (value & NEXT_COLOR_COMMAND_CLRINTR) {
        next_color_set_irq(s, false);
    }

    s->command = value &
        (NEXT_COLOR_COMMAND_INTRENA | NEXT_COLOR_COMMAND_UNBLANK);
    timer_del(&s->retrace_timer);
    if (s->command & NEXT_COLOR_COMMAND_INTRENA) {
        next_color_schedule_retrace(s);
    }
    s->invalidate = true;
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
        .max_access_size = 1,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
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

    timer_del(&s->retrace_timer);
    next_color_set_irq(s, false);
    s->dac_address = 0;
    s->dac_component = 0;
    memset(s->palette, 0, sizeof(s->palette));
    memset(s->general, 0, sizeof(s->general));
    s->command = 0;
    s->dram_timing = 0;
    s->vram_timing = 0;
    s->invalidate = false;
}

static int next_color_video_post_load(void *opaque, int version_id)
{
    NextColorVideoState *s = opaque;

    s->dac_address &= 0x3ff;
    s->dac_component %= 3;
    s->command &=
        NEXT_COLOR_COMMAND_INTRENA | NEXT_COLOR_COMMAND_UNBLANK;
    qemu_set_irq(s->irq, s->irq_level);
    if (!(s->command & NEXT_COLOR_COMMAND_INTRENA)) {
        timer_del(&s->retrace_timer);
    }

    return 0;
}

static const VMStateDescription vmstate_next_color_video = {
    .name = TYPE_NEXT_COLOR_VIDEO,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = next_color_video_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(dac_address, NextColorVideoState),
        VMSTATE_UINT8(dac_component, NextColorVideoState),
        VMSTATE_UINT8_2DARRAY(palette, NextColorVideoState,
                             NEXT_BT463_ENTRIES, 3),
        VMSTATE_UINT8_2DARRAY(general, NextColorVideoState,
                             NEXT_BT463_ENTRIES, 3),
        VMSTATE_UINT8(command, NextColorVideoState),
        VMSTATE_UINT8(dram_timing, NextColorVideoState),
        VMSTATE_UINT8(vram_timing, NextColorVideoState),
        VMSTATE_BOOL(irq_level, NextColorVideoState),
        VMSTATE_TIMER(retrace_timer, NextColorVideoState),
        VMSTATE_END_OF_LIST()
    },
};

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
    dc->vmsd = &vmstate_next_color_video;
    rc->phases.hold = next_color_video_reset_hold;
}

static void next_color_video_init(Object *obj)
{
    NextColorVideoState *s = NEXT_COLOR_VIDEO(obj);

    timer_init_ns(&s->retrace_timer, QEMU_CLOCK_VIRTUAL,
                  next_color_retrace, s);
}

static const TypeInfo next_color_video_info = {
    .name = TYPE_NEXT_COLOR_VIDEO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NextColorVideoState),
    .instance_init = next_color_video_init,
    .class_init = next_color_video_class_init,
};

static void next_color_video_register_types(void)
{
    type_register_static(&next_color_video_info);
}

type_init(next_color_video_register_types)
