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
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "ui/console.h"
#include "hw/display/bt463.h"
#include "hw/display/framebuffer.h"
#include "ui/pixel_ops.h"

#define NEXT_C16_VRAM_SIZE (2 * MiB)
#define NEXT_C16_WIDTH 1120
#define NEXT_C16_HEIGHT 832
#define NEXT_C16_STRIDE (1152 * 2)
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
    Bt463State bt463;
    /* Incoming-only storage for the pre-Bt463-model migration format. */
    Bt463LegacyState bt463_legacy;
    uint8_t command;
    uint8_t dram_timing;
    uint8_t vram_timing;
    bool irq_level;
    bool invalidate;
};

static void next_color_video_draw_line(void *opaque, uint8_t *dst,
                                       const uint8_t *src, int width,
                                       int pitch)
{
    NextColorVideoState *s = opaque;
    uint32_t *out = (uint32_t *)dst;
    Bt463LoadPhase seed = BT463_LOAD_LOWER;

    if (width > 0) {
        const uint16_t first_pixel = lduw_be_p(src);

        seed = bt463_load_phase_seed(&s->bt463, first_pixel & 0xf);
    }

    for (int x = 0; x < width; x++) {
        uint16_t pixel = lduw_be_p(src + x * 2);
        uint32_t pixel_pins = ((uint32_t)(pixel >> 12) & 0xf) << 4;
        uint32_t rgb;

        /*
         * Warp9C drives only the four high planes of each color octet;
         * WT0-WT3 carry the low framebuffer nibble.  Overlay pins P24-P27
         * are inactive for normal framebuffer scanout.
         */
        pixel_pins |= ((uint32_t)(pixel >> 8) & 0xf) << 12;
        pixel_pins |= ((uint32_t)(pixel >> 4) & 0xf) << 20;
        rgb = bt463_lookup_rgb(
            &s->bt463, pixel_pins, pixel & 0xf,
            bt463_load_phase_at(&s->bt463, seed, x));
        out[x] = rgb_to_pixel32((rgb >> 16) & 0xff, (rgb >> 8) & 0xff,
                                rgb & 0xff);
    }
}

static bool next_color_video_update(void *opaque)
{
    NextColorVideoState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->console);
    int first = 0;
    int last = 0;

    if (!(s->command & NEXT_COLOR_COMMAND_UNBLANK)) {
        if (!s->invalidate) {
            return true;
        }
        memset(surface_data(surface), 0,
               (size_t)surface_stride(surface) * surface_height(surface));
        qemu_console_update(s->console, 0, 0, surface_width(surface),
                            surface_height(surface));
        s->invalidate = false;
        return true;
    }

    if (s->invalidate) {
        framebuffer_update_memory_section(&s->vram_section, &s->vram, 0,
                                          NEXT_C16_HEIGHT,
                                          NEXT_C16_STRIDE);
    }

    framebuffer_update_display(surface, &s->vram_section,
                               NEXT_C16_WIDTH, NEXT_C16_HEIGHT,
                               NEXT_C16_STRIDE, NEXT_C16_WIDTH * 4, 0,
                               s->invalidate, next_color_video_draw_line,
                               s, &first, &last);
    if (first >= 0) {
        qemu_console_update(s->console, 0, first, NEXT_C16_WIDTH,
                            last - first + 1);
    }
    s->invalidate = false;

    return true;
}

static void next_color_video_invalidate(void *opaque)
{
    NextColorVideoState *s = opaque;

    s->invalidate = true;
}

static const GraphicHwOps next_color_video_ops = {
    .invalidate = next_color_video_invalidate,
    .gfx_update = next_color_video_update,
};

static uint64_t next_color_dac_read(void *opaque, hwaddr addr, unsigned size)
{
    NextColorVideoState *s = opaque;

    switch (addr) {
    case 0:
        return bt463_address_read(&s->bt463, false);
    case 1:
        return bt463_address_read(&s->bt463, true);
    case 2:
        return bt463_general_read(&s->bt463);
    case 3:
        return bt463_palette_read(&s->bt463);
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
        bt463_address_write(&s->bt463, false, value);
        break;
    case 1:
        bt463_address_write(&s->bt463, true, value);
        break;
    case 2:
        if (bt463_general_write(&s->bt463, value)) {
            s->invalidate = true;
        }
        break;
    case 3:
        if (bt463_palette_write(&s->bt463, value)) {
            s->invalidate = true;
        }
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

static void next_color_schedule_retrace(NextColorVideoState *s);

static void next_color_retrace(void *opaque)
{
    NextColorVideoState *s = opaque;

    if (bt463_retrace_step(&s->bt463)) {
        s->invalidate = true;
    }
    if (s->command & NEXT_COLOR_COMMAND_INTRENA) {
        next_color_set_irq(s, true);
    }
    /* Retrace drives blink even when the board interrupt is masked. */
    next_color_schedule_retrace(s);
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
    /* CSR acknowledgements must not move the continuous retrace cadence. */
    if (!timer_pending(&s->retrace_timer)) {
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
    bt463_reset(&s->bt463);
    s->command = 0;
    s->dram_timing = 0;
    s->vram_timing = 0;
    s->invalidate = true;
    next_color_schedule_retrace(s);
}

static bool next_color_video_legacy_state(void *opaque G_GNUC_UNUSED,
                                          int version_id)
{
    return version_id == 1;
}

static bool next_color_video_bt463_v2_state(void *opaque G_GNUC_UNUSED,
                                            int version_id)
{
    return version_id == 2;
}

static bool next_color_video_bt463_v3_state(void *opaque G_GNUC_UNUSED,
                                            int version_id)
{
    return version_id >= 3;
}

static int next_color_video_post_load(void *opaque, int version_id)
{
    NextColorVideoState *s = opaque;

    if (version_id == 1) {
        bt463_import_legacy(&s->bt463, &s->bt463_legacy);
    } else if (version_id == 2) {
        /* v2 nested Bt463 streams predate the blink fields. */
        s->bt463.blink_counter = 0;
        s->bt463.blink_phase = true;
    }
    s->command &=
        NEXT_COLOR_COMMAND_INTRENA | NEXT_COLOR_COMMAND_UNBLANK;
    qemu_set_irq(s->irq, s->irq_level);
    if (!timer_pending(&s->retrace_timer)) {
        next_color_schedule_retrace(s);
    }
    s->invalidate = true;

    return 0;
}

static const VMStateDescription vmstate_next_color_video = {
    .name = TYPE_NEXT_COLOR_VIDEO,
    /* v1 used generic DAC arrays; v3 adds the Bt463 blink fields. */
    .version_id = 3,
    .minimum_version_id = 1,
    .post_load = next_color_video_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_TEST(bt463_legacy, NextColorVideoState,
                            next_color_video_legacy_state, 1,
                            vmstate_bt463_legacy, Bt463LegacyState),
        VMSTATE_VSTRUCT_TEST(bt463, NextColorVideoState,
                             next_color_video_bt463_v2_state, 2,
                             vmstate_bt463, Bt463State, 1),
        VMSTATE_VSTRUCT_TEST(bt463, NextColorVideoState,
                             next_color_video_bt463_v3_state, 3,
                             vmstate_bt463, Bt463State, 2),
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

    s->invalidate = true;
    s->console =
        qemu_graphic_console_create(dev, 0, &next_color_video_ops, s);
    qemu_console_resize(s->console, NEXT_C16_WIDTH, NEXT_C16_HEIGHT);
}

static void next_color_video_unrealize(DeviceState *dev)
{
    NextColorVideoState *s = NEXT_COLOR_VIDEO(dev);

    timer_del(&s->retrace_timer);
    if (s->vram_section.mr) {
        memory_region_set_log(s->vram_section.mr, false, DIRTY_MEMORY_VGA);
        memory_region_unref(s->vram_section.mr);
        s->vram_section.mr = NULL;
    }
    if (s->console) {
        qemu_graphic_console_close(s->console);
        s->console = NULL;
    }
}

static void next_color_video_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    dc->realize = next_color_video_realize;
    dc->unrealize = next_color_video_unrealize;
    dc->vmsd = &vmstate_next_color_video;
    rc->phases.hold = next_color_video_reset_hold;
}

static void next_color_video_init(Object *obj)
{
    NextColorVideoState *s = NEXT_COLOR_VIDEO(obj);

    bt463_init(&s->bt463);
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
