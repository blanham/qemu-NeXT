/* SPDX-License-Identifier: NCSA
 *
 * Copyright (c) 2011-2026 Bryce Lanham
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without
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
 * IMPLIED, INCLUDING, BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS WITH THE SOFTWARE.
 */
#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "hw/display/next-fb.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "qom/object.h"
#include "ui/console.h"
#include "hw/display/framebuffer.h"
#include "ui/pixel_ops.h"
#include "trace.h"

#define NEXT_FB_RETRACE_HZ 68
#define NEXT_FB_RETRACE_NS (NANOSECONDS_PER_SECOND / NEXT_FB_RETRACE_HZ)

OBJECT_DECLARE_SIMPLE_TYPE(NeXTFbState, NEXTFB)

struct NeXTFbState {
    SysBusDevice parent_obj;

    MemoryRegion fb_mr;
    MemoryRegionSection fbsection;
    QemuConsole *con;
    qemu_irq retrace_irq;
    QEMUTimer retrace_timer;

    uint32_t cols;
    uint32_t rows;
    int invalidate;
};

static void nextfb_schedule_retrace(NeXTFbState *s)
{
    timer_mod(&s->retrace_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + NEXT_FB_RETRACE_NS);
}

static void nextfb_retrace(void *opaque)
{
    NeXTFbState *s = opaque;

    qemu_irq_pulse(s->retrace_irq);
    nextfb_schedule_retrace(s);
}

static void nextfb_draw_line(void *opaque, uint8_t *d, const uint8_t *s,
                             int width, int pitch)
{
    NeXTFbState *nfbstate = NEXTFB(opaque);
    static const uint32_t pal[4] = {
        0xFFFFFFFF, 0xFFAAAAAA, 0xFF555555, 0xFF000000
    };
    uint32_t *buf = (uint32_t *)d;
    int i = 0;

    for (i = 0; i < nfbstate->cols / 4; i++) {
        int j = i * 4;
        uint8_t src = s[i];
        buf[j + 3] = pal[src & 0x3];
        src >>= 2;
        buf[j + 2] = pal[src & 0x3];
        src >>= 2;
        buf[j + 1] = pal[src & 0x3];
        src >>= 2;
        buf[j + 0] = pal[src & 0x3];
    }
}

static bool nextfb_update(void *opaque)
{
    NeXTFbState *s = NEXTFB(opaque);
    bool invalidate = s->invalidate;
    int dest_width = 4;
    int src_width;
    int first = 0;
    int last  = 0;
    DisplaySurface *surface = qemu_console_surface(s->con);

    src_width = s->cols / 4 + 8;
    dest_width = s->cols * 4;

    if (s->invalidate) {
        framebuffer_update_memory_section(&s->fbsection, &s->fb_mr, 0,
                                          s->cols, src_width);
    }

    framebuffer_update_display(surface, &s->fbsection, s->cols, s->rows,
                               src_width, dest_width, 0, invalidate,
                               nextfb_draw_line, s, &first, &last);
    trace_nextfb_update(first, last, invalidate);

    if (first >= 0) {
        qemu_console_update(s->con, 0, first, s->cols, last - first + 1);
    }
    s->invalidate = 0;

    return true;
}

static void nextfb_invalidate(void *opaque)
{
    NeXTFbState *s = NEXTFB(opaque);
    s->invalidate = 1;
}

static const GraphicHwOps nextfb_ops = {
    .invalidate  = nextfb_invalidate,
    .gfx_update  = nextfb_update,
};

static void nextfb_reset_hold(Object *obj, ResetType type)
{
    NeXTFbState *s = NEXTFB(obj);

    timer_del(&s->retrace_timer);
    s->invalidate = 1;
    nextfb_schedule_retrace(s);
}

static const VMStateDescription vmstate_nextfb = {
    .name = TYPE_NEXTFB,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(retrace_timer, NeXTFbState),
        VMSTATE_END_OF_LIST()
    },
};

static void nextfb_realize(DeviceState *dev, Error **errp)
{
    NeXTFbState *s = NEXTFB(dev);

    memory_region_init_ram(&s->fb_mr, OBJECT(dev), "next-video", 0x1CB100,
                           &error_fatal);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->fb_mr);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->retrace_irq);

    s->invalidate = 1;
    s->cols = 1120;
    s->rows = 832;

    s->con = qemu_graphic_console_create(dev, 0, &nextfb_ops, s);
    qemu_console_resize(s->con, s->cols, s->rows);
}

static void nextfb_unrealize(DeviceState *dev)
{
    NeXTFbState *s = NEXTFB(dev);

    timer_del(&s->retrace_timer);
    if (s->fbsection.mr) {
        memory_region_set_log(s->fbsection.mr, false, DIRTY_MEMORY_VGA);
        memory_region_unref(s->fbsection.mr);
        s->fbsection.mr = NULL;
    }
    if (s->con) {
        qemu_graphic_console_close(s->con);
        s->con = NULL;
    }
}

static void nextfb_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    dc->realize = nextfb_realize;
    dc->unrealize = nextfb_unrealize;
    dc->vmsd = &vmstate_nextfb;
    rc->phases.hold = nextfb_reset_hold;
}

static void nextfb_init(Object *obj)
{
    NeXTFbState *s = NEXTFB(obj);

    timer_init_ns(&s->retrace_timer, QEMU_CLOCK_VIRTUAL, nextfb_retrace, s);
}

static const TypeInfo nextfb_info = {
    .name          = TYPE_NEXTFB,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NeXTFbState),
    .instance_init = nextfb_init,
    .class_init    = nextfb_class_init,
};

static void nextfb_register_types(void)
{
    type_register_static(&nextfb_info);
}

type_init(nextfb_register_types)
