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
#include "hw/rtc/next-rtc.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/bitops.h"
#include "qemu/cutils.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/rtc.h"
#include "system/system.h"

#define NEXT_RTC_STATUS_NEW_CLOCK  0x80
#define NEXT_RTC_CONTROL_START     0x80
#define NEXT_RTC_CONTROL_AUTO_PON  0x20
#define NEXT_RTC_CONTROL_ALARM_EN  0x10
#define NEXT_RTC_CONTROL_ALARM_CLR 0x08
#define NEXT_RTC_CONTROL_FTU_CLR   0x04
#define NEXT_RTC_CONTROL_LOW_BATT  0x02
#define NEXT_RTC_CONTROL_RPD_CLR   0x01
#define NEXT_RTC_CONTROL_STORED    (NEXT_RTC_CONTROL_START | \
                                    NEXT_RTC_CONTROL_AUTO_PON | \
                                    NEXT_RTC_CONTROL_ALARM_EN | \
                                    NEXT_RTC_CONTROL_LOW_BATT)

static bool next_rtc_cmd_is_write(uint8_t cmd)
{
    return cmd & 0x80;
}

static uint32_t next_rtc_counter_value(NeXTRTC *rtc)
{
    int64_t elapsed_ns;

    if (!(rtc->control & NEXT_RTC_CONTROL_START)) {
        return rtc->counter;
    }

    elapsed_ns = qemu_clock_get_ns(rtc_clock) - rtc->counter_ref_ns;
    return rtc->counter + elapsed_ns / NANOSECONDS_PER_SECOND;
}

static void next_rtc_set_control(NeXTRTC *rtc, uint8_t value)
{
    bool was_running = rtc->control & NEXT_RTC_CONTROL_START;
    bool now_running = value & NEXT_RTC_CONTROL_START;
    int64_t now = qemu_clock_get_ns(rtc_clock);

    if (was_running && !now_running) {
        rtc->counter = next_rtc_counter_value(rtc);
    } else if (!was_running && now_running) {
        rtc->counter_ref_ns = now;
    }

    rtc->control = value & NEXT_RTC_CONTROL_STORED;
    if (value & NEXT_RTC_CONTROL_FTU_CLR) {
        rtc->status &= ~0x18;
        qemu_irq_lower(rtc->power_irq);
    }
    if (value & NEXT_RTC_CONTROL_ALARM_CLR) {
        rtc->status &= ~0x02;
    }
    if (value & NEXT_RTC_CONTROL_RPD_CLR) {
        rtc->status &= ~0x01;
    }
}

static void next_rtc_load_read_value(NeXTRTC *rtc)
{
    uint8_t addr = rtc->command & 0x3f;

    rtc->retval = 0;
    if (addr <= 0x1f) {
        rtc->retval = next_nvram_read(&rtc->nvram, addr);
    } else if (addr <= 0x23) {
        unsigned int shift = (0x23 - addr) * 8;

        if (addr == 0x20) {
            rtc->counter_latch = next_rtc_counter_value(rtc);
        }
        rtc->retval = rtc->counter_latch >> shift;
    } else if (addr <= 0x27) {
        unsigned int shift = (0x27 - addr) * 8;

        rtc->retval = rtc->alarm >> shift;
    } else if (addr == 0x30) {
        rtc->retval = rtc->status;
    } else if (addr == 0x31) {
        rtc->retval = rtc->control;
    }
}

static void next_rtc_store_write_value(NeXTRTC *rtc)
{
    uint8_t addr = rtc->command & 0x3f;

    if (addr <= 0x1f) {
        next_nvram_write(&rtc->nvram, addr, rtc->value);
    } else if (addr <= 0x23) {
        unsigned int shift = (0x23 - addr) * 8;

        rtc->counter = deposit32(rtc->counter, shift, 8, rtc->value);
        rtc->counter_ref_ns = qemu_clock_get_ns(rtc_clock);
    } else if (addr <= 0x27) {
        unsigned int shift = (0x27 - addr) * 8;

        rtc->alarm = deposit32(rtc->alarm, shift, 8, rtc->value);
    } else if (addr == 0x31) {
        next_rtc_set_control(rtc, rtc->value);
    }
}

static void next_rtc_advance_byte(NeXTRTC *rtc)
{
    rtc->command = (rtc->command & 0x80) |
                   ((rtc->command + 1) & 0x3f);
    rtc->phase = 8;
    rtc->value = 0;
    if (!next_rtc_cmd_is_write(rtc->command)) {
        next_rtc_load_read_value(rtc);
    }
}

static void next_rtc_data_in_irq(void *opaque, int n, int level)
{
    NeXTRTC *rtc = NEXT_RTC(opaque);

    if (rtc->phase < 8) {
        rtc->command = (rtc->command << 1) | level;
        rtc->phase++;
        if (rtc->phase == 8 && !next_rtc_cmd_is_write(rtc->command)) {
            next_rtc_load_read_value(rtc);
        }
        return;
    }

    if (rtc->phase >= 8 && rtc->phase < 16) {
        if (next_rtc_cmd_is_write(rtc->command)) {
            rtc->value = (rtc->value << 1) | level;
        } else {
            if (rtc->retval & (0x80 >> (rtc->phase - 8))) {
                qemu_irq_raise(rtc->data_out_irq);
            } else {
                qemu_irq_lower(rtc->data_out_irq);
            }
        }
    }

    rtc->phase++;
    if (rtc->phase == 16) {
        if (next_rtc_cmd_is_write(rtc->command)) {
            next_rtc_store_write_value(rtc);
        }
        next_rtc_advance_byte(rtc);
    }
}

static void next_rtc_cmd_reset_irq(void *opaque, int n, int level)
{
    NeXTRTC *rtc = NEXT_RTC(opaque);

    if (level) {
        rtc->phase = 0;
        rtc->command = 0;
        rtc->value = 0;
    }
}

static void next_rtc_reset_hold(Object *obj, ResetType type)
{
    NeXTRTC *rtc = NEXT_RTC(obj);
    struct tm tm;

    rtc->phase = 0;
    rtc->command = 0;
    rtc->value = 0;
    rtc->retval = 0;
    rtc->status = NEXT_RTC_STATUS_NEW_CLOCK;
    rtc->control = NEXT_RTC_CONTROL_START;
    qemu_get_timedate(&tm, 0);
    rtc->counter = mktimegm(&tm);
    rtc->counter_latch = rtc->counter;
    rtc->counter_ref_ns = qemu_clock_get_ns(rtc_clock);
    rtc->alarm = 0;
}

static void next_rtc_reset_exit(Object *obj, ResetType type)
{
    NeXTRTC *rtc = NEXT_RTC(obj);

    qemu_irq_lower(rtc->data_out_irq);
    qemu_irq_lower(rtc->power_irq);
}

static int next_rtc_pre_save(void *opaque)
{
    NeXTRTC *rtc = opaque;

    if (rtc->control & NEXT_RTC_CONTROL_START) {
        rtc->counter = next_rtc_counter_value(rtc);
        rtc->counter_ref_ns = qemu_clock_get_ns(rtc_clock);
    }
    return 0;
}

static bool next_rtc_post_load_errp(void *opaque, int version_id, Error **errp)
{
    NeXTRTC *rtc = opaque;

    if (version_id < 4) {
        struct tm tm;

        qemu_get_timedate(&tm, 0);
        rtc->counter = mktimegm(&tm);
        rtc->counter_latch = rtc->counter;
        rtc->alarm = 0;
    }
    rtc->counter_ref_ns = qemu_clock_get_ns(rtc_clock);
    return next_nvram_flush(&rtc->nvram, errp);
}

static void next_rtc_init(Object *obj)
{
    NeXTRTC *rtc = NEXT_RTC(obj);

    next_nvram_init(&rtc->nvram);
    qdev_init_gpio_in_named(DEVICE(obj), next_rtc_data_in_irq,
                            "rtc-data-in", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &rtc->data_out_irq,
                             "rtc-data-out", 1);
    qdev_init_gpio_in_named(DEVICE(obj), next_rtc_cmd_reset_irq,
                            "rtc-cmd-reset", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &rtc->power_irq,
                             "rtc-power-out", 1);
}

static void next_rtc_realize(DeviceState *dev, Error **errp)
{
    NeXTRTC *rtc = NEXT_RTC(dev);

    next_nvram_realize(&rtc->nvram, errp);
}

static void next_rtc_unrealize(DeviceState *dev)
{
    NeXTRTC *rtc = NEXT_RTC(dev);

    next_nvram_unrealize(&rtc->nvram);
}

static const VMStateDescription next_rtc_vmstate = {
    .name = "next-rtc",
    .version_id = 4,
    .minimum_version_id = 3,
    .pre_save = next_rtc_pre_save,
    .post_load_errp = next_rtc_post_load_errp,
    .fields = (const VMStateField[]) {
        VMSTATE_INT8(phase, NeXTRTC),
        VMSTATE_UINT8_ARRAY(nvram.data, NeXTRTC, NEXT_NVRAM_SIZE),
        VMSTATE_UINT8(command, NeXTRTC),
        VMSTATE_UINT8(value, NeXTRTC),
        VMSTATE_UINT8(status, NeXTRTC),
        VMSTATE_UINT8(control, NeXTRTC),
        VMSTATE_UINT8(retval, NeXTRTC),
        VMSTATE_UINT32_V(counter, NeXTRTC, 4),
        VMSTATE_UINT32_V(counter_latch, NeXTRTC, 4),
        VMSTATE_UINT32_V(alarm, NeXTRTC, 4),
        VMSTATE_END_OF_LIST()
    },
};

static const Property next_rtc_properties[] = {
    DEFINE_PROP_STRING("nvram-file", NeXTRTC, nvram.filename),
};

static void next_rtc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "NeXT RTC";
    dc->vmsd = &next_rtc_vmstate;
    dc->realize = next_rtc_realize;
    dc->unrealize = next_rtc_unrealize;
    device_class_set_props(dc, next_rtc_properties);
    rc->phases.hold = next_rtc_reset_hold;
    rc->phases.exit = next_rtc_reset_exit;
}

static const TypeInfo next_rtc_info = {
    .name = TYPE_NEXT_RTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_init = next_rtc_init,
    .instance_size = sizeof(NeXTRTC),
    .class_init = next_rtc_class_init,
};

static void next_rtc_register_type(void)
{
    type_register_static(&next_rtc_info);
}

type_init(next_rtc_register_type)
