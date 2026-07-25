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
#include "qapi/util.h"
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
#define NEXT_RTC_OLD_CONTROL_XTAL  0x30
#define NEXT_RTC_OLD_CONTROL_STORED (NEXT_RTC_CONTROL_START | \
                                     NEXT_RTC_OLD_CONTROL_XTAL)
#define NEXT_RTC_OLD_HOUR_12       0x80
#define NEXT_RTC_OLD_HOUR_PM       0x20
#define NEXT_RTC_SECONDS_PER_DAY    (24 * 60 * 60)

static const QEnumLookup next_rtc_chip_lookup = {
    .array = (const char *const[]) {
        [NEXT_RTC_CHIP_MCS1850] = "mcs1850",
        [NEXT_RTC_CHIP_MC68HC68T1] = "mc68hc68t1",
    },
    .size = NEXT_RTC_CHIP__MAX,
};

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

static uint8_t next_rtc_to_bcd(unsigned int value)
{
    return ((value / 10) << 4) | (value % 10);
}

static unsigned int next_rtc_from_bcd(uint8_t value)
{
    return ((value >> 4) * 10) + (value & 0x0f);
}

static void next_rtc_old_get_tm(NeXTRTC *rtc, struct tm *tm)
{
    time_t now = next_rtc_counter_value(rtc);

    gmtime_r(&now, tm);
}

static void next_rtc_old_update_weekday(NeXTRTC *rtc)
{
    uint32_t day = next_rtc_counter_value(rtc) / NEXT_RTC_SECONDS_PER_DAY;
    int64_t elapsed_days = (int64_t)day - rtc->old_weekday_day;

    if (elapsed_days) {
        int64_t weekday = (rtc->old_weekday + elapsed_days) % 7;

        rtc->old_weekday = weekday < 0 ? weekday + 7 : weekday;
        rtc->old_weekday_day = day;
    }
}

static void next_rtc_old_set_tm(NeXTRTC *rtc, const struct tm *tm)
{
    struct tm new_tm = *tm;

    rtc->counter = mktimegm(&new_tm);
    rtc->counter_ref_ns = qemu_clock_get_ns(rtc_clock);
    rtc->old_weekday_day = rtc->counter / NEXT_RTC_SECONDS_PER_DAY;
}

static uint8_t next_rtc_old_calendar_read(NeXTRTC *rtc, uint8_t addr)
{
    struct tm tm;

    next_rtc_old_get_tm(rtc, &tm);
    switch (addr) {
    case 0x20:
        return next_rtc_to_bcd(tm.tm_sec) & 0x7f;
    case 0x21:
        return next_rtc_to_bcd(tm.tm_min) & 0x7f;
    case 0x22:
        if (rtc->old_hour_12) {
            unsigned int hour = tm.tm_hour % 12;

            return NEXT_RTC_OLD_HOUR_12 |
                   (tm.tm_hour >= 12 ? NEXT_RTC_OLD_HOUR_PM : 0) |
                   next_rtc_to_bcd(hour ? hour : 12);
        }
        return next_rtc_to_bcd(tm.tm_hour) & 0x3f;
    case 0x23:
        next_rtc_old_update_weekday(rtc);
        return next_rtc_to_bcd(rtc->old_weekday) & 0x07;
    case 0x24:
        return next_rtc_to_bcd(tm.tm_mday) & 0x3f;
    case 0x25:
        return next_rtc_to_bcd(tm.tm_mon + 1) & 0x1f;
    case 0x26:
        return next_rtc_to_bcd((tm.tm_year + 1900) % 100);
    default:
        return 0;
    }
}

static void next_rtc_old_calendar_write(NeXTRTC *rtc, uint8_t addr,
                                         uint8_t value)
{
    struct tm tm;

    next_rtc_old_update_weekday(rtc);
    next_rtc_old_get_tm(rtc, &tm);
    switch (addr) {
    case 0x20:
        tm.tm_sec = next_rtc_from_bcd(value & 0x7f);
        break;
    case 0x21:
        tm.tm_min = next_rtc_from_bcd(value & 0x7f);
        break;
    case 0x22:
        if (value & NEXT_RTC_OLD_HOUR_12) {
            unsigned int hour = next_rtc_from_bcd(value & 0x1f);

            rtc->old_hour_12 = true;
            if (value & NEXT_RTC_OLD_HOUR_PM) {
                tm.tm_hour = hour == 12 ? 12 : hour + 12;
            } else {
                tm.tm_hour = hour == 12 ? 0 : hour;
            }
        } else {
            rtc->old_hour_12 = false;
            tm.tm_hour = next_rtc_from_bcd(value & 0x3f);
        }
        break;
    case 0x23:
        rtc->old_weekday = next_rtc_from_bcd(value & 0x07) % 7;
        rtc->old_weekday_day = rtc->counter / NEXT_RTC_SECONDS_PER_DAY;
        return;
    case 0x24:
        tm.tm_mday = next_rtc_from_bcd(value & 0x3f);
        break;
    case 0x25:
        tm.tm_mon = next_rtc_from_bcd(value & 0x1f) - 1;
        break;
    case 0x26: {
        unsigned int year = next_rtc_from_bcd(value);

        tm.tm_year = year >= 69 ? year : year + 100;
        break;
    }
    default:
        return;
    }

    next_rtc_old_set_tm(rtc, &tm);
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

    if (rtc->chip == NEXT_RTC_CHIP_MC68HC68T1) {
        rtc->control = value & NEXT_RTC_OLD_CONTROL_STORED;
        return;
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

static void next_rtc_load_read_value(NeXTRTC *rtc, bool new_command)
{
    uint8_t addr = rtc->command & 0x3f;

    rtc->retval = 0;
    if (addr <= 0x1f) {
        rtc->retval = next_nvram_read(&rtc->nvram, addr);
    } else if (rtc->chip == NEXT_RTC_CHIP_MC68HC68T1) {
        if (addr <= 0x26) {
            rtc->retval = next_rtc_old_calendar_read(rtc, addr);
        } else if (addr >= 0x28 && addr <= 0x2a) {
            rtc->retval = rtc->old_alarm[addr - 0x28];
        } else if (addr == 0x30) {
            rtc->retval = rtc->status & ~NEXT_RTC_STATUS_NEW_CLOCK;
        } else if (addr == 0x31) {
            rtc->retval = rtc->control;
        } else if (addr == 0x32) {
            rtc->retval = rtc->old_intctl;
        }
    } else if (addr <= 0x23) {
        unsigned int shift = (0x23 - addr) * 8;

        if (new_command) {
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
    } else if (rtc->chip == NEXT_RTC_CHIP_MC68HC68T1) {
        if (addr <= 0x26) {
            next_rtc_old_calendar_write(rtc, addr, rtc->value);
        } else if (addr >= 0x28 && addr <= 0x2a) {
            rtc->old_alarm[addr - 0x28] = rtc->value;
        } else if (addr == 0x31) {
            next_rtc_set_control(rtc, rtc->value);
        } else if (addr == 0x32) {
            rtc->old_intctl = rtc->value;
        }
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
        next_rtc_load_read_value(rtc, false);
    }
}

static void next_rtc_data_in_irq(void *opaque, int n, int level)
{
    NeXTRTC *rtc = NEXT_RTC(opaque);

    if (rtc->phase < 8) {
        rtc->command = (rtc->command << 1) | level;
        rtc->phase++;
        if (rtc->phase == 8 && !next_rtc_cmd_is_write(rtc->command)) {
            next_rtc_load_read_value(rtc, true);
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
    rtc->status = rtc->chip == NEXT_RTC_CHIP_MCS1850 ?
                  NEXT_RTC_STATUS_NEW_CLOCK : 0;
    rtc->control = rtc->chip == NEXT_RTC_CHIP_MCS1850 ?
                   NEXT_RTC_CONTROL_START :
                   NEXT_RTC_CONTROL_START | NEXT_RTC_OLD_CONTROL_XTAL;
    qemu_get_timedate(&tm, 0);
    rtc->counter = mktimegm(&tm);
    rtc->counter_latch = rtc->counter;
    rtc->counter_ref_ns = qemu_clock_get_ns(rtc_clock);
    rtc->alarm = 0;
    memset(rtc->old_alarm, 0, sizeof(rtc->old_alarm));
    rtc->old_intctl = 0;
    rtc->old_hour_12 = false;
    next_rtc_old_get_tm(rtc, &tm);
    rtc->old_weekday = tm.tm_wday;
    rtc->old_weekday_day = rtc->counter / NEXT_RTC_SECONDS_PER_DAY;
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
    if (version_id < 5 && rtc->chip != NEXT_RTC_CHIP_MCS1850) {
        error_setg(errp, "cannot load pre-v5 state into an old RTC chip");
        return false;
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
    .version_id = 5,
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
        VMSTATE_UINT32_EQUAL_V(chip, NeXTRTC, 5),
        VMSTATE_UINT8_ARRAY_V(old_alarm, NeXTRTC, 3, 5),
        VMSTATE_UINT8_V(old_intctl, NeXTRTC, 5),
        VMSTATE_BOOL_V(old_hour_12, NeXTRTC, 5),
        VMSTATE_UINT8_V(old_weekday, NeXTRTC, 5),
        VMSTATE_UINT32_V(old_weekday_day, NeXTRTC, 5),
        VMSTATE_END_OF_LIST()
    },
};

static const Property next_rtc_properties[] = {
    DEFINE_PROP_STRING("nvram-file", NeXTRTC, nvram.filename),
};

static int next_rtc_get_chip(Object *obj, Error **errp G_GNUC_UNUSED)
{
    return NEXT_RTC(obj)->chip;
}

static void next_rtc_set_chip(Object *obj, int value, Error **errp)
{
    NeXTRTC *rtc = NEXT_RTC(obj);

    if (DEVICE(rtc)->realized) {
        error_setg(errp, "rtc-chip cannot be changed after realize");
        return;
    }
    if (value < 0 || value >= NEXT_RTC_CHIP__MAX) {
        error_setg(errp, "invalid rtc-chip value %d", value);
        return;
    }
    rtc->chip = value;
}

static void next_rtc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "NeXT RTC";
    dc->vmsd = &next_rtc_vmstate;
    dc->realize = next_rtc_realize;
    dc->unrealize = next_rtc_unrealize;
    device_class_set_props(dc, next_rtc_properties);
    object_class_property_add_enum(klass, "rtc-chip", "NextRTCChip",
                                   &next_rtc_chip_lookup,
                                   next_rtc_get_chip, next_rtc_set_chip);
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
