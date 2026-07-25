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

#ifndef HW_RTC_NEXT_RTC_H
#define HW_RTC_NEXT_RTC_H

#include "hw/core/sysbus.h"
#include "hw/nvram/next-nvram.h"

#define TYPE_NEXT_RTC "next-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(NeXTRTC, NEXT_RTC)

typedef enum NextRTCChip {
    NEXT_RTC_CHIP_MCS1850,
    NEXT_RTC_CHIP_MC68HC68T1,
    NEXT_RTC_CHIP__MAX,
} NextRTCChip;

struct NeXTRTC {
    SysBusDevice parent_obj;

    uint32_t chip;
    int8_t phase;
    NextNVRAMState nvram;
    uint8_t command;
    uint8_t value;
    uint8_t status;
    uint8_t control;
    uint8_t retval;
    uint32_t counter;
    uint32_t counter_latch;
    uint32_t alarm;
    uint8_t old_alarm[3];
    uint8_t old_calendar[7];
    uint8_t old_intctl;
    bool old_hour_12;
    uint8_t old_weekday;
    uint32_t old_weekday_day;
    int64_t counter_ref_ns;

    qemu_irq data_out_irq;
    qemu_irq power_irq;
};

#endif /* HW_RTC_NEXT_RTC_H */
