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
#include "hw/char/escc.h"
#include "hw/char/next-serial.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define NEXT_SERIAL_MMIO_SIZE 5
#define NEXT_SERIAL_PCLK_HZ   3684000
#define NEXT_SERIAL_RTXC_HZ   4000000

#define NEXT_SERIAL_PCLK_ESCLK 0x10
#define NEXT_SERIAL_B_4MHZ     0x08
#define NEXT_SERIAL_B_ESCLK    0x04
#define NEXT_SERIAL_A_4MHZ     0x02
#define NEXT_SERIAL_A_ESCLK    0x01

struct NextSerialState {
    SysBusDevice parent_obj;

    ESCCState escc;
    MemoryRegion mmio;
    MemoryRegion clock_mem;
    qemu_irq irq;
    bool irq_level[2];
    uint8_t clock_select;
};

static void next_serial_update_irq(NextSerialState *s)
{
    qemu_set_irq(s->irq, s->irq_level[0] || s->irq_level[1]);
}

static void next_serial_set_irq(void *opaque, int n, int level)
{
    NextSerialState *s = NEXT_SERIAL(opaque);

    s->irq_level[n] = level;
    next_serial_update_irq(s);
}

static void next_serial_apply_clocks(NextSerialState *s)
{
    uint32_t pclk_hz =
        s->clock_select & NEXT_SERIAL_PCLK_ESCLK ?
        NEXT_SERIAL_RTXC_HZ : NEXT_SERIAL_PCLK_HZ;
    uint32_t ch_b_rtxc_hz =
        s->clock_select & (NEXT_SERIAL_B_4MHZ | NEXT_SERIAL_B_ESCLK) ?
        NEXT_SERIAL_RTXC_HZ : NEXT_SERIAL_PCLK_HZ;
    uint32_t ch_a_rtxc_hz =
        s->clock_select & (NEXT_SERIAL_A_4MHZ | NEXT_SERIAL_A_ESCLK) ?
        NEXT_SERIAL_RTXC_HZ : NEXT_SERIAL_PCLK_HZ;

    escc_set_clock_inputs(&s->escc, pclk_hz,
                          ch_b_rtxc_hz, ch_a_rtxc_hz);
}

static uint64_t next_serial_clock_read(void *opaque, hwaddr addr,
                                       unsigned size)
{
    NextSerialState *s = NEXT_SERIAL(opaque);

    return s->clock_select;
}

static void next_serial_clock_write(void *opaque, hwaddr addr,
                                    uint64_t value, unsigned size)
{
    NextSerialState *s = NEXT_SERIAL(opaque);

    s->clock_select = value;
    if (s->clock_select & (NEXT_SERIAL_PCLK_ESCLK |
                           NEXT_SERIAL_B_ESCLK |
                           NEXT_SERIAL_A_ESCLK)) {
        qemu_log_mask(LOG_UNIMP,
                      "next-serial: ESCLK selected; using 4 MHz timing\n");
    }
    next_serial_apply_clocks(s);
}

static const MemoryRegionOps next_serial_clock_ops = {
    .read = next_serial_clock_read,
    .write = next_serial_clock_write,
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

static void next_serial_reset(DeviceState *dev)
{
    NextSerialState *s = NEXT_SERIAL(dev);

    s->clock_select = 0;
    s->irq_level[0] = false;
    s->irq_level[1] = false;
    next_serial_apply_clocks(s);
    next_serial_update_irq(s);
}

static int next_serial_post_load(void *opaque, int version_id)
{
    NextSerialState *s = opaque;

    next_serial_apply_clocks(s);
    next_serial_update_irq(s);
    return 0;
}

static const VMStateDescription next_serial_vmstate = {
    .name = "next-serial",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = next_serial_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(clock_select, NextSerialState),
        VMSTATE_BOOL_ARRAY(irq_level, NextSerialState, 2),
        VMSTATE_END_OF_LIST()
    },
};

static void next_serial_realize(DeviceState *dev, Error **errp)
{
    NextSerialState *s = NEXT_SERIAL(dev);
    DeviceState *escc_dev = DEVICE(&s->escc);
    SysBusDevice *escc_sbd = SYS_BUS_DEVICE(escc_dev);

    qdev_prop_set_uint32(escc_dev, "disabled", 0);
    qdev_prop_set_uint32(escc_dev, "frequency", 2 * NEXT_SERIAL_PCLK_HZ);
    qdev_prop_set_uint32(escc_dev, "it_shift", 0);
    qdev_prop_set_bit(escc_dev, "bit_swap", true);
    qdev_prop_set_uint32(escc_dev, "chnBtype", escc_serial);
    qdev_prop_set_uint32(escc_dev, "chnAtype", escc_serial);

    if (!sysbus_realize(escc_sbd, errp)) {
        return;
    }

    memory_region_add_subregion(&s->mmio, 0,
                                sysbus_mmio_get_region(escc_sbd, 0));
    sysbus_connect_irq(escc_sbd, 0, qdev_get_gpio_in(dev, 0));
    sysbus_connect_irq(escc_sbd, 1, qdev_get_gpio_in(dev, 1));
    next_serial_apply_clocks(s);
}

static void next_serial_init(Object *obj)
{
    NextSerialState *s = NEXT_SERIAL(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    object_initialize_child(obj, "escc", &s->escc, TYPE_ESCC);
    object_property_add_alias(obj, "chrA", OBJECT(&s->escc), "chrA");
    object_property_add_alias(obj, "chrB", OBJECT(&s->escc), "chrB");

    memory_region_init(&s->mmio, obj, "next.serial",
                       NEXT_SERIAL_MMIO_SIZE);
    memory_region_init_io(&s->clock_mem, obj, &next_serial_clock_ops, s,
                          "next.serial-clock", 1);
    memory_region_add_subregion(&s->mmio, 4, &s->clock_mem);
    sysbus_init_mmio(sbd, &s->mmio);

    qdev_init_gpio_in(DEVICE(obj), next_serial_set_irq, 2);
    sysbus_init_irq(sbd, &s->irq);
}

static void next_serial_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "NeXT dual-channel serial controller";
    dc->realize = next_serial_realize;
    dc->vmsd = &next_serial_vmstate;
    device_class_set_legacy_reset(dc, next_serial_reset);
}

static const TypeInfo next_serial_info = {
    .name = TYPE_NEXT_SERIAL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NextSerialState),
    .instance_init = next_serial_init,
    .class_init = next_serial_class_init,
};

static void next_serial_register_types(void)
{
    type_register_static(&next_serial_info);
}

type_init(next_serial_register_types)
