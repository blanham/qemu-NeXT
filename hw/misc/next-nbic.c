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
#include "hw/core/sysbus.h"
#include "hw/misc/next-nbic.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define NEXT_NBIC_MMIO_SIZE     8
#define NEXT_NBIC_CONTROL_MASK  0x1c000000
#define NEXT_NBIC_ID_MASK       0xffff0000

struct NextNBICState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint32_t control;
    uint32_t id;
};

static unsigned next_nbic_lane_shift(hwaddr addr, unsigned size)
{
    return (4 - ((addr & 3) + size)) * 8;
}

static uint32_t next_nbic_size_mask(unsigned size)
{
    return size == 4 ? UINT32_MAX : (1U << (size * 8)) - 1;
}

static uint32_t *next_nbic_register(NextNBICState *s, hwaddr addr)
{
    return addr < 4 ? &s->control : &s->id;
}

static uint32_t next_nbic_writable_mask(hwaddr addr)
{
    return addr < 4 ? NEXT_NBIC_CONTROL_MASK : NEXT_NBIC_ID_MASK;
}

static uint64_t next_nbic_read(void *opaque, hwaddr addr, unsigned size)
{
    NextNBICState *s = NEXT_NBIC(opaque);
    unsigned shift = next_nbic_lane_shift(addr, size);

    return (*next_nbic_register(s, addr) >> shift) &
           next_nbic_size_mask(size);
}

static void next_nbic_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned size)
{
    NextNBICState *s = NEXT_NBIC(opaque);
    uint32_t *reg = next_nbic_register(s, addr);
    unsigned shift = next_nbic_lane_shift(addr, size);
    uint32_t lane_mask = next_nbic_size_mask(size) << shift;
    uint32_t merged = (*reg & ~lane_mask) |
                      (((uint32_t)value << shift) & lane_mask);

    *reg = merged & next_nbic_writable_mask(addr);
}

static bool next_nbic_access_valid(void *opaque, hwaddr addr,
                                   unsigned size, bool is_write,
                                   MemTxAttrs attrs)
{
    if (addr >= NEXT_NBIC_MMIO_SIZE || size < 1 || size > 4) {
        return false;
    }

    return (addr & 3) + size <= 4;
}

static const MemoryRegionOps next_nbic_ops = {
    .read = next_nbic_read,
    .write = next_nbic_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
        .unaligned = true,
        .accepts = next_nbic_access_valid,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
        .unaligned = true,
    },
};

static void next_nbic_reset_hold(Object *obj, ResetType type)
{
    NextNBICState *s = NEXT_NBIC(obj);

    s->control = 0;
    s->id = 0;
}

static const VMStateDescription vmstate_next_nbic = {
    .name = TYPE_NEXT_NBIC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(control, NextNBICState),
        VMSTATE_UINT32(id, NextNBICState),
        VMSTATE_END_OF_LIST()
    },
};

static void next_nbic_init(Object *obj)
{
    NextNBICState *s = NEXT_NBIC(obj);

    memory_region_init_io(&s->mmio, obj, &next_nbic_ops, s,
                          TYPE_NEXT_NBIC, NEXT_NBIC_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void next_nbic_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "NeXTbus interface controller";
    dc->vmsd = &vmstate_next_nbic;
    rc->phases.hold = next_nbic_reset_hold;
}

static const TypeInfo next_nbic_info = {
    .name = TYPE_NEXT_NBIC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NextNBICState),
    .instance_init = next_nbic_init,
    .class_init = next_nbic_class_init,
};

static void next_nbic_register_types(void)
{
    type_register_static(&next_nbic_info);
}

type_init(next_nbic_register_types)
