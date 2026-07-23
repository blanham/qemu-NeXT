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
#include "hw/misc/next-memctl.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define NEXT_MEMCTL_TIMING_COUNT 5

struct NextMemCtlState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    uint8_t timing[NEXT_MEMCTL_TIMING_COUNT];
};

static uint64_t next_memctl_read(void *opaque, hwaddr addr,
                                 unsigned int size)
{
    NextMemCtlState *s = NEXT_MEMCTL(opaque);

    return s->timing[addr];
}

static void next_memctl_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned int size)
{
    NextMemCtlState *s = NEXT_MEMCTL(opaque);

    s->timing[addr] = value;
}

static bool next_memctl_access_valid(void *opaque, hwaddr addr,
                                     unsigned int size, bool is_write,
                                     MemTxAttrs attrs)
{
    return size == 1;
}

static const MemoryRegionOps next_memctl_ops = {
    .read = next_memctl_read,
    .write = next_memctl_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        /*
         * Preserve word and long transactions so accepts() can reject them.
         * A max of one would silently split them into valid byte accesses.
         */
        .max_access_size = 4,
        .unaligned = false,
        .accepts = next_memctl_access_valid,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void next_memctl_reset_hold(Object *obj, ResetType type)
{
    NextMemCtlState *s = NEXT_MEMCTL(obj);

    memset(s->timing, 0, sizeof(s->timing));
}

static const VMStateDescription vmstate_next_memctl = {
    .name = "next-memctl",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(timing, NextMemCtlState,
                            NEXT_MEMCTL_TIMING_COUNT),
        VMSTATE_END_OF_LIST()
    },
};

static void next_memctl_init(Object *obj)
{
    NextMemCtlState *s = NEXT_MEMCTL(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &next_memctl_ops, s,
                          "next.memctl", NEXT_MEMCTL_TIMING_COUNT);
    sysbus_init_mmio(sbd, &s->mmio);
}

static void next_memctl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "NeXT memory timing registers";
    dc->vmsd = &vmstate_next_memctl;
    rc->phases.hold = next_memctl_reset_hold;
}

static const TypeInfo next_memctl_info = {
    .name = TYPE_NEXT_MEMCTL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NextMemCtlState),
    .instance_init = next_memctl_init,
    .class_init = next_memctl_class_init,
};

static void next_memctl_register_types(void)
{
    type_register_static(&next_memctl_info);
}

type_init(next_memctl_register_types)
