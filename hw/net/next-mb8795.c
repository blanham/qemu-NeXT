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
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/net/next-mb8795.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define NEXT_MB8795_MMIO_SIZE 0x10

#define NEXT_MB8795_TXSTAT    0x00
#define NEXT_MB8795_TXMASK    0x01
#define NEXT_MB8795_RXSTAT    0x02
#define NEXT_MB8795_RXMASK    0x03
#define NEXT_MB8795_TXMODE    0x04
#define NEXT_MB8795_RXMODE    0x05
#define NEXT_MB8795_RESET     0x06
#define NEXT_MB8795_ADDR      0x08
#define NEXT_MB8795_ADDR_SIZE 6

#define NEXT_MB8795_TXSTAT_READY 0x80
#define NEXT_MB8795_RESET_MODE   0x80

struct NextMB8795State {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    NICState *nic;
    NICConf conf;
    NextDMAState *dma;
    QEMUTimer tx_timer;
    qemu_irq tx_irq;
    qemu_irq rx_irq;
    uint8_t tx_status;
    uint8_t tx_mask;
    uint8_t rx_status;
    uint8_t rx_mask;
    uint8_t tx_mode;
    uint8_t rx_mode;
    uint8_t station[6];
    bool reset;
};

static void next_mb8795_update_tx_irq(NextMB8795State *s)
{
    qemu_set_irq(s->tx_irq, !!(s->tx_status & s->tx_mask));
}

static void next_mb8795_update_rx_irq(NextMB8795State *s)
{
    qemu_set_irq(s->rx_irq, !!(s->rx_status & s->rx_mask));
}

static void next_mb8795_update_irqs(NextMB8795State *s)
{
    next_mb8795_update_tx_irq(s);
    next_mb8795_update_rx_irq(s);
}

static void next_mb8795_enter_reset(NextMB8795State *s)
{
    timer_del(&s->tx_timer);
    s->tx_status = 0;
    s->tx_mask = 0;
    s->rx_status = 0;
    s->rx_mask = 0;
    s->tx_mode = 0;
    s->rx_mode = 0;
    s->reset = true;
    qemu_irq_lower(s->tx_irq);
    qemu_irq_lower(s->rx_irq);
}

static void next_mb8795_leave_reset(NextMB8795State *s)
{
    s->reset = false;
    s->tx_status |= NEXT_MB8795_TXSTAT_READY;
    next_mb8795_update_tx_irq(s);
}

static uint64_t next_mb8795_read(void *opaque, hwaddr addr,
                                 unsigned int size)
{
    NextMB8795State *s = NEXT_MB8795(opaque);

    switch (addr) {
    case NEXT_MB8795_TXSTAT:
        return s->tx_status;
    case NEXT_MB8795_TXMASK:
        return s->tx_mask;
    case NEXT_MB8795_RXSTAT:
        return s->rx_status;
    case NEXT_MB8795_RXMASK:
        return s->rx_mask;
    case NEXT_MB8795_TXMODE:
        return s->tx_mode;
    case NEXT_MB8795_RXMODE:
        return s->rx_mode;
    case NEXT_MB8795_RESET:
        return s->reset ? NEXT_MB8795_RESET_MODE : 0;
    case NEXT_MB8795_ADDR ... NEXT_MB8795_ADDR + NEXT_MB8795_ADDR_SIZE - 1:
        return s->station[addr - NEXT_MB8795_ADDR];
    default:
        return 0;
    }
}

static void next_mb8795_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned int size)
{
    NextMB8795State *s = NEXT_MB8795(opaque);

    switch (addr) {
    case NEXT_MB8795_TXSTAT:
        s->tx_status &= ~value;
        next_mb8795_update_tx_irq(s);
        break;
    case NEXT_MB8795_TXMASK:
        s->tx_mask = value;
        next_mb8795_update_tx_irq(s);
        break;
    case NEXT_MB8795_RXSTAT:
        s->rx_status &= ~value;
        next_mb8795_update_rx_irq(s);
        break;
    case NEXT_MB8795_RXMASK:
        s->rx_mask = value;
        next_mb8795_update_rx_irq(s);
        break;
    case NEXT_MB8795_TXMODE:
        s->tx_mode = value;
        next_mb8795_update_tx_irq(s);
        break;
    case NEXT_MB8795_RXMODE:
        s->rx_mode = value;
        next_mb8795_update_rx_irq(s);
        break;
    case NEXT_MB8795_RESET:
        if (value & NEXT_MB8795_RESET_MODE) {
            next_mb8795_enter_reset(s);
        } else {
            next_mb8795_leave_reset(s);
        }
        break;
    case NEXT_MB8795_ADDR ... NEXT_MB8795_ADDR + NEXT_MB8795_ADDR_SIZE - 1:
        s->station[addr - NEXT_MB8795_ADDR] = value;
        break;
    default:
        break;
    }
}

static bool next_mb8795_access_valid(void *opaque, hwaddr addr,
                                     unsigned int size, bool is_write,
                                     MemTxAttrs attrs)
{
    return size == 1;
}

static const MemoryRegionOps next_mb8795_ops = {
    .read = next_mb8795_read,
    .write = next_mb8795_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        /*
         * Keep the original transaction intact until accepts() rejects
         * non-byte accesses.  A max of one here would make AddressSpace
         * silently split word and long requests into valid byte accesses.
         */
        .max_access_size = 4,
        .unaligned = false,
        .accepts = next_mb8795_access_valid,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static bool next_mb8795_can_receive(NetClientState *nc)
{
    return false;
}

static ssize_t next_mb8795_receive(NetClientState *nc,
                                   const uint8_t *buf, size_t size)
{
    return 0;
}

static NetClientInfo next_mb8795_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = next_mb8795_can_receive,
    .receive = next_mb8795_receive,
};

static void next_mb8795_tx_timer(void *opaque)
{
    /*
     * Task 7 supplies the bounded ENTX transfer.  Keeping the callback
     * harmless now makes reset and migration safe without inventing a data
     * path.
     */
}

static void next_mb8795_tx_kick(void *opaque)
{
    NextMB8795State *s = NEXT_MB8795(opaque);

    if (!s->reset) {
        timer_mod(&s->tx_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1);
    }
}

static void next_mb8795_rx_ready_changed(void *opaque, bool ready)
{
    NextMB8795State *s = NEXT_MB8795(opaque);

    if (ready && s->nic) {
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
    }
}

const NextDMAEthernetNotify next_mb8795_dma_notify = {
    .tx_kick = next_mb8795_tx_kick,
    .rx_ready_changed = next_mb8795_rx_ready_changed,
};

static void next_mb8795_reset_hold(Object *obj, ResetType type)
{
    NextMB8795State *s = NEXT_MB8795(obj);

    /*
     * Retained NeXT and NetBSD drivers begin controller initialization in
     * RESET_MODE.  Keep that safe latch on QOM reset while preserving the
     * configured/programmed station address.
     */
    next_mb8795_enter_reset(s);
}

static int next_mb8795_post_load(void *opaque, int version_id)
{
    NextMB8795State *s = opaque;

    if (s->reset &&
        (s->tx_status || s->tx_mask || s->rx_status || s->rx_mask ||
         s->tx_mode || s->rx_mode || timer_pending(&s->tx_timer))) {
        return -EINVAL;
    }

    next_mb8795_update_irqs(s);
    return 0;
}

static const VMStateDescription vmstate_next_mb8795 = {
    .name = "next-mb8795",
    .priority = MIG_PRI_LOW,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = next_mb8795_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(tx_status, NextMB8795State),
        VMSTATE_UINT8(tx_mask, NextMB8795State),
        VMSTATE_UINT8(rx_status, NextMB8795State),
        VMSTATE_UINT8(rx_mask, NextMB8795State),
        VMSTATE_UINT8(tx_mode, NextMB8795State),
        VMSTATE_UINT8(rx_mode, NextMB8795State),
        VMSTATE_UINT8_ARRAY(station, NextMB8795State,
                            NEXT_MB8795_ADDR_SIZE),
        VMSTATE_BOOL(reset, NextMB8795State),
        VMSTATE_TIMER(tx_timer, NextMB8795State),
        VMSTATE_END_OF_LIST()
    },
};

static void next_mb8795_realize(DeviceState *dev, Error **errp)
{
    NextMB8795State *s = NEXT_MB8795(dev);

    if (!s->dma) {
        error_setg(errp, "'dma' link is not set");
        return;
    }

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    memcpy(s->station, s->conf.macaddr.a, sizeof(s->station));
    s->nic = qemu_new_nic(&next_mb8795_net_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->station);
}

static void next_mb8795_unrealize(DeviceState *dev)
{
    NextMB8795State *s = NEXT_MB8795(dev);

    timer_del(&s->tx_timer);
    next_dma_set_ethernet_notify(s->dma, NULL, NULL);
    qemu_del_nic(s->nic);
    s->nic = NULL;
}

static void next_mb8795_init(Object *obj)
{
    NextMB8795State *s = NEXT_MB8795(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    sysbus_init_irq(sbd, &s->tx_irq);
    sysbus_init_irq(sbd, &s->rx_irq);
    memory_region_init_io(&s->mmio, obj, &next_mb8795_ops, s,
                          "next.mb8795", NEXT_MB8795_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->mmio);
    timer_init_ns(&s->tx_timer, QEMU_CLOCK_VIRTUAL,
                  next_mb8795_tx_timer, s);
}

static const Property next_mb8795_properties[] = {
    DEFINE_PROP_LINK("dma", NextMB8795State, dma,
                     TYPE_NEXT_DMA, NextDMAState *),
    DEFINE_NIC_PROPERTIES(NextMB8795State, conf),
};

static void next_mb8795_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "NeXT MB8795 Ethernet controller";
    dc->realize = next_mb8795_realize;
    dc->unrealize = next_mb8795_unrealize;
    dc->vmsd = &vmstate_next_mb8795;
    device_class_set_props(dc, next_mb8795_properties);
    rc->phases.hold = next_mb8795_reset_hold;
}

static const TypeInfo next_mb8795_info = {
    .name = TYPE_NEXT_MB8795,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NextMB8795State),
    .instance_init = next_mb8795_init,
    .class_init = next_mb8795_class_init,
};

static void next_mb8795_register_types(void)
{
    type_register_static(&next_mb8795_info);
}

type_init(next_mb8795_register_types)
