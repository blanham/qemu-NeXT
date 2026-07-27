/* SPDX-License-Identifier: NCSA
 *
 * QEMU NeXT Keyboard/Mouse emulation
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
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "hw/audio/next-sound.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/m68k/next-cube.h"
#include "standard-headers/linux/input-event-codes.h"
#include "ui/console.h"
#include "migration/vmstate.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(NextKBDState, NEXTKBD)

/* following definitions from next68k netbsd */
#define CSR_INT 0x00800000
#define CSR_DATA 0x00400000
#define CSR_OVR 0x00200000
#define CSR_BASE 0x00008300

#define KD_KEYMASK    0x007f
#define KD_DIRECTION  0x0080 /* pressed or released */
#define KD_CNTL       0x0100
#define KD_LSHIFT     0x0200
#define KD_RSHIFT     0x0400
#define KD_LCOMM      0x0800
#define KD_RCOMM      0x1000
#define KD_LALT       0x2000
#define KD_RALT       0x4000
#define KD_VALID      0x8000 /* only set for scancode keys ? */
#define KD_MODS       0x4f00

#define KBD_QUEUE_SIZE 256
#define NEXTKBD_KEY_COUNT 128

typedef struct {
    uint32_t data;
    bool keyboard;
} KBDQueueEntry;

typedef struct {
    KBDQueueEntry entries[KBD_QUEUE_SIZE];
    int rptr, wptr, count;
} KBDQueue;


struct NextKBDState {
    SysBusDevice sbd;
    MemoryRegion mr;
    QemuInputHandlerState *hs;
    qemu_irq irq;
    NextSoundState *sound;
    bool absolute_pointer;
    int32_t absolute_x;
    int32_t absolute_y;
    bool absolute_x_valid;
    bool absolute_y_valid;
    bool absolute_dirty;
    bool absolute_anchor_valid;
    int32_t absolute_anchor_x;
    int32_t absolute_anchor_y;
    KBDQueue queue;
    uint8_t command;
    uint32_t monitor_data;
    uint16_t shift;
    bool key_down[NEXTKBD_KEY_COUNT];
    bool overrun;
    int64_t mouse_dx;
    int64_t mouse_dy;
    bool mouse_left;
    bool mouse_right;
    bool mouse_button_pending;
};

static uint32_t nextkbd_csr(NextKBDState *s)
{
    uint32_t value = CSR_BASE | s->command;

    if (s->sound) {
        value |= next_sound_monitor_csr(s->sound);
    }

    if (s->queue.count || s->overrun) {
        value |= CSR_INT;
    }
    if (s->queue.count) {
        value |= CSR_DATA;
    }
    if (s->overrun) {
        value |= CSR_OVR;
    }

    return value;
}

/* lots of magic numbers here */
static uint32_t kbd_read_byte(void *opaque, hwaddr addr)
{
    NextKBDState *s = NEXTKBD(opaque);
    unsigned offset = addr & 0xf;

    if (offset < 4) {
        return nextkbd_csr(s) >> ((3 - offset) * 8) & 0xff;
    }

    qemu_log_mask(LOG_UNIMP, "NeXT kbd read byte %"HWADDR_PRIx"\n", addr);
    return 0;
}

static uint32_t kbd_read_word(void *opaque, hwaddr addr)
{
    NextKBDState *s = NEXTKBD(opaque);
    unsigned offset = addr & 0xf;

    if (offset == 0) {
        return nextkbd_csr(s) >> 16;
    }
    if (offset == 2) {
        return nextkbd_csr(s) & 0xffff;
    }

    qemu_log_mask(LOG_UNIMP, "NeXT kbd read word %"HWADDR_PRIx"\n", addr);
    return 0;
}

/* even more magic numbers */
static uint32_t kbd_read_long(void *opaque, hwaddr addr)
{
    uint32_t data;
    NextKBDState *s = NEXTKBD(opaque);
    KBDQueue *q = &s->queue;

    switch (addr & 0xf) {
    case 0x0:   /* 0xe000 */
        return nextkbd_csr(s);

    case 0x4:   /* 0xe004 */
        return s->monitor_data;

    case 0x8:   /* 0xe008 */
        if (q->count > 0) {
            KBDQueueEntry *entry = &q->entries[q->rptr];

            data = entry->data;
            if (entry->keyboard) {
                data &= ~(KD_LSHIFT | KD_RSHIFT);
                data |= s->shift;
            }
            if (++q->rptr == KBD_QUEUE_SIZE) {
                q->rptr = 0;
            }

            q->count--;
            qemu_set_irq(s->irq, q->count || s->overrun);
            return data;
        } else {
            return 0;
        }

    case 0xc:   /* 0xe00c */
        return 0;

    default:
        qemu_log_mask(LOG_UNIMP, "NeXT kbd read long %"HWADDR_PRIx"\n", addr);
        return 0;
    }
}

static uint64_t kbd_readfn(void *opaque, hwaddr addr, unsigned size)
{
    switch (size) {
    case 1:
        return kbd_read_byte(opaque, addr);
    case 2:
        return kbd_read_word(opaque, addr);
    case 4:
        return kbd_read_long(opaque, addr);
    default:
        g_assert_not_reached();
    }
}

static void kbd_writefn(void *opaque, hwaddr addr, uint64_t value,
                        unsigned size)
{
    NextKBDState *s = NEXTKBD(opaque);
    unsigned offset = addr & 0xf;

    if (offset < 4 && offset + size <= 4) {
        for (unsigned i = 0; i < size; i++) {
            uint8_t byte = value >> ((size - i - 1) * 8);

            switch (offset + i) {
            case 0:
                if (s->sound) {
                    next_sound_monitor_csr_write(s->sound, byte);
                }
                break;
            case 1:
                if (byte & (CSR_OVR >> 16)) {
                    s->overrun = false;
                    qemu_set_irq(s->irq, s->queue.count > 0);
                }
                break;
            case 3:
                s->command = byte;
                break;
            default:
                break;
            }
        }
        return;
    }

    if (offset == 4 && size == 4) {
        s->monitor_data = value;
        if (s->sound) {
            next_sound_monitor_command(s->sound, s->command,
                                       s->monitor_data);
        }
        return;
    }

    qemu_log_mask(LOG_UNIMP, "NeXT kbd write: size=%u addr=0x%"HWADDR_PRIx
                  "val=0x%"PRIx64"\n", size, addr, value);
}

static const MemoryRegionOps kbd_ops = {
    .read = kbd_readfn,
    .write = kbd_writefn,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .endianness = DEVICE_BIG_ENDIAN,
};

static const int linux_to_nextkbd_keycode[] = {
    [KEY_ESC]        = 0x49,
    [KEY_1]          = 0x4a,
    [KEY_2]          = 0x4b,
    [KEY_3]          = 0x4c,
    [KEY_4]          = 0x4d,
    [KEY_5]          = 0x50,
    [KEY_6]          = 0x4f,
    [KEY_7]          = 0x4e,
    [KEY_8]          = 0x1e,
    [KEY_9]          = 0x1f,
    [KEY_0]          = 0x20,
    [KEY_MINUS]      = 0x1d,
    [KEY_EQUAL]      = 0x1c,
    [KEY_BACKSPACE]  = 0x1b,

    [KEY_Q]          = 0x42,
    [KEY_W]          = 0x43,
    [KEY_E]          = 0x44,
    [KEY_R]          = 0x45,
    [KEY_T]          = 0x48,
    [KEY_Y]          = 0x47,
    [KEY_U]          = 0x46,
    [KEY_I]          = 0x06,
    [KEY_O]          = 0x07,
    [KEY_P]          = 0x08,
    [KEY_ENTER]      = 0x2a,
    [KEY_A]          = 0x39,
    [KEY_S]          = 0x3a,

    [KEY_D]          = 0x3b,
    [KEY_F]          = 0x3c,
    [KEY_G]          = 0x3d,
    [KEY_H]          = 0x40,
    [KEY_J]          = 0x3f,
    [KEY_K]          = 0x3e,
    [KEY_L]          = 0x2d,
    [KEY_SEMICOLON]  = 0x2c,
    [KEY_APOSTROPHE] = 0x2b,
    [KEY_GRAVE]      = 0x26,
    [KEY_Z]          = 0x31,
    [KEY_X]          = 0x32,
    [KEY_C]          = 0x33,
    [KEY_V]          = 0x34,

    [KEY_B]          = 0x35,
    [KEY_N]          = 0x37,
    [KEY_M]          = 0x36,
    [KEY_COMMA]      = 0x2e,
    [KEY_DOT]        = 0x2f,
    [KEY_SLASH]      = 0x30,

    [KEY_SPACE]      = 0x38,
};

static bool nextkbd_put_packet(NextKBDState *s, uint32_t packet,
                               bool keyboard)
{
    KBDQueue *q = &s->queue;

    if (q->count >= KBD_QUEUE_SIZE) {
        s->overrun = true;
        qemu_irq_raise(s->irq);
        return false;
    }

    q->entries[q->wptr].data = packet;
    q->entries[q->wptr].keyboard = keyboard;
    if (++q->wptr == KBD_QUEUE_SIZE) {
        q->wptr = 0;
    }

    q->count++;
    qemu_irq_raise(s->irq);
    return true;
}

static void nextkbd_key_event(NextKBDState *s, QemuInputEvent *evt)
{
    int keycode;

    if (evt->key.key >= ARRAY_SIZE(linux_to_nextkbd_keycode)) {
        return;
    }

    /* Shift key currently has no keycode, so handle separately */
    if (evt->key.key == KEY_LEFTSHIFT) {
        if (evt->key.down) {
            s->shift |= KD_LSHIFT;
        } else {
            s->shift &= ~KD_LSHIFT;
        }
    }

    if (evt->key.key == KEY_RIGHTSHIFT) {
        if (evt->key.down) {
            s->shift |= KD_RSHIFT;
        } else {
            s->shift &= ~KD_RSHIFT;
        }
    }

    keycode = linux_to_nextkbd_keycode[evt->key.key];
    if (!keycode) {
        return;
    }

    if (s->key_down[keycode] == evt->key.down) {
        return;
    }
    s->key_down[keycode] = evt->key.down;

    /* If key release event, create keyboard break code */
    if (!evt->key.down) {
        keycode |= 0x80;
    }

    nextkbd_put_packet(s, 0x10000000 | KD_VALID | s->shift | keycode, true);
}

static void nextkbd_button_event(NextKBDState *s, QemuInputEvent *evt)
{
    bool *button;

    switch (evt->btn.button) {
    case INPUT_BUTTON_LEFT:
        button = &s->mouse_left;
        break;
    case INPUT_BUTTON_RIGHT:
        button = &s->mouse_right;
        break;
    default:
        return;
    }

    if (*button != evt->btn.down) {
        *button = evt->btn.down;
        s->mouse_button_pending = true;
    }
}

static void nextkbd_relative_event(NextKBDState *s, QemuInputEvent *evt)
{
    int64_t *delta;

    if (evt->rel.axis == INPUT_AXIS_X) {
        delta = &s->mouse_dx;
    } else if (evt->rel.axis == INPUT_AXIS_Y) {
        delta = &s->mouse_dy;
    } else {
        return;
    }

    if (sadd64_overflow(*delta, evt->rel.value, delta)) {
        *delta = evt->rel.value < 0 ? INT64_MIN : INT64_MAX;
    }
}

static void nextkbd_absolute_event(NextKBDState *s, QemuInputEvent *evt)
{
    if (evt->abs.axis == INPUT_AXIS_X) {
        s->absolute_x = evt->abs.value;
        s->absolute_x_valid = true;
        s->absolute_dirty = true;
    } else if (evt->abs.axis == INPUT_AXIS_Y) {
        s->absolute_y = evt->abs.value;
        s->absolute_y_valid = true;
        s->absolute_dirty = true;
    }
}

static void nextkbd_event(DeviceState *dev, QemuConsole *src,
                          QemuInputEvent *evt)
{
    NextKBDState *s = NEXTKBD(dev);

    switch (evt->type) {
    case INPUT_EVENT_KIND_KEY:
        nextkbd_key_event(s, evt);
        break;
    case INPUT_EVENT_KIND_BTN:
        nextkbd_button_event(s, evt);
        break;
    case INPUT_EVENT_KIND_REL:
        nextkbd_relative_event(s, evt);
        break;
    case INPUT_EVENT_KIND_ABS:
        nextkbd_absolute_event(s, evt);
        break;
    default:
        break;
    }
}

static int nextkbd_mouse_delta(int64_t *delta)
{
    int raw;

    if (*delta > 64) {
        raw = -64;
    } else if (*delta < -63) {
        raw = 63;
    } else {
        raw = -*delta;
    }

    *delta += raw;
    return raw;
}

static bool nextkbd_put_mouse_packet(NextKBDState *s, int raw_dx, int raw_dy,
                                     bool left, bool right)
{
    uint32_t packet = 0x11000000 |
        ((raw_dy & 0x7f) << 9) |
        ((right ? 0 : 1) << 8) |
        ((raw_dx & 0x7f) << 1) |
        (left ? 0 : 1);

    return nextkbd_put_packet(s, packet, false);
}

static void nextkbd_relative_sync(NextKBDState *s)
{
    int64_t scaled_dx = s->mouse_dx / 3;
    int64_t scaled_dy = s->mouse_dy / 3;

    s->mouse_dx %= 3;
    s->mouse_dy %= 3;

    while (scaled_dx || scaled_dy || s->mouse_button_pending) {
        int raw_dx = nextkbd_mouse_delta(&scaled_dx);
        int raw_dy = nextkbd_mouse_delta(&scaled_dy);

        if (!nextkbd_put_mouse_packet(s, raw_dx, raw_dy,
                                      s->mouse_left, s->mouse_right)) {
            scaled_dx = 0;
            scaled_dy = 0;
            s->mouse_dx = 0;
            s->mouse_dy = 0;
        }
        s->mouse_button_pending = false;
    }
}

static void nextkbd_absolute_sync(NextKBDState *s)
{
    if (s->absolute_dirty &&
        s->absolute_x_valid && s->absolute_y_valid) {
        s->absolute_anchor_x = s->absolute_x;
        s->absolute_anchor_y = s->absolute_y;
        s->absolute_anchor_valid = true;
    }
    s->absolute_dirty = false;

    if (s->mouse_button_pending) {
        nextkbd_put_mouse_packet(s, 0, 0,
                                 s->mouse_left, s->mouse_right);
        s->mouse_button_pending = false;
    }
}

static void nextkbd_sync(DeviceState *dev)
{
    NextKBDState *s = NEXTKBD(dev);

    if (s->absolute_pointer) {
        nextkbd_absolute_sync(s);
    } else {
        nextkbd_relative_sync(s);
    }
}

static const QemuInputHandler nextkbd_absolute_handler = {
    .name = "QEMU NeXT Keyboard/Mouse",
    .mask = INPUT_EVENT_MASK_KEY | INPUT_EVENT_MASK_BTN |
            INPUT_EVENT_MASK_ABS,
    .event = nextkbd_event,
    .sync = nextkbd_sync,
};

static const QemuInputHandler nextkbd_relative_handler = {
    .name = "QEMU NeXT Keyboard/Mouse",
    .mask = INPUT_EVENT_MASK_KEY | INPUT_EVENT_MASK_BTN |
            INPUT_EVENT_MASK_REL,
    .event = nextkbd_event,
    .sync = nextkbd_sync,
};

static void nextkbd_reset(DeviceState *dev)
{
    NextKBDState *nks = NEXTKBD(dev);

    memset(&nks->queue, 0, sizeof(KBDQueue));
    nks->shift = 0;
    memset(nks->key_down, 0, sizeof(nks->key_down));
    nks->overrun = false;
    nks->command = 0;
    nks->monitor_data = 0;
    nks->mouse_dx = 0;
    nks->mouse_dy = 0;
    nks->mouse_left = false;
    nks->mouse_right = false;
    nks->mouse_button_pending = false;
    qemu_irq_lower(nks->irq);
}

static void nextkbd_realize(DeviceState *dev, Error **errp)
{
    NextKBDState *s = NEXTKBD(dev);

    memory_region_init_io(&s->mr, OBJECT(dev), &kbd_ops, s, "next.kbd", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mr);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    s->hs = qemu_input_handler_register(
        dev, s->absolute_pointer
        ? &nextkbd_absolute_handler : &nextkbd_relative_handler);
}

static void nextkbd_unrealize(DeviceState *dev)
{
    NextKBDState *s = NEXTKBD(dev);

    g_clear_pointer(&s->hs, qemu_input_handler_unregister);
}

static const VMStateDescription nextkbd_queue_entry_vmstate = {
    .name = TYPE_NEXTKBD "/queue-entry",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(data, KBDQueueEntry),
        VMSTATE_BOOL(keyboard, KBDQueueEntry),
        VMSTATE_END_OF_LIST()
    },
};

static int nextkbd_post_load(void *opaque, int version_id)
{
    NextKBDState *s = opaque;

    if (s->queue.rptr < 0 || s->queue.rptr >= KBD_QUEUE_SIZE ||
        s->queue.wptr < 0 || s->queue.wptr >= KBD_QUEUE_SIZE ||
        s->queue.count < 0 || s->queue.count > KBD_QUEUE_SIZE ||
        (s->queue.rptr + s->queue.count) % KBD_QUEUE_SIZE !=
            s->queue.wptr ||
        (s->shift & ~(KD_LSHIFT | KD_RSHIFT))) {
        return -EINVAL;
    }

    qemu_set_irq(s->irq, s->queue.count || s->overrun);
    return 0;
}

static const VMStateDescription nextkbd_vmstate = {
    .name = TYPE_NEXTKBD,
    .version_id = 2,
    .minimum_version_id = 1,
    .post_load = nextkbd_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(queue.entries, NextKBDState, KBD_QUEUE_SIZE, 1,
                             nextkbd_queue_entry_vmstate, KBDQueueEntry),
        VMSTATE_INT32(queue.rptr, NextKBDState),
        VMSTATE_INT32(queue.wptr, NextKBDState),
        VMSTATE_INT32(queue.count, NextKBDState),
        VMSTATE_UINT8(command, NextKBDState),
        VMSTATE_UINT32(monitor_data, NextKBDState),
        VMSTATE_UINT16(shift, NextKBDState),
        VMSTATE_BOOL_ARRAY_V(key_down, NextKBDState, NEXTKBD_KEY_COUNT, 2),
        VMSTATE_BOOL(overrun, NextKBDState),
        VMSTATE_INT64(mouse_dx, NextKBDState),
        VMSTATE_INT64(mouse_dy, NextKBDState),
        VMSTATE_BOOL(mouse_left, NextKBDState),
        VMSTATE_BOOL(mouse_right, NextKBDState),
        VMSTATE_BOOL(mouse_button_pending, NextKBDState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property nextkbd_properties[] = {
    DEFINE_PROP_BOOL("absolute-pointer", NextKBDState,
                     absolute_pointer, true),
    DEFINE_PROP_LINK("sound", NextKBDState, sound, TYPE_NEXT_SOUND,
                     NextSoundState *),
};

static void nextkbd_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
    dc->vmsd = &nextkbd_vmstate;
    dc->realize = nextkbd_realize;
    dc->unrealize = nextkbd_unrealize;
    device_class_set_props(dc, nextkbd_properties);
    device_class_set_legacy_reset(dc, nextkbd_reset);
}

static const TypeInfo nextkbd_info = {
    .name          = TYPE_NEXTKBD,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NextKBDState),
    .class_init    = nextkbd_class_init,
};

static void nextkbd_register_types(void)
{
    type_register_static(&nextkbd_info);
}

type_init(nextkbd_register_types)
