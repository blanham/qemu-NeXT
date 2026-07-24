/* SPDX-License-Identifier: NCSA
 *
 * QEMU NeXT sound output emulation
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
#include "hw/audio/next-sound.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "hw/dma/next-dma.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/audio.h"
#include "qemu/bswap.h"
#include "qemu/module.h"

#define NEXT_DMAOUT_DMAEN 0x80000000
#define NEXT_DMAOUT_OVR   0x20000000

#define MON_SOUND_OUT        0xc7
#define MON_GP_OUT           0xc4
#define MON_SNDOUT_CTRL_MASK 0x07

#define SOUT_ENAB 0x01
#define SOUT_DOUB 0x02
#define SOUT_ZERO 0x04

#define SGP_SPKREN 0x10

#define NEXT_SOUND_FRAME_BYTES 4
#define NEXT_SOUND_INPUT_BYTES 4096
#define NEXT_SOUND_PENDING_BYTES (NEXT_SOUND_INPUT_BYTES * 2)

struct NextSoundState {
    DeviceState parent_obj;

    NextDMAState *dma;
    AudioBackend *audio_be;
    SWVoiceOut *voice;
    qemu_irq overrun_irq;

    uint8_t control;
    uint8_t gp_flags;
    bool dma_enabled;
    bool overrun;
    uint8_t pending[NEXT_SOUND_PENDING_BYTES];
    uint32_t pending_offset;
    uint32_t pending_length;
};

static bool next_sound_enabled(const NextSoundState *s)
{
    return (s->control & SOUT_ENAB) &&
           (s->dma_enabled || s->pending_length);
}

static void next_sound_set_overrun(NextSoundState *s, bool overrun)
{
    s->overrun = overrun;
    qemu_set_irq(s->overrun_irq, overrun);
}

static void next_sound_update_volume(NextSoundState *s)
{
    audio_be_set_volume_out_lr(s->audio_be, s->voice,
                               s->gp_flags & SGP_SPKREN, 255, 255);
}

static void next_sound_set_active(NextSoundState *s)
{
    audio_be_set_active_out(s->audio_be, s->voice, next_sound_enabled(s));
}

static void next_sound_compact_pending(NextSoundState *s)
{
    if (!s->pending_length) {
        s->pending_offset = 0;
    } else if (s->pending_offset) {
        memmove(s->pending, s->pending + s->pending_offset,
                s->pending_length);
        s->pending_offset = 0;
    }
}

static void next_sound_emit_frame(uint8_t **output, const uint8_t *input)
{
    memcpy(*output, input, NEXT_SOUND_FRAME_BYTES);
    *output += NEXT_SOUND_FRAME_BYTES;
}

static NextDMAResult next_sound_fill_pending(NextSoundState *s)
{
    uint8_t input[NEXT_SOUND_INPUT_BYTES];
    size_t output_space;
    size_t input_capacity;
    size_t input_length;
    uint8_t *output;
    bool double_rate = s->control & SOUT_DOUB;
    NextDMAResult result;

    next_sound_compact_pending(s);
    output_space = sizeof(s->pending) - s->pending_length;
    input_capacity = double_rate ? output_space / 2 : output_space;
    input_capacity = MIN(input_capacity, sizeof(input));
    input_capacity &= ~(size_t)(NEXT_SOUND_FRAME_BYTES - 1);
    if (!input_capacity) {
        return NEXT_DMA_NO_SPACE;
    }

    result = next_dma_sound_out_read(s->dma, input, input_capacity,
                                     &input_length);
    if (result != NEXT_DMA_OK) {
        return result;
    }

    output = s->pending + s->pending_length;
    for (size_t offset = 0; offset < input_length;
         offset += NEXT_SOUND_FRAME_BYTES) {
        next_sound_emit_frame(&output, input + offset);
        if (double_rate) {
            if (s->control & SOUT_ZERO) {
                memset(output, 0, NEXT_SOUND_FRAME_BYTES);
                output += NEXT_SOUND_FRAME_BYTES;
            } else {
                next_sound_emit_frame(&output, input + offset);
            }
        }
    }
    s->pending_length = output - s->pending;
    return NEXT_DMA_OK;
}

static bool next_sound_pump(NextSoundState *s, size_t available,
                            bool explicit_kick)
{
    bool produced = s->pending_length != 0;

    while (next_sound_enabled(s) && available >= NEXT_SOUND_FRAME_BYTES) {
        size_t written;

        if (!s->pending_length) {
            NextDMAResult result = next_sound_fill_pending(s);

            if (result == NEXT_DMA_NOT_READY) {
                break;
            }
            if (result != NEXT_DMA_OK) {
                next_sound_set_overrun(s, true);
                break;
            }
            produced = true;
        }

        written = audio_be_write(s->audio_be, s->voice,
                                 s->pending + s->pending_offset,
                                 MIN(available,
                                     (size_t)s->pending_length));
        if (!written) {
            break;
        }
        s->pending_offset += written;
        s->pending_length -= written;
        available -= written;
        if (!s->pending_length) {
            s->pending_offset = 0;
        }
    }

    if (explicit_kick && !produced && !s->pending_length &&
        next_sound_enabled(s)) {
        next_sound_set_overrun(s, true);
    }
    return produced;
}

static void next_sound_kick(NextSoundState *s, bool explicit_kick)
{
    int available;

    next_sound_set_active(s);
    if (!next_sound_enabled(s)) {
        return;
    }
    available = audio_be_get_buffer_size_out(s->audio_be, s->voice);
    next_sound_pump(s, MAX(available, NEXT_SOUND_FRAME_BYTES),
                    explicit_kick);
}

static void next_sound_out_cb(void *opaque, int available)
{
    NextSoundState *s = opaque;

    if (available > 0) {
        next_sound_pump(s, available, false);
    }
}

static bool next_sound_queue_direct_frame(NextSoundState *s, uint32_t data)
{
    uint8_t frame[NEXT_SOUND_FRAME_BYTES];
    uint8_t *output;
    size_t needed = (s->control & SOUT_DOUB) ?
                    2 * NEXT_SOUND_FRAME_BYTES : NEXT_SOUND_FRAME_BYTES;

    next_sound_compact_pending(s);
    if (sizeof(s->pending) - s->pending_length < needed) {
        return false;
    }

    stl_be_p(frame, data);
    output = s->pending + s->pending_length;
    next_sound_emit_frame(&output, frame);
    if (s->control & SOUT_DOUB) {
        if (s->control & SOUT_ZERO) {
            memset(output, 0, NEXT_SOUND_FRAME_BYTES);
            output += NEXT_SOUND_FRAME_BYTES;
        } else {
            next_sound_emit_frame(&output, frame);
        }
    }
    s->pending_length = output - s->pending;
    return true;
}

static void next_sound_stage_kickstarted_dma(NextSoundState *s)
{
    NextDMAResult result;

    if (!(s->control & SOUT_ENAB) || !s->dma_enabled) {
        return;
    }

    result = next_sound_fill_pending(s);
    if (result != NEXT_DMA_OK && result != NEXT_DMA_NOT_READY) {
        next_sound_set_overrun(s, true);
    }
}

uint32_t next_sound_monitor_csr(NextSoundState *s)
{
    uint32_t value = 0;

    if (s->dma_enabled) {
        value |= NEXT_DMAOUT_DMAEN;
    }
    if (s->overrun) {
        value |= NEXT_DMAOUT_OVR;
    }
    return value;
}

void next_sound_monitor_csr_write(NextSoundState *s, uint8_t value)
{
    if (value & (NEXT_DMAOUT_OVR >> 24)) {
        next_sound_set_overrun(s, false);
    }
    s->dma_enabled = value & (NEXT_DMAOUT_DMAEN >> 24);
    next_sound_kick(s, false);
}

void next_sound_monitor_command(NextSoundState *s, uint8_t command,
                                uint32_t data)
{
    if (command == MON_SOUND_OUT) {
        if (!next_sound_queue_direct_frame(s, data)) {
            next_sound_set_overrun(s, true);
        }
        next_sound_stage_kickstarted_dma(s);
        next_sound_kick(s, true);
        return;
    }
    if (command == MON_GP_OUT) {
        s->gp_flags = data >> 24;
        next_sound_update_volume(s);
        return;
    }
    if ((command & MON_SNDOUT_CTRL_MASK) == MON_SNDOUT_CTRL_MASK) {
        s->control = (command >> 3) & (SOUT_ENAB | SOUT_DOUB | SOUT_ZERO);
        if (!(s->control & SOUT_ENAB)) {
            s->pending_offset = 0;
            s->pending_length = 0;
        }
        next_sound_kick(s, true);
    }
}

static int next_sound_post_load(void *opaque, int version_id)
{
    NextSoundState *s = opaque;

    if (s->pending_offset > sizeof(s->pending) ||
        s->pending_length > sizeof(s->pending) - s->pending_offset ||
        (s->pending_offset | s->pending_length) &
        (NEXT_SOUND_FRAME_BYTES - 1)) {
        return -EINVAL;
    }
    qemu_set_irq(s->overrun_irq, s->overrun);
    next_sound_update_volume(s);
    next_sound_set_active(s);
    return 0;
}

static const VMStateDescription vmstate_next_sound = {
    .name = TYPE_NEXT_SOUND,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = next_sound_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(control, NextSoundState),
        VMSTATE_UINT8(gp_flags, NextSoundState),
        VMSTATE_BOOL(dma_enabled, NextSoundState),
        VMSTATE_BOOL(overrun, NextSoundState),
        VMSTATE_UINT8_ARRAY(pending, NextSoundState,
                            NEXT_SOUND_PENDING_BYTES),
        VMSTATE_UINT32(pending_offset, NextSoundState),
        VMSTATE_UINT32(pending_length, NextSoundState),
        VMSTATE_END_OF_LIST()
    },
};

static void next_sound_reset_hold(Object *obj, ResetType type)
{
    NextSoundState *s = NEXT_SOUND(obj);

    s->control = 0;
    s->gp_flags = 0;
    s->dma_enabled = false;
    s->pending_offset = 0;
    s->pending_length = 0;
    next_sound_set_overrun(s, false);
    next_sound_update_volume(s);
    next_sound_set_active(s);
}

static void next_sound_realize(DeviceState *dev, Error **errp)
{
    NextSoundState *s = NEXT_SOUND(dev);
    struct audsettings settings = {
        .freq = 44100,
        .nchannels = 2,
        .fmt = AUDIO_FORMAT_S16,
        .big_endian = true,
    };

    if (!s->dma) {
        error_setg(errp, "next-sound requires a next-dma link");
        return;
    }
    if (!audio_be_check(&s->audio_be, errp)) {
        return;
    }

    s->voice = audio_be_open_out(s->audio_be, NULL, "next-sound.out", s,
                                 next_sound_out_cb, &settings);
    if (!s->voice) {
        error_setg(errp, "initializing NeXT sound output failed");
        return;
    }
    next_sound_update_volume(s);
    next_sound_set_active(s);
}

static void next_sound_unrealize(DeviceState *dev)
{
    NextSoundState *s = NEXT_SOUND(dev);

    audio_be_close_out(s->audio_be, s->voice);
    s->voice = NULL;
}

static const Property next_sound_properties[] = {
    DEFINE_PROP_LINK("dma", NextSoundState, dma, TYPE_NEXT_DMA,
                     NextDMAState *),
    DEFINE_AUDIO_PROPERTIES(NextSoundState, audio_be),
};

static void next_sound_init(Object *obj)
{
    NextSoundState *s = NEXT_SOUND(obj);

    qdev_init_gpio_out(DEVICE(obj), &s->overrun_irq, 1);
}

static void next_sound_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = next_sound_realize;
    dc->unrealize = next_sound_unrealize;
    dc->vmsd = &vmstate_next_sound;
    device_class_set_props(dc, next_sound_properties);
    set_bit(DEVICE_CATEGORY_SOUND, dc->categories);
    rc->phases.hold = next_sound_reset_hold;
}

static const TypeInfo next_sound_info = {
    .name = TYPE_NEXT_SOUND,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(NextSoundState),
    .instance_init = next_sound_init,
    .class_init = next_sound_class_init,
};

static void next_sound_register_types(void)
{
    type_register_static(&next_sound_info);
}

type_init(next_sound_register_types)
