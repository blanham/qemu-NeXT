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
#include "qemu/timer.h"
#include "system/reset.h"

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
#define NEXT_SOUND_SAMPLE_RATE 44100
#define NEXT_SOUND_TIMER_FRAMES 128
#define NEXT_SOUND_TIMER_NS \
    DIV_ROUND_UP(INT64_C(1000000000) * NEXT_SOUND_TIMER_FRAMES, \
                 NEXT_SOUND_SAMPLE_RATE)
#define NEXT_SOUND_MAX_CATCHUP_FRAMES NEXT_SOUND_SAMPLE_RATE
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
    QEMUTimer dma_timer;
    int64_t dma_clock_ns;
    uint64_t dma_fraction;
};

static bool next_sound_enabled(const NextSoundState *s)
{
    return (s->control & SOUT_ENAB) &&
           (s->dma_enabled || s->pending_length);
}

static bool next_sound_dma_enabled(const NextSoundState *s)
{
    return (s->control & SOUT_ENAB) && s->dma_enabled;
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

static void next_sound_append_dma_frames(NextSoundState *s,
                                         const uint8_t *input,
                                         size_t input_length)
{
    uint8_t *output;
    bool double_rate = s->control & SOUT_DOUB;

    next_sound_compact_pending(s);
    output = s->pending + s->pending_length;
    for (size_t offset = 0; offset < input_length;
         offset += NEXT_SOUND_FRAME_BYTES) {
        size_t output_length = double_rate ? 2 * NEXT_SOUND_FRAME_BYTES
                                           : NEXT_SOUND_FRAME_BYTES;

        if (sizeof(s->pending) - (output - s->pending) < output_length) {
            break;
        }
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
}

static bool next_sound_consume_dma(NextSoundState *s, size_t frames,
                                   bool underrun_if_not_ready)
{
    uint8_t input[NEXT_SOUND_INPUT_BYTES];
    bool produced = false;

    while (frames) {
        size_t input_frames =
            MIN(frames, sizeof(input) / NEXT_SOUND_FRAME_BYTES);
        size_t input_capacity = input_frames * NEXT_SOUND_FRAME_BYTES;
        size_t input_length;
        NextDMAResult result;

        result = next_dma_sound_out_read(s->dma, input, input_capacity,
                                         &input_length);
        if (result != NEXT_DMA_OK) {
            if (result != NEXT_DMA_NOT_READY ||
                (!produced && underrun_if_not_ready)) {
                next_sound_set_overrun(s, true);
            }
            break;
        }
        if (!input_length) {
            break;
        }
        produced = true;
        frames -= input_length / NEXT_SOUND_FRAME_BYTES;
        next_sound_append_dma_frames(s, input, input_length);
    }

    return produced;
}

static void next_sound_flush(NextSoundState *s, size_t available)
{
    while (s->pending_length && available >= NEXT_SOUND_FRAME_BYTES) {
        size_t written = audio_be_write(s->audio_be, s->voice,
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
}

static void next_sound_schedule(NextSoundState *s)
{
    int64_t now;

    if (!next_sound_dma_enabled(s) ||
        next_dma_sound_out_complete(s->dma)) {
        timer_del(&s->dma_timer);
        s->dma_clock_ns = 0;
        s->dma_fraction = 0;
        return;
    }
    if (timer_pending(&s->dma_timer)) {
        return;
    }
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->dma_clock_ns = now;
    s->dma_fraction = 0;
    timer_mod(&s->dma_timer, now + NEXT_SOUND_TIMER_NS);
}

static void next_sound_dma_timer(void *opaque)
{
    NextSoundState *s = opaque;
    uint64_t scaled;
    uint64_t frames;
    uint32_t rate;
    int64_t elapsed;
    int64_t now;

    if (!next_sound_dma_enabled(s) ||
        next_dma_sound_out_complete(s->dma)) {
        return;
    }

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    elapsed = now - s->dma_clock_ns;
    if (elapsed < 0 ||
        (uint64_t)elapsed > (UINT64_MAX - s->dma_fraction) /
                            NEXT_SOUND_SAMPLE_RATE) {
        s->dma_clock_ns = now;
        s->dma_fraction = 0;
        timer_mod(&s->dma_timer, now + NEXT_SOUND_TIMER_NS);
        return;
    }

    rate = (s->control & SOUT_DOUB) ? NEXT_SOUND_SAMPLE_RATE / 2
                                    : NEXT_SOUND_SAMPLE_RATE;
    scaled = (uint64_t)elapsed * rate + s->dma_fraction;
    frames = scaled / INT64_C(1000000000);
    s->dma_fraction = scaled % INT64_C(1000000000);
    s->dma_clock_ns = now;

    if (frames) {
        next_sound_consume_dma(
            s, MIN(frames, (uint64_t)NEXT_SOUND_MAX_CATCHUP_FRAMES), true);
        next_sound_set_active(s);
        next_sound_flush(s,
            audio_be_get_buffer_size_out(s->audio_be, s->voice));
    }
    if (!next_sound_dma_enabled(s) ||
        next_dma_sound_out_complete(s->dma)) {
        return;
    }
    timer_mod(&s->dma_timer, now + NEXT_SOUND_TIMER_NS);
}

static void next_sound_kick(NextSoundState *s)
{
    next_sound_set_active(s);
    next_sound_flush(s,
        audio_be_get_buffer_size_out(s->audio_be, s->voice));
    next_sound_schedule(s);
}

static void next_sound_out_cb(void *opaque, int available)
{
    NextSoundState *s = opaque;

    if (available > 0) {
        next_sound_flush(s, available);
    }
}

static void next_sound_dma_state_changed(void *opaque)
{
    NextSoundState *s = opaque;

    next_sound_schedule(s);
}

static const NextDMASoundOutNotify next_sound_dma_notify = {
    .state_changed = next_sound_dma_state_changed,
};

static void next_sound_queue_direct_frame(NextSoundState *s, uint32_t data)
{
    uint8_t frame[NEXT_SOUND_FRAME_BYTES];
    uint8_t *output;
    size_t needed = (s->control & SOUT_DOUB) ?
                    2 * NEXT_SOUND_FRAME_BYTES : NEXT_SOUND_FRAME_BYTES;

    next_sound_compact_pending(s);
    if (sizeof(s->pending) - s->pending_length < needed) {
        return;
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
    next_sound_kick(s);
}

void next_sound_monitor_command(NextSoundState *s, uint8_t command,
                                uint32_t data)
{
    if (command == MON_SOUND_OUT) {
        next_sound_queue_direct_frame(s, data);
        if (next_sound_dma_enabled(s) &&
            !next_dma_sound_out_complete(s->dma)) {
            next_sound_consume_dma(s, 1, false);
        }
        next_sound_kick(s);
        return;
    }
    if (command == MON_GP_OUT) {
        s->gp_flags = data >> 24;
        next_sound_update_volume(s);
        return;
    }
    if ((command & MON_SNDOUT_CTRL_MASK) == MON_SNDOUT_CTRL_MASK) {
        if (s->control !=
            ((command >> 3) & (SOUT_ENAB | SOUT_DOUB | SOUT_ZERO))) {
            timer_del(&s->dma_timer);
            s->dma_clock_ns = 0;
            s->dma_fraction = 0;
        }
        s->control = (command >> 3) & (SOUT_ENAB | SOUT_DOUB | SOUT_ZERO);
        if (!(s->control & SOUT_ENAB)) {
            s->pending_offset = 0;
            s->pending_length = 0;
        }
        next_sound_kick(s);
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
    if (version_id >= 2 && s->dma_fraction >= INT64_C(1000000000)) {
        return -EINVAL;
    }
    if (version_id < 2) {
        s->dma_clock_ns = 0;
        s->dma_fraction = 0;
    }
    qemu_set_irq(s->overrun_irq, s->overrun);
    next_sound_update_volume(s);
    next_sound_set_active(s);
    if (next_sound_dma_enabled(s) &&
        !next_dma_sound_out_complete(s->dma)) {
        if (!timer_pending(&s->dma_timer)) {
            next_sound_schedule(s);
        }
    } else {
        timer_del(&s->dma_timer);
    }
    return 0;
}

static const VMStateDescription vmstate_next_sound = {
    .name = TYPE_NEXT_SOUND,
    .version_id = 2,
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
        VMSTATE_TIMER_V(dma_timer, NextSoundState, 2),
        VMSTATE_INT64_V(dma_clock_ns, NextSoundState, 2),
        VMSTATE_UINT64_V(dma_fraction, NextSoundState, 2),
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
    s->dma_clock_ns = 0;
    s->dma_fraction = 0;
    timer_del(&s->dma_timer);
    next_sound_set_overrun(s, false);
    next_sound_update_volume(s);
    next_sound_set_active(s);
}

static void next_sound_realize(DeviceState *dev, Error **errp)
{
    NextSoundState *s = NEXT_SOUND(dev);
    struct audsettings settings = {
        .freq = NEXT_SOUND_SAMPLE_RATE,
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

    /*
     * The monitor sound device is linked to the keyboard controller rather
     * than attached to a bus, so register it explicitly for system reset.
     */
    qemu_register_resettable(OBJECT(s));
    next_dma_set_sound_out_notify(s->dma, &next_sound_dma_notify, s);
    next_sound_update_volume(s);
    next_sound_set_active(s);
}

static void next_sound_unrealize(DeviceState *dev)
{
    NextSoundState *s = NEXT_SOUND(dev);

    next_dma_set_sound_out_notify(s->dma, NULL, NULL);
    qemu_unregister_resettable(OBJECT(s));
    timer_del(&s->dma_timer);
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

    timer_init_ns(&s->dma_timer, QEMU_CLOCK_VIRTUAL,
                  next_sound_dma_timer, s);
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
