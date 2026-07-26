/* SPDX-License-Identifier: NCSA
 *
 * QEMU NeXT sound DMA clock helpers
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

#ifndef HW_AUDIO_NEXT_SOUND_CLOCK_H
#define HW_AUDIO_NEXT_SOUND_CLOCK_H

#include "qemu/timer.h"

static inline bool next_sound_clock_elapsed_frames(
    int64_t now, int64_t clock_ns, uint64_t fraction, uint32_t rate,
    uint64_t *frames, uint64_t *next_fraction)
{
    uint64_t elapsed;
    uint64_t scaled;

    if (now < 0 || clock_ns < 0 || now < clock_ns || rate == 0 ||
        fraction >= NANOSECONDS_PER_SECOND) {
        return false;
    }

    elapsed = (uint64_t)now - (uint64_t)clock_ns;
    if (elapsed > (UINT64_MAX - fraction) / rate) {
        return false;
    }

    scaled = elapsed * rate + fraction;
    *frames = scaled / NANOSECONDS_PER_SECOND;
    *next_fraction = scaled % NANOSECONDS_PER_SECOND;
    return true;
}

#endif /* HW_AUDIO_NEXT_SOUND_CLOCK_H */
