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
#include "hw/nvram/next-nvram.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"

void next_nvram_init(NextNVRAMState *s)
{
    const NextNVRAMSettings settings = {
        .reset = 9,
        .allow_eject = true,
        .brightness = 61,
        .boot_any = true,
        .any_command = true,
    };

    memset(s, 0, sizeof(*s));
    s->fd = -1;

    next_nvram_encode_settings(s, &settings);
    next_nvram_set_simm(s, 0xfb6d);
    s->data[NEXT_NVRAM_POT] = 0x4b;
    s->data[NEXT_NVRAM_POT + 1] = 0x00;
    s->data[NEXT_NVRAM_POT + 2] = 0x41;
    s->data[NEXT_NVRAM_BOOT_COMMAND] = ' ';
    next_nvram_update_checksum(s);
}

uint8_t next_nvram_read(const NextNVRAMState *s, unsigned address)
{
    if (address >= NEXT_NVRAM_SIZE) {
        return 0;
    }

    return s->data[address];
}

void next_nvram_write(NextNVRAMState *s, unsigned address, uint8_t value)
{
    if (address >= NEXT_NVRAM_SIZE) {
        return;
    }

    s->data[address] = value;
}

void next_nvram_decode_settings(const NextNVRAMState *s,
                                NextNVRAMSettings *settings)
{
    uint32_t word = ldl_be_p(&s->data[NEXT_NVRAM_SETTINGS]);

    settings->reset = extract32(word, 28, 4);
    settings->alt_console = extract32(word, 27, 1);
    settings->allow_eject = extract32(word, 26, 1);
    settings->volume_right = extract32(word, 20, 6);
    settings->brightness = extract32(word, 14, 6);
    settings->hw_password = extract32(word, 10, 4);
    settings->volume_left = extract32(word, 4, 6);
    settings->speaker = extract32(word, 3, 1);
    settings->lowpass = extract32(word, 2, 1);
    settings->boot_any = extract32(word, 1, 1);
    settings->any_command = extract32(word, 0, 1);
}

void next_nvram_encode_settings(NextNVRAMState *s,
                                const NextNVRAMSettings *settings)
{
    uint32_t word = 0;

    word = deposit32(word, 28, 4, settings->reset);
    word = deposit32(word, 27, 1, settings->alt_console);
    word = deposit32(word, 26, 1, settings->allow_eject);
    word = deposit32(word, 20, 6, settings->volume_right);
    word = deposit32(word, 14, 6, settings->brightness);
    word = deposit32(word, 10, 4, settings->hw_password);
    word = deposit32(word, 4, 6, settings->volume_left);
    word = deposit32(word, 3, 1, settings->speaker);
    word = deposit32(word, 2, 1, settings->lowpass);
    word = deposit32(word, 1, 1, settings->boot_any);
    word = deposit32(word, 0, 1, settings->any_command);
    stl_be_p(&s->data[NEXT_NVRAM_SETTINGS], word);
}

void next_nvram_decode_clock_config(const NextNVRAMState *s,
                                    NextNVRAMClockConfig *config)
{
    uint8_t value = s->data[NEXT_NVRAM_CLOCK_CONFIG];

    config->new_clock_chip = extract32(value, 7, 1);
    config->auto_poweron = extract32(value, 6, 1);
    config->use_console_slot = extract32(value, 5, 1);
    config->console_slot = extract32(value, 3, 2);
}

void next_nvram_encode_clock_config(NextNVRAMState *s,
                                    const NextNVRAMClockConfig *config)
{
    uint32_t value = s->data[NEXT_NVRAM_CLOCK_CONFIG] & 0x07;

    value = deposit32(value, 7, 1, config->new_clock_chip);
    value = deposit32(value, 6, 1, config->auto_poweron);
    value = deposit32(value, 5, 1, config->use_console_slot);
    value = deposit32(value, 3, 2, config->console_slot);
    s->data[NEXT_NVRAM_CLOCK_CONFIG] = value;
}

uint16_t next_nvram_get_simm(const NextNVRAMState *s)
{
    return lduw_be_p(&s->data[NEXT_NVRAM_SIMM]);
}

void next_nvram_set_simm(NextNVRAMState *s, uint16_t simm)
{
    stw_be_p(&s->data[NEXT_NVRAM_SIMM], simm);
}

uint16_t next_nvram_get_stored_checksum(const NextNVRAMState *s)
{
    return lduw_be_p(&s->data[NEXT_NVRAM_CHECKSUM]);
}

uint16_t next_nvram_compute_checksum(const NextNVRAMState *s)
{
    uint32_t sum = 0;
    unsigned offset;

    for (offset = 0; offset < NEXT_NVRAM_SIZE; offset += 2) {
        uint16_t word = offset == NEXT_NVRAM_CHECKSUM ?
                        0 : lduw_be_p(&s->data[offset]);

        sum += word;
        sum = (sum & 0xffff) + (sum >> 16);
    }

    return ~sum & 0xffff;
}

bool next_nvram_checksum_is_valid(const NextNVRAMState *s)
{
    uint16_t computed = next_nvram_compute_checksum(s);

    return computed != 0 && computed == next_nvram_get_stored_checksum(s);
}

void next_nvram_update_checksum(NextNVRAMState *s)
{
    stw_be_p(&s->data[NEXT_NVRAM_CHECKSUM],
             next_nvram_compute_checksum(s));
}
