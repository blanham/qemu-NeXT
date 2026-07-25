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

#ifndef HW_NVRAM_NEXT_NVRAM_H
#define HW_NVRAM_NEXT_NVRAM_H

#include "qapi/error.h"

#define NEXT_NVRAM_SIZE 32

enum {
    NEXT_NVRAM_SETTINGS = 0x00,
    NEXT_NVRAM_EP = 0x04,
    NEXT_NVRAM_SIMM = 0x0a,
    NEXT_NVRAM_ADOBE = 0x0c,
    NEXT_NVRAM_POT = 0x0e,
    NEXT_NVRAM_CLOCK_CONFIG = 0x11,
    NEXT_NVRAM_BOOT_COMMAND = 0x12,
    NEXT_NVRAM_CHECKSUM = 0x1e,
};

typedef struct NextNVRAMSettings {
    uint8_t reset;
    bool alt_console;
    bool allow_eject;
    uint8_t volume_right;
    uint8_t brightness;
    uint8_t hw_password;
    uint8_t volume_left;
    bool speaker;
    bool lowpass;
    bool boot_any;
    bool any_command;
} NextNVRAMSettings;

typedef struct NextNVRAMClockConfig {
    bool new_clock_chip;
    bool auto_poweron;
    bool use_console_slot;
    uint8_t console_slot;
} NextNVRAMClockConfig;

typedef struct NextNVRAMState {
    uint8_t data[NEXT_NVRAM_SIZE];
    char *filename;
    int fd;
    bool dirty;
    bool write_error_reported;
} NextNVRAMState;

void next_nvram_init(NextNVRAMState *s);

uint8_t next_nvram_read(const NextNVRAMState *s, unsigned address);
void next_nvram_write(NextNVRAMState *s, unsigned address, uint8_t value);

void next_nvram_decode_settings(const NextNVRAMState *s,
                                NextNVRAMSettings *settings);
void next_nvram_encode_settings(NextNVRAMState *s,
                                const NextNVRAMSettings *settings);
void next_nvram_decode_clock_config(const NextNVRAMState *s,
                                    NextNVRAMClockConfig *config);
void next_nvram_encode_clock_config(NextNVRAMState *s,
                                    const NextNVRAMClockConfig *config);

uint16_t next_nvram_get_simm(const NextNVRAMState *s);
void next_nvram_set_simm(NextNVRAMState *s, uint16_t simm);

uint16_t next_nvram_get_stored_checksum(const NextNVRAMState *s);
uint16_t next_nvram_compute_checksum(const NextNVRAMState *s);
bool next_nvram_checksum_is_valid(const NextNVRAMState *s);
void next_nvram_update_checksum(NextNVRAMState *s);

bool next_nvram_realize(NextNVRAMState *s, Error **errp);
bool next_nvram_flush(NextNVRAMState *s, Error **errp);
void next_nvram_unrealize(NextNVRAMState *s);

#endif /* HW_NVRAM_NEXT_NVRAM_H */
