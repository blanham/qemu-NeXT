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

#ifndef HW_DISPLAY_BT463_H
#define HW_DISPLAY_BT463_H

#include <stdbool.h>
#include <stdint.h>

#include "migration/vmstate.h"

#define BT463_ADDRESS_MASK       0x0fff
#define BT463_PALETTE_ENTRIES    0x210
#define BT463_CURSOR_COLORS      2
#define BT463_WTT_ENTRIES       16
#define BT463_LEGACY_ENTRIES    0x400
#define BT463_PIXEL_PIN_MASK    0x0fffffffU

/* Window-type display mode encodings (WTT bits B11:B9). */
typedef enum Bt463DisplayMode {
    BT463_WTT_TRUE_COLOR = 0,
    BT463_WTT_PSEUDO_COLOR = 1,
    BT463_WTT_BANK_SELECT = 2,
    BT463_WTT_RESERVED_3 = 3,
    BT463_TRUE_COLOR_LOAD_INTERLEAVE = 4,
    BT463_PSEUDO_COLOR_LOAD_INTERLEAVE = 5,
    BT463_WTT_RESERVED_6 = 6,
    BT463_WTT_RESERVED_7 = 7,
} Bt463DisplayMode;

/* Nibble selected by a load-interleave input cycle. */
typedef enum Bt463LoadPhase {
    BT463_LOAD_LOWER = 0,
    BT463_LOAD_UPPER = 1,
} Bt463LoadPhase;

/*
 * State emitted by the NeXT color-video device before the Bt463 model was
 * extracted.  This remains solely as an incoming migration format.
 */
typedef struct Bt463LegacyState {
    uint16_t dac_address;
    uint8_t dac_component;
    uint8_t palette[BT463_LEGACY_ENTRIES][3];
    uint8_t general[BT463_LEGACY_ENTRIES][3];
} Bt463LegacyState;

typedef struct Bt463State {
    uint16_t address;
    uint8_t component;

    uint8_t palette[BT463_PALETTE_ENTRIES][3];
    uint8_t cursor[BT463_CURSOR_COLORS][3];
    uint8_t command[3];
    uint8_t read_mask[4];
    uint8_t blink_mask[4];
    uint8_t test_register;
    uint16_t input_signature;
    uint8_t output_signature[3];
    uint32_t wtt[BT463_WTT_ENTRIES];

    /* The WTT shifts a complete 24-bit word into the table on B16-B23. */
    uint32_t wtt_write_latch;
    uint32_t wtt_read_latch;
} Bt463State;

extern const VMStateDescription vmstate_bt463;
extern const VMStateDescription vmstate_bt463_legacy;

void bt463_init(Bt463State *s);
void bt463_reset(Bt463State *s);
void bt463_import_legacy(Bt463State *s, const Bt463LegacyState *legacy);

uint8_t bt463_address_read(Bt463State *s, bool high);
void bt463_address_write(Bt463State *s, bool high, uint8_t value);

uint8_t bt463_palette_read(Bt463State *s);
bool bt463_palette_write(Bt463State *s, uint8_t value);
uint8_t bt463_general_read(Bt463State *s);
bool bt463_general_write(Bt463State *s, uint8_t value);

/*
 * Convert one 28-bit Bt463 pixel-port word to 0xRRGGBB.  pixel_pins uses
 * bit N for physical input pin PN; window_type selects WT0-WT3 and phase is
 * used by the two load-interleave display modes.  Invalid/reserved WTT
 * configurations, and palette addresses outside the 528-entry RAM, return
 * black.
 */
uint32_t bt463_lookup_rgb(const Bt463State *s, uint32_t pixel_pins,
                          uint8_t window_type, Bt463LoadPhase phase);

#endif /* HW_DISPLAY_BT463_H */
