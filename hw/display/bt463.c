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
#include "hw/display/bt463.h"

enum Bt463GeneralAccess {
    BT463_GENERAL_INVALID,
    BT463_GENERAL_BYTE,
    BT463_GENERAL_CURSOR,
    BT463_GENERAL_INPUT_SIGNATURE,
    BT463_GENERAL_OUTPUT_SIGNATURE,
    BT463_GENERAL_WTT,
};

static void bt463_advance_address(Bt463State *s)
{
    s->address = (s->address + 1) & BT463_ADDRESS_MASK;
}

static void bt463_advance_component(Bt463State *s, bool triplet)
{
    if (!triplet) {
        s->component = 0;
        bt463_advance_address(s);
        return;
    }

    s->component++;
    if (s->component == 3) {
        s->component = 0;
        bt463_advance_address(s);
    }
}

static enum Bt463GeneralAccess bt463_general_access(uint16_t address)
{
    if (address == 0x100 || address == 0x101) {
        return BT463_GENERAL_CURSOR;
    }
    if (address == 0x20e) {
        return BT463_GENERAL_INPUT_SIGNATURE;
    }
    if (address == 0x20f) {
        return BT463_GENERAL_OUTPUT_SIGNATURE;
    }
    if (address >= 0x300 && address <= 0x30f) {
        return BT463_GENERAL_WTT;
    }

    switch (address) {
    case 0x200:
    case 0x201 ... 0x203:
    case 0x205 ... 0x20d:
    case 0x220:
        return BT463_GENERAL_BYTE;
    default:
        return BT463_GENERAL_INVALID;
    }
}

static uint8_t bt463_command_mask(uint16_t address)
{
    switch (address) {
    case 0x201:
        /* CR7-CR6 and CR3-CR2 are defined; the remaining bits are reserved. */
        return 0xcc;
    case 0x202:
        /* CR16-CR10 are defined; CR17 is reserved. */
        return 0x7f;
    case 0x203:
        /* CR27-CR26 and CR22-CR20 are defined. */
        return 0xc7;
    default:
        return 0;
    }
}

void bt463_init(Bt463State *s)
{
    bt463_reset(s);
}

void bt463_reset(Bt463State *s)
{
    memset(s, 0, sizeof(*s));
}

void bt463_import_legacy(Bt463State *s, const Bt463LegacyState *legacy)
{
    bt463_reset(s);
    s->address = legacy->dac_address & BT463_ADDRESS_MASK;
    s->component = legacy->dac_component % 3;

    /* The old generic palette was larger than the architected Bt463 RAM. */
    memcpy(s->palette, legacy->palette, sizeof(s->palette));
    memcpy(s->cursor[0], legacy->general[0x100], sizeof(s->cursor[0]));
    memcpy(s->cursor[1], legacy->general[0x101], sizeof(s->cursor[1]));

    for (unsigned i = 0; i < G_N_ELEMENTS(s->command); i++) {
        s->command[i] = legacy->general[0x201 + i][0] &
            bt463_command_mask(0x201 + i);
    }
    for (unsigned i = 0; i < G_N_ELEMENTS(s->read_mask); i++) {
        s->read_mask[i] = legacy->general[0x205 + i][0];
        s->blink_mask[i] = legacy->general[0x209 + i][0];
    }
    s->test_register = legacy->general[0x20d][0];
    s->input_signature = legacy->general[0x20e][0] |
        ((uint16_t)legacy->general[0x20e][1] << 8);
    memcpy(s->output_signature, legacy->general[0x20f],
           sizeof(s->output_signature));

    for (unsigned i = 0; i < BT463_WTT_ENTRIES; i++) {
        const uint8_t *entry = legacy->general[0x300 + i];

        s->wtt[i] = entry[0] | ((uint32_t)entry[1] << 8) |
            ((uint32_t)entry[2] << 16);
    }

    /* Resume a partially completed WTT triplet from the imported entry. */
    if (s->address >= 0x300 &&
        s->address < 0x300 + BT463_WTT_ENTRIES && s->component != 0) {
        uint32_t wtt = s->wtt[s->address - 0x300];

        s->wtt_write_latch = wtt;
        s->wtt_read_latch = wtt;
    }
}

uint8_t bt463_address_read(Bt463State *s, bool high)
{
    s->component = 0;
    if (high) {
        return (s->address >> 8) & 0x0f;
    }
    return s->address & 0xff;
}

void bt463_address_write(Bt463State *s, bool high, uint8_t value)
{
    if (high) {
        s->address = (s->address & 0x00ff) | ((value & 0x0f) << 8);
    } else {
        s->address = (s->address & 0x0f00) | value;
    }
    s->component = 0;
}

uint8_t bt463_palette_read(Bt463State *s)
{
    uint8_t value = 0;

    if (s->address < BT463_PALETTE_ENTRIES) {
        value = s->palette[s->address][s->component];
    }
    bt463_advance_component(s, true);

    return value;
}

bool bt463_palette_write(Bt463State *s, uint8_t value)
{
    const bool valid = s->address < BT463_PALETTE_ENTRIES;

    if (valid) {
        s->palette[s->address][s->component] = value;
    }
    bt463_advance_component(s, true);

    return valid;
}

uint8_t bt463_general_read(Bt463State *s)
{
    const enum Bt463GeneralAccess access = bt463_general_access(s->address);
    uint8_t value = 0;

    switch (access) {
    case BT463_GENERAL_CURSOR:
        value = s->cursor[s->address - 0x100][s->component];
        bt463_advance_component(s, true);
        break;
    case BT463_GENERAL_INPUT_SIGNATURE:
        if (s->component == 0) {
            value = s->input_signature & 0xff;
        } else if (s->component == 1) {
            value = s->input_signature >> 8;
        }
        bt463_advance_component(s, true);
        break;
    case BT463_GENERAL_OUTPUT_SIGNATURE:
        value = s->output_signature[s->component];
        bt463_advance_component(s, true);
        break;
    case BT463_GENERAL_WTT:
        if (s->component == 0) {
            s->wtt_read_latch = s->wtt[s->address - 0x300] & 0xffffff;
        }
        value = (s->wtt_read_latch >> (s->component * 8)) & 0xff;
        bt463_advance_component(s, true);
        break;
    case BT463_GENERAL_BYTE:
        switch (s->address) {
        case 0x200:
            value = 0x2a;
            break;
        case 0x201 ... 0x203:
            value = s->command[s->address - 0x201];
            break;
        case 0x205 ... 0x208:
            value = s->read_mask[s->address - 0x205];
            break;
        case 0x209 ... 0x20c:
            value = s->blink_mask[s->address - 0x209];
            break;
        case 0x20d:
            value = s->test_register;
            break;
        case 0x220:
            value = 0xb0;
            break;
        default:
            g_assert_not_reached();
        }
        bt463_advance_component(s, false);
        break;
    case BT463_GENERAL_INVALID:
        bt463_advance_component(s, false);
        break;
    }

    return value;
}

bool bt463_general_write(Bt463State *s, uint8_t value)
{
    const enum Bt463GeneralAccess access = bt463_general_access(s->address);
    bool changed = false;

    switch (access) {
    case BT463_GENERAL_CURSOR:
        s->cursor[s->address - 0x100][s->component] = value;
        changed = true;
        bt463_advance_component(s, true);
        break;
    case BT463_GENERAL_INPUT_SIGNATURE:
        if (s->component == 0) {
            s->input_signature = (s->input_signature & 0xff00) | value;
        } else if (s->component == 1) {
            s->input_signature = (s->input_signature & 0x00ff) |
                ((uint16_t)value << 8);
        }
        bt463_advance_component(s, true);
        break;
    case BT463_GENERAL_OUTPUT_SIGNATURE:
        s->output_signature[s->component] = value;
        bt463_advance_component(s, true);
        break;
    case BT463_GENERAL_WTT:
        if (s->component == 0) {
            s->wtt_write_latch = s->wtt[s->address - 0x300] & 0xffffff;
        }
        s->wtt_write_latch &= ~(0xffU << (s->component * 8));
        s->wtt_write_latch |= (uint32_t)value << (s->component * 8);
        if (s->component == 2) {
            s->wtt[s->address - 0x300] = s->wtt_write_latch & 0xffffff;
            changed = true;
        }
        bt463_advance_component(s, true);
        break;
    case BT463_GENERAL_BYTE:
        switch (s->address) {
        case 0x201 ... 0x203: {
            uint8_t *command = &s->command[s->address - 0x201];
            uint8_t new_value = value & bt463_command_mask(s->address);

            changed = *command != new_value;
            *command = new_value;
            break;
        }
        case 0x205 ... 0x208:
            changed = s->read_mask[s->address - 0x205] != value;
            s->read_mask[s->address - 0x205] = value;
            break;
        case 0x209 ... 0x20c:
            changed = s->blink_mask[s->address - 0x209] != value;
            s->blink_mask[s->address - 0x209] = value;
            break;
        case 0x20d:
            s->test_register = value;
            break;
        case 0x200:
        case 0x220:
            /* ID and revision are read-only. */
            break;
        default:
            g_assert_not_reached();
        }
        bt463_advance_component(s, false);
        break;
    case BT463_GENERAL_INVALID:
        bt463_advance_component(s, false);
        break;
    }

    return changed;
}

static uint32_t bt463_mask_pixel(const Bt463State *s, uint32_t pixel)
{
    uint32_t masked = 0;

    pixel &= BT463_PIXEL_PIN_MASK;
    for (unsigned i = 0; i < 4; i++) {
        uint32_t value = (pixel >> (i * 8)) & 0xff;
        uint32_t mask = s->read_mask[i];

        if (i == 3) {
            mask &= 0x0f;
        }
        masked |= (value & mask) << (i * 8);
    }

    return masked;
}

static uint32_t bt463_extract_bits(uint32_t value, unsigned first,
                                   unsigned count)
{
    if (!count) {
        return 0;
    }
    return (value >> first) & ((1U << count) - 1);
}

static uint32_t bt463_overlay_input(uint32_t masked, uint32_t shifted,
                                    unsigned planes, unsigned mode,
                                    bool overlay_location)
{
    if (!overlay_location) {
        /* P24-P27 are a fixed overlay port and are not shifted. */
        return (masked >> 24) & 0x0f;
    }

    if (mode == BT463_WTT_PSEUDO_COLOR) {
        return (shifted >> planes) & 0x0f;
    }

    /* True-color and bank-select use P17, P0, P8, P16 as OL3..OL0. */
    return (((shifted >> 17) & 1) << 3) |
        (((shifted >> 0) & 1) << 2) |
        (((shifted >> 8) & 1) << 1) |
        (((shifted >> 16) & 1) << 0);
}

static void bt463_compact_overlay(uint32_t input, unsigned mask,
                                  uint32_t *value)
{
    unsigned count = 0;
    uint32_t compact = 0;

    for (unsigned i = 0; i < 4; i++) {
        if (mask & (1U << i)) {
            compact |= ((input >> i) & 1) << count;
            count++;
        }
    }
    *value = compact;
}

static bool bt463_palette_component(const Bt463State *s, unsigned address,
                                    unsigned component, uint8_t *value)
{
    if (address >= BT463_PALETTE_ENTRIES || component >= 3) {
        return false;
    }
    *value = s->palette[address][component];
    return true;
}

static bool bt463_lookup_palette_rgb(const Bt463State *s, unsigned address,
                                     uint8_t rgb[3])
{
    for (unsigned i = 0; i < 3; i++) {
        if (!bt463_palette_component(s, address, i, &rgb[i])) {
            return false;
        }
    }
    return true;
}

static uint32_t bt463_pack_rgb(const uint8_t rgb[3])
{
    return ((uint32_t)rgb[0] << 16) | ((uint32_t)rgb[1] << 8) | rgb[2];
}

uint32_t bt463_lookup_rgb(const Bt463State *s, uint32_t pixel_pins,
                          uint8_t window_type, Bt463LoadPhase phase)
{
    const uint32_t wtt = s->wtt[window_type & 0x0f] & 0xffffff;
    const unsigned shift = wtt & 0x1f;
    const unsigned planes = (wtt >> 5) & 0x0f;
    const unsigned mode = (wtt >> 9) & 0x07;
    const bool overlay_location = (wtt >> 12) & 1;
    const unsigned overlay_mask = (wtt >> 13) & 0x0f;
    const unsigned start = ((wtt >> 17) & 0x3f) << 4;
    const bool bypass = (wtt >> 23) & 1;
    const uint32_t masked = bt463_mask_pixel(s, pixel_pins);
    const uint32_t shifted = shift < 28 ? masked >> shift : 0;
    uint8_t rgb[3];

    if (shift > 27) {
        return 0;
    }

    switch (mode) {
    case BT463_WTT_TRUE_COLOR: {
        if (planes > 8 || shift + planes * 3 > 28) {
            return 0;
        }
        if (bypass) {
            if (planes != 8 || shift + 24 > 28) {
                return 0;
            }
            for (unsigned i = 0; i < 3; i++) {
                rgb[i] = bt463_extract_bits(shifted, i * 8, 8);
            }
            return bt463_pack_rgb(rgb);
        }
        for (unsigned i = 0; i < 3; i++) {
            const unsigned value = bt463_extract_bits(shifted, i * planes,
                                                      planes);
            if (!bt463_palette_component(s, start + value, i, &rgb[i])) {
                return 0;
            }
        }
        return bt463_pack_rgb(rgb);
    }

    case BT463_WTT_PSEUDO_COLOR: {
        if (planes > 9 || shift + planes > 28) {
            return 0;
        }
        if (bypass) {
            if (planes != 8 || shift + 8 > 28) {
                return 0;
            }
            const unsigned value = bt463_extract_bits(shifted, 0, 8);
            rgb[0] = value;
            rgb[1] = value;
            rgb[2] = value;
            return bt463_pack_rgb(rgb);
        }

        const unsigned value = bt463_extract_bits(shifted, 0, planes);
        if (!bt463_lookup_palette_rgb(s, start + value, rgb)) {
            return 0;
        }
        return bt463_pack_rgb(rgb);
    }

    case BT463_WTT_BANK_SELECT: {
        uint32_t overlay_value;
        unsigned value;

        if (bypass || planes > 8 || shift + planes > 28) {
            return 0;
        }
        bt463_compact_overlay(
            bt463_overlay_input(masked, shifted, planes, mode,
                                overlay_location),
            overlay_mask, &overlay_value);
        value = bt463_extract_bits(shifted, 0, planes);
        value |= overlay_value << planes;
        if (!bt463_lookup_palette_rgb(s, start + value, rgb)) {
            return 0;
        }
        return bt463_pack_rgb(rgb);
    }

    case BT463_TRUE_COLOR_LOAD_INTERLEAVE: {
        if (bypass || planes != 4 || shift + 24 > 28) {
            return 0;
        }
        for (unsigned i = 0; i < 3; i++) {
            const unsigned value = bt463_extract_bits(
                shifted, i * 8 + (phase == BT463_LOAD_UPPER ? 4 : 0), 4);
            if (!bt463_palette_component(s, start + value, i, &rgb[i])) {
                return 0;
            }
        }
        return bt463_pack_rgb(rgb);
    }

    case BT463_PSEUDO_COLOR_LOAD_INTERLEAVE: {
        unsigned red;
        unsigned green;
        unsigned value;

        if (bypass || planes != 8 || shift + 16 > 28) {
            return 0;
        }
        red = bt463_extract_bits(shifted,
                                 phase == BT463_LOAD_UPPER ? 4 : 0, 4);
        green = bt463_extract_bits(shifted,
                                   8 + (phase == BT463_LOAD_UPPER ? 4 : 0),
                                   4);
        value = (green << 4) | red;
        if (!bt463_lookup_palette_rgb(s, start + value, rgb)) {
            return 0;
        }
        return bt463_pack_rgb(rgb);
    }

    default:
        return 0;
    }
}

static int bt463_post_load(void *opaque, int version_id)
{
    Bt463State *s = opaque;

    s->address &= BT463_ADDRESS_MASK;
    s->component %= 3;
    s->wtt_write_latch &= 0xffffff;
    s->wtt_read_latch &= 0xffffff;
    for (unsigned i = 0; i < G_N_ELEMENTS(s->command); i++) {
        s->command[i] &= bt463_command_mask(0x201 + i);
    }
    for (unsigned i = 0; i < BT463_WTT_ENTRIES; i++) {
        s->wtt[i] &= 0xffffff;
    }

    return 0;
}

const VMStateDescription vmstate_bt463_legacy = {
    .name = "bt463-legacy",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(dac_address, Bt463LegacyState),
        VMSTATE_UINT8(dac_component, Bt463LegacyState),
        VMSTATE_UINT8_2DARRAY(palette, Bt463LegacyState,
                             BT463_LEGACY_ENTRIES, 3),
        VMSTATE_UINT8_2DARRAY(general, Bt463LegacyState,
                             BT463_LEGACY_ENTRIES, 3),
        VMSTATE_END_OF_LIST()
    }
};

const VMStateDescription vmstate_bt463 = {
    .name = "bt463",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = bt463_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(address, Bt463State),
        VMSTATE_UINT8(component, Bt463State),
        VMSTATE_UINT8_2DARRAY(palette, Bt463State,
                             BT463_PALETTE_ENTRIES, 3),
        VMSTATE_UINT8_2DARRAY(cursor, Bt463State,
                             BT463_CURSOR_COLORS, 3),
        VMSTATE_UINT8_ARRAY(command, Bt463State, 3),
        VMSTATE_UINT8_ARRAY(read_mask, Bt463State, 4),
        VMSTATE_UINT8_ARRAY(blink_mask, Bt463State, 4),
        VMSTATE_UINT8(test_register, Bt463State),
        VMSTATE_UINT16(input_signature, Bt463State),
        VMSTATE_UINT8_ARRAY(output_signature, Bt463State, 3),
        VMSTATE_UINT32_ARRAY(wtt, Bt463State, BT463_WTT_ENTRIES),
        VMSTATE_UINT32(wtt_write_latch, Bt463State),
        VMSTATE_UINT32(wtt_read_latch, Bt463State),
        VMSTATE_END_OF_LIST()
    }
};
