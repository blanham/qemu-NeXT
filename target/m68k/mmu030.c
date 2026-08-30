/*
 * MC68030 PMMU architectural state.
 *
 * Copyright (c) 2026 Bryce Lanham
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "target/m68k/mmu030.h"

void m68k_mmu030_reset(M68KMMU030State *state)
{
    memset(state, 0, sizeof(*state));
}
