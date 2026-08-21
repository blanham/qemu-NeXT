# NeXT Monochrome Framebuffer Dirty Refresh Design

## Goal

Stop converting and publishing the complete 1120x832 monochrome display on
every refresh when only part of VRAM changed, or when VRAM did not change at
all.

## Design

`nextfb_update()` will retain the existing full refresh for realize, reset,
and explicit display invalidation.  On ordinary refreshes it will pass the
device's actual invalidation state to `framebuffer_update_display()`, allowing
QEMU's VGA dirty bitmap to select changed rows.  It will call
`qemu_console_update()` only when at least one row was rendered, and will
publish only the inclusive `[first, last]` row range returned by the generic
framebuffer helper.  Invalidation is cleared after the update attempt.

A disabled-by-default trace event will report the invalidation state and the
resulting row range.  The qtest will use that event to verify four externally
triggered refreshes: initial full redraw, clean no-op, partial redraw after a
VRAM write, and full redraw after reset.  The test remains GPL-licensed; the
NeXT device and trace declaration remain NCSA-licensed project code.

## Scope

This change does not alter pixel conversion, VRAM layout, retrace timing,
migration state, color video, or guest-visible hardware behavior.
