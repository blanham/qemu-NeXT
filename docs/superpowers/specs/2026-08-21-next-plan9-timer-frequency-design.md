# NeXT Plan 9 Timer Frequency Override

## Purpose

The Second Edition Plan 9 NeXT kernel defines `HZ` as 68 but programs the
NeXT system timer with `0xffff`.  At the hardware default of 1 MHz, that
produces about 15.26 interrupts per second.  The mismatch makes scheduler
affinity delays and process startup much longer than the kernel intended.

QEMU must continue to model the real 1 MHz timer by default.  An explicit
override will let the unmodified historical Plan 9 kernel run with its
intended effective clock rate without changing other guests or the NeXT event
counter.

## Interface

Add an unsigned `system-timer-frequency` property to the internal `next-pc`
device.  Its default is 1,000,000 Hz.  Values of zero or greater than
1,000,000,000 Hz are rejected during realization because QEMU's virtual clock
cannot represent a shorter-than-one-nanosecond timer tick.

The Plan 9 launcher will opt in with:

```text
-global next-pc.system-timer-frequency=4456448
```

The value is `65536 * 68`.  It preserves the historical kernel binary while
leaving NeXTSTEP, NetBSD, firmware, and ordinary NeXT machine launches on the
hardware default.

## Timer Model

Only system-timer deadline and remaining-count conversions use the property.
The event counter remains fixed at one microsecond per tick.  Counter, latch,
enable, update, interrupt acknowledgement, reset, and reload behavior remain
unchanged.

Deadline conversion rounds up to the next representable nanosecond so an
interrupt cannot occur before its programmed count has elapsed.  Remaining
time converts back to counter ticks with the same conservative rounding.

## Migration

The frequency is machine configuration, like other QOM properties, rather
than guest-visible mutable state.  It is not added to the migration stream;
source and destination must use the same command-line property.  Existing
timer migration state and post-load rescheduling continue to use the selected
frequency.

## Tests

Extend the NeXT timer qtests to prove:

1. The default 1 MHz deadline is unchanged.
2. The 4,456,448 Hz override fires a `0xffff` timer at the scaled deadline and
   not one nanosecond early.
3. The event counter still advances at 1 MHz while the override is active.
4. Zero and unrepresentable frequencies fail realization.
5. The Plan 9 launch contract includes the explicit override while other
   launch profiles remain unchanged.

Run the focused timer and Plan 9 launcher tests, the broader NeXT qtest set,
and a fresh `qemu-system-m68k` build before integration.
