# Plan 9 First Edition authentication implementation plan

> Execute test-first.  Keep the archived Plan 9 tree immutable.

1. Extend the 9P codec and its literal-frame tests for fixed-size First
   Edition `Tauth` and `Rauth` messages.
2. Add a failing server test for an unauthenticated First Edition `none`
   attach, then implement the explicitly historical exception.
3. Add failing tests for the First Edition encrypted challenge/ticket flow:
   success, bad client key, mismatched ticket challenge, and ticket replay.
4. Implement per-server legacy-auth state and handlers using the existing
   historical Plan 9 DES/key database primitives; reject invalid tickets with
   the existing generic auth error and clear state on reset/close.
5. Rebuild `qemu-system-m68k`, run the complete unit suite, and visibly boot
   the untouched First Edition NeXT kernel with a packet trace.
6. Record the verified launch command and then proceed to the preserved
   Second Edition release.
