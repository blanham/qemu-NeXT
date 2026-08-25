# Boot GIF Pacing Design

## Goal

Retiming the five public NeXT guest boot recordings must make visible progress
begin promptly, keep important interactions readable, and shorten the final
pause without losing the decorated QEMU frame or the demonstrated endpoint.

## Assets

The affected recordings are:

- NeXTSTEP 3.1 boot to Workspace Manager;
- NetBSD 1.5 full ROM/TFTP/NFS boot;
- NetBSD 1.5 network-loader/NFS excerpt;
- Plan 9 Second Edition TCP boot to 8½; and
- Plan 9 Second Edition authenticated IL boot to 8½.

The lossless MKV recordings in the local capture directory remain the source
of truth. The existing GIFs are not used as re-encoding inputs.

## Pacing

Each recording uses ordered, manually reviewed source intervals:

- visually static startup and hardware-delay spans run at 4×;
- routine boot output runs at 2×;
- prompts, typed input, login/authentication, shell arrival, and desktop
  transitions remain at 1×; and
- the demonstrated final shell or desktop is held for three seconds.

Intervals join on 15 fps frame boundaries. The retimed stream is continuous:
there are no skipped required prompts, reverse jumps, inserted title cards, or
changes to the captured pixels. A short transition may remain at the slower
neighboring rate when that avoids cutting through an interaction.

## Encoding and validation

The lab GIF encoder continues to generate a two-pass palette, 15 fps output,
and infinite-loop metadata. Its minimum accepted final hold changes from five
seconds to three seconds, including its CLI verifier, tests, and README text.
All other validation remains unchanged: exact dimensions, stable inputs,
positive integer frame delays matching 15 fps, an identical final tail, and a
20 MiB maximum.

Each candidate is checked for:

1. visible motion or changing boot output near the beginning;
2. readable prompts and authentication/login interactions;
3. the same successful shell or desktop endpoint;
4. exactly one decorated QEMU window with no surrounding X root pixels;
5. a final identical hold of at least three seconds; and
6. successful standalone verification.

## Publication

The verified GIFs replace the same five paths in the lab repository and in
QEMU's `docs/boot/` directory. The lab change is merged locally because that
repository has no remote. QEMU's `metachicken` branch receives the replacement
assets and is pushed to `github` so the existing `QEMU-NeXT-README.md` image
URLs update without documentation churn.
