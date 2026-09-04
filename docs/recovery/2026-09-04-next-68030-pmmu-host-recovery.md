# MC68030 PMMU Host-Recovery Snapshot

This is an unfinished recovery snapshot.  It is not reviewed, accepted, or
eligible for merge into `metachicken`.

- Captured: 2026-09-04
- Source worktree: `/mnt/build/NeXT/worktrees/next-68030-qemu`
- Base: `6bd0494564ad9ffd5ce60198c3038224244487cf`
- Pre-commit tracked binary diff SHA-256: `e7c2c729cee40a1c41e4a2c8b0f7c7697f0f691135166cf16cc091c858911ce4`
- Tracked paths: 13 modified
- Untracked source/test paths: 2

Verbatim pre-commit porcelain-v2 status:

```text
1 .M N... 100644 100644 100644 7fbcd58690623383ae1322ae2d52b003b60c402c 7fbcd58690623383ae1322ae2d52b003b60c402c hw/m68k/next-cube.c
1 .M N... 100644 100644 100644 c5a641095c6235e03501be9d49c7d0ebd09c71df c5a641095c6235e03501be9d49c7d0ebd09c71df hw/net/next-mb8795.c
1 .M N... 100644 100644 100644 1942c7df0c67253c64105e6a9e91561fb8bd950e 1942c7df0c67253c64105e6a9e91561fb8bd950e target/m68k/cpu.c
1 .M N... 100644 100644 100644 7c7fbaafda23c48be33dfa161211c0b38117ab16 7c7fbaafda23c48be33dfa161211c0b38117ab16 target/m68k/helper.c
1 .M N... 100644 100644 100644 7dd9ccefa28fe3b6c6170bc80bc14ec5a2fc9fb6 7dd9ccefa28fe3b6c6170bc80bc14ec5a2fc9fb6 target/m68k/mmu030-vmstate.c
1 .M N... 100644 100644 100644 2128630ca36a1e184286e40619f9a3ecb5fdd282 2128630ca36a1e184286e40619f9a3ecb5fdd282 target/m68k/mmu030.c
1 .M N... 100644 100644 100644 c9df9b40ad8a5b66b0b05002c345aaeadcdb9766 c9df9b40ad8a5b66b0b05002c345aaeadcdb9766 target/m68k/mmu030.h
1 .M N... 100644 100644 100644 7fa93ba566b69f0b67faa9c93a46d401d3276af2 7fa93ba566b69f0b67faa9c93a46d401d3276af2 target/m68k/op_helper.c
1 .M N... 100644 100644 100644 e653e6b7ad5acccaede5643e5d9e84bd6a966edd e653e6b7ad5acccaede5643e5d9e84bd6a966edd target/m68k/translate.c
1 .M N... 100644 100644 100644 0d7cf852595874b43eb127a64d0623ea1eac87ff 0d7cf852595874b43eb127a64d0623ea1eac87ff tests/qtest/next-mb8795-test.c
1 .M N... 100644 100644 100644 be564f61b2e4156fae866cb86c1676a4a08169d7 be564f61b2e4156fae866cb86c1676a4a08169d7 tests/tcg/m68k/Makefile.softmmu-target
1 .M N... 100644 100644 100644 17e221461012feb53a4bfe5f7ebbdddd19bcd0bf 17e221461012feb53a4bfe5f7ebbdddd19bcd0bf tests/tcg/m68k/system/pmmu-fault-restart.S
1 .M N... 100644 100644 100644 e3368ccaa665654f146bc4b3559546b994ceed6e e3368ccaa665654f146bc4b3559546b994ceed6e tests/unit/test-m68k-mmu030.c
? tests/tcg/m68k/system/pmmu-access-frame-rte.S
? tests/tcg/m68k/system/run-pmmu-access-frame-rte.sh
```

Verbatim tracked diff statistics:

```text
 hw/m68k/next-cube.c                        |   4 +
 hw/net/next-mb8795.c                       |  17 +-
 target/m68k/cpu.c                          |  20 +-
 target/m68k/helper.c                       |   3 +-
 target/m68k/mmu030-vmstate.c               |  93 ++++-
 target/m68k/mmu030.c                       | 397 ++++++++++++++++++--
 target/m68k/mmu030.h                       |  90 +++++
 target/m68k/op_helper.c                    | 117 +++++-
 target/m68k/translate.c                    |  20 -
 tests/qtest/next-mb8795-test.c             |  21 ++
 tests/tcg/m68k/Makefile.softmmu-target     |   9 +
 tests/tcg/m68k/system/pmmu-fault-restart.S |  78 ++++
 tests/unit/test-m68k-mmu030.c              | 568 ++++++++++++++++++++++++++++-
 13 files changed, 1368 insertions(+), 69 deletions(-)
```

Untracked blob identities:

- `tests/tcg/m68k/system/pmmu-access-frame-rte.S`: `7db79db701ad0c2537049d1ad8d8df3b0c42d5fceeb8e6896fcd29886185932d`
- `tests/tcg/m68k/system/run-pmmu-access-frame-rte.sh`: `c7060a284cf1a7388f58ebcac0a417579c4db89b2e10774e843f90989b86e742`

The snapshot contains ongoing PMMU translation, access-frame, instruction
restart, VMState, Ethernet, and regression work.  Behavioral verification and
reconciliation with the separate v41 snapshot are intentionally deferred.
