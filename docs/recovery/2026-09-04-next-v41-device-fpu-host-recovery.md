# MC68030 v41 Device/FPU Host-Recovery Snapshot

This is an unfinished recovery snapshot.  It is not reviewed, accepted, or
eligible for merge into `metachicken`.

- Captured: 2026-09-04
- Source worktree: `/tmp/next-v41-work`
- Base: `c387cbb0b8e8cd9fe036d44b99be4b82b234dc8b`
- Pre-commit tracked binary diff SHA-256: `9ad32b24d7f62b32da692f07aa021a634f9dbc275a690607d588d2de9462d50f`
- Tracked paths: 18 modified
- Untracked source/test paths: 3

Verbatim pre-commit porcelain-v2 status:

```text
1 .M N... 100644 100644 100644 d0d32dcdaff57cb9ad2b04a296b397a373a02fb6 d0d32dcdaff57cb9ad2b04a296b397a373a02fb6 docs/system/target-m68k.rst
1 .M N... 100644 100644 100644 257f77e21e9e5205edf491341e70c8da96d3c649 257f77e21e9e5205edf491341e70c8da96d3c649 hw/char/next-serial.c
1 .M N... 100644 100644 100644 2a971ea37be48ff07e338e6352786c12ae3e1899 2a971ea37be48ff07e338e6352786c12ae3e1899 hw/dma/next-dma.c
1 .M N... 100644 100644 100644 af46fb29cfdec5e5a3bd9dfd9cdb477a878e742d af46fb29cfdec5e5a3bd9dfd9cdb477a878e742d hw/m68k/next-cube.c
1 .M N... 100644 100644 100644 8c3d67bc93cbd76f0f77a128cd18ebb2ee488272 8c3d67bc93cbd76f0f77a128cd18ebb2ee488272 hw/net/next-mb8795.c
1 .M N... 100644 100644 100644 3884833e9cd145979f3fc3bc6e7601841c912d4b 3884833e9cd145979f3fc3bc6e7601841c912d4b target/m68k/cpu.c
1 .M N... 100644 100644 100644 5e587ad5808976cae6f054f552b4041a813e882f 5e587ad5808976cae6f054f552b4041a813e882f target/m68k/cpu.h
1 .M N... 100644 100644 100644 f6ab353fb851da75e8ec35e84624c7430ffcb108 f6ab353fb851da75e8ec35e84624c7430ffcb108 target/m68k/fpu_helper.c
1 .M N... 100644 100644 100644 4d1d749363378546b4e5bbe6bd69c4c5362a189d 4d1d749363378546b4e5bbe6bd69c4c5362a189d target/m68k/helper.c
1 .M N... 100644 100644 100644 65ab096ab4beaf7e8fc64790dad3bcf9a7aa3d49 65ab096ab4beaf7e8fc64790dad3bcf9a7aa3d49 target/m68k/translate.c
1 .M N... 100644 100644 100644 a987fcac779920f2acf697aa1aa1f9a4d2155034 a987fcac779920f2acf697aa1aa1f9a4d2155034 tests/qtest/next-cube-rtc-test.c
1 .M N... 100644 100644 100644 1999a7e7d41021abb2c5768f7c6b1022a5b6be57 1999a7e7d41021abb2c5768f7c6b1022a5b6be57 tests/qtest/next-cube-serial-test.c
1 .M N... 100644 100644 100644 bf9b537d6df5170800e27b2f5617d68e0ad1f216 bf9b537d6df5170800e27b2f5617d68e0ad1f216 tests/qtest/next-dma-test.c
1 .M N... 100644 100644 100644 e80169dd163f0269fa42ceb27ab69640247d86cd e80169dd163f0269fa42ceb27ab69640247d86cd tests/qtest/next-machine-test.c
1 .M N... 100644 100644 100644 0550f09e0e02f1a9e6bf5be7fb868a001b7b7a89 0550f09e0e02f1a9e6bf5be7fb868a001b7b7a89 tests/qtest/next-mb8795-test.c
1 .M N... 100644 100644 100644 2997c6cc88ad4dcf58f5261cffe131ec485a13e8 2997c6cc88ad4dcf58f5261cffe131ec485a13e8 tests/tcg/m68k/Makefile.softmmu-target
1 .M N... 100755 100755 100755 86f227718c1290bb423d35878b28d9adb193a67e 86f227718c1290bb423d35878b28d9adb193a67e tests/tcg/m68k/system/run-pmmu-control.sh
1 .M N... 100644 100644 100644 8c42ee97bc00d89e574df432c22b7ac73238e601 8c42ee97bc00d89e574df432c22b7ac73238e601 tests/unit/test-m68k-mmu030.c
? tests/tcg/m68k/system/fpu-030-state.S
? tests/tcg/m68k/system/movec-caar.S
? tests/tcg/m68k/system/run-fpu-030-state.sh
```

Verbatim tracked diff statistics:

```text
 docs/system/target-m68k.rst               |  36 ++++++--
 hw/char/next-serial.c                     |  37 +++++++-
 hw/dma/next-dma.c                         |  59 ++++++++++++-
 hw/m68k/next-cube.c                       |  33 ++++++--
 hw/net/next-mb8795.c                      |  34 +++++++-
 target/m68k/cpu.c                         |  22 +++++
 target/m68k/cpu.h                         |   1 +
 target/m68k/fpu_helper.c                  | 106 +++++++++++++++++++++--
 target/m68k/helper.c                      |  11 +++
 target/m68k/translate.c                   |   6 +-
 tests/qtest/next-cube-rtc-test.c          |  44 +++++++++-
 tests/qtest/next-cube-serial-test.c       |  45 +++++++++-
 tests/qtest/next-dma-test.c               |  26 ++++++
 tests/qtest/next-machine-test.c           |  68 +++++++++++++++
 tests/qtest/next-mb8795-test.c            | 136 +++++++++++++++++++++++++++---
 tests/tcg/m68k/Makefile.softmmu-target    |   9 ++
 tests/tcg/m68k/system/run-pmmu-control.sh |   9 ++
 tests/unit/test-m68k-mmu030.c             |  18 +++-
 18 files changed, 653 insertions(+), 47 deletions(-)
```

Untracked blob identities:

- `tests/tcg/m68k/system/fpu-030-state.S`: `e65369df46845d2aeb2aaeeca62c5ab180710a0258d3b60c585c2d9acad43f98`
- `tests/tcg/m68k/system/movec-caar.S`: `27179dadecc187ab6ab45bfcc495b5abc19dbfa07ab6f305aa87119afb20cf46`
- `tests/tcg/m68k/system/run-fpu-030-state.sh`: `cb5e89fd3fd3319af72097913a64a7cbbd937decd573ecbec78bd818b46dab27`

The snapshot contains ongoing original-machine device integration, MC68882,
DMA, serial, Ethernet, documentation, and regression work.  Behavioral
verification and reconciliation with the PMMU snapshot are intentionally
deferred.
