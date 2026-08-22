#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later

import subprocess
import sys


FORBIDDEN_SYMBOLS = {
    "nfs2_handle_table_fail_next_mac_for_test",
    "nfs2_handle_table_set_next_generation_for_test",
}


def main() -> int:
    result = subprocess.run(
        [sys.argv[1], "-g", "--defined-only", sys.argv[2]],
        check=True,
        capture_output=True,
        text=True,
    )
    exported = {
        line.split()[-1]
        for line in result.stdout.splitlines()
        if line.split()
    }
    forbidden = FORBIDDEN_SYMBOLS & exported
    if forbidden:
        print("production NFS handle object exports test-only symbols:")
        for symbol in sorted(forbidden):
            print(f"  {symbol}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
