#!/usr/bin/env python3
#
# Exact icount period option tests
#
# Copyright 2026 Bryce Lanham
#
# This work is licensed under the terms of the GNU GPL, version 2 or later.
# See the COPYING file in the top-level directory.

from qemu_test import QemuSystemTest


class IcountPeriod(QemuSystemTest):

    def _launch_valid(self, period):
        self.vm.set_machine('none')
        self.vm.add_args('-S', '-nodefaults',
                         '-icount', f'ns-per-insn={period}')
        self.vm.launch()
        self.assertEqual(self.vm.cmd('query-status')['status'], 'prelaunch')

    def _launch_invalid(self, option, message):
        self.vm.add_args('-S', '-display', 'none', '-machine', 'none',
                         '-nodefaults', '-icount', option)
        self.vm.set_qmp_monitor(enabled=False)
        self.vm.launch()
        self.vm.wait()
        self.assertEqual(self.vm.exitcode(), 1)
        self.assertRegex(self.vm.get_log(), message)

    def test_exact_period(self):
        self._launch_valid(143)

    def test_maximum_period(self):
        self._launch_valid(1024)

    def test_zero_period(self):
        self._launch_invalid('ns-per-insn=0',
                             r'Invalid ns-per-insn value')

    def test_period_too_large(self):
        self._launch_invalid('ns-per-insn=1025',
                             r'Invalid ns-per-insn value')

    def test_period_conflicts_with_shift(self):
        self._launch_invalid('shift=7,ns-per-insn=143',
                             r'shift and ns-per-insn are mutually exclusive')


if __name__ == '__main__':
    QemuSystemTest.main()
