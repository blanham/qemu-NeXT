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
#include "libqtest.h"

#define NEXT_MB8795_BASE 0x02106000
#define NEXT_MEMCTL_BASE 0x02106010
#define NEXT_MEMCTL_SIZE 5
#define NEXT_ROM_SIZE    (128 * 1024)

typedef struct TestROM {
    int fd;
    char *path;
} TestROM;

static void cleanup_test_rom(void *opaque)
{
    TestROM *rom = opaque;

    qtest_remove_abrt_handler(rom);
    if (rom->fd >= 0) {
        close(rom->fd);
    }
    if (rom->path) {
        g_unlink(rom->path);
        g_free(rom->path);
    }
    g_free(rom);
}

static QTestState *next_memctl_start(void)
{
    TestROM *rom = g_new0(TestROM, 1);
    g_autofree char *quoted_rom_path = NULL;

    rom->fd = -1;
    qtest_add_abrt_handler(cleanup_test_rom, rom);
    g_test_queue_destroy(cleanup_test_rom, rom);

    rom->fd = g_file_open_tmp("next-memctl-rom-XXXXXX", &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;

    quoted_rom_path = g_shell_quote(rom->path);
    return qtest_initf("-machine next-cube -bios %s", quoted_rom_path);
}

static void test_memory_map(void)
{
    static const char mb8795_line[] =
        "0000000002106000-000000000210600f (prio 0, i/o): next.mb8795";
    static const char memctl_line[] =
        "0000000002106010-0000000002106014 (prio 0, i/o): next.memctl";
    QTestState *qts = next_memctl_start();
    g_autofree char *flatview = qtest_hmp(qts, "info mtree -f");

    g_assert_nonnull(strstr(flatview, mb8795_line));
    g_assert_nonnull(strstr(flatview, memctl_line));
    g_assert_null(strstr(flatview,
        "0000000002106000-0000000002106014 (prio 0, i/o)"));
    g_assert_null(strstr(flatview,
        "0000000002106000-0000000002106fff (prio 0, i/o)"));

    qtest_quit(qts);
}

static void test_rom_programming_and_reset(void)
{
    static const uint8_t timing[NEXT_MEMCTL_SIZE] = {
        0x00, 0xc0, 0x10, 0xc0, 0x10,
    };
    QTestState *qts = next_memctl_start();
    size_t i;

    for (i = 0; i < ARRAY_SIZE(timing); i++) {
        qtest_writeb(qts, NEXT_MEMCTL_BASE + i, timing[i]);
    }
    for (i = 0; i < ARRAY_SIZE(timing); i++) {
        g_assert_cmphex(qtest_readb(qts, NEXT_MEMCTL_BASE + i), ==,
                        timing[i]);
    }

    qtest_system_reset(qts);
    for (i = 0; i < ARRAY_SIZE(timing); i++) {
        g_assert_cmphex(qtest_readb(qts, NEXT_MEMCTL_BASE + i), ==, 0);
    }

    qtest_quit(qts);
}

static void test_access_widths(void)
{
    static const uint8_t timing[NEXT_MEMCTL_SIZE] = {
        0x11, 0x22, 0x33, 0x44, 0x55,
    };
    QTestState *qts = next_memctl_start();
    size_t i;

    for (i = 0; i < ARRAY_SIZE(timing); i++) {
        qtest_writeb(qts, NEXT_MEMCTL_BASE + i, timing[i]);
    }

    g_assert_cmphex(qtest_readw(qts, NEXT_MEMCTL_BASE), ==, 0);
    qtest_writew(qts, NEXT_MEMCTL_BASE, 0xaabb);
    g_assert_cmphex(qtest_readl(qts, NEXT_MEMCTL_BASE), ==, 0);
    qtest_writel(qts, NEXT_MEMCTL_BASE, 0xaabbccdd);

    for (i = 0; i < ARRAY_SIZE(timing); i++) {
        g_assert_cmphex(qtest_readb(qts, NEXT_MEMCTL_BASE + i), ==,
                        timing[i]);
    }

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/next-memctl/memory-map", test_memory_map);
    qtest_add_func("/next-memctl/rom-programming-reset",
                   test_rom_programming_and_reset);
    qtest_add_func("/next-memctl/access-widths", test_access_widths);

    return g_test_run();
}
