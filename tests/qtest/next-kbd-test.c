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

#define NEXT_INTR_STATUS  0x02007000
#define NEXT_KBD_CSR      0x0200e000
#define NEXT_KBD_DATA     0x0200e008
#define NEXT_INTR_KBD     0x00000008
#define NEXT_KBD_INT      0x00800000
#define NEXT_KBD_DAV      0x00400000
#define NEXT_KBD_OVR      0x00200000
#define NEXT_KBD_VALID    0x00008000
#define NEXT_KBD_DEVICE_1 0x10000000
#define NEXT_KEY_A        0x39
#define NEXT_KEY_UP       0x80
#define NEXT_ROM_SIZE     (128 * 1024)

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

static QTestState *next_cube_kbd_start(void)
{
    TestROM *rom = g_new0(TestROM, 1);
    g_autofree char *quoted_rom_path = NULL;
    QTestState *qts;

    rom->fd = -1;
    qtest_add_abrt_handler(cleanup_test_rom, rom);
    g_test_queue_destroy(cleanup_test_rom, rom);

    rom->fd = g_file_open_tmp("next-kbd-rom-XXXXXX", &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;

    quoted_rom_path = g_shell_quote(rom->path);
    qts = qtest_initf("-machine next-cube -bios %s", quoted_rom_path);
    return qts;
}

static void send_key(QTestState *qts, const char *qcode, bool down)
{
    qtest_qmp_assert_success(
        qts,
        "{ 'execute': 'input-send-event', 'arguments': { 'events': ["
        "{ 'type': 'key', 'data': { 'down': %i, "
        "'key': { 'type': 'qcode', 'data': %s } } } ] } }",
        down, qcode);
}

static void test_key_irq_and_data(void)
{
    QTestState *qts = next_cube_kbd_start();
    uint32_t csr;

    csr = qtest_readl(qts, NEXT_KBD_CSR);
    g_assert_cmphex(csr & (NEXT_KBD_INT | NEXT_KBD_DAV | NEXT_KBD_OVR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_KBD, ==, 0);

    send_key(qts, "a", true);
    send_key(qts, "a", false);

    csr = qtest_readl(qts, NEXT_KBD_CSR);
    g_assert_cmphex(csr & (NEXT_KBD_INT | NEXT_KBD_DAV),
                    ==, NEXT_KBD_INT | NEXT_KBD_DAV);
    g_assert_cmphex(csr & NEXT_KBD_OVR, ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_KBD,
                    ==, NEXT_INTR_KBD);

    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID | NEXT_KEY_A);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_KBD,
                    ==, NEXT_INTR_KBD);

    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID |
                    NEXT_KEY_UP | NEXT_KEY_A);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_KBD, ==, 0);
    csr = qtest_readl(qts, NEXT_KBD_CSR);
    g_assert_cmphex(csr & (NEXT_KBD_INT | NEXT_KBD_DAV | NEXT_KBD_OVR), ==, 0);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/next-cube/kbd/key-irq-and-data",
                   test_key_irq_and_data);
    return g_test_run();
}
