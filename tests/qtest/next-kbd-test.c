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
#define NEXT_KBD_CTX      0x00001000
#define NEXT_KBD_VALID    0x00008000
#define NEXT_KBD_LSHIFT   0x00000200
#define NEXT_KBD_DEVICE_1 0x10000000
#define NEXT_MOUSE_PACKET         0x11000000
#define NEXT_MOUSE_RIGHT_RELEASED 0x00000100
#define NEXT_MOUSE_LEFT_RELEASED  0x00000001
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

static void send_mouse_motion(QTestState *qts, int x, int y)
{
    qtest_qmp_assert_success(
        qts,
        "{ 'execute': 'input-send-event', 'arguments': { 'events': ["
        "{ 'type': 'rel', 'data': { 'axis': 'x', 'value': %d } },"
        "{ 'type': 'rel', 'data': { 'axis': 'y', 'value': %d } } ] } }",
        x, y);
}

static void send_mouse_button(QTestState *qts, const char *button, bool down)
{
    qtest_qmp_assert_success(
        qts,
        "{ 'execute': 'input-send-event', 'arguments': { 'events': ["
        "{ 'type': 'btn', 'data': { 'down': %i, "
        "'button': %s } } ] } }",
        down, button);
}

static void send_mouse_motion_and_button(QTestState *qts, int x, int y,
                                         const char *button, bool down)
{
    qtest_qmp_assert_success(
        qts,
        "{ 'execute': 'input-send-event', 'arguments': { 'events': ["
        "{ 'type': 'rel', 'data': { 'axis': 'x', 'value': %d } },"
        "{ 'type': 'rel', 'data': { 'axis': 'y', 'value': %d } },"
        "{ 'type': 'btn', 'data': { 'down': %i, "
        "'button': %s } } ] } }",
        x, y, down, button);
}

static int decode_mouse_delta(uint32_t field)
{
    field &= 0x7f;
    return field & 0x40 ? (int)field - 0x80 : (int)field;
}

static void test_idle_csr_ctx_clear(void)
{
    QTestState *qts = next_cube_kbd_start();
    uint8_t ctx_byte = qtest_readb(qts, NEXT_KBD_CSR + 2);
    uint32_t csr = qtest_readl(qts, NEXT_KBD_CSR);

    g_assert_cmphex(ctx_byte, ==, (csr >> 8) & 0xff);
    g_assert_cmphex(ctx_byte & 0x10, ==, 0);
    g_assert_cmphex(csr & NEXT_KBD_CTX, ==, 0);

    qtest_quit(qts);
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

static void test_key_dequeue_modifiers(void)
{
    QTestState *qts = next_cube_kbd_start();

    send_key(qts, "a", true);
    send_key(qts, "shift", true);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID |
                    NEXT_KBD_LSHIFT | NEXT_KEY_A);

    send_key(qts, "a", false);
    send_key(qts, "shift", false);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID |
                    NEXT_KEY_UP | NEXT_KEY_A);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_KBD, ==, 0);

    qtest_quit(qts);
}

static void test_mouse_packet_irq_and_data(void)
{
    QTestState *qts = next_cube_kbd_start();
    uint32_t csr;

    csr = qtest_readl(qts, NEXT_KBD_CSR);
    g_assert_cmphex(csr & (NEXT_KBD_INT | NEXT_KBD_DAV | NEXT_KBD_OVR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_KBD, ==, 0);

    /*
     * Host motion (3, -2) is reported as raw (-3, 2).  Right is released
     * (bit 8 set) and left is pressed (bit 0 clear).
     */
    send_mouse_motion_and_button(qts, 3, -2, "left", true);

    csr = qtest_readl(qts, NEXT_KBD_CSR);
    g_assert_cmphex(csr & (NEXT_KBD_INT | NEXT_KBD_DAV),
                    ==, NEXT_KBD_INT | NEXT_KBD_DAV);
    g_assert_cmphex(csr & NEXT_KBD_OVR, ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_KBD,
                    ==, NEXT_INTR_KBD);

    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==, 0x110005fa);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_KBD, ==, 0);
    csr = qtest_readl(qts, NEXT_KBD_CSR);
    g_assert_cmphex(csr & (NEXT_KBD_INT | NEXT_KBD_DAV | NEXT_KBD_OVR), ==, 0);

    qtest_quit(qts);
}

static void test_mouse_dequeue_modifier_isolation(void)
{
    QTestState *qts = next_cube_kbd_start();

    send_mouse_motion(qts, 0, -2);
    send_key(qts, "shift", true);

    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==, 0x11000501);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_KBD, ==, 0);

    qtest_quit(qts);
}

static void test_mouse_button_only(void)
{
    QTestState *qts = next_cube_kbd_start();
    uint32_t csr;

    send_mouse_button(qts, "right", true);

    csr = qtest_readl(qts, NEXT_KBD_CSR);
    g_assert_cmphex(csr & (NEXT_KBD_INT | NEXT_KBD_DAV),
                    ==, NEXT_KBD_INT | NEXT_KBD_DAV);
    g_assert_cmphex(csr & NEXT_KBD_OVR, ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_KBD,
                    ==, NEXT_INTR_KBD);

    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_MOUSE_PACKET | NEXT_MOUSE_LEFT_RELEASED);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_KBD, ==, 0);
    csr = qtest_readl(qts, NEXT_KBD_CSR);
    g_assert_cmphex(csr & (NEXT_KBD_INT | NEXT_KBD_DAV | NEXT_KBD_OVR), ==, 0);

    qtest_quit(qts);
}

static void test_mouse_large_motion(void)
{
    const int host_x = 150;
    const int host_y = -130;
    QTestState *qts = next_cube_kbd_start();
    unsigned int packet_count = 0;
    int guest_x = 0;
    int guest_y = 0;
    uint32_t csr;

    send_mouse_motion(qts, host_x, host_y);

    csr = qtest_readl(qts, NEXT_KBD_CSR);
    g_assert_cmphex(csr & (NEXT_KBD_INT | NEXT_KBD_DAV),
                    ==, NEXT_KBD_INT | NEXT_KBD_DAV);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_KBD,
                    ==, NEXT_INTR_KBD);

    do {
        uint32_t packet = qtest_readl(qts, NEXT_KBD_DATA);

        g_assert_cmphex(packet & 0xffff0000, ==, NEXT_MOUSE_PACKET);
        g_assert_cmphex(packet & (NEXT_MOUSE_RIGHT_RELEASED |
                                  NEXT_MOUSE_LEFT_RELEASED),
                        ==, NEXT_MOUSE_RIGHT_RELEASED |
                            NEXT_MOUSE_LEFT_RELEASED);
        guest_x += decode_mouse_delta(packet >> 1);
        guest_y += decode_mouse_delta(packet >> 9);
        packet_count++;
        g_assert_cmpuint(packet_count, <, 16);
        csr = qtest_readl(qts, NEXT_KBD_CSR);
        if (csr & NEXT_KBD_DAV) {
            g_assert_cmphex(csr & NEXT_KBD_INT, ==, NEXT_KBD_INT);
            g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                            NEXT_INTR_KBD, ==, NEXT_INTR_KBD);
        }
    } while (csr & NEXT_KBD_DAV);

    g_assert_cmpuint(packet_count, >, 1);
    g_assert_cmpint(-guest_x, ==, host_x);
    g_assert_cmpint(-guest_y, ==, host_y);
    g_assert_cmphex(csr & (NEXT_KBD_INT | NEXT_KBD_DAV | NEXT_KBD_OVR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_KBD, ==, 0);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/next-cube/kbd/key-irq-and-data",
                   test_key_irq_and_data);
    qtest_add_func("/next-cube/kbd/key-dequeue-modifiers",
                   test_key_dequeue_modifiers);
    qtest_add_func("/next-cube/kbd/idle-csr-ctx-clear",
                   test_idle_csr_ctx_clear);
    qtest_add_func("/next-cube/mouse/packet-irq-and-data",
                   test_mouse_packet_irq_and_data);
    qtest_add_func("/next-cube/mouse/dequeue-modifier-isolation",
                   test_mouse_dequeue_modifier_isolation);
    qtest_add_func("/next-cube/mouse/button-only",
                   test_mouse_button_only);
    qtest_add_func("/next-cube/mouse/large-motion",
                   test_mouse_large_motion);
    return g_test_run();
}
