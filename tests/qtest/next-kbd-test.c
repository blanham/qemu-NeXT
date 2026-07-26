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
#include "qobject/qdict.h"

#define NEXT_INTR_STATUS  0x02007000
#define NEXT_KBD_CSR      0x0200e000
#define NEXT_MON_DATA     0x0200e004
#define NEXT_KBD_DATA     0x0200e008
#define NEXT_INTR_KBD     0x00000008
#define NEXT_INTR_SND_OVR 0x00000100
#define NEXT_DMAOUT_DMAEN 0x80000000
#define NEXT_DMAOUT_OVR   0x20000000
#define NEXT_KBD_INT      0x00800000
#define NEXT_KBD_DAV      0x00400000
#define NEXT_KBD_OVR      0x00200000
#define NEXT_MON_CTX_PEND 0x00002000
#define NEXT_KBD_CTX      0x00001000
#define NEXT_MON_DTX      0x00004000
#define NEXT_KBD_VALID    0x00008000
#define NEXT_KBD_LSHIFT   0x00000200
#define NEXT_KBD_DEVICE_1 0x10000000
#define NEXT_MOUSE_PACKET         0x11000000
#define NEXT_MOUSE_RIGHT_RELEASED 0x00000100
#define NEXT_MOUSE_LEFT_RELEASED  0x00000001
#define NEXT_KEY_A        0x39
#define NEXT_KEY_UP       0x80
#define NEXT_ROM_SIZE     (128 * 1024)
#define NEXT_POLL_LIMIT   10000
#define NEXT_SOUND_TIMER_NS INT64_C(3000000)

#define MON_SNDOUT_CTRL(options) (0x07 | ((options) << 3))
#define SOUT_ENAB                  0x01

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

static QTestState *next_cube_kbd_start_with_args(const char *extra_args)
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
    qts = qtest_initf("-machine next-cube -bios %s %s",
                      quoted_rom_path, extra_args);
    return qts;
}

static QTestState *next_cube_kbd_start(void)
{
    return next_cube_kbd_start_with_args("");
}

static void wait_migration_complete(QTestState *qts, const char *operation)
{
    unsigned int i;

    for (i = 0; i < NEXT_POLL_LIMIT; i++) {
        QDict *response = qtest_qmp_assert_success_ref(
            qts, "{ 'execute': 'query-migrate' }");
        const char *status = qdict_get_str(response, "status");

        if (!strcmp(status, "completed")) {
            qobject_unref(response);
            return;
        }
        if (!strcmp(status, "failed") || !strcmp(status, "cancelled")) {
            g_error("%s migration entered terminal state '%s'",
                    operation, status);
        }
        qobject_unref(response);
        g_usleep(1000);
    }

    g_error("timed out waiting for %s migration", operation);
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

static void assert_mouse_queue_empty(QTestState *qts)
{
    uint32_t csr = qtest_readl(qts, NEXT_KBD_CSR);

    g_assert_cmphex(csr & (NEXT_KBD_INT | NEXT_KBD_DAV | NEXT_KBD_OVR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_KBD, ==, 0);
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

static void test_sound_monitor_handshake_and_overrun(void)
{
    QTestState *qts = next_cube_kbd_start();
    uint32_t csr;

    csr = qtest_readl(qts, NEXT_KBD_CSR);
    g_assert_cmphex(csr & (NEXT_DMAOUT_DMAEN | NEXT_DMAOUT_OVR), ==, 0);

    qtest_writeb(qts, NEXT_KBD_CSR, NEXT_DMAOUT_DMAEN >> 24);
    g_assert_cmphex(qtest_readb(qts, NEXT_KBD_CSR), ==,
                    NEXT_DMAOUT_DMAEN >> 24);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_CSR) & NEXT_DMAOUT_DMAEN,
                    ==, NEXT_DMAOUT_DMAEN);

    qtest_writeb(qts, NEXT_KBD_CSR + 3, MON_SNDOUT_CTRL(SOUT_ENAB));
    qtest_writel(qts, NEXT_MON_DATA, 0);

    csr = qtest_readl(qts, NEXT_KBD_CSR);
    g_assert_cmphex(csr & (NEXT_MON_CTX_PEND | NEXT_KBD_CTX | NEXT_MON_DTX),
                    ==, 0);
    g_assert_cmphex(csr & NEXT_DMAOUT_OVR, ==, 0);

    qtest_clock_step(qts, NEXT_SOUND_TIMER_NS);
    csr = qtest_readl(qts, NEXT_KBD_CSR);
    g_assert_cmphex(csr & NEXT_DMAOUT_OVR, ==, NEXT_DMAOUT_OVR);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_SND_OVR,
                    ==, NEXT_INTR_SND_OVR);

    qtest_writeb(qts, NEXT_KBD_CSR,
                 (NEXT_DMAOUT_DMAEN | NEXT_DMAOUT_OVR) >> 24);
    csr = qtest_readl(qts, NEXT_KBD_CSR);
    g_assert_cmphex(csr & NEXT_DMAOUT_DMAEN, ==, NEXT_DMAOUT_DMAEN);
    g_assert_cmphex(csr & NEXT_DMAOUT_OVR, ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_SND_OVR,
                    ==, 0);

    qtest_writeb(qts, NEXT_KBD_CSR, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_CSR) & NEXT_DMAOUT_DMAEN, ==, 0);

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
     * Host motion (3, -3) is reported as raw (-1, 1).  Right is released
     * (bit 8 set) and left is pressed (bit 0 clear).
     */
    send_mouse_motion_and_button(qts, 3, -3, "left", true);

    csr = qtest_readl(qts, NEXT_KBD_CSR);
    g_assert_cmphex(csr & (NEXT_KBD_INT | NEXT_KBD_DAV),
                    ==, NEXT_KBD_INT | NEXT_KBD_DAV);
    g_assert_cmphex(csr & NEXT_KBD_OVR, ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_KBD,
                    ==, NEXT_INTR_KBD);

    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==, 0x110003fe);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_KBD, ==, 0);
    csr = qtest_readl(qts, NEXT_KBD_CSR);
    g_assert_cmphex(csr & (NEXT_KBD_INT | NEXT_KBD_DAV | NEXT_KBD_OVR), ==, 0);

    qtest_quit(qts);
}

static void test_mouse_dequeue_modifier_isolation(void)
{
    QTestState *qts = next_cube_kbd_start();

    send_mouse_motion(qts, 0, -6);
    send_key(qts, "shift", true);

    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==, 0x11000501);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_KBD, ==, 0);

    qtest_quit(qts);
}

static void test_mouse_scaled_signed_remainder(void)
{
    QTestState *qts = next_cube_kbd_start();

    send_mouse_motion(qts, 1, -1);
    assert_mouse_queue_empty(qts);
    send_mouse_motion(qts, 1, -1);
    assert_mouse_queue_empty(qts);
    send_mouse_motion(qts, 1, -1);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==, 0x110003ff);
    assert_mouse_queue_empty(qts);

    send_mouse_motion(qts, -1, 1);
    assert_mouse_queue_empty(qts);
    send_mouse_motion(qts, -1, 1);
    assert_mouse_queue_empty(qts);
    send_mouse_motion(qts, -1, 1);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==, 0x1100ff03);
    assert_mouse_queue_empty(qts);

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
    const int host_x = 450;
    const int host_y = -391;
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
    g_assert_cmpint(-guest_x, ==, host_x / 3);
    g_assert_cmpint(-guest_y, ==, host_y / 3);
    assert_mouse_queue_empty(qts);

    send_mouse_motion(qts, 0, -2);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==, 0x11000301);
    assert_mouse_queue_empty(qts);

    qtest_quit(qts);
}

static void test_migrate_queued_input(void)
{
    g_autofree char *migration_path = NULL;
    g_autofree char *quoted_migration_path = NULL;
    g_autofree char *outgoing_uri = NULL;
    g_autofree char *incoming_uri = NULL;
    QTestState *source = next_cube_kbd_start();
    QTestState *destination;
    uint32_t csr;
    int migration_fd;

    send_key(source, "a", true);
    send_mouse_motion_and_button(source, 3, -3, "left", true);
    send_key(source, "a", false);

    /*
     * Preserve a signed sub-packet remainder across migration.  These first
     * two motions are below the 3:1 scale threshold, so they do not enqueue
     * packets.
     */
    send_mouse_motion(source, 1, -1);
    send_mouse_motion(source, 1, -1);

    csr = qtest_readl(source, NEXT_KBD_CSR);
    g_assert_cmphex(csr & (NEXT_KBD_INT | NEXT_KBD_DAV),
                    ==, NEXT_KBD_INT | NEXT_KBD_DAV);
    g_assert_cmphex(csr & NEXT_KBD_OVR, ==, 0);
    g_assert_cmphex(qtest_readl(source, NEXT_INTR_STATUS) & NEXT_INTR_KBD,
                    ==, NEXT_INTR_KBD);

    migration_fd = g_file_open_tmp("next-kbd-migration-XXXXXX",
                                   &migration_path, NULL);
    g_assert_cmpint(migration_fd, >=, 0);
    close(migration_fd);
    quoted_migration_path = g_shell_quote(migration_path);
    outgoing_uri = g_strdup_printf("exec: cat > %s",
                                   quoted_migration_path);
    qtest_qmp_assert_success(
        source, "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }",
        outgoing_uri);
    wait_migration_complete(source, "outgoing");
    qtest_quit(source);

    destination = next_cube_kbd_start_with_args("-incoming defer");
    incoming_uri = g_strdup_printf("exec: cat %s", quoted_migration_path);
    qtest_qmp_assert_success(
        destination,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        incoming_uri);
    wait_migration_complete(destination, "incoming");

    csr = qtest_readl(destination, NEXT_KBD_CSR);
    g_assert_cmphex(csr & (NEXT_KBD_INT | NEXT_KBD_DAV),
                    ==, NEXT_KBD_INT | NEXT_KBD_DAV);
    g_assert_cmphex(csr & NEXT_KBD_OVR, ==, 0);
    g_assert_cmphex(qtest_readl(destination, NEXT_INTR_STATUS) &
                    NEXT_INTR_KBD, ==, NEXT_INTR_KBD);

    g_assert_cmphex(qtest_readl(destination, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID | NEXT_KEY_A);
    g_assert_cmphex(qtest_readl(destination, NEXT_INTR_STATUS) &
                    NEXT_INTR_KBD, ==, NEXT_INTR_KBD);
    g_assert_cmphex(qtest_readl(destination, NEXT_KBD_DATA), ==, 0x110003fe);
    g_assert_cmphex(qtest_readl(destination, NEXT_INTR_STATUS) &
                    NEXT_INTR_KBD, ==, NEXT_INTR_KBD);
    g_assert_cmphex(qtest_readl(destination, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID |
                    NEXT_KEY_UP | NEXT_KEY_A);
    assert_mouse_queue_empty(destination);

    /*
     * The third sub-threshold motion must combine with the migrated signed
     * remainder, while retaining the migrated pressed-left button state.
     */
    send_mouse_motion(destination, 1, -1);
    g_assert_cmphex(qtest_readl(destination, NEXT_KBD_DATA), ==, 0x110003fe);
    assert_mouse_queue_empty(destination);

    qtest_quit(destination);
    g_assert_cmpint(g_unlink(migration_path), ==, 0);
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
    qtest_add_func("/next-cube/monitor/sound-handshake-and-overrun",
                   test_sound_monitor_handshake_and_overrun);
    qtest_add_func("/next-cube/mouse/packet-irq-and-data",
                   test_mouse_packet_irq_and_data);
    qtest_add_func("/next-cube/mouse/dequeue-modifier-isolation",
                   test_mouse_dequeue_modifier_isolation);
    qtest_add_func("/next-cube/mouse/button-only",
                   test_mouse_button_only);
    qtest_add_func("/next-cube/mouse/scaled-signed-remainder",
                   test_mouse_scaled_signed_remainder);
    qtest_add_func("/next-cube/mouse/large-motion",
                   test_mouse_large_motion);
    qtest_add_func("/next-cube/kbd/migrate-queued-input",
                   test_migrate_queued_input);
    return g_test_run();
}
