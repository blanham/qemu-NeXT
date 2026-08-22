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
#include "qemu/timer.h"
#include "qemu/units.h"
#include "libqtest.h"
#include "qobject/qdict.h"
#include "qobject/qlist.h"

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
#define NEXT_KEY_LEFT_ARROW  0x09
#define NEXT_KEY_DOWN_ARROW  0x0f
#define NEXT_KEY_RIGHT_ARROW 0x10
#define NEXT_KEY_UP_ARROW    0x16
#define NEXT_KEY_ESC         0x49
#define NEXT_KEY_DEBUGGER    0x57
#define NEXT_ROM_SIZE     (128 * 1024)
#define NEXT_POLL_LIMIT   10000
#define NEXT_SOUND_TIMER_NS INT64_C(3000000)
#define NEXT_POINTER_WIDTH       1120
#define NEXT_POINTER_HEIGHT      832
#define NEXT_INPUT_ABS_MAX       0x7fff
#define NEXT_POINTER_TICK_NS \
    (NANOSECONDS_PER_SECOND / 68)
#define BARRIER_XORG_KEY_A 38

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

static QTestState *next_kbd_start(const char *machine, bool relative_pointer,
                                  const char *extra_args)
{
    TestROM *rom = g_new0(TestROM, 1);
    g_autofree char *quoted_rom_path = NULL;
    const char *pointer_arg = relative_pointer
        ? "-global next-kbd.absolute-pointer=off" : "";

    rom->fd = -1;
    qtest_add_abrt_handler(cleanup_test_rom, rom);
    g_test_queue_destroy(cleanup_test_rom, rom);

    rom->fd = g_file_open_tmp("next-kbd-rom-XXXXXX", &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;

    quoted_rom_path = g_shell_quote(rom->path);
    return qtest_initf("-machine %s -bios %s %s %s",
                       machine, quoted_rom_path, pointer_arg,
                       extra_args ?: "");
}

static QTestState *next_cube_kbd_start_with_args(const char *extra_args)
{
    return next_kbd_start("next-cube", true, extra_args);
}

static QTestState *next_cube_kbd_start(void)
{
    return next_cube_kbd_start_with_args(NULL);
}

static QTestState *next_cube_absolute_kbd_start(void)
{
    return next_kbd_start("next-cube", false, NULL);
}

static bool query_next_mouse_absolute(QTestState *qts)
{
    g_autoptr(QDict) response =
        qtest_qmp(qts, "{ 'execute': 'query-mice' }");
    QList *mice = qdict_get_qlist(response, "return");
    QListEntry *entry;

    QLIST_FOREACH_ENTRY(mice, entry) {
        QDict *mouse = qobject_to(QDict, qlist_entry_obj(entry));

        if (!strcmp(qdict_get_str(mouse, "name"),
                    "QEMU NeXT Keyboard/Mouse")) {
            return qdict_get_bool(mouse, "absolute");
        }
    }
    g_error("NeXT mouse missing from query-mice");
}

static void test_default_pointer_is_absolute(gconstpointer opaque)
{
    const char *machine = opaque;
    QTestState *qts = next_kbd_start(machine, false, NULL);

    g_assert_true(query_next_mouse_absolute(qts));
    qtest_quit(qts);
}

static void test_relative_pointer_opt_out(void)
{
    QTestState *qts = next_cube_kbd_start();

    g_assert_false(query_next_mouse_absolute(qts));
    qtest_quit(qts);
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

static void wait_migration_failed(QTestState *qts, const char *operation)
{
    unsigned int i;

    for (i = 0; i < NEXT_POLL_LIMIT; i++) {
        QDict *response = qtest_qmp_assert_success_ref(
            qts, "{ 'execute': 'query-migrate' }");
        const char *status = qdict_get_str(response, "status");

        if (!strcmp(status, "failed")) {
            const char *error_desc =
                qdict_get_try_str(response, "error-desc");

            g_assert_nonnull(error_desc);
            g_assert_nonnull(strstr(error_desc, "next-kbd"));
            g_test_message("incoming migration failure: %s", error_desc);
            qobject_unref(response);
            return;
        }
        if (!strcmp(status, "completed") || !strcmp(status, "cancelled")) {
            g_error("%s migration entered unexpected terminal state '%s'",
                    operation, status);
        }
        qobject_unref(response);
        g_usleep(1000);
    }

    g_error("timed out waiting for failed %s migration", operation);
}

static void save_next_kbd_migration(QTestState *source,
                                    char **migration_path)
{
    g_autofree char *path = NULL;
    g_autofree char *quoted_path = NULL;
    g_autofree char *outgoing_uri = NULL;
    int migration_fd;

    migration_fd = g_file_open_tmp("next-kbd-migration-XXXXXX",
                                   &path, NULL);
    g_assert_cmpint(migration_fd, >=, 0);
    close(migration_fd);
    quoted_path = g_shell_quote(path);
    outgoing_uri = g_strdup_printf("exec: cat > %s", quoted_path);
    qtest_qmp_assert_success(
        source, "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }",
        outgoing_uri);
    wait_migration_complete(source, "outgoing");
    qtest_quit(source);
    *migration_path = g_steal_pointer(&path);
}

static QTestState *start_next_kbd_incoming(const char *migration_path,
                                           bool relative_pointer,
                                           bool expect_failure)
{
    g_autofree char *quoted_path = g_shell_quote(migration_path);
    g_autofree char *incoming_uri =
        g_strdup_printf("exec: cat %s", quoted_path);
    QTestState *destination =
        next_kbd_start("next-cube", relative_pointer, "-incoming defer");

    if (expect_failure) {
        qtest_qmp_assert_success(
            destination,
            "{ 'execute': 'migrate-incoming', 'arguments': { "
            "'uri': %s, 'exit-on-error': false } }",
            incoming_uri);
    } else {
        qtest_qmp_assert_success(
            destination,
            "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
            incoming_uri);
    }
    return destination;
}

static QTestState *load_next_kbd_migration(const char *migration_path,
                                           bool relative_pointer)
{
    QTestState *destination =
        start_next_kbd_incoming(migration_path, relative_pointer, false);

    wait_migration_complete(destination, "incoming");
    return destination;
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

static int barrier_listen(uint16_t *port)
{
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
    };
    socklen_t addrlen = sizeof(addr);
    int fd;

    fd = socket(AF_INET, SOCK_STREAM, 0);
    g_assert_cmpint(fd, >=, 0);
    g_assert_cmpint(bind(fd, (struct sockaddr *)&addr, sizeof(addr)), ==, 0);
    g_assert_cmpint(getsockname(fd, (struct sockaddr *)&addr, &addrlen), ==, 0);
    g_assert_cmpint(listen(fd, 1), ==, 0);
    *port = ntohs(addr.sin_port);
    return fd;
}

static void barrier_write_all(int fd, const void *data, size_t size)
{
    const uint8_t *p = data;

    while (size) {
        ssize_t written = send(fd, p, size, 0);

        g_assert_cmpint(written, >, 0);
        p += written;
        size -= written;
    }
}

static void barrier_send_key(int fd, const char command[4], uint16_t repeat)
{
    uint8_t packet[16] = { 0 };
    uint32_t payload_size = repeat ? 12 : 10;
    uint32_t net_payload_size = htonl(payload_size);
    uint16_t net_repeat = htons(repeat);
    uint16_t net_button = htons(BARRIER_XORG_KEY_A);

    memcpy(packet, &net_payload_size, sizeof(net_payload_size));
    memcpy(packet + 4, command, 4);
    if (repeat) {
        memcpy(packet + 12, &net_repeat, sizeof(net_repeat));
        memcpy(packet + 14, &net_button, sizeof(net_button));
    } else {
        memcpy(packet + 12, &net_button, sizeof(net_button));
    }
    barrier_write_all(fd, packet, payload_size + sizeof(net_payload_size));
}

static void wait_for_keyboard_data(QTestState *qts)
{
    for (unsigned int i = 0; i < NEXT_POLL_LIMIT; i++) {
        if (qtest_readl(qts, NEXT_KBD_CSR) & NEXT_KBD_DAV) {
            return;
        }
        g_usleep(1000);
    }

    g_error("timed out waiting for NeXT keyboard data");
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

static int pixel_to_absolute(int pixel, int maximum_pixel)
{
    return ((int64_t)pixel * NEXT_INPUT_ABS_MAX +
            maximum_pixel - 1) / maximum_pixel;
}

static void send_absolute_pointer(QTestState *qts, int x, int y)
{
    int abs_x = pixel_to_absolute(x, NEXT_POINTER_WIDTH - 1);
    int abs_y = pixel_to_absolute(y, NEXT_POINTER_HEIGHT - 1);

    qtest_qmp_assert_success(
        qts,
        "{ 'execute': 'input-send-event', 'arguments': { 'events': ["
        "{ 'type': 'abs', 'data': { 'axis': 'x', 'value': %d } },"
        "{ 'type': 'abs', 'data': { 'axis': 'y', 'value': %d } } ] } }",
        abs_x, abs_y);
}

static void send_absolute_pointer_and_button(QTestState *qts, int x, int y,
                                             const char *button, bool down)
{
    int abs_x = pixel_to_absolute(x, NEXT_POINTER_WIDTH - 1);
    int abs_y = pixel_to_absolute(y, NEXT_POINTER_HEIGHT - 1);

    qtest_qmp_assert_success(
        qts,
        "{ 'execute': 'input-send-event', 'arguments': { 'events': ["
        "{ 'type': 'abs', 'data': { 'axis': 'x', 'value': %d } },"
        "{ 'type': 'abs', 'data': { 'axis': 'y', 'value': %d } },"
        "{ 'type': 'btn', 'data': { 'down': %i, "
        "'button': %s } } ] } }",
        abs_x, abs_y, down, button);
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

static int nextstep_factor(int raw_x, int raw_y)
{
    int distance = ABS(raw_x) + ABS(raw_y);

    if (distance <= 2) {
        return 1;
    }
    if (distance == 3) {
        return 2;
    }
    if (distance == 4) {
        return 4;
    }
    if (distance == 5) {
        return 6;
    }
    if (distance == 6) {
        return 8;
    }
    return 10;
}

static void accelerated_packet_motion(uint32_t packet, int *x, int *y)
{
    int raw_x = decode_mouse_delta(packet >> 1);
    int raw_y = decode_mouse_delta(packet >> 9);
    int factor = nextstep_factor(raw_x, raw_y);

    *x = -raw_x * factor;
    *y = -raw_y * factor;
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

static void test_cursor_keys_and_rom_monitor_shortcut(void)
{
    static const struct {
        const char *qcode;
        uint8_t next_code;
    } keys[] = {
        { "left", NEXT_KEY_LEFT_ARROW },
        { "down", NEXT_KEY_DOWN_ARROW },
        { "right", NEXT_KEY_RIGHT_ARROW },
        { "up", NEXT_KEY_UP_ARROW },
    };
    QTestState *qts = next_cube_kbd_start();

    for (size_t i = 0; i < ARRAY_SIZE(keys); i++) {
        send_key(qts, keys[i].qcode, true);
        g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                        NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID |
                        keys[i].next_code);

        send_key(qts, keys[i].qcode, false);
        g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                        NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID |
                        NEXT_KEY_UP | keys[i].next_code);
    }

    /* F11 is the historic NeXT extended-keyboard debugger key. */
    send_key(qts, "f11", true);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID | NEXT_KEY_DEBUGGER);
    send_key(qts, "f11", false);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID |
                    NEXT_KEY_UP | NEXT_KEY_DEBUGGER);

    /* Keep the existing physical Esc/debugger key unchanged. */
    send_key(qts, "esc", true);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID | NEXT_KEY_ESC);
    send_key(qts, "esc", false);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID |
                    NEXT_KEY_UP | NEXT_KEY_ESC);

    qtest_quit(qts);
}

static void test_duplicate_key_state_ignored(void)
{
    QTestState *qts = next_cube_kbd_start();

    send_key(qts, "a", true);
    send_key(qts, "a", true);

    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID | NEXT_KEY_A);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_CSR) &
                    (NEXT_KBD_INT | NEXT_KBD_DAV), ==, 0);

    send_key(qts, "a", false);
    send_key(qts, "a", false);

    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID |
                    NEXT_KEY_UP | NEXT_KEY_A);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_CSR) &
                    (NEXT_KBD_INT | NEXT_KBD_DAV), ==, 0);

    qtest_quit(qts);
}

static void test_key_state_cleared_by_reset(void)
{
    QTestState *qts = next_cube_kbd_start();

    send_key(qts, "a", true);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID | NEXT_KEY_A);
    assert_mouse_queue_empty(qts);

    qtest_qmp_assert_success(qts, "{ 'execute': 'system_reset' }");

    send_key(qts, "a", true);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID | NEXT_KEY_A);
    assert_mouse_queue_empty(qts);

    qtest_quit(qts);
}

static void test_barrier_repeat_is_key_down(void)
{
    static const bool expected_down[] = {
        true, true, true, true, false,
    };
    g_autofree char *args = NULL;
    g_autofree char *trace_contents = NULL;
    g_autofree char *trace_path = NULL;
    g_autofree char *quoted_trace_path = NULL;
    g_auto(GStrv) trace_lines = NULL;
    QTestState *qts;
    unsigned int event_count = 0;
    uint16_t port;
    int trace_fd;
    int listener;
    int client;

    listener = barrier_listen(&port);
    trace_fd = g_file_open_tmp("next-kbd-barrier-trace-XXXXXX",
                               &trace_path, NULL);
    g_assert_cmpint(trace_fd, >=, 0);
    close(trace_fd);
    quoted_trace_path = g_shell_quote(trace_path);
    args = g_strdup_printf("-object input-barrier,id=barrier0,name=test,"
                           "server=127.0.0.1,port=%u "
                           "-D %s -trace enable=input_event_key_qcode",
                           port, quoted_trace_path);
    qts = next_cube_kbd_start_with_args(args);
    client = accept(listener, NULL, NULL);
    g_assert_cmpint(client, >=, 0);

    barrier_send_key(client, "DKDN", 0);
    wait_for_keyboard_data(qts);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID | NEXT_KEY_A);
    assert_mouse_queue_empty(qts);

    barrier_send_key(client, "DKRP", 3);
    barrier_send_key(client, "DKUP", 0);
    wait_for_keyboard_data(qts);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID |
                    NEXT_KEY_UP | NEXT_KEY_A);
    assert_mouse_queue_empty(qts);

    qtest_quit(qts);
    close(client);
    close(listener);

    g_assert_true(g_file_get_contents(trace_path, &trace_contents,
                                      NULL, NULL));
    trace_lines = g_strsplit(trace_contents, "\n", -1);
    for (unsigned int i = 0; trace_lines[i]; i++) {
        if (!strstr(trace_lines[i], "input_event_key_qcode")) {
            continue;
        }

        g_assert_cmpuint(event_count, <, ARRAY_SIZE(expected_down));
        g_assert_nonnull(strstr(trace_lines[i], "key qcode a"));
        g_assert_nonnull(strstr(trace_lines[i],
                                expected_down[event_count] ?
                                "down 1" : "down 0"));
        event_count++;
    }
    g_assert_cmpuint(event_count, ==, ARRAY_SIZE(expected_down));
    g_assert_cmpint(g_unlink(trace_path), ==, 0);
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

static void test_absolute_acceleration_inverse(void)
{
    const int movements[] = { 1, 6, 16, 30, 48, 70, 100 };
    QTestState *qts = next_cube_absolute_kbd_start();
    int x = 100;

    send_absolute_pointer(qts, x, 100);
    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    assert_mouse_queue_empty(qts);

    for (size_t i = 0; i < ARRAY_SIZE(movements); i++) {
        uint32_t packet;
        int motion_x;
        int motion_y;

        x += movements[i];
        send_absolute_pointer(qts, x, 100);
        qtest_clock_step(qts, NEXT_POINTER_TICK_NS - 1);
        assert_mouse_queue_empty(qts);
        qtest_clock_step(qts, 1);
        packet = qtest_readl(qts, NEXT_KBD_DATA);
        accelerated_packet_motion(packet, &motion_x, &motion_y);
        g_assert_cmpint(motion_x, ==, movements[i]);
        g_assert_cmpint(motion_y, ==, 0);
        assert_mouse_queue_empty(qts);
    }

    qtest_quit(qts);
}

static void test_absolute_diagonal_and_residual(void)
{
    const int residual_x[] = { 6, 2, 2 };
    QTestState *qts = next_cube_absolute_kbd_start();
    int motion_x = 0;
    int motion_y = 0;

    send_absolute_pointer(qts, 200, 200);
    send_absolute_pointer(qts, 208, 208);
    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    accelerated_packet_motion(qtest_readl(qts, NEXT_KBD_DATA),
                              &motion_x, &motion_y);
    g_assert_cmpint(motion_x, ==, 8);
    g_assert_cmpint(motion_y, ==, 8);
    assert_mouse_queue_empty(qts);

    send_absolute_pointer(qts, 218, 208);
    for (unsigned int i = 0; i < 3; i++) {
        int packet_x;
        int packet_y;

        qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
        accelerated_packet_motion(qtest_readl(qts, NEXT_KBD_DATA),
                                  &packet_x, &packet_y);
        g_assert_cmpint(packet_x, ==, residual_x[i]);
        g_assert_cmpint(packet_y, ==, 0);
        motion_x += packet_x;
        motion_y += packet_y;
        assert_mouse_queue_empty(qts);
    }
    g_assert_cmpint(motion_x, ==, 18);
    g_assert_cmpint(motion_y, ==, 8);

    qtest_quit(qts);
}

static void test_absolute_surface_endpoints(void)
{
    QTestState *qts = next_cube_absolute_kbd_start();
    int motion_x;
    int motion_y;

    send_absolute_pointer(qts, 1, 1);
    send_absolute_pointer(qts, 0, 0);
    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    accelerated_packet_motion(qtest_readl(qts, NEXT_KBD_DATA),
                              &motion_x, &motion_y);
    g_assert_cmpint(motion_x, ==, -1);
    g_assert_cmpint(motion_y, ==, -1);
    assert_mouse_queue_empty(qts);
    qtest_quit(qts);

    qts = next_cube_absolute_kbd_start();
    send_absolute_pointer(qts, 1118, 830);
    send_absolute_pointer(qts, 1119, 831);
    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    accelerated_packet_motion(qtest_readl(qts, NEXT_KBD_DATA),
                              &motion_x, &motion_y);
    g_assert_cmpint(motion_x, ==, 1);
    g_assert_cmpint(motion_y, ==, 1);
    assert_mouse_queue_empty(qts);
    qtest_quit(qts);
}

static void test_absolute_large_motion_converges(void)
{
    QTestState *qts = next_cube_absolute_kbd_start();
    int motion_x = 0;
    int motion_y = 0;

    send_absolute_pointer(qts, 0, 100);
    send_absolute_pointer(qts, 500, 100);
    send_absolute_pointer(qts, 1000, 100);
    for (unsigned int i = 0; i < 2; i++) {
        int packet_x;
        int packet_y;

        qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
        accelerated_packet_motion(qtest_readl(qts, NEXT_KBD_DATA),
                                  &packet_x, &packet_y);
        motion_x += packet_x;
        motion_y += packet_y;
        assert_mouse_queue_empty(qts);
    }
    g_assert_cmpint(motion_x, ==, 1000);
    g_assert_cmpint(motion_y, ==, 0);

    qtest_quit(qts);
}

static void test_absolute_reset_discards_pending_and_anchor(void)
{
    QTestState *qts = next_cube_absolute_kbd_start();
    int motion_x;
    int motion_y;
    uint32_t packet;

    send_absolute_pointer(qts, 0, 100);
    send_mouse_button(qts, "left", true);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_MOUSE_PACKET | NEXT_MOUSE_RIGHT_RELEASED);
    assert_mouse_queue_empty(qts);

    send_absolute_pointer(qts, 500, 100);
    send_absolute_pointer_and_button(qts, 1000, 100, "left", false);
    qtest_system_reset(qts);

    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    assert_mouse_queue_empty(qts);

    send_absolute_pointer(qts, 600, 400);
    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    assert_mouse_queue_empty(qts);

    send_absolute_pointer(qts, 606, 400);
    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    packet = qtest_readl(qts, NEXT_KBD_DATA);
    accelerated_packet_motion(packet, &motion_x, &motion_y);
    g_assert_cmpint(motion_x, ==, 6);
    g_assert_cmpint(motion_y, ==, 0);
    g_assert_cmphex(packet & (NEXT_MOUSE_RIGHT_RELEASED |
                              NEXT_MOUSE_LEFT_RELEASED),
                    ==, NEXT_MOUSE_RIGHT_RELEASED |
                        NEXT_MOUSE_LEFT_RELEASED);
    assert_mouse_queue_empty(qts);

    send_mouse_button(qts, "left", true);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_MOUSE_PACKET | NEXT_MOUSE_RIGHT_RELEASED);
    assert_mouse_queue_empty(qts);

    qtest_quit(qts);
}

static void test_absolute_discontinuity_reanchors(void)
{
    QTestState *qts = next_cube_absolute_kbd_start();
    int motion_x;
    int motion_y;
    uint32_t packet;

    send_absolute_pointer(qts, 100, 100);
    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    assert_mouse_queue_empty(qts);

    send_absolute_pointer(qts, 106, 108);
    send_absolute_pointer(qts, 1000, 700);
    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    assert_mouse_queue_empty(qts);

    send_absolute_pointer(qts, 1006, 700);
    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    packet = qtest_readl(qts, NEXT_KBD_DATA);
    accelerated_packet_motion(packet, &motion_x, &motion_y);
    g_assert_cmpint(motion_x, ==, 6);
    g_assert_cmpint(motion_y, ==, 0);
    g_assert_cmphex(packet & (NEXT_MOUSE_RIGHT_RELEASED |
                              NEXT_MOUSE_LEFT_RELEASED),
                    ==, NEXT_MOUSE_RIGHT_RELEASED |
                        NEXT_MOUSE_LEFT_RELEASED);
    assert_mouse_queue_empty(qts);

    qtest_quit(qts);
}

static void test_absolute_cancelled_motion_delivers_button(void)
{
    QTestState *qts = next_cube_absolute_kbd_start();

    send_absolute_pointer(qts, 100, 100);
    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    assert_mouse_queue_empty(qts);

    send_absolute_pointer_and_button(qts, 106, 100, "left", true);
    send_absolute_pointer(qts, 100, 100);
    assert_mouse_queue_empty(qts);

    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_MOUSE_PACKET | NEXT_MOUSE_RIGHT_RELEASED);
    assert_mouse_queue_empty(qts);

    qtest_quit(qts);
}

static void test_absolute_discontinuity_delivers_waiting_button(void)
{
    QTestState *qts = next_cube_absolute_kbd_start();

    send_absolute_pointer(qts, 100, 100);
    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    assert_mouse_queue_empty(qts);

    send_absolute_pointer_and_button(qts, 106, 108, "left", true);
    send_absolute_pointer(qts, 1000, 700);
    assert_mouse_queue_empty(qts);

    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_MOUSE_PACKET | NEXT_MOUSE_RIGHT_RELEASED);
    assert_mouse_queue_empty(qts);

    qtest_quit(qts);
}

static void test_absolute_move_then_click(void)
{
    QTestState *qts = next_cube_absolute_kbd_start();
    int motion_x;
    int motion_y;
    uint32_t packet;

    send_absolute_pointer(qts, 100, 100);
    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    assert_mouse_queue_empty(qts);

    send_absolute_pointer_and_button(qts, 106, 100, "left", true);
    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    packet = qtest_readl(qts, NEXT_KBD_DATA);
    accelerated_packet_motion(packet, &motion_x, &motion_y);
    g_assert_cmpint(motion_x, ==, 6);
    g_assert_cmpint(motion_y, ==, 0);
    g_assert_cmphex(packet & (NEXT_MOUSE_RIGHT_RELEASED |
                              NEXT_MOUSE_LEFT_RELEASED),
                    ==, NEXT_MOUSE_RIGHT_RELEASED |
                        NEXT_MOUSE_LEFT_RELEASED);
    assert_mouse_queue_empty(qts);

    send_absolute_pointer(qts, 106, 100);
    assert_mouse_queue_empty(qts);

    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    packet = qtest_readl(qts, NEXT_KBD_DATA);
    accelerated_packet_motion(packet, &motion_x, &motion_y);
    g_assert_cmpint(motion_x, ==, 0);
    g_assert_cmpint(motion_y, ==, 0);
    g_assert_cmphex(packet & (NEXT_MOUSE_RIGHT_RELEASED |
                              NEXT_MOUSE_LEFT_RELEASED),
                    ==, NEXT_MOUSE_RIGHT_RELEASED);
    assert_mouse_queue_empty(qts);

    qtest_quit(qts);
}

static void test_absolute_button_only(void)
{
    QTestState *qts = next_cube_absolute_kbd_start();
    uint32_t csr;

    send_absolute_pointer(qts, 100, 100);
    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    assert_mouse_queue_empty(qts);

    send_mouse_button(qts, "left", true);
    csr = qtest_readl(qts, NEXT_KBD_CSR);
    g_assert_cmphex(csr & (NEXT_KBD_INT | NEXT_KBD_DAV),
                    ==, NEXT_KBD_INT | NEXT_KBD_DAV);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_INTR_KBD,
                    ==, NEXT_INTR_KBD);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_MOUSE_PACKET | NEXT_MOUSE_RIGHT_RELEASED);
    assert_mouse_queue_empty(qts);

    send_mouse_button(qts, "left", false);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_MOUSE_PACKET | NEXT_MOUSE_RIGHT_RELEASED |
                    NEXT_MOUSE_LEFT_RELEASED);
    assert_mouse_queue_empty(qts);

    qtest_quit(qts);
}

static void test_absolute_button_queue_full_retries(void)
{
    QTestState *qts = next_cube_absolute_kbd_start();

    send_absolute_pointer(qts, 100, 100);
    for (unsigned int i = 0; i < 256; i++) {
        send_key(qts, "a", !(i & 1));
    }
    send_mouse_button(qts, "left", true);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_CSR) & NEXT_KBD_OVR,
                    ==, NEXT_KBD_OVR);

    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID | NEXT_KEY_A);
    qtest_writeb(qts, NEXT_KBD_CSR + 1, NEXT_KBD_OVR >> 16);
    send_absolute_pointer(qts, 100, 100);
    for (unsigned int i = 0; i < 255; i++) {
        uint32_t packet = qtest_readl(qts, NEXT_KBD_DATA);

        g_assert_cmphex(packet & 0xf0000000, ==, NEXT_KBD_DEVICE_1);
    }
    assert_mouse_queue_empty(qts);

    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_MOUSE_PACKET | NEXT_MOUSE_RIGHT_RELEASED);
    assert_mouse_queue_empty(qts);

    qtest_quit(qts);
}

static void test_absolute_queue_full_retries_motion(void)
{
    QTestState *qts = next_cube_absolute_kbd_start();
    int motion_x;
    int motion_y;

    send_absolute_pointer(qts, 100, 100);
    for (unsigned int i = 0; i < 256; i++) {
        send_key(qts, "a", !(i & 1));
    }
    send_absolute_pointer(qts, 106, 100);
    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_CSR) & NEXT_KBD_OVR,
                    ==, NEXT_KBD_OVR);

    g_assert_cmphex(qtest_readl(qts, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID | NEXT_KEY_A);
    qtest_writeb(qts, NEXT_KBD_CSR + 1, NEXT_KBD_OVR >> 16);
    qtest_clock_step(qts, NEXT_POINTER_TICK_NS);
    for (unsigned int i = 0; i < 255; i++) {
        uint32_t packet = qtest_readl(qts, NEXT_KBD_DATA);

        g_assert_cmphex(packet & 0xf0000000, ==, NEXT_KBD_DEVICE_1);
    }
    accelerated_packet_motion(qtest_readl(qts, NEXT_KBD_DATA),
                              &motion_x, &motion_y);
    g_assert_cmpint(motion_x, ==, 6);
    g_assert_cmpint(motion_y, ==, 0);
    assert_mouse_queue_empty(qts);

    qtest_quit(qts);
}

static void test_migrate_queued_input(void)
{
    g_autofree char *migration_path = NULL;
    QTestState *source = next_cube_kbd_start();
    QTestState *destination;
    uint32_t csr;

    send_key(source, "a", true);
    send_mouse_motion_and_button(source, 3, -3, "left", true);
    send_key(source, "a", false);
    send_key(source, "a", true);

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

    save_next_kbd_migration(source, &migration_path);
    destination = load_next_kbd_migration(migration_path, true);

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
    g_assert_cmphex(qtest_readl(destination, NEXT_KBD_DATA), ==,
                    NEXT_KBD_DEVICE_1 | NEXT_KBD_VALID | NEXT_KEY_A);
    assert_mouse_queue_empty(destination);

    send_key(destination, "a", true);
    assert_mouse_queue_empty(destination);
    send_key(destination, "a", false);
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

static void test_migrate_absolute_pending_timer(void)
{
    g_autofree char *migration_path = NULL;
    QTestState *source = next_cube_absolute_kbd_start();
    QTestState *destination;
    uint32_t packet;
    int motion_x;
    int motion_y;
    int64_t source_clock;

    send_absolute_pointer(source, 100, 100);
    send_absolute_pointer(source, 106, 100);
    source_clock = qtest_clock_step(source, NEXT_POINTER_TICK_NS / 2);

    save_next_kbd_migration(source, &migration_path);
    destination = load_next_kbd_migration(migration_path, false);
    qtest_clock_set(destination, source_clock);

    qtest_clock_step(destination, NEXT_POINTER_TICK_NS / 2 - 1);
    assert_mouse_queue_empty(destination);
    qtest_clock_step(destination, 1);
    packet = qtest_readl(destination, NEXT_KBD_DATA);
    accelerated_packet_motion(packet, &motion_x, &motion_y);
    g_assert_cmpint(motion_x, ==, 6);
    g_assert_cmpint(motion_y, ==, 0);
    assert_mouse_queue_empty(destination);

    send_absolute_pointer(destination, 900, 700);
    qtest_clock_step(destination, NEXT_POINTER_TICK_NS);
    assert_mouse_queue_empty(destination);

    qtest_quit(destination);
    g_assert_cmpint(g_unlink(migration_path), ==, 0);
}

static void test_migrate_absolute_deferred_button(void)
{
    g_autofree char *migration_path = NULL;
    QTestState *source = next_cube_absolute_kbd_start();
    QTestState *destination;
    uint32_t packet;
    int motion_x;
    int motion_y;
    int64_t source_clock;

    send_absolute_pointer(source, 100, 100);
    qtest_clock_step(source, NEXT_POINTER_TICK_NS);
    assert_mouse_queue_empty(source);

    send_mouse_button(source, "left", true);
    g_assert_cmphex(qtest_readl(source, NEXT_KBD_DATA), ==,
                    NEXT_MOUSE_PACKET | NEXT_MOUSE_RIGHT_RELEASED);
    assert_mouse_queue_empty(source);

    send_absolute_pointer_and_button(source, 106, 100, "left", false);
    qtest_clock_step(source, NEXT_POINTER_TICK_NS);
    packet = qtest_readl(source, NEXT_KBD_DATA);
    accelerated_packet_motion(packet, &motion_x, &motion_y);
    g_assert_cmpint(motion_x, ==, 6);
    g_assert_cmpint(motion_y, ==, 0);
    g_assert_cmphex(packet & (NEXT_MOUSE_RIGHT_RELEASED |
                              NEXT_MOUSE_LEFT_RELEASED),
                    ==, NEXT_MOUSE_RIGHT_RELEASED);
    assert_mouse_queue_empty(source);

    source_clock = qtest_clock_step(source, NEXT_POINTER_TICK_NS / 2);
    save_next_kbd_migration(source, &migration_path);
    destination = load_next_kbd_migration(migration_path, false);
    qtest_clock_set(destination, source_clock);

    qtest_clock_step(destination, NEXT_POINTER_TICK_NS / 2 - 1);
    assert_mouse_queue_empty(destination);
    qtest_clock_step(destination, 1);
    g_assert_cmphex(qtest_readl(destination, NEXT_KBD_DATA), ==,
                    NEXT_MOUSE_PACKET | NEXT_MOUSE_RIGHT_RELEASED |
                    NEXT_MOUSE_LEFT_RELEASED);
    assert_mouse_queue_empty(destination);

    send_absolute_pointer(destination, 900, 700);
    qtest_clock_step(destination, NEXT_POINTER_TICK_NS);
    assert_mouse_queue_empty(destination);

    qtest_quit(destination);
    g_assert_cmpint(g_unlink(migration_path), ==, 0);
}

static void test_migrate_pointer_mode_mismatch_rejected(void)
{
    g_autofree char *migration_path = NULL;
    QTestState *source = next_cube_absolute_kbd_start();
    QTestState *destination;

    save_next_kbd_migration(source, &migration_path);
    destination = start_next_kbd_incoming(migration_path, true, true);
    wait_migration_failed(destination, "incoming");

    qtest_quit(destination);
    g_assert_cmpint(g_unlink(migration_path), ==, 0);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/next-cube/kbd/key-irq-and-data",
                   test_key_irq_and_data);
    qtest_add_func("/next-cube/kbd/cursor-keys-and-rom-monitor-shortcut",
                   test_cursor_keys_and_rom_monitor_shortcut);
    qtest_add_func("/next-cube/kbd/duplicate-key-state-ignored",
                   test_duplicate_key_state_ignored);
    qtest_add_func("/next-cube/kbd/key-state-cleared-by-reset",
                   test_key_state_cleared_by_reset);
    qtest_add_func("/next-cube/kbd/barrier-repeat-is-key-down",
                   test_barrier_repeat_is_key_down);
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
    g_test_add_data_func("/next-cube/mouse/default-pointer-is-absolute",
                         "next-cube", test_default_pointer_is_absolute);
    g_test_add_data_func("/next-station/mouse/default-pointer-is-absolute",
                         "next-station", test_default_pointer_is_absolute);
    g_test_add_data_func(
        "/next-station-color/mouse/default-pointer-is-absolute",
        "next-station-color", test_default_pointer_is_absolute);
    qtest_add_func("/next-cube/mouse/relative-pointer-opt-out",
                   test_relative_pointer_opt_out);
    qtest_add_func("/next-cube/mouse/absolute/acceleration-inverse",
                   test_absolute_acceleration_inverse);
    qtest_add_func("/next-cube/mouse/absolute/diagonal-and-residual",
                   test_absolute_diagonal_and_residual);
    qtest_add_func("/next-cube/mouse/absolute/surface-endpoints",
                   test_absolute_surface_endpoints);
    qtest_add_func("/next-cube/mouse/absolute/large-motion-converges",
                   test_absolute_large_motion_converges);
    qtest_add_func("/next-cube/mouse/absolute/reset-discards-pending-and-anchor",
                   test_absolute_reset_discards_pending_and_anchor);
    qtest_add_func("/next-cube/mouse/absolute/discontinuity-reanchors",
                   test_absolute_discontinuity_reanchors);
    qtest_add_func("/next-cube/mouse/absolute/cancelled-motion-delivers-button",
                   test_absolute_cancelled_motion_delivers_button);
    qtest_add_func(
        "/next-cube/mouse/absolute/discontinuity-delivers-waiting-button",
        test_absolute_discontinuity_delivers_waiting_button);
    qtest_add_func("/next-cube/mouse/absolute/move-then-click",
                   test_absolute_move_then_click);
    qtest_add_func("/next-cube/mouse/absolute/button-only",
                   test_absolute_button_only);
    qtest_add_func("/next-cube/mouse/absolute/button-queue-full-retries",
                   test_absolute_button_queue_full_retries);
    qtest_add_func("/next-cube/mouse/absolute/queue-full-retries-motion",
                   test_absolute_queue_full_retries_motion);
    qtest_add_func("/next-cube/kbd/migrate-queued-input",
                   test_migrate_queued_input);
    qtest_add_func("/next-cube/kbd/migrate-absolute-pending-timer",
                   test_migrate_absolute_pending_timer);
    qtest_add_func("/next-cube/kbd/migrate-absolute-deferred-button",
                   test_migrate_absolute_deferred_button);
    qtest_add_func("/next-cube/kbd/migrate-pointer-mode-mismatch-rejected",
                   test_migrate_pointer_mode_mismatch_rejected);
    return g_test_run();
}
