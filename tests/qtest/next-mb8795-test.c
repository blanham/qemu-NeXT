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
#define NEXT_DMA_BASE    0x02000000
#define NEXT_ENTX_CSR    (NEXT_DMA_BASE + 0x0110)
#define NEXT_ENTX_SAVED_NEXT  (NEXT_ENTX_CSR + 0x3ff0)
#define NEXT_ENTX_SAVED_LIMIT (NEXT_ENTX_CSR + 0x3ff4)
#define NEXT_ENTX_SAVED_START (NEXT_ENTX_CSR + 0x3ff8)
#define NEXT_ENTX_SAVED_STOP  (NEXT_ENTX_CSR + 0x3ffc)
#define NEXT_ENTX_NEXT        (NEXT_ENTX_CSR + 0x4000)
#define NEXT_ENTX_LIMIT       (NEXT_ENTX_CSR + 0x4004)
#define NEXT_ENTX_START       (NEXT_ENTX_CSR + 0x4008)
#define NEXT_ENTX_STOP        (NEXT_ENTX_CSR + 0x400c)
#define NEXT_ENTX_NEXT_INIT   (NEXT_ENTX_CSR + 0x4200)
#define NEXT_ENRX_CSR         (NEXT_DMA_BASE + 0x0150)
#define NEXT_ENRX_SAVED_NEXT  (NEXT_ENRX_CSR + 0x3ff0)
#define NEXT_ENRX_SAVED_LIMIT (NEXT_ENRX_CSR + 0x3ff4)
#define NEXT_ENRX_SAVED_START (NEXT_ENRX_CSR + 0x3ff8)
#define NEXT_ENRX_SAVED_STOP  (NEXT_ENRX_CSR + 0x3ffc)
#define NEXT_ENRX_NEXT        (NEXT_ENRX_CSR + 0x4000)
#define NEXT_ENRX_LIMIT       (NEXT_ENRX_CSR + 0x4004)
#define NEXT_ENRX_START       (NEXT_ENRX_CSR + 0x4008)
#define NEXT_ENRX_STOP        (NEXT_ENRX_CSR + 0x400c)
#define NEXT_ENRX_NEXT_INIT   (NEXT_ENRX_CSR + 0x4200)
#define NEXT_INTR_STATUS 0x02007000
#define NEXT_ROM_SIZE    (128 * 1024)
#define NEXT_TX_BUFFER   0x04010000
#define NEXT_RX_BUFFER   0x04012000
#define NEXT_RX_BUFFER_2 0x04014000
#define NEXT_ENRX_IRQ    (1U << 9)
#define NEXT_ENTX_IRQ    (1U << 10)
#define NEXT_ENRX_DMA_IRQ (1U << 27)
#define NEXT_ENTX_DMA_IRQ (1U << 28)

#define DMA_SETENABLE      0x00010000
#define DMA_SETSUPDATE     0x00020000
#define DMA_CLRCOMPLETE    0x00080000
#define DMA_RESET          0x00100000
#define DMA_ENABLE         0x01000000
#define DMA_SUPDATE        0x02000000
#define DMA_COMPLETE       0x08000000
#define DMA_BUSEXC         0x10000000

#define ENTX_EOP           0x80000000
#define ENTX_ADDR_MASK     0x0fffffff
#define ENTX_END_BIAS      15

enum {
    EN_TXSTAT = 0x00,
    EN_TXMASK = 0x01,
    EN_RXSTAT = 0x02,
    EN_RXMASK = 0x03,
    EN_TXMODE = 0x04,
    EN_RXMODE = 0x05,
    EN_RESET = 0x06,
    EN_ADDR = 0x08,
};

#define EN_TXSTAT_READY     0x80
#define EN_TXSTAT_TXRECV    0x20
#define EN_TXSTAT_UNDERFLOW 0x08
#define EN_TXMASK_TXRXIE    0x20
#define EN_TXMODE_NO_LBC    0x02
#define EN_RXSTAT_OK        0x80
#define EN_RXSTAT_OVERFLOW  0x01
#define EN_RESET_MODE       0x80

#ifndef _WIN32
static const uint8_t rx_frame[60] = {
    0x52, 0x54, 0x00, 0x12, 0x34, 0x56,
    0x52, 0x54, 0x00, 0x65, 0x43, 0x21,
    0x08, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
    0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d,
};

static const uint8_t rx_fcs[4] = { 0xd5, 0xbd, 0x90, 0xc3 };

static const uint8_t arp_request[60] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x52, 0x54, 0x00, 0x12, 0x34, 0x56,
    0x08, 0x06,
    0x00, 0x01, 0x08, 0x00, 0x06, 0x04, 0x00, 0x01,
    0x52, 0x54, 0x00, 0x12, 0x34, 0x56,
    0x0a, 0x00, 0x02, 0x0f,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0a, 0x00, 0x02, 0x02,
};
#endif

typedef struct TestROM {
    int fd;
    char *path;
} TestROM;

#ifndef _WIN32
typedef struct TxHarness {
    QTestState *qts;
    int backend_fd;
} TxHarness;

typedef struct TxPointers {
    uint32_t saved_next;
    uint32_t saved_limit;
    uint32_t saved_start;
    uint32_t saved_stop;
    uint32_t next;
    uint32_t limit;
    uint32_t start;
    uint32_t stop;
    uint32_t next_init;
} TxPointers;

typedef struct RxPointers {
    uint32_t saved_next;
    uint32_t saved_limit;
    uint32_t next;
    uint32_t limit;
    uint32_t start;
    uint32_t stop;
    uint32_t next_init;
} RxPointers;
#endif

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

static QTestState *next_mb8795_start_with_args(const char *extra_args)
{
    TestROM *rom = g_new0(TestROM, 1);
    g_autofree char *quoted_rom_path = NULL;

    rom->fd = -1;
    qtest_add_abrt_handler(cleanup_test_rom, rom);
    g_test_queue_destroy(cleanup_test_rom, rom);

    rom->fd = g_file_open_tmp("next-mb8795-rom-XXXXXX",
                              &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;

    quoted_rom_path = g_shell_quote(rom->path);
    return qtest_initf("-machine next-cube -bios %s %s",
                       quoted_rom_path, extra_args ?: "");
}

static QTestState *next_mb8795_start(void)
{
    return next_mb8795_start_with_args(NULL);
}

#ifndef _WIN32
static TxHarness tx_harness_start(void)
{
    int pair[2];
    g_autofree char *args = NULL;
    TxHarness harness;

    g_assert_cmpint(socketpair(PF_UNIX, SOCK_STREAM, 0, pair), ==, 0);
    args = g_strdup_printf("-netdev socket,id=nextnet,fd=%d "
                           "-net nic,model=next-mb8795,netdev=nextnet",
                           pair[1]);
    harness.qts = next_mb8795_start_with_args(args);
    harness.backend_fd = pair[0];
    close(pair[1]);
    return harness;
}

static void tx_harness_stop(TxHarness *harness)
{
    qtest_quit(harness->qts);
    close(harness->backend_fd);
}

static void socket_read_exact(int fd, void *buf, size_t length)
{
    uint8_t *bytes = buf;
    size_t offset = 0;

    while (offset < length) {
        GPollFD pollfd = {
            .fd = fd,
            .events = G_IO_IN,
        };
        ssize_t ret;

        g_assert_cmpint(g_poll(&pollfd, 1, 1000), ==, 1);
        do {
            ret = recv(fd, bytes + offset, length - offset, 0);
        } while (ret < 0 && errno == EINTR);
        g_assert_cmpint(ret, >, 0);
        offset += ret;
    }
}

static void socket_assert_empty(int fd)
{
    uint8_t byte;
    ssize_t ret;

    do {
        ret = recv(fd, &byte, sizeof(byte), MSG_DONTWAIT);
    } while (ret < 0 && errno == EINTR);
    g_assert_cmpint(ret, ==, -1);
    g_assert_true(errno == EAGAIN || errno == EWOULDBLOCK);
}

static void socket_read_frame(int fd, uint8_t *frame, size_t length)
{
    uint32_t wire_length;

    socket_read_exact(fd, &wire_length, sizeof(wire_length));
    g_assert_cmpuint(ntohl(wire_length), ==, length);
    socket_read_exact(fd, frame, length);
}

static void socket_write_frame(int fd, const uint8_t *frame, size_t length)
{
    uint32_t wire_length = htonl(length);
    const uint8_t *parts[] = {
        (const uint8_t *)&wire_length,
        frame,
    };
    const size_t lengths[] = { sizeof(wire_length), length };
    size_t part;

    for (part = 0; part < ARRAY_SIZE(parts); part++) {
        size_t offset = 0;

        while (offset < lengths[part]) {
            ssize_t ret;

            do {
                ret = send(fd, parts[part] + offset,
                           lengths[part] - offset, 0);
            } while (ret < 0 && errno == EINTR);
            g_assert_cmpint(ret, >, 0);
            offset += ret;
        }
    }
}

static void rx_program(QTestState *qts, uint32_t first_start,
                       uint32_t first_limit, uint32_t second_start,
                       uint32_t second_limit, bool chain)
{
    qtest_writel(qts, NEXT_ENRX_CSR, DMA_RESET);
    qtest_writel(qts, NEXT_ENRX_NEXT, first_start);
    qtest_writel(qts, NEXT_ENRX_LIMIT, first_limit);
    qtest_writel(qts, NEXT_ENRX_START, second_start);
    qtest_writel(qts, NEXT_ENRX_STOP, second_limit);
    qtest_writel(qts, NEXT_ENRX_CSR,
                 DMA_SETENABLE | (chain ? DMA_SETSUPDATE : 0));
}

static void rx_prepare_controller(QTestState *qts, uint8_t mode)
{
    qtest_writeb(qts, NEXT_MB8795_BASE + EN_RESET, 0);
    qtest_writeb(qts, NEXT_MB8795_BASE + EN_RXMODE, mode);
}

static RxPointers rx_read_pointers(QTestState *qts)
{
    RxPointers pointers = {
        .saved_next = qtest_readl(qts, NEXT_ENRX_SAVED_NEXT),
        .saved_limit = qtest_readl(qts, NEXT_ENRX_SAVED_LIMIT),
        .next = qtest_readl(qts, NEXT_ENRX_NEXT),
        .limit = qtest_readl(qts, NEXT_ENRX_LIMIT),
        .start = qtest_readl(qts, NEXT_ENRX_START),
        .stop = qtest_readl(qts, NEXT_ENRX_STOP),
        .next_init = qtest_readl(qts, NEXT_ENRX_NEXT_INIT),
    };

    return pointers;
}

static void rx_assert_pointers_equal(const RxPointers *actual,
                                     const RxPointers *expected)
{
    g_assert_cmphex(actual->saved_next, ==, expected->saved_next);
    g_assert_cmphex(actual->saved_limit, ==, expected->saved_limit);
    g_assert_cmphex(actual->next, ==, expected->next);
    g_assert_cmphex(actual->limit, ==, expected->limit);
    g_assert_cmphex(actual->start, ==, expected->start);
    g_assert_cmphex(actual->stop, ==, expected->stop);
    g_assert_cmphex(actual->next_init, ==, expected->next_init);
}

static void tx_write_saved_sentinels(QTestState *qts)
{
    qtest_writel(qts, NEXT_ENTX_SAVED_NEXT, 0x11223344);
    qtest_writel(qts, NEXT_ENTX_SAVED_LIMIT, 0x55667788);
    qtest_writel(qts, NEXT_ENTX_SAVED_START, 0x99aabbcc);
    qtest_writel(qts, NEXT_ENTX_SAVED_STOP, 0xddeeff00);
}

static TxPointers tx_read_pointers(QTestState *qts)
{
    TxPointers pointers = {
        .saved_next = qtest_readl(qts, NEXT_ENTX_SAVED_NEXT),
        .saved_limit = qtest_readl(qts, NEXT_ENTX_SAVED_LIMIT),
        .saved_start = qtest_readl(qts, NEXT_ENTX_SAVED_START),
        .saved_stop = qtest_readl(qts, NEXT_ENTX_SAVED_STOP),
        .next = qtest_readl(qts, NEXT_ENTX_NEXT),
        .limit = qtest_readl(qts, NEXT_ENTX_LIMIT),
        .start = qtest_readl(qts, NEXT_ENTX_START),
        .stop = qtest_readl(qts, NEXT_ENTX_STOP),
        .next_init = qtest_readl(qts, NEXT_ENTX_NEXT_INIT),
    };

    return pointers;
}

static void tx_assert_pointers_equal(const TxPointers *actual,
                                     const TxPointers *expected)
{
    g_assert_cmphex(actual->saved_next, ==, expected->saved_next);
    g_assert_cmphex(actual->saved_limit, ==, expected->saved_limit);
    g_assert_cmphex(actual->saved_start, ==, expected->saved_start);
    g_assert_cmphex(actual->saved_stop, ==, expected->saved_stop);
    g_assert_cmphex(actual->next, ==, expected->next);
    g_assert_cmphex(actual->limit, ==, expected->limit);
    g_assert_cmphex(actual->start, ==, expected->start);
    g_assert_cmphex(actual->stop, ==, expected->stop);
    g_assert_cmphex(actual->next_init, ==, expected->next_init);
}

static void tx_prepare_controller(QTestState *qts)
{
    qtest_writeb(qts, NEXT_MB8795_BASE + EN_RESET, 0);
    qtest_writeb(qts, NEXT_MB8795_BASE + EN_TXMODE, EN_TXMODE_NO_LBC);
    qtest_writeb(qts, NEXT_MB8795_BASE + EN_TXSTAT, EN_TXSTAT_READY);
    g_assert_cmphex(qtest_readb(qts, NEXT_MB8795_BASE + EN_TXSTAT), ==, 0);
}
#endif

static char *find_unattached_device(QTestState *qts, const char *type)
{
    g_autoptr(QDict) response = NULL;
    g_autofree char *child_type = g_strdup_printf("child<%s>", type);
    g_autofree char *path = NULL;
    QList *children;
    QListEntry *entry;

    response = qtest_qmp(
        qts, "{ 'execute': 'qom-list', "
        "'arguments': { 'path': '/machine/unattached' } }");
    g_assert_nonnull(response);
    g_assert_true(qdict_haskey(response, "return"));
    children = qdict_get_qlist(response, "return");
    QLIST_FOREACH_ENTRY(children, entry) {
        QDict *child = qobject_to(QDict, qlist_entry_obj(entry));

        if (!strcmp(qdict_get_str(child, "type"), child_type)) {
            g_assert_null(path);
            path = g_strdup_printf("/machine/unattached/%s",
                                   qdict_get_str(child, "name"));
        }
    }
    g_assert_nonnull(path);

    return g_steal_pointer(&path);
}

static void assert_single_mb8795(QTestState *qts)
{
    g_autoptr(QDict) response = NULL;
    QList *children;
    QListEntry *entry;
    unsigned int count = 0;

    response = qtest_qmp(
        qts, "{ 'execute': 'qom-list', "
        "'arguments': { 'path': '/machine' } }");
    g_assert_nonnull(response);
    g_assert_true(qdict_haskey(response, "return"));
    children = qdict_get_qlist(response, "return");
    QLIST_FOREACH_ENTRY(children, entry) {
        QDict *child = qobject_to(QDict, qlist_entry_obj(entry));

        if (!strcmp(qdict_get_str(child, "type"),
                    "child<next-mb8795>")) {
            g_assert_cmpstr(qdict_get_str(child, "name"), ==, "mb8795");
            count++;
        }
    }
    g_assert_cmpuint(count, ==, 1);
}

static void unrealize_next_kbd(QTestState *qts)
{
    g_autofree char *path = find_unattached_device(qts, "next-kbd");

    qtest_qmp_assert_success(
        qts,
        "{ 'execute': 'qom-set', 'arguments': { "
        "'path': %s, 'property': 'realized', 'value': false } }", path);
}

static void migrate_wait(QTestState *source, QTestState *destination,
                         const char *uri)
{
    qtest_qmp_assert_success(
        source,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    qtest_qmp_eventwait(source, "STOP");
    qtest_qmp_eventwait(destination, "RESUME");
}

static uint8_t en_readb(QTestState *qts, uint64_t reg)
{
    return qtest_readb(qts, NEXT_MB8795_BASE + reg);
}

static void en_writeb(QTestState *qts, uint64_t reg, uint8_t value)
{
    qtest_writeb(qts, NEXT_MB8795_BASE + reg, value);
}

static uint32_t controller_irqs(QTestState *qts)
{
    return qtest_readl(qts, NEXT_INTR_STATUS) &
           (NEXT_ENTX_IRQ | NEXT_ENRX_IRQ);
}

static void test_register_reset(void)
{
    QTestState *qts = next_mb8795_start();

    /*
     * Retained NeXT and NetBSD drivers enter RESET_MODE before programming
     * the controller, so machine reset leaves the device safely latched in
     * reset.  The station address is separate persistent configuration.
     */
    g_assert_cmphex(en_readb(qts, EN_RESET), ==, EN_RESET_MODE);
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==, 0);
    g_assert_cmphex(en_readb(qts, EN_TXMASK), ==, 0);
    g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==, 0);
    g_assert_cmphex(en_readb(qts, EN_RXMASK), ==, 0);
    g_assert_cmphex(en_readb(qts, EN_TXMODE), ==, 0);
    g_assert_cmphex(en_readb(qts, EN_RXMODE), ==, 0);
    g_assert_cmphex(controller_irqs(qts), ==, 0);

    en_writeb(qts, EN_RESET, 0);
    g_assert_cmphex(en_readb(qts, EN_RESET), ==, 0);
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==, EN_TXSTAT_READY);

    en_writeb(qts, EN_TXMASK, EN_TXSTAT_READY);
    en_writeb(qts, EN_RXMASK, EN_RXSTAT_OK | EN_RXSTAT_OVERFLOW);
    en_writeb(qts, EN_TXMODE, 0xa5);
    en_writeb(qts, EN_RXMODE, 0x5a);
    en_writeb(qts, EN_RESET, EN_RESET_MODE);
    g_assert_cmphex(en_readb(qts, EN_RESET), ==, EN_RESET_MODE);
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==, 0);
    g_assert_cmphex(en_readb(qts, EN_TXMASK), ==, 0);
    g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==, 0);
    g_assert_cmphex(en_readb(qts, EN_RXMASK), ==, 0);
    g_assert_cmphex(en_readb(qts, EN_TXMODE), ==, 0);
    g_assert_cmphex(en_readb(qts, EN_RXMODE), ==, 0);
    g_assert_cmphex(controller_irqs(qts), ==, 0);

    en_writeb(qts, EN_RESET, 0);
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==, EN_TXSTAT_READY);

    qtest_quit(qts);
}

static void test_station_address(void)
{
    static const uint8_t configured[6] = {
        0x52, 0x54, 0x00, 0x12, 0x34, 0x56,
    };
    static const uint8_t programmed[6] = {
        0x00, 0x00, 0x0f, 0xaa, 0xbb, 0xcc,
    };
    QTestState *qts = next_mb8795_start_with_args(
        "-netdev user,id=nextnet "
        "-net nic,model=next-mb8795,netdev=nextnet,"
        "macaddr=52:54:00:12:34:56");
    size_t i;

    assert_single_mb8795(qts);
    for (i = 0; i < ARRAY_SIZE(configured); i++) {
        g_assert_cmphex(en_readb(qts, EN_ADDR + i), ==, configured[i]);
        en_writeb(qts, EN_ADDR + i, programmed[i]);
        g_assert_cmphex(en_readb(qts, EN_ADDR + i), ==, programmed[i]);
    }

    en_writeb(qts, EN_RESET, 0);
    en_writeb(qts, EN_RESET, EN_RESET_MODE);
    qtest_system_reset(qts);
    for (i = 0; i < ARRAY_SIZE(programmed); i++) {
        g_assert_cmphex(en_readb(qts, EN_ADDR + i), ==, programmed[i]);
    }

    qtest_quit(qts);
}

static void test_access_widths(void)
{
    static const uint64_t holes[] = { 0x07, 0x0e, 0x0f };
    QTestState *qts = next_mb8795_start();
    size_t i;

    en_writeb(qts, EN_TXMASK, 0xa5);
    en_writeb(qts, EN_RXMASK, 0x5a);
    en_writeb(qts, EN_TXMODE, 0x3c);
    en_writeb(qts, EN_RXMODE, 0xc3);

    g_assert_cmphex(qtest_readw(qts, NEXT_MB8795_BASE + EN_TXSTAT), ==, 0);
    qtest_writew(qts, NEXT_MB8795_BASE + EN_TXSTAT, 0xffff);
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==, 0);
    g_assert_cmphex(en_readb(qts, EN_TXMASK), ==, 0xa5);
    g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==, 0);

    g_assert_cmphex(qtest_readl(qts, NEXT_MB8795_BASE + EN_TXMODE), ==, 0);
    qtest_writel(qts, NEXT_MB8795_BASE + EN_TXMODE, 0xffffffff);
    g_assert_cmphex(en_readb(qts, EN_TXMODE), ==, 0x3c);
    g_assert_cmphex(en_readb(qts, EN_RXMODE), ==, 0xc3);
    g_assert_cmphex(en_readb(qts, EN_RESET), ==, EN_RESET_MODE);

    for (i = 0; i < ARRAY_SIZE(holes); i++) {
        en_writeb(qts, holes[i], 0xff);
        g_assert_cmphex(en_readb(qts, holes[i]), ==, 0);
    }

    qtest_quit(qts);
}

static void test_status_w1c(void)
{
    QTestState *qts = next_mb8795_start();

    en_writeb(qts, EN_RESET, 0);
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==, EN_TXSTAT_READY);

    en_writeb(qts, EN_TXSTAT, 0);
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==, EN_TXSTAT_READY);
    en_writeb(qts, EN_TXSTAT, EN_TXSTAT_UNDERFLOW);
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==, EN_TXSTAT_READY);
    en_writeb(qts, EN_TXSTAT, EN_TXSTAT_READY);
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==, 0);

    en_writeb(qts, EN_RXSTAT, 0);
    en_writeb(qts, EN_RXSTAT, EN_RXSTAT_OK | EN_RXSTAT_OVERFLOW);
    g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==, 0);

    qtest_quit(qts);
}

static void test_dma_next_init_read_alias(void)
{
    QTestState *qts = next_mb8795_start();

    qtest_writel(qts, NEXT_ENRX_CSR, DMA_RESET);
    qtest_writel(qts, NEXT_ENRX_NEXT, 0x04012000);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_NEXT_INIT), ==,
                    0x04012000);
    qtest_writel(qts, NEXT_ENRX_SAVED_START, 0x04014000);
    qtest_writel(qts, NEXT_ENRX_SAVED_STOP, 0x04015000);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_SAVED_START), ==,
                    0x04014000);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_SAVED_STOP), ==,
                    0x04015000);

    qtest_writel(qts, NEXT_ENTX_CSR, DMA_RESET);
    qtest_writel(qts, NEXT_ENTX_NEXT_INIT, 0x04010000);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_NEXT), ==,
                    0x04010000);

    qtest_quit(qts);
}

static void test_mask_status_irqs(void)
{
    QTestState *qts = next_mb8795_start();
    g_autofree char *pc_path = find_unattached_device(qts, "next-pc");

    en_writeb(qts, EN_RESET, 0);
    g_assert_cmphex(controller_irqs(qts), ==, 0);

    en_writeb(qts, EN_TXMASK, EN_TXSTAT_UNDERFLOW);
    g_assert_cmphex(controller_irqs(qts), ==, 0);
    en_writeb(qts, EN_TXMASK,
              EN_TXSTAT_READY | EN_TXSTAT_UNDERFLOW);
    g_assert_cmphex(controller_irqs(qts), ==, NEXT_ENTX_IRQ);

    en_writeb(qts, EN_RXMASK, EN_RXSTAT_OK | EN_RXSTAT_OVERFLOW);
    g_assert_cmphex(controller_irqs(qts), ==, NEXT_ENTX_IRQ);

    /*
     * Drive the independent board RX input to prove a TX status
     * acknowledgement cannot clear NEXT_ENRX_I.
     */
    qtest_set_irq_in(qts, pc_path, NULL, 3, 1);
    g_assert_cmphex(controller_irqs(qts), ==,
                    NEXT_ENTX_IRQ | NEXT_ENRX_IRQ);
    en_writeb(qts, EN_TXSTAT, EN_TXSTAT_READY);
    g_assert_cmphex(controller_irqs(qts), ==, NEXT_ENRX_IRQ);
    qtest_set_irq_in(qts, pc_path, NULL, 3, 0);
    g_assert_cmphex(controller_irqs(qts), ==, 0);

    qtest_quit(qts);
}

static void test_mtree_window(void)
{
    static const char line[] =
        "0000000002106000-000000000210600f (prio 0, i/o): next.mb8795";
    QTestState *qts = next_mb8795_start();
    g_autofree char *flatview = qtest_hmp(qts, "info mtree -f");
    const char *match = flatview;
    unsigned int count = 0;

    assert_single_mb8795(qts);
    while ((match = strstr(match, line))) {
        count++;
        match += strlen(line);
    }
    g_assert_cmpuint(count, ==, 1);
    g_assert_null(strstr(flatview,
        "0000000002106000-000000000210601f (prio 0, i/o): next.en"));

    qtest_quit(qts);
}

#ifndef _WIN32
static void test_tx_single_buffer(void)
{
    enum { FRAME_LENGTH = 64 };
    TxHarness harness = tx_harness_start();
    QTestState *qts = harness.qts;
    uint8_t frame[FRAME_LENGTH];
    uint8_t received[FRAME_LENGTH];
    uint8_t rx_before[FRAME_LENGTH + sizeof(rx_fcs)];
    uint8_t rx_after[sizeof(rx_before)];
    TxPointers saved;
    size_t i;

    for (i = 0; i < sizeof(frame); i++) {
        frame[i] = 0x40 + i;
    }
    memset(rx_before, 0xa5, sizeof(rx_before));
    qtest_memwrite(qts, NEXT_TX_BUFFER, frame, sizeof(frame));
    qtest_memwrite(qts, NEXT_RX_BUFFER, rx_before, sizeof(rx_before));
    tx_write_saved_sentinels(qts);
    qtest_writel(qts, NEXT_ENTX_NEXT, NEXT_TX_BUFFER + 0x1000);
    qtest_writel(qts, NEXT_ENTX_LIMIT,
                 ENTX_EOP | (NEXT_TX_BUFFER + FRAME_LENGTH +
                             ENTX_END_BIAS));
    qtest_writel(qts, NEXT_ENTX_NEXT_INIT, NEXT_TX_BUFFER);
    saved = tx_read_pointers(qts);
    rx_prepare_controller(qts, 3);
    rx_program(qts, NEXT_RX_BUFFER, NEXT_RX_BUFFER + 0x1000, 0, 0,
               false);
    tx_prepare_controller(qts);

    qtest_writel(qts, NEXT_ENTX_CSR, DMA_SETENABLE);
    qtest_clock_step(qts, 1);
    socket_read_frame(harness.backend_fd, received, sizeof(received));
    qtest_memread(qts, NEXT_RX_BUFFER, rx_after, sizeof(rx_after));

    g_assert_cmpmem(received, sizeof(received), frame, sizeof(frame));
    g_assert_cmpmem(rx_after, sizeof(rx_after),
                    rx_before, sizeof(rx_before));
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_CSR) &
                    (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_ENABLE);
    g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_CSR) &
                    (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_NEXT), ==,
                    NEXT_TX_BUFFER + FRAME_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_LIMIT), ==, saved.limit);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_SAVED_NEXT), ==,
                    saved.saved_next);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_SAVED_LIMIT), ==,
                    saved.saved_limit);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_SAVED_START), ==,
                    saved.saved_start);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_SAVED_STOP), ==,
                    saved.saved_stop);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    NEXT_ENTX_DMA_IRQ, ==, NEXT_ENTX_DMA_IRQ);
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==, EN_TXSTAT_READY);
    socket_assert_empty(harness.backend_fd);

    tx_harness_stop(&harness);
}

static void test_tx_internal_loopback(void)
{
    TxHarness harness = tx_harness_start();
    QTestState *qts = harness.qts;
    uint8_t received[sizeof(rx_frame) + sizeof(rx_fcs)];
    size_t i;

    memset(received, 0xa5, sizeof(received));
    qtest_memwrite(qts, NEXT_TX_BUFFER, rx_frame, sizeof(rx_frame));
    qtest_memwrite(qts, NEXT_RX_BUFFER, received, sizeof(received));
    qtest_writel(qts, NEXT_ENTX_NEXT, NEXT_TX_BUFFER);
    qtest_writel(qts, NEXT_ENTX_LIMIT,
                 ENTX_EOP | (NEXT_TX_BUFFER + sizeof(rx_frame) +
                             ENTX_END_BIAS));
    for (i = 0; i < 6; i++) {
        en_writeb(qts, EN_ADDR + i, rx_frame[i]);
    }
    rx_prepare_controller(qts, 1);
    rx_program(qts, NEXT_RX_BUFFER, NEXT_RX_BUFFER + 0x1000, 0, 0,
               false);
    tx_prepare_controller(qts);
    en_writeb(qts, EN_TXMASK, EN_TXMASK_TXRXIE);
    en_writeb(qts, EN_TXMODE, 0);

    qtest_writel(qts, NEXT_ENTX_CSR, DMA_SETENABLE);
    qtest_clock_step(qts, 1);
    qtest_memread(qts, NEXT_RX_BUFFER, received, sizeof(received));

    g_assert_cmpmem(received, sizeof(rx_frame),
                    rx_frame, sizeof(rx_frame));
    g_assert_cmpmem(received + sizeof(rx_frame), sizeof(rx_fcs),
                    rx_fcs, sizeof(rx_fcs));
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_CSR) &
                    (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_CSR) &
                    (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_COMPLETE);
    g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==, EN_RXSTAT_OK);
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==,
                    EN_TXSTAT_READY | EN_TXSTAT_TXRECV);
    g_assert_cmphex(controller_irqs(qts), ==, NEXT_ENTX_IRQ);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    (NEXT_ENRX_DMA_IRQ | NEXT_ENTX_DMA_IRQ),
                    ==, NEXT_ENRX_DMA_IRQ | NEXT_ENTX_DMA_IRQ);
    socket_assert_empty(harness.backend_fd);

    qtest_writel(qts, NEXT_ENTX_CSR, DMA_CLRCOMPLETE);
    qtest_writel(qts, NEXT_ENTX_NEXT, NEXT_TX_BUFFER);
    qtest_writel(qts, NEXT_ENTX_LIMIT,
                 ENTX_EOP | (NEXT_TX_BUFFER + sizeof(rx_frame) +
                             ENTX_END_BIAS));
    qtest_writel(qts, NEXT_ENTX_CSR, DMA_SETENABLE);
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==, EN_TXSTAT_READY);
    g_assert_cmphex(controller_irqs(qts), ==, 0);

    tx_harness_stop(&harness);
}

static void tx_internal_loopback_failure(uint8_t rx_mode,
                                         uint32_t rx_window,
                                         uint8_t expected_rx_status,
                                         uint32_t expected_rx_csr)
{
    TxHarness harness = tx_harness_start();
    QTestState *qts = harness.qts;
    uint8_t before[sizeof(rx_frame) + sizeof(rx_fcs)];
    uint8_t after[sizeof(before)];
    size_t i;

    memset(before, 0xa5, sizeof(before));
    qtest_memwrite(qts, NEXT_TX_BUFFER, rx_frame, sizeof(rx_frame));
    qtest_memwrite(qts, NEXT_RX_BUFFER, before, sizeof(before));
    qtest_writel(qts, NEXT_ENTX_NEXT, NEXT_TX_BUFFER);
    qtest_writel(qts, NEXT_ENTX_LIMIT,
                 ENTX_EOP | (NEXT_TX_BUFFER + sizeof(rx_frame) +
                             ENTX_END_BIAS));
    for (i = 0; i < 6; i++) {
        en_writeb(qts, EN_ADDR + i, rx_frame[i]);
    }
    rx_prepare_controller(qts, rx_mode);
    rx_program(qts, NEXT_RX_BUFFER, NEXT_RX_BUFFER + rx_window, 0, 0,
               false);
    tx_prepare_controller(qts);
    en_writeb(qts, EN_TXMASK, EN_TXMASK_TXRXIE);
    en_writeb(qts, EN_TXMODE, 0);

    qtest_writel(qts, NEXT_ENTX_CSR, DMA_SETENABLE);
    qtest_clock_step(qts, 1);
    qtest_memread(qts, NEXT_RX_BUFFER, after, sizeof(after));

    g_assert_cmpmem(after, sizeof(after), before, sizeof(before));
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==, EN_TXSTAT_READY);
    g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==, expected_rx_status);
    g_assert_cmphex(controller_irqs(qts), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_CSR) &
                    (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC),
                    ==, expected_rx_csr);
    socket_assert_empty(harness.backend_fd);

    tx_harness_stop(&harness);
}

static void test_tx_internal_loopback_filtered(void)
{
    tx_internal_loopback_failure(0, 0x1000, 0, DMA_ENABLE);
}

static void test_tx_internal_loopback_rx_failure(void)
{
    tx_internal_loopback_failure(1, 32, EN_RXSTAT_OVERFLOW,
                                 DMA_COMPLETE | DMA_BUSEXC);
}

static void test_tx_two_segment(void)
{
    enum {
        FIRST_LENGTH = 36,
        SECOND_LENGTH = 32,
        FRAME_LENGTH = FIRST_LENGTH + SECOND_LENGTH,
    };
    const uint32_t second = NEXT_TX_BUFFER + 0x200;
    TxHarness harness = tx_harness_start();
    QTestState *qts = harness.qts;
    uint8_t first[FIRST_LENGTH];
    uint8_t second_data[SECOND_LENGTH];
    uint8_t expected[FRAME_LENGTH];
    uint8_t received[FRAME_LENGTH];
    TxPointers saved;
    size_t i;

    for (i = 0; i < sizeof(first); i++) {
        first[i] = 0x20 + i;
    }
    for (i = 0; i < sizeof(second_data); i++) {
        second_data[i] = 0xa0 + i;
    }
    memcpy(expected, first, sizeof(first));
    memcpy(expected + sizeof(first), second_data, sizeof(second_data));
    qtest_memwrite(qts, NEXT_TX_BUFFER, first, sizeof(first));
    qtest_memwrite(qts, second, second_data, sizeof(second_data));
    tx_write_saved_sentinels(qts);
    qtest_writel(qts, NEXT_ENTX_NEXT, NEXT_TX_BUFFER);
    qtest_writel(qts, NEXT_ENTX_LIMIT, NEXT_TX_BUFFER + FIRST_LENGTH);
    qtest_writel(qts, NEXT_ENTX_START, second);
    qtest_writel(qts, NEXT_ENTX_STOP,
                 ENTX_EOP | (second + SECOND_LENGTH + ENTX_END_BIAS));
    saved = tx_read_pointers(qts);
    tx_prepare_controller(qts);

    qtest_writel(qts, NEXT_ENTX_CSR,
                 DMA_SETENABLE | DMA_SETSUPDATE);
    qtest_clock_step(qts, 1);
    socket_read_frame(harness.backend_fd, received, sizeof(received));

    g_assert_cmpmem(received, sizeof(received),
                    expected, sizeof(expected));
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE |
                     DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_NEXT), ==,
                    second + SECOND_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_LIMIT), ==, saved.stop);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_SAVED_NEXT), ==,
                    saved.saved_next);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_SAVED_LIMIT), ==,
                    saved.saved_limit);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_SAVED_START), ==,
                    saved.saved_start);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_SAVED_STOP), ==,
                    saved.saved_stop);
    socket_assert_empty(harness.backend_fd);

    tx_harness_stop(&harness);
}

typedef enum TxRejectionKind {
    TX_REJECT_NO_EOP,
    TX_REJECT_REVERSED,
    TX_REJECT_SUBTRACTION_UNDERFLOW,
    TX_REJECT_ADDRESS_WRAP,
    TX_REJECT_EXCESSIVE_LENGTH,
    TX_REJECT_DISABLED,
} TxRejectionKind;

typedef struct TxRejectionCase {
    const char *name;
    TxRejectionKind kind;
} TxRejectionCase;

static void tx_program_rejection(QTestState *qts, TxRejectionKind kind)
{
    qtest_writel(qts, NEXT_ENTX_NEXT, NEXT_TX_BUFFER);
    qtest_writel(qts, NEXT_ENTX_LIMIT,
                 ENTX_EOP | (NEXT_TX_BUFFER + 64 + ENTX_END_BIAS));
    qtest_writel(qts, NEXT_ENTX_START, NEXT_TX_BUFFER + 0x400);
    qtest_writel(qts, NEXT_ENTX_STOP, 0);
    qtest_writel(qts, NEXT_ENTX_NEXT_INIT, NEXT_TX_BUFFER);

    switch (kind) {
    case TX_REJECT_NO_EOP:
        qtest_writel(qts, NEXT_ENTX_LIMIT,
                     NEXT_TX_BUFFER + 64 + ENTX_END_BIAS);
        break;
    case TX_REJECT_REVERSED:
        qtest_writel(qts, NEXT_ENTX_LIMIT,
                     ENTX_EOP | (NEXT_TX_BUFFER + ENTX_END_BIAS - 1));
        break;
    case TX_REJECT_SUBTRACTION_UNDERFLOW:
        qtest_writel(qts, NEXT_ENTX_LIMIT, ENTX_EOP | 14);
        break;
    case TX_REJECT_ADDRESS_WRAP:
        qtest_writel(qts, NEXT_ENTX_NEXT_INIT, ENTX_ADDR_MASK + 1);
        qtest_writel(qts, NEXT_ENTX_LIMIT, ENTX_EOP | 79);
        break;
    case TX_REJECT_EXCESSIVE_LENGTH:
        qtest_writel(qts, NEXT_ENTX_LIMIT,
                     ENTX_EOP | (NEXT_TX_BUFFER + 1515 +
                                 ENTX_END_BIAS));
        break;
    case TX_REJECT_DISABLED:
        break;
    default:
        g_assert_not_reached();
    }
}

static void test_tx_range_rejection(void)
{
    static const TxRejectionCase cases[] = {
        { "no-eop", TX_REJECT_NO_EOP },
        { "reversed", TX_REJECT_REVERSED },
        { "subtraction-underflow", TX_REJECT_SUBTRACTION_UNDERFLOW },
        { "address-wrap", TX_REJECT_ADDRESS_WRAP },
        { "excessive-length", TX_REJECT_EXCESSIVE_LENGTH },
        { "disabled", TX_REJECT_DISABLED },
    };
    uint8_t before[1600];
    uint8_t after[1600];
    size_t case_index;

    memset(before, 0xa5, sizeof(before));
    for (case_index = 0; case_index < ARRAY_SIZE(cases); case_index++) {
        const TxRejectionCase *test = &cases[case_index];
        TxHarness harness;
        QTestState *qts;
        TxPointers expected;
        TxPointers actual;

        g_test_message("rejection case: %s", test->name);
        harness = tx_harness_start();
        qts = harness.qts;
        qtest_memwrite(qts, NEXT_TX_BUFFER, before, sizeof(before));
        tx_write_saved_sentinels(qts);
        tx_program_rejection(qts, test->kind);
        expected = tx_read_pointers(qts);
        tx_prepare_controller(qts);

        qtest_writel(qts, NEXT_ENTX_CSR, DMA_SETENABLE);
        if (test->kind == TX_REJECT_DISABLED) {
            qtest_writel(qts, NEXT_ENTX_CSR, DMA_RESET);
        }
        qtest_clock_step(qts, 1);

        socket_assert_empty(harness.backend_fd);
        qtest_memread(qts, NEXT_TX_BUFFER, after, sizeof(after));
        g_assert_cmpmem(after, sizeof(after), before, sizeof(before));
        actual = tx_read_pointers(qts);
        tx_assert_pointers_equal(&actual, &expected);
        g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_CSR) &
                        (DMA_ENABLE | DMA_SUPDATE |
                         DMA_COMPLETE | DMA_BUSEXC),
                        ==, DMA_COMPLETE | DMA_BUSEXC);
        g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                        ((1U << 26) | (1U << 27) | NEXT_ENTX_DMA_IRQ),
                        ==, NEXT_ENTX_DMA_IRQ);
        g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==,
                        EN_TXSTAT_UNDERFLOW);

        tx_harness_stop(&harness);
    }
}

static void test_tx_async(void)
{
    enum { FRAME_LENGTH = 60 };
    TxHarness harness = tx_harness_start();
    QTestState *qts = harness.qts;
    uint8_t frame[FRAME_LENGTH];
    uint8_t deferred_frame[FRAME_LENGTH];
    uint8_t received[FRAME_LENGTH];

    memset(frame, 0x6c, sizeof(frame));
    memset(deferred_frame, 0x7d, sizeof(deferred_frame));
    qtest_memwrite(qts, NEXT_TX_BUFFER, frame, sizeof(frame));
    qtest_writel(qts, NEXT_ENTX_NEXT, NEXT_TX_BUFFER);
    qtest_writel(qts, NEXT_ENTX_LIMIT,
                 ENTX_EOP | (NEXT_TX_BUFFER + FRAME_LENGTH +
                             ENTX_END_BIAS));
    tx_prepare_controller(qts);

    qtest_writel(qts, NEXT_ENTX_CSR, DMA_SETENABLE);
    socket_assert_empty(harness.backend_fd);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_CSR) &
                    (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_ENABLE);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_NEXT), ==, NEXT_TX_BUFFER);
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==, 0);
    qtest_memwrite(qts, NEXT_TX_BUFFER,
                   deferred_frame, sizeof(deferred_frame));

    qtest_clock_step(qts, 1);
    socket_read_frame(harness.backend_fd, received, sizeof(received));
    g_assert_cmpmem(received, sizeof(received),
                    deferred_frame, sizeof(deferred_frame));
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_CSR) & DMA_COMPLETE,
                    ==, DMA_COMPLETE);
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==, EN_TXSTAT_READY);

    tx_harness_stop(&harness);
}

static void test_tx_ack_isolation(void)
{
    enum { FRAME_LENGTH = 60 };
    TxHarness harness = tx_harness_start();
    QTestState *qts = harness.qts;
    uint8_t frame[FRAME_LENGTH];
    uint8_t received[FRAME_LENGTH];

    memset(frame, 0x93, sizeof(frame));
    qtest_memwrite(qts, NEXT_TX_BUFFER, frame, sizeof(frame));
    qtest_writel(qts, NEXT_ENTX_NEXT, NEXT_TX_BUFFER);
    qtest_writel(qts, NEXT_ENTX_LIMIT,
                 ENTX_EOP | (NEXT_TX_BUFFER + FRAME_LENGTH +
                             ENTX_END_BIAS));
    tx_prepare_controller(qts);
    en_writeb(qts, EN_TXMASK, EN_TXSTAT_READY);

    qtest_writel(qts, NEXT_ENTX_CSR, DMA_SETENABLE);
    qtest_clock_step(qts, 1);
    socket_read_frame(harness.backend_fd, received, sizeof(received));
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==, EN_TXSTAT_READY);
    g_assert_cmphex(controller_irqs(qts), ==, NEXT_ENTX_IRQ);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_CSR) & DMA_COMPLETE,
                    ==, DMA_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    NEXT_ENTX_DMA_IRQ, ==, NEXT_ENTX_DMA_IRQ);

    en_writeb(qts, EN_TXSTAT, EN_TXSTAT_READY);
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==, 0);
    g_assert_cmphex(controller_irqs(qts), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_CSR) & DMA_COMPLETE,
                    ==, DMA_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    NEXT_ENTX_DMA_IRQ, ==, NEXT_ENTX_DMA_IRQ);

    en_writeb(qts, EN_RESET, EN_RESET_MODE);
    en_writeb(qts, EN_RESET, 0);
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==, EN_TXSTAT_READY);
    qtest_writel(qts, NEXT_ENTX_CSR, DMA_CLRCOMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENTX_CSR) & DMA_COMPLETE, ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    NEXT_ENTX_DMA_IRQ, ==, 0);
    g_assert_cmphex(en_readb(qts, EN_TXSTAT), ==, EN_TXSTAT_READY);

    tx_harness_stop(&harness);
}

static void test_tx_reply_waits_for_rearmed_rx(void)
{
    QTestState *qts = next_mb8795_start_with_args(
        "-netdev user,id=nextnet "
        "-net nic,model=next-mb8795,netdev=nextnet,"
        "macaddr=52:54:00:12:34:56");
    uint8_t stale_before[sizeof(arp_request) + sizeof(rx_fcs)];
    uint8_t stale_after[sizeof(stale_before)];
    uint8_t reply[sizeof(stale_before)];

    memset(stale_before, 0xa5, sizeof(stale_before));
    memset(reply, 0xa5, sizeof(reply));
    qtest_memwrite(qts, NEXT_RX_BUFFER,
                   stale_before, sizeof(stale_before));
    qtest_memwrite(qts, NEXT_RX_BUFFER_2, reply, sizeof(reply));
    qtest_memwrite(qts, NEXT_TX_BUFFER,
                   arp_request, sizeof(arp_request));

    rx_prepare_controller(qts, 3);
    rx_program(qts, NEXT_RX_BUFFER, NEXT_RX_BUFFER + 0x1000,
               0, 0, false);
    tx_prepare_controller(qts);
    qtest_writel(qts, NEXT_ENTX_NEXT, NEXT_TX_BUFFER);
    qtest_writel(qts, NEXT_ENTX_LIMIT,
                 ENTX_EOP | (NEXT_TX_BUFFER + sizeof(arp_request) +
                             ENTX_END_BIAS));

    qtest_writel(qts, NEXT_ENTX_CSR, DMA_SETENABLE);
    qtest_clock_step(qts, 1);

    qtest_memread(qts, NEXT_RX_BUFFER,
                  stale_after, sizeof(stale_after));
    g_assert_cmpmem(stale_after, sizeof(stale_after),
                    stale_before, sizeof(stale_before));
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_CSR) &
                    (DMA_ENABLE | DMA_COMPLETE), ==, DMA_ENABLE);

    rx_program(qts, NEXT_RX_BUFFER_2, NEXT_RX_BUFFER_2 + 0x1000,
               0, 0, false);
    qtest_clock_step(qts, 2 * 1000 * 1000);

    qtest_memread(qts, NEXT_RX_BUFFER_2, reply, sizeof(reply));
    g_assert_cmphex(reply[12], ==, 0x08);
    g_assert_cmphex(reply[13], ==, 0x06);
    g_assert_cmphex(reply[20], ==, 0x00);
    g_assert_cmphex(reply[21], ==, 0x02);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_SAVED_NEXT), ==,
                    NEXT_RX_BUFFER_2);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_CSR) &
                    (DMA_ENABLE | DMA_COMPLETE), ==, DMA_COMPLETE);

    qtest_quit(qts);
}

static void test_rx_fcs_single_buffer(void)
{
    TxHarness harness = tx_harness_start();
    QTestState *qts = harness.qts;
    uint8_t received[sizeof(rx_frame) + sizeof(rx_fcs)];

    memset(received, 0xa5, sizeof(received));
    qtest_memwrite(qts, NEXT_RX_BUFFER, received, sizeof(received));
    rx_prepare_controller(qts, 3);
    rx_program(qts, NEXT_RX_BUFFER, NEXT_RX_BUFFER + 0x1000, 0, 0,
               false);

    socket_write_frame(harness.backend_fd, rx_frame, sizeof(rx_frame));
    qtest_clock_step(qts, 1);
    qtest_memread(qts, NEXT_RX_BUFFER, received, sizeof(received));

    g_assert_cmpmem(received, sizeof(rx_frame),
                    rx_frame, sizeof(rx_frame));
    g_assert_cmpmem(received + sizeof(rx_frame), sizeof(rx_fcs),
                    rx_fcs, sizeof(rx_fcs));
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_SAVED_NEXT), ==,
                    NEXT_RX_BUFFER);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_SAVED_LIMIT), ==,
                    NEXT_RX_BUFFER + sizeof(received));
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_NEXT), ==,
                    NEXT_RX_BUFFER + sizeof(received));
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE |
                     DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_COMPLETE);
    g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==, EN_RXSTAT_OK);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    NEXT_ENRX_DMA_IRQ, ==, NEXT_ENRX_DMA_IRQ);

    tx_harness_stop(&harness);
}

static void rx_filter_case(uint8_t mode, const uint8_t destination[6],
                           const uint8_t station[6], bool accepted)
{
    TxHarness harness = tx_harness_start();
    QTestState *qts = harness.qts;
    uint8_t frame[sizeof(rx_frame)];
    uint8_t before[sizeof(rx_frame) + sizeof(rx_fcs)];
    uint8_t after[sizeof(before)];
    RxPointers expected;
    RxPointers actual;
    size_t i;

    memcpy(frame, rx_frame, sizeof(frame));
    memcpy(frame, destination, 6);
    memset(before, 0xa5, sizeof(before));
    qtest_memwrite(qts, NEXT_RX_BUFFER, before, sizeof(before));
    for (i = 0; i < 6; i++) {
        en_writeb(qts, EN_ADDR + i, station[i]);
    }
    rx_prepare_controller(qts, mode);
    rx_program(qts, NEXT_RX_BUFFER, NEXT_RX_BUFFER + 0x1000, 0, 0,
               false);
    expected = rx_read_pointers(qts);

    socket_write_frame(harness.backend_fd, frame, sizeof(frame));
    qtest_clock_step(qts, 1);
    qtest_memread(qts, NEXT_RX_BUFFER, after, sizeof(after));

    if (accepted) {
        g_assert_cmpmem(after, sizeof(frame), frame, sizeof(frame));
        g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==, EN_RXSTAT_OK);
        g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_CSR) & DMA_COMPLETE,
                        ==, DMA_COMPLETE);
    } else {
        g_assert_cmpmem(after, sizeof(after), before, sizeof(before));
        actual = rx_read_pointers(qts);
        rx_assert_pointers_equal(&actual, &expected);
        g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==, 0);
        g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_CSR) &
                        (DMA_ENABLE | DMA_COMPLETE | DMA_BUSEXC),
                        ==, DMA_ENABLE);
    }

    tx_harness_stop(&harness);
}

static void test_rx_filter_modes(void)
{
    static const uint8_t station[6] = {
        0x52, 0x54, 0x00, 0x12, 0x34, 0x56,
    };
    static const uint8_t station_unicast[6] = {
        0x52, 0x54, 0x00, 0x12, 0x34, 0x56,
    };
    static const uint8_t broadcast[6] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    };
    static const uint8_t multicast[6] = {
        0x01, 0x00, 0x5e, 0x11, 0x22, 0x33,
    };
    static const uint8_t other_unicast[6] = {
        0x52, 0x54, 0x00, 0xaa, 0xbb, 0xcc,
    };
    static const uint8_t limited_station[6] = {
        0x53, 0x54, 0x00, 0x12, 0x34, 0x56,
    };
    static const uint8_t limited_multicast[6] = {
        0x53, 0x54, 0x00, 0xaa, 0xbb, 0xcc,
    };

    rx_filter_case(0, station_unicast, station, false);
    rx_filter_case(1, station_unicast, station, true);
    rx_filter_case(1, broadcast, station, true);
    rx_filter_case(1, limited_multicast, limited_station, true);
    rx_filter_case(1, multicast, station, false);
    rx_filter_case(2, multicast, station, true);
    rx_filter_case(2, other_unicast, station, false);
    rx_filter_case(3, other_unicast, station, true);
    rx_filter_case(0x83, station_unicast, station, false);
    rx_filter_case(0x07, station_unicast, station, false);
}

static void test_rx_addrsize(void)
{
    static const uint8_t station[6] = {
        0x52, 0x54, 0x00, 0x12, 0x34, 0x56,
    };
    static const uint8_t differing_last[6] = {
        0x52, 0x54, 0x00, 0x12, 0x34, 0xa5,
    };

    rx_filter_case(1, differing_last, station, false);
    rx_filter_case(0x11, differing_last, station, true);
}

static void test_rx_crosses_chain(void)
{
    enum { FIRST_LENGTH = 20 };
    const uint32_t second = NEXT_RX_BUFFER + 0x200;
    const size_t wire_length = sizeof(rx_frame) + sizeof(rx_fcs);
    TxHarness harness = tx_harness_start();
    QTestState *qts = harness.qts;
    uint8_t first[FIRST_LENGTH];
    uint8_t second_data[sizeof(rx_frame) + sizeof(rx_fcs) - FIRST_LENGTH];
    uint8_t expected[sizeof(rx_frame) + sizeof(rx_fcs)];

    memcpy(expected, rx_frame, sizeof(rx_frame));
    memcpy(expected + sizeof(rx_frame), rx_fcs, sizeof(rx_fcs));
    rx_prepare_controller(qts, 3);
    rx_program(qts, NEXT_RX_BUFFER, NEXT_RX_BUFFER + FIRST_LENGTH,
               second, second + 0x100, true);

    socket_write_frame(harness.backend_fd, rx_frame, sizeof(rx_frame));
    qtest_clock_step(qts, 1);
    qtest_memread(qts, NEXT_RX_BUFFER, first, sizeof(first));
    qtest_memread(qts, second, second_data, sizeof(second_data));

    g_assert_cmpmem(first, sizeof(first), expected, sizeof(first));
    g_assert_cmpmem(second_data, sizeof(second_data),
                    expected + sizeof(first), sizeof(second_data));
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_SAVED_NEXT), ==,
                    NEXT_RX_BUFFER);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_SAVED_LIMIT), ==,
                    NEXT_RX_BUFFER + FIRST_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_NEXT), ==,
                    second + wire_length - FIRST_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_LIMIT), ==,
                    second + 0x100);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE |
                     DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_COMPLETE);

    tx_harness_stop(&harness);
}

static void test_rx_first_buffer_supdate(void)
{
    const uint32_t second = NEXT_RX_BUFFER + 0x200;
    const size_t wire_length = sizeof(rx_frame) + sizeof(rx_fcs);
    TxHarness harness = tx_harness_start();
    QTestState *qts = harness.qts;
    uint8_t received[sizeof(rx_frame) + sizeof(rx_fcs)];

    rx_prepare_controller(qts, 3);
    rx_program(qts, NEXT_RX_BUFFER, NEXT_RX_BUFFER + 0x100,
               second, second + 0x100, true);
    socket_write_frame(harness.backend_fd, rx_frame, sizeof(rx_frame));
    qtest_clock_step(qts, 1);
    qtest_memread(qts, NEXT_RX_BUFFER, received, sizeof(received));

    g_assert_cmpmem(received, sizeof(rx_frame),
                    rx_frame, sizeof(rx_frame));
    g_assert_cmpmem(received + sizeof(rx_frame), sizeof(rx_fcs),
                    rx_fcs, sizeof(rx_fcs));
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_SAVED_NEXT), ==,
                    NEXT_RX_BUFFER);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_SAVED_LIMIT), ==,
                    NEXT_RX_BUFFER + wire_length);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_NEXT), ==, second);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_LIMIT), ==,
                    second + 0x100);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE |
                     DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_ENABLE | DMA_COMPLETE);

    tx_harness_stop(&harness);
}

static void test_rx_overflow(void)
{
    TxHarness harness = tx_harness_start();
    QTestState *qts = harness.qts;
    uint8_t before[sizeof(rx_frame) + sizeof(rx_fcs)];
    uint8_t after[sizeof(before)];
    RxPointers expected;
    RxPointers actual;

    memset(before, 0xa5, sizeof(before));
    qtest_memwrite(qts, NEXT_RX_BUFFER, before, sizeof(before));
    rx_prepare_controller(qts, 3);
    rx_program(qts, NEXT_RX_BUFFER, NEXT_RX_BUFFER + 32, 0, 0, false);
    expected = rx_read_pointers(qts);

    socket_write_frame(harness.backend_fd, rx_frame, sizeof(rx_frame));
    qtest_clock_step(qts, 1);

    qtest_memread(qts, NEXT_RX_BUFFER, after, sizeof(after));
    g_assert_cmpmem(after, sizeof(after), before, sizeof(before));
    actual = rx_read_pointers(qts);
    rx_assert_pointers_equal(&actual, &expected);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_CSR) &
                    (DMA_ENABLE | DMA_SUPDATE |
                     DMA_COMPLETE | DMA_BUSEXC),
                    ==, DMA_COMPLETE | DMA_BUSEXC);
    g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==,
                    EN_RXSTAT_OVERFLOW);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    ((1U << 26) | NEXT_ENRX_DMA_IRQ |
                     NEXT_ENTX_DMA_IRQ),
                    ==, NEXT_ENRX_DMA_IRQ);

    tx_harness_stop(&harness);
}

static void test_rx_backpressure_flush(void)
{
    {
        TxHarness harness = tx_harness_start();
        QTestState *qts = harness.qts;
        const uint32_t second_buffer = NEXT_RX_BUFFER + 0x1000;
        const uint8_t second_fcs[4] = { 0x58, 0x52, 0x92, 0xee };
        uint8_t second_frame[sizeof(rx_frame)];
        uint8_t before[sizeof(rx_frame) + sizeof(rx_fcs)];
        uint8_t after[sizeof(before)];

        memcpy(second_frame, rx_frame, sizeof(second_frame));
        second_frame[sizeof(second_frame) - 1] ^= 0xff;
        memset(before, 0xa5, sizeof(before));
        qtest_memwrite(qts, NEXT_RX_BUFFER, before, sizeof(before));
        qtest_memwrite(qts, second_buffer, before, sizeof(before));
        qtest_writel(qts, NEXT_ENRX_NEXT, NEXT_RX_BUFFER);
        qtest_writel(qts, NEXT_ENRX_LIMIT, NEXT_RX_BUFFER + 0x1000);
        rx_prepare_controller(qts, 3);

        socket_write_frame(harness.backend_fd,
                           rx_frame, sizeof(rx_frame));
        socket_write_frame(harness.backend_fd,
                           second_frame, sizeof(second_frame));
        qtest_clock_step(qts, 1);
        qtest_memread(qts, NEXT_RX_BUFFER, after, sizeof(after));
        g_assert_cmpmem(after, sizeof(after), before, sizeof(before));
        qtest_memread(qts, second_buffer, after, sizeof(after));
        g_assert_cmpmem(after, sizeof(after), before, sizeof(before));
        g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==, 0);

        qtest_writel(qts, NEXT_ENRX_CSR, DMA_SETENABLE);
        qtest_clock_step(qts, 1);
        qtest_memread(qts, NEXT_RX_BUFFER, after, sizeof(after));
        g_assert_cmpmem(after, sizeof(rx_frame),
                        rx_frame, sizeof(rx_frame));
        g_assert_cmpmem(after + sizeof(rx_frame), sizeof(rx_fcs),
                        rx_fcs, sizeof(rx_fcs));
        g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==, EN_RXSTAT_OK);
        g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_CSR) &
                        (DMA_COMPLETE | DMA_BUSEXC), ==, DMA_COMPLETE);

        en_writeb(qts, EN_RXSTAT, EN_RXSTAT_OK);
        rx_program(qts, second_buffer, second_buffer + 0x1000, 0, 0,
                   false);
        qtest_clock_step(qts, 1);
        qtest_memread(qts, second_buffer, after, sizeof(after));
        g_assert_cmpmem(after, sizeof(second_frame),
                        second_frame, sizeof(second_frame));
        g_assert_cmpmem(after + sizeof(second_frame), sizeof(second_fcs),
                        second_fcs, sizeof(second_fcs));
        g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==, EN_RXSTAT_OK);
        g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_CSR) &
                        (DMA_COMPLETE | DMA_BUSEXC), ==, DMA_COMPLETE);

        tx_harness_stop(&harness);
    }

    {
        TxHarness harness = tx_harness_start();
        QTestState *qts = harness.qts;
        uint8_t before[sizeof(rx_frame) + sizeof(rx_fcs)];
        uint8_t after[sizeof(before)];
        RxPointers expected;
        RxPointers actual;

        memset(before, 0xa5, sizeof(before));
        qtest_memwrite(qts, NEXT_RX_BUFFER, before, sizeof(before));
        rx_prepare_controller(qts, 3);
        rx_program(qts, 0x03000000, 0x03000100, 0, 0, false);
        expected = rx_read_pointers(qts);

        socket_write_frame(harness.backend_fd,
                           rx_frame, sizeof(rx_frame));
        qtest_clock_step(qts, 1);
        qtest_memread(qts, NEXT_RX_BUFFER, after, sizeof(after));
        g_assert_cmpmem(after, sizeof(after), before, sizeof(before));
        actual = rx_read_pointers(qts);
        rx_assert_pointers_equal(&actual, &expected);
        g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_CSR) &
                        (DMA_ENABLE | DMA_SUPDATE |
                         DMA_COMPLETE | DMA_BUSEXC),
                        ==, DMA_ENABLE);
        g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==, 0);
        g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                        NEXT_ENRX_DMA_IRQ, ==, 0);

        qtest_writel(qts, NEXT_ENRX_NEXT, NEXT_RX_BUFFER);
        qtest_memread(qts, NEXT_RX_BUFFER, after, sizeof(after));
        g_assert_cmpmem(after, sizeof(after), before, sizeof(before));
        g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==, 0);

        qtest_writel(qts, NEXT_ENRX_LIMIT, NEXT_RX_BUFFER + 0x1000);
        qtest_clock_step(qts, 1);
        qtest_memread(qts, NEXT_RX_BUFFER, after, sizeof(after));
        g_assert_cmpmem(after, sizeof(rx_frame),
                        rx_frame, sizeof(rx_frame));
        g_assert_cmpmem(after + sizeof(rx_frame), sizeof(rx_fcs),
                        rx_fcs, sizeof(rx_fcs));
        g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==, EN_RXSTAT_OK);
        g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_CSR) &
                        (DMA_ENABLE | DMA_SUPDATE |
                         DMA_COMPLETE | DMA_BUSEXC),
                        ==, DMA_COMPLETE);

        tx_harness_stop(&harness);
    }
}

static void test_rx_ack_isolation(void)
{
    const uint32_t second = NEXT_RX_BUFFER + 0x200;
    TxHarness harness = tx_harness_start();
    QTestState *qts = harness.qts;

    rx_prepare_controller(qts, 3);
    en_writeb(qts, EN_RXMASK, EN_RXSTAT_OK);
    rx_program(qts, NEXT_RX_BUFFER, NEXT_RX_BUFFER + 0x1000, 0, 0,
               false);
    socket_write_frame(harness.backend_fd, rx_frame, sizeof(rx_frame));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(controller_irqs(qts), ==, NEXT_ENRX_IRQ);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    NEXT_ENRX_DMA_IRQ, ==, NEXT_ENRX_DMA_IRQ);

    en_writeb(qts, EN_RXSTAT, EN_RXSTAT_OK);
    g_assert_cmphex(controller_irqs(qts), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_ENRX_CSR) & DMA_COMPLETE,
                    ==, DMA_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    NEXT_ENRX_DMA_IRQ, ==, NEXT_ENRX_DMA_IRQ);
    qtest_writel(qts, NEXT_ENRX_CSR, DMA_CLRCOMPLETE);

    rx_program(qts, second, second + 0x1000, 0, 0, false);
    socket_write_frame(harness.backend_fd, rx_frame, sizeof(rx_frame));
    qtest_clock_step(qts, 1);
    g_assert_cmphex(controller_irqs(qts), ==, NEXT_ENRX_IRQ);
    g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==, EN_RXSTAT_OK);
    qtest_writel(qts, NEXT_ENRX_CSR, DMA_CLRCOMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    NEXT_ENRX_DMA_IRQ, ==, 0);
    g_assert_cmphex(en_readb(qts, EN_RXSTAT), ==, EN_RXSTAT_OK);
    g_assert_cmphex(controller_irqs(qts), ==, NEXT_ENRX_IRQ);

    en_writeb(qts, EN_RXSTAT, EN_RXSTAT_OK);
    g_assert_cmphex(controller_irqs(qts), ==, 0);
    tx_harness_stop(&harness);
}
#else
static void test_tx_transport_unavailable(void)
{
    g_test_skip("socket-backed Ethernet transport is unavailable on Windows");
}
#endif

static void test_migration_register_state(void)
{
    static const uint8_t station[6] = {
        0x00, 0x00, 0x0f, 0x12, 0x34, 0x56,
    };
    g_autoptr(GError) error = NULL;
    g_autofree char *tmpdir = NULL;
    g_autofree char *socket_path = NULL;
    g_autofree char *uri = NULL;
    g_autofree char *quoted_uri = NULL;
    g_autofree char *incoming_args = NULL;
    QTestState *source;
    QTestState *destination;
    size_t i;

    tmpdir = g_dir_make_tmp("next-mb8795-migration-XXXXXX", &error);
    g_assert_no_error(error);
    g_assert_nonnull(tmpdir);
    socket_path = g_build_filename(tmpdir, "migration.sock", NULL);
    uri = g_strdup_printf("unix:%s", socket_path);
    quoted_uri = g_shell_quote(uri);
    incoming_args = g_strdup_printf("-incoming %s", quoted_uri);

    destination = next_mb8795_start_with_args(incoming_args);
    source = next_mb8795_start();
    unrealize_next_kbd(source);
    unrealize_next_kbd(destination);

    en_writeb(source, EN_RESET, 0);
    en_writeb(source, EN_TXMASK, EN_TXSTAT_READY | EN_TXSTAT_UNDERFLOW);
    en_writeb(source, EN_RXMASK, EN_RXSTAT_OK | EN_RXSTAT_OVERFLOW);
    en_writeb(source, EN_TXMODE, 0xa5);
    en_writeb(source, EN_RXMODE, 0x5a);
    for (i = 0; i < ARRAY_SIZE(station); i++) {
        en_writeb(source, EN_ADDR + i, station[i]);
    }
    g_assert_cmphex(controller_irqs(source), ==, NEXT_ENTX_IRQ);

    /*
     * Migrate stale cleared board derived state.  The controller output is
     * still asserted, so destination post-load must reconstruct bit 10.
     */
    qtest_writel(source, NEXT_INTR_STATUS, 0);
    g_assert_cmphex(controller_irqs(source), ==, 0);

    migrate_wait(source, destination, uri);

    g_assert_cmphex(en_readb(destination, EN_TXSTAT), ==,
                    EN_TXSTAT_READY);
    g_assert_cmphex(en_readb(destination, EN_TXMASK), ==,
                    EN_TXSTAT_READY | EN_TXSTAT_UNDERFLOW);
    g_assert_cmphex(en_readb(destination, EN_RXSTAT), ==, 0);
    g_assert_cmphex(en_readb(destination, EN_RXMASK), ==,
                    EN_RXSTAT_OK | EN_RXSTAT_OVERFLOW);
    g_assert_cmphex(en_readb(destination, EN_TXMODE), ==, 0xa5);
    g_assert_cmphex(en_readb(destination, EN_RXMODE), ==, 0x5a);
    g_assert_cmphex(en_readb(destination, EN_RESET), ==, 0);
    for (i = 0; i < ARRAY_SIZE(station); i++) {
        g_assert_cmphex(en_readb(destination, EN_ADDR + i), ==, station[i]);
    }
    g_assert_cmphex(controller_irqs(destination), ==, NEXT_ENTX_IRQ);

    qtest_quit(source);
    qtest_quit(destination);
    g_unlink(socket_path);
    g_assert_cmpint(g_rmdir(tmpdir), ==, 0);
}

static void test_migration_reset_programming(void)
{
    static const uint8_t station[6] = {
        0x00, 0x00, 0x0f, 0xa5, 0x5a, 0xc3,
    };
    g_autoptr(GError) error = NULL;
    g_autofree char *tmpdir = NULL;
    g_autofree char *socket_path = NULL;
    g_autofree char *uri = NULL;
    g_autofree char *quoted_uri = NULL;
    g_autofree char *incoming_args = NULL;
    QTestState *source;
    QTestState *destination;
    size_t i;

    tmpdir = g_dir_make_tmp("next-mb8795-reset-migration-XXXXXX", &error);
    g_assert_no_error(error);
    g_assert_nonnull(tmpdir);
    socket_path = g_build_filename(tmpdir, "migration.sock", NULL);
    uri = g_strdup_printf("unix:%s", socket_path);
    quoted_uri = g_shell_quote(uri);
    incoming_args = g_strdup_printf("-incoming %s", quoted_uri);

    destination = next_mb8795_start_with_args(incoming_args);
    source = next_mb8795_start();
    unrealize_next_kbd(source);
    unrealize_next_kbd(destination);

    g_assert_cmphex(en_readb(source, EN_RESET), ==, EN_RESET_MODE);
    en_writeb(source, EN_TXMASK, EN_TXSTAT_READY | EN_TXSTAT_UNDERFLOW);
    en_writeb(source, EN_RXMASK, EN_RXSTAT_OK | EN_RXSTAT_OVERFLOW);
    en_writeb(source, EN_TXMODE, 0xa5);
    en_writeb(source, EN_RXMODE, 0x5a);
    for (i = 0; i < ARRAY_SIZE(station); i++) {
        en_writeb(source, EN_ADDR + i, station[i]);
    }
    g_assert_cmphex(en_readb(source, EN_RESET), ==, EN_RESET_MODE);
    g_assert_cmphex(en_readb(source, EN_TXSTAT), ==, 0);
    g_assert_cmphex(en_readb(source, EN_TXMASK), ==,
                    EN_TXSTAT_READY | EN_TXSTAT_UNDERFLOW);
    g_assert_cmphex(en_readb(source, EN_RXSTAT), ==, 0);
    g_assert_cmphex(en_readb(source, EN_RXMASK), ==,
                    EN_RXSTAT_OK | EN_RXSTAT_OVERFLOW);
    g_assert_cmphex(en_readb(source, EN_TXMODE), ==, 0xa5);
    g_assert_cmphex(en_readb(source, EN_RXMODE), ==, 0x5a);
    for (i = 0; i < ARRAY_SIZE(station); i++) {
        g_assert_cmphex(en_readb(source, EN_ADDR + i), ==, station[i]);
    }
    g_assert_cmphex(controller_irqs(source), ==, 0);

    migrate_wait(source, destination, uri);

    g_assert_cmphex(en_readb(destination, EN_RESET), ==, EN_RESET_MODE);
    g_assert_cmphex(en_readb(destination, EN_TXSTAT), ==, 0);
    g_assert_cmphex(en_readb(destination, EN_TXMASK), ==,
                    EN_TXSTAT_READY | EN_TXSTAT_UNDERFLOW);
    g_assert_cmphex(en_readb(destination, EN_RXSTAT), ==, 0);
    g_assert_cmphex(en_readb(destination, EN_RXMASK), ==,
                    EN_RXSTAT_OK | EN_RXSTAT_OVERFLOW);
    g_assert_cmphex(en_readb(destination, EN_TXMODE), ==, 0xa5);
    g_assert_cmphex(en_readb(destination, EN_RXMODE), ==, 0x5a);
    for (i = 0; i < ARRAY_SIZE(station); i++) {
        g_assert_cmphex(en_readb(destination, EN_ADDR + i), ==, station[i]);
    }
    g_assert_cmphex(controller_irqs(destination), ==, 0);

    qtest_quit(source);
    qtest_quit(destination);
    g_unlink(socket_path);
    g_assert_cmpint(g_rmdir(tmpdir), ==, 0);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/next-cube/mb8795/register-reset",
                   test_register_reset);
    qtest_add_func("/next-cube/mb8795/station-address",
                   test_station_address);
    qtest_add_func("/next-cube/mb8795/access-widths",
                   test_access_widths);
    qtest_add_func("/next-cube/mb8795/status-w1c",
                   test_status_w1c);
    qtest_add_func("/next-cube/mb8795/dma-next-init-read-alias",
                   test_dma_next_init_read_alias);
    qtest_add_func("/next-cube/mb8795/mask-status-irqs",
                   test_mask_status_irqs);
    qtest_add_func("/next-cube/mb8795/mtree-window",
                   test_mtree_window);
#ifndef _WIN32
    qtest_add_func("/next-cube/mb8795/tx-single-buffer",
                   test_tx_single_buffer);
    qtest_add_func("/next-cube/mb8795/tx-internal-loopback",
                   test_tx_internal_loopback);
    qtest_add_func("/next-cube/mb8795/tx-internal-loopback-filtered",
                   test_tx_internal_loopback_filtered);
    qtest_add_func("/next-cube/mb8795/tx-internal-loopback-rx-failure",
                   test_tx_internal_loopback_rx_failure);
    qtest_add_func("/next-cube/mb8795/tx-two-segment",
                   test_tx_two_segment);
    qtest_add_func("/next-cube/mb8795/tx-range-rejection",
                   test_tx_range_rejection);
    qtest_add_func("/next-cube/mb8795/tx-async",
                   test_tx_async);
    qtest_add_func("/next-cube/mb8795/tx-ack-isolation",
                   test_tx_ack_isolation);
    qtest_add_func("/next-cube/mb8795/tx-reply-waits-for-rearmed-rx",
                   test_tx_reply_waits_for_rearmed_rx);
    qtest_add_func("/next-cube/mb8795/rx-filter-modes",
                   test_rx_filter_modes);
    qtest_add_func("/next-cube/mb8795/rx-addrsize",
                   test_rx_addrsize);
    qtest_add_func("/next-cube/mb8795/rx-fcs-single-buffer",
                   test_rx_fcs_single_buffer);
    qtest_add_func("/next-cube/mb8795/rx-crosses-chain",
                   test_rx_crosses_chain);
    qtest_add_func("/next-cube/mb8795/rx-first-buffer-supdate",
                   test_rx_first_buffer_supdate);
    qtest_add_func("/next-cube/mb8795/rx-overflow",
                   test_rx_overflow);
    qtest_add_func("/next-cube/mb8795/rx-backpressure-flush",
                   test_rx_backpressure_flush);
    qtest_add_func("/next-cube/mb8795/rx-ack-isolation",
                   test_rx_ack_isolation);
#else
    qtest_add_func("/next-cube/mb8795/tx-single-buffer",
                   test_tx_transport_unavailable);
    qtest_add_func("/next-cube/mb8795/tx-internal-loopback",
                   test_tx_transport_unavailable);
    qtest_add_func("/next-cube/mb8795/tx-internal-loopback-filtered",
                   test_tx_transport_unavailable);
    qtest_add_func("/next-cube/mb8795/tx-internal-loopback-rx-failure",
                   test_tx_transport_unavailable);
    qtest_add_func("/next-cube/mb8795/tx-two-segment",
                   test_tx_transport_unavailable);
    qtest_add_func("/next-cube/mb8795/tx-range-rejection",
                   test_tx_transport_unavailable);
    qtest_add_func("/next-cube/mb8795/tx-async",
                   test_tx_transport_unavailable);
    qtest_add_func("/next-cube/mb8795/tx-ack-isolation",
                   test_tx_transport_unavailable);
    qtest_add_func("/next-cube/mb8795/tx-reply-waits-for-rearmed-rx",
                   test_tx_transport_unavailable);
    qtest_add_func("/next-cube/mb8795/rx-filter-modes",
                   test_tx_transport_unavailable);
    qtest_add_func("/next-cube/mb8795/rx-addrsize",
                   test_tx_transport_unavailable);
    qtest_add_func("/next-cube/mb8795/rx-fcs-single-buffer",
                   test_tx_transport_unavailable);
    qtest_add_func("/next-cube/mb8795/rx-crosses-chain",
                   test_tx_transport_unavailable);
    qtest_add_func("/next-cube/mb8795/rx-first-buffer-supdate",
                   test_tx_transport_unavailable);
    qtest_add_func("/next-cube/mb8795/rx-overflow",
                   test_tx_transport_unavailable);
    qtest_add_func("/next-cube/mb8795/rx-backpressure-flush",
                   test_tx_transport_unavailable);
    qtest_add_func("/next-cube/mb8795/rx-ack-isolation",
                   test_tx_transport_unavailable);
#endif
    qtest_add_func("/next-cube/mb8795/migration-register-state",
                   test_migration_register_state);
    qtest_add_func("/next-cube/mb8795/migration-reset-programming",
                   test_migration_reset_programming);
    return g_test_run();
}
