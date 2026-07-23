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
#define NEXT_INTR_STATUS 0x02007000
#define NEXT_ROM_SIZE    (128 * 1024)
#define NEXT_ENRX_IRQ    (1U << 9)
#define NEXT_ENTX_IRQ    (1U << 10)

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
#define EN_TXSTAT_UNDERFLOW 0x08
#define EN_RXSTAT_OK        0x80
#define EN_RXSTAT_OVERFLOW  0x01
#define EN_RESET_MODE       0x80

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
    qtest_add_func("/next-cube/mb8795/mask-status-irqs",
                   test_mask_status_irqs);
    qtest_add_func("/next-cube/mb8795/mtree-window",
                   test_mtree_window);
    qtest_add_func("/next-cube/mb8795/migration-register-state",
                   test_migration_register_state);
    qtest_add_func("/next-cube/mb8795/migration-reset-programming",
                   test_migration_reset_programming);
    return g_test_run();
}
