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

#define NEXT_INTR_STATUS 0x02007000
#define NEXT_SCC_BASE    0x02118000
#define NEXT_SCC_CLOCK   (NEXT_SCC_BASE + 4)
#define NEXT_INTR_SCC    (1U << 17)
#define NEXT_ROM_SIZE    (128 * 1024)

#define NEXT_SCC_B_CTRL  (NEXT_SCC_BASE + 0)
#define NEXT_SCC_A_CTRL  (NEXT_SCC_BASE + 1)
#define NEXT_SCC_B_DATA  (NEXT_SCC_BASE + 2)
#define NEXT_SCC_A_DATA  (NEXT_SCC_BASE + 3)

#define SCC_RR0_RXAVAIL  0x01

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

static char *next_cube_serial_args(void)
{
    TestROM *rom = g_new0(TestROM, 1);
    g_autofree char *quoted_rom_path = NULL;

    rom->fd = -1;
    qtest_add_abrt_handler(cleanup_test_rom, rom);
    g_test_queue_destroy(cleanup_test_rom, rom);

    rom->fd = g_file_open_tmp("next-serial-rom-XXXXXX", &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;

    quoted_rom_path = g_shell_quote(rom->path);
    return g_strdup_printf("-machine next-cube -bios %s", quoted_rom_path);
}

static QTestState *next_cube_serial_start(void)
{
    g_autofree char *args = next_cube_serial_args();

    return qtest_init(args);
}

static QTestState *next_cube_serial_start_with_backend(int *sock_fd)
{
    g_autofree char *args = next_cube_serial_args();

    return qtest_init_with_serial(args, sock_fd);
}

static void scc_write_reg(QTestState *qts, uint64_t control,
                          uint8_t reg, uint8_t value)
{
    qtest_writeb(qts, control, reg);
    qtest_writeb(qts, control, value);
}

static void scc_configure_rx_tx(QTestState *qts, uint64_t control,
                                bool loopback)
{
    scc_write_reg(qts, control, 1, 0x10);
    scc_write_reg(qts, control, 3, 0x01);
    scc_write_reg(qts, control, 5, 0x68);
    if (loopback) {
        scc_write_reg(qts, control, 14, 0x10);
    }
}

static void assert_scc_irq(QTestState *qts, bool level)
{
    uint32_t status = qtest_readl(qts, NEXT_INTR_STATUS);

    g_assert_cmphex(status & NEXT_INTR_SCC, ==,
                    level ? NEXT_INTR_SCC : 0);
}

static void test_clock_select(void)
{
    QTestState *qts = next_cube_serial_start();

    g_assert_cmphex(qtest_readb(qts, NEXT_SCC_CLOCK), ==, 0);
    qtest_writeb(qts, NEXT_SCC_CLOCK, 0x0a);
    g_assert_cmphex(qtest_readb(qts, NEXT_SCC_CLOCK), ==, 0x0a);

    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readb(qts, NEXT_SCC_CLOCK), ==, 0);

    qtest_quit(qts);
}

static void test_local_loopback(uint64_t control, uint64_t data)
{
    QTestState *qts = next_cube_serial_start();

    scc_configure_rx_tx(qts, control, true);
    assert_scc_irq(qts, false);

    qtest_writeb(qts, data, 'N');
    assert_scc_irq(qts, true);
    g_assert_cmphex(qtest_readb(qts, data), ==, 'N');
    assert_scc_irq(qts, false);

    qtest_quit(qts);
}

static void test_channel_b_loopback(void)
{
    test_local_loopback(NEXT_SCC_B_CTRL, NEXT_SCC_B_DATA);
}

static void test_channel_a_loopback(void)
{
    test_local_loopback(NEXT_SCC_A_CTRL, NEXT_SCC_A_DATA);
}

static void wait_for_rx_available(QTestState *qts)
{
    unsigned int i;

    for (i = 0; i < 10000; i++) {
        if (qtest_readb(qts, NEXT_SCC_A_CTRL) & SCC_RR0_RXAVAIL) {
            return;
        }
    }
    g_error("timed out waiting for channel A receive data");
}

static void test_serial0_backend_round_trip(void)
{
    int sock_fd;
    uint8_t byte;
    QTestState *qts = next_cube_serial_start_with_backend(&sock_fd);

    scc_configure_rx_tx(qts, NEXT_SCC_A_CTRL, false);

    qtest_writeb(qts, NEXT_SCC_A_DATA, 'T');
    g_assert_cmpint(recv(sock_fd, &byte, 1, 0), ==, 1);
    g_assert_cmphex(byte, ==, 'T');

    g_assert_cmpint(send(sock_fd, "R", 1, 0), ==, 1);
    wait_for_rx_available(qts);
    assert_scc_irq(qts, true);
    g_assert_cmphex(qtest_readb(qts, NEXT_SCC_A_DATA), ==, 'R');
    assert_scc_irq(qts, false);

    close(sock_fd);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/next-cube/serial/clock-select", test_clock_select);
    qtest_add_func("/next-cube/serial/channel-b-loopback",
                   test_channel_b_loopback);
    qtest_add_func("/next-cube/serial/channel-a-loopback",
                   test_channel_a_loopback);
    qtest_add_func("/next-cube/serial/serial0-backend-round-trip",
                   test_serial0_backend_round_trip);

    return g_test_run();
}
