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
#define NEXT_INTR_MASK   0x02007800
#define NEXT_SCC_BASE    0x02118000
#define NEXT_SCC_CLOCK   (NEXT_SCC_BASE + 4)
#define NEXT_COMPUTER_SCC_BASE  0x02018000
#define NEXT_COMPUTER_SCC_CLOCK (NEXT_COMPUTER_SCC_BASE + 4)
#define NEXT_INTR_SCC    (1U << 17)
#define NEXT_ROM_SIZE    (128 * 1024)

#define NEXT_SCC_B_CTRL  (NEXT_SCC_BASE + 0)
#define NEXT_SCC_A_CTRL  (NEXT_SCC_BASE + 1)
#define NEXT_SCC_B_DATA  (NEXT_SCC_BASE + 2)
#define NEXT_SCC_A_DATA  (NEXT_SCC_BASE + 3)

#define SCC_RR0_RXAVAIL  0x01
#define SCC_RR3_TX_IP_A  0x10
#define SCC_WR0_RESET_TX_IP 0x28

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

static char *next_serial_args(const char *machine)
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
    return g_strdup_printf("-machine %s -bios %s", machine, quoted_rom_path);
}

static char *next_cube_serial_args(void)
{
    return next_serial_args("next-cube");
}

static QTestState *next_cube_serial_start(void)
{
    g_autofree char *args = next_cube_serial_args();

    return qtest_init(args);
}

static QTestState *next_computer_serial_start(void)
{
    g_autofree char *args = next_serial_args("next-computer");

    return qtest_init(args);
}

static QTestState *next_cube_serial_start_with_backend(int *sock_fd)
{
    g_autofree char *args = next_cube_serial_args();

    return qtest_init_with_serial(args, sock_fd);
}

#ifdef CONFIG_TRACE_LOG
static QTestState *next_cube_serial_start_with_backend_args(
    const char *extra_args, int *sock_fd)
{
    g_autofree char *base_args = next_cube_serial_args();
    g_autofree char *args = g_strdup_printf("%s %s", base_args, extra_args);

    return qtest_init_with_serial(args, sock_fd);
}
#endif

static void scc_write_reg(QTestState *qts, uint64_t control,
                          uint8_t reg, uint8_t value)
{
    qtest_writeb(qts, control, reg);
    qtest_writeb(qts, control, value);
}

static uint8_t scc_read_reg(QTestState *qts, uint64_t control, uint8_t reg)
{
    qtest_writeb(qts, control, reg);
    return qtest_readb(qts, control);
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
    int sock_fd;
#ifdef CONFIG_TRACE_LOG
    g_autofree char *log = NULL;
    g_autofree char *log_path = NULL;
    g_autofree char *quoted_log_path = NULL;
    g_autofree char *trace_args = NULL;
    gsize log_len;
    const char *pclk_trace;
    const char *rtxc_trace;
    int log_fd;
    QTestState *qts;

    log_fd = g_file_open_tmp("next-serial-clock-XXXXXX",
                             &log_path, NULL);
    g_assert_cmpint(log_fd, >=, 0);
    close(log_fd);
    quoted_log_path = g_shell_quote(log_path);
    trace_args = g_strdup_printf("-trace escc_update_parameters -D %s",
                                 quoted_log_path);
    qts = next_cube_serial_start_with_backend_args(trace_args, &sock_fd);
#else
    QTestState *qts = next_cube_serial_start_with_backend(&sock_fd);
#endif

    g_assert_cmphex(qtest_readb(qts, NEXT_SCC_CLOCK), ==, 0);
    qtest_writeb(qts, NEXT_SCC_CLOCK, 0x0a);
    g_assert_cmphex(qtest_readb(qts, NEXT_SCC_CLOCK), ==, 0x0a);

    /*
     * With a zero time constant and x16 clock, the baud-rate generator
     * must use PCLK/2/(2*16) when WR14_BRPCLK is set, then RTxC/2/(2*16)
     * when it is clear.
     */
    scc_write_reg(qts, NEXT_SCC_A_CTRL, 4, 0x44);
    scc_write_reg(qts, NEXT_SCC_A_CTRL, 12, 0);
    scc_write_reg(qts, NEXT_SCC_A_CTRL, 13, 0);
    scc_write_reg(qts, NEXT_SCC_A_CTRL, 14, 0x02);
    scc_write_reg(qts, NEXT_SCC_A_CTRL, 14, 0x00);

    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readb(qts, NEXT_SCC_CLOCK), ==, 0);

    close(sock_fd);
    qtest_quit(qts);

#ifdef CONFIG_TRACE_LOG
    g_assert_true(g_file_get_contents(log_path, &log, &log_len, NULL));
    pclk_trace = g_strstr_len(log, log_len,
                             "channel a: speed=57562");
    g_assert_nonnull(pclk_trace);
    rtxc_trace = strstr(pclk_trace, "channel a: speed=62500");
    g_assert_nonnull(rtxc_trace);
    g_unlink(log_path);
#endif
}

static void test_clock_select_rejects_long(void)
{
    QTestState *qts = next_cube_serial_start();

    /* The X15 serial clock register remains byte-wide. */
    qtest_writel(qts, NEXT_SCC_CLOCK, 0x0000000a);
    g_assert_cmphex(qtest_readb(qts, NEXT_SCC_CLOCK), ==, 0);

    qtest_quit(qts);
}

static void test_computer_clock_select_long(void)
{
    QTestState *qts = next_computer_serial_start();

    /* Rev. 1.0 v41 writes the clock-select register with move.l. */
    qtest_writel(qts, NEXT_COMPUTER_SCC_CLOCK, 0x0000000a);
    g_assert_cmphex(qtest_readb(qts, NEXT_COMPUTER_SCC_CLOCK), ==, 0x0a);
    g_assert_cmphex(qtest_readl(qts, NEXT_COMPUTER_SCC_CLOCK), ==, 0x0a);

    qtest_quit(qts);
}

static void test_computer_clock_select_rejects_byte_offsets(void)
{
    QTestState *qts = next_computer_serial_start();
    unsigned offset;

    qtest_writeb(qts, NEXT_COMPUTER_SCC_CLOCK, 0x5a);
    g_assert_cmphex(qtest_readb(qts, NEXT_COMPUTER_SCC_CLOCK), ==, 0x5a);

    for (offset = 1; offset <= 3; offset++) {
        qtest_writeb(qts, NEXT_COMPUTER_SCC_CLOCK + offset, 0xa5);
        g_assert_cmphex(qtest_readb(qts, NEXT_COMPUTER_SCC_CLOCK), ==, 0x5a);
        g_assert_cmphex(qtest_readb(qts,
                                    NEXT_COMPUTER_SCC_CLOCK + offset), ==, 0);
    }

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

static void test_tx_interrupt_enabled_after_buffer_empty(void)
{
    QTestState *qts = next_cube_serial_start();

    /*
     * A transmit buffer can become empty while its interrupt is disabled.
     * Enabling the interrupt later must expose the pending condition in RR3;
     * otherwise software sees an asserted IRQ with no serviceable source.
     */
    scc_write_reg(qts, NEXT_SCC_A_CTRL, 5, 0x68);
    qtest_writeb(qts, NEXT_SCC_A_DATA, 'P');
    assert_scc_irq(qts, false);

    scc_write_reg(qts, NEXT_SCC_A_CTRL, 1, 0x02);
    g_assert_cmphex(scc_read_reg(qts, NEXT_SCC_A_CTRL, 3) &
                    SCC_RR3_TX_IP_A, ==, SCC_RR3_TX_IP_A);
    assert_scc_irq(qts, true);

    qtest_writeb(qts, NEXT_SCC_A_CTRL, SCC_WR0_RESET_TX_IP);
    g_assert_cmphex(scc_read_reg(qts, NEXT_SCC_A_CTRL, 3) &
                    SCC_RR3_TX_IP_A, ==, 0);
    assert_scc_irq(qts, false);

    qtest_quit(qts);
}

static void test_shared_irq_clears_across_channels(void)
{
    QTestState *qts = next_cube_serial_start();

    scc_write_reg(qts, NEXT_SCC_A_CTRL, 5, 0x68);
    qtest_writeb(qts, NEXT_SCC_A_DATA, 'P');
    scc_write_reg(qts, NEXT_SCC_A_CTRL, 1, 0x02);
    assert_scc_irq(qts, true);

    /* Make channel B observe channel A's shared pending condition. */
    scc_write_reg(qts, NEXT_SCC_B_CTRL, 1, 0);

    qtest_writeb(qts, NEXT_SCC_A_CTRL, SCC_WR0_RESET_TX_IP);
    g_assert_cmphex(scc_read_reg(qts, NEXT_SCC_A_CTRL, 3), ==, 0);
    assert_scc_irq(qts, false);

    qtest_quit(qts);
}

#ifdef CONFIG_TRACE_LOG
static void test_scc_irq_bypasses_interrupt_mask(void)
{
    g_autofree char *log = NULL;
    g_autofree char *log_path = NULL;
    g_autofree char *quoted_log_path = NULL;
    g_autofree char *trace_args = NULL;
    gsize log_len;
    const char *first_tx_irq;
    const char *tx_irq_clear;
    const char *second_tx_irq;
    int log_fd;
    QTestState *qts;

    log_fd = g_file_open_tmp("next-serial-irq-XXXXXX", &log_path, NULL);
    g_assert_cmpint(log_fd, >=, 0);
    close(log_fd);
    quoted_log_path = g_shell_quote(log_path);
    trace_args = g_strdup_printf("-trace next_irq_update -D %s",
                                 quoted_log_path);
    qts = next_cube_serial_start_with_backend_args(trace_args, &log_fd);

    qtest_writel(qts, NEXT_INTR_MASK, 0);
    scc_write_reg(qts, NEXT_SCC_A_CTRL, 5, 0x68);
    qtest_writeb(qts, NEXT_SCC_A_DATA, 'P');
    scc_write_reg(qts, NEXT_SCC_A_CTRL, 1, 0x02);
    assert_scc_irq(qts, true);

    /* Loading the next byte must clear and then reassert TxIP. */
    qtest_writeb(qts, NEXT_SCC_A_DATA, 'Q');
    assert_scc_irq(qts, true);

    qtest_writel(qts, NEXT_INTR_STATUS, 1U << 12);
    close(log_fd);
    qtest_quit(qts);

    g_assert_true(g_file_get_contents(log_path, &log, &log_len, NULL));
    first_tx_irq = g_strstr_len(
        log, log_len,
        "next_irq_update level=5 vector=29 pending=0x20000 "
        "status=0x20000 mask=0x0");
    g_assert_nonnull(first_tx_irq);
    tx_irq_clear = strstr(first_tx_irq,
                          "next_irq_update level=0 vector=0 pending=0x0 "
                          "status=0x0 mask=0x0");
    g_assert_nonnull(tx_irq_clear);
    second_tx_irq = strstr(tx_irq_clear,
                           "next_irq_update level=5 vector=29 "
                           "pending=0x20000 status=0x20000 mask=0x0");
    g_assert_nonnull(second_tx_irq);
    g_assert_nonnull(g_strstr_len(
        log, log_len,
        "next_irq_update level=0 vector=0 pending=0x0 "
        "status=0x1000 mask=0x0"));
    g_unlink(log_path);
}
#endif

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
    GPollFD pollfd = {
        .events = G_IO_IN,
    };
    uint8_t byte;
    QTestState *qts = next_cube_serial_start_with_backend(&sock_fd);

    scc_configure_rx_tx(qts, NEXT_SCC_A_CTRL, false);

    qtest_writeb(qts, NEXT_SCC_A_DATA, 'T');
    pollfd.fd = sock_fd;
    g_assert_cmpint(g_poll(&pollfd, 1, 1000), ==, 1);
    g_assert_true(pollfd.revents & G_IO_IN);
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
    qtest_add_func("/next-cube/serial/clock-select-rejects-long",
                   test_clock_select_rejects_long);
    qtest_add_func("/next-computer/serial/clock-select-long",
                   test_computer_clock_select_long);
    qtest_add_func("/next-computer/serial/clock-select-rejects-byte-offsets",
                   test_computer_clock_select_rejects_byte_offsets);
    qtest_add_func("/next-cube/serial/channel-b-loopback",
                   test_channel_b_loopback);
    qtest_add_func("/next-cube/serial/channel-a-loopback",
                   test_channel_a_loopback);
    qtest_add_func("/next-cube/serial/tx-interrupt-enable-after-empty",
                   test_tx_interrupt_enabled_after_buffer_empty);
    qtest_add_func("/next-cube/serial/shared-irq-clears-across-channels",
                   test_shared_irq_clears_across_channels);
#ifdef CONFIG_TRACE_LOG
    qtest_add_func("/next-cube/serial/scc-irq-bypasses-interrupt-mask",
                   test_scc_irq_bypasses_interrupt_mask);
#endif
    qtest_add_func("/next-cube/serial/serial0-backend-round-trip",
                   test_serial0_backend_round_trip);

    return g_test_run();
}
