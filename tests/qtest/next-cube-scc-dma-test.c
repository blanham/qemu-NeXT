/* SPDX-License-Identifier: NCSA
 *
 * Copyright (c) 2011-2026 Bryce Lanham
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define NEXT_DMA_BASE       0x02000000
#define NEXT_DMA_SCC_CSR    (NEXT_DMA_BASE + 0x00c0)
#define NEXT_DMA_NEXT       (NEXT_DMA_SCC_CSR + 0x4000)
#define NEXT_DMA_LIMIT      (NEXT_DMA_SCC_CSR + 0x4004)
#define NEXT_DMA_SETENABLE  0x00010000
#define NEXT_DMA_READ       0x00040000
#define NEXT_DMA_CLRCOMPLETE 0x00080000
#define NEXT_DMA_ENABLE     0x01000000
#define NEXT_DMA_SUPDATE    0x02000000
#define NEXT_DMA_COMPLETE   0x08000000
#define NEXT_DMA_BUSEXC     0x10000000

#define NEXT_INTR_STATUS    0x02007000
#define NEXT_SCC_PIO_IRQ    (1U << 17)
#define NEXT_SCC_BASE       0x02118000
#define NEXT_SCC_B_CTRL     (NEXT_SCC_BASE + 0)
#define NEXT_SCC_A_CTRL     0x02118001
#define NEXT_SCC_B_DATA     (NEXT_SCC_BASE + 2)
#define NEXT_SCC_A_DATA     (NEXT_SCC_BASE + 3)
#define NEXT_TEST_RAM       0x04010000
#define NEXT_SCC_DMA_IRQ    (1U << 21)
#define NEXT_ROM_SIZE       (128 * 1024)
#define NEXT_SCC_BACKPRESSURE_LENGTH (128 * 1024)

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

static QTestState *next_cube_scc_dma_start(int *sock_fd)
{
    TestROM *rom = g_new0(TestROM, 1);
    g_autofree char *quoted_rom_path = NULL;
    g_autofree char *args = NULL;

    rom->fd = -1;
    qtest_add_abrt_handler(cleanup_test_rom, rom);
    g_test_queue_destroy(cleanup_test_rom, rom);

    rom->fd = g_file_open_tmp("next-scc-dma-rom-XXXXXX", &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;

    quoted_rom_path = g_shell_quote(rom->path);
    args = g_strdup_printf("-machine next-cube -bios %s", quoted_rom_path);
    return qtest_init_with_serial(args, sock_fd);
}

static QTestState *next_cube_scc_dma_start_file(const char *path)
{
    TestROM *rom = g_new0(TestROM, 1);
    g_autofree char *quoted_rom_path = NULL;
    g_autofree char *quoted_chardev_path = NULL;
    g_autofree char *args = NULL;

    rom->fd = -1;
    qtest_add_abrt_handler(cleanup_test_rom, rom);
    g_test_queue_destroy(cleanup_test_rom, rom);

    rom->fd = g_file_open_tmp("next-scc-dma-rom-XXXXXX", &rom->path, NULL);
    g_assert_cmpint(rom->fd, >=, 0);
    g_assert_cmpint(ftruncate(rom->fd, NEXT_ROM_SIZE), ==, 0);
    close(rom->fd);
    rom->fd = -1;

    quoted_rom_path = g_shell_quote(rom->path);
    quoted_chardev_path = g_shell_quote(path);
    args = g_strdup_printf(
        "-machine next-cube -bios %s "
        "-chardev file,id=s0,path=%s -serial chardev:s0",
        quoted_rom_path, quoted_chardev_path);
    return qtest_init(args);
}

static void scc_write_reg(QTestState *qts, uint8_t reg, uint8_t value)
{
    qtest_writeb(qts, NEXT_SCC_A_CTRL, reg);
    qtest_writeb(qts, NEXT_SCC_A_CTRL, value);
}

static void scc_write_reg_at(QTestState *qts, uint64_t control,
                             uint8_t reg, uint8_t value)
{
    qtest_writeb(qts, control, reg);
    qtest_writeb(qts, control, value);
}

static void scc_configure_tx(QTestState *qts)
{
    scc_write_reg(qts, 3, 0x01);
    scc_write_reg(qts, 5, 0x68);
    scc_write_reg(qts, 1, 0xc0);
}

static void scc_configure_tx_at(QTestState *qts, uint64_t control,
                                bool loopback)
{
    scc_write_reg_at(qts, control, 3, 0x01);
    scc_write_reg_at(qts, control, 5, 0x68);
    if (loopback) {
        scc_write_reg_at(qts, control, 14, 0x10);
    }
    scc_write_reg_at(qts, control, 1, 0xc0);
}

static void scc_configure_rx(QTestState *qts, uint64_t control)
{
    scc_write_reg_at(qts, control, 1, 0xe0);
    scc_write_reg_at(qts, control, 3, 0x01);
}

static void wait_for_rx_available(QTestState *qts, uint64_t control)
{
    unsigned int i;

    for (i = 0; i < 10000; i++) {
        if (qtest_readb(qts, control) & 0x01) {
            return;
        }
    }
    g_error("timed out waiting for SCC receive data");
}

static bool receive_exact(int fd, uint8_t *buf, size_t length)
{
    const gint64 deadline = g_get_monotonic_time() + G_TIME_SPAN_SECOND;
    size_t received = 0;

    while (received < length) {
        GPollFD pollfd = { .fd = fd, .events = G_IO_IN };
        gint64 remaining = deadline - g_get_monotonic_time();
        gint64 timeout_ms;
        ssize_t count;

        if (remaining <= 0) {
            return false;
        }
        timeout_ms = (remaining + G_TIME_SPAN_MILLISECOND - 1) /
                     G_TIME_SPAN_MILLISECOND;
        timeout_ms = MIN(timeout_ms, G_MAXINT);
        if (g_poll(&pollfd, 1, timeout_ms) != 1 ||
            !(pollfd.revents & G_IO_IN)) {
            return false;
        }
        count = recv(fd, buf + received, length - received, MSG_DONTWAIT);
        if (count <= 0) {
            return false;
        }
        received += count;
    }
    return true;
}

static void test_scc_dma_transmit(void)
{
    static const uint8_t message[] = "DMA!";
    uint8_t received[sizeof(message) - 1];
    GPollFD pollfd = { .events = G_IO_IN };
    int sock_fd;
    QTestState *qts = next_cube_scc_dma_start(&sock_fd);

    scc_configure_tx(qts);
    qtest_memwrite(qts, NEXT_TEST_RAM, message, sizeof(message) - 1);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_TEST_RAM);
    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_TEST_RAM + sizeof(message) - 1);
    qtest_writel(qts, NEXT_DMA_SCC_CSR, NEXT_DMA_SETENABLE);

    g_assert_true(receive_exact(sock_fd, received, sizeof(received)));
    pollfd.fd = sock_fd;
    pollfd.revents = 0;
    g_assert_cmpint(g_poll(&pollfd, 1, 0), ==, 0);
    g_assert_cmpmem(received, sizeof(received), message, sizeof(received));
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_COMPLETE,
                    ==, NEXT_DMA_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_TEST_RAM + sizeof(message) - 1);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_ENABLE,
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_SCC_DMA_IRQ,
                    ==, NEXT_SCC_DMA_IRQ);

    close(sock_fd);
    qtest_quit(qts);
}

static void test_scc_dma_zero_length(void)
{
    int sock_fd;
    QTestState *qts = next_cube_scc_dma_start(&sock_fd);

    scc_configure_tx(qts);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_TEST_RAM);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_TEST_RAM);
    qtest_writel(qts, NEXT_DMA_SCC_CSR, NEXT_DMA_SETENABLE);

    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_COMPLETE,
                    ==, NEXT_DMA_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_BUSEXC,
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_ENABLE,
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_SUPDATE,
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, NEXT_TEST_RAM);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_SCC_DMA_IRQ,
                    ==, NEXT_SCC_DMA_IRQ);

    close(sock_fd);
    qtest_quit(qts);
}

static void test_scc_dma_read_direction_pending(void)
{
    static const uint8_t message[] = "DMA!";
    GPollFD pollfd = { .events = G_IO_IN };
    int sock_fd;
    QTestState *qts = next_cube_scc_dma_start(&sock_fd);

    scc_configure_tx(qts);
    qtest_memwrite(qts, NEXT_TEST_RAM, message, sizeof(message) - 1);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_TEST_RAM);
    qtest_writel(qts, NEXT_DMA_LIMIT,
                 NEXT_TEST_RAM + sizeof(message) - 1);
    qtest_writel(qts, NEXT_DMA_SCC_CSR, NEXT_DMA_SETENABLE | NEXT_DMA_READ);

    pollfd.fd = sock_fd;
    g_assert_cmpint(g_poll(&pollfd, 1, 100), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_ENABLE,
                    ==, NEXT_DMA_ENABLE);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_COMPLETE,
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_BUSEXC,
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, NEXT_TEST_RAM);

    close(sock_fd);
    qtest_quit(qts);
}

static void test_scc_dma_transmit_rejected(void)
{
    const uint8_t value = 'X';
    QTestState *qts = next_cube_scc_dma_start_file("/dev/full");

    scc_configure_tx(qts);
    qtest_memwrite(qts, NEXT_TEST_RAM, &value, sizeof(value));
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_TEST_RAM);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_TEST_RAM + sizeof(value));
    qtest_writel(qts, NEXT_DMA_SCC_CSR, NEXT_DMA_SETENABLE);

    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, NEXT_TEST_RAM);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_ENABLE,
                    ==, NEXT_DMA_ENABLE);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_COMPLETE,
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_BUSEXC,
                    ==, 0);

    qtest_quit(qts);
}

static void test_scc_dma_transmit_backpressure(void)
{
    g_autofree uint8_t *message = g_malloc(NEXT_SCC_BACKPRESSURE_LENGTH);
    g_autofree uint8_t *received = g_malloc(NEXT_SCC_BACKPRESSURE_LENGTH);
    const int receive_buffer_size = 4096;
    const uint32_t limit = NEXT_TEST_RAM + NEXT_SCC_BACKPRESSURE_LENGTH;
    uint32_t blocked_next = NEXT_TEST_RAM;
    uint32_t previous;
    int sock_fd;
    size_t i;
    unsigned int stable_reads;
    QTestState *qts = next_cube_scc_dma_start(&sock_fd);
    gint64 deadline;

    g_assert_cmpint(setsockopt(sock_fd, SOL_SOCKET, SO_RCVBUF,
                               &receive_buffer_size,
                               sizeof(receive_buffer_size)), ==, 0);
    for (i = 0; i < NEXT_SCC_BACKPRESSURE_LENGTH; i++) {
        message[i] = (uint8_t)(i * 53 + 7);
    }

    scc_configure_tx(qts);
    qtest_memwrite(qts, NEXT_TEST_RAM, message,
                   NEXT_SCC_BACKPRESSURE_LENGTH);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_TEST_RAM);
    qtest_writel(qts, NEXT_DMA_LIMIT, limit);
    qtest_writel(qts, NEXT_DMA_SCC_CSR, NEXT_DMA_SETENABLE);

    previous = NEXT_TEST_RAM;
    stable_reads = 0;
    deadline = g_get_monotonic_time() + 2 * G_TIME_SPAN_SECOND;
    while (g_get_monotonic_time() < deadline) {
        blocked_next = qtest_readl(qts, NEXT_DMA_NEXT);
        if (blocked_next > NEXT_TEST_RAM && blocked_next < limit &&
            blocked_next == previous) {
            if (++stable_reads == 4) {
                break;
            }
        } else {
            stable_reads = 0;
        }
        previous = blocked_next;
    }
    g_assert_cmpuint(stable_reads, ==, 4);
    g_assert_cmphex(blocked_next, >, NEXT_TEST_RAM);
    g_assert_cmphex(blocked_next, <, limit);

    /* The rejected byte must remain pending while the peer is not drained. */
    for (i = 0; i < 4; i++) {
        g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, blocked_next);
    }
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_ENABLE,
                    ==, NEXT_DMA_ENABLE);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_COMPLETE,
                    ==, 0);

    /* Draining the socket must wake DMA and preserve the entire stream. */
    g_assert_true(receive_exact(sock_fd, received,
                                NEXT_SCC_BACKPRESSURE_LENGTH));
    g_assert_cmpmem(received, NEXT_SCC_BACKPRESSURE_LENGTH,
                    message, NEXT_SCC_BACKPRESSURE_LENGTH);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, limit);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_COMPLETE,
                    ==, NEXT_DMA_COMPLETE);

    close(sock_fd);
    qtest_quit(qts);
}

static void test_scc_dma_pio_irq_independence(void)
{
    const uint8_t value = 'P';
    uint32_t status;
    int sock_fd;
    QTestState *qts = next_cube_scc_dma_start(&sock_fd);

    scc_write_reg_at(qts, NEXT_SCC_A_CTRL, 1, 0xf0);
    scc_write_reg_at(qts, NEXT_SCC_A_CTRL, 3, 0x01);
    g_assert_cmpint(send(sock_fd, &value, sizeof(value), 0), ==,
                    sizeof(value));
    wait_for_rx_available(qts, NEXT_SCC_A_CTRL);

    status = qtest_readl(qts, NEXT_INTR_STATUS);
    g_assert_cmphex(status & NEXT_SCC_PIO_IRQ, ==, NEXT_SCC_PIO_IRQ);
    g_assert_cmphex(status & NEXT_SCC_DMA_IRQ, ==, 0);

    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_TEST_RAM);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_TEST_RAM);
    qtest_writel(qts, NEXT_DMA_SCC_CSR, NEXT_DMA_SETENABLE | NEXT_DMA_READ);
    status = qtest_readl(qts, NEXT_INTR_STATUS);
    g_assert_cmphex(status & NEXT_SCC_PIO_IRQ, ==, NEXT_SCC_PIO_IRQ);
    g_assert_cmphex(status & NEXT_SCC_DMA_IRQ, ==, NEXT_SCC_DMA_IRQ);

    g_assert_cmphex(qtest_readb(qts, NEXT_SCC_A_DATA), ==, value);
    status = qtest_readl(qts, NEXT_INTR_STATUS);
    g_assert_cmphex(status & NEXT_SCC_PIO_IRQ, ==, 0);
    g_assert_cmphex(status & NEXT_SCC_DMA_IRQ, ==, NEXT_SCC_DMA_IRQ);

    qtest_writel(qts, NEXT_DMA_SCC_CSR,
                 NEXT_DMA_READ | NEXT_DMA_CLRCOMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_SCC_DMA_IRQ,
                    ==, 0);

    close(sock_fd);
    qtest_quit(qts);
}

static void test_scc_dma_transmit_quantum(void)
{
    const size_t length = 4097;
    g_autofree uint8_t *message = g_malloc(length);
    g_autofree uint8_t *received = g_malloc(length);
    int sock_fd;
    size_t i;
    QTestState *qts;

    for (i = 0; i < length; i++) {
        message[i] = (uint8_t)(i * 37 + 11);
    }

    qts = next_cube_scc_dma_start(&sock_fd);
    scc_configure_tx(qts);
    qtest_memwrite(qts, NEXT_TEST_RAM, message, length);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_TEST_RAM);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_TEST_RAM + length);
    qtest_writel(qts, NEXT_DMA_SCC_CSR, NEXT_DMA_SETENABLE);

    /*
     * This length crosses the per-BH quantum.  qtest cannot reliably sample
     * the intermediate pointer because queued BHs may run before the next
     * synchronous command, so exact output verifies the continuation.
     */
    g_assert_true(receive_exact(sock_fd, received, length));
    g_assert_cmpmem(received, length, message, length);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_TEST_RAM + length);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_COMPLETE,
                    ==, NEXT_DMA_COMPLETE);

    close(sock_fd);
    qtest_quit(qts);
}

static void test_scc_dma_receive(void)
{
    uint8_t received = 0;
    const uint8_t message = 'R';
    int sock_fd;
    QTestState *qts = next_cube_scc_dma_start(&sock_fd);

    scc_configure_rx(qts, NEXT_SCC_A_CTRL);
    g_assert_cmpint(send(sock_fd, &message, sizeof(message), 0), ==,
                    sizeof(message));
    wait_for_rx_available(qts, NEXT_SCC_A_CTRL);
    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_TEST_RAM);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_TEST_RAM + sizeof(message));
    qtest_writel(qts, NEXT_DMA_SCC_CSR,
                 NEXT_DMA_SETENABLE | NEXT_DMA_READ);

    qtest_memread(qts, NEXT_TEST_RAM, &received, sizeof(received));
    g_assert_cmphex(received, ==, message);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_COMPLETE,
                    ==, NEXT_DMA_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_TEST_RAM + sizeof(message));
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_ENABLE,
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_SCC_DMA_IRQ,
                    ==, NEXT_SCC_DMA_IRQ);

    close(sock_fd);
    qtest_quit(qts);
}

static void test_scc_dma_local_loopback(uint64_t control, uint64_t data,
                                        uint32_t source, uint32_t target,
                                        uint8_t value)
{
    uint8_t received = 0;
    int sock_fd;
    QTestState *qts = next_cube_scc_dma_start(&sock_fd);

    scc_configure_tx_at(qts, control, true);
    qtest_memwrite(qts, source, &value, sizeof(value));
    qtest_writel(qts, NEXT_DMA_NEXT, source);
    qtest_writel(qts, NEXT_DMA_LIMIT, source + sizeof(value));
    qtest_writel(qts, NEXT_DMA_SCC_CSR, NEXT_DMA_SETENABLE);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_COMPLETE,
                    ==, NEXT_DMA_COMPLETE);

    scc_write_reg_at(qts, control, 1, 0xe0);
    qtest_writel(qts, NEXT_DMA_NEXT, target);
    qtest_writel(qts, NEXT_DMA_LIMIT, target + sizeof(value));
    qtest_writel(qts, NEXT_DMA_SCC_CSR,
                 NEXT_DMA_SETENABLE | NEXT_DMA_READ | NEXT_DMA_CLRCOMPLETE);

    qtest_memread(qts, target, &received, sizeof(received));
    g_assert_cmphex(received, ==, value);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_SCC_CSR) & NEXT_DMA_COMPLETE,
                    ==, NEXT_DMA_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    target + sizeof(value));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) & NEXT_SCC_DMA_IRQ,
                    ==, NEXT_SCC_DMA_IRQ);

    close(sock_fd);
    qtest_quit(qts);
}

static void test_scc_dma_local_loopback_a(void)
{
    test_scc_dma_local_loopback(NEXT_SCC_A_CTRL, NEXT_SCC_A_DATA,
                                NEXT_TEST_RAM + 0x10,
                                NEXT_TEST_RAM + 0x20, 'A');
}

static void test_scc_dma_local_loopback_b(void)
{
    test_scc_dma_local_loopback(NEXT_SCC_B_CTRL, NEXT_SCC_B_DATA,
                                NEXT_TEST_RAM + 0x30,
                                NEXT_TEST_RAM + 0x40, 'B');
}

static void test_scc_dma_arbitration(void)
{
    uint8_t first = 0;
    uint8_t second = 0;
    const uint8_t value_a = 'A';
    const uint8_t value_b = 'B';
    int sock_fd;
    QTestState *qts = next_cube_scc_dma_start(&sock_fd);

    scc_configure_tx_at(qts, NEXT_SCC_A_CTRL, true);
    scc_configure_tx_at(qts, NEXT_SCC_B_CTRL, true);
    qtest_writeb(qts, NEXT_SCC_A_DATA, value_a);
    qtest_writeb(qts, NEXT_SCC_B_DATA, value_b);
    scc_write_reg_at(qts, NEXT_SCC_A_CTRL, 1, 0xe0);
    scc_write_reg_at(qts, NEXT_SCC_B_CTRL, 1, 0xe0);

    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_TEST_RAM + 0x50);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_TEST_RAM + 0x51);
    qtest_writel(qts, NEXT_DMA_SCC_CSR,
                 NEXT_DMA_SETENABLE | NEXT_DMA_READ);
    qtest_memread(qts, NEXT_TEST_RAM + 0x50, &first, sizeof(first));
    g_assert_cmphex(first, ==, value_a);
    g_assert_cmphex(qtest_readb(qts, NEXT_SCC_A_CTRL) & 0x01, ==, 0);
    g_assert_cmphex(qtest_readb(qts, NEXT_SCC_B_CTRL) & 0x01, ==, 0x01);

    qtest_writel(qts, NEXT_DMA_NEXT, NEXT_TEST_RAM + 0x60);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_TEST_RAM + 0x61);
    qtest_writel(qts, NEXT_DMA_SCC_CSR,
                 NEXT_DMA_SETENABLE | NEXT_DMA_READ | NEXT_DMA_CLRCOMPLETE);
    qtest_memread(qts, NEXT_TEST_RAM + 0x60, &second, sizeof(second));
    g_assert_cmphex(second, ==, value_b);

    close(sock_fd);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/next-cube/scc-dma/transmit", test_scc_dma_transmit);
    qtest_add_func("/next-cube/scc-dma/zero-length", test_scc_dma_zero_length);
    qtest_add_func("/next-cube/scc-dma/read-direction-pending",
                   test_scc_dma_read_direction_pending);
    qtest_add_func("/next-cube/scc-dma/transmit-rejected",
                   test_scc_dma_transmit_rejected);
    qtest_add_func("/next-cube/scc-dma/transmit-backpressure",
                   test_scc_dma_transmit_backpressure);
    qtest_add_func("/next-cube/scc-dma/pio-irq-independence",
                   test_scc_dma_pio_irq_independence);
    qtest_add_func("/next-cube/scc-dma/transmit-quantum",
                   test_scc_dma_transmit_quantum);
    qtest_add_func("/next-cube/scc-dma/receive", test_scc_dma_receive);
    qtest_add_func("/next-cube/scc-dma/local-loopback-a",
                   test_scc_dma_local_loopback_a);
    qtest_add_func("/next-cube/scc-dma/local-loopback-b",
                   test_scc_dma_local_loopback_b);
    qtest_add_func("/next-cube/scc-dma/arbitration", test_scc_dma_arbitration);
    return g_test_run();
}
