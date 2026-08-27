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
#define NEXT_DMA_ENABLE     0x01000000
#define NEXT_DMA_SUPDATE    0x02000000
#define NEXT_DMA_COMPLETE   0x08000000
#define NEXT_DMA_BUSEXC     0x10000000

#define NEXT_INTR_STATUS    0x02007000
#define NEXT_SCC_A_CTRL     0x02118001
#define NEXT_TEST_RAM       0x04010000
#define NEXT_SCC_DMA_IRQ    (1U << 21)
#define NEXT_ROM_SIZE       (128 * 1024)

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

static void scc_write_reg(QTestState *qts, uint8_t reg, uint8_t value)
{
    qtest_writeb(qts, NEXT_SCC_A_CTRL, reg);
    qtest_writeb(qts, NEXT_SCC_A_CTRL, value);
}

static void scc_configure_tx(QTestState *qts)
{
    scc_write_reg(qts, 3, 0x01);
    scc_write_reg(qts, 5, 0x68);
    scc_write_reg(qts, 1, 0xc0);
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

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/next-cube/scc-dma/transmit", test_scc_dma_transmit);
    qtest_add_func("/next-cube/scc-dma/zero-length", test_scc_dma_zero_length);
    qtest_add_func("/next-cube/scc-dma/read-direction-pending",
                   test_scc_dma_read_direction_pending);
    return g_test_run();
}
