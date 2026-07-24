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

#define NEXT_FDC_BASE          0x02114100
#define NEXT_FDC_SRA           (NEXT_FDC_BASE + 0)
#define NEXT_FDC_DOR           (NEXT_FDC_BASE + 2)
#define NEXT_FDC_MSR_DSR       (NEXT_FDC_BASE + 4)
#define NEXT_FDC_FIFO          (NEXT_FDC_BASE + 5)
#define NEXT_FDC_CCR           (NEXT_FDC_BASE + 7)
#define NEXT_FLOPPY_CONTROL    (NEXT_FDC_BASE + 8)
#define NEXT_SCSI_CONTROL      0x02114020
#define NEXT_SCSI_STATUS       0x02114021
#define NEXT_ROM_SCSI_CONTROL  0x02014020
#define NEXT_ROM_SCSI_STATUS   0x02014021

#define NEXT_DMA_CSR           0x02000010
#define NEXT_DMA_NEXT          0x02004010
#define NEXT_DMA_LIMIT         0x02004014
#define NEXT_DMA_START         0x02004018
#define NEXT_DMA_STOP          0x0200401c
#define NEXT_DMA_NEXT_INIT     0x02004210
#define NEXT_INTR_STATUS       0x02007000

#define NEXT_FLOPPY_IRQ        (1U << 7)
#define NEXT_SCSI_DMA_IRQ      (1U << 26)
#define NEXT_RELEVANT_IRQS     (NEXT_FLOPPY_IRQ | NEXT_SCSI_DMA_IRQ)

#define DMA_SETENABLE          0x00010000
#define DMA_SETSUPDATE         0x00020000
#define DMA_SETREAD            0x00040000
#define DMA_CLRCOMPLETE        0x00080000
#define DMA_RESET              0x00100000
#define DMA_ENABLE             0x01000000
#define DMA_SUPDATE            0x02000000
#define DMA_READ               0x04000000
#define DMA_COMPLETE           0x08000000
#define DMA_BUSEXC             0x10000000
#define DMA_STATE_MASK         (DMA_ENABLE | DMA_SUPDATE | DMA_READ | \
                                DMA_COMPLETE | DMA_BUSEXC)

#define FDC_MSR_RQM            0x80
#define FDC_MSR_DIO            0x40
#define FDC_MSR_COMMAND_BUSY   0x10
#define FDC_RESULT_MSR         (FDC_MSR_RQM | FDC_MSR_DIO | \
                                FDC_MSR_COMMAND_BUSY)

#define NEXT_ROM_SIZE          (128 * 1024)
#define NEXT_FLOPPY_SIZE       1474560
#define NEXT_SECTOR_SIZE       512
#define NEXT_ROM_RESET_HOLD_NS (250 * 1000)
#define NEXT_DMA_BUFFER        0x04010000
#define NEXT_POLL_LIMIT        10000
#define NEXT_RESET_POLL_STEPS  256
#define NEXT_MEMORY_SENTINEL   0xa5

typedef struct TestFixture {
    int rom_fd;
    int floppy_fd;
    char *rom_path;
    char *floppy_path;
    char *migration_path;
    uint8_t disk_pattern[NEXT_SECTOR_SIZE];
} TestFixture;

static void cleanup_fixture(void *opaque)
{
    TestFixture *fixture = opaque;

    qtest_remove_abrt_handler(fixture);
    if (fixture->rom_fd >= 0) {
        close(fixture->rom_fd);
    }
    if (fixture->floppy_fd >= 0) {
        close(fixture->floppy_fd);
    }
    if (fixture->rom_path) {
        g_unlink(fixture->rom_path);
        g_free(fixture->rom_path);
    }
    if (fixture->floppy_path) {
        g_unlink(fixture->floppy_path);
        g_free(fixture->floppy_path);
    }
    if (fixture->migration_path) {
        g_unlink(fixture->migration_path);
        g_free(fixture->migration_path);
    }
    g_free(fixture);
}

static TestFixture *fixture_new(void)
{
    TestFixture *fixture = g_new0(TestFixture, 1);
    ssize_t bytes;
    size_t i;

    fixture->rom_fd = -1;
    fixture->floppy_fd = -1;
    qtest_add_abrt_handler(cleanup_fixture, fixture);
    g_test_queue_destroy(cleanup_fixture, fixture);

    fixture->rom_fd = g_file_open_tmp("next-floppy-rom-XXXXXX",
                                      &fixture->rom_path, NULL);
    g_assert_cmpint(fixture->rom_fd, >=, 0);
    g_assert_cmpint(ftruncate(fixture->rom_fd, NEXT_ROM_SIZE), ==, 0);
    close(fixture->rom_fd);
    fixture->rom_fd = -1;

    fixture->floppy_fd = g_file_open_tmp("next-floppy-disk-XXXXXX",
                                         &fixture->floppy_path, NULL);
    g_assert_cmpint(fixture->floppy_fd, >=, 0);
    g_assert_cmpint(ftruncate(fixture->floppy_fd, NEXT_FLOPPY_SIZE), ==, 0);
    for (i = 0; i < sizeof(fixture->disk_pattern); i++) {
        fixture->disk_pattern[i] = (i * 37 + 0x5b) & 0xff;
    }
    bytes = pwrite(fixture->floppy_fd, fixture->disk_pattern,
                   sizeof(fixture->disk_pattern), 0);
    g_assert_cmpint(bytes, ==, sizeof(fixture->disk_pattern));

    return fixture;
}

static QTestState *next_cube_start(TestFixture *fixture, bool with_media)
{
    g_autofree char *quoted_rom_path = g_shell_quote(fixture->rom_path);

    if (with_media) {
        g_autofree char *quoted_floppy_path =
            g_shell_quote(fixture->floppy_path);

        return qtest_initf("-machine next-cube -bios %s "
                           "-drive if=floppy,format=raw,file=%s",
                           quoted_rom_path, quoted_floppy_path);
    }

    return qtest_initf("-machine next-cube -bios %s", quoted_rom_path);
}

static QTestState *next_cube_start_migration(TestFixture *fixture,
                                             bool incoming)
{
    g_autofree char *quoted_rom_path = g_shell_quote(fixture->rom_path);
    g_autofree char *quoted_floppy_path =
        g_shell_quote(fixture->floppy_path);

    return qtest_initf("-machine next-cube -bios %s "
                       "-drive if=floppy,format=raw,readonly=on,file=%s %s",
                       quoted_rom_path, quoted_floppy_path,
                       incoming ? "-incoming defer" : "");
}

/* Keep controller mapping failures distinct from media-attachment failures. */
static void assert_controller_mapped(TestFixture *fixture)
{
    QTestState *qts = next_cube_start(fixture, false);

    qtest_writeb(qts, NEXT_FDC_DOR, 0x04);
    g_assert_cmphex(qtest_readb(qts, NEXT_FDC_MSR_DSR), ==, FDC_MSR_RQM);
    qtest_quit(qts);
}

static uint8_t wait_fdc_msr(QTestState *qts, uint8_t mask,
                            uint8_t expected, const char *operation)
{
    uint8_t value = 0;
    unsigned int i;

    for (i = 0; i < NEXT_POLL_LIMIT; i++) {
        value = qtest_readb(qts, NEXT_FDC_MSR_DSR);
        if ((value & mask) == expected) {
            return value;
        }
        qtest_clock_step(qts, 1);
    }

    g_error("timed out waiting for FDC %s: MSR 0x%02x, "
            "mask 0x%02x, expected 0x%02x",
            operation, value, mask, expected);
    return 0;
}

static void fdc_write_fifo(QTestState *qts, uint8_t value)
{
    wait_fdc_msr(qts, FDC_MSR_RQM | FDC_MSR_DIO,
                 FDC_MSR_RQM, "FIFO write");
    qtest_writeb(qts, NEXT_FDC_FIFO, value);
}

static uint8_t fdc_read_fifo(QTestState *qts)
{
    wait_fdc_msr(qts, FDC_MSR_RQM | FDC_MSR_DIO,
                 FDC_MSR_RQM | FDC_MSR_DIO, "FIFO read");
    return qtest_readb(qts, NEXT_FDC_FIFO);
}

static void fdc_send_command(QTestState *qts, const uint8_t *command,
                             size_t length)
{
    size_t i;

    for (i = 0; i < length; i++) {
        fdc_write_fifo(qts, command[i]);
    }
}

static void fdc_read_result(QTestState *qts, const uint8_t *expected,
                            size_t length)
{
    size_t i;

    wait_fdc_msr(qts, 0xf0, FDC_RESULT_MSR, "result phase");
    for (i = 0; i < length; i++) {
        g_assert_cmphex(fdc_read_fifo(qts), ==, expected[i]);
    }
}

static uint32_t wait_interrupts(QTestState *qts, uint32_t mask,
                                uint32_t expected, const char *operation)
{
    uint32_t value = 0;
    unsigned int i;

    for (i = 0; i < NEXT_POLL_LIMIT; i++) {
        value = qtest_readl(qts, NEXT_INTR_STATUS);
        if ((value & mask) == expected) {
            return value;
        }
        qtest_clock_step(qts, 1);
    }

    g_error("timed out waiting for %s: interrupt status 0x%08" PRIx32
            ", mask 0x%08" PRIx32 ", expected 0x%08" PRIx32,
            operation, value, mask, expected);
    return 0;
}

static uint32_t wait_dma_state(QTestState *qts, uint32_t mask,
                               uint32_t expected, const char *operation)
{
    uint32_t value = 0;
    unsigned int i;

    for (i = 0; i < NEXT_POLL_LIMIT; i++) {
        value = qtest_readl(qts, NEXT_DMA_CSR);
        if ((value & mask) == expected) {
            return value;
        }
        qtest_clock_step(qts, 1);
    }

    g_error("timed out waiting for %s: DMA CSR 0x%08" PRIx32
            ", mask 0x%08" PRIx32 ", expected 0x%08" PRIx32,
            operation, value, mask, expected);
    return 0;
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

static void prepare_dma_fdc(QTestState *qts)
{
    static const uint8_t configure[] = { 0x13, 0x00, 0x58, 0x00 };
    static const uint8_t specify[] = { 0x03, 0xd2, 0x10 };

    qtest_writeb(qts, NEXT_FLOPPY_CONTROL, 0x40);
    qtest_writeb(qts, NEXT_FDC_DOR, 0x04);
    qtest_writeb(qts, NEXT_FDC_CCR, 0x00);
    qtest_writeb(qts, NEXT_FDC_MSR_DSR, 0x00);
    fdc_send_command(qts, configure, sizeof(configure));
    fdc_send_command(qts, specify, sizeof(specify));
}

static void program_dma(QTestState *qts, uint32_t buffer, uint32_t limit,
                        uint32_t command)
{
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | (command & DMA_SETREAD));
    qtest_writel(qts, NEXT_DMA_NEXT_INIT, buffer);
    qtest_writel(qts, NEXT_DMA_LIMIT, limit);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_SETENABLE | command);
}

static void assert_relevant_interrupts(QTestState *qts, uint32_t expected)
{
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    NEXT_RELEVANT_IRQS, ==, expected);
}

static void assert_guest_memory_filled(QTestState *qts, uint8_t value)
{
    uint8_t received[NEXT_SECTOR_SIZE];
    size_t i;

    qtest_memread(qts, NEXT_DMA_BUFFER, received, sizeof(received));
    for (i = 0; i < sizeof(received); i++) {
        g_assert_cmphex(received[i], ==, value);
    }
}

static void test_controller_and_media(void)
{
    static const uint8_t version[] = { 0x10 };
    static const uint8_t seek[] = { 0x0f, 0x00, 0x01 };
    static const uint8_t sense[] = { 0x08 };
    static const uint8_t seek_result[] = { 0x20, 0x01 };
    TestFixture *fixture = fixture_new();
    QTestState *qts;

    assert_controller_mapped(fixture);
    qts = next_cube_start(fixture, true);

    g_assert_cmphex(qtest_readb(qts, NEXT_FDC_DOR), ==, 0x04);
    qtest_writeb(qts, NEXT_FDC_DOR, 0x04);
    g_assert_cmphex(qtest_readb(qts, NEXT_FDC_MSR_DSR), ==, FDC_MSR_RQM);

    fdc_send_command(qts, version, sizeof(version));
    g_assert_cmphex(fdc_read_fifo(qts), ==, 0x90);
    g_assert_cmphex(qtest_readb(qts, NEXT_FLOPPY_CONTROL), ==, 0x42);
    qtest_writeb(qts, NEXT_FLOPPY_CONTROL, 0xc0);
    g_assert_cmphex(qtest_readb(qts, NEXT_FLOPPY_CONTROL), ==, 0xc2);
    qtest_writeb(qts, NEXT_FLOPPY_CONTROL, 0x40);
    g_assert_cmphex(qtest_readb(qts, NEXT_FLOPPY_CONTROL), ==, 0x42);

    fdc_send_command(qts, seek, sizeof(seek));
    wait_interrupts(qts, NEXT_FLOPPY_IRQ, NEXT_FLOPPY_IRQ,
                    "floppy seek interrupt");
    fdc_send_command(qts, sense, sizeof(sense));
    fdc_read_result(qts, seek_result, sizeof(seek_result));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    NEXT_FLOPPY_IRQ, ==, 0);

    fdc_send_command(qts, sense, sizeof(sense));
    g_assert_cmphex(fdc_read_fifo(qts), ==, 0x80);
    g_assert_cmphex(wait_fdc_msr(qts, 0xff, FDC_MSR_RQM,
                                 "one-byte SENSE completion"),
                    ==, FDC_MSR_RQM);

    qtest_quit(qts);
}

static void test_rom_scsi_dma_control_alias(void)
{
    TestFixture *fixture = fixture_new();
    QTestState *qts = next_cube_start(fixture, false);

    /*
     * The v66 ROM and NeXT floppy driver use the 0x02014020 window.  It is
     * the same SCSI/floppy control and status pair also decoded at
     * 0x02114020, including clock select, DMA direction/mode, and FIFO flush.
     */
    qtest_writeb(qts, NEXT_ROM_SCSI_CONTROL, 0x9c);
    g_assert_cmphex(qtest_readb(qts, NEXT_ROM_SCSI_CONTROL), ==, 0x9c);
    g_assert_cmphex(qtest_readb(qts, NEXT_SCSI_CONTROL), ==, 0x9c);

    qtest_writeb(qts, NEXT_SCSI_CONTROL, 0x50);
    g_assert_cmphex(qtest_readb(qts, NEXT_ROM_SCSI_CONTROL), ==, 0x50);

    qtest_writeb(qts, NEXT_ROM_SCSI_STATUS, 0xa5);
    g_assert_cmphex(qtest_readb(qts, NEXT_SCSI_STATUS), ==, 0xa5);

    qtest_writeb(qts, NEXT_SCSI_STATUS, 0x3c);
    g_assert_cmphex(qtest_readb(qts, NEXT_ROM_SCSI_STATUS), ==, 0x3c);

    qtest_quit(qts);
}

static void test_rom_reset_configure_recalibrate(void)
{
    static const uint8_t configure[] = { 0x13, 0x00, 0x58, 0x00 };
    static const uint8_t specify_2880k[] = { 0x03, 0xa4, 0x20 };
    static const uint8_t specify_720k[] = { 0x03, 0xe1, 0x08 };
    static const uint8_t recalibrate[] = { 0x07, 0x00 };
    static const uint8_t sense[] = { 0x08 };
    static const uint8_t recalibrate_result[] = { 0x20, 0x00 };
    TestFixture *fixture = fixture_new();
    QTestState *qts;

    assert_controller_mapped(fixture);
    qts = next_cube_start(fixture, true);

    /*
     * NeXT ROM fc_82077_reset(): CONFIGURE within 250 us suppresses the
     * controller's reset polling interrupt before the ROM installs its
     * interrupt handler.
     */
    qtest_writeb(qts, NEXT_FDC_DOR, 0x00);
    qtest_clock_step(qts, NEXT_ROM_RESET_HOLD_NS);
    qtest_writeb(qts, NEXT_FDC_DOR, 0x04);
    qtest_writeb(qts, NEXT_FDC_MSR_DSR, 0x00);
    qtest_writeb(qts, NEXT_FDC_CCR, 0x00);
    qtest_writeb(qts, NEXT_FLOPPY_CONTROL, 0x40);
    fdc_send_command(qts, configure, sizeof(configure));
    fdc_send_command(qts, specify_2880k, sizeof(specify_2880k));
    g_assert_cmphex(qtest_readl(qts, NEXT_INTR_STATUS) &
                    NEXT_FLOPPY_IRQ, ==, 0);

    /*
     * fd_attach() changes to 720K timing, starts the motor, recalibrates,
     * and services the seek interrupt with SENSE INTERRUPT STATUS.
     */
    fdc_send_command(qts, configure, sizeof(configure));
    fdc_send_command(qts, specify_720k, sizeof(specify_720k));
    qtest_writeb(qts, NEXT_FDC_DOR, 0x14);
    fdc_send_command(qts, recalibrate, sizeof(recalibrate));
    wait_interrupts(qts, NEXT_FLOPPY_IRQ, NEXT_FLOPPY_IRQ,
                    "ROM recalibrate interrupt");
    fdc_send_command(qts, sense, sizeof(sense));
    fdc_read_result(qts, recalibrate_result, sizeof(recalibrate_result));
    g_assert_cmphex(qtest_readb(qts, NEXT_FDC_SRA) & 0x10, ==, 0);

    qtest_quit(qts);
}

static void test_media_to_ram_dma(void)
{
    static const uint8_t read_command[] = {
        0x46, 0x00, 0x00, 0x00, 0x01, 0x02, 0x01, 0x1b, 0xff,
    };
    static const uint8_t expected_result[] = {
        0x20, 0x00, 0x00, 0x01, 0x00, 0x01, 0x02,
    };
    TestFixture *fixture = fixture_new();
    uint8_t received[NEXT_SECTOR_SIZE];
    QTestState *qts;
    size_t i;

    assert_controller_mapped(fixture);
    qts = next_cube_start(fixture, true);
    prepare_dma_fdc(qts);

    qtest_memset(qts, NEXT_DMA_BUFFER, NEXT_MEMORY_SENTINEL,
                 NEXT_SECTOR_SIZE);
    program_dma(qts, NEXT_DMA_BUFFER,
                NEXT_DMA_BUFFER + NEXT_SECTOR_SIZE, DMA_SETREAD);
    fdc_send_command(qts, read_command, sizeof(read_command));

    qtest_memread(qts, NEXT_DMA_BUFFER, received, sizeof(received));
    for (i = 0; i < sizeof(received); i++) {
        g_assert_cmphex(received[i], ==, NEXT_MEMORY_SENTINEL);
    }
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) & DMA_COMPLETE, ==, 0);

    qtest_writeb(qts, NEXT_ROM_SCSI_CONTROL, 0x18);
    wait_dma_state(qts, DMA_COMPLETE, DMA_COMPLETE,
                   "floppy media-to-memory completion");

    qtest_memread(qts, NEXT_DMA_BUFFER, received, sizeof(received));
    g_assert_cmpmem(received, sizeof(received),
                    fixture->disk_pattern, sizeof(fixture->disk_pattern));
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER + NEXT_SECTOR_SIZE);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) & DMA_STATE_MASK, ==,
                    DMA_READ | DMA_COMPLETE);
    wait_interrupts(qts, NEXT_RELEVANT_IRQS, NEXT_RELEVANT_IRQS,
                    "floppy and shared-DMA interrupts");

    g_assert_cmphex(qtest_readb(qts, NEXT_FDC_MSR_DSR), ==, FDC_RESULT_MSR);
    fdc_read_result(qts, expected_result, sizeof(expected_result));
    assert_relevant_interrupts(qts, NEXT_SCSI_DMA_IRQ);

    qtest_writel(qts, NEXT_DMA_CSR, DMA_CLRCOMPLETE | DMA_SETREAD);
    assert_relevant_interrupts(qts, 0);

    qtest_quit(qts);
}

static void test_ram_to_media_dma(void)
{
    static const uint8_t write_command[] = {
        0x45, 0x00, 0x00, 0x00, 0x01, 0x02, 0x01, 0x1b, 0xff,
    };
    static const uint8_t expected_result[] = {
        0x20, 0x00, 0x00, 0x01, 0x00, 0x01, 0x02,
    };
    TestFixture *fixture = fixture_new();
    uint8_t source[NEXT_SECTOR_SIZE];
    uint8_t stored[NEXT_SECTOR_SIZE];
    QTestState *qts;
    ssize_t bytes;
    size_t i;

    for (i = 0; i < sizeof(source); i++) {
        source[i] = (i * 13 + 0xc7) & 0xff;
    }

    assert_controller_mapped(fixture);
    qts = next_cube_start(fixture, true);
    prepare_dma_fdc(qts);

    qtest_memwrite(qts, NEXT_DMA_BUFFER, source, sizeof(source));
    program_dma(qts, NEXT_DMA_BUFFER,
                NEXT_DMA_BUFFER + NEXT_SECTOR_SIZE + 16, 0);
    fdc_send_command(qts, write_command, sizeof(write_command));
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, 0);

    qtest_writeb(qts, NEXT_ROM_SCSI_CONTROL, 0x10);
    wait_interrupts(qts, NEXT_FLOPPY_IRQ, NEXT_FLOPPY_IRQ,
                    "floppy memory-to-media interrupt");

    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER + NEXT_SECTOR_SIZE);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_LIMIT), ==,
                    NEXT_DMA_BUFFER + NEXT_SECTOR_SIZE + 16);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) & DMA_STATE_MASK, ==,
                    DMA_ENABLE);
    assert_relevant_interrupts(qts, NEXT_FLOPPY_IRQ);

    fdc_read_result(qts, expected_result, sizeof(expected_result));
    assert_relevant_interrupts(qts, 0);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) & DMA_STATE_MASK, ==, 0);

    qtest_quit(qts);

    bytes = pread(fixture->floppy_fd, stored, sizeof(stored), 0);
    g_assert_cmpint(bytes, ==, sizeof(stored));
    g_assert_cmpmem(stored, sizeof(stored), source, sizeof(source));
}

static void test_reset_cancels_gated_dma_request(void)
{
    static const uint8_t read_command[] = {
        0x46, 0x00, 0x00, 0x00, 0x01, 0x02, 0x01, 0x1b, 0xff,
    };
    TestFixture *fixture = fixture_new();
    QTestState *qts;
    unsigned int i;

    assert_controller_mapped(fixture);
    qts = next_cube_start(fixture, true);
    prepare_dma_fdc(qts);

    qtest_memset(qts, NEXT_DMA_BUFFER, NEXT_MEMORY_SENTINEL,
                 NEXT_SECTOR_SIZE);
    program_dma(qts, NEXT_DMA_BUFFER,
                NEXT_DMA_BUFFER + NEXT_SECTOR_SIZE, DMA_SETREAD);
    fdc_send_command(qts, read_command, sizeof(read_command));

    assert_guest_memory_filled(qts, NEXT_MEMORY_SENTINEL);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) & DMA_STATE_MASK, ==,
                    DMA_ENABLE | DMA_READ);
    assert_relevant_interrupts(qts, 0);

    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) & DMA_STATE_MASK, ==, 0);
    assert_relevant_interrupts(qts, 0);

    /*
     * Re-arm DMA without issuing another FDC command.  Opening the gate must
     * not resurrect the pre-reset DREQ.
     */
    program_dma(qts, NEXT_DMA_BUFFER,
                NEXT_DMA_BUFFER + NEXT_SECTOR_SIZE, DMA_SETREAD);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) & DMA_STATE_MASK, ==,
                    DMA_ENABLE | DMA_READ);
    qtest_writeb(qts, NEXT_ROM_SCSI_CONTROL, 0x18);
    for (i = 0; i < NEXT_RESET_POLL_STEPS; i++) {
        qtest_clock_step(qts, 1);
        g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) & DMA_COMPLETE,
                        ==, 0);
    }

    assert_guest_memory_filled(qts, NEXT_MEMORY_SENTINEL);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) & DMA_STATE_MASK, ==,
                    DMA_ENABLE | DMA_READ);
    assert_relevant_interrupts(qts, 0);

    qtest_quit(qts);
}

static void test_chained_media_to_ram_dma(void)
{
    static const uint8_t read_command[] = {
        0x46, 0x00, 0x00, 0x00, 0x01, 0x02, 0x01, 0x1b, 0xff,
    };
    static const uint8_t expected_result[] = {
        0x20, 0x00, 0x00, 0x01, 0x00, 0x01, 0x02,
    };
    TestFixture *fixture = fixture_new();
    uint8_t received[NEXT_SECTOR_SIZE];
    QTestState *qts;
    size_t i;

    assert_controller_mapped(fixture);
    qts = next_cube_start(fixture, true);
    prepare_dma_fdc(qts);

    qtest_memset(qts, NEXT_DMA_BUFFER, NEXT_MEMORY_SENTINEL,
                 NEXT_SECTOR_SIZE);
    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET | DMA_SETREAD);
    qtest_writel(qts, NEXT_DMA_NEXT_INIT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_DMA_BUFFER + 256);
    qtest_writel(qts, NEXT_DMA_START, NEXT_DMA_BUFFER + 256);
    qtest_writel(qts, NEXT_DMA_STOP,
                 NEXT_DMA_BUFFER + NEXT_SECTOR_SIZE);
    qtest_writel(qts, NEXT_DMA_CSR,
                 DMA_SETENABLE | DMA_SETSUPDATE | DMA_SETREAD);

    fdc_send_command(qts, read_command, sizeof(read_command));
    qtest_writeb(qts, NEXT_ROM_SCSI_CONTROL, 0x18);
    wait_dma_state(qts, DMA_COMPLETE, DMA_COMPLETE,
                   "first chained floppy segment");

    qtest_memread(qts, NEXT_DMA_BUFFER, received, sizeof(received));
    g_assert_cmpmem(received, 256, fixture->disk_pattern, 256);
    for (i = 256; i < sizeof(received); i++) {
        g_assert_cmphex(received[i], ==, NEXT_MEMORY_SENTINEL);
    }
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) & DMA_STATE_MASK, ==,
                    DMA_ENABLE | DMA_READ | DMA_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER + 256);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_LIMIT), ==,
                    NEXT_DMA_BUFFER + NEXT_SECTOR_SIZE);
    assert_relevant_interrupts(qts, NEXT_SCSI_DMA_IRQ);
    g_assert_cmphex(qtest_readb(qts, NEXT_FDC_MSR_DSR) &
                    (FDC_MSR_RQM | FDC_MSR_DIO), ==, 0);

    qtest_writel(qts, NEXT_DMA_CSR, DMA_CLRCOMPLETE | DMA_SETREAD);
    wait_dma_state(qts, DMA_COMPLETE, DMA_COMPLETE,
                   "final chained floppy segment");

    qtest_memread(qts, NEXT_DMA_BUFFER, received, sizeof(received));
    g_assert_cmpmem(received, sizeof(received),
                    fixture->disk_pattern, sizeof(fixture->disk_pattern));
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER + NEXT_SECTOR_SIZE);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) & DMA_STATE_MASK, ==,
                    DMA_READ | DMA_COMPLETE);
    wait_interrupts(qts, NEXT_RELEVANT_IRQS, NEXT_RELEVANT_IRQS,
                    "final floppy and shared-DMA interrupts");
    fdc_read_result(qts, expected_result, sizeof(expected_result));

    qtest_quit(qts);
}

static void test_chained_scan_equal_compares_full_sector(void)
{
    static const uint8_t scan_command[] = {
        0x51, 0x00, 0x00, 0x00, 0x01, 0x02, 0x01, 0x1b, 0xff,
    };
    static const uint8_t expected_result[] = {
        0x20, 0x00, 0x04, 0x01, 0x00, 0x01, 0x02,
    };
    TestFixture *fixture = fixture_new();
    uint8_t compare[NEXT_SECTOR_SIZE];
    QTestState *qts;

    memcpy(compare, fixture->disk_pattern, sizeof(compare));
    compare[300] ^= 0xff;

    assert_controller_mapped(fixture);
    qts = next_cube_start(fixture, true);
    prepare_dma_fdc(qts);
    qtest_memwrite(qts, NEXT_DMA_BUFFER, compare, sizeof(compare));

    qtest_writel(qts, NEXT_DMA_CSR, DMA_RESET);
    qtest_writel(qts, NEXT_DMA_NEXT_INIT, NEXT_DMA_BUFFER);
    qtest_writel(qts, NEXT_DMA_LIMIT, NEXT_DMA_BUFFER + 256);
    qtest_writel(qts, NEXT_DMA_START, NEXT_DMA_BUFFER + 256);
    qtest_writel(qts, NEXT_DMA_STOP,
                 NEXT_DMA_BUFFER + NEXT_SECTOR_SIZE);
    qtest_writel(qts, NEXT_DMA_CSR,
                 DMA_SETENABLE | DMA_SETSUPDATE);

    fdc_send_command(qts, scan_command, sizeof(scan_command));
    qtest_writeb(qts, NEXT_ROM_SCSI_CONTROL, 0x10);
    wait_dma_state(qts, DMA_COMPLETE, DMA_COMPLETE,
                   "first chained SCAN segment");

    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER + 256);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) & DMA_STATE_MASK, ==,
                    DMA_ENABLE | DMA_COMPLETE);
    assert_relevant_interrupts(qts, NEXT_SCSI_DMA_IRQ);
    g_assert_cmphex(qtest_readb(qts, NEXT_FDC_MSR_DSR) &
                    (FDC_MSR_RQM | FDC_MSR_DIO), ==, 0);

    qtest_writel(qts, NEXT_DMA_CSR, DMA_CLRCOMPLETE);
    wait_dma_state(qts, DMA_COMPLETE, DMA_COMPLETE,
                   "final chained SCAN segment");

    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER + NEXT_SECTOR_SIZE);
    g_assert_cmphex(qtest_readl(qts, NEXT_DMA_CSR) & DMA_STATE_MASK, ==,
                    DMA_COMPLETE);
    wait_interrupts(qts, NEXT_RELEVANT_IRQS, NEXT_RELEVANT_IRQS,
                    "final SCAN and shared-DMA interrupts");
    fdc_read_result(qts, expected_result, sizeof(expected_result));

    qtest_quit(qts);
}

static void test_migrate_pending_gated_dma_request(void)
{
    static const uint8_t read_command[] = {
        0x46, 0x00, 0x00, 0x00, 0x01, 0x02, 0x01, 0x1b, 0xff,
    };
    static const uint8_t expected_result[] = {
        0x20, 0x00, 0x00, 0x01, 0x00, 0x01, 0x02,
    };
    TestFixture *fixture = fixture_new();
    g_autofree char *outgoing_uri = NULL;
    g_autofree char *incoming_uri = NULL;
    g_autofree char *quoted_migration_path = NULL;
    uint8_t received[NEXT_SECTOR_SIZE];
    QTestState *source;
    QTestState *destination;
    int migration_fd;
    unsigned int i;

    assert_controller_mapped(fixture);
    source = next_cube_start_migration(fixture, false);
    prepare_dma_fdc(source);
    qtest_memset(source, NEXT_DMA_BUFFER, NEXT_MEMORY_SENTINEL,
                 NEXT_SECTOR_SIZE);
    program_dma(source, NEXT_DMA_BUFFER,
                NEXT_DMA_BUFFER + NEXT_SECTOR_SIZE, DMA_SETREAD);
    fdc_send_command(source, read_command, sizeof(read_command));

    assert_guest_memory_filled(source, NEXT_MEMORY_SENTINEL);
    g_assert_cmphex(qtest_readl(source, NEXT_DMA_NEXT), ==, 0);
    g_assert_cmphex(qtest_readl(source, NEXT_DMA_CSR) & DMA_STATE_MASK, ==,
                    DMA_ENABLE | DMA_READ);
    assert_relevant_interrupts(source, 0);

    migration_fd = g_file_open_tmp("next-floppy-migration-XXXXXX",
                                   &fixture->migration_path, NULL);
    g_assert_cmpint(migration_fd, >=, 0);
    close(migration_fd);
    quoted_migration_path = g_shell_quote(fixture->migration_path);
    outgoing_uri = g_strdup_printf("exec: cat > %s",
                                   quoted_migration_path);
    qtest_qmp_assert_success(
        source, "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }",
        outgoing_uri);
    wait_migration_complete(source, "outgoing");
    qtest_quit(source);

    destination = next_cube_start_migration(fixture, true);
    incoming_uri = g_strdup_printf("exec: cat %s", quoted_migration_path);
    qtest_qmp_assert_success(
        destination,
        "{ 'execute': 'migrate-incoming', 'arguments': { 'uri': %s } }",
        incoming_uri);
    wait_migration_complete(destination, "incoming");

    assert_guest_memory_filled(destination, NEXT_MEMORY_SENTINEL);
    g_assert_cmphex(qtest_readl(destination, NEXT_DMA_NEXT), ==, 0);
    g_assert_cmphex(qtest_readl(destination, NEXT_DMA_CSR) & DMA_STATE_MASK,
                    ==, DMA_ENABLE | DMA_READ);
    assert_relevant_interrupts(destination, 0);

    qtest_writeb(destination, NEXT_ROM_SCSI_CONTROL, 0x18);
    wait_dma_state(destination, DMA_COMPLETE, DMA_COMPLETE,
                   "migrated floppy DMA completion");

    qtest_memread(destination, NEXT_DMA_BUFFER, received, sizeof(received));
    g_assert_cmpmem(received, sizeof(received),
                    fixture->disk_pattern, sizeof(fixture->disk_pattern));
    g_assert_cmphex(qtest_readl(destination, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER + NEXT_SECTOR_SIZE);
    wait_interrupts(destination, NEXT_RELEVANT_IRQS, NEXT_RELEVANT_IRQS,
                    "migrated floppy and shared-DMA interrupts");
    fdc_read_result(destination, expected_result, sizeof(expected_result));
    qtest_writel(destination, NEXT_DMA_CSR,
                 DMA_CLRCOMPLETE | DMA_SETREAD);
    for (i = 0; i < NEXT_RESET_POLL_STEPS; i++) {
        qtest_clock_step(destination, 1);
    }
    g_assert_cmphex(qtest_readl(destination, NEXT_DMA_NEXT), ==,
                    NEXT_DMA_BUFFER + NEXT_SECTOR_SIZE);
    assert_relevant_interrupts(destination, 0);

    qtest_quit(destination);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/next-cube/floppy/controller-and-media",
                   test_controller_and_media);
    qtest_add_func("/next-cube/floppy/rom-reset-configure-recalibrate",
                   test_rom_reset_configure_recalibrate);
    qtest_add_func("/next-cube/floppy/rom-scsi-dma-control-alias",
                   test_rom_scsi_dma_control_alias);
    qtest_add_func("/next-cube/floppy/media-to-ram-dma",
                   test_media_to_ram_dma);
    qtest_add_func("/next-cube/floppy/ram-to-media-dma",
                   test_ram_to_media_dma);
    qtest_add_func("/next-cube/floppy/reset-cancels-gated-dma-request",
                   test_reset_cancels_gated_dma_request);
    qtest_add_func("/next-cube/floppy/chained-media-to-ram-dma",
                   test_chained_media_to_ram_dma);
    qtest_add_func("/next-cube/floppy/chained-scan-equal-full-sector",
                   test_chained_scan_equal_compares_full_sector);
    qtest_add_func("/next-cube/floppy/migrate-pending-gated-dma-request",
                   test_migrate_pending_gated_dma_request);

    return g_test_run();
}
