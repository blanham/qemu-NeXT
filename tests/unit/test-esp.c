/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "hw/scsi/esp.h"
#include "migration/vmstate.h"
#include "monitor/monitor.h"
#include "system/dma.h"
#include "system/memory.h"
#include "system/memory-internal.h"
#include "system/ramblock.h"
#include "system/runstate-action.h"
#include "system/tcg.h"

/*
 * The ESP and SCSI bus sources use these system-mode hooks from code which is
 * not exercised by this state-machine test.  Keep the unit target small while
 * retaining the real ESP FIFO and SCSI continuation paths under test.
 */
const VMStateInfo vmstate_info_timer = { 0 };
bool tcg_allowed;
ShutdownAction shutdown_action = SHUTDOWN_ACTION_POWEROFF;

bool qemu_ram_is_migratable(const RAMBlock *rb)
{
    return false;
}

void qemu_flush_coalesced_mmio_buffer(void)
{
}

MemTxResult dma_buf_read(void *ptr, dma_addr_t len, dma_addr_t *residual,
                         QEMUSGList *sg, MemTxAttrs attrs)
{
    return MEMTX_OK;
}

MemTxResult dma_buf_write(void *ptr, dma_addr_t len, dma_addr_t *residual,
                          QEMUSGList *sg, MemTxAttrs attrs)
{
    return MEMTX_OK;
}

int monitor_printf(Monitor *mon, const char *fmt, ...)
{
    return 0;
}

AddressSpaceDispatch *address_space_dispatch_new(FlatView *fv)
{
    return NULL;
}

void address_space_dispatch_free(AddressSpaceDispatch *d)
{
}

void flatview_add_to_dispatch(FlatView *fv, MemoryRegionSection *section)
{
}

void address_space_dispatch_compact(AddressSpaceDispatch *d)
{
}

void qdev_simple_device_unplug_cb(HotplugHandler *hotplug_dev,
                                  DeviceState *dev, Error **errp)
{
}

typedef struct FakeESPRequest {
    SCSIRequest req;
    uint8_t chunks[2];
    unsigned read_data_calls;
    unsigned dma_write_calls;
    uint8_t dma_byte;
} FakeESPRequest;

static uint8_t *fake_get_buf(SCSIRequest *req)
{
    FakeESPRequest *fake = container_of(req, FakeESPRequest, req);

    g_assert_cmpuint(fake->read_data_calls, <, G_N_ELEMENTS(fake->chunks));
    return &fake->chunks[fake->read_data_calls];
}

static void fake_read_data(SCSIRequest *req)
{
    FakeESPRequest *fake = container_of(req, FakeESPRequest, req);

    fake->read_data_calls++;
    if (fake->read_data_calls == 1) {
        esp_transfer_data(req, 1);
    }
}

static const SCSIReqOps fake_req_ops = {
    .size = sizeof(FakeESPRequest),
    .read_data = fake_read_data,
    .get_buf = fake_get_buf,
};

static void fake_dma_write(void *opaque, uint8_t *buf, int len)
{
    FakeESPRequest *fake = opaque;

    g_assert_cmpint(len, ==, 1);
    fake->dma_write_calls++;
    fake->dma_byte = buf[0];
}

static void test_esp_mixed_pio_chunk_reentry(void)
{
    ESPState esp = { 0 };
    SCSIDevice fake_dev = {
        .id = 0,
    };
    FakeESPRequest fake = {
        .req = {
            .dev = &fake_dev,
            .ops = &fake_req_ops,
            .hba_private = &esp,
            .cmd = {
                .mode = SCSI_XFER_FROM_DEV,
            },
        },
        .chunks = { 0xa1, 0xb2 },
    };

    fifo8_create(&esp.fifo, ESP_FIFO_SZ);
    esp.rregs[ESP_RSTAT] = STAT_DI;
    esp.rregs[ESP_CMD] = CMD_TI | CMD_DMA;
    esp.rregs[ESP_TCLO] = 2;
    esp.ti_size = 2;
    esp.dma = 1;
    esp.dma_enabled = 0;
    esp.data_ready = true;
    esp.current_req = &fake.req;
    esp.async_buf = &fake.chunks[0];
    esp.async_len = 1;
    esp.dma_memory_write = fake_dma_write;
    esp.dma_opaque = &fake;

    g_assert_cmphex(esp_reg_read(&esp, ESP_FIFO), ==, 0xa1);
    g_assert_cmpuint(fake.read_data_calls, ==, 1);
    g_assert_cmpuint(esp.async_len, ==, 1);
    g_assert_true(esp.async_buf == &fake.chunks[1]);
    g_assert_cmpuint(esp.ti_size, ==, 1);
    g_assert_cmpuint(esp.rregs[ESP_TCLO], ==, 1);
    g_assert_true(fifo8_is_empty(&esp.fifo));
    g_assert_cmpuint(fake.dma_write_calls, ==, 0);
    g_assert_nonnull(esp.dma_cb);

    esp_dma_enable(&esp, 0, 1);

    g_assert_cmpuint(fake.dma_write_calls, ==, 1);
    g_assert_cmphex(fake.dma_byte, ==, 0xb2);
    g_assert_cmpuint(esp.async_len, ==, 0);
    g_assert_cmpuint(esp.ti_size, ==, 0);
    g_assert_cmpuint(esp.rregs[ESP_TCLO], ==, 0);
    g_assert_true(esp.rregs[ESP_RSTAT] & STAT_TC);
    g_assert_null(esp.dma_cb);
    g_assert_cmpuint(fake.read_data_calls, ==, 2);

    fifo8_destroy(&esp.fifo);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/esp/mixed-pio-chunk-reentry",
                    test_esp_mixed_pio_chunk_reentry);
    return g_test_run();
}
