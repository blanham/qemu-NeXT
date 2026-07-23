/*
 * NeXT Cube System Driver
 *
 * Copyright (c) 2011 Bryce Lanham
 * Copyright (c) 2024 Mark Cave-Ayland
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "exec/hwaddr.h"
#include "exec/cpu-common.h"
#include "exec/cpu-interrupt.h"
#include "system/physmem.h"
#include "system/rtc.h"
#include "system/system.h"
#include "system/qtest.h"
#include "hw/core/irq.h"
#include "hw/m68k/next-cube.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/scsi/esp.h"
#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "qom/object.h"
#include "hw/char/escc.h" /* ZILOG 8530 Serial Emulation */
#include "hw/block/fdc.h"
#include "hw/misc/empty_slot.h"
#include "hw/core/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/cutils.h"
#include "qemu/timer.h"
#include "ui/console.h"
#include "target/m68k/cpu.h"
#include "migration/vmstate.h"
#include "trace.h"

/* #define DEBUG_NEXT */
#ifdef DEBUG_NEXT
#define DPRINTF(fmt, ...) \
    do { printf("NeXT: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do { } while (0)
#endif

#define ENTRY       0x0100001e
#define RAM_SIZE    0x4000000
#define ROM_FILE    "Rev_2.5_v66.bin"

#define NEXT_DMA_BASE        0x02000000
#define NEXT_SCSI_BASE       0x02114000
#define NEXT_SCSI_CSR_OFFSET 0x20
#define NEXT_SCSI_CSR_BASE   (NEXT_SCSI_BASE + NEXT_SCSI_CSR_OFFSET)
#define NEXT_ESP_CLOCK_HZ    20000000

#define NEXT_TIMER_ENABLE       0x80
#define NEXT_TIMER_UPDATE       0x40
#define NEXT_TIMER_TICK_NS      INT64_C(1000)
#define NEXT_TIMER_FULL_PERIOD  0x10000
#define NEXT_TIMER_IRQ_STATUS   0x20000000
#define NEXT_SCR2_TIMER_IPL7    0x00008000
#define NEXT_SCR2_SOFTINT_SHIFT 24
#define NEXT_IRQ_SOFTINT_MASK   0x00000003
#define NEXT_EVENTC_MASK        0x000fffff

#define NEXT_RTC_STATUS_NEW_CLOCK  0x80
#define NEXT_RTC_CONTROL_START     0x80
#define NEXT_RTC_CONTROL_AUTO_PON  0x20
#define NEXT_RTC_CONTROL_ALARM_EN  0x10
#define NEXT_RTC_CONTROL_ALARM_CLR 0x08
#define NEXT_RTC_CONTROL_FTU_CLR   0x04
#define NEXT_RTC_CONTROL_LOW_BATT  0x02
#define NEXT_RTC_CONTROL_RPD_CLR   0x01
#define NEXT_RTC_CONTROL_STORED    (NEXT_RTC_CONTROL_START | \
                                    NEXT_RTC_CONTROL_AUTO_PON | \
                                    NEXT_RTC_CONTROL_ALARM_EN | \
                                    NEXT_RTC_CONTROL_LOW_BATT)

#define NEXT_IRQ_IPL7_MASK      0xc0000000
#define NEXT_IRQ_IPL6_MASK      0x3ffc0000
#define NEXT_IRQ_IPL5_MASK      0x00038000
#define NEXT_IRQ_IPL4_MASK      0x00004000
#define NEXT_IRQ_IPL3_MASK      0x00003ffc
#define NEXT_IRQ_IPL2_MASK      0x00000002
#define NEXT_IRQ_IPL1_MASK      0x00000001


#define TYPE_NEXT_RTC "next-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(NeXTRTC, NEXT_RTC)

struct NeXTRTC {
    SysBusDevice parent_obj;

    int8_t phase;
    uint8_t ram[32];
    uint8_t command;
    uint8_t value;
    uint8_t status;
    uint8_t control;
    uint8_t retval;
    uint32_t counter;
    uint32_t counter_latch;
    uint32_t alarm;
    int64_t counter_ref_ns;

    qemu_irq data_out_irq;
    qemu_irq power_irq;
};

#define TYPE_NEXT_SCSI "next-scsi"
OBJECT_DECLARE_SIMPLE_TYPE(NeXTSCSI, NEXT_SCSI)

typedef struct NeXTTraceReadSampler {
    hwaddr addr;
    uint64_t value;
    uint64_t repeats;
} NeXTTraceReadSampler;

/* NeXT SCSI Controller */
struct NeXTSCSI {
    SysBusDevice parent_obj;

    MemoryRegion scsi_mem;

    SysBusESPState sysbus_esp;

    MemoryRegion scsi_csr_mem;
    uint8_t scsi_csr_1;
    uint8_t scsi_csr_2;

    NeXTTraceReadSampler trace_csr_read;
};

#define TYPE_NEXT_PC "next-pc"
OBJECT_DECLARE_SIMPLE_TYPE(NeXTPC, NEXT_PC)

/* NeXT Peripheral Controller */
struct NeXTPC {
    SysBusDevice parent_obj;

    M68kCPU *cpu;

    MemoryRegion floppy_mem;
    MemoryRegion system_timer_mem;
    MemoryRegion eventc_mem;
    MemoryRegion dummyen_mem;
    MemoryRegion dsp_mem;
    MemoryRegion printer_mem;
    MemoryRegion mmiomem;
    MemoryRegion scrmem;

    uint32_t scr1;
    uint32_t scr2;
    uint32_t old_scr2;
    uint32_t int_mask;
    uint32_t int_status;
    uint32_t led;

    QEMUTimer system_timer;
    uint16_t timer_latch;
    uint32_t timer_counter;
    uint8_t timer_csr;
    bool timer_irq_pending;
    uint32_t eventc_latched;

    NeXTSCSI next_scsi;

    qemu_irq scsi_reset;
    qemu_irq scsi_dma;

    ESCCState escc;

    NeXTRTC rtc;
    qemu_irq rtc_data_irq;
    qemu_irq rtc_cmd_reset_irq;
};

typedef struct next_dma {
    uint32_t csr;

    uint32_t saved_next;
    uint32_t saved_limit;
    uint32_t saved_start;
    uint32_t saved_stop;

    uint32_t next;
    uint32_t limit;
    uint32_t start;
    uint32_t stop;

    uint32_t next_initbuf;
    uint32_t size;

    uint8_t stage[16];
    uint8_t stage_len;
    uint8_t stage_flushes;
} next_dma;

#define TYPE_NEXT_MACHINE MACHINE_TYPE_NAME("next-cube")
OBJECT_DECLARE_SIMPLE_TYPE(NeXTState, NEXT_MACHINE)

struct NeXTState {
    MachineState parent;

    MemoryRegion rom;
    MemoryRegion rom2;
    MemoryRegion dmamem;
    MemoryRegion bmapm1;
    MemoryRegion bmapm2;

    next_dma dma[10];

    NeXTTraceReadSampler trace_scsi_dma_read;
};

/* Thanks to NeXT forums for this */
/*
static const uint8_t rtc_ram3[32] = {
    0x94, 0x0f, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xfb, 0x6d, 0x00, 0x00, 0x7B, 0x00,
    0x00, 0x00, 0x65, 0x6e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x13
};
*/
static const uint8_t rtc_ram2[32] = {
    0x94, 0x0f, 0x40, 0x03, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xfb, 0x6d, 0x00, 0x00, 0x4b, 0x00,
    0x41, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x7e,
};

#define SCR2_RTCLK 0x2
#define SCR2_RTDATA 0x4

static void next_scr2_led_update(NeXTPC *s)
{
    if (s->scr2 & 0x1) {
        DPRINTF("fault!\n");
        s->led++;
        if (s->led == 10) {
            DPRINTF("LED flashing, possible fault!\n");
            s->led = 0;
        }
    }
}

static void next_scr2_rtc_update(NeXTPC *s)
{
    uint8_t old_scr2_rtc, scr2_rtc;

    old_scr2_rtc = extract32(s->old_scr2, 8, 8);
    scr2_rtc = extract32(s->scr2, 8, 8);

    if (scr2_rtc & 0x1) {
        /* DPRINTF("RTC %x phase %i\n", scr2_2, rtc->phase); */
        /* If we are in going down clock... do something */
        if (((old_scr2_rtc & SCR2_RTCLK) != (scr2_rtc & SCR2_RTCLK)) &&
                ((scr2_rtc & SCR2_RTCLK) == 0)) {
            if (scr2_rtc & SCR2_RTDATA) {
                qemu_irq_raise(s->rtc_data_irq);
            } else {
                qemu_irq_lower(s->rtc_data_irq);
            }
        }
    } else {
        /* else end or abort */
        qemu_irq_raise(s->rtc_cmd_reset_irq);
    }
}

static int next_timer_irq_level(const NeXTPC *s)
{
    return s->scr2 & NEXT_SCR2_TIMER_IPL7 ? 7 : 6;
}

static int next_timer_irq_vector(const NeXTPC *s)
{
    return next_timer_irq_level(s) == 7 ? 31 : 30;
}

static void next_update_irq(NeXTPC *s)
{
    uint32_t pending = s->int_status & s->int_mask;
    int level = 0;

    if ((pending & NEXT_IRQ_IPL7_MASK) ||
        ((pending & NEXT_TIMER_IRQ_STATUS) &&
         (s->scr2 & NEXT_SCR2_TIMER_IPL7))) {
        level = 7;
    } else if (pending & NEXT_IRQ_IPL6_MASK) {
        level = 6;
    } else if (pending & NEXT_IRQ_IPL5_MASK) {
        level = 5;
    } else if (pending & NEXT_IRQ_IPL4_MASK) {
        level = 4;
    } else if (pending & NEXT_IRQ_IPL3_MASK) {
        level = 3;
    } else if (pending & NEXT_IRQ_IPL2_MASK) {
        level = 2;
    } else if (pending & NEXT_IRQ_IPL1_MASK) {
        level = 1;
    }

    m68k_set_irq_level(s->cpu, level, level ? level + 24 : 0);
}

static void next_scr2_update_softints(NeXTPC *s)
{
    uint32_t softints = extract32(s->scr2, NEXT_SCR2_SOFTINT_SHIFT, 2);

    s->int_status &= ~NEXT_IRQ_SOFTINT_MASK;
    s->int_status |= softints;
    next_update_irq(s);
}

static void next_timer_reroute_irq(NeXTPC *s)
{
    int level = next_timer_irq_level(s);
    int vector = next_timer_irq_vector(s);

    next_update_irq(s);
    trace_next_timer_irq(1, level, vector, s->int_status, s->scr2);
}

static uint64_t next_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    NeXTPC *s = NEXT_PC(opaque);
    uint64_t val;

    switch (addr) {
    case 0x2000:    /* 0x2007000 */
        /* DPRINTF("Read INT status: %x\n", s->int_status); */
        val = s->int_status;
        break;

    case 0x2800:    /* 0x2007800 */
        DPRINTF("MMIO Read INT mask: %x\n", s->int_mask);
        val = s->int_mask;
        break;

    case 0x7000 ... 0x7003:    /* 0x200c000 */
        val = extract32(s->scr1, (4 - (addr - 0x7000) - size) << 3,
                        size << 3);
        break;

    case 0x8000 ... 0x8003:    /* 0x200d000 */
        val = extract32(s->scr2, (4 - (addr - 0x8000) - size) << 3,
                        size << 3);
        break;

    default:
        val = 0;
        DPRINTF("MMIO Read @ 0x%"HWADDR_PRIx" size %d\n", addr, size);
        break;
    }

    return val;
}

static void next_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    NeXTPC *s = NEXT_PC(opaque);

    switch (addr) {
    case 0x2000:    /* 0x2007000 */
        DPRINTF("INT Status old: %x new: %x\n", s->int_status,
                (unsigned int)val);
        s->int_status = val;
        next_update_irq(s);
        break;

    case 0x2800:    /* 0x2007800 */
        DPRINTF("INT Mask old: %x new: %x\n", s->int_mask, (unsigned int)val);
        s->int_mask  = val;
        next_update_irq(s);
        break;

    case 0x7000 ... 0x7003:    /* 0x200c000 */
        DPRINTF("SCR1 Write: %x\n", (unsigned int)val);
        s->scr1 = deposit32(s->scr1, (4 - (addr - 0x7000) - size) << 3,
                            size << 3, val);
        break;

    case 0x8000 ... 0x8003:    /* 0x200d000 */
    {
        uint32_t previous_scr2 = s->scr2;

        s->scr2 = deposit32(s->scr2, (4 - (addr - 0x8000) - size) << 3,
                            size << 3, val);
        next_scr2_led_update(s);
        next_scr2_rtc_update(s);
        next_scr2_update_softints(s);
        if (s->timer_irq_pending &&
            ((previous_scr2 ^ s->scr2) & NEXT_SCR2_TIMER_IPL7)) {
            next_timer_reroute_irq(s);
        }
        s->old_scr2 = s->scr2;
        break;
    }

    default:
        DPRINTF("MMIO Write @ 0x%"HWADDR_PRIx " with 0x%x size %u\n", addr,
                (unsigned int)val, size);
    }
}

static const MemoryRegionOps next_mmio_ops = {
    .read = next_mmio_read,
    .write = next_mmio_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .endianness = DEVICE_BIG_ENDIAN,
};

#define SCSICSR_ENABLE  0x01
#define SCSICSR_RESET   0x02  /* reset scsi dma */
#define SCSICSR_FIFOFL  0x04
#define SCSICSR_DMADIR  0x08  /* if set, scsi to mem */
#define SCSICSR_CPUDMA  0x10  /* if set, dma enabled */
#define SCSICSR_INTMASK 0x20  /* if set, interrupt enabled */

#define NEXTDMA_SCSI(x)      (0x10 + x)
#define NEXTDMA_FD(x)        (0x10 + x)
#define NEXTDMA_ENTX(x)      (0x110 + x)
#define NEXTDMA_ENRX(x)      (0x150 + x)
#define NEXTDMA_CSR          0x0
#define NEXTDMA_NEXT         0x4000
#define NEXTDMA_LIMIT        0x4004
#define NEXTDMA_START        0x4008
#define NEXTDMA_STOP         0x400c
#define NEXTDMA_NEXT_INIT    0x4200
#define NEXTDMA_SIZE         0x4204

static bool next_trace_read_sample(NeXTTraceReadSampler *sampler,
                                   hwaddr addr, uint64_t value)
{
    if (sampler->repeats && sampler->addr == addr &&
        sampler->value == value) {
        if (sampler->repeats != UINT64_MAX) {
            sampler->repeats++;
        }
    } else {
        sampler->addr = addr;
        sampler->value = value;
        sampler->repeats = 1;
    }

    return !(sampler->repeats & (sampler->repeats - 1));
}

static void next_irq(void *opaque, int number, int level);

static void next_dma_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned int size)
{
    NeXTState *next_state = NEXT_MACHINE(qdev_get_machine());
    bool scsi_irq_ack = false;
    bool scsi_reg = false;

    switch (addr) {
    case NEXTDMA_ENRX(NEXTDMA_CSR):
        if (val & DMA_DEV2M) {
            next_state->dma[NEXTDMA_ENRX].csr |= DMA_DEV2M;
        }

        if (val & DMA_SETENABLE) {
            /* DPRINTF("SCSI DMA ENABLE\n"); */
            next_state->dma[NEXTDMA_ENRX].csr |= DMA_ENABLE;
        }
        if (val & DMA_SETSUPDATE) {
            next_state->dma[NEXTDMA_ENRX].csr |= DMA_SUPDATE;
        }
        if (val & DMA_CLRCOMPLETE) {
            next_state->dma[NEXTDMA_ENRX].csr &= ~DMA_COMPLETE;
        }

        if (val & DMA_RESET) {
            next_state->dma[NEXTDMA_ENRX].csr &= ~(DMA_COMPLETE | DMA_SUPDATE |
                                                  DMA_ENABLE | DMA_DEV2M);
        }
        /* DPRINTF("RXCSR \tWrite: %x\n",value); */
        break;

    case NEXTDMA_ENRX(NEXTDMA_NEXT_INIT):
        next_state->dma[NEXTDMA_ENRX].next_initbuf = val;
        break;

    case NEXTDMA_ENRX(NEXTDMA_NEXT):
        next_state->dma[NEXTDMA_ENRX].next = val;
        break;

    case NEXTDMA_ENRX(NEXTDMA_LIMIT):
        next_state->dma[NEXTDMA_ENRX].limit = val;
        break;

    case NEXTDMA_SCSI(NEXTDMA_CSR):
        scsi_reg = true;
        if (val & DMA_SETENABLE) {
            /* DPRINTF("SCSI DMA ENABLE\n"); */
            next_state->dma[NEXTDMA_SCSI].csr |= DMA_ENABLE;
        }
        if (val & DMA_SETSUPDATE) {
            next_state->dma[NEXTDMA_SCSI].csr |= DMA_SUPDATE;
        }
        if (val & DMA_CLRCOMPLETE) {
            next_state->dma[NEXTDMA_SCSI].csr &= ~DMA_COMPLETE;
            scsi_irq_ack = true;
        }

        if (val & DMA_RESET) {
            next_state->dma[NEXTDMA_SCSI].csr &= ~(DMA_COMPLETE | DMA_SUPDATE |
                                                  DMA_ENABLE);
            next_state->dma[NEXTDMA_SCSI].stage_len = 0;
            next_state->dma[NEXTDMA_SCSI].stage_flushes = 0;
            scsi_irq_ack = true;
            /* DPRINTF("SCSI DMA RESET\n"); */
        }
        /* DPRINTF("RXCSR \tWrite: %x\n",value); */
        break;

    case NEXTDMA_SCSI(NEXTDMA_NEXT):
        next_state->dma[NEXTDMA_SCSI].next = val;
        scsi_reg = true;
        break;

    case NEXTDMA_SCSI(NEXTDMA_LIMIT):
        next_state->dma[NEXTDMA_SCSI].limit = val;
        scsi_reg = true;
        break;

    case NEXTDMA_SCSI(NEXTDMA_START):
        next_state->dma[NEXTDMA_SCSI].start = val;
        scsi_reg = true;
        break;

    case NEXTDMA_SCSI(NEXTDMA_STOP):
        next_state->dma[NEXTDMA_SCSI].stop = val;
        scsi_reg = true;
        break;

    case NEXTDMA_SCSI(NEXTDMA_NEXT_INIT):
        next_state->dma[NEXTDMA_SCSI].next_initbuf = val;
        scsi_reg = true;
        break;

    default:
        DPRINTF("DMA write @ %x w/ %x\n", (unsigned)addr, (unsigned)val);
    }

    if (scsi_reg) {
        next_dma *dma = &next_state->dma[NEXTDMA_SCSI];

        trace_next_scsi_dma_reg_write(NEXT_DMA_BASE + addr, val, dma->csr,
                                      dma->next, dma->next_initbuf,
                                      dma->limit, dma->start, dma->stop);
    }
    if (scsi_irq_ack) {
        next_irq(opaque, NEXT_SCSI_DMA_I, 0);
    }
}

static uint64_t next_dma_read(void *opaque, hwaddr addr, unsigned int size)
{
    NeXTState *next_state = NEXT_MACHINE(qdev_get_machine());
    uint64_t val;
    bool scsi_reg = false;

    switch (addr) {
    case NEXTDMA_SCSI(NEXTDMA_CSR):
        DPRINTF("SCSI DMA CSR READ\n");
        val = next_state->dma[NEXTDMA_SCSI].csr;
        scsi_reg = true;
        break;

    case NEXTDMA_ENRX(NEXTDMA_CSR):
        val = next_state->dma[NEXTDMA_ENRX].csr;
        break;

    case NEXTDMA_ENRX(NEXTDMA_NEXT_INIT):
        val = next_state->dma[NEXTDMA_ENRX].next_initbuf;
        break;

    case NEXTDMA_ENRX(NEXTDMA_NEXT):
        val = next_state->dma[NEXTDMA_ENRX].next;
        break;

    case NEXTDMA_ENRX(NEXTDMA_LIMIT):
        val = next_state->dma[NEXTDMA_ENRX].limit;
        break;

    case NEXTDMA_SCSI(NEXTDMA_NEXT):
        val = next_state->dma[NEXTDMA_SCSI].next;
        scsi_reg = true;
        break;

    case NEXTDMA_SCSI(NEXTDMA_NEXT_INIT):
        val = next_state->dma[NEXTDMA_SCSI].next_initbuf;
        scsi_reg = true;
        break;

    case NEXTDMA_SCSI(NEXTDMA_LIMIT):
        val = next_state->dma[NEXTDMA_SCSI].limit;
        scsi_reg = true;
        break;

    case NEXTDMA_SCSI(NEXTDMA_START):
        val = next_state->dma[NEXTDMA_SCSI].start;
        scsi_reg = true;
        break;

    case NEXTDMA_SCSI(NEXTDMA_STOP):
        val = next_state->dma[NEXTDMA_SCSI].stop;
        scsi_reg = true;
        break;

    default:
        DPRINTF("DMA read @ %x\n", (unsigned int)addr);
        val = 0;
    }

    /*
     * once the csr's are done, subtract 0x3FEC from the addr, and that will
     * normalize the upper registers
     */

    if (scsi_reg &&
        trace_event_get_state_backends(TRACE_NEXT_SCSI_DMA_REG_READ)) {
        hwaddr trace_addr = NEXT_DMA_BASE + addr;

        if (next_trace_read_sample(&next_state->trace_scsi_dma_read,
                                   trace_addr, val)) {
            next_dma *dma = &next_state->dma[NEXTDMA_SCSI];

            trace_next_scsi_dma_reg_read(
                trace_addr, val, dma->csr, dma->next, dma->next_initbuf,
                dma->limit, dma->start, dma->stop,
                next_state->trace_scsi_dma_read.repeats);
        }
    }

    return val;
}

static const MemoryRegionOps next_dma_ops = {
    .read = next_dma_read,
    .write = next_dma_write,
    .impl.min_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_BIG_ENDIAN,
};

static void next_irq(void *opaque, int number, int level)
{
    NeXTPC *s = NEXT_PC(opaque);
    int shift = 0;

    /* first switch sets interrupt status */
    /* DPRINTF("IRQ %i\n",number); */
    switch (number) {
    /* level 3 - floppy, kbd/mouse, power, ether rx/tx, scsi, clock */
    case NEXT_FD_I:
        shift = 7;
        break;
    case NEXT_KBD_I:
        shift = 3;
        break;
    case NEXT_PWR_I:
        shift = 2;
        break;
    case NEXT_ENRX_I:
        shift = 9;
        break;
    case NEXT_ENTX_I:
        shift = 10;
        break;
    case NEXT_SCSI_I:
        shift = 12;
        break;
    case NEXT_CLK_I:
        shift = 29;
        break;

    /* level 5 - scc (serial) */
    case NEXT_SCC_I:
        shift = 17;
        break;

    /* level 6 - audio etherrx/tx dma */
    case NEXT_ENTX_DMA_I:
        shift = 28;
        break;
    case NEXT_ENRX_DMA_I:
        shift = 27;
        break;
    case NEXT_SCSI_DMA_I:
        shift = 26;
        break;
    case NEXT_SND_I:
        shift = 23;
        break;
    case NEXT_SCC_DMA_I:
        shift = 21;
        break;

    }
    if (level) {
        s->int_status |= 1U << shift;
    } else {
        s->int_status &= ~(1U << shift);
    }
    next_update_irq(s);

    if (number == NEXT_SCSI_I || number == NEXT_SCSI_DMA_I) {
        trace_next_scsi_irq(number == NEXT_SCSI_I ? "esp" : "dma", level,
                            s->int_status, s->int_mask, s->cpu->env.pc);
    }
}

static void next_timer_set_irq(NeXTPC *s, bool pending, bool force)
{
    int level = next_timer_irq_level(s);
    int vector = next_timer_irq_vector(s);

    if (!force && s->timer_irq_pending == pending) {
        return;
    }

    s->timer_irq_pending = pending;
    next_irq(s, NEXT_CLK_I, pending);
    trace_next_timer_irq(pending, level, vector, s->int_status, s->scr2);
}

static int nextdma_irq(int type)
{
    switch (type) {
    case NEXTDMA_SCSI:
        return NEXT_SCSI_DMA_I;
    default:
        return 0;
    }
}

static bool nextdma_segment_complete(void *opaque, next_dma *dma, int type)
{
    bool updating = dma->csr & DMA_SUPDATE;

    dma->csr |= DMA_COMPLETE;
    if (updating) {
        dma->csr &= ~DMA_SUPDATE;
        dma->next = dma->start;
        dma->limit = dma->stop;
    } else {
        dma->csr &= ~DMA_ENABLE;
    }

    next_irq(opaque, nextdma_irq(type), 1);
    return updating;
}

static bool nextdma_advance(void *opaque, next_dma *dma, int type,
                            uint32_t amount)
{
    dma->next += amount;
    if (dma->limit && dma->next == dma->limit) {
        return nextdma_segment_complete(opaque, dma, type);
    }
    return true;
}

static void nextdma_flush_stage(void *opaque, int type)
{
    NeXTState *next_state = NEXT_MACHINE(qdev_get_machine());
    next_dma *dma = &next_state->dma[type];
    uint8_t beat[16] = { 0 };
    int staged;

    if (!dma->stage_len || !dma->stage_flushes) {
        return;
    }
    if (--dma->stage_flushes) {
        return;
    }

    staged = dma->stage_len;
    memcpy(beat, dma->stage, staged);
    physical_memory_write(dma->next, beat, sizeof(beat));
    dma->stage_len = 0;
    nextdma_advance(opaque, dma, type, sizeof(beat));

    trace_next_scsi_dma_transfer(
        "flush", staged, sizeof(beat), dma->next - sizeof(beat),
        dma->csr, dma->next, dma->next_initbuf, dma->limit,
        dma->saved_next, dma->saved_limit);
}

static void nextdma_write(void *opaque, uint8_t *buf, int size, int type)
{
    uint32_t base_addr;
    uint32_t capacity;
    int committed = 0;
    int requested = size;
    NeXTState *next_state = NEXT_MACHINE(qdev_get_machine());
    next_dma *dma = &next_state->dma[type];

    if (!(dma->csr & DMA_ENABLE)) {
        trace_next_scsi_dma_transfer(
            "disabled", requested, committed, dma->next,
            dma->csr, dma->next, dma->next_initbuf, dma->limit,
            dma->saved_next, dma->saved_limit);
        return;
    }

    /*
     * prom sets the dma start using initbuf while the bootloader uses next
     * so we check to see if initbuf is 0
     */
    if (dma->next_initbuf == 0) {
        base_addr = dma->next;
    } else {
        base_addr = dma->next_initbuf;
        dma->next = base_addr;
        dma->next_initbuf = 0;
    }

    trace_next_scsi_dma_transfer(
        "entry", requested, committed, base_addr,
        dma->csr, dma->next, dma->next_initbuf, dma->limit,
        dma->saved_next, dma->saved_limit);

    while (size) {
        int chunk;

        if (dma->stage_len) {
            chunk = MIN(size, sizeof(dma->stage) - dma->stage_len);
            memcpy(dma->stage + dma->stage_len, buf, chunk);
            dma->stage_len += chunk;
            dma->stage_flushes = 4;
            buf += chunk;
            size -= chunk;
            if (dma->stage_len != sizeof(dma->stage)) {
                break;
            }

            physical_memory_write(dma->next, dma->stage,
                                  sizeof(dma->stage));
            dma->stage_len = 0;
            dma->stage_flushes = 0;
            committed += sizeof(dma->stage);
            if (!nextdma_advance(opaque, dma, type, sizeof(dma->stage)) &&
                size) {
                break;
            }
            continue;
        }

        if (size < sizeof(dma->stage)) {
            memcpy(dma->stage, buf, size);
            dma->stage_len = size;
            dma->stage_flushes = 4;
            size = 0;
            break;
        }

        capacity = dma->limit && dma->limit > dma->next ?
                   dma->limit - dma->next : size;
        chunk = MIN(size & ~(sizeof(dma->stage) - 1),
                    capacity & ~(sizeof(dma->stage) - 1));
        if (!chunk) {
            break;
        }
        physical_memory_write(dma->next, buf, chunk);
        buf += chunk;
        size -= chunk;
        committed += chunk;
        if (!nextdma_advance(opaque, dma, type, chunk) && size) {
            break;
        }
    }

    trace_next_scsi_dma_transfer(
        dma->stage_len ? "staged" : "complete", requested, committed,
        base_addr,
        dma->csr, dma->next, dma->next_initbuf, dma->limit,
        dma->saved_next, dma->saved_limit);
}

static void nextdma_read(void *opaque, uint8_t *buf, int size, int type)
{
    uint32_t base_addr;
    int requested = size;
    int transferred = 0;
    NeXTState *next_state = NEXT_MACHINE(qdev_get_machine());
    next_dma *dma = &next_state->dma[type];

    if (!(dma->csr & DMA_ENABLE)) {
        trace_next_scsi_dma_read(
            "disabled", requested, transferred, dma->next,
            dma->csr, dma->next, dma->next_initbuf, dma->limit,
            dma->saved_next, dma->saved_limit);
        return;
    }

    /*
     * The PROM uses initbuf while the boot loader and kernel use next.
     * Consume exactly the amount requested by ESP: unlike a DMA write into
     * guest memory, rounding here would overrun ESP's transfer buffer.
     */
    if (dma->next_initbuf == 0) {
        base_addr = dma->next;
    } else {
        base_addr = dma->next_initbuf;
        dma->next = base_addr;
        dma->next_initbuf = 0;
    }

    trace_next_scsi_dma_read(
        "entry", requested, transferred, base_addr,
        dma->csr, dma->next, dma->next_initbuf, dma->limit,
        dma->saved_next, dma->saved_limit);

    while (size) {
        uint32_t capacity = dma->limit && dma->limit > dma->next ?
                            dma->limit - dma->next : size;
        int chunk = MIN((uint32_t)size, capacity);

        if (!chunk) {
            break;
        }
        physical_memory_read(dma->next, buf, chunk);
        buf += chunk;
        size -= chunk;
        transferred += chunk;
        if (!nextdma_advance(opaque, dma, type, chunk) && size) {
            break;
        }
    }

    trace_next_scsi_dma_read(
        "complete", requested, transferred, base_addr,
        dma->csr, dma->next, dma->next_initbuf, dma->limit,
        dma->saved_next, dma->saved_limit);
}

static void nextscsi_read(void *opaque, uint8_t *buf, int len)
{
    DPRINTF("SCSI READ: %x\n", len);
    nextdma_read(opaque, buf, len, NEXTDMA_SCSI);
}

static void nextscsi_write(void *opaque, uint8_t *buf, int size)
{
    DPRINTF("SCSI WRITE: %i\n", size);
    nextdma_write(opaque, buf, size, NEXTDMA_SCSI);
}

static void next_scsi_csr_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size)
{
    NeXTSCSI *s = NEXT_SCSI(opaque);
    NeXTPC *pc = NEXT_PC(container_of(s, NeXTPC, next_scsi));
    uint8_t old;

    switch (addr) {
    case 0:
        old = s->scsi_csr_1;
        if (val & SCSICSR_FIFOFL) {
            DPRINTF("SCSICSR FIFO Flush\n");
            if (!(old & SCSICSR_FIFOFL)) {
                nextdma_flush_stage(pc, NEXTDMA_SCSI);
            }
        }

        if (val & SCSICSR_ENABLE) {
            DPRINTF("SCSICSR Enable\n");
            /*
             * qemu_irq_raise(s->scsi_dma);
             * s->scsi_csr_1 = 0xc0;
             * s->scsi_csr_1 |= 0x1;
             * qemu_irq_pulse(s->scsi_dma);
             */
        }
        /*
         * else
         *     s->scsi_csr_1 &= ~SCSICSR_ENABLE;
         */

        if (val & SCSICSR_RESET) {
            DPRINTF("SCSICSR Reset\n");
            /* I think this should set DMADIR. CPUDMA and INTMASK to 0 */
            qemu_irq_raise(pc->scsi_reset);
            s->scsi_csr_1 &= ~(SCSICSR_INTMASK | 0x80 | 0x1);
            qemu_irq_lower(pc->scsi_reset);
        }
        if (val & SCSICSR_DMADIR) {
            DPRINTF("SCSICSR DMAdir\n");
        }
        if (val & SCSICSR_CPUDMA) {
            DPRINTF("SCSICSR CPUDMA\n");
        }
        if (val & SCSICSR_INTMASK) {
            DPRINTF("SCSICSR INTMASK\n");
            /*
             * int_mask &= ~0x1000;
             * s->scsi_csr_1 |= val;
             * s->scsi_csr_1 &= ~SCSICSR_INTMASK;
             * if (s->scsi_queued) {
             *     s->scsi_queued = 0;
             *     next_irq(s, NEXT_SCSI_I, level);
             * }
             */
        } else {
            /* int_mask |= 0x1000; */
        }
        if (val & 0x80) {
            /* int_mask |= 0x1000; */
            /* s->scsi_csr_1 |= 0x80; */
        }
        DPRINTF("SCSICSR1 Write: %"PRIx64 "\n", val);
        trace_next_scsi_csr_write(
            NEXT_SCSI_CSR_BASE + addr, old, val,
            !!(val & SCSICSR_ENABLE),
            !!(val & SCSICSR_RESET), !!(val & SCSICSR_FIFOFL),
            !!(val & SCSICSR_DMADIR), !!(val & SCSICSR_CPUDMA),
            !!(val & SCSICSR_INTMASK));
        s->scsi_csr_1 = val;
        break;

    case 1:
        old = s->scsi_csr_2;
        DPRINTF("SCSICSR2 Write: %"PRIx64 "\n", val);
        trace_next_scsi_csr_write(
            NEXT_SCSI_CSR_BASE + addr, old, val,
            0, 0, 0, 0, 0, 0);
        s->scsi_csr_2 = val;
        break;

    default:
        g_assert_not_reached();
    }
}

static uint64_t next_scsi_csr_read(void *opaque, hwaddr addr, unsigned size)
{
    NeXTSCSI *s = NEXT_SCSI(opaque);
    uint64_t val;

    switch (addr) {
    case 0:
        DPRINTF("SCSI 4020  STATUS READ %X\n", s->scsi_csr_1);
        val = s->scsi_csr_1;
        break;

    case 1:
        DPRINTF("SCSI 4021 STATUS READ %X\n", s->scsi_csr_2);
        val = s->scsi_csr_2;
        break;

    default:
        g_assert_not_reached();
    }

    if (trace_event_get_state_backends(TRACE_NEXT_SCSI_CSR_READ)) {
        hwaddr trace_addr = NEXT_SCSI_CSR_BASE + addr;

        if (next_trace_read_sample(&s->trace_csr_read, trace_addr, val)) {
            trace_next_scsi_csr_read(trace_addr, val,
                                     s->trace_csr_read.repeats);
        }
    }

    return val;
}

static const MemoryRegionOps next_scsi_csr_ops = {
    .read = next_scsi_csr_read,
    .write = next_scsi_csr_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 1,
    .endianness = DEVICE_BIG_ENDIAN,
};

static void next_scsi_init(Object *obj)
{
    NeXTSCSI *s = NEXT_SCSI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    object_initialize_child(obj, "esp", &s->sysbus_esp, TYPE_SYSBUS_ESP);

    memory_region_init_io(&s->scsi_csr_mem, obj, &next_scsi_csr_ops,
                          s, "csrs", 2);

    memory_region_init(&s->scsi_mem, obj, "next.scsi", 0x40);
    sysbus_init_mmio(sbd, &s->scsi_mem);
}

static void next_scsi_realize(DeviceState *dev, Error **errp)
{
    NeXTSCSI *s = NEXT_SCSI(dev);
    SysBusESPState *sysbus_esp;
    SysBusDevice *sbd;
    ESPState *esp;
    NeXTPC *pcdev;

    pcdev = NEXT_PC(container_of(s, NeXTPC, next_scsi));

    /* ESP */
    sysbus_esp = SYSBUS_ESP(&s->sysbus_esp);
    esp = &sysbus_esp->esp;
    esp->dma_memory_read = nextscsi_read;
    esp->dma_memory_write = nextscsi_write;
    esp->dma_opaque = pcdev;
    sysbus_esp->it_shift = 0;
    esp->dma_enabled = 1;
    clock_set_hz(esp->clock, NEXT_ESP_CLOCK_HZ);
    sbd = SYS_BUS_DEVICE(sysbus_esp);
    if (!sysbus_realize(sbd, errp)) {
        return;
    }
    memory_region_add_subregion(&s->scsi_mem, 0x0,
                                sysbus_mmio_get_region(sbd, 0));

    /* SCSI CSRs */
    memory_region_add_subregion(&s->scsi_mem, NEXT_SCSI_CSR_OFFSET,
                                &s->scsi_csr_mem);

    scsi_bus_legacy_handle_cmdline(&s->sysbus_esp.esp.bus);
}

static const VMStateDescription next_scsi_vmstate = {
    .name = "next-scsi",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(scsi_csr_1, NeXTSCSI),
        VMSTATE_UINT8(scsi_csr_2, NeXTSCSI),
        VMSTATE_END_OF_LIST()
    },
};

static void next_scsi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "NeXT SCSI Controller";
    dc->realize = next_scsi_realize;
    dc->vmsd = &next_scsi_vmstate;
}

static const TypeInfo next_scsi_info = {
    .name = TYPE_NEXT_SCSI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_init = next_scsi_init,
    .instance_size = sizeof(NeXTSCSI),
    .class_init = next_scsi_class_init,
};

static void next_floppy_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    switch (addr) {
    case 0:
        DPRINTF("FDCSR Write: %"PRIx64 "\n", val);
        if (val == 0x0) {
            /* qemu_irq_raise(s->fd_irq[0]); */
        }
        break;

    default:
        g_assert_not_reached();
    }
}

static uint64_t next_floppy_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t val;

    switch (addr) {
    case 0:
        DPRINTF("FD read @ %x\n", (unsigned int)addr);
        val = 0x40 | 0x04 | 0x2 | 0x1;
        break;

    default:
        g_assert_not_reached();
    }

    return val;
}

static const MemoryRegionOps next_floppy_ops = {
    .read = next_floppy_read,
    .write = next_floppy_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .endianness = DEVICE_BIG_ENDIAN,
};

static uint32_t next_system_timer_remaining(NeXTPC *s)
{
    int64_t now;
    int64_t deadline;
    uint64_t remaining;

    if (!(s->timer_csr & NEXT_TIMER_ENABLE) ||
        !timer_pending(&s->system_timer)) {
        return s->timer_counter;
    }

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    deadline = timer_expire_time_ns(&s->system_timer);
    if (deadline <= now) {
        return 0;
    }

    remaining = DIV_ROUND_UP((uint64_t)(deadline - now),
                             NEXT_TIMER_TICK_NS);
    return MIN(remaining, (uint64_t)NEXT_TIMER_FULL_PERIOD);
}

static void next_system_timer_schedule(NeXTPC *s)
{
    int64_t now;

    timer_del(&s->system_timer);
    if (!(s->timer_csr & NEXT_TIMER_ENABLE) || !s->timer_counter) {
        return;
    }

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_mod(&s->system_timer,
              now + s->timer_counter * NEXT_TIMER_TICK_NS);
}

static void next_system_timer_expire(void *opaque)
{
    NeXTPC *s = opaque;

    s->timer_counter = 0;
    next_timer_set_irq(s, true, false);
}

static void next_system_timer_write(void *opaque, hwaddr addr, uint64_t val,
                                    unsigned size)
{
    NeXTPC *s = opaque;
    bool was_enabled;

    switch (addr) {
    case 0:
        s->timer_latch = deposit32(s->timer_latch, 8, 8, val);
        break;
    case 1:
        s->timer_latch = deposit32(s->timer_latch, 0, 8, val);
        break;
    case 2:
    case 3:
        break;
    case 4:
        was_enabled = s->timer_csr & NEXT_TIMER_ENABLE;
        s->timer_counter = next_system_timer_remaining(s);

        if (val & NEXT_TIMER_UPDATE) {
            s->timer_counter = s->timer_latch ?
                               s->timer_latch : NEXT_TIMER_FULL_PERIOD;
        }

        s->timer_csr = val & NEXT_TIMER_ENABLE;
        if (!(s->timer_csr & NEXT_TIMER_ENABLE)) {
            timer_del(&s->system_timer);
        } else if ((val & NEXT_TIMER_UPDATE) || !was_enabled) {
            next_system_timer_schedule(s);
        }
        break;
    default:
        g_assert_not_reached();
    }
}

static uint64_t next_system_timer_read(void *opaque, hwaddr addr,
                                       unsigned size)
{
    NeXTPC *s = opaque;
    uint32_t counter;
    uint8_t csr;

    switch (addr) {
    case 0:
        counter = next_system_timer_remaining(s);
        return extract32(counter, 8, 8);
    case 1:
        counter = next_system_timer_remaining(s);
        return extract32(counter, 0, 8);
    case 2:
    case 3:
        return 0;
    case 4:
        csr = s->timer_csr & NEXT_TIMER_ENABLE;
        if (s->timer_irq_pending) {
            next_timer_set_irq(s, false, false);
        }
        return csr;
    default:
        g_assert_not_reached();
    }
}

static const MemoryRegionOps next_system_timer_ops = {
    .read = next_system_timer_read,
    .write = next_system_timer_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 1,
    .endianness = DEVICE_BIG_ENDIAN,
};

static void next_eventc_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
}

static uint64_t next_eventc_read(void *opaque, hwaddr addr, unsigned size)
{
    NeXTPC *s = opaque;

    switch (addr) {
    case 0:
        s->eventc_latched =
            (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / NEXT_TIMER_TICK_NS) &
            NEXT_EVENTC_MASK;
        return 0;
    case 1:
        return extract32(s->eventc_latched, 16, 4);
    case 2:
        return extract32(s->eventc_latched, 8, 8);
    case 3:
        return extract32(s->eventc_latched, 0, 8);
    default:
        g_assert_not_reached();
    }
}

static const MemoryRegionOps next_eventc_ops = {
    .read = next_eventc_read,
    .write = next_eventc_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 1,
    .endianness = DEVICE_BIG_ENDIAN,
};

static void next_dummy_en_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size)
{
    /* Do nothing */
}

static uint64_t next_dummy_en_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t val;

    switch (addr) {
    case 0:
        /* For now return dummy byte to allow the Ethernet test to timeout */
        val = 0xff;
        break;

    default:
        val = 0;
    }

    return val;
}

static const MemoryRegionOps next_dummy_en_ops = {
    .read = next_dummy_en_read,
    .write = next_dummy_en_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .endianness = DEVICE_BIG_ENDIAN,
};

static void next_dsp_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    /* Do nothing */
}

static uint64_t next_dsp_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static const MemoryRegionOps next_dsp_ops = {
    .read = next_dsp_read,
    .write = next_dsp_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 1,
    .endianness = DEVICE_BIG_ENDIAN,
};

static void next_printer_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
    /* Do nothing */
}

static uint64_t next_printer_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static const MemoryRegionOps next_printer_ops = {
    .read = next_printer_read,
    .write = next_printer_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .endianness = DEVICE_BIG_ENDIAN,
};

static bool next_rtc_cmd_is_write(uint8_t cmd)
{
    return cmd & 0x80;
}

static uint32_t next_rtc_counter_value(NeXTRTC *rtc)
{
    int64_t elapsed_ns;

    if (!(rtc->control & NEXT_RTC_CONTROL_START)) {
        return rtc->counter;
    }

    elapsed_ns = qemu_clock_get_ns(rtc_clock) - rtc->counter_ref_ns;
    return rtc->counter + elapsed_ns / NANOSECONDS_PER_SECOND;
}

static void next_rtc_set_control(NeXTRTC *rtc, uint8_t value)
{
    bool was_running = rtc->control & NEXT_RTC_CONTROL_START;
    bool now_running = value & NEXT_RTC_CONTROL_START;
    int64_t now = qemu_clock_get_ns(rtc_clock);

    if (was_running && !now_running) {
        rtc->counter = next_rtc_counter_value(rtc);
    } else if (!was_running && now_running) {
        rtc->counter_ref_ns = now;
    }

    rtc->control = value & NEXT_RTC_CONTROL_STORED;
    if (value & NEXT_RTC_CONTROL_FTU_CLR) {
        rtc->status &= ~0x18;
        qemu_irq_lower(rtc->power_irq);
    }
    if (value & NEXT_RTC_CONTROL_ALARM_CLR) {
        rtc->status &= ~0x02;
    }
    if (value & NEXT_RTC_CONTROL_RPD_CLR) {
        rtc->status &= ~0x01;
    }
}

static void next_rtc_load_read_value(NeXTRTC *rtc)
{
    uint8_t addr = rtc->command & 0x3f;

    rtc->retval = 0;
    if (addr <= 0x1f) {
        rtc->retval = rtc->ram[addr];
    } else if (addr <= 0x23) {
        unsigned int shift = (0x23 - addr) * 8;

        if (addr == 0x20) {
            rtc->counter_latch = next_rtc_counter_value(rtc);
        }
        rtc->retval = rtc->counter_latch >> shift;
    } else if (addr <= 0x27) {
        unsigned int shift = (0x27 - addr) * 8;

        rtc->retval = rtc->alarm >> shift;
    } else if (addr == 0x30) {
        rtc->retval = rtc->status;
    } else if (addr == 0x31) {
        rtc->retval = rtc->control;
    }
}

static void next_rtc_store_write_value(NeXTRTC *rtc)
{
    uint8_t addr = rtc->command & 0x3f;

    if (addr <= 0x1f) {
        rtc->ram[addr] = rtc->value;
    } else if (addr <= 0x23) {
        unsigned int shift = (0x23 - addr) * 8;

        rtc->counter = deposit32(rtc->counter, shift, 8, rtc->value);
        rtc->counter_ref_ns = qemu_clock_get_ns(rtc_clock);
    } else if (addr <= 0x27) {
        unsigned int shift = (0x27 - addr) * 8;

        rtc->alarm = deposit32(rtc->alarm, shift, 8, rtc->value);
    } else if (addr == 0x31) {
        next_rtc_set_control(rtc, rtc->value);
    }
}

static void next_rtc_advance_byte(NeXTRTC *rtc)
{
    rtc->command = (rtc->command & 0x80) |
                   ((rtc->command + 1) & 0x3f);
    rtc->phase = 8;
    rtc->value = 0;
    if (!next_rtc_cmd_is_write(rtc->command)) {
        next_rtc_load_read_value(rtc);
    }
}

static void next_rtc_data_in_irq(void *opaque, int n, int level)
{
    NeXTRTC *rtc = NEXT_RTC(opaque);

    if (rtc->phase < 8) {
        rtc->command = (rtc->command << 1) | level;
        rtc->phase++;
        if (rtc->phase == 8 && !next_rtc_cmd_is_write(rtc->command)) {
            next_rtc_load_read_value(rtc);
        }
        return;
    }

    if (rtc->phase >= 8 && rtc->phase < 16) {
        if (next_rtc_cmd_is_write(rtc->command)) {
            rtc->value = (rtc->value << 1) | level;
        } else {
            if (rtc->retval & (0x80 >> (rtc->phase - 8))) {
                qemu_irq_raise(rtc->data_out_irq);
            } else {
                qemu_irq_lower(rtc->data_out_irq);
            }
        }
    }

    rtc->phase++;
    if (rtc->phase == 16) {
        if (next_rtc_cmd_is_write(rtc->command)) {
            next_rtc_store_write_value(rtc);
        }
        next_rtc_advance_byte(rtc);
    }
}

static void next_rtc_cmd_reset_irq(void *opaque, int n, int level)
{
    NeXTRTC *rtc = NEXT_RTC(opaque);

    if (level) {
        rtc->phase = 0;
        rtc->command = 0;
        rtc->value = 0;
    }
}

static void next_rtc_reset_hold(Object *obj, ResetType type)
{
    NeXTRTC *rtc = NEXT_RTC(obj);
    struct tm tm;

    rtc->status = NEXT_RTC_STATUS_NEW_CLOCK;
    rtc->control = NEXT_RTC_CONTROL_START;
    qemu_get_timedate(&tm, 0);
    rtc->counter = mktimegm(&tm);
    rtc->counter_latch = rtc->counter;
    rtc->counter_ref_ns = qemu_clock_get_ns(rtc_clock);
    rtc->alarm = 0;

    /* Load RTC RAM - TODO: provide possibility to load contents from file */
    memcpy(rtc->ram, rtc_ram2, 32);
}

static int next_rtc_pre_save(void *opaque)
{
    NeXTRTC *rtc = opaque;

    if (rtc->control & NEXT_RTC_CONTROL_START) {
        rtc->counter = next_rtc_counter_value(rtc);
        rtc->counter_ref_ns = qemu_clock_get_ns(rtc_clock);
    }
    return 0;
}

static int next_rtc_post_load(void *opaque, int version_id)
{
    NeXTRTC *rtc = opaque;

    if (version_id < 4) {
        struct tm tm;

        qemu_get_timedate(&tm, 0);
        rtc->counter = mktimegm(&tm);
        rtc->counter_latch = rtc->counter;
        rtc->alarm = 0;
    }
    rtc->counter_ref_ns = qemu_clock_get_ns(rtc_clock);
    return 0;
}

static void next_rtc_init(Object *obj)
{
    NeXTRTC *rtc = NEXT_RTC(obj);

    qdev_init_gpio_in_named(DEVICE(obj), next_rtc_data_in_irq,
                            "rtc-data-in", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &rtc->data_out_irq,
                             "rtc-data-out", 1);
    qdev_init_gpio_in_named(DEVICE(obj), next_rtc_cmd_reset_irq,
                            "rtc-cmd-reset", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &rtc->power_irq,
                             "rtc-power-out", 1);
}

static const VMStateDescription next_rtc_vmstate = {
    .name = "next-rtc",
    .version_id = 4,
    .minimum_version_id = 3,
    .pre_save = next_rtc_pre_save,
    .post_load = next_rtc_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_INT8(phase, NeXTRTC),
        VMSTATE_UINT8_ARRAY(ram, NeXTRTC, 32),
        VMSTATE_UINT8(command, NeXTRTC),
        VMSTATE_UINT8(value, NeXTRTC),
        VMSTATE_UINT8(status, NeXTRTC),
        VMSTATE_UINT8(control, NeXTRTC),
        VMSTATE_UINT8(retval, NeXTRTC),
        VMSTATE_UINT32_V(counter, NeXTRTC, 4),
        VMSTATE_UINT32_V(counter_latch, NeXTRTC, 4),
        VMSTATE_UINT32_V(alarm, NeXTRTC, 4),
        VMSTATE_END_OF_LIST()
    },
};

static void next_rtc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "NeXT RTC";
    dc->vmsd = &next_rtc_vmstate;
    rc->phases.hold = next_rtc_reset_hold;
}

static const TypeInfo next_rtc_info = {
    .name = TYPE_NEXT_RTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_init = next_rtc_init,
    .instance_size = sizeof(NeXTRTC),
    .class_init = next_rtc_class_init,
};

static void next_pc_rtc_data_in_irq(void *opaque, int n, int level)
{
    NeXTPC *s = NEXT_PC(opaque);
    uint8_t scr2_2 = extract32(s->scr2, 8, 8);

    if (level) {
        scr2_2 |= SCR2_RTDATA;
    } else {
        scr2_2 &= ~SCR2_RTDATA;
    }

    s->scr2 = deposit32(s->scr2, 8, 8, scr2_2);
}

static void next_pc_reset_hold(Object *obj, ResetType type)
{
    NeXTPC *s = NEXT_PC(obj);

    timer_del(&s->system_timer);

    /* Set internal registers to initial values */
    /*     0x0000XX00 << vital bits */
    s->scr1 = 0x00011102;
    s->scr2 = 0x00ff0c80;
    s->old_scr2 = s->scr2;
    s->timer_latch = 0;
    s->timer_counter = 0;
    s->timer_csr = 0;
    s->eventc_latched = 0;
    next_scr2_update_softints(s);
    next_timer_set_irq(s, false, true);
}

static void next_pc_realize(DeviceState *dev, Error **errp)
{
    NeXTPC *s = NEXT_PC(dev);
    SysBusDevice *sbd;
    DeviceState *d;

    /* SCSI */
    sbd = SYS_BUS_DEVICE(&s->next_scsi);
    if (!sysbus_realize(sbd, errp)) {
        return;
    }

    d = DEVICE(object_resolve_path_component(OBJECT(&s->next_scsi), "esp"));
    sysbus_connect_irq(SYS_BUS_DEVICE(d), 0,
                       qdev_get_gpio_in(DEVICE(s), NEXT_SCSI_I));

    s->scsi_reset = qdev_get_gpio_in(d, 0);
    s->scsi_dma = qdev_get_gpio_in(d, 1);

    /* ESCC */
    d = DEVICE(&s->escc);
    qdev_prop_set_uint32(d, "disabled", 0);
    qdev_prop_set_uint32(d, "frequency", 9600 * 384);
    qdev_prop_set_uint32(d, "it_shift", 0);
    qdev_prop_set_bit(d, "bit_swap", true);
    qdev_prop_set_chr(d, "chrB", serial_hd(1));
    qdev_prop_set_chr(d, "chrA", serial_hd(0));
    qdev_prop_set_uint32(d, "chnBtype", escc_serial);
    qdev_prop_set_uint32(d, "chnAtype", escc_serial);

    sbd = SYS_BUS_DEVICE(d);
    if (!sysbus_realize(sbd, errp)) {
        return;
    }
    sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(dev, NEXT_SCC_I));
    sysbus_connect_irq(sbd, 1, qdev_get_gpio_in(dev, NEXT_SCC_DMA_I));

    /* RTC */
    d = DEVICE(&s->rtc);
    if (!sysbus_realize(SYS_BUS_DEVICE(d), errp)) {
        return;
    }
    /* Data from NeXTPC to RTC */
    qdev_connect_gpio_out_named(dev, "rtc-data-out", 0,
                                qdev_get_gpio_in_named(d, "rtc-data-in", 0));
    /* Data from RTC to NeXTPC */
    qdev_connect_gpio_out_named(d, "rtc-data-out", 0,
                                qdev_get_gpio_in_named(dev,
                                                       "rtc-data-in", 0));
    qdev_connect_gpio_out_named(dev, "rtc-cmd-reset", 0,
                                qdev_get_gpio_in_named(d, "rtc-cmd-reset", 0));
    qdev_connect_gpio_out_named(d, "rtc-power-out", 0,
                                qdev_get_gpio_in(dev, NEXT_PWR_I));
}

static void next_pc_init(Object *obj)
{
    NeXTPC *s = NEXT_PC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    qdev_init_gpio_in(DEVICE(obj), next_irq, NEXT_NUM_IRQS);

    memory_region_init_io(&s->mmiomem, OBJECT(s), &next_mmio_ops, s,
                          "next.mmio", 0x9000);
    sysbus_init_mmio(sbd, &s->mmiomem);

    memory_region_init_io(&s->dummyen_mem, OBJECT(s), &next_dummy_en_ops, s,
                          "next.en", 0x20);
    sysbus_init_mmio(sbd, &s->dummyen_mem);

    memory_region_init_io(&s->dsp_mem, OBJECT(s), &next_dsp_ops, s,
                          "next.dsp", 8);
    sysbus_init_mmio(sbd, &s->dsp_mem);

    memory_region_init_io(&s->printer_mem, OBJECT(s), &next_printer_ops, s,
                          "next.printer", 8);
    sysbus_init_mmio(sbd, &s->printer_mem);

    object_initialize_child(obj, "next-scsi", &s->next_scsi, TYPE_NEXT_SCSI);
    sysbus_init_mmio(sbd,
                     sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->next_scsi), 0));

    memory_region_init_io(&s->floppy_mem, OBJECT(s), &next_floppy_ops, s,
                          "next.floppy", 4);
    sysbus_init_mmio(sbd, &s->floppy_mem);

    object_initialize_child(obj, "escc", &s->escc, TYPE_ESCC);
    sysbus_init_mmio(sbd,
                     sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->escc), 0));

    timer_init_ns(&s->system_timer, QEMU_CLOCK_VIRTUAL,
                  next_system_timer_expire, s);

    memory_region_init_io(&s->system_timer_mem, OBJECT(s),
                          &next_system_timer_ops, s,
                          "next.system-timer", 5);
    sysbus_init_mmio(sbd, &s->system_timer_mem);

    memory_region_init_io(&s->eventc_mem, OBJECT(s), &next_eventc_ops, s,
                          "next.event-counter", 4);
    sysbus_init_mmio(sbd, &s->eventc_mem);

    object_initialize_child(obj, "rtc", &s->rtc, TYPE_NEXT_RTC);

    qdev_init_gpio_in_named(DEVICE(obj), next_pc_rtc_data_in_irq,
                            "rtc-data-in", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->rtc_data_irq,
                             "rtc-data-out", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->rtc_cmd_reset_irq,
                             "rtc-cmd-reset", 1);
}

/*
 * If the m68k CPU implemented its inbound irq lines as GPIO lines
 * rather than via the m68k_set_irq_level() function we would not need
 * this cpu link property and could instead provide outbound IRQ lines
 * that the board could wire up to the CPU.
 */
static const Property next_pc_properties[] = {
    DEFINE_PROP_LINK("cpu", NeXTPC, cpu, TYPE_M68K_CPU, M68kCPU *),
};

static int next_pc_post_load(void *opaque, int version_id)
{
    NeXTPC *s = opaque;

    next_scr2_update_softints(s);

    if (version_id < 5) {
        timer_del(&s->system_timer);
        s->timer_latch = 0;
        s->timer_counter = 0;
        s->timer_csr = 0;
        s->eventc_latched = 0;
        s->timer_irq_pending = false;
        s->int_status &= ~NEXT_TIMER_IRQ_STATUS;
        next_update_irq(s);
        return 0;
    }

    if (s->timer_counter > NEXT_TIMER_FULL_PERIOD) {
        return -EINVAL;
    }

    s->timer_csr &= NEXT_TIMER_ENABLE;
    s->eventc_latched &= NEXT_EVENTC_MASK;
    if (!(s->timer_csr & NEXT_TIMER_ENABLE) || !s->timer_counter) {
        timer_del(&s->system_timer);
    } else if (!timer_pending(&s->system_timer)) {
        next_system_timer_schedule(s);
    }

    if (s->timer_irq_pending) {
        next_timer_set_irq(s, true, true);
    } else {
        s->timer_irq_pending = false;
        s->int_status &= ~NEXT_TIMER_IRQ_STATUS;
        next_update_irq(s);
    }
    return 0;
}

static const VMStateDescription next_pc_vmstate = {
    .name = "next-pc",
    .version_id = 5,
    .minimum_version_id = 4,
    .post_load = next_pc_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(scr1, NeXTPC),
        VMSTATE_UINT32(scr2, NeXTPC),
        VMSTATE_UINT32(old_scr2, NeXTPC),
        VMSTATE_UINT32(int_mask, NeXTPC),
        VMSTATE_UINT32(int_status, NeXTPC),
        VMSTATE_UINT32(led, NeXTPC),
        VMSTATE_TIMER_V(system_timer, NeXTPC, 5),
        VMSTATE_UINT16_V(timer_latch, NeXTPC, 5),
        VMSTATE_UINT32_V(timer_counter, NeXTPC, 5),
        VMSTATE_UINT8_V(timer_csr, NeXTPC, 5),
        VMSTATE_BOOL_V(timer_irq_pending, NeXTPC, 5),
        VMSTATE_UINT32_V(eventc_latched, NeXTPC, 5),
        VMSTATE_END_OF_LIST()
    },
};

static void next_pc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "NeXT Peripheral Controller";
    dc->realize = next_pc_realize;
    device_class_set_props(dc, next_pc_properties);
    dc->vmsd = &next_pc_vmstate;
    rc->phases.hold = next_pc_reset_hold;
}

static const TypeInfo next_pc_info = {
    .name = TYPE_NEXT_PC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_init = next_pc_init,
    .instance_size = sizeof(NeXTPC),
    .class_init = next_pc_class_init,
};

static void next_cube_init(MachineState *machine)
{
    NeXTState *m = NEXT_MACHINE(machine);
    M68kCPU *cpu;
    CPUM68KState *env;
    MemoryRegion *sysmem = get_system_memory();
    const char *bios_name = machine->firmware ?: ROM_FILE;
    DeviceState *pcdev;

    /* Initialize the cpu core */
    cpu = M68K_CPU(cpu_create(machine->cpu_type));
    if (!cpu) {
        error_report("Unable to find m68k CPU definition");
        exit(1);
    }
    env = &cpu->env;

    /* Initialize CPU registers.  */
    env->vbr = 0;
    env->sr  = 0x2700;

    /* Peripheral Controller */
    pcdev = qdev_new(TYPE_NEXT_PC);
    object_property_set_link(OBJECT(pcdev), "cpu", OBJECT(cpu), &error_abort);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pcdev), &error_fatal);

    /* 64MB RAM starting at 0x04000000  */
    memory_region_add_subregion(sysmem, 0x04000000, machine->ram);

    /* Framebuffer */
    sysbus_create_simple(TYPE_NEXTFB, 0x0B000000, NULL);

    /* MMIO */
    sysbus_mmio_map(SYS_BUS_DEVICE(pcdev), 0, 0x02005000);

    /* en network (dummy) */
    sysbus_mmio_map(SYS_BUS_DEVICE(pcdev), 1, 0x02106000);

    /* DSP host interface */
    sysbus_mmio_map(SYS_BUS_DEVICE(pcdev), 2, 0x02108000);

    /* Printer interface */
    sysbus_mmio_map(SYS_BUS_DEVICE(pcdev), 3, 0x0200f000);

    /* unknown: Brightness control register? */
    empty_slot_init("next.unknown.0", 0x02110000, 0x10);
    /* unknown: Magneto-Optical drive controller? */
    empty_slot_init("next.unknown.1", 0x02112000, 0x10);

    /* SCSI */
    sysbus_mmio_map(SYS_BUS_DEVICE(pcdev), 4, NEXT_SCSI_BASE);
    /* Floppy */
    sysbus_mmio_map(SYS_BUS_DEVICE(pcdev), 5, 0x02114108);
    /* ESCC */
    sysbus_mmio_map(SYS_BUS_DEVICE(pcdev), 6, 0x02118000);

    /* unknown: Serial clock configuration register? */
    empty_slot_init("next.unknown.2", 0x02118004, 0x10);

    /* System timer and event counter */
    sysbus_mmio_map(SYS_BUS_DEVICE(pcdev), 7, 0x02116000);
    sysbus_mmio_map(SYS_BUS_DEVICE(pcdev), 8, 0x0211a000);

    /* BMAP memory */
    memory_region_init_ram_flags_nomigrate(&m->bmapm1, NULL, "next.bmapmem",
                                           64, RAM_SHARED, &error_fatal);
    memory_region_add_subregion(sysmem, 0x020c0000, &m->bmapm1);
    /* The Rev_2.5_v66.bin firmware accesses it at 0x820c0020, too */
    memory_region_init_alias(&m->bmapm2, NULL, "next.bmapmem2", &m->bmapm1,
                             0x0, 64);
    memory_region_add_subregion(sysmem, 0x820c0000, &m->bmapm2);

    /* KBD */
    sysbus_create_simple(TYPE_NEXTKBD, 0x0200e000, NULL);

    /* Load ROM here */
    memory_region_init_rom(&m->rom, NULL, "next.rom", 0x20000, &error_fatal);
    memory_region_add_subregion(sysmem, 0x01000000, &m->rom);
    memory_region_init_alias(&m->rom2, NULL, "next.rom2", &m->rom, 0x0,
                             0x20000);
    memory_region_add_subregion(sysmem, 0x0, &m->rom2);
    Error *local_err = NULL;
    if (load_image_targphys(bios_name, 0x01000000, 0x20000, &local_err) < 8) {
        if (!qtest_enabled()) {
            if (local_err) {
                error_report_err(local_err);
            } else {
                error_report("Firmware image '%s' is too short.", bios_name);
            }
        } else {
            error_free(local_err);
        }
    } else {
        uint8_t *ptr;
        /* Initial PC is always at offset 4 in firmware binaries */
        ptr = rom_ptr(0x01000004, 4);
        g_assert(ptr != NULL);
        env->pc = ldl_be_p(ptr);
        if (env->pc >= 0x01020000) {
            error_report("'%s' does not seem to be a valid firmware image.",
                         bios_name);
            exit(1);
        }
    }

    /* DMA */
    memory_region_init_io(&m->dmamem, NULL, &next_dma_ops, pcdev,
                          "next.dma", 0x5000);
    memory_region_add_subregion(sysmem, NEXT_DMA_BASE, &m->dmamem);
}

static void next_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "NeXT Cube";
    mc->init = next_cube_init;
    mc->block_default_type = IF_SCSI;
    mc->default_ram_size = RAM_SIZE;
    mc->default_ram_id = "next.ram";
    mc->default_cpu_type = M68K_CPU_TYPE_NAME("m68040");
    mc->no_cdrom = true;
}

static const TypeInfo next_typeinfo = {
    .name = TYPE_NEXT_MACHINE,
    .parent = TYPE_MACHINE,
    .class_init = next_machine_class_init,
    .instance_size = sizeof(NeXTState),
};

static void next_register_type(void)
{
    type_register_static(&next_typeinfo);
    type_register_static(&next_pc_info);
    type_register_static(&next_scsi_info);
    type_register_static(&next_rtc_info);
}

type_init(next_register_type)
