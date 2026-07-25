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
#include "system/system.h"
#include "system/qtest.h"
#include "hw/core/irq.h"
#include "hw/m68k/next-cube.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/audio/next-sound.h"
#include "hw/dma/next-dma.h"
#include "hw/display/next-color-video.h"
#include "hw/display/next-fb.h"
#include "hw/misc/next-memctl.h"
#include "hw/misc/next-nbic.h"
#include "hw/net/next-mb8795.h"
#include "hw/rtc/next-rtc.h"
#include "hw/scsi/esp.h"
#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "qom/object.h"
#include "hw/char/next-serial.h"
#include "hw/block/fdc.h"
#include "hw/block/next-floppy.h"
#include "hw/isa/isa.h"
#include "hw/misc/empty_slot.h"
#include "hw/core/qdev-properties.h"
#include "qapi/error.h"
#include "qapi/util.h"
#include "qemu/error-report.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "ui/console.h"
#include "target/m68k/cpu.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "trace.h"

/* #define DEBUG_NEXT */
#ifdef DEBUG_NEXT
#define DPRINTF(fmt, ...) \
    do { printf("NeXT: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do { } while (0)
#endif

#define ENTRY       0x0100001e

#define NEXT_DMA_BASE        0x02000000
#define NEXT_NBIC_BASE       0x02020000
#define NEXT_SCSI_ROM_CSR_BASE 0x02014020
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

#define NEXT_IRQ_IPL7_MASK      0xc0000000
#define NEXT_IRQ_IPL6_MASK      0x3ffc0000
#define NEXT_IRQ_IPL5_MASK      0x00038000
#define NEXT_IRQ_IPL4_MASK      0x00004000
#define NEXT_IRQ_IPL3_MASK      0x00003ffc
#define NEXT_IRQ_IPL2_MASK      0x00000002
#define NEXT_IRQ_IPL1_MASK      0x00000001


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

    NextDMAState *dma;

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
    NextDMAState *dma;

    MemoryRegion system_timer_mem;
    MemoryRegion eventc_mem;
    MemoryRegion dsp_mem;
    MemoryRegion printer_mem;
    MemoryRegion mmiomem;
    MemoryRegion scrmem;
    MemoryRegion scsi_csr_rom_alias;

    uint32_t scr1;
    uint32_t scr1_reset;
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

    NeXTRTC rtc;
    qemu_irq rtc_data_irq;
    qemu_irq rtc_cmd_reset_irq;
};

#define TYPE_NEXT_MACHINE "next-machine"
#define TYPE_NEXT_CUBE_MACHINE MACHINE_TYPE_NAME("next-cube")
#define TYPE_NEXT_STATION_MACHINE MACHINE_TYPE_NAME("next-station")
#define TYPE_NEXT_STATION_COLOR_MACHINE \
    MACHINE_TYPE_NAME("next-station-color")

typedef enum NeXTVideoKind {
    NEXT_VIDEO_MONO,
    NEXT_VIDEO_COLOR,
} NeXTVideoKind;

typedef enum NeXTDiskMuxKind {
    NEXT_DISK_MUX_FLPCTL,
    NEXT_DISK_MUX_CUBE_OD,
} NeXTDiskMuxKind;

typedef struct NeXTBoardProfile {
    const char *product_name;
    const char *default_bios;
    uint8_t slot_id;
    uint8_t dma_revision;
    uint8_t machine_type;
    uint8_t board_revision;
    uint8_t video_memory_speed;
    uint8_t main_memory_speed;
    uint8_t cpu_clock;
    ram_addr_t default_ram_size;
    ram_addr_t maximum_ram_size;
    NeXTVideoKind video_kind;
    NeXTDiskMuxKind disk_mux_kind;
    bool has_nextbus;
} NeXTBoardProfile;

typedef struct NeXTMachineClass {
    MachineClass parent_class;

    const NeXTBoardProfile *profile;
} NeXTMachineClass;

OBJECT_DECLARE_TYPE(NeXTState, NeXTMachineClass, NEXT_MACHINE)

struct NeXTState {
    MachineState parent;

    MemoryRegion rom;
    MemoryRegion rom2;
    MemoryRegion bmapm1;
    MemoryRegion bmapm2;

    NextDMAState *dma;
    NextSoundState *sound;
    NextMB8795State *mb8795;
    char *nvram_file;
    NextRTCChip rtc_chip;
    bool rtc_chip_locked;
};

static const NeXTBoardProfile next_cube_profile = {
    .product_name = "NeXTcube (68040, X15)",
    .default_bios = "Rev_2.5_v66.bin",
    .dma_revision = 1,
    .machine_type = 2,
    .board_revision = 0,
    .cpu_clock = 2,
    .default_ram_size = 64 * MiB,
    .maximum_ram_size = 64 * MiB,
    .video_kind = NEXT_VIDEO_MONO,
    .disk_mux_kind = NEXT_DISK_MUX_FLPCTL,
    .has_nextbus = true,
};

static const NeXTBoardProfile next_station_profile = {
    .product_name = "NeXTstation (Warp 9)",
    .default_bios = "Rev_2.5_v66.bin",
    .dma_revision = 1,
    .machine_type = 1,
    .board_revision = 0,
    .cpu_clock = 2,
    .default_ram_size = 64 * MiB,
    .maximum_ram_size = 64 * MiB,
    .video_kind = NEXT_VIDEO_MONO,
    .disk_mux_kind = NEXT_DISK_MUX_FLPCTL,
    .has_nextbus = false,
};

static const NeXTBoardProfile next_station_color_profile = {
    .product_name = "NeXTstation Color (Warp 9C)",
    .default_bios = "Rev_2.5_v66.bin",
    .dma_revision = 1,
    .machine_type = 3,
    .board_revision = 0,
    .cpu_clock = 2,
    .default_ram_size = 32 * MiB,
    .maximum_ram_size = 32 * MiB,
    .video_kind = NEXT_VIDEO_COLOR,
    .disk_mux_kind = NEXT_DISK_MUX_FLPCTL,
    .has_nextbus = false,
};

static uint32_t next_profile_scr1(const NeXTBoardProfile *profile)
{
    return ((uint32_t)(profile->slot_id & 0xf) << 28) |
           ((uint32_t)profile->dma_revision << 16) |
           ((uint32_t)(profile->machine_type & 0xf) << 12) |
           ((uint32_t)(profile->board_revision & 0xf) << 8) |
           ((uint32_t)(profile->video_memory_speed & 0x3) << 6) |
           ((uint32_t)(profile->main_memory_speed & 0x3) << 4) |
           (uint32_t)(profile->cpu_clock & 0x3);
}

static const QEnumLookup next_machine_rtc_chip_lookup = {
    .array = (const char *const[]) {
        [NEXT_RTC_CHIP_MCS1850] = "mcs1850",
        [NEXT_RTC_CHIP_MC68HC68T1] = "mc68hc68t1",
    },
    .size = NEXT_RTC_CHIP__MAX,
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

static void next_irq(void *opaque, int number, int level)
{
    NeXTPC *s = NEXT_PC(opaque);
    int shift;

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
    case NEXT_SOUND_OVRUN_I:
        shift = 8;
        break;
    case NEXT_VIDEO_I:
        shift = 5;
        break;
    case NEXT_C16_VIDEO_I:
        shift = 13;
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
    case NEXT_OPTICAL_DMA_I:
        shift = 25;
        break;
    case NEXT_PRINTER_DMA_I:
        shift = 24;
        break;
    case NEXT_SOUND_OUT_DMA_I:
        shift = 23;
        break;
    case NEXT_SOUND_IN_DMA_I:
        shift = 22;
        break;
    case NEXT_SCC_DMA_I:
        shift = 21;
        break;
    case NEXT_DSP_DMA_I:
        shift = 20;
        break;
    case NEXT_M2R_DMA_I:
        shift = 19;
        break;
    case NEXT_R2M_DMA_I:
        shift = 18;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unknown interrupt input %d\n",
                      __func__, number);
        return;
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

static void nextscsi_read(void *opaque, uint8_t *buf, int len)
{
    DPRINTF("SCSI READ: %x\n", len);
    next_dma_scsi_read(opaque, buf, len);
}

static void nextscsi_write(void *opaque, uint8_t *buf, int size)
{
    DPRINTF("SCSI WRITE: %i\n", size);
    next_dma_scsi_write(opaque, buf, size);
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
        if (val & SCSICSR_RESET) {
            DPRINTF("SCSICSR Reset\n");
            /* I think this should set DMADIR. CPUDMA and INTMASK to 0 */
            next_dma_scsi_fifo_reset(s->dma);
            qemu_irq_raise(pc->scsi_reset);
            s->scsi_csr_1 &= ~(SCSICSR_INTMASK | 0x80 | 0x1);
            qemu_irq_lower(pc->scsi_reset);
        }
        if (val & SCSICSR_FIFOFL) {
            DPRINTF("SCSICSR FIFO Flush\n");
            if (!(old & SCSICSR_FIFOFL)) {
                next_dma_scsi_fifo_flush(s->dma);
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
        next_dma_set_scsi_control(s->dma, val);
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

    if (!s->dma) {
        error_setg(errp, "'dma' link is not set");
        return;
    }

    /* ESP */
    sysbus_esp = SYSBUS_ESP(&s->sysbus_esp);
    esp = &sysbus_esp->esp;
    esp->dma_memory_read = nextscsi_read;
    esp->dma_memory_write = nextscsi_write;
    esp->dma_opaque = s->dma;
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

static void next_scsi_reset(DeviceState *dev)
{
    NeXTSCSI *s = NEXT_SCSI(dev);

    s->scsi_csr_1 = 0;
    s->scsi_csr_2 = 0;
    next_dma_set_scsi_control(s->dma, 0);
}

static int next_scsi_post_load(void *opaque, int version_id)
{
    NeXTSCSI *s = opaque;

    next_dma_set_scsi_control(s->dma, s->scsi_csr_1);
    return 0;
}

static const VMStateDescription next_scsi_vmstate = {
    .name = "next-scsi",
    .version_id = 0,
    .minimum_version_id = 0,
    .post_load = next_scsi_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(scsi_csr_1, NeXTSCSI),
        VMSTATE_UINT8(scsi_csr_2, NeXTSCSI),
        VMSTATE_END_OF_LIST()
    },
};

static const Property next_scsi_properties[] = {
    DEFINE_PROP_LINK("dma", NeXTSCSI, dma, TYPE_NEXT_DMA, NextDMAState *),
};

static void next_scsi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "NeXT SCSI Controller";
    dc->realize = next_scsi_realize;
    device_class_set_legacy_reset(dc, next_scsi_reset);
    device_class_set_props(dc, next_scsi_properties);
    dc->vmsd = &next_scsi_vmstate;
}

static const TypeInfo next_scsi_info = {
    .name = TYPE_NEXT_SCSI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_init = next_scsi_init,
    .instance_size = sizeof(NeXTSCSI),
    .class_init = next_scsi_class_init,
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

    /*
     * Expiry reloads the counter from the latch.  NeXT kernels load the
     * immediate deadline, start the timer, then leave 0xffff in the latch
     * as the fallback period until the interrupt handler programs it again.
     */
    s->timer_counter = s->timer_latch ?
                       s->timer_latch : NEXT_TIMER_FULL_PERIOD;
    next_system_timer_schedule(s);
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
    int64_t now;

    switch (addr) {
    case 0:
        now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->eventc_latched = (now / NEXT_TIMER_TICK_NS) & NEXT_EVENTC_MASK;
        trace_next_eventc_latch(s->eventc_latched, now);
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
    s->scr1 = s->scr1_reset;
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

    if (!s->dma) {
        error_setg(errp, "'dma' link is not set");
        return;
    }
    if (!object_property_set_link(OBJECT(&s->next_scsi), "dma",
                                  OBJECT(s->dma), errp)) {
        return;
    }

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

    memory_region_init_io(&s->dsp_mem, OBJECT(s), &next_dsp_ops, s,
                          "next.dsp", 8);
    sysbus_init_mmio(sbd, &s->dsp_mem);

    memory_region_init_io(&s->printer_mem, OBJECT(s), &next_printer_ops, s,
                          "next.printer", 8);
    sysbus_init_mmio(sbd, &s->printer_mem);

    object_initialize_child(obj, "next-scsi", &s->next_scsi, TYPE_NEXT_SCSI);
    sysbus_init_mmio(sbd,
                     sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->next_scsi), 0));

    timer_init_ns(&s->system_timer, QEMU_CLOCK_VIRTUAL,
                  next_system_timer_expire, s);

    memory_region_init_io(&s->system_timer_mem, OBJECT(s),
                          &next_system_timer_ops, s,
                          "next.system-timer", 5);
    sysbus_init_mmio(sbd, &s->system_timer_mem);

    memory_region_init_io(&s->eventc_mem, OBJECT(s), &next_eventc_ops, s,
                          "next.event-counter", 4);
    sysbus_init_mmio(sbd, &s->eventc_mem);

    memory_region_init_alias(&s->scsi_csr_rom_alias, OBJECT(s),
                             "next.scsi-csr-rom-alias",
                             &s->next_scsi.scsi_csr_mem, 0, 2);
    sysbus_init_mmio(sbd, &s->scsi_csr_rom_alias);

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
    DEFINE_PROP_LINK("dma", NeXTPC, dma, TYPE_NEXT_DMA, NextDMAState *),
    DEFINE_PROP_UINT32("scr1-reset", NeXTPC, scr1_reset, 0x00011002),
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

static void next_machine_create_fdc_and_flpctl(
    MachineState *machine G_GNUC_UNUSED, NeXTState *m, DeviceState *pcdev)
{
    DeviceState *fdc_dev;
    DeviceState *floppy_ctrl_dev;
    DriveInfo *fds[MAX_FD];

    /* The 82077 and ESP share the physical SCSI DMA channel. */
    fds[0] = drive_get(IF_FLOPPY, 0, 0);
    fds[1] = drive_get(IF_FLOPPY, 0, 1);
    fdc_dev = fdctrl_init_sysbus_dma(
        qdev_get_gpio_in(pcdev, NEXT_FD_I), 0x02114100, fds,
        ISADMA(m->dma), NEXT_DMA_SCSI, true);

    floppy_ctrl_dev = qdev_new(TYPE_NEXT_FLOPPY_CTRL);
    object_property_set_link(OBJECT(floppy_ctrl_dev), "fdc",
                             OBJECT(fdc_dev), &error_abort);
    object_property_set_link(OBJECT(floppy_ctrl_dev), "dma",
                             OBJECT(m->dma), &error_abort);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(floppy_ctrl_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(floppy_ctrl_dev), 0, 0x02114108);
}

static void next_machine_init(MachineState *machine)
{
    static const int dma_irq_inputs[NEXT_DMA_CHANNEL_COUNT] = {
        [NEXT_DMA_SCSI] = NEXT_SCSI_DMA_I,
        [NEXT_DMA_SOUND_OUT] = NEXT_SOUND_OUT_DMA_I,
        [NEXT_DMA_OPTICAL] = NEXT_OPTICAL_DMA_I,
        [NEXT_DMA_SOUND_IN] = NEXT_SOUND_IN_DMA_I,
        [NEXT_DMA_PRINTER] = NEXT_PRINTER_DMA_I,
        [NEXT_DMA_SCC] = NEXT_SCC_DMA_I,
        [NEXT_DMA_DSP] = NEXT_DSP_DMA_I,
        [NEXT_DMA_ENTX] = NEXT_ENTX_DMA_I,
        [NEXT_DMA_ENRX] = NEXT_ENRX_DMA_I,
        [NEXT_DMA_VIDEO] = NEXT_VIDEO_I,
        [NEXT_DMA_R2M] = NEXT_R2M_DMA_I,
        [NEXT_DMA_M2R] = NEXT_M2R_DMA_I,
    };
    const NeXTBoardProfile *profile =
        NEXT_MACHINE_GET_CLASS(machine)->profile;
    NeXTState *m = NEXT_MACHINE(machine);
    M68kCPU *cpu;
    CPUM68KState *env;
    MemoryRegion *sysmem = get_system_memory();
    const char *bios_name = machine->firmware ?: profile->default_bios;
    DeviceState *dma_dev;
    DeviceState *kbd_dev;
    DeviceState *mbdev;
    DeviceState *memctl_dev;
    DeviceState *nbic_dev;
    DeviceState *pcdev;
    DeviceState *serial_dev;
    DeviceState *sound_dev;
    int channel;

    if (machine->ram_size > profile->maximum_ram_size) {
        error_report("%s supports at most %" PRIu64 " MiB of RAM",
                     profile->product_name,
                     profile->maximum_ram_size / MiB);
        exit(EXIT_FAILURE);
    }

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

    /* DMA */
    dma_dev = qdev_new(TYPE_NEXT_DMA);
    m->dma = NEXT_DMA(dma_dev);
    object_property_add_child(OBJECT(machine), "next-dma", OBJECT(dma_dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dma_dev), &error_fatal);

    /* Peripheral Controller */
    pcdev = qdev_new(TYPE_NEXT_PC);
    object_property_set_link(OBJECT(pcdev), "cpu", OBJECT(cpu), &error_abort);
    object_property_set_link(OBJECT(pcdev), "dma", OBJECT(m->dma),
                             &error_abort);
    qdev_prop_set_uint32(pcdev, "scr1-reset", next_profile_scr1(profile));
    if (m->nvram_file && m->nvram_file[0]) {
        qdev_prop_set_string(DEVICE(&NEXT_PC(pcdev)->rtc), "nvram-file",
                             m->nvram_file);
    }
    object_property_set_str(OBJECT(&NEXT_PC(pcdev)->rtc), "rtc-chip",
                            qapi_enum_lookup(&next_machine_rtc_chip_lookup,
                                             m->rtc_chip),
                            &error_abort);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pcdev), &error_fatal);
    m->rtc_chip_locked = true;

    sysbus_mmio_map(SYS_BUS_DEVICE(m->dma), 0, NEXT_DMA_BASE);
    for (channel = 0; channel < NEXT_DMA_CHANNEL_COUNT; channel++) {
        if (dma_irq_inputs[channel] >= 0) {
            sysbus_connect_irq(SYS_BUS_DEVICE(m->dma), channel,
                               qdev_get_gpio_in(pcdev,
                                                dma_irq_inputs[channel]));
        }
    }

    switch (profile->disk_mux_kind) {
    case NEXT_DISK_MUX_FLPCTL:
        next_machine_create_fdc_and_flpctl(machine, m, pcdev);
        break;
    case NEXT_DISK_MUX_CUBE_OD:
    default:
        g_assert_not_reached();
    }

    /* Serial ports and clock select */
    serial_dev = qdev_new(TYPE_NEXT_SERIAL);
    qdev_prop_set_chr(serial_dev, "chrA", serial_hd(0));
    qdev_prop_set_chr(serial_dev, "chrB", serial_hd(1));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(serial_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(serial_dev), 0, 0x02118000);
    sysbus_connect_irq(SYS_BUS_DEVICE(serial_dev), 0,
                       qdev_get_gpio_in(pcdev, NEXT_SCC_I));

    /* Sound output */
    sound_dev = qdev_new(TYPE_NEXT_SOUND);
    m->sound = NEXT_SOUND(sound_dev);
    object_property_add_child(OBJECT(machine), "next-sound",
                              OBJECT(sound_dev));
    object_property_set_link(OBJECT(sound_dev), "dma", OBJECT(m->dma),
                             &error_abort);
    if (machine->audiodev) {
        qdev_prop_set_string(sound_dev, "audiodev", machine->audiodev);
    }
    qdev_realize_and_unref(sound_dev, NULL, &error_fatal);
    qdev_connect_gpio_out(sound_dev, 0,
                          qdev_get_gpio_in(pcdev, NEXT_SOUND_OVRUN_I));

    /* Ethernet controller */
    mbdev = qdev_new(TYPE_NEXT_MB8795);
    m->mb8795 = NEXT_MB8795(mbdev);
    object_property_add_child(OBJECT(machine), "mb8795", OBJECT(mbdev));
    object_property_set_link(OBJECT(mbdev), "dma",
                             OBJECT(m->dma), &error_abort);
    qemu_configure_nic_device(mbdev, true, NULL);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(mbdev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(m->mb8795), 0, 0x02106000);
    sysbus_connect_irq(SYS_BUS_DEVICE(m->mb8795), 0,
                       qdev_get_gpio_in(pcdev, NEXT_ENTX_I));
    sysbus_connect_irq(SYS_BUS_DEVICE(m->mb8795), 1,
                       qdev_get_gpio_in(pcdev, NEXT_ENRX_I));
    next_dma_set_ethernet_notify(m->dma, &next_mb8795_dma_notify,
                                 m->mb8795);

    /* Memory timing registers */
    memctl_dev = qdev_new(TYPE_NEXT_MEMCTL);
    object_property_add_child(OBJECT(machine), "memctl",
                              OBJECT(memctl_dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(memctl_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(memctl_dev), 0, 0x02106010);

    if (profile->has_nextbus) {
        nbic_dev = qdev_new(TYPE_NEXT_NBIC);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(nbic_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(nbic_dev), 0, NEXT_NBIC_BASE);
    }

    /* RAM starting at 0x04000000 */
    memory_region_add_subregion(sysmem, 0x04000000, machine->ram);

    /* Framebuffer */
    if (profile->video_kind == NEXT_VIDEO_MONO) {
        sysbus_create_simple(TYPE_NEXTFB, 0x0B000000, NULL);
    } else {
        DeviceState *color_video_dev = qdev_new(TYPE_NEXT_COLOR_VIDEO);
        SysBusDevice *color_video_sbd = SYS_BUS_DEVICE(color_video_dev);

        sysbus_realize_and_unref(color_video_sbd, &error_fatal);
        sysbus_mmio_map(color_video_sbd, 0, 0x2c000000);
        sysbus_mmio_map(color_video_sbd, 1, 0x02118100);
        sysbus_mmio_map(color_video_sbd, 2, 0x02118180);
        sysbus_mmio_map(color_video_sbd, 3, 0x02118190);
        sysbus_mmio_map(color_video_sbd, 4, 0x02118198);
        sysbus_connect_irq(color_video_sbd, 0,
                           qdev_get_gpio_in(pcdev, NEXT_C16_VIDEO_I));
    }

    /* MMIO */
    sysbus_mmio_map(SYS_BUS_DEVICE(pcdev), 0, 0x02005000);

    /* DSP host interface */
    sysbus_mmio_map(SYS_BUS_DEVICE(pcdev), 1, 0x02108000);

    /* Printer interface */
    sysbus_mmio_map(SYS_BUS_DEVICE(pcdev), 2, 0x0200f000);

    /* unknown: Brightness control register? */
    empty_slot_init("next.unknown.0", 0x02110000, 0x10);
    /* unknown: Magneto-Optical drive controller? */
    empty_slot_init("next.unknown.1", 0x02112000, 0x10);

    /* SCSI */
    sysbus_mmio_map(SYS_BUS_DEVICE(pcdev), 3, NEXT_SCSI_BASE);
    /* System timer and event counter */
    sysbus_mmio_map(SYS_BUS_DEVICE(pcdev), 4, 0x02116000);
    sysbus_mmio_map(SYS_BUS_DEVICE(pcdev), 5, 0x0211a000);
    /* The v66 ROM and NeXT floppy driver use this SCSI CSR decode. */
    sysbus_mmio_map(SYS_BUS_DEVICE(pcdev), 6, NEXT_SCSI_ROM_CSR_BASE);

    /* BMAP memory */
    memory_region_init_ram_flags_nomigrate(&m->bmapm1, NULL, "next.bmapmem",
                                           64, RAM_SHARED, &error_fatal);
    memory_region_add_subregion(sysmem, 0x020c0000, &m->bmapm1);
    /* The Rev_2.5_v66.bin firmware accesses it at 0x820c0020, too */
    memory_region_init_alias(&m->bmapm2, NULL, "next.bmapmem2", &m->bmapm1,
                             0x0, 64);
    memory_region_add_subregion(sysmem, 0x820c0000, &m->bmapm2);

    /* Monitor keyboard, mouse, and sound command interface */
    kbd_dev = qdev_new(TYPE_NEXTKBD);
    object_property_set_link(OBJECT(kbd_dev), "sound", OBJECT(m->sound),
                             &error_abort);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(kbd_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(kbd_dev), 0, 0x0200e000);
    sysbus_connect_irq(SYS_BUS_DEVICE(kbd_dev), 0,
                       qdev_get_gpio_in(pcdev, NEXT_KBD_I));

    /* Load ROM here */
    memory_region_init_rom(&m->rom, NULL, "next.rom", 0x20000, &error_fatal);
    memory_region_add_subregion(sysmem, 0x01000000, &m->rom);
    memory_region_init_alias(&m->rom2, NULL, "next.rom2", &m->rom, 0x0,
                             0x20000);
    memory_region_add_subregion(sysmem, 0x0, &m->rom2);
    Error *local_err = NULL;
    if (load_image_targphys(bios_name, 0x01000000, 0x20000, &local_err) < 8) {
        if (!qtest_enabled()) {
            error_report("Could not load ROM image '%s'", bios_name);
            if (local_err) {
                error_report_err(local_err);
            } else {
                error_report("Firmware image '%s' is too short.", bios_name);
            }
            exit(EXIT_FAILURE);
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
}

static char *next_machine_get_nvram_file(Object *obj, Error **errp)
{
    return g_strdup(NEXT_MACHINE(obj)->nvram_file);
}

static void next_machine_set_nvram_file(Object *obj, const char *value,
                                        Error **errp)
{
    NeXTState *s = NEXT_MACHINE(obj);

    g_free(s->nvram_file);
    s->nvram_file = g_strdup(value);
}

static int next_machine_get_rtc_chip(Object *obj, Error **errp G_GNUC_UNUSED)
{
    return NEXT_MACHINE(obj)->rtc_chip;
}

static void next_machine_set_rtc_chip(Object *obj, int value, Error **errp)
{
    NeXTState *s = NEXT_MACHINE(obj);

    if (s->rtc_chip_locked) {
        error_setg(errp, "rtc-chip cannot be changed after machine init");
        return;
    }
    if (value < 0 || value >= NEXT_RTC_CHIP__MAX) {
        error_setg(errp, "invalid rtc-chip value %d", value);
        return;
    }
    s->rtc_chip = value;
}

static void next_machine_finalize(Object *obj)
{
    NeXTState *s = NEXT_MACHINE(obj);

    g_free(s->nvram_file);
}

static void next_machine_class_init(ObjectClass *oc, const void *data)
{
    ObjectProperty *prop;

    object_class_property_add_str(oc, "nvram-file",
                                  next_machine_get_nvram_file,
                                  next_machine_set_nvram_file);
    object_class_property_set_description(
        oc, "nvram-file", "Path to the persistent 32-byte NeXT NVRAM image");
    prop = object_class_property_add_enum(oc, "rtc-chip", "NextRTCChip",
                                          &next_machine_rtc_chip_lookup,
                                          next_machine_get_rtc_chip,
                                          next_machine_set_rtc_chip);
    object_property_set_default_str(prop, "mcs1850");
    object_class_property_set_description(
        oc, "rtc-chip", "NeXT RTC chip model (mcs1850 or mc68hc68t1)");
}

static const char * const next_040_cpu_types[] = {
    M68K_CPU_TYPE_NAME("m68040"),
    NULL,
};

static void next_machine_common_class_init(
    ObjectClass *oc, const NeXTBoardProfile *profile, const char *description)
{
    NeXTMachineClass *nmc = NEXT_MACHINE_CLASS(oc);
    MachineClass *mc = MACHINE_CLASS(oc);

    nmc->profile = profile;
    mc->desc = description;
    mc->init = next_machine_init;
    mc->block_default_type = IF_SCSI;
    mc->default_ram_size = profile->default_ram_size;
    mc->default_ram_id = "next.ram";
    mc->default_cpu_type = M68K_CPU_TYPE_NAME("m68040");
    mc->valid_cpu_types = next_040_cpu_types;
    mc->default_nic = TYPE_NEXT_MB8795;
    mc->no_cdrom = true;
    machine_add_audiodev_property(mc);
}

static void next_cube_machine_class_init(ObjectClass *oc, const void *data)
{
    next_machine_common_class_init(oc, &next_cube_profile,
                                   "NeXTcube (68040, X15)");
}

static void next_station_machine_class_init(ObjectClass *oc, const void *data)
{
    next_machine_common_class_init(oc, &next_station_profile,
                                   "NeXTstation (Warp 9)");
}

static void next_station_color_machine_class_init(ObjectClass *oc,
                                                  const void *data)
{
    next_machine_common_class_init(oc, &next_station_color_profile,
                                   "NeXTstation Color (Warp 9C)");
}

static const TypeInfo next_machine_typeinfo = {
    .name = TYPE_NEXT_MACHINE,
    .parent = TYPE_MACHINE,
    .abstract = true,
    .class_init = next_machine_class_init,
    .class_size = sizeof(NeXTMachineClass),
    .instance_size = sizeof(NeXTState),
    .instance_finalize = next_machine_finalize,
};

static const TypeInfo next_cube_machine_typeinfo = {
    .name = TYPE_NEXT_CUBE_MACHINE,
    .parent = TYPE_NEXT_MACHINE,
    .class_init = next_cube_machine_class_init,
};

static const TypeInfo next_station_machine_typeinfo = {
    .name = TYPE_NEXT_STATION_MACHINE,
    .parent = TYPE_NEXT_MACHINE,
    .class_init = next_station_machine_class_init,
};

static const TypeInfo next_station_color_machine_typeinfo = {
    .name = TYPE_NEXT_STATION_COLOR_MACHINE,
    .parent = TYPE_NEXT_MACHINE,
    .class_init = next_station_color_machine_class_init,
};

static void next_register_type(void)
{
    type_register_static(&next_machine_typeinfo);
    type_register_static(&next_cube_machine_typeinfo);
    type_register_static(&next_station_machine_typeinfo);
    type_register_static(&next_station_color_machine_typeinfo);
    type_register_static(&next_pc_info);
    type_register_static(&next_scsi_info);
}

type_init(next_register_type)
