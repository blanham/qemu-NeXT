#include "sysbus.h"
#include "scsi.h"
#include "esp-new.h"

/* debug ESP card */
#define DEBUG_ESP

#ifdef DEBUG_ESP
#define DPRINTF(fmt, ...)                                       \
        do { printf("ESP: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

#define ESP_ERROR(fmt, ...)                                             \
        do { printf("ESP ERROR: %s: " fmt, __func__ , ## __VA_ARGS__); } while (0)

#define ESP_REGS 16
#define TI_BUFSZ 16

typedef struct ESP2State ESP2State;

struct ESP2State {
        SysBusDevice busdev;
        uint32_t it_shift;
        qemu_irq irq;
        uint8_t rregs[ESP_REGS];
        uint8_t wregs[ESP_REGS];
        /* new vars */
        int irq_status;
        uint8_t fifo_buf[TI_BUFSZ];
        uint8_t fifo_rptr, fifo_wptr, fifo_size;
        uint16_t transfer_count;
        /* will reuse some of these */
        int32_t ti_size;
        uint32_t ti_rptr, ti_wptr;
        uint8_t ti_buf[TI_BUFSZ];
        /*   */
        uint32_t status;
        uint32_t dma;
        SCSIBus bus;
        SCSIDevice *current_dev;
        SCSIRequest *current_req;
        /*  */
        uint8_t cmdbuf[TI_BUFSZ];
        uint32_t cmdlen;
        uint32_t do_cmd;

        /* The amount of data left in the current DMA transfer.  */
        uint32_t dma_left;
        /* The size of the current DMA transfer.  Zero if no transfer is in
           progress.  */
        uint32_t dma_counter;
        uint8_t *async_buf;
        uint32_t async_len;

        ESP2DMAMemoryReadWriteFunc dma_memory_read;
        ESP2DMAMemoryReadWriteFunc dma_memory_write;
        void *dma_opaque;
        int dma_enabled;
        void (*dma_cb)(ESP2State *s);
};
/* R/W Registers */
#define ESP_TCLO   0x0
#define ESP_TCMID  0x1
#define ESP_FIFO   0x2
#define ESP_CMD    0x3
#define ESP_CFG1   0x8
#define ESP_CFG2   0xb
#define ESP_CFG3   0xc
#define ESP_RES3   0xd
#define ESP_TCHI   0xe
#define ESP_RES4   0xf
/* Read-only registers */
#define ESP_RSTAT  0x4
#define ESP_RINTR  0x5
#define ESP_RSEQ   0x6
#define ESP_RFLAGS 0x7
#define ESP_RRES1  0x9
#define ESP_RRES2  0xa
/* Write-only registers */
#define ESP_WBUSID 0x4
#define ESP_WSEL   0x5
#define ESP_WSYNTP 0x6
#define ESP_WSYNO  0x7
#define ESP_WCCF   0x9
#define ESP_WTEST  0xa

#define CMD_DMA 0x80
#define CMD_CMD 0x7f

#define CMD_NOP      0x00
#define CMD_FLUSH    0x01
#define CMD_RESET    0x02
#define CMD_BUSRESET 0x03
#define CMD_TI       0x10
#define CMD_ICCS     0x11
#define CMD_MSGACC   0x12
#define CMD_PAD      0x18
#define CMD_SATN     0x1a
#define CMD_SEL      0x41
#define CMD_SELATN   0x42
#define CMD_SELATNS  0x43
#define CMD_ENSEL    0x44

#define STAT_DO 0x00
#define STAT_DI 0x01
#define STAT_CD 0x02
#define STAT_ST 0x03
#define STAT_MO 0x06
#define STAT_MI 0x07
#define STAT_PIO_MASK 0x06

#define STAT_TC 0x10
#define STAT_PE 0x20
#define STAT_GE 0x40
#define STAT_INT 0x80

#define BUSID_DID 0x07

#define INTR_FC 0x08
#define INTR_BS 0x10
#define INTR_DC 0x20
#define INTR_RST 0x80

#define SEQ_0 0x0
#define SEQ_CD 0x4

#define CFG1_RESREPT 0x40

#define TCHI_FAS100A 0x4

/* wrapper functions that set the interrupt register */
static void esp_raise_irq(ESP2State *s)
{
    if (!(s->rregs[ESP_RSTAT] & STAT_INT)) {
        /* set the INT bit in status register */
        s->rregs[ESP_RSTAT] |= STAT_INT;
        s->irq_status = 1;
        qemu_irq_raise(s->irq);
        DPRINTF("Raise IRQ\n");
    }
}

static void esp_lower_irq(ESP2State *s)
{
    if (s->rregs[ESP_RSTAT] & STAT_INT) {
        /* clear INT bit in status register */
        s->rregs[ESP_RSTAT] &= ~STAT_INT;
        s->irq_status = 0;
        qemu_irq_lower(s->irq);
        DPRINTF("Lower IRQ\n");
    }
}

static void esp_dma_enable(void *opaque, int irq, int level)
{
    DeviceState *d = opaque;
    ESP2State *s = container_of(d, ESP2State, busdev.qdev);

    if (level) {
        s->dma_enabled = 1;
        DPRINTF("Raise enable\n");
        /* if we have queued dma, do the callback */
        if (s->dma_cb) {
            s->dma_cb(s);
            s->dma_cb = NULL;
        }
    } else {
        DPRINTF("Lower enable\n");
        s->dma_enabled = 0;
    }
}




static void esp_hard_reset(DeviceState *d)
{
    ESP2State *s = container_of(d, ESP2State, busdev.qdev);
    
    /* set all registers to 0 */
    memset(s->rregs, 0, ESP_REGS);
    memset(s->wregs, 0, ESP_REGS);
    
    /* FAS100A identity, need to look at other docs to see
     * if this is necessary for other chips */
    s->rregs[ESP_TCHI] = TCHI_FAS100A;
    
    /* clear fifo */
    s->fifo_size = s->fifo_wptr = s->fifo_rptr = 0;
    
    /* non-dma mode */
    s->dma = 0;
   
    /* reset transfer count */
    s->transfer_count = 0;
     
    //s->do_cmd = 0;
    //s->dma_cb = NULL;
    
    /* DERP? */
    s->rregs[ESP_CFG1] = 7;
}

static void esp_soft_reset(DeviceState *d)
{
    ESP2State *s = container_of(d, ESP2State, busdev.qdev);

    qemu_irq_lower(s->irq);
    //esp_hard_reset(d);//<- WTF?
    /* set the Reset interrupt bit */
    s->rregs[ESP_RINTR] = INTR_RST;
}

static void parent_esp_reset(void *opaque, int irq, int level)
{
    if (level) {
        esp_soft_reset(opaque);
    }
}
static void esp_gpio_demux(void *opaque, int irq, int level)
{
    switch (irq) {
        case 0:
            parent_esp_reset(opaque, irq, level);
            break;
        case 1:
            esp_dma_enable(opaque, irq, level);
            break;
    }
}
static uint32_t get_cmd(ESP2State *s, uint8_t *buf)
{
    uint32_t dmalen = 0;
    int target = 0;

 /*   target = s->wregs[ESP_WBUSID] & BUSID_DID;
    if (s->dma) {
        dmalen = s->rregs[ESP_TCLO] | (s->rregs[ESP_TCMID] << 8);
        s->dma_memory_read(s->dma_opaque, buf, dmalen);
    } else {
        dmalen = s->ti_size;
        memcpy(buf, s->ti_buf, dmalen);
        buf[0] = buf[2] >> 5;
    }
    DPRINTF("get_cmd: len %d target %d\n", dmalen, target);

    s->ti_size = 0;
    s->ti_rptr = 0;
    s->ti_wptr = 0;*/
    /* zero FIFO count */
    //s->rregs[ESP_RFLAGS] &= 0xe0;
    
    if (s->current_req) {
        /* Started a new command before the old one finished.  Cancel it.  */
        //scsi_req_cancel(s->current_req);
        //s->async_len = 0;
    }

    if (target >= ESP_MAX_DEVS || !s->bus.devs[target]) {
        // No such drive
        s->rregs[ESP_RSTAT] = 0;
        s->rregs[ESP_RINTR] = INTR_DC;
        s->rregs[ESP_RSEQ] = SEQ_0;
        esp_raise_irq(s);
        return 0;
    }
    //s->current_dev = s->bus.devs[target];
    return dmalen;
}
static void do_busid_cmd(ESP2State *s, uint8_t *buf, uint8_t busid)
{
    //int32_t datalen;
    //int lun;

    DPRINTF("do_busid_cmd: busid 0x%x\n", busid);
    //lun = busid & 7;
    //s->current_req = scsi_req_new(s->current_dev, 0, lun, NULL);
    //datalen = scsi_req_enqueue(s->current_req, buf);
    //s->ti_size = datalen;
    //if (datalen != 0) {
   //     s->rregs[ESP_RSTAT] = STAT_TC;
    //    s->dma_left = 0;
    //    s->dma_counter = 0;
    //    if (datalen > 0) {
    //        s->rregs[ESP_RSTAT] |= STAT_DI;
    //    } else {
    //        s->rregs[ESP_RSTAT] |= STAT_DO;
    //    }
   //     scsi_req_continue(s->current_req);
   // }
   // s->rregs[ESP_RINTR] = INTR_BS | INTR_FC;
   // s->rregs[ESP_RSEQ] = SEQ_CD;
   // esp_raise_irq(s);
}

static void do_cmd(ESP2State *s, uint8_t *buf)
{
    uint8_t busid = buf[0];

    do_busid_cmd(s, &buf[1], busid);
}

static void handle_satn(ESP2State *s)
{
    uint8_t buf[32];
    int len;

    if (!s->dma_enabled) {
        s->dma_cb = handle_satn;
        return;
    }
    len = get_cmd(s, buf);
    if (len)
        do_cmd(s, buf);
}

static void handle_s_without_atn(ESP2State *s)
{
    uint8_t buf[32];
    int len;

    if (!s->dma_enabled) {
        s->dma_cb = handle_s_without_atn;
        return;
    }
    len = get_cmd(s, buf);
    if (len) {
        do_busid_cmd(s, buf, 0);
    }
}

static void handle_satn_stop(ESP2State *s)
{
    if (!s->dma_enabled) {
        s->dma_cb = handle_satn_stop;
        return;
    }
    s->cmdlen = get_cmd(s, s->cmdbuf);
    if (s->cmdlen) {
        DPRINTF("Set ATN & Stop: cmdlen %d\n", s->cmdlen);
        s->do_cmd = 1;
        s->rregs[ESP_RSTAT] = STAT_TC | STAT_CD;
        s->rregs[ESP_RINTR] = INTR_BS | INTR_FC;
        s->rregs[ESP_RSEQ] = SEQ_CD;
        esp_raise_irq(s);
    }
}

static uint32_t esp_mem_readb(void *opaque, target_phys_addr_t addr)
{
    ESP2State *s = (ESP2State *)opaque;
    uint32_t saddr;
    uint32_t old_val;
    saddr = addr >> s->it_shift;
    DPRINTF("read reg[%d]: 0x%2.2x\n", saddr, s->rregs[saddr]);
    switch(saddr)    
    {
        case ESP_TCLO:
                DPRINTF("Read TCLO\n");
                break;
        case ESP_TCMID:
                DPRINTF("Read TCMID\n");
                break;
        case ESP_FIFO:
            DPRINTF("Read FIFO\n");
            if (s->fifo_size > 0) {
                s->fifo_size--;
                if ((s->rregs[ESP_RSTAT] & STAT_PIO_MASK) == 0) {
                    /* Data out.  */
                    ESP_ERROR("PIO data read not implemented\n");
                    s->rregs[ESP_FIFO] = 0;
                } else {
                    s->rregs[ESP_FIFO] = s->fifo_buf[s->fifo_rptr++];
                }   
                esp_raise_irq(s);
            }
        
            if (s->fifo_size == 0) {
                s->fifo_rptr = 0;
                s->fifo_wptr = 0;
                /* zero out FIFO count */
                s->rregs[ESP_RFLAGS] &= 0xe0;
            }
            break;
        case ESP_CMD:
                DPRINTF("Read CMD\n");
                break;
        case ESP_RSTAT:
                DPRINTF("Read RSTAT\n");
                break;
        case ESP_RINTR:
            if(s->irq_status == 1){
                old_val = s->rregs[ESP_RINTR];
                s->rregs[ESP_RINTR] = 0;
                s->rregs[ESP_RSTAT] &= ~STAT_TC;
                s->rregs[ESP_RSEQ] = SEQ_CD;
                esp_lower_irq(s);
                s->irq_status = 0;
                return old_val;
            }
            break;
        case ESP_RSEQ:
                DPRINTF("Read RESQ\n");
                break;
        case ESP_RFLAGS:
                DPRINTF("Read RFLAGS\n");
                break;
        case ESP_RRES1:
                DPRINTF("Read RRES1\n");
                break;
        case ESP_RRES2:
                DPRINTF("Read RRES2\n");
                break;

        default:
                DPRINTF("readb @ %x\n",addr);
    }

    return s->rregs[saddr];
}
static void esp_mem_writeb(void *opaque, target_phys_addr_t addr, uint32_t val)
{
    ESP2State *s = (ESP2State *)opaque;
    uint32_t saddr;

    saddr = addr >> s->it_shift;
    DPRINTF("write reg[%d]: 0x%2.2x -> 0x%2.2x\n", saddr, s->wregs[saddr],
            val);
    switch (saddr) {
        case ESP_TCLO:
        case ESP_TCMID:
                /* in theory any write to these registers clears the
                 * transfer count bit, as 0 represents maximum transfer
                 * length (65536 bytes)
                 */
                s->rregs[ESP_RSTAT] &= ~STAT_TC;
                break;
        case ESP_FIFO:
            if (s->fifo_size == TI_BUFSZ - 1) {
                ESP_ERROR("fifo overrun\n");
            } else {
                /* fifo has another byte in it */
                s->fifo_size++;
                /* write value to fifo buffer */
                s->fifo_buf[s->fifo_wptr++] = val & 0xff;
                /* set FIFO count in RFLAGS */
                s->rregs[ESP_RFLAGS] = (s->rregs[ESP_RFLAGS]&0xe0) | (s->ti_wptr);
            }
            break;
        case ESP_CMD:
            /* next comand is a dma command if val & CMD_DMA */
            s->dma = (val & CMD_DMA ? 1 : 0); 
            
            switch(val & CMD_CMD)
            {   
                case CMD_NOP:
                    DPRINTF("NOP (%x)\n",val);
                    break; 
                case CMD_FLUSH:
                    DPRINTF("Fifo Flush (%x)\n",val);
                    /* should read pointer also be set to zero? */
                    s->fifo_wptr = 0;
                    s->fifo_size = 0;
    
                    /* dunno if this is right either */ 
                    s->rregs[ESP_RINTR] = INTR_FC;
                    
                    /* not sure if this command follows the datasheet */
                    //s->rregs[ESP_RSEQ] = 0;
                    
                    /* clear fifo count in fifo flags register */
                    s->rregs[ESP_RFLAGS] &= 0xe0;
                    break;
                case CMD_RESET:
                    DPRINTF("Reset (%x)\n",val);
                    /* might want to rewrite this function, I'm not completely
                     * certain of the relationship between external reset interrupt
                     * an this reset */
                    esp_soft_reset(&s->busdev.qdev);
                    break;
                case CMD_BUSRESET:
                    DPRINTF("Bus Reset (%x)\n",val);
                    
                    /* clear transfer count zero */
                    s->rregs[ESP_RSTAT] &= ~(STAT_TC);

                    /* also needs: reset dma, reset seq step
                    *clear sequence bits Enable,resel, Target, Initiator
                    *set FIFO to empty
                    */

                    /* this could be a seperate function */
                    s->rregs[ESP_RINTR] = INTR_RST;
                    if (!(s->wregs[ESP_CFG1] & CFG1_RESREPT)) {
                        esp_raise_irq(s);
                    }
                    break;
                case CMD_SEL:
                    DPRINTF("Select without ATN (%x)\n",val);
                    handle_s_without_atn(s);
                    break;
                case CMD_SELATN:
                    DPRINTF("Select with ATN (%x)\n",val);
                    handle_satn(s);
                    break;
                case CMD_SELATNS:
                    DPRINTF("Select with ATN and Stop (%x)\n",val);
                    handle_satn_stop(s);
                    break;
                default:
                    ESP_ERROR("Unhandled ESP command (%2.2x)\n", val);
                    abort();
                    break;
            }
            break;
        case ESP_WBUSID:
                break;
        case ESP_WSEL:
                DPRINTF("Write SEL %x\n",val);
                break;
        case ESP_WSYNTP:
        case ESP_WSYNO:
            /* I imagine that these registers are unneeded, given the level
             * of scsi emulation in qemu (we don't emulate REQ/ACK cycles) 
             */    
            break;
        case ESP_CFG1:
            s->rregs[saddr] = val;
            break;
        case ESP_WCCF:
            /* Don't need this since we don't do exact timings */
            break;
        case ESP_WTEST:
            ESP_ERROR("ESP TEST command unimplemented (%2.2x)\n", val);
            return;

        default:
            ESP_ERROR("invalid write of 0x%02x at [0x%x]\n", val, saddr);
            return;
    }
    
    s->wregs[saddr] = val;
}


static CPUReadMemoryFunc * const esp_mem_read[3] = {
    esp_mem_readb,
    NULL,
    NULL
};

static CPUWriteMemoryFunc * const esp_mem_write[3] = {
    esp_mem_writeb,
    NULL,
    esp_mem_writeb
};

static const VMStateDescription vmstate_esp = {
    .name ="esp-new",
    .version_id = 3,
    .minimum_version_id = 3,
    .minimum_version_id_old = 3,
    .fields      = (VMStateField []) {
        VMSTATE_BUFFER(rregs, ESP2State),
        VMSTATE_BUFFER(wregs, ESP2State),
        VMSTATE_INT32(ti_size, ESP2State),
        VMSTATE_UINT32(ti_rptr, ESP2State),
        VMSTATE_UINT32(ti_wptr, ESP2State),
        VMSTATE_BUFFER(ti_buf, ESP2State),
        VMSTATE_UINT32(status, ESP2State),
        VMSTATE_UINT32(dma, ESP2State),
        VMSTATE_BUFFER(cmdbuf, ESP2State),
        VMSTATE_UINT32(cmdlen, ESP2State),
        VMSTATE_UINT32(do_cmd, ESP2State),
        VMSTATE_UINT32(dma_left, ESP2State),
        VMSTATE_END_OF_LIST()
    }
};
void esp_new_init(target_phys_addr_t espaddr, int it_shift,
                ESP2DMAMemoryReadWriteFunc dma_memory_read,
                ESP2DMAMemoryReadWriteFunc dma_memory_write,
                void *dma_opaque, qemu_irq irq, qemu_irq *reset,
                qemu_irq *dma_enable)
{
    DeviceState *dev;
    SysBusDevice *s;
    ESP2State *esp;

    dev = qdev_create(NULL, "esp-new");
    esp = DO_UPCAST(ESP2State, busdev.qdev, dev);

    esp->dma_memory_read = dma_memory_read;
    esp->dma_memory_write = dma_memory_write;
    esp->dma_opaque = dma_opaque;

    esp->it_shift = it_shift;

    /* XXX for now until rc4030 has been changed to use DMA enable signal */
    esp->dma_enabled = 1; //perhaps this could be another source of trouble
    esp->irq_status = 0;

    qdev_init_nofail(dev);
    s = sysbus_from_qdev(dev);

    sysbus_connect_irq(s, 0, irq);
    sysbus_mmio_map(s, 0, espaddr);

    *reset = qdev_get_gpio_in(dev, 0);
    *dma_enable = qdev_get_gpio_in(dev, 1);
}
/* SCSI call backs */
static void esp_transfer_data(SCSIRequest *req, uint32_t len)
{
    DPRINTF("Transfer data %x\n",len);
}

static void esp_command_complete(SCSIRequest *req, uint32_t status)
{
    DPRINTF("Command complete %x\n",status);
}

static void esp_request_cancelled(SCSIRequest *req)
{
    DPRINTF("Request cancelled\n");
}

static const struct SCSIBusOps esp_scsi_ops = {
    .transfer_data = esp_transfer_data,
    .complete = esp_command_complete,
    .cancel = esp_request_cancelled
};

static int esp_init1(SysBusDevice *dev)
{
    ESP2State *s = FROM_SYSBUS(ESP2State, dev);
    int esp_io_memory;

    sysbus_init_irq(dev, &s->irq);
    assert(s->it_shift != -1);

    esp_io_memory = cpu_register_io_memory(esp_mem_read, esp_mem_write, s,
        DEVICE_NATIVE_ENDIAN);

    sysbus_init_mmio(dev, ESP_REGS << s->it_shift, esp_io_memory);

    qdev_init_gpio_in(&dev->qdev, esp_gpio_demux, 2);

    scsi_bus_new(&s->bus, &dev->qdev, 0, ESP_MAX_DEVS, &esp_scsi_ops);

    return scsi_bus_legacy_handle_cmdline(&s->bus);
}

static SysBusDeviceInfo esp_info = {
        .init = esp_init1,
        .qdev.name  = "esp-new",
        .qdev.size  = sizeof(ESP2State),
        .qdev.vmsd  = &vmstate_esp,
        .qdev.reset = esp_hard_reset,
        .qdev.props = (Property[]) {
                {.name = NULL}
        }
};

static void esp_register_devices(void)
{
        sysbus_register_withprop(&esp_info);
}

device_init(esp_register_devices)
