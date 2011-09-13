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
        uint8_t fifo_pbuf[TI_BUFSZ];
        uint8_t fifo_rptr, fifo_wptr; 

        int8_t fifo_size;
        uint16_t transfer_count;
        int32_t data_len;
        
        uint32_t cmdlen;
        uint32_t do_cmd;

        uint32_t status;
        uint32_t dma;
        SCSIBus bus;
        SCSIDevice *current_dev;
        SCSIRequest *current_req;

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

        
        void (*deferred_cmd)(ESP2State *s);
        int cmd_count;
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

#define CMD_DMA      0x80
#define CMD_CMD      0x7f

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

#define STAT_DO      0x00
#define STAT_DI      0x01
#define STAT_CD      0x02
#define STAT_ST      0x03
#define STAT_MO      0x06
#define STAT_MI      0x07
#define STAT_PIO_MASK 0x06

#define STAT_TC      0x10
#define STAT_PE      0x20
#define STAT_GE      0x40
#define STAT_INT     0x80

#define BUSID_DID    0x07

#define INTR_FC      0x08
#define INTR_BS      0x10
#define INTR_DC      0x20
#define INTR_RST     0x80

#define SEQ_0        0x00
#define SEQ_CD       0x04

#define CFG1_RESREPT 0x40

#define CFG2_ENFEA   0x40

#define TCHI_FAS100A 0x04

/* chip versions, may have to add version select to esp_init() */
#define ESP100      0x1
#define NCR53C90    0x1
#define ESP100A     0x2
#define NCR53C90A   0x2
/*
int esp_parity(uint8_t x)
{
    x ^= x >> 1;
    x ^= x >> 2;
    x ^= x >> 4;
    x ^= x >> 8;
    x ^= x >> 16;
    
    return x & 1;
}
*/
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
    /* this is wrong */
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
     
    s->do_cmd = 0;
    s->dma_cb = NULL;
    
    /* Double check to see if this is correct? */
    s->rregs[ESP_CFG1] &= 7;
}
/* needs a way to differentiate between chip and hardware soft resets */
static void esp_soft_reset(DeviceState *d)
{
    ESP2State *s = container_of(d, ESP2State, busdev.qdev);
    DPRINTF("Soft Reset\n");
    qemu_irq_lower(s->irq);
    //esp_hard_reset(d);//<- WTF?
    
    /* set all registers to 0 */
    //memset(s->rregs, 0, ESP_REGS);
    //memset(s->wregs, 0, ESP_REGS);
    
    /* FAS100A identity, need to look at other docs to see
     * if this is necessary for other chips */
    s->rregs[ESP_TCHI] = TCHI_FAS100A;
    
    /* clear fifo */
    //s->fifo_size = s->fifo_wptr = s->fifo_rptr = 0;
    
    /* non-dma mode */
    s->dma = 0;
   
    /* reset transfer count */
    //s->transfer_count = 0;
    s->data_len = 0;
     
    s->do_cmd = 0;
    s->dma_cb = NULL;
    
    /* Double check to see if this is correct? */
    s->rregs[ESP_CFG1] &= 7;

    /* set the RINT RSTAT and RSEQ */
   // s->rregs[ESP_RSTAT] = 0;
    s->rregs[ESP_RINTR] = INTR_RST;
   // s->rregs[ESP_RSEQ] = 0;

    esp_raise_irq(s);
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
static void esp_dma_done(ESP2State *s)
{
    s->rregs[ESP_RSTAT] |= STAT_TC;
    s->rregs[ESP_RINTR] = INTR_BS;
    s->rregs[ESP_RSEQ] = 0;
    s->rregs[ESP_RFLAGS] = 0;
    s->rregs[ESP_TCLO] = 0;
    s->rregs[ESP_TCMID] = 0;
    esp_raise_irq(s);
}

static void esp_do_dma(ESP2State *s)
{
    uint32_t len;
    int to_device;

    to_device = (s->data_len < 0);

    len = s->dma_left;
    /* fix this when I fix esp_transfer_data*/
    if (s->do_cmd) {
        abort();
        DPRINTF("command len %d + %d\n", s->cmdlen, len);
        s->dma_memory_read(s->dma_opaque, &s->fifo_buf[s->cmdlen], len);
        s->fifo_size = 0;
        s->cmdlen = 0;
        s->do_cmd = 0;
        //do_cmd(s, s->cmdbuf);
        return;
    }
    /* handle async transfers to/from scsi bus */
    if (s->async_len == 0) {
        /* Defer until data is available.  */
        return;
    }
   
    /* check if this is backwards */ 
    if (len > s->async_len) {
        len = s->async_len;
    }
    
    if (to_device) {
        s->dma_memory_read(s->dma_opaque, s->async_buf, len);
    } else {
        s->dma_memory_write(s->dma_opaque, s->async_buf, len);
    }
    
    s->dma_left -= len;
    s->async_buf += len;
    s->async_len -= len;
    
    if (to_device)
        s->data_len += len;
    else
        s->data_len -= len;

    if (s->async_len == 0) {
        scsi_req_continue(s->current_req);
        /* If there is still data to be read from the device then
           complete the DMA operation immediately.  Otherwise defer
           until the scsi layer has completed.  */
        if (to_device || s->dma_left != 0 || s->fifo_size == 0) {
            return;
        }
    }

    /* Partially filled a scsi buffer. Complete immediately.  */
    esp_dma_done(s);
}


/* TODO: This doesn't handle stacked commands */
static uint32_t get_cmd(ESP2State *s, uint8_t *buf)
{
    uint32_t xfer_len = 0;
    
    int target = s->wregs[ESP_WBUSID] & BUSID_DID;
    
    if (s->dma) {
        /* check for zero should happen here, not in do_busid_cmd()? */
        xfer_len = s->rregs[ESP_TCLO] | (s->rregs[ESP_TCMID] << 8);
        s->dma_memory_read(s->dma_opaque, buf, xfer_len);
    } else {
        xfer_len = s->fifo_size;
        memcpy(buf, s->fifo_buf, s->fifo_size);
        
        /* what does this do? */
        //buf[0] = buf[2] >> 5;
        //buf[0] = 0;
        
        /* now i think the fifo should be cleared */
        s->fifo_size = s->fifo_rptr = s->fifo_wptr = 0;
        /* zero FIFO count */
        s->rregs[ESP_RFLAGS] &= 0xe0;
    }
    DPRINTF("get_cmd: len %d target %d\n", xfer_len, target);

    /* Derp, why do we clear the fifo, when it might be a dma transfer? */
    //s->ti_size = 0;
    //s->ti_rptr = 0;
    //s->ti_wptr = 0;
    /* zero FIFO count */
    //s->rregs[ESP_RFLAGS] &= 0xe0;
    
    if (s->current_req) {
        /* Started a new command before the old one finished.  Cancel it.  */
        scsi_req_cancel(s->current_req);
        /* where else is this used? */
        s->async_len = 0;
    }

    if (target >= ESP_MAX_DEVS || !s->bus.devs[target]) {
        // No such drive
        /* I think these are right */
        s->rregs[ESP_RSTAT] = 0;
        s->rregs[ESP_RINTR] = INTR_DC;
        s->rregs[ESP_RSEQ] = SEQ_0;
        esp_raise_irq(s);
        return 0;
    }
    s->current_dev = s->bus.devs[target];
    return xfer_len;
}
static void do_busid_cmd(ESP2State *s, uint8_t *buf, uint8_t busid)
{
    int lun;

    DPRINTF("do_busid_cmd: busid 0x%x\n", busid);
    lun = busid & 7;
    s->current_req = scsi_req_new(s->current_dev, 0, lun, NULL);
    s->data_len = scsi_req_enqueue(s->current_req, buf);
    
    if (s->data_len != 0) {
        DPRINTF("executing command\n");
        /* not sure about this */
        s->rregs[ESP_RSTAT] = STAT_TC;

        /* where are these used? */
        s->dma_left = 0;
        s->dma_counter = 0;

        if (s->data_len > 0) {
            DPRINTF("DATA IN\n");
            s->rregs[ESP_RSTAT] |= STAT_DI;
        } else {
            DPRINTF("DATA OUT\n");
            s->rregs[ESP_RSTAT] |= STAT_DO;
        }
        
        scsi_req_continue(s->current_req);
    }
    s->rregs[ESP_RINTR] = INTR_BS | INTR_FC;
    s->rregs[ESP_RSEQ] = SEQ_CD;
   
    esp_raise_irq(s);
}

static void do_cmd(ESP2State *s, uint8_t *buf)
{
    /* is this really necessary? can't we just get the value and use it? */ 
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
    if(!(len == 0 || len == 7 || len == 11 || len == 13)){
        fprintf(stderr,"invalid command length %i in handle_satn\n",
            len);
        abort();
    }

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
    if(!(len == 0 || len == 7 || len == 11 || len == 13)){
        fprintf(stderr,"invalid command length %i in handle_s_without_atn\n",
            len);
        abort();
    }

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
    s->cmdlen = get_cmd(s, s->fifo_buf);
    if (s->cmdlen) {
        /* check these to be sure they are right */
        DPRINTF("Set ATN & Stop: cmdlen %d\n", s->cmdlen);
        s->do_cmd = 1;
        s->rregs[ESP_RSTAT] = STAT_TC | STAT_CD;
        s->rregs[ESP_RINTR] = INTR_BS | INTR_FC;
        s->rregs[ESP_RSEQ] = SEQ_CD;
        esp_raise_irq(s);
    }
}
static void handle_ti(ESP2State *s)
{
    uint32_t dma_len, min_len;

    dma_len = s->rregs[ESP_TCLO] | (s->rregs[ESP_TCMID] << 8);
    
    if(s->rregs[ESP_CFG2] & CFG2_ENFEA){
        /* disabled for now, until we properly handle device
         * type inquiry */
        //dma_len |= s->rregs[ESP_TCHI];
    }	

	if (dma_len == 0) {
        /* 53c90/ESP100 have a default DMA size of 64k
         * 53c6X has a default size of 16MB
         */
        if(s->rregs[ESP_CFG2] & CFG2_ENFEA)
        {
            dma_len = 0x1000000;
        }else{
            dma_len = 0x10000;
        }
    }
    
    s->dma_counter = dma_len;

    if (s->do_cmd)
        min_len = (dma_len < 32) ? dma_len : 32;
    /* everything after this point i am unsure about */
    else if (s->data_len < 0)
        min_len = (dma_len < -s->data_len) ? dma_len : -s->data_len;
    else
        min_len = (dma_len > s->data_len) ? dma_len : s->data_len;
    
    DPRINTF("Transfer Information len %d\n", min_len);
    
    if (s->dma) {
        s->dma_left = min_len;
        s->rregs[ESP_RSTAT] &= ~STAT_TC;
        esp_do_dma(s);
    } else if (s->do_cmd) {
        DPRINTF("command len %d\n", s->cmdlen);
        s->data_len = 0;
        s->cmdlen = 0;
        s->do_cmd = 0;
        do_cmd(s, s->fifo_buf);
        return;
    }
}
static void write_response(ESP2State *s)
{
    DPRINTF("Transfer status (status=%d)\n", s->status);
    s->fifo_buf[0] = s->status;
    s->fifo_buf[1] = 0;
    if (s->dma) {
        s->dma_memory_write(s->dma_opaque, s->fifo_buf, 2);
        s->rregs[ESP_RSTAT] = STAT_TC | STAT_ST;
        s->rregs[ESP_RINTR] = INTR_BS | INTR_FC;
        s->rregs[ESP_RSEQ] = SEQ_CD;
    } else {
        s->fifo_size = 2;
        /* are these pointers right? */
        s->fifo_rptr = 0;
        s->fifo_wptr = 0;
        s->rregs[ESP_RFLAGS] = (s->rregs[ESP_RFLAGS] & 0xe0) | 2;
        
    }
    esp_raise_irq(s);
}
static void esp_chip_reset(ESP2State *s)
{
    /* set all registers to 0 */
    memset(s->rregs, 0, ESP_REGS);
    memset(s->wregs, 0, ESP_REGS);
    
    /* FAS100A identity, need to look at other docs to see
     * if this is necessary for other chips */
    s->rregs[ESP_TCHI] = TCHI_FAS100A;
    
    /* clear fifo */
    s->fifo_size = s->fifo_wptr = s->fifo_rptr = 0;
    s->fifo_buf[0] = s->fifo_pbuf[0] = 0;
     
    /* non-dma mode */
    s->dma = 0;
   
    /* reset transfer count */
    //s->transfer_count = 0;
    s->data_len = 0;
     
    s->do_cmd = 0;
    s->dma_cb = NULL;
    
    /* We are always busid 7 */
    s->rregs[ESP_CFG1] &= 7;

    /* set the RINT RSTAT and RSEQ */
    s->rregs[ESP_RSTAT] = 0;
    s->rregs[ESP_RINTR] = 0;
    s->rregs[ESP_RSEQ] = 0;
}

static void fifo_flush(ESP2State *s)
{
    /* should read pointer also be set to zero? */
    s->fifo_wptr = 0;
    s->fifo_rptr = 0;
    s->fifo_size = 0;
    s->fifo_buf[0] = 0; 
    /* dunno if this is right either */
    /* nope from what i can tell from the 53c90a datasheet
    * clearing fifo only clears position, first byte of buffer and flags */ 
    //s->rregs[ESP_RINTR] = INTR_FC;
    
    /* not sure if this command follows the datasheet */
    //s->rregs[ESP_RSEQ] = 0;
   
    /* clear fifo count in fifo flags register */
    s->rregs[ESP_RFLAGS] &= 0xe0;
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
                    /* TODO: add PIO support */
                    ESP_ERROR("PIO data read not implemented\n");
                    /* added abort to make it easier to check if a
                       guest tried to use this */
                    abort();
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
            /* should doublecheck if these are right */
                old_val = s->rregs[ESP_RINTR];
                s->rregs[ESP_RINTR] = 0;
                s->rregs[ESP_RSTAT] &= ~STAT_TC;
                s->rregs[ESP_RSEQ] = SEQ_CD;
                        
                esp_lower_irq(s);
                //s->irq_status = 0;//DERP
                return old_val;
            } else {



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
        case ESP_CFG2:
            break;
        case ESP_TCHI:
            return TCHI_FAS100A;
        default:
                DPRINTF("readb @ %x\n",(uint32_t)addr);
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
                //should probably be set when stuff is copied to xfer_cnt
                s->rregs[ESP_RSTAT] &= ~STAT_TC; 
                
                s->rregs[saddr] = val & 0xFF;
                break;
        case ESP_FIFO:
            if (s->fifo_size == TI_BUFSZ - 1) {
                ESP_ERROR("fifo overrun\n");
            } else {
                /* fifo has another byte in it */
                s->fifo_size++;
                
                /* write value to fifo buffer */
                //s->fifo_pbuf[s->fifo_wptr] = esp_parity(val & 0xFF);
                
                s->fifo_buf[s->fifo_wptr++] = val & 0xFF;
                
                /* set FIFO count in RFLAGS */
                s->rregs[ESP_RFLAGS] = (s->rregs[ESP_RFLAGS]&0xe0) | (s->fifo_wptr);
            }
            break;
        /* this is actually a two-deep FIFO, might need to handle that accordiingly */
        case ESP_CMD:
            /* next comand is a dma command if val & CMD_DMA */
            s->dma = (val & CMD_DMA ? 1 : 0); 
            s->cmd_count++; 
            switch(val & CMD_CMD)
            {  
                /* Miscellaneous Group */ 
                case CMD_NOP:
                    /* some commands should be deferred and run here */
                    DPRINTF("NOP (%x)\n",val);
                    /* transfer counter is filled on a DMA NOP */
                    if(val & CMD_DMA){
                        /* transfer counter = TCMID + TCLO /TCLHI */ 
                        /* s->xfer_cnt = s->rregs[ESP_TCLO] | (s-rregs[ESP_TCMID] << 8);
                         * if(s->rregs[ESP_CFG2] & CFG2_ENFEA){
                         *      s->xfer_cnt |= s->rregs[ESP_TCHI] << 16;
                         *  }
                         */
                    }
                    break; 
                case CMD_FLUSH:
                    DPRINTF("Fifo Flush (%x)\n",val);
                    fifo_flush(s);
                    break;
                case CMD_RESET:
                    DPRINTF("Reset (%x)\n",val);
                    /* might want to rewrite this function, I'm not completely
                     * certain of the relationship between external reset interrupt
                     * an this reset */
                    /* doesn't interrupt, therefore calling esp_soft_reset is incorrect */
                    esp_chip_reset(s);
                    break;
                case CMD_BUSRESET:
                    DPRINTF("Bus Reset (%x)\n",val);
                    /* this could be a seperate function */
                    /* Need to reset all devices on the bus */
                    DeviceState *dev;
                    int id;
                    for (id = 0; id < s->bus.ndev; id++) {
                        if (s->bus.devs[id]) {
                            dev = &s->bus.devs[id]->qdev;
                            dev->info->reset(dev);
                        }
                    }

                    if (!(s->rregs[ESP_CFG1] & CFG1_RESREPT)) {
                        DPRINTF("Bus Reset raising IRQ\n");
                        s->rregs[ESP_RINTR] = INTR_RST;
                        esp_raise_irq(s);
                    }
                    break;
                /* Initiator State Group */
                case CMD_TI:/* INT */
                    DPRINTF("Transfer Information (%x)\n",val);
                    handle_ti(s);
                    break;
                case CMD_ICCS:/* INT */
                    DPRINTF("Initiator Command Complete Sequence (%2.2x)\n", 
                        val);
                    write_response(s);
                    /* these are supposed to be conditional */
                    s->rregs[ESP_RINTR] = INTR_FC;
                    s->rregs[ESP_RSTAT] |= STAT_MI;
                    break;
                case CMD_MSGACC:/* INT */
                    DPRINTF("Message Accepted (%2.2x)\n", val);
                    s->rregs[ESP_RINTR] = INTR_DC;
                    /* are these right ? */
                    s->rregs[ESP_RSEQ] = 0;
                    s->rregs[ESP_RFLAGS] = 0;
                    esp_raise_irq(s);
                    break;
                //case CMD_PAD: /* INT */
                //case CMD_SATN: /*No INT */
                
                /* Target State Group 0x20 - 0x2b (and 0x4) not implemented */

                /* Disconnected State Group */
                /* error catching needs to be added to catch uninitialized registers */
                /* CMD_SEL - CMD_SEL3 need to check if disconnected or not */
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
                case CMD_ENSEL:
                    DPRINTF("Enable selection (%2.2x)\n", val);
                    s->rregs[ESP_RINTR] = 0;
                    break;

                default:
                    ESP_ERROR("Unhandled ESP command (%2.2x)\n", val);
                    abort();
                    break;
            }
            /* CMD can be read back */
            s->rregs[ESP_CMD] = val;
            break;
        case ESP_WBUSID:
                break;
        case ESP_WSEL:
                DPRINTF("Write SEL %x\n",val);
                break;
        case ESP_WSYNTP:
            break;
        case ESP_WSYNO:
            DPRINTF("Write SYNO Offset %x\n",val);
            
            s->rregs[saddr] = val;
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
        /* 53C90A/ESP(FAS)100A and up register */
        /* NeXT uses this to detect version of chip */
        case ESP_CFG2:
            s->rregs[saddr] = val;
            break;

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
        VMSTATE_INT8(fifo_size, ESP2State),
        VMSTATE_UINT8(fifo_rptr, ESP2State),
        VMSTATE_UINT8(fifo_wptr, ESP2State),
        VMSTATE_BUFFER(fifo_buf, ESP2State),
        VMSTATE_UINT32(status, ESP2State),
        VMSTATE_UINT32(dma, ESP2State),
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
    ESP2State *s = DO_UPCAST(ESP2State, busdev.qdev, req->bus->qbus.parent);

    DPRINTF("transfer %d/%d\n", s->dma_left, s->data_len);
    s->async_len = len;
    s->async_buf = scsi_req_get_buf(req);
    if (s->dma_left) {
        esp_do_dma(s);
    } else if (s->dma_counter != 0 && s->data_len <= 0) {
        /* If this was the last part of a DMA transfer then the
           completion interrupt is deferred to here.  */
        esp_dma_done(s);
    }

}
/* this needs to call deferred commands */
static void esp_command_complete(SCSIRequest *req, uint32_t status)
{
    ESP2State *s = DO_UPCAST(ESP2State, busdev.qdev, req->bus->qbus.parent);

    DPRINTF("SCSI Command complete\n");
    if (s->data_len != 0) {
        DPRINTF("SCSI command completed unexpectedly\n");
    }
    s->data_len = 0;
    s->dma_left = 0;
    s->async_len = 0;
    if (status) {
        DPRINTF("Command failed\n");
    }
    s->status = status;
    /* is this right? */
    s->rregs[ESP_RSTAT] = STAT_ST;
    esp_dma_done(s);
    if (s->current_req) {
        scsi_req_unref(s->current_req);
        s->current_req = NULL;
        s->current_dev = NULL;
    }
}
static void esp_request_cancelled(SCSIRequest *req)
{
    ESP2State *s = DO_UPCAST(ESP2State, busdev.qdev, req->bus->qbus.parent);

    DPRINTF("Request cancelled\n");
    
    if (req == s->current_req) {
        scsi_req_unref(s->current_req);
        s->current_req = NULL;
        s->current_dev = NULL;
        
        s->rregs[ESP_RINTR] |= INTR_DC;
        esp_raise_irq(s);
    }else{

        s->rregs[ESP_RINTR] |= INTR_DC;
        esp_raise_irq(s);

    }
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