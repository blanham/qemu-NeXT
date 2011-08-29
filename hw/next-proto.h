/* I seperated all of the major prototypes from next-cube.c into this file to
   make it easier to read, I'll probably add these back in when this is upstreamed*/
static uint32_t mmio_readb(void*opaque, target_phys_addr_t addr);
static uint32_t mmio_readw(void*opaque, target_phys_addr_t addr);
static uint32_t mmio_readl(void*opaque, target_phys_addr_t addr);

static void mmio_writeb(void*opaque, target_phys_addr_t addr, uint32_t val);
static void mmio_writew(void*opaque, target_phys_addr_t addr, uint32_t val);
static void mmio_writel(void*opaque, target_phys_addr_t addr, uint32_t val);
static CPUReadMemoryFunc *mmio_read[3] = {
	mmio_readb,
	mmio_readw,
	mmio_readl
};

static CPUWriteMemoryFunc *mmio_write[3] = {
	mmio_writeb,
	mmio_writew,
	mmio_writel
};
static uint32_t scr_readb(void*opaque, target_phys_addr_t addr);
static uint32_t scr_readw(void*opaque, target_phys_addr_t addr);
static uint32_t scr_readl(void*opaque, target_phys_addr_t addr);
static CPUReadMemoryFunc *scr_read[3] = {
    scr_readb,
    scr_readw,
    scr_readl

};

static void scr_writeb(void*opaque, target_phys_addr_t addr, uint32_t value);
static void scr_writew(void*opaque, target_phys_addr_t addr, uint32_t value);
static void scr_writel(void*opaque, target_phys_addr_t addr, uint32_t value);
static CPUWriteMemoryFunc * const scr_write[3] = {
    scr_writeb,
    scr_writew,
    scr_writel
};
void nextscsi_read(void *opaque, uint8_t *buf, int len);
void nextscsi_write(void *opaque, uint8_t *buf, int len);
void nextscsi_irq(void *opaque, int n, int level);

void nextfdc_irq(void *opaque, int n, int level);
void serial_irq(void *opaque, int n, int level);
static uint32_t dma_readl(void*opaque, target_phys_addr_t addr);

static CPUReadMemoryFunc *dma_read[3] = {
	NULL,
	NULL,
	dma_readl
};

static void dma_writel(void*opaque, target_phys_addr_t addr, uint32_t value);
static CPUWriteMemoryFunc * const dma_write[3] = {
	NULL,
	NULL,
	dma_writel
};


void next_dma_write(void *opaque, const uint8_t *buf, int len, int type);

void next_dma_check(void);
