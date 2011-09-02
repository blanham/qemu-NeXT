

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

    /* need to add a callback function pointer */
} next_dma;


enum next_dma_chan {
    NEXTDMA_FD,
    NEXTDMA_ENRX,
    NEXTDMA_ENTX,
    NEXTDMA_SCSI,
    NEXTDMA_SCC,
    NEXTDMA_SND
};
#define DMA_ENABLE      0x01000000
#define DMA_SUPDATE     0x02000000
#define DMA_COMPLETE    0x08000000

#define DMA_M2DEV       0x0
#define DMA_SETENABLE   0x00010000
#define DMA_SETSUPDATE  0x00020000
#define DMA_DEV2M       0x00040000
#define DMA_CLRCOMPLETE 0x00080000
#define DMA_RESET       0x00100000

/* SCSI CSR defines */
#define SCSI_SELECT 1<<0
#define SCSI_RESET  1<<1
#define SCSI_FIFO   1<<2
#define SCSI_DMADIR 1<<3
#define SCSI_CPUDMA 1<<4
#define SCSI_INTMSK 1<<5

#include "hw.h"
enum next_irqs {
    NEXT_FD_I,
    NEXT_KBD_I,
    NEXT_PWR_I,
    NEXT_ENRX_I,
    NEXT_ENTX_I,
    NEXT_SCSI_I,
    NEXT_CLK_I,
    NEXT_SCC_I,
    NEXT_ENTX_DMA_I,
    NEXT_ENRX_DMA_I,
    NEXT_SCSI_DMA_I,
    NEXT_SCC_DMA_I,
    NEXT_SND_I
};
void next_irq(void *opaque, int number, int level);

typedef struct {
    uint32_t int_mask;
    uint32_t int_status;

    uint8_t scsi_csr_1;
    uint8_t scsi_csr_2;
    next_dma dma[10];
    qemu_irq *scsi_irq;
    qemu_irq scsi_dma;
    qemu_irq scsi_reset;
    qemu_irq *fd_irq;
} next_state_t;
extern next_state_t next_state;
/* next-kbd.c  */
void nextkbd_init(void *opaque);
/* next-net.c */
void nextnet_init(void *opaque);
/* next-fb.c */
typedef struct {
	DisplayState *ds;
	uint32_t base;
	uint32_t pitch;
	uint32_t cols;
	uint32_t rows;
	int invalidate;


} nextfb_state_t;

extern nextfb_state_t nextfb_state;
void nextfb_draw_line(void *opaque, uint8_t *d, const uint8_t *s, int width, int pitch);

void nextfb_init(nextfb_state_t *s);
