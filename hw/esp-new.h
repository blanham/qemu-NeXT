#ifndef QEMU_HW_ESP_NEW_H
#define QEMU_HW_ESP_NEW_H

/* esp-new.c */
#define ESP_MAX_DEVS 7
typedef void (*ESP2DMAMemoryReadWriteFunc)(void *opaque, uint8_t *buf, int len);
void esp_new_init(target_phys_addr_t espaddr, int it_shift,
              ESP2DMAMemoryReadWriteFunc dma_memory_read,
              ESP2DMAMemoryReadWriteFunc dma_memory_write,
              void *dma_opaque, qemu_irq irq, qemu_irq *reset,
              qemu_irq *dma_enable);

#endif
