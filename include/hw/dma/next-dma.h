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

#ifndef HW_DMA_NEXT_DMA_H
#define HW_DMA_NEXT_DMA_H

#include "qom/object.h"

#define TYPE_NEXT_DMA "next-dma"
OBJECT_DECLARE_SIMPLE_TYPE(NextDMAState, NEXT_DMA)

typedef enum NextDMAChannel {
    NEXT_DMA_SCSI,
    NEXT_DMA_SOUND_OUT,
    NEXT_DMA_OPTICAL,
    NEXT_DMA_SOUND_IN,
    NEXT_DMA_PRINTER,
    NEXT_DMA_SCC,
    NEXT_DMA_DSP,
    NEXT_DMA_ENTX,
    NEXT_DMA_ENRX,
    NEXT_DMA_VIDEO,
    NEXT_DMA_R2M,
    NEXT_DMA_M2R,
    NEXT_DMA_CHANNEL_COUNT,
} NextDMAChannel;

typedef enum NextDMAResult {
    NEXT_DMA_OK,
    NEXT_DMA_NOT_READY,
    NEXT_DMA_RANGE_ERROR,
    NEXT_DMA_NO_SPACE,
} NextDMAResult;

typedef struct NextDMAEthernetNotify {
    void (*tx_kick)(void *opaque);
    void (*rx_ready_changed)(void *opaque, bool ready);
} NextDMAEthernetNotify;

typedef struct NextDMAOpticalNotify {
    void (*enabled)(void *opaque);
} NextDMAOpticalNotify;

typedef struct NextDMASoundOutNotify {
    void (*state_changed)(void *opaque);
} NextDMASoundOutNotify;

void next_dma_scsi_read(NextDMAState *s, uint8_t *buf, size_t len);
void next_dma_scsi_write(NextDMAState *s, const uint8_t *buf, size_t len);
void next_dma_scsi_fifo_reset(NextDMAState *s);
void next_dma_scsi_fifo_flush(NextDMAState *s);
void next_dma_set_scsi_control(NextDMAState *s, uint8_t control);
void next_dma_set_floppy_selected(NextDMAState *s, bool selected);

NextDMAResult next_dma_sound_out_read(NextDMAState *s, uint8_t *samples,
                                      size_t capacity, size_t *length);
bool next_dma_sound_out_complete(NextDMAState *s);
void next_dma_set_sound_out_notify(NextDMAState *s,
                                   const NextDMASoundOutNotify *notify,
                                   void *opaque);

NextDMAResult next_dma_enet_tx_read(NextDMAState *s, uint8_t *frame,
                                    size_t capacity, size_t *length);
void next_dma_enet_tx_complete(NextDMAState *s, NextDMAResult result);
bool next_dma_enet_rx_ready(NextDMAState *s);
NextDMAResult next_dma_enet_rx_write(NextDMAState *s,
                                     const uint8_t *frame_fcs,
                                     size_t length);
void next_dma_enet_rx_complete(NextDMAState *s, NextDMAResult result);
void next_dma_set_ethernet_notify(NextDMAState *s,
                                  const NextDMAEthernetNotify *notify,
                                  void *opaque);
NextDMAResult next_dma_optical_read(NextDMAState *s, uint8_t *buffer,
                                    size_t length);
NextDMAResult next_dma_optical_write(NextDMAState *s,
                                     const uint8_t *buffer,
                                     size_t length);
NextDMAResult next_dma_optical_abort(NextDMAState *s);
void next_dma_set_optical_notify(NextDMAState *s,
                                 const NextDMAOpticalNotify *notify,
                                 void *opaque);

#endif /* HW_DMA_NEXT_DMA_H */
