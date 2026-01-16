#pragma once

#include "bus.h"

typedef struct Dma {
    Bus *bus;
    u8 reg[0x100];
    u8 ppu_state;

    u32 sad[4];
    u32 dad[4];
    u16 cnt_l[4];

    // saved copy of high byte of cnt_h for each channel. used to detect
    // changes of DMA Enable bit from 0 to 1.
    u8 saved_cnt_h[4];
} Dma;

Dma *dma_init(Bus *bus);
void dma_update(Dma *dma);
