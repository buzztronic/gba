#pragma once

#include "bus.h"

typedef struct Dma {
    Bus *bus;
    u8 reg[0x100];
} Dma;

Dma *dma_init(Bus *bus);
void dma_update(Dma *dma);
