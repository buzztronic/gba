#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "dma.h"

#define REG_DMA0_CNT 0xBA
#define REG_DMA1_CNT 0xC6

static u32 dma_read(void *, u32 addr, u8 width);
static void dma_write(void *, u32 addr, u8 width, u32 data);

static int dma_should_start(Dma *dma, uint c);
static void dma_start(Dma *dma, uint c);

Dma *dma_init(Bus *bus)
{
    Dma *dma = malloc(sizeof(Dma));

    memset(dma, 0, sizeof(Dma));

    dma->bus = bus;

    BusDev dma_dev = {dma, dma_read, dma_write};
    bus_attach_dma(bus, &dma_dev);

    return dma;
}

void dma_update(Dma *dma)
{
    for (uint i = 0; i < 4; i++) {
        if (dma_should_start(dma, i)) {
            dma_start(dma, i);
            break;
        }
    }
}

static int dma_should_start(Dma *this, uint c)
{
    u32 index = REG_DMA0_CNT + c * (REG_DMA1_CNT - REG_DMA0_CNT);
    u16 dmacnt = this->reg[index] | (this->reg[index+1] << 8);

    if (!bit(dmacnt, 15))
        return 0;

    u8 timing = bits(dmacnt, 12, 2);

    switch (timing) {
        case 0:
            return 1;
        break;
        case 1:
            return bus_get_ppu_state(this->bus) == PPU_STATE_VBLANK;
        break;
        case 2:
            return bus_get_ppu_state(this->bus) == PPU_STATE_HBLANK;
        break;
        case 3:
            // TODO
            if (c == 3)
                return 1;
            return 0;
        break;
    }

    return 1;
}

static void dma_start(Dma *this, uint c)
{
    u32 offset = 0xB0 + c * (REG_DMA1_CNT - REG_DMA0_CNT);

    u32 src_addr = read_memory(this->reg + offset, WIDTH_32);
    u32 dst_addr = read_memory(this->reg + offset + 4, WIDTH_32);
    u16 count = read_memory(this->reg + offset + 8, WIDTH_16);
    u16 cnt = read_memory(this->reg + offset + 10, WIDTH_16);

    u8 width = bit(cnt, 10) ? 4 : 2;
    u8 src_cnt = bits(cnt, 5, 2);
    u8 dst_cnt = bits(cnt, 7, 2);

    for (; count > 0; count--) {
        if (width == 2) {
            u16 data = bus_read16(this->bus, src_addr);
            bus_write16(this->bus, dst_addr, data);
        } else {
            u32 data = bus_read32(this->bus, src_addr);
            bus_write32(this->bus, dst_addr, data);
        }

        if (dst_cnt == 0 || dst_cnt == 3)
            dst_addr += width;
        if (dst_cnt == 1)
            dst_addr -= width;

        if (src_cnt == 0)
            src_addr += width;
        if (src_cnt == 1)
            src_addr -= width;
    }

    // Reload
    if (dst_cnt == 3) {
        write_memory(this->reg + offset + 4, WIDTH_32, dst_addr);
    }

    // repeat
    if (!bit(cnt, 9)) {
        this->reg[offset+11] &= ~BIT_7;
    }

    if (bit(cnt, 14)) {
        bus_send_irq(this->bus, 1 << (c + 8));
    }
}

static u32 dma_read(void *dev, u32 addr, u8 width)
{
    Dma *this = dev;
    addr &= align_mask[width];
    return read_memory(this->reg+addr, width);
}

static void dma_write(void *dev, u32 addr, u8 width, u32 data)
{
    Dma *this = dev;
    addr &= align_mask[width];
    write_memory(this->reg+addr, width, data);
}
