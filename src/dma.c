#include <stdlib.h>
#include <string.h>

#include "dma.h"

const u32 REG_DMAX_CNT_H[] = { 0xBA, 0xC6, 0xD2, 0xDE }; // Control
const u32 REG_DMAX_CNT_L[] = { 0xB8, 0xC4, 0xD0, 0xDC }; // Word count
const u32 REG_DMAX_SAD[] = { 0xB0, 0xBC, 0xC8, 0xD4 };
const u32 REG_DMAX_DAD[] = { 0xB4, 0xC0, 0xCC, 0xD8 };

static u32 dma_read(void *, u32 addr, u8 width);
static void dma_write(void *, u32 addr, u8 width, u32 data);

static int dma_should_start(Dma *dma, uint c);
static void dma_start(Dma *dma, uint c);

Dma *dma_init(Bus *bus)
{
    Dma *dma = malloc(sizeof(Dma));

    memset(dma, 0, sizeof(Dma));

    dma->bus = bus;
    dma->ppu_state = PPU_STATE_HDRAW;

    BusDev dma_dev = {dma, dma_read, dma_write};
    bus_attach_dma(bus, &dma_dev);

    return dma;
}

void dma_update(Dma *this)
{
    for (uint i = 0; i < 4; i++) {
        u8 cnt = this->reg[REG_DMAX_CNT_H[i]+1];
        if (bit(this->saved_cnt_h[i], 7) == 0 && bit(cnt, 7) == 1) {
            // Reload
            this->sad[i] = read_memory(this->reg + REG_DMAX_SAD[i], WIDTH_32);
            this->dad[i] = read_memory(this->reg + REG_DMAX_DAD[i], WIDTH_32);
            this->cnt_l[i] = read_memory(this->reg + REG_DMAX_CNT_L[i], WIDTH_16);
        }
        this->saved_cnt_h[i] = cnt;
    }

    for (uint i = 0; i < 4; i++) {
        if (dma_should_start(this, i)) {
            dma_start(this, i);
            break;
        }
    }

    this->ppu_state = bus_get_ppu_state(this->bus);
}

static int dma_should_start(Dma *this, uint c)
{
    u16 cnt = read_memory(this->reg + REG_DMAX_CNT_H[c], WIDTH_16);

    // DMA disabled
    if (!bit(cnt, 15))
        return 0;

    u8 timing = bits(cnt, 12, 2);

    switch (timing) {
        case 0:
            return 1;
            break;
        case 1:
            return this->ppu_state != PPU_STATE_VBLANK &&
                bus_get_ppu_state(this->bus) == PPU_STATE_VBLANK;
            break;
        case 2:
            return this->ppu_state != PPU_STATE_HBLANK &&
                bus_get_ppu_state(this->bus) == PPU_STATE_HBLANK;
            break;
        case 3:
            // TODO
            return c == 3;
            break;
    }

    return 1;
}

static void dma_start(Dma *this, uint c)
{
    u16 cnt = read_memory(this->reg + REG_DMAX_CNT_H[c], WIDTH_16);

    u8 width = bit(cnt, 10) ? 4 : 2;
    u8 dst_cnt = bits(cnt, 5, 2);
    u8 src_cnt = bits(cnt, 7, 2);
    u8 timing = bits(cnt, 12, 2);

    u32 sad = this->sad[c];
    u32 dad = this->dad[c];
    u32 count = this->cnt_l[c];

    // Clear DMA Enable bit
    if (!bit(cnt, 9) || timing == 0) {
        this->reg[REG_DMAX_CNT_H[c]+1] &= ~BIT_7;
    }

    if (c == 3) {
        count &= BIT_16 - 1;
        if (count == 0) {
            count = BIT_16;
        }
    } else {
        count &= BIT_14 - 1;
        if (count == 0) {
            count = BIT_14;
        }
    }

    for (; count > 0; count--) {
        if (width == 2) {
            u16 data = bus_read16(this->bus, sad);
            bus_write16(this->bus, dad, data);
        } else {
            u32 data = bus_read32(this->bus, sad);
            bus_write32(this->bus, dad, data);
        }

        if (dst_cnt == 0 || dst_cnt == 3)
            dad += width;
        else if (dst_cnt == 1)
            dad -= width;

        if (src_cnt == 0)
            sad += width;
        else if (src_cnt == 1)
            sad -= width;
    }

    // write back SAD and DAD: only useful for repeated DMA
    // the count stays the same
    this->sad[c] = sad;

    // don't change DAD if reload is enabled
    if (dst_cnt != 3)
        this->dad[c] = dad;

    // IRQ
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
