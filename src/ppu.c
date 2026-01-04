#include <assert.h>

#include "ppu.h"

#define WIN_TITLE "GBA Emulator"
#define WIN_X SDL_WINDOWPOS_CENTERED
#define WIN_Y SDL_WINDOWPOS_CENTERED
#define FRAME_W 240
#define FRAME_H 160
#define WIN_SCALE 4

#define REG_DISPCNT 0
#define REG_DISPSTAT 4
#define REG_VCOUNT 6

static void ppu_draw_scaneline(Ppu *this);
static void ppu_set_ly(Ppu *this, u8 ly);

static u32 plt_read(void *, u32 addr, u8 width);
static u32 oam_read(void *, u32 addr, u8 width);
static u32 vram_read(void *, u32 addr, u8 width);
static u32 lcd_read(void *, u32 addr, u8 width);
static void plt_write(void *, u32 addr, u8 width, u32 data);
static void oam_write(void *, u32 addr, u8 width, u32 data);
static void vram_write(void *, u32 addr, u8 width, u32 data);
static void lcd_write(void *, u32 addr, u8 width, u32 data);

enum PpuState {PPU_STATE_HDRAW, PPU_STATE_HBLANK, PPU_STATE_VBLANK};

Ppu *ppu_init(Bus *bus)
{
    Ppu *ppu = malloc(sizeof(Ppu));

    BusDev plt_dev = {ppu, plt_read, plt_write};
    BusDev oam_dev = {ppu, oam_read, oam_write};
    BusDev vram_dev = {ppu, vram_read, vram_write};
    BusDev lcd_dev = {ppu, lcd_read, lcd_write};

    ppu->bus = bus;
    bus_attach_plt(bus, &plt_dev);
    bus_attach_oam(bus, &oam_dev);
    bus_attach_vram(bus, &vram_dev);
    bus_attach_lcd(bus, &lcd_dev);

    memset(ppu->reg, 0, sizeof(ppu->reg));

    ppu->cycles = 0;
    ppu->state = PPU_STATE_HDRAW;

    ppu->sdl_win = SDL_CreateWindow(WIN_TITLE,
        WIN_X,
        WIN_Y,
        FRAME_W * WIN_SCALE,
        FRAME_H * WIN_SCALE,
        0);
    ppu->sdl_ren = SDL_CreateRenderer(ppu->sdl_win, -1, SDL_RENDERER_ACCELERATED|SDL_RENDERER_PRESENTVSYNC);
    //ppu->sdl_ren = SDL_CreateRenderer(ppu->sdl_win, -1, SDL_RENDERER_ACCELERATED);
    ppu->sdl_frame = SDL_CreateRGBSurfaceWithFormat(0, FRAME_W, FRAME_H, 15, SDL_PIXELFORMAT_BGR555);

    return ppu;
}

void ppu_update(Ppu *this, u32 cycles)
{
    this->cycles += cycles;

    u8 dispstat = this->reg[REG_DISPSTAT];
    u8 ly = this->reg[REG_VCOUNT];

    switch (this->state) {
        case PPU_STATE_HDRAW:
            if (this->cycles >= 960) {
                ppu_draw_scaneline(this);
                this->state = PPU_STATE_HBLANK;

                dispstat &= ~3;
                dispstat |= BIT_1;
            }
        break;
        case PPU_STATE_HBLANK:
            if (this->cycles >= 1232) {
                this->cycles %= 1232;
                ly++;
                ppu_set_ly(this, ly);
                if (ly == 160) {
                    this->state = PPU_STATE_VBLANK;

                    dispstat &= ~3;
                    dispstat |= 1;

                    SDL_Texture *texture = SDL_CreateTextureFromSurface(this->sdl_ren, this->sdl_frame);
                    SDL_RenderClear(this->sdl_ren);
                    SDL_RenderCopy(this->sdl_ren, texture, NULL, NULL);
                    SDL_RenderPresent(this->sdl_ren);
                    SDL_DestroyTexture(texture);
                } else {
                    this->state = PPU_STATE_HDRAW;

                    dispstat &= ~3;
                }
            }
        break;
        case PPU_STATE_VBLANK:
            if (this->cycles >= 1232) {
                this->cycles %= 1232;
                ly++;
                ppu_set_ly(this, ly);
                if (ly == 160+68) {
                    this->state = PPU_STATE_HDRAW;
                    dispstat &= ~3;
                }
            }
        break;
    }

    this->reg[REG_DISPSTAT] = dispstat;
}

void ppu_free(Ppu *this)
{
    SDL_DestroyWindow(this->sdl_win);
    SDL_DestroyRenderer(this->sdl_ren);
}

static void ppu_draw_scaneline(Ppu *this)
{
    u8 mode = bus_read(this->bus, 0x4000000) & 7;
    switch (mode) {
        case 3: {
            u32 offset = this->reg[REG_VCOUNT] * FRAME_W * 2;
            u8 *line = this->vram + this->reg[REG_VCOUNT] * FRAME_W * 2;
            memcpy((u8 *)this->sdl_frame->pixels + offset, line, FRAME_W * 2);
        }
        break;
        case 4: {
            u8 *line = this->vram + this->reg[REG_VCOUNT] * FRAME_W;
            u8 *pixels = (u8 *)this->sdl_frame->pixels + this->reg[REG_VCOUNT] * FRAME_W * 2;
            for (uint x = 0; x < FRAME_W; ++x) {
                pixels[x*2+0] = this->plt[line[x] * 2];
                pixels[x*2+1] = this->plt[line[x] * 2 + 1];
            }
        }
        break;
    }
}

static void ppu_set_ly(Ppu *this, u8 ly)
{
    assert(ly <= 160+68);

    if (ly == 160+68) {
        ly = 0;
    }

    u8 dispstat = this->reg[REG_DISPSTAT];
    u8 lyc = this->reg[REG_DISPSTAT+1];
    if (ly == lyc) {
        // set V-Counter flag
        dispstat |= BIT_2;

        // set bit 2 of IF
        if (bit(dispstat, 5)) {
            // TODO
        }
    } else {
        // clear V-Counter flag
        dispstat &= ~BIT_2;
    }

    this->reg[REG_VCOUNT] = ly;
    this->reg[REG_DISPSTAT] = dispstat;
}

static u32 plt_read(void *dev, u32 addr, u8 width)
{
    Ppu *this = dev;
    addr &= PLT_SIZE - 1;
    return read_memory(this->plt+addr, width);
}

static u32 oam_read(void *dev, u32 addr, u8 width)
{
    Ppu *this = dev;
    addr &= OAM_SIZE - 1;
    return read_memory(this->oam+addr, width);
}

static u32 vram_read(void *dev, u32 addr, u8 width)
{
    Ppu *this = dev;
    addr -= VRAM_ADDR;
    return read_memory(this->vram+addr, width);
}

static u32 lcd_read(void *dev, u32 addr, u8 width)
{
    Ppu *this = dev;
    addr &= 0xFF;
    return read_memory(this->reg+addr, width);
}

static void plt_write(void *dev, u32 addr, u8 width, u32 data)
{
    Ppu *this = dev;
    addr &= PLT_SIZE - 1;
    write_memory(this->plt+addr, width, data);
}

static void oam_write(void *dev, u32 addr, u8 width, u32 data)
{
    Ppu *this = dev;
    addr &= OAM_SIZE - 1;
    write_memory(this->oam+addr, width, data);
}

static void vram_write(void *dev, u32 addr, u8 width, u32 data)
{
    Ppu *this = dev;
    addr -= VRAM_ADDR;
    write_memory(this->vram+addr, width, data);
}

static void lcd_write(void *dev, u32 addr, u8 width, u32 data)
{
    Ppu *this = dev;
    addr &= 0xFF;
    write_memory(this->reg+addr, width, data);
}
