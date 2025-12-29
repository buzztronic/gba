#include "ppu.h"

#define WIN_TITLE "GBA Emulator"
#define WIN_X SDL_WINDOWPOS_CENTERED
#define WIN_Y SDL_WINDOWPOS_CENTERED
#define FRAME_W 240
#define FRAME_H 160
#define WIN_SCALE 4

static void ppu_draw_scaneline(Ppu *this);

enum PpuState {PPU_STATE_HDRAW, PPU_STATE_HBLANK, PPU_STATE_VBLANK};

Ppu *ppu_init(Bus *bus)
{
    Ppu *ppu = malloc(sizeof(Ppu));

    ppu->bus = bus;
    ppu->ly = 0;
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

    switch (this->state) {
        case PPU_STATE_HDRAW:
            if (this->cycles >= 960) {
                ppu_draw_scaneline(this);
                this->state = PPU_STATE_HBLANK;
            }
        break;
        case PPU_STATE_HBLANK:
            if (this->cycles >= 1232) {
                this->cycles %= 1232;
                this->ly += 1;
                if (this->ly == 160) {
                    this->state = PPU_STATE_VBLANK;

                    SDL_Texture *texture = SDL_CreateTextureFromSurface(this->sdl_ren, this->sdl_frame);
                    SDL_RenderClear(this->sdl_ren);
                    SDL_RenderCopy(this->sdl_ren, texture, NULL, NULL);
                    SDL_RenderPresent(this->sdl_ren);
                    SDL_DestroyTexture(texture);

                    // enter vblank
                    // TODO: update LCD STAT
                    bus_write(this->bus, IO_ADDR+4, 1);
                } else {
                    this->state = PPU_STATE_HDRAW;
                }
            }
        break;
        case PPU_STATE_VBLANK:
            if (this->cycles >= 1232) {
                this->cycles %= 1232;
                this->ly += 1;
                if (this->ly == 160+68) {
                    this->ly = 0;
                    this->state = PPU_STATE_HDRAW;
                    // leave vblank
                    // TODO: update LCD STAT
                    bus_write(this->bus, IO_ADDR+4, 0);
                }
            }
        break;
    }
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
            u32 offset = this->ly * FRAME_W * 2;
            u8 *line = bus_getvram(this->bus) + this->ly * FRAME_W * 2;
            memcpy((u8 *)this->sdl_frame->pixels + offset, line, FRAME_W * 2);
        }
        break;
        case 4: {
            u8 *line = bus_getvram(this->bus) + this->ly * FRAME_W;
            u8 *pixels = (u8 *)this->sdl_frame->pixels + this->ly * FRAME_W * 2;
            for (uint x = 0; x < FRAME_W; ++x) {
                u16 color = bus_read16(this->bus, 0x05000000 + line[x] * 2);
                pixels[x*2+0] = bits(color, 0, 8);
                pixels[x*2+1] = bits(color, 8, 8);
            }
        }
        break;
    }
}
