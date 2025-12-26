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
                    SDL_RenderPresent(this->sdl_ren);
                    // enter vblank
                    // TODO: update LCD STAT
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
    // 05000000-050001FF - BG Palette RAM (512 bytes, 256 colors)
    // 06000000-06009FFF  40 KBytes Frame 0 buffer (only 37.5K used in Mode 4)
    // 0600A000-06013FFF  40 KBytes Frame 1 buffer (only 37.5K used in Mode 4)

    u8 *line = bus_getvram(this->bus) + this->ly * FRAME_W * 2;
    SDL_Rect rect;

    rect.w = rect.h = WIN_SCALE;
    for (uint x = 0; x < FRAME_W; x++) {
        u32 color = line[x*2] | (line[x*2+1] << 8);

        u32 r = bits(color, 0, 5);
        u32 g = bits(color, 5, 5);
        u32 b = bits(color, 10, 5);

        r = (r << 3) | (r >> 2);
        g = (g << 3) | (g >> 2);
        b = (b << 3) | (b >> 2);

        rect.x = x * WIN_SCALE;
        rect.y = this->ly * WIN_SCALE;

        SDL_SetRenderDrawColor(this->sdl_ren, r, g, b, 0);
        SDL_RenderFillRect(this->sdl_ren, &rect);
    }
}
