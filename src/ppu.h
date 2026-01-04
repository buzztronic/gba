#pragma once

#include <SDL.h>
#include "bus.h"

typedef struct Ppu {
    SDL_Window *sdl_win;
    SDL_Renderer *sdl_ren;
    SDL_Surface *sdl_frame;

    u8 plt[PLT_SIZE];
    u8 vram[VRAM_SIZE];
    u8 oam[OAM_SIZE];

    Bus *bus;

    uint cycles;
    u32 state;

    // LCD I/O Registers
    u8 reg[0x60];
} Ppu;

Ppu *ppu_init(Bus *bus);
void ppu_update(Ppu *this, u32 cycles);
void ppu_free(Ppu *this);
