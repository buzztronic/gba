#pragma once

#include <SDL.h>
#include "bus.h"

typedef struct Ppu {
    SDL_Window *sdl_win;
    SDL_Renderer *sdl_ren;

    Bus *bus;

    uint ly;
    uint cycles;
    u32 state;
} Ppu;

Ppu *ppu_init(Bus *bus);
void ppu_update(Ppu *this, u32 cycles);
void ppu_free(Ppu *this);
