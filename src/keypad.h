#pragma once

#include <SDL.h>

#include "bus.h"

typedef struct Keypad {
    Bus *bus;

    u8 reg[0x10];
} Keypad;

Keypad *keypad_init(Bus *bus);

void update_keypad(Keypad *this, const SDL_Event *ev);
