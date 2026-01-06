#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <SDL.h>

#include "cpu.h"
#include "bus.h"
#include "dma.h"
#include "ppu.h"
#include "keypad.h"

enum State {STATE_RUNNING, STATE_PAUSED, STATE_QUIT};

#define KEY_PAUSE   SDLK_SPACE
#define KEY_CLOSE   SDLK_q

u32 handle_inputs(Keypad *keyp, u32 state)
{
    SDL_Event ev;

    while (SDL_PollEvent(&ev)) {
        if (ev.type == SDL_QUIT) {
            return STATE_QUIT;
            puts("quit pls");
        }

        if (ev.type == SDL_KEYDOWN) {
            switch (ev.key.keysym.sym) {
                case KEY_CLOSE:
                    return STATE_QUIT;
                case KEY_PAUSE:
                    if (state == STATE_RUNNING)
                        return STATE_PAUSED;
                    else
                        return STATE_RUNNING;
                break;
            }
        }

        if (ev.type == SDL_KEYDOWN || ev.type == SDL_KEYUP) {
            update_keypad(keyp, &ev);
        }
    }
    return state;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        puts("usage: gba BIOS ROM");
        return 1;
    }

    SDL_Init(SDL_INIT_EVERYTHING);

    Bus *bus = bus_init(argv[2], argv[1]);
    Cpu *cpu = cpu_init(bus);
    Ppu *ppu = ppu_init(bus);
    Dma *dma = dma_init(bus);
    Keypad *keyp = keypad_init(bus);

    u32 state = STATE_RUNNING;
    u32 counter = 0;
    while (1) {
        if (counter >= 1000) {
            counter = 0;
            state = handle_inputs(keyp, state);
        }
        if (state == STATE_PAUSED) {
            counter += 1;
            continue;
        } else if (state == STATE_QUIT) {
            break;
        }

        dma_update(dma);

        // proper timing will be implemented later
        int n = cpu_step(cpu);
        counter += n;

        if (n == 0)
            break;

        ppu_update(ppu, 4);
    }

    // do we really need to free memory now?
    free(bus);
    free(cpu);
    ppu_free(ppu);

    SDL_Quit();

	return 0;
}
