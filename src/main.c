#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <SDL.h>

#include "cpu.h"
#include "bus.h"
#include "ppu.h"

enum State {STATE_RUNNING, STATE_PAUSED, STATE_QUIT};

u32 update_input(u32 state)
{
    SDL_Event ev;
    static u32 saved_time = 0;

    if (SDL_GetTicks64() - saved_time < 16) {
        return state;
    }
    saved_time = SDL_GetTicks64();

    while (SDL_PollEvent(&ev)) {
        if (ev.type == SDL_QUIT) {
            return STATE_QUIT;
            puts("quit pls");
        }

        if (ev.type == SDL_KEYDOWN) {
            switch (ev.key.keysym.sym) {
                case SDLK_q:
                    return STATE_QUIT;
                case SDLK_p:
                    if (state == STATE_RUNNING)
                        return STATE_PAUSED;
                    else
                        return STATE_RUNNING;
                break;
            }
        }
    }
    return state;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        puts("usage: gba ROM BIOS");
        return 1;
    }

    SDL_Init(SDL_INIT_EVERYTHING);

    Bus *bus = bus_init(argv[1], argv[2]);
    Cpu *cpu = cpu_init(bus);
    Ppu *ppu = ppu_init(bus);

    u32 state = STATE_RUNNING;
    u32 counter = 0;
    while (1) {
        if (counter >= 10000) {
            counter = 0;
            state = update_input(state);
        }
        if (state == STATE_PAUSED) {
            continue;
        } else if (state == STATE_QUIT) {
            break;
        }

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
