#include <assert.h>

#include "keypad.h"

#define KEY_UP      SDLK_p
#define KEY_DOWN    SDLK_SEMICOLON
#define KEY_LEFT    SDLK_l
#define KEY_RIGHT   SDLK_QUOTE
#define KEY_A       SDLK_c
#define KEY_B       SDLK_x
#define KEY_R       SDLK_s
#define KEY_L       SDLK_a
#define KEY_START   SDLK_RETURN
#define KEY_SELECT  SDLK_BACKSPACE

static u32 keypad_read(void *this, u32 addr, u8 width);
static void keypad_write(void *this, u32 addr, u8 width, u32 data);

static const SDL_Keycode keymap [10] = {
    KEY_A,
    KEY_B,
    KEY_SELECT,
    KEY_START,
    KEY_RIGHT,
    KEY_LEFT,
    KEY_UP,
    KEY_DOWN,
    KEY_R,
    KEY_L,
};

Keypad *keypad_init(Bus *bus)
{
    Keypad *keyp = malloc(sizeof(Keypad));

    keyp->bus = bus;

    // all buttons released
    keyp->reg[0] = 0xFF;
    keyp->reg[1] = 0xFF;

    BusDev dev = {keyp, keypad_read, keypad_write};
    bus_attach_keypad(bus, &dev);

    return keyp;
}

void update_keypad(Keypad *this, const SDL_Event *ev)
{
    u16 buttons = this->reg[0] | (this->reg[1] << 8);

    for (uint i = 0; i < len(keymap); i++) {
        if (ev->key.keysym.sym != keymap[i])
            continue;

        if (ev->type == SDL_KEYDOWN) {
            clear_bit(buttons, i);
        } else {
            set_bit(buttons, i);
        }

        break;
    }
    this->reg[0] = buttons;
    this->reg[1] = buttons >> 8;
}

static void write_keypad_register(Keypad *this, u32 addr, u8 data)
{
    if (addr > 1) {
        this->reg[addr] = data;
    }
}

static u32 keypad_read(void *dev, u32 addr, u8 width)
{
    Keypad *this = dev;
    addr &= align_mask[width];
    addr &= 0xF;
    return read_memory(this->reg+addr, width);
}

static void keypad_write(void *dev, u32 addr, u8 width, u32 data)
{
    Keypad *this = dev;
    addr &= align_mask[width];
    addr &= 0xF;
    switch (width) {
        case WIDTH_32:
            write_keypad_register(this, addr+3, data >> 24);
            write_keypad_register(this, addr+2, data >> 16);
        case WIDTH_16:
            write_keypad_register(this, addr+1, data >> 8);
        case WIDTH_8:
            write_keypad_register(this, addr, data);
        break;
        default:
            assert(0);
        break;
    }
}
