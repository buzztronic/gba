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
#define REG_BLDCNT 0x50

const u32 REG_BGX_CNT[] = { 0x08, 0x0A, 0x0C, 0x0E };
const u32 REG_BGX_X_OFF[] = { 0x10, 0x14, 0x18, 0x1C };
const u32 REG_BGX_Y_OFF[] = { 0x12, 0x16, 0x1A, 0x1E };
const u32 REG_BGX_PA[] = { 0x20, 0x30 };
const u32 REG_BGX_PB[] = { 0x22, 0x32 };
const u32 REG_BGX_PC[] = { 0x24, 0x34 };
const u32 REG_BGX_PD[] = { 0x26, 0x36 };
const u32 REG_BGX_X[] = { 0x28, 0x38 };
const u32 REG_BGX_Y[] = { 0x2C, 0x3C };

typedef struct WinDim {
    u32 x1, x2;
    u32 y1, y2;
} WinDim;

typedef struct Obj {
    // att0
    i16 y;
    u8 affine;
    u8 double_size;
    u8 disable;
    u8 mode;
    u8 depth; // 0: 4bpp, 1: 8bpp
    u8 shape; // 0: square, 1: horizontal, 2: vertical

    // att1
    i16 x;
    u8 affine_idx;
    u8 hflip;
    u8 vflip;
    u8 size;

    // att 2
    u16 tile_no; // 0-1023
    u8 palette; // used for depth 4bpp
    u8 priority;
} Obj;

typedef struct ObjSize {
    u32 w;
    u32 h;
} ObjSize;

static void ppu_draw_scaneline(Ppu *this);
static void ppu_set_ly(Ppu *this, u8 ly);
static void ppu_render_mode0(Ppu *this, u16 line[FRAME_W], u8 bg);
static void ppu_render_affine(Ppu *this, u16 line[FRAME_W], u8 bg);
static void ppu_render_objs(Ppu *this, u16 line[FRAME_W], u8 mode[FRAME_W], u8 priority[FRAME_W], u8 win[FRAME_W]);
static int is_inside_win(u32 x, u32 y, const WinDim *dim);
static inline double fixed_point32_to_double(u32 n);
static inline double fixed_point16_to_double(u16 n);
static void ppu_reload_bg_reference(Ppu *this, u8 bg);
static void ppu_increment_bg_reference(Ppu *this);
static inline void ppu_apply_mosaic(Ppu *this, u32 *x, u32 *y);
static inline void ppu_apply_obj_mosaic(Ppu *this, u32 *x, u32 *y);

static u32 plt_read(void *, u32 addr, u8 width);
static u32 oam_read(void *, u32 addr, u8 width);
static u32 vram_read(void *, u32 addr, u8 width);
static u32 lcd_read(void *, u32 addr, u8 width);
static void plt_write(void *, u32 addr, u8 width, u32 data);
static void oam_write(void *, u32 addr, u8 width, u32 data);
static void vram_write(void *, u32 addr, u8 width, u32 data);
static void lcd_write(void *, u32 addr, u8 width, u32 data);


Ppu *ppu_init(Bus *bus)
{
    Ppu *ppu = malloc(sizeof(Ppu));
    memset(ppu, 0, sizeof(*ppu));

    BusDev plt_dev = {ppu, plt_read, plt_write};
    BusDev oam_dev = {ppu, oam_read, oam_write};
    BusDev vram_dev = {ppu, vram_read, vram_write};
    BusDev lcd_dev = {ppu, lcd_read, lcd_write};

    ppu->bus = bus;
    bus_attach_plt(bus, &plt_dev);
    bus_attach_oam(bus, &oam_dev);
    bus_attach_vram(bus, &vram_dev);
    bus_attach_lcd(bus, &lcd_dev);

    ppu->cycles = 0;
    ppu->state = PPU_STATE_HDRAW;
    ppu->reg[REG_BGX_PA[0]+1] = 1;
    ppu->reg[REG_BGX_PD[0]+1] = 1;
    ppu->reg[REG_BGX_PA[1]+1] = 1;
    ppu->reg[REG_BGX_PD[1]+1] = 1;

    bus_notify_ppu_state(bus, ppu->state);

    ppu->sdl_win = SDL_CreateWindow(WIN_TITLE,
        WIN_X,
        WIN_Y,
        FRAME_W * WIN_SCALE,
        FRAME_H * WIN_SCALE,
        0);
    ppu->sdl_ren = SDL_CreateRenderer(ppu->sdl_win, -1, SDL_RENDERER_ACCELERATED|SDL_RENDERER_PRESENTVSYNC);
    //ppu->sdl_ren = SDL_CreateRenderer(ppu->sdl_win, -1, SDL_RENDERER_ACCELERATED);
    ppu->sdl_frame = SDL_CreateRGBSurfaceWithFormat(0, FRAME_W, FRAME_H, 15, SDL_PIXELFORMAT_BGR555);
    memset(ppu->sdl_frame->pixels, 0x55, FRAME_W * FRAME_H * 2);

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
                bus_notify_ppu_state(this->bus, this->state);

                if (dispstat & BIT_4) {
                    bus_send_irq(this->bus, IRQ_HBLANK);
                }

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
                    bus_notify_ppu_state(this->bus, this->state);

                    dispstat &= ~3;
                    dispstat |= 1;

                    if (dispstat & BIT_3) {
                        bus_send_irq(this->bus, IRQ_VBLANK);
                    }

                    SDL_Texture *texture = SDL_CreateTextureFromSurface(this->sdl_ren, this->sdl_frame);
                    SDL_RenderClear(this->sdl_ren);
                    SDL_RenderCopy(this->sdl_ren, texture, NULL, NULL);
                    SDL_RenderPresent(this->sdl_ren);
                    SDL_DestroyTexture(texture);
                } else {
                    this->state = PPU_STATE_HDRAW;
                    bus_notify_ppu_state(this->bus, this->state);

                    dispstat &= ~3;
                }
            }
        break;
        case PPU_STATE_VBLANK:
            // H-Blank flag also set inside vblank period
            if (this->cycles - cycles < 960  && this->cycles >= 960) {
                if (dispstat & BIT_4) {
                    bus_send_irq(this->bus, IRQ_HBLANK);
                }

                dispstat &= ~3;
                dispstat |= BIT_0 | BIT_1;
            }

            if (this->cycles >= 1232) {
                dispstat &= ~3;
                dispstat |= BIT_0;

                this->cycles %= 1232;
                ly++;
                ppu_set_ly(this, ly);
                if (ly == 227) {
                    dispstat &= ~3;
                } else if (ly == 228) {
                    this->state = PPU_STATE_HDRAW;
                    bus_notify_ppu_state(this->bus, this->state);

                    ppu_reload_bg_reference(this, 2);
                    ppu_reload_bg_reference(this, 3);
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
    const u16 dispcnt = read_memory(this->reg + REG_DISPCNT, WIDTH_16);
    const u32 y = this->reg[REG_VCOUNT];

    if (bit(dispcnt, 7)) {
        // force blank
        // TODO: restrict to vram, oam and plt when force blank is off
        // and when allow access to oam during h-blank is off
        memset((u8 *)this->sdl_frame->pixels + y * FRAME_W * 2, 0xFF, FRAME_W * 2);
        return;
    }

    const u8 bgmode = bits(dispcnt, 0, 3);
    const u8 win0_enable = bit(dispcnt, 13);
    const u8 win1_enable = bit(dispcnt, 14);
    const u8 winobj_enable = bit(dispcnt, 15) && bit(dispcnt, 12);

    u16 backdrop = read_memory(this->plt, WIDTH_16);

    // since bit 15 in colors is not used. I'll use it here to flag
    // transparent pixels. 0 = normal, 1 = transparent
    u16 bg_line[4][FRAME_W] = {0};
    u16 obj_line[FRAME_W] = {0};
    u8 obj_priority[FRAME_W] = {0}; // priority relative to bg
    u8 obj_mode[FRAME_W] = {0}; // used to detect semi-transparent objs
    u8 obj_win_mask[FRAME_W] = {0}; // represents obj window

    const u8 win_0 = this->reg[0x48];
    const u8 win_1 = this->reg[0x49];
    const u8 win_out = this->reg[0x4A];
    const u8 win_obj = this->reg[0x4B];

    const u8 obj_enable = bit(dispcnt, 12);
    const u8 bg_enable[4] = {
        bit(dispcnt, 8 + 0) && bgmode < 2,
        bit(dispcnt, 8 + 1) && bgmode < 2,
        bit(dispcnt, 8 + 2),
        bit(dispcnt, 8 + 3) && (bgmode == 2 || bgmode == 0),
    };

    const u8 bg_priority[4] = {
        bits(this->reg[REG_BGX_CNT[0]], 0, 2),
        bits(this->reg[REG_BGX_CNT[1]], 0, 2),
        bits(this->reg[REG_BGX_CNT[2]], 0, 2),
        bits(this->reg[REG_BGX_CNT[3]], 0, 2),
    };

    const u16 bldcnt = read_memory(this->reg + REG_BLDCNT, WIDTH_16);
    const u8 efx_mode = bits(bldcnt, 6, 2);

    const WinDim win0_dim = {
        .x2 = this->reg[0x40], .x1 = this->reg[0x41],
        .y2 = this->reg[0x44], .y1 = this->reg[0x45],
    };

    const WinDim win1_dim = {
        .x2 = this->reg[0x42], .x1 = this->reg[0x43],
        .y2 = this->reg[0x46], .y1 = this->reg[0x47],
    };

    // reload background affine reference
    if (this->reload_bg_ref[0]) {
        this->reload_bg_ref[0] = 0;
        ppu_reload_bg_reference(this, 2);
    }

    if (this->reload_bg_ref[1]) {
        this->reload_bg_ref[1] = 0;
        ppu_reload_bg_reference(this, 3);
    }

    // render backgrounds
    switch (bgmode) {
        case 0:
            ppu_render_mode0(this, bg_line[0], 0);
            ppu_render_mode0(this, bg_line[1], 1);
            ppu_render_mode0(this, bg_line[2], 2);
            ppu_render_mode0(this, bg_line[3], 3);
        break;
        case 1:
            ppu_render_mode0(this, bg_line[0], 0);
            ppu_render_mode0(this, bg_line[1], 1);
            ppu_render_affine(this, bg_line[2], 2);
        break;
        case 2:
            ppu_render_affine(this, bg_line[2], 2);
            ppu_render_affine(this, bg_line[3], 3);
        break;
        case 3:
            ppu_render_affine(this, bg_line[2], 2);
        break;
        case 4:
            ppu_render_affine(this, bg_line[2], 2);
        break;
        case 5:
            ppu_render_affine(this, bg_line[2], 2);
        break;
    }

    // render objs
    if (obj_enable) {
        ppu_render_objs(this, obj_line, obj_mode, obj_priority, obj_win_mask);
    }

    u8 *const pixels = (u8 *)this->sdl_frame->pixels + y * FRAME_W * 2;
    for (u32 x = 0; x < FRAME_W; x++) {
        // determine active window
        u8 win = 0xFF;
        if (win0_enable && is_inside_win(x, y, &win0_dim)) {
            win = win_0;
        } else if (win1_enable && is_inside_win(x, y, &win1_dim)) {
            win = win_1;
        } else if (winobj_enable && obj_win_mask[x]) {
            win = win_obj;
        } else if (win0_enable || win1_enable || winobj_enable) {
            win = win_out;
        }

        const u8 efx = bit(win, 5);

        // determine the 2 highest priority pixels t1 and t2
        u16 t1 = backdrop;
        u16 t2 = backdrop;
        u8 t1_idx = 5;
        u8 t2_idx = 5;

        struct PixelInfo {
            u16 color;
            u8 enable;
            u8 priority;
            u8 bld_idx;
        } const pinfo[5] = {
            { obj_line[x], obj_enable && bit(win, 4), obj_priority[x], 4 },
            { bg_line[0][x], bg_enable[0] && bit(win, 0), bg_priority[0], 0 },
            { bg_line[1][x], bg_enable[1] && bit(win, 1), bg_priority[1], 1 },
            { bg_line[2][x], bg_enable[2] && bit(win, 2), bg_priority[2], 2 },
            { bg_line[3][x], bg_enable[3] && bit(win, 3), bg_priority[3], 3 },
        };

        u32 p = 0;
        // determine t1
        for (; p < 4; p++) {
            u8 found = 0;
            for (u32 i = 0; i < 5; i++) {
                if (pinfo[i].enable && pinfo[i].priority == p && bit(pinfo[i].color, 15)) {
                    t1 = pinfo[i].color;
                    t1_idx = pinfo[i].bld_idx;
                    found++;
                    break;
                }
            }

            if (found)
                break;
        }

        // determine t2
        for (; t1_idx != 5 && efx && p < 4; p++) {
            u8 found = 0;
            for (u32 i = 0; i < 5; i++) {
                if (pinfo[i].enable && pinfo[i].priority == p &&
                    bit(pinfo[i].color, 15) && pinfo[i].bld_idx != t1_idx)
                {
                    t2 = pinfo[i].color;
                    t2_idx = pinfo[i].bld_idx;
                    found = 1;
                    break;
                }
            }

            if (found)
                break;
        }

        // determine the effect to apply
        const u8 cond_table[4] = {
            1,
            bit(bldcnt, t1_idx) && bit(bldcnt, 8+t2_idx),
            bit(bldcnt, t1_idx),
            bit(bldcnt, t1_idx),
        };

        u8 actual_efx_mode = efx_mode;
        if (t1_idx == 4 && obj_mode[x] == 1 && bit(bldcnt, 8+t2_idx)) {
            // TODO: does this apply for all values of efx_mode or just 0?
            actual_efx_mode = 1;
        } else if (!efx || !cond_table[efx_mode]) {
            actual_efx_mode = 0;
        }

        // apply color effect
        if (actual_efx_mode != 0) {
            u16 r1 = bits(t1, 0, 5);
            u16 g1 = bits(t1, 5, 5);
            u16 b1 = bits(t1, 10, 5);
            if (actual_efx_mode == 1) {
                // alpha blending
                const double eva = min(16, bits(this->reg[0x52], 0, 5)) / 16.0;
                const double evb = min(16, bits(this->reg[0x53], 0, 5)) / 16.0;
                const u16 r2 = bits(t2, 0, 5);
                const u16 g2 = bits(t2, 5, 5);
                const u16 b2 = bits(t2, 10, 5);

                r1 = min(31, r1*eva + r2*evb);
                g1 = min(31, g1*eva + g2*evb);
                b1 = min(31, b1*eva + b2*evb);
            } else {
                const double evy = min(16, bits(this->reg[0x54], 0, 5)) / 16.0;
                if (actual_efx_mode == 2) {
                    // increase brightness
                    r1 += (31-r1) * evy;
                    g1 += (31-g1) * evy;
                    b1 += (31-b1) * evy;
                } else {
                    // decrease brightness
                    r1 -= r1 * evy;
                    g1 -= g1 * evy;
                    b1 -= b1 * evy;
                }
            }
            r1 &= 31;
            g1 &= 31;
            b1 &= 31;
            t1 = r1 | (g1 << 5) | (b1 << 10);
        }

        pixels[x*2+0] = t1;
        pixels[x*2+1] = t1 >> 8;
    }

    ppu_increment_bg_reference(this);
}

static int is_inside_win(u32 x, u32 y, const WinDim *dim)
{
    const int cond_y = (dim->y1 <= dim->y2) ? (y >= dim->y1 && y < dim->y2) : (y >= dim->y1 || y <= dim->y2);
    const int cond_x = (dim->x1 <= dim->x2) ? (x >= dim->x1 && x < dim->x2) : (x >= dim->x1 || x <= dim->x2);

    return cond_x && cond_y;
}

static inline u32 pixel_offset_from_tile(u32 x, u32 y, u8 depth)
{
    // 8bpp
    if (depth == 1)
        return 8 * y + x;

    // 4bpp
    return 4 * y + x / 2;
}

// return an index into the tile map matrix
// all values are in pixel unit
static inline u32 get_map_entry_idx(u32 x, u32 y, u32 x_off, u32 y_off, u32 width, u32 height)
{
    // scroll and wrap around
    x = (x + x_off) % width;
    y = (y + y_off) % height;

    // divide x and y by 8 to convert from pixel unit to tile unit
    const u32 map_x = x % 256;
    const u32 map_y = y % 256;
    const u32 map_entry_no = (map_y / 8) * 32 + (map_x / 8);

    const u32 off = (y / 256) * (width / 256) + (x / 256);

    return (2048 * off) + 2 * map_entry_no;
}

static inline void get_map_size(u8 size, u32 *w, u32 *h)
{
    switch (size) {
        case 0: *w = 256; *h = 256; break;
        case 1: *w = 512; *h = 256; break;
        case 2: *w = 256; *h = 512; break;
        case 3: *w = 512; *h = 512; break;
    }
}

static inline void get_affine_map_size(u8 size, u32 *w, u32 *h)
{
    switch (size) {
        case 0: *w = 128; *h = 128; break;
        case 1: *w = 256; *h = 256; break;
        case 2: *w = 512; *h = 512; break;
        case 3: *w = 1024; *h = 1024; break;
    }
}

static inline void ppu_apply_mosaic(Ppu *this, u32 *x, u32 *y)
{
    const u8 mos_h = bits(this->reg[0x4C], 0, 4) + 1;
    const u8 mos_v = bits(this->reg[0x4C], 4, 4) + 1;
    *x = (*x / mos_h) * mos_h;
    *y = (*y / mos_v) * mos_v;
}

static inline void ppu_apply_obj_mosaic(Ppu *this, u32 *x, u32 *y)
{
    const u8 mos_h = bits(this->reg[0x4D], 0, 4) + 1;
    const u8 mos_v = bits(this->reg[0x4D], 4, 4) + 1;
    *x = (*x / mos_h) * mos_h;
    *y = (*y / mos_v) * mos_v;
}

static void ppu_render_mode0(Ppu *this, u16 line[FRAME_W], u8 bg)
{
    const u16 dispcnt = read_memory(this->reg + REG_DISPCNT, WIDTH_16);

    if (!bit(dispcnt, 8 + bg)) {
        return;
    }

    const u16 bgcnt = read_memory(this->reg + REG_BGX_CNT[bg], WIDTH_16);

    const u32 cbase = bits(bgcnt, 2, 2) * 16 * 1024;
    const u32 sbase = bits(bgcnt, 8, 5) * 2 * 1024;
    const u8 size = bits(bgcnt, 14, 2);
    const u8 depth = bit(bgcnt, 7);

    const u32 x_off = bits(read_memory(this->reg + REG_BGX_X_OFF[bg], WIDTH_16), 0, 9);
    const u32 y_off = bits(read_memory(this->reg + REG_BGX_Y_OFF[bg], WIDTH_16), 0, 9);


    u32 width = 0, height = 0;
    get_map_size(size, &width, &height);

    const u32 sy = this->reg[REG_VCOUNT];
    for (u32 sx = 0; sx < FRAME_W; sx++) {
        u32 x = sx, y = sy;

        // mosaic
        if (bit(bgcnt, 6)) {
            ppu_apply_mosaic(this, &x, &y);
        }

        const u32 map_entry_idx = get_map_entry_idx(x, y, x_off, y_off, width, height);
        const u16 map_entry = read_memory(this->vram + sbase + map_entry_idx, WIDTH_16);
        const u16 tile_no = bits(map_entry, 0, 10);
        const u8 hflip = bit(map_entry, 10);
        const u8 vflip = bit(map_entry, 11);

        x = (x + x_off) % 8;
        y = (y + y_off) % 8;
        x = hflip ? 7 - x: x;
        y = vflip ? 7 - y: y;

        const u32 pixel_idx = tile_no * (depth == 1 ? 64 : 32) + pixel_offset_from_tile(x, y, depth);

        u8 id = this->vram[cbase + pixel_idx];
        if (depth == 0) {
            if (bit(x, 0) == 1) {
                id >>= 4;
            }

            id &= 0xF;
            if (id != 0) {
                id |= bits(map_entry, 12, 4) << 4;
            }
        }

        if (id != 0) {
            line[sx] = BIT_15 | this->plt[id*2] | (this->plt[id*2+1] << 8);
        }
    }
}

static inline double fixed_point32_to_double(u32 n)
{
    u32 integer = bits(n, 0, 27);

    // sign extend
    if (bit(n, 27))
        integer |= (u32)~0 << 27;

    return (i32)integer / 256.0;
}

static inline double fixed_point16_to_double(u16 n)
{
    return (i16)n / 256.0;
}

static void ppu_reload_bg_reference(Ppu *this, u8 bg)
{
    const u32 x0 = read_memory(this->reg + REG_BGX_X[bg-2], WIDTH_32);
    const u32 y0 = read_memory(this->reg + REG_BGX_Y[bg-2], WIDTH_32);
    this->x0[bg-2] = fixed_point32_to_double(x0);
    this->y0[bg-2] = fixed_point32_to_double(y0);
}

static void ppu_increment_bg_reference(Ppu *this)
{
    const u16 bg2_pb = read_memory(this->reg + REG_BGX_PB[0], WIDTH_16);
    const u16 bg2_pd = read_memory(this->reg + REG_BGX_PD[0], WIDTH_16);
    const double bg2_dmx = fixed_point16_to_double(bg2_pb);
    const double bg2_dmy = fixed_point16_to_double(bg2_pd);
    this->x0[0] += bg2_dmx;
    this->y0[0] += bg2_dmy;

    const u16 bg3_pb = read_memory(this->reg + REG_BGX_PB[1], WIDTH_16);
    const u16 bg3_pd = read_memory(this->reg + REG_BGX_PD[1], WIDTH_16);
    const double bg3_dmx = fixed_point16_to_double(bg3_pb);
    const double bg3_dmy = fixed_point16_to_double(bg3_pd);
    this->x0[1] += bg3_dmx;
    this->y0[1] += bg3_dmy;
}

static inline void ppu_get_dx_dy(Ppu *this, double *dx, double *dy, u8 bg)
{
    const u16 reg_pa = read_memory(this->reg + REG_BGX_PA[bg-2], WIDTH_16);
    const u16 reg_pc = read_memory(this->reg + REG_BGX_PC[bg-2], WIDTH_16);

    *dx = fixed_point16_to_double(reg_pa);
    *dy = fixed_point16_to_double(reg_pc);
}

static void ppu_render_affine(Ppu *this, u16 line[FRAME_W], u8 bg)
{
    assert(bg == 2 || bg == 3);
    const u16 dispcnt = read_memory(this->reg + REG_DISPCNT, WIDTH_16);
    const u8 bgmode = bits(dispcnt, 0, 3);

    if (!bit(dispcnt, 8 + bg)) {
        return;
    }

    const u16 bgcnt = read_memory(this->reg + REG_BGX_CNT[bg], WIDTH_16);

    const u32 cbase = bits(bgcnt, 2, 2) * 16 * 1024;
    const u32 sbase = bits(bgcnt, 8, 5) * 2 * 1024;
    const u8 size = bits(bgcnt, 14, 2);

    double dx, dy;
    ppu_get_dx_dy(this, &dx, &dy, bg);

    u32 width = 240, height = 160;
    if (bgmode < 3) {
        get_affine_map_size(size, &width, &height);
    }

    double x2 = this->x0[bg-2];
    double y2 = this->y0[bg-2];
    for (u32 sx = 0; sx < FRAME_W; sx++, x2 += dx, y2 += dy) {
        i32 x = x2;
        i32 y = y2;

        if (x < 0 || y < 0 || x >= (i32)width || y >= (i32)height) {
            if (bgmode > 2 || !bit(bgcnt, 13)) {
                line[sx] = 0;
                continue;
            } else {
                if (x < 0) {
                    x = width - (-x) % width;
                } else {
                    x %= width;
                }

                if (y < 0) {
                    y = height - (-y) % height;
                } else{
                    y %= height;
                }
            }
        }

        if (bit(bgcnt, 6)) {
            ppu_apply_mosaic(this, (u32 *)&x, (u32 *)&y);
        }

        if (bgmode <= 2) {
            const u32 map_entry_idx = (width / 8) * (y / 8) + (x / 8);
            const u8 tile_no = this->vram[sbase + map_entry_idx];

            u8 id = this->vram[cbase + tile_no * 64 + pixel_offset_from_tile(x % 8, y % 8, 1)];

            if (id != 0) {
                line[sx] = BIT_15 | this->plt[id*2] | (this->plt[id*2+1] << 8);
            }
            continue;
        }

        if (bgmode == 3) {
            const u32 off = y * FRAME_W * 2;
            line[sx] = BIT_15 | read_memory(this->vram + off + x * 2, WIDTH_16);
            continue;
        }

        if (bgmode == 4) {
            u32 off = y * FRAME_W;
            if (bit(dispcnt, 4))
                off += 0xA000;

            u8 id = this->vram[off + x];

            if (id != 0) {
                line[sx] = BIT_15 | this->plt[id*2] | (this->plt[id*2+1] << 8);
            }
            continue;
        }

        if (bgmode == 5) {
            u32 off = y * 160 * 2;
            if (bit(dispcnt, 4))
                off += 0xA000;

            if (x < (i32)width && y < (i32)height) {
                line[sx] = BIT_15 | read_memory(this->vram + off + x * 2, WIDTH_16);
            }
            continue;
        }
    }
}

static inline const ObjSize* get_obj_size(const Obj *o)
{
    const static ObjSize size[4][4] = {
        { {8, 8}, {16, 16}, {32, 32}, {64, 64}, },
        { {16, 8}, {32, 8}, {32, 16}, {64, 32}, },
        { {8, 16}, {8, 32}, {16, 32}, {32, 64}, },
    };

    return &size[o->shape][o->size];
}

static void get_obj(Ppu *this, Obj *o, u8 i)
{
    const u32 addr = i * 8;

    u16 att0 = read_memory(this->oam+addr+0, WIDTH_16);
    u16 att1 = read_memory(this->oam+addr+2, WIDTH_16);
    u16 att2 = read_memory(this->oam+addr+4, WIDTH_16);

    // attribute 0
    o->y = bits(att0, 0, 8);
    if (o->y >= FRAME_H)
        o->y -= 256;

    o->affine = bit(att0, 8);
    o->double_size = bit(att0, 9);
    o->disable = bit(att0, 9);
    o->mode = bits(att0, 10, 2);
    o->depth = bit(att0, 13);
    o->shape = bits(att0, 14, 2);

    // attribute 1
    o->x = bits(att1, 0, 9);
    if (o->x >= FRAME_W)
        o->x -= 512;

    o->affine_idx = bits(att1, 9, 5);
    o->hflip = bit(att1, 12);
    o->vflip = bit(att1, 13);
    o->size = bits(att1, 14, 2);

    // attribute 2
    o->tile_no = bits(att2, 0, 10);
    o->priority = bits(att2, 10, 2);
    o->palette = bits(att2, 12, 4);
}

static u8 get_obj_pixel_idx(Ppu *this, const Obj *o, const ObjSize *size, u32 x, u32 y)
{
    u8 mapping = bit(this->reg[REG_DISPCNT], 6);

    if (o->vflip && !o->affine) {
        y = size->h - 1 - y;
    }

    if (o->hflip && !o->affine) {
        x = size->w - 1 - x;
    }

    const u32 tile_y = y / 8;
    const u32 tile_x = x / 8;

    u16 tile_no = o->tile_no;
    u32 idx = 0;

    // 1D
    if (mapping == 1) {
        idx = tile_no + tile_y * (size->w / 8);
    }

    // 2D
    if (mapping == 0) {
        if (o->depth)
            tile_no &= ~(u32)0 << 1;
        idx = (tile_no / 32 + tile_y) * 32 + (tile_no % 32);
    }

    idx *= 32;
    idx += tile_x * (o->depth ? 64 : 32);

    idx += pixel_offset_from_tile(x % 8, y % 8, o->depth);

    u8 plt_idx = this->vram[0x10000 + idx];

    // 4bpp
    if (o->depth == 0) {
        if (bit(x, 0) == 1) {
            plt_idx >>= 4;
        }

        plt_idx &= 0xF;
        if (plt_idx != 0) {
            plt_idx |= o->palette << 4;
        }
    }

    return plt_idx;
}

static inline void ppu_load_obj_affine_matrix(Ppu *this, u8 idx, double *pa, double *pb, double *pc, double *pd)
{
    const u16 reg_pa = read_memory(this->oam + idx * 0x20 + 0x06, WIDTH_16);
    const u16 reg_pb = read_memory(this->oam + idx * 0x20 + 0x0E, WIDTH_16);
    const u16 reg_pc = read_memory(this->oam + idx * 0x20 + 0x16, WIDTH_16);
    const u16 reg_pd = read_memory(this->oam + idx * 0x20 + 0x1E, WIDTH_16);
    *pa = fixed_point16_to_double(reg_pa);
    *pb = fixed_point16_to_double(reg_pb);
    *pc = fixed_point16_to_double(reg_pc);
    *pd = fixed_point16_to_double(reg_pd);
}

static void ppu_render_objs(Ppu *this, u16 line[FRAME_W], u8 mode[FRAME_W], u8 priority[FRAME_W], u8 win[FRAME_W])
{
    const i16 sy = this->reg[REG_VCOUNT];
    const u8 bgmode = bits(this->reg[REG_DISPCNT], 0, 3);
    Obj objs[128];

    for (u32 i = 0; i < 128; i++) {
        get_obj(this, &objs[i], i);
    }

    for (i32 sx = 0; sx < FRAME_W; sx++) {
        u8 found_win = 0;
        u8 found_dot = 0;
        for (u32 i = 0; i < 128; i++) {
            const Obj obj = objs[i];
            const ObjSize *size = get_obj_size(&obj);

            // for bitmap modes objs aren't allowed to use tiles 0-511
            if (bgmode > 2 && obj.tile_no < 512)
                continue;

            if (obj.disable && !obj.affine) {
                continue;
            }

            // only used for checking boundaries
            u16 width = size->w;
            u16 height = size->h;

            if (obj.double_size && obj.affine) {
                width = size->w * 2;
                height = size->h * 2;
            }

            // TODO: from GBATEK
            // "Caution: A very large OBJ (of 128 pixels vertically,
            // ie. a 64 pixels OBJ in a Double Size area) located at Y>128
            // will be treated as at Y>-128, the OBJ is then displayed parts
            // offscreen at the TOP of the display, it is then NOT displayed
            // at the bottom."
            if (sy < obj.y || sy >= obj.y + (i16)height ||
                sx < obj.x || sx >= obj.x + (i16)width)
            {
                continue;
            }

            i16 x = sx-obj.x;
            i16 y = sy-obj.y;
            if (obj.affine) {
                double pa, pb, pc, pd;
                ppu_load_obj_affine_matrix(this, obj.affine_idx, &pa, &pb, &pc, &pd);

                if (obj.double_size) {
                    x -= size->w / 2;
                    y -= size->h / 2;
                }

                const i16 x0 = size->w / 2;
                const i16 y0 = size->h / 2;
                const i16 _x = x;
                const i16 _y = y;
                x = pa*(_x-x0) + pb*(_y-y0) + x0;
                y = pc*(_x-x0) + pd*(_y-y0) + y0;
                if (x < 0 || y < 0 || x >= (i16)size->w || y >= (i16)size->h)
                    continue;
            }

            // not accurate. but close enough?
            ppu_apply_obj_mosaic(this, (u32 *)&x, (u32 *)&y);

            const u8 idx = get_obj_pixel_idx(this, &obj, size, x, y);

            if (idx == 0)
                continue;

            if (!found_dot) {
                found_dot = 1;
                line[sx] = BIT_15 | read_memory(this->plt + 0x200 + idx * 2, WIDTH_16);
                mode[sx] = obj.mode;
                priority[sx] = obj.priority;
            }

            if (!found_win && obj.mode == 2) {
                found_win = 1;
                win[sx] = 1;
            }

            if (found_dot && found_win)
                break;
        }
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

        // send irq
        if (bit(dispstat, 5)) {
            bus_send_irq(this->bus, IRQ_VMATCH);
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
    addr &= align_mask[width];
    addr &= PLT_SIZE - 1;
    return read_memory(this->plt+addr, width);
}

static u32 oam_read(void *dev, u32 addr, u8 width)
{
    Ppu *this = dev;
    addr &= align_mask[width];
    addr &= OAM_SIZE - 1;
    return read_memory(this->oam+addr, width);
}

static u32 vram_read(void *dev, u32 addr, u8 width)
{
    Ppu *this = dev;
    addr &= align_mask[width];
    addr &= 0x1FFFF;
    if (addr >= 0x18000)
        addr -= 0x8000;
    return read_memory(this->vram+addr, width);
}

static u32 lcd_read(void *dev, u32 addr, u8 width)
{
    Ppu *this = dev;
    addr &= align_mask[width];
    addr &= 0xFF;
    return read_memory(this->reg+addr, width);
}

static void plt_write(void *dev, u32 addr, u8 width, u32 data)
{
    if (width == WIDTH_8) {
        width = WIDTH_16;
        data |= data << 8;
    }
    addr &= align_mask[width];
    Ppu *this = dev;
    addr &= PLT_SIZE - 1;
    write_memory(this->plt+addr, width, data);
}

static void oam_write(void *dev, u32 addr, u8 width, u32 data)
{
    if (width == WIDTH_8)
        return;
    Ppu *this = dev;
    addr &= align_mask[width];
    addr &= OAM_SIZE - 1;
    write_memory(this->oam+addr, width, data);
}

static void vram_write(void *dev, u32 addr, u8 width, u32 data)
{
    Ppu *this = dev;
    addr &= align_mask[width];
    addr &= 0x1FFFF;

    u8 mode = this->reg[REG_DISPCNT] & 7;
    if (width == WIDTH_8) {
        if (mode < 3 && addr >= 0x10000)
            return;
        else if (mode >= 3 && addr >= 0x14000)
            return;
        width = WIDTH_16;
        data |= data << 8;
    }

    if (addr >= 0x18000)
        addr -= 0x8000;
    write_memory(this->vram+addr, width, data);
}

static void write_lcd_register(Ppu *this, u32 addr, u8 data)
{
    if (addr == 4) {
        this->reg[addr] &= 7;
        this->reg[addr] |= data;
    } else {
        this->reg[addr] = data;
    }

    if (in_range(addr, 0x28, 0x2C+3)) {
        this->reload_bg_ref[0] = 1;
    } else if (in_range(addr, 0x38, 0x3C+3)) {
        this->reload_bg_ref[1] = 1;
    }
}

static void lcd_write(void *dev, u32 addr, u8 width, u32 data)
{
    Ppu *this = dev;
    addr &= align_mask[width];
    addr &= 0xFF;
    switch (width) {
        case WIDTH_32:
            write_lcd_register(this, addr+3, data >> 24);
            write_lcd_register(this, addr+2, data >> 16);
        case WIDTH_16:
            write_lcd_register(this, addr+1, data >> 8);
        case WIDTH_8:
            write_lcd_register(this, addr, data);
        break;
        default:
            assert(0);
        break;
    }
}
