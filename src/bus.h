#pragma once

#include "common.h"

#define BIOS_SIZE (16 * 1024)
#define EWRAM_SIZE (256 * 1024)
#define IWRAM_SIZE (32 * 1024)
#define ROM_SIZE (32 * 1024 * 1024)
#define PLT_SIZE (1024)
#define VRAM_SIZE (96 * 1024)
#define OAM_SIZE (1024)
#define SRAM_SIZE (64 * 1024)
#define IO_SIZE (0x3FE)

#define BIOS_ADDR  0x00000000
#define EWRAM_ADDR 0x02000000
#define IWRAM_ADDR 0x03000000
#define IO_ADDR    0x04000000
#define PLT_ADDR   0x05000000
#define VRAM_ADDR  0x06000000
#define OAM_ADDR   0x07000000
#define ROM_ADDR1  0x08000000
#define ROM_ADDR2  0x0A000000
#define ROM_ADDR3  0x0C000000
#define SRAM_ADDR  0x0E000000

enum {WIDTH_8, WIDTH_16, WIDTH_32};

extern u32 align_mask[3];

typedef struct BusDev {
    void *this;
    u32 (*read)(void *dev, u32 addr, u8 width);
    void (*write)(void *dev, u32 addr, u8 width, u32 data);
} BusDev;

typedef struct Bus {
    u8 bios[BIOS_SIZE];
    u8 ewram[EWRAM_SIZE];
    u8 iwram[IWRAM_SIZE];
    u8 rom[ROM_SIZE];
    u8 sram[SRAM_SIZE];
    u8 io[IO_SIZE];

    u8 sysctl[0x10];

    BusDev dev[0x100];
    BusDev io_dev[0x100];
} Bus;

Bus *bus_init(const char *rom, const char *bios);

u8 bus_read(Bus *this, u32 addr);
void bus_write(Bus *this, u32 addr, u8 data);

u16 bus_read16(Bus *this, u32 addr);
void bus_write16(Bus *this, u32 addr, u16 data);

u32 bus_read32(Bus *this, u32 addr);
void bus_write32(Bus *this, u32 addr, u32 data);

void bus_attach_plt(Bus *this, const BusDev *dev);
void bus_attach_vram(Bus *this, const BusDev *dev);
void bus_attach_oam(Bus *this, const BusDev *dev);
void bus_attach_lcd(Bus *this, const BusDev *dev);
void bus_attach_keypad(Bus *this, const BusDev *dev);

void bus_send_irq(Bus *this, u16 irq);

u32 read_memory(u8 *mem, u8 width);
void write_memory(u8 *mem, u8 width, u32 data);
void write_register(u8 *mem, u8 width, u32 data);
