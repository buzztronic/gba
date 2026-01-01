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

typedef struct MemMap {
    u32 start;
    u32 end;
    u8 *mem;
} MemMap;

typedef struct Bus {
    u8 bios[BIOS_SIZE];
    u8 ewram[EWRAM_SIZE];
    u8 iwram[IWRAM_SIZE];
    u8 rom[ROM_SIZE];
    u8 plt[PLT_SIZE];
    u8 vram[VRAM_SIZE];
    u8 oam[OAM_SIZE];
    u8 sram[SRAM_SIZE];
    u8 io[IO_SIZE];

    MemMap map[0x10];

    // NOTE: for the moment everything is stored this strucut
    // including I/O registers but that will probably change later
} Bus;

Bus *bus_init(const char *rom, const char *bios);

u8 bus_read(Bus *this, u32 addr);
void bus_write(Bus *this, u32 addr, u8 data);

u16 bus_read16(Bus *this, u32 addr);
void bus_write16(Bus *this, u32 addr, u16 data);

u32 bus_read32(Bus *this, u32 addr);
void bus_write32(Bus *this, u32 addr, u32 data);

u8 *bus_getvram(Bus *this);
