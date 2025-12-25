#include <stdlib.h>
#include <stdio.h>

#include "bus.h"

static void bus_load_bios(Bus *this, const char *bios);
static void bus_load_rom(Bus *this, const char *rom);
static u8 *bus_get_ptr(Bus *this, u32 addr);

Bus *bus_init(const char *rom)
{
    Bus *bus = malloc(sizeof(Bus));

    bus_load_rom(bus, rom);

    bus->map[0].start = BIOS_ADDR;
    bus->map[0].end = BIOS_ADDR + BIOS_SIZE;
    bus->map[0].mem = bus->bios;

    bus->map[1].start = EWRAM_ADDR;
    bus->map[1].end = EWRAM_ADDR + EWRAM_SIZE;
    bus->map[1].mem = bus->ewram;

    bus->map[2].start = IWRAM_ADDR;
    bus->map[2].end = IWRAM_ADDR + IWRAM_SIZE;
    bus->map[2].mem = bus->iwram;

    bus->map[3].start = IO_ADDR;
    bus->map[3].end = IO_ADDR + IO_SIZE;
    bus->map[3].mem = bus->io;

    bus->map[4].start = PLT_ADDR;
    bus->map[4].end = PLT_ADDR + PLT_SIZE;
    bus->map[4].mem = bus->plt;

    bus->map[5].start = VRAM_ADDR;
    bus->map[5].end = VRAM_ADDR + VRAM_SIZE;
    bus->map[5].mem = bus->vram;

    bus->map[6].start = OAM_ADDR;
    bus->map[6].end = OAM_ADDR + OAM_SIZE;
    bus->map[6].mem = bus->oam;

    bus->map[7].start = ROM_ADDR1;
    bus->map[7].end = ROM_ADDR1 + ROM_SIZE;
    bus->map[7].mem = bus->rom;

    bus->map[8].start = ROM_ADDR2;
    bus->map[8].end = ROM_ADDR2 + ROM_SIZE;
    bus->map[8].mem = bus->rom;

    bus->map[9].start = ROM_ADDR3;
    bus->map[9].end = ROM_ADDR3 + ROM_SIZE;
    bus->map[9].mem = bus->rom;

    bus->map[10].start = SRAM_ADDR;
    bus->map[10].end = SRAM_ADDR + SRAM_SIZE;
    bus->map[10].mem = bus->sram;

    return bus;
}

static u8 *bus_get_ptr(Bus *this, u32 addr)
{
    for (uint i = 0; i < len(this->map); ++i) {
        u32 start = this->map[i].start;
        if (in_range(addr, start, this->map[i].end)) {
            return &this->map[i].mem[addr-start];
        }
    }

    return NULL;
}

u8 bus_read(Bus *this, u32 addr)
{
    u8 *ptr = bus_get_ptr(this, addr);
    if (ptr != NULL)
        return *ptr;

    return 0xFF;
}

void bus_write(Bus *this, u32 addr, u8 data)
{
    u8 *ptr = bus_get_ptr(this, addr);
    if (ptr == NULL)
        return;

    *ptr = data;
}

u16 bus_read16(Bus *this, u32 addr)
{
    u8 *ptr = bus_get_ptr(this, addr);
    if (ptr == NULL)
        return 0xFFFF;

    return ptr[0] | (ptr[1] << 8);
}

void bus_write16(Bus *this, u32 addr, u16 data)
{
    u8 *ptr = bus_get_ptr(this, addr);
    if (ptr == NULL)
        return;

    ptr[0] = data & 0xFF;
    ptr[1] = data >> 8;
}

u32 bus_read32(Bus *this, u32 addr)
{
    u8 *ptr = bus_get_ptr(this, addr);
    if (ptr == NULL)
        return 0xFFFFFFFF;

    return ptr[0] | (ptr[1] << 8) | (ptr[2] << 16) | (ptr[3] << 24);
}

void bus_write32(Bus *this, u32 addr, u32 data)
{
    u8 *ptr = bus_get_ptr(this, addr);
    if (ptr == NULL)
        return;

    ptr[0] = data;
    ptr[1] = data >> 8;
    ptr[2] = data >> 16;
    ptr[3] = data >> 24;
}

static void bus_load_bios(Bus *this, const char *bios)
{
    FILE *f = fopen(bios, "r");
    if (f == NULL) {
        perror(bios);
        exit(1);
    }

    fread(this->bios, sizeof(u8), BIOS_SIZE, f);

    fclose(f);
}

static void bus_load_rom(Bus *this, const char *rom)
{
    FILE *f = fopen(rom, "r");
    if (f == NULL) {
        perror(rom);
        exit(1);
    }

    fread(this->rom, sizeof(u8), ROM_SIZE, f);

    fclose(f);
}
