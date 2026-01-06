#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bus.h"

static u32 bios_read(void *, u32 addr, u8 width);
static u32 rom_read(void *, u32 addr, u8 width);
static u32 ewram_read(void *, u32 addr, u8 width);
static u32 iwram_read(void *, u32 addr, u8 width);
static u32 sram_read(void *, u32 addr, u8 width);
static u32 invalid_read(void *, u32 addr, u8 width);
static u32 io_read(void *, u32 addr, u8 width);
static u32 sysctl_read(void *, u32 addr, u8 width);

static void ewram_write(void *, u32 addr, u8 width, u32 data);
static void iwram_write(void *, u32 addr, u8 width, u32 data);
static void sram_write(void *, u32 addr, u8 width, u32 data);
static void invalid_write(void *, u32 addr, u8 width, u32 data);
static void io_write(void *, u32 addr, u8 width, u32 data);
static void sysctl_write(void *, u32 addr, u8 width, u32 data);

static void write_sysctl_register(Bus *this, u32 addr, u8 data);

static void bus_load_bios(Bus *this, const char *bios);
static void bus_load_rom(Bus *this, const char *rom);

Bus *bus_init(const char *rom, const char *bios)
{
    Bus *bus = malloc(sizeof(Bus));

    bus_load_rom(bus, rom);
    bus_load_bios(bus, bios);

    bus->dev[0] = (BusDev){bus, bios_read, invalid_write};
    bus->dev[1] = (BusDev){bus, bios_read, invalid_write};
    bus->dev[2] = (BusDev){bus, ewram_read, ewram_write};
    bus->dev[3] = (BusDev){bus, iwram_read, iwram_write};
    bus->dev[4] = (BusDev){bus, io_read, io_write};

    // initially invalid. ppu will attach it self later
    bus->dev[5] = (BusDev){bus, invalid_read, invalid_write};
    bus->dev[6] = (BusDev){bus, invalid_read, invalid_write};
    bus->dev[7] = (BusDev){bus, invalid_read, invalid_write};

    // ROM
    for (uint i = 8; i <= 0xD; i++) {
        bus->dev[i] = (BusDev){bus, rom_read, invalid_write};
    }

    bus->dev[0xE] = (BusDev){bus, sram_read, sram_write};
    bus->dev[0xF] = (BusDev){bus, sram_read, sram_write};


    // invalid
    for (uint i = 0x10; i <= 0xFF; i++) {
        bus->dev[i] = (BusDev){bus, invalid_read, invalid_write};
    }

    // I/O
    for (uint i = 0; i <= 0xFF; i++) {
        bus->io_dev[i] = (BusDev){bus, invalid_read, invalid_write};
    }

    bus->io_dev[0x20] = (BusDev){bus, sysctl_read, sysctl_write};

    return bus;
}

u8 bus_read(Bus *this, u32 addr)
{
    BusDev *dev = &this->dev[addr >> 24];
    return dev->read(dev->this, addr, WIDTH_8);
}

void bus_write(Bus *this, u32 addr, u8 data)
{
    BusDev *dev = &this->dev[addr >> 24];
    dev->write(dev->this, addr, WIDTH_8, data);
}

u16 bus_read16(Bus *this, u32 addr)
{
    BusDev *dev = &this->dev[addr >> 24];
    return dev->read(dev->this, addr, WIDTH_16);
}

void bus_write16(Bus *this, u32 addr, u16 data)
{
    BusDev *dev = &this->dev[addr >> 24];
    dev->write(dev->this, addr, WIDTH_16, data);
}

u32 bus_read32(Bus *this, u32 addr)
{
    BusDev *dev = &this->dev[addr >> 24];
    return dev->read(dev->this, addr, WIDTH_32);
}

void bus_write32(Bus *this, u32 addr, u32 data)
{
    BusDev *dev = &this->dev[addr >> 24];
    dev->write(dev->this, addr, WIDTH_32, data);
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

static u32 bios_read(void *dev, u32 addr, u8 width)
{
    Bus *this = dev;
    addr &= align_mask[width];
    return read_memory(this->bios+addr, width);
}

static u32 rom_read(void *dev, u32 addr, u8 width)
{
    Bus *this = dev;
    addr &= align_mask[width];
    addr &= ROM_SIZE - 1;
    return read_memory(this->rom+addr, width);
}

static u32 ewram_read(void *dev, u32 addr, u8 width)
{
    Bus *this = dev;
    addr &= align_mask[width];
    addr &= 0x40000 - 1;
    return read_memory(this->ewram+addr, width);
}

static u32 iwram_read(void *dev, u32 addr, u8 width)
{
    Bus *this = dev;
    addr &= align_mask[width];
    addr &= 0x8000 - 1;
    return read_memory(this->iwram+addr, width);
}

static u32 sram_read(void *dev, u32 addr, u8 width)
{
    Bus *this = dev;
    width = WIDTH_8;
    addr &= 0xFFFF;
    // TODO
    return read_memory(this->sram+addr, width);
}

static u32 invalid_read(void *dev, u32 addr, u8 width)
{
    return 0xFFFFFFFF;
}

static u32 io_read(void *dev, u32 addr, u8 width)
{
    Bus *this = dev;
    addr &= align_mask[width];
    addr -= 0x04000000;

    BusDev *io_dev = &this->io_dev[(addr >> 4) & 0xFF];
    return io_dev->read(io_dev->this, addr, width);
}

static u32 sysctl_read(void *dev, u32 addr, u8 width)
{
    Bus *this = dev;
    addr &= align_mask[width];
    addr &= 0xF;
    return read_memory(this->sysctl+addr, width);
}

static void ewram_write(void *dev, u32 addr, u8 width, u32 data)
{
    Bus *this = dev;
    addr &= align_mask[width];
    addr &= 0x40000 - 1;
    write_memory(this->ewram+addr, width, data);
}

static void iwram_write(void *dev, u32 addr, u8 width, u32 data)
{
    Bus *this = dev;
    addr &= align_mask[width];
    addr &= 0x8000 - 1;
    write_memory(this->iwram+addr, width, data);
}

static void sram_write(void *dev, u32 addr, u8 width, u32 data)
{
    Bus *this = dev;
    addr -= SRAM_ADDR;
    write_memory(this->sram+addr, width, data);
}

static void invalid_write(void *dev, u32 addr, u8 width, u32 data)
{
}

static void io_write(void *dev, u32 addr, u8 width, u32 data)
{
    Bus *this = dev;
    addr &= align_mask[width];
    addr -= 0x04000000;

    BusDev *io_dev = &this->io_dev[(addr >> 4) & 0xFF];
    io_dev->write(io_dev->this, addr, width, data);
}

static void write_sysctl_register(Bus *this, u32 addr, u8 data)
{
    if (addr == 2 || addr == 3) {
        this->sysctl[addr] &= ~data;
    } else {
        this->sysctl[addr] = data;
    }
}

static void sysctl_write(void *dev, u32 addr, u8 width, u32 data)
{
    Bus *this = dev;
    addr &= align_mask[width];
    addr &= 0xF;
    switch (width) {
        case WIDTH_32:
            write_sysctl_register(this, addr+3, data >> 24);
            write_sysctl_register(this, addr+2, data >> 16);
        case WIDTH_16:
            write_sysctl_register(this, addr+1, data >> 8);
        case WIDTH_8:
            write_sysctl_register(this, addr, data);
        break;
        default:
            assert(0);
        break;
    }
}

void bus_attach_plt(Bus *this, const BusDev *dev)
{
    memcpy(&this->dev[5], dev, sizeof(BusDev));
}

void bus_attach_vram(Bus *this, const BusDev *dev)
{
    memcpy(&this->dev[6], dev, sizeof(BusDev));
}

void bus_attach_oam(Bus *this, const BusDev *dev)
{
    memcpy(&this->dev[7], dev, sizeof(BusDev));
}

void bus_attach_lcd(Bus *this, const BusDev *dev)
{
    for (uint i = 0; i <= 5; i++)
        memcpy(&this->io_dev[i], dev, sizeof(BusDev));
}

void bus_attach_keypad(Bus *this, const BusDev *dev)
{
    memcpy(&this->io_dev[0x13], dev, sizeof(BusDev));
}

void bus_send_irq(Bus *this, u16 irq)
{
    this->sysctl[2] |= irq;
    this->sysctl[3] |= irq >> 8;
}

u32 read_memory(u8 *mem, u8 width)
{
    switch (width) {
        case WIDTH_8:
            return mem[0];
        break;
        case WIDTH_16:
            return mem[0] | (mem[1] << 8);
        break;
        case WIDTH_32:
            return mem[0] | (mem[1] << 8) | (mem[2] << 16) | (mem[3] << 24);
        break;
        default:
            assert(0);
        break;
    }
}

void write_memory(u8 *mem, u8 width, u32 data)
{
    switch (width) {
        case WIDTH_8:
            mem[0] = data;
        break;
        case WIDTH_16:
            mem[0] = data;
            mem[1] = data >> 8;
        break;
        case WIDTH_32:
            mem[0] = data;
            mem[1] = data >> 8;
            mem[2] = data >> 16;
            mem[3] = data >> 24;
        break;
        default:
            assert(0);
        break;
    }
}
