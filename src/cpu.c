#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include "cpu.h"
#include "thumb.h"
#include "common.h"

enum InstrType {IT_NOT_IMPLEMENTED, IT_BRANCH};

// declarations
int cond_pass(u32 opcode, u32 cpsr);
uint cpu_step_arm(Cpu *this);
u32 cpu_fetch_arm(Cpu *this);
void cpu_reset_pipeline(Cpu *this);

uint cpu_execute_not_implemented(Cpu *this, u32 opcode);
uint cpu_execute_branch(Cpu *this, u32 opcode);

enum InstrType cpu_decode(u32 opcode);
uint (*cpu_execute[]) (Cpu *, u32) = {
    [IT_NOT_IMPLEMENTED] = cpu_execute_not_implemented,
    [IT_BRANCH] = cpu_execute_branch,
};

// functions
Cpu *cpu_init(Bus *bus)
{
    Cpu *cpu = malloc(sizeof(Cpu));

    for (int i = 0; i < 16; ++i) {
		cpu->reg[i] = &(cpu->reg_usr[i]);
	}

    // initial state
    cpu->cpsr = 0x5F;
    *cpu->reg[15] = 0x8000000;
    *cpu->reg[13] = 0x3007F00;

    cpu->bus = bus;
    cpu->pc_changed = 1;

    return cpu;
}

uint cpu_step(Cpu *this)
{
    if (is_clear(this->cpsr, PSR_MASK_T)) {
        return cpu_step_arm(this);
    } else {
        return cpu_step_thumb(this);
    }
    return 0;
}

uint cpu_step_arm(Cpu *this)
{
    // check for interrupts
    // break into debugger

    u32 opcode = cpu_fetch_arm(this);
    uint cycles = 0;

    printf("%08X %08X ", reg(15) - 8, opcode);

    // decode and execute
    if (cond_pass(opcode, this->cpsr)) {
        cycles = cpu_execute[cpu_decode(opcode)](this, opcode);
    } else {
        cycles = 1;
    }

    return cycles;
}

u32 cpu_fetch_arm(Cpu *this)
{
    u32 opcode;

    if (this->pc_changed) {
        cpu_reset_pipeline(this);
        this->pc_changed = 0;
    } else {
        reg(15) += 4;
    }

    opcode = this->execute_opcode;
    this->execute_opcode = this->decode_opcode;
    this->decode_opcode = bus_read32(this->bus, reg(15));

    return opcode;
}

void cpu_reset_pipeline(Cpu *this)
{
    this->execute_opcode = bus_read32(this->bus, reg(15));
    this->decode_opcode = bus_read32(this->bus, reg(15) + 4);
    reg(15) += 8;
}


uint cpu_execute_not_implemented(Cpu *this, u32 opcode)
{
    puts("Not Implemented");
    return 0;
}

uint cpu_execute_branch(Cpu *this, u32 opcode)
{
    if (bit(opcode, 24)) {
        // branch with link
        printf("BL ");

        reg(14) = reg(15) + 4;
    } else {
        printf("B ");
    }

    u32 offset = bits(opcode, 0, 24);
    if (bit(offset, 23)) {
        // sign extention
        offset |= 0xFF000000;
    }
    offset <<= 2;

    reg(15) += (i32)offset;
    this->pc_changed = 1;

    printf("#%X (%d)\n", 8 + offset, 8 + offset);

    return 1;
}

enum InstrType cpu_decode(u32 opcode)
{
    if (bits(opcode, 25, 3) == 5) {
        return IT_BRANCH;
    }

    return IT_NOT_IMPLEMENTED;
}

int cond_pass(u32 opcode, u32 cpsr)
{
    switch (bits(opcode, 28, 4)) {
        case 0x0:
            // EQ
            return is_set(cpsr, PSR_MASK_Z);
        break;
        case 0x1:
            // NE
            return is_clear(cpsr, PSR_MASK_Z);
        break;
        case 0x2:
            // CS
            return is_set(cpsr, PSR_MASK_C);
        break;
        case 0x3:
            // CC
            return is_clear(cpsr, PSR_MASK_C);
        break;
        case 0x4:
            // MI
            return is_set(cpsr, PSR_MASK_N);
        break;
        case 0x5:
            // PL
            return is_clear(cpsr, PSR_MASK_N);
        break;
        case 0x6:
            // VS
            return is_set(cpsr, PSR_MASK_V);
        break;
        case 0x7:
            // VC
            return is_clear(cpsr, PSR_MASK_V);
        break;
        case 0x8:
            // HI
            return is_set(cpsr, PSR_MASK_C) && is_clear(cpsr, PSR_MASK_Z);
        break;
        case 0x9:
            // LS
            return is_clear(cpsr, PSR_MASK_C) && is_set(cpsr, PSR_MASK_Z);
        break;
        case 0xA:
            // GE
            return bit(cpsr, PSR_MASK_N) == bit(cpsr, PSR_MASK_V);
        break;
        case 0xB:
            // LT
            return bit(cpsr, PSR_MASK_N) != bit(cpsr, PSR_MASK_V);
        break;
        case 0xC:
            // GT
            return is_clear(cpsr, PSR_MASK_Z) && bit(cpsr, PSR_MASK_N) == bit(cpsr, PSR_MASK_V);
        break;
        case 0xD:
            // LE
            return is_clear(cpsr, PSR_MASK_Z) && bit(cpsr, PSR_MASK_N) != bit(cpsr, PSR_MASK_V);
        break;
        case 0xE:
            // AL
            return 1;
        break;
    }
    assert(0);
    return 1;
}
