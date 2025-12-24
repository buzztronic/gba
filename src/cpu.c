#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include "cpu.h"
#include "thumb.h"
#include "common.h"

// declarations
static int cond_pass(u32 opcode, u32 cpsr);
static uint cpu_step_arm(Cpu *this);
static u32 cpu_fetch_arm(Cpu *this);
static void cpu_reset_pipeline(Cpu *this);

static uint cpu_execute_not_implemented(Cpu *this, u32 opcode);
static uint cpu_execute_branch(Cpu *this, u32 opcode);
static uint cpu_execute_alu(Cpu *this, u32 opcode);
static void cpu_build_decode_table(Cpu *this);

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

    cpu_build_decode_table(cpu);

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
        u32 index = (bits(opcode, 20, 8) << 4) | bits(opcode, 4, 4);
        cycles = this->decode[index](this, opcode);
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

static uint cpu_execute_alu(Cpu *this, u32 opcode)
{
    puts("ALU");
    return 0;
}

static void cpu_build_decode_table(Cpu *this)
{
    for (uint idx = 0; idx < (1 << 12); ++idx) {
        // unpack idx to make it easier to follow documentation
        // bits[0:3] become bits[4:7]
        // bits[4:12] become bits[20:27]
        u32 opcode = (bits(idx, 4, 8) << 20) | (bits(idx, 0, 4) << 4);
        if (bits(opcode, 25, 3) == 5) {
            this->decode[idx] = cpu_execute_branch;
        } else if (bits(opcode, 26, 2) == 0) {
            u32 alu_opcode = bits(opcode, 21, 4);
            // if bit[25] == 0 bit[4] == 1 bit[7] == 1 then it is not an ALU
            // instruction.
            // if alu opcode is one of CMP/CMN/TST/TEQ then bit[20] must be 1
            // otherwise it is not an ALU instruction.
            if (bit(opcode, 25) == 0 && bit(opcode, 4) && bit(opcode, 7)) {
                // not ALU but something else
                this->decode[idx] = cpu_execute_not_implemented;
            } else if (bit(opcode, 20) == 0 && alu_opcode >= 8 && alu_opcode <= 11) {
                // not ALU but something else
                this->decode[idx] = cpu_execute_not_implemented;
            } else {
                this->decode[idx] = cpu_execute_alu;
            }
        } else {
            this->decode[idx] = cpu_execute_not_implemented;
        }
    }
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
