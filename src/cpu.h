#pragma once

#include "bus.h"

#define reg(n) (*this->reg[n])

#define PSR_MASK_MODE 0xF
#define PSR_BIT_T 5
#define PSR_BIT_F 6
#define PSR_BIT_I 7
#define PSR_BIT_V 28
#define PSR_BIT_C 29
#define PSR_BIT_Z 30
#define PSR_BIT_N 31

// SPSR_USR is just for padding
enum CpuMode {
    CPU_MODE_USR,
    CPU_MODE_FIQ,
    CPU_MODE_IRQ,
    CPU_MODE_SVC,
    CPU_MODE_ABT = 7,
    CPU_MODE_UND = 11,
    CPU_MODE_SYS = 15,
};

typedef struct Cpu {
    Bus *bus;

    // decoding look up table
    uint (*decode[1 << 12]) (struct Cpu *, u32);
    uint (*decode_arm[1 << 8]) (struct Cpu *, u16);

    // lookup table to check for conditions
    // low nibble = condition code from the instruction
    // high nibble = condition flags from cpsr
    u8 cond_pass[1 << 8];

    u32 *reg[16];

    u32 reg_usr[16];
	u32 reg_fiq[7]; // r8 through r14

	// r13 and r14 for each mode
	u32 reg_svc[2];
	u32 reg_irq[2];
	u32 reg_und[2];
	u32 reg_abt[2]; // NOTE: I think we don't need this

    u32 spsr[16]; // indexed by cpsr [0:3]
	u32 cpsr;

    // pipeline state
	u32 execute_opcode;
	u32 decode_opcode;

    // triggers cpu_reset_pipeline when 1
    u32 pc_changed;
} Cpu;

Cpu *cpu_init(Bus *bus);
uint cpu_step(Cpu *this);
