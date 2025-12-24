#pragma once

#include "bus.h"

#define reg(n) (*this->reg[n])

#define PSR_MASK_MODE 0x1F
#define PSR_MASK_T (1 << 5)
#define PSR_MASK_F (1 << 6)
#define PSR_MASK_I (1 << 7)
#define PSR_MASK_V (1 << 28)
#define PSR_MASK_C (1 << 29)
#define PSR_MASK_Z (1 << 30)
#define PSR_MASK_N (1 << 31)

enum Spsr_index {SPSR_FIQ, SPSR_SVC, SPSR_ABT, SPSR_IRQ, SPSR_UND};

typedef struct Cpu {
    Bus *bus;

    u32 *reg[16];

    u32 reg_usr[16];
	u32 reg_fiq[7]; // r8 through r14

	// r13 and r14 for each mode
	u32 reg_svc[2];
	u32 reg_irq[2];
	u32 reg_und[2];
	u32 reg_abt[2]; // NOTE: I think we don't need this

    u32 spsr[5]; // indexed by Spsr_index
	u32 cpsr;

    // pipeline state
	u32 execute_opcode;
	u32 decode_opcode;

    // triggers cpu_reset_pipeline when 1
    u32 pc_changed;
} Cpu;

Cpu *cpu_init(Bus *bus);
uint cpu_step(Cpu *this);
