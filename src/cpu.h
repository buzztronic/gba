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

#ifdef RUN_CPU_TESTS
typedef struct Transaction {
    u32 kind;
    u32 size;
    u32 addr;
    u32 data;
} Transaction;
#endif

typedef struct Cpu {
    Bus *bus;

#ifdef RUN_CPU_TESTS
    Transaction *tlist;
    u32 tlist_len;
    u32 tidx;
    u32 topcode, topcode_reverse;
    u32 tbase;

    u32 test_no;

    struct Cpu *initial;
#endif

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

// lookup table to check for conditions
// low nibble = condition code from the instruction
// high nibble = condition flags from cpsr
extern u8 cpu_cond_lut[1 << 8];

void cpu_init_global(void);
Cpu *cpu_init(Bus *bus);
uint cpu_step(Cpu *this);

void cpu_bank_registers(Cpu *this);
uint cpu_software_interrupt(Cpu *this, u32 opcode);
void cpu_handle_interrupts(Cpu *this);

u32 alu_sub(u32 op1, u32 op2, u32 *cpsr);
u32 alu_add(u32 op1, u32 op2, u32 *cpsr);
u32 alu_and(u32 op1, u32 op2, u32 *cpsr);
u32 alu_eor(u32 op1, u32 op2, u32 *cpsr);
u32 alu_rsb(u32 op1, u32 op2, u32 *cpsr);
u32 alu_adc(u32 op1, u32 op2, u32 *cpsr);
u32 alu_sbc(u32 op1, u32 op2, u32 *cpsr);
u32 alu_rsc(u32 op1, u32 op2, u32 *cpsr);
u32 alu_orr(u32 op1, u32 op2, u32 *cpsr);
u32 alu_mov(u32 op1, u32 op2, u32 *cpsr);
u32 alu_bic(u32 op1, u32 op2, u32 *cpsr);
u32 alu_mvn(u32 op1, u32 op2, u32 *cpsr);

void cpu_update_zn(u32 result, u32 *cpsr);

#ifdef RUN_CPU_TESTS
void cpu_test_all(const char *path);
u32 test_read(Cpu *this, u32 kind, u32 size, u32 addr);
void test_write(Cpu *this, u32 kind, u32 size, u32 addr, u32 data);

#define bus_read(bus, addr) test_read(this, 1, 1, addr)
#define bus_read16(bus, addr) test_read(this, 1, 2, addr)
#define bus_read32(bus, addr) test_read(this, 1, 4, addr)

#define bus_write(bus, addr, data) test_write(this, 2, 1, addr, data)
#define bus_write16(bus, addr, data) test_write(this, 2, 2, addr, data)
#define bus_write32(bus, addr, data) test_write(this, 2, 4, addr, data)

#define instruction_read16(addr) test_read(this, 0, 2, addr)
#define instruction_read32(addr) test_read(this, 0, 4, addr)

#endif
