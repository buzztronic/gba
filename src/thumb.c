#include <stdio.h>
#include <assert.h>
#include <stdlib.h>

#include "thumb.h"

// declarations
static u16 cpu_fetch_thumb(Cpu *this);
static void cpu_reset_pipeline_thumb(Cpu *this);
static void cpu_build_decode_table_thumb(Cpu *this);

static uint thumb_execute_not_implemented(Cpu *this, u16 opcode);
static uint thumb_move_immediate(Cpu *this, u16 opcode);
static uint thumb_cond_branch(Cpu *this, u16 opcode);
static uint thumb_branch(Cpu *this, u16 opcode);
static uint thumb_move_shifted_register(Cpu *this, u16 opcode);
static uint thumb_add_sub(Cpu *this, u16 opcode);
static uint thumb_alu(Cpu *this, u16 opcode);
static uint thumb_branch_exchange(Cpu *this, u16 opcode);
static uint thumb_hi_operation(Cpu *this, u16 opcode);
static uint thumb_load_address(Cpu *this, u16 opcode);
static uint thumb_add_stack_pointer(Cpu *this, u16 opcode);

static const char *bin8_str(u8 data);
static const char *bin16_str(u16 data);

// defined in cpu.c
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

static u32 alu_lsl(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_lsr(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_asr(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_ror(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_neg(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_mul(u32 op1, u32 op2, u32 *cpsr);

void cpu_update_zn(u32 result, u32 *cpsr);

static u32 (*alu_thumb[])(u32, u32, u32 *) = {
    alu_and,
    alu_eor,
    alu_lsl,
    alu_lsr,
    alu_asr,
    alu_adc,
    alu_sbc,
    alu_ror,
    alu_and, // tst
    alu_neg,
    alu_sub, // cmp
    alu_add, // cmn
    alu_orr,
    alu_mul,
    alu_bic,
    alu_mvn,
};

static const char *const alu_mnemonic_thumb[] = {
    "AND",
    "EOR",
    "LSL",
    "LSR",
    "ASR",
    "ADC",
    "SBC",
    "ROR",
    "TST",
    "NEG",
    "CMP",
    "CMN",
    "ORR",
    "MUL",
    "BIC",
    "MVN"
};

void cpu_init_thumb(Cpu *this)
{
    cpu_build_decode_table_thumb(this);
}

uint cpu_step_thumb(Cpu *this)
{
    u16 opcode = cpu_fetch_thumb(this);
    uint cycles = 0;

    printf("%08X %04X %s ", (reg(15) & ~1) - 4, opcode, bin8_str(opcode >> 8));

    cycles = this->decode_thumb[opcode >> 8](this, opcode);

    return cycles;
}

static u16 cpu_fetch_thumb(Cpu *this)
{
    u16 opcode;

    reg(15) &= ~1;
    if (this->pc_changed) {
        cpu_reset_pipeline_thumb(this);
        this->pc_changed = 0;
    } else {
        reg(15) += 2;
    }

    opcode = this->execute_opcode;
    this->execute_opcode = this->decode_opcode;
    this->decode_opcode = bus_read16(this->bus, reg(15) & ~1);

    return opcode;
}

static void cpu_reset_pipeline_thumb(Cpu *this)
{
    this->execute_opcode = bus_read16(this->bus, reg(15) & ~1);
    this->decode_opcode = bus_read16(this->bus, (reg(15) & ~1) + 2);
    reg(15) += 4;
}

static void cpu_build_decode_table_thumb(Cpu *this)
{
    for (uint idx = 0; idx <= 0xFF; idx++) {
        u16 opcode = idx << 8;
        this->decode_thumb[idx] = thumb_execute_not_implemented;
        if (bits(opcode, 13, 3) == 0) {
            if (bits(opcode, 11, 2) != 3) {
                // move shifted register
                this->decode_thumb[idx] = thumb_move_shifted_register;
            } else {
                // Add/Sub
                this->decode_thumb[idx] = thumb_add_sub;
            }
            continue;
        }

        if (bits(opcode, 13, 3) == 1) {
            // mov, cmp, add, sub immediate
            this->decode_thumb[idx] = thumb_move_immediate;
            continue;
        }

        if (bits(opcode, 10, 6) == 0x10) {
            // ALU operation
            printf("ALU: %s\n", bin8_str(opcode >> 8));
            this->decode_thumb[idx] = thumb_alu;
            continue;
        }

        if (bits(opcode, 10, 6) == 0x11) {
            if (bits(opcode, 8, 2) == 3) {
                // bx
                this->decode_thumb[idx] = thumb_branch_exchange;
            } else {
                // Hi register operation
                this->decode_thumb[idx] = thumb_hi_operation;
            }
            continue;
        }


        if (bits(opcode, 11, 5) == 9) {
            // PC relative load
            continue;
        }

        if (bits(opcode, 12, 4) == 5) {
            if (bit(opcode, 9) == 0) {
                // Load/store with register offset
            } else {
                // Load/store sign-extended byte/halfword
            }
            continue;
        }

        if (bits(opcode, 13, 3) == 3) {
            // Load/store with immediate offset
            continue;
        }

        if (bits(opcode, 12, 4) == 8) {
            // Load/store halfword
            continue;
        }

        if (bits(opcode, 12, 4) == 9) {
            // SP-relative load/store
            continue;
        }

        if (bits(opcode, 12, 4) == 10) {
            // Load address
            this->decode_thumb[idx] = thumb_load_address;
            continue;
        }

        if (bits(opcode, 12, 4) == 11) {
            if (bit(opcode, 10) == 0) {
                // Add offset to stack pointer
                this->decode_thumb[idx] = thumb_add_stack_pointer;
            } else {
                // Push/pop registers
            }
            continue;
        }

        if (bits(opcode, 12, 4) == 12) {
            // Multiple load/store
            continue;
        }

        if (bits(opcode, 12, 4) == 13) {
            if (bits(opcode, 8, 4) != 0xF) {
                // Conditional branch
                this->decode_thumb[idx] = thumb_cond_branch;
            } else {
                // Software Interrupt
            }
            continue;
        }

        if (bits(opcode, 12, 4) == 14) {
            // Unconditional branch
            this->decode_thumb[idx] = thumb_branch;
            continue;
        }

        if (bits(opcode, 12, 4) == 0xF) {
            // Long branch with link
            continue;
        }

        assert(0);
    }
}

static uint thumb_execute_not_implemented(Cpu *this, u16 opcode)
{
    puts("Not Implemented");
    printf("R7: %d\n", reg(7));
    return 0;
}

static uint thumb_move_immediate(Cpu *this, u16 opcode)
{
    u32 imm = opcode & 0xFF;
    u32 rd = bits(opcode, 8, 3);
    u32 op = bits(opcode, 11, 2);

    u32 result = imm;
    switch (op) {
        case 0:
            // MOV
            puts("MOV");
        break;
        case 1:
            // CMP
            puts("CMP");
            result = alu_sub(reg(rd), imm, &this->cpsr);
        break;
        case 2:
            // ADD
            puts("ADD");
            result = alu_add(reg(rd), imm, &this->cpsr);
        break;
        case 3:
            // SUB
            puts("SUB");
            result = alu_sub(reg(rd), imm, &this->cpsr);
        break;
    }

    cpu_update_zn(result, &this->cpsr);

    if (op != 1) {
        reg(rd) = result;
    }

    return 1;
}

static uint thumb_cond_branch(Cpu *this, u16 opcode)
{
    puts("B");
    u32 cond_idx = bits(opcode, 8, 4);
    cond_idx |= bits(this->cpsr, 28, 4) << 4;
    if (this->cond_pass[cond_idx]) {
        u32 offset = opcode & 0xFF;

        reg(15) += ((i8)offset) * 2;

        this->pc_changed = 1;
    }
    return 1;
}

static uint thumb_branch(Cpu *this, u16 opcode)
{
    puts("B");
    u32 offset = opcode & 0xFF;

    reg(15) += ((i8)offset) * 2;
    this->pc_changed = 1;

    return 1;
}

static uint thumb_move_shifted_register(Cpu *this, u16 opcode)
{
    u32 rd = opcode & 7;
    u32 rs = bits(opcode, 3, 3);
    u32 imm = bits(opcode, 6, 5);
    u32 op = opcode >> 11;

    u32 carry = this->cpsr & PSR_BIT_C;
    if (op == 0) {
        // LSL
        puts("LSL");
        if (imm == 0) {
            reg(rd) = reg(rs);
        } else {
            carry = bit(reg(rs), 32 - imm);
            reg(rd) = reg(rs) << imm;
        }
    } else if (op == 1) {
        // LSR
        puts("LSR");
        if (imm == 0) {
            carry = bit(reg(rs), 31);
            reg(rd) = 0;
        } else {
            carry = bit(reg(rs), imm - 1);
            reg(rd) = reg(rs) >> imm;
        }
    } else if (op == 2) {
        // ASR
        puts("ASR");
        if (imm == 0) {
            carry = bit(reg(rs), 31);
            reg(rd) = carry ? 0xFFFFFFFF : 0;
        } else {
            carry = bit(reg(rs), imm - 1);
            reg(rd) = asr32(reg(rs), imm);
        }
    }

    if (carry)
        set_bit(this->cpsr, PSR_BIT_C);
    else
        clear_bit(this->cpsr, PSR_BIT_C);

    if (bit(reg(rd), 31))
        set_bit(this->cpsr, PSR_BIT_N);
    else
        clear_bit(this->cpsr, PSR_BIT_N);

    cpu_update_zn(reg(rd), &this->cpsr);

    return 1;
}

static uint thumb_add_sub(Cpu *this, u16 opcode)
{
    u32 rd = opcode & 7;
    u32 rs = bits(opcode, 3, 3);
    u32 op = bits(opcode, 6, 3);

    if (!(opcode & BIT_10)) {
        // register
        op = reg(op);
    }

    if (opcode & BIT_9) {
        // sub
        puts("SUB");
        reg(rd) = alu_sub(reg(rs), op, &this->cpsr);
    } else {
        // add
        puts("ADD");
        reg(rd) = alu_add(reg(rs), op, &this->cpsr);
    }

    cpu_update_zn(reg(rd), &this->cpsr);

    return 1;
}

static uint thumb_alu(Cpu *this, u16 opcode)
{
    u32 rd = opcode & 7;
    u32 rs = bits(opcode, 3, 3);
    u32 op = bits(opcode, 6, 4);

    puts(alu_mnemonic_thumb[op]);

    u32 result = alu_thumb[op](reg(rd), reg(rs), &this->cpsr);

    cpu_update_zn(result, &this->cpsr);

    if (op != 8 && op != 10 && op != 11) {
        reg(rd) = result;
    }

    return 1;
}

static uint thumb_branch_exchange(Cpu *this, u16 opcode)
{
    puts("BX");

    u32 rs = bits(opcode, 3, 4);

    if (rs == 15) {
        rs = reg(15) & ~1;
    } else {
        rs = reg(rs);
    }

    reg(15) = rs;
    this->pc_changed = 1;

    if (rs & 1) {
        // Switch to Thumb
        set_bit(this->cpsr, PSR_BIT_T);
        puts("SWITCH TO THUMB from thumb :)");
    } else {
        // Switch to ARM
        clear_bit(this->cpsr, PSR_BIT_T);
        puts("SWITCH TO ARM");
    }

    return 1;
}

static uint thumb_hi_operation(Cpu *this, u16 opcode)
{
    u8 rs = bits(opcode, 3, 4);
    u8 rd = (bit(opcode, 7) << 3) | bits(opcode, 0, 3);
    u8 op = bits(opcode, 8, 2);

    u32 dummy;
    switch (op) {
        case 0:
            puts("ADD HI");
            reg(rd) = alu_add(reg(rd), reg(rs), &dummy);
        break;
        case 1: {
            puts("CMP HI");
            u32 result = alu_sub(reg(rd), reg(rs), &this->cpsr);
            cpu_update_zn(result, &this->cpsr);
        }
        break;
        case 2:
            puts("MOV HI");
            reg(rd) = reg(rs);
        break;
    }

    if (rd == 15 && op != 1) {
        this->pc_changed = 1;
    }

    return 1;
}

static uint thumb_load_address(Cpu *this, u16 opcode)
{
    u32 imm = bits(opcode, 0, 8) << 2;
    u8 rd = bits(opcode, 8, 3);

    puts("ADR");

    if (bit(opcode, 11)) {
        reg(rd) = reg(13) + imm;
    } else {
        reg(rd) = (reg(15) & ~2) + imm;
    }

    return 1;
}

static uint thumb_add_stack_pointer(Cpu *this, u16 opcode)
{
    u32 offset = bits(opcode, 0, 7) << 2;

    if (bit(opcode, 7)) {
        reg(13) -= offset;
    } else {
        reg(13) += offset;
    }

    return 1;
}

static u32 alu_lsl(u32 op1, u32 op2, u32 *cpsr)
{
    u32 result = op1;
    u32 carry = bit(*cpsr, PSR_BIT_C);
    u8 byte = bits(op2, 0, 8);

    if (byte == 0) {
        // unaffected
    } else if (byte < 32) {
        carry = bit(op1, 32 - byte);
        result = op1 << byte;
    } else if (byte == 32) {
        carry = bit(op1, 0);
        result = 0;
    } else {
        carry = 0;
        result = 0;
    }

    if (carry)
        set_bit(*cpsr, PSR_BIT_C);
    else
        clear_bit(*cpsr, PSR_BIT_C);

    return result;
}

static u32 alu_lsr(u32 op1, u32 op2, u32 *cpsr)
{
    u32 result = op1;
    u32 carry = bit(*cpsr, PSR_BIT_C);
    u8 byte = bits(op2, 0, 8);

    if (byte == 0) {
        // unaffected
    } else if (byte < 32) {
        carry = bit(op1, byte - 1);
        result = op1 >> byte;
    } else if (byte == 32) {
        carry = bit(op1, 31);
        result = 0;
    } else {
        carry = 0;
        result = 0;
    }

    if (carry)
        set_bit(*cpsr, PSR_BIT_C);
    else
        clear_bit(*cpsr, PSR_BIT_C);

    return result;
}

static u32 alu_asr(u32 op1, u32 op2, u32 *cpsr)
{
    u32 result = op1;
    u32 carry = bit(*cpsr, PSR_BIT_C);
    u8 byte = bits(op2, 0, 8);

    if (byte == 0) {
        // unaffected
    } else if (byte < 32) {
        carry = bit(op1, byte - 1);
        result = asr32(op1, byte);
    } else {
        carry = bit(op1, 31);
        if (bit(op1, 31)) {
            result = 0xFFFFFFFF;
        } else {
            result = 0;
        }
    }

    if (carry)
        set_bit(*cpsr, PSR_BIT_C);
    else
        clear_bit(*cpsr, PSR_BIT_C);

    return result;
}

static u32 alu_ror(u32 op1, u32 op2, u32 *cpsr)
{
    u32 result = op1;
    u32 carry = bit(*cpsr, PSR_BIT_C);
    u8 byte = bits(op2, 0, 8);
    u8 nibble = bits(op2, 0, 4);

    if (byte == 0) {
        // unaffected
    } else if (nibble == 0) {
        carry = bit(op1, 31);
    } else {
        carry = bit(op1, nibble - 1);
        result = ror32(op1, nibble);
    }

    if (carry)
        set_bit(*cpsr, PSR_BIT_C);
    else
        clear_bit(*cpsr, PSR_BIT_C);

    return result;
    return 0;
}

static u32 alu_neg(u32 op1, u32 op2, u32 *cpsr)
{
    return 0;
}

static u32 alu_mul(u32 op1, u32 op2, u32 *cpsr)
{
    return 0;
}

static const char *bin16_str(u16 data)
{
    static char buff[] = "XXXX_XXXX_XXXX_XXXX";
    for (int i = 18; i >= 0; i--) {
        if (buff[i] == '_')
            continue;
        buff[i] = "01"[data & 1];
        data >>= 1;
    }
    return buff;
}

static const char *bin8_str(u8 data)
{
    static char buff[] = "XXXX_XXXX";
    for (int i = 8; i >= 0; i--) {
        if (buff[i] == '_')
            continue;
        buff[i] = "01"[data & 1];
        data >>= 1;
    }
    return buff;
}
