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
static uint thumb_branch_link(Cpu *this, u16 opcode);
static uint thumb_load_pc_relative(Cpu *this, u16 opcode);
static uint thumb_ldst_register_offset(Cpu *this, u16 opcode);
static uint thumb_ldst_signed(Cpu *this, u16 opcode);
static uint thumb_ldst_immediate(Cpu *this, u16 opcode);
static uint thumb_ldst_halfword(Cpu *this, u16 opcode);
static uint thumb_ldst_sp_relative(Cpu *this, u16 opcode);
static uint thumb_push_pop_registers(Cpu *this, u16 opcode);
static uint thumb_ldst_multiple(Cpu *this, u16 opcode);
static uint thumb_software_interrupt(Cpu *this, u16 opcode);

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
            this->decode_thumb[idx] = thumb_load_pc_relative;
            continue;
        }

        if (bits(opcode, 12, 4) == 5) {
            if (bit(opcode, 9) == 0) {
                // Load/store with register offset
                this->decode_thumb[idx] = thumb_ldst_register_offset;
            } else {
                // Load/store sign-extended byte/halfword
                this->decode_thumb[idx] = thumb_ldst_signed;
            }
            continue;
        }

        if (bits(opcode, 13, 3) == 3) {
            // Load/store with immediate offset
            this->decode_thumb[idx] = thumb_ldst_immediate;
            continue;
        }

        if (bits(opcode, 12, 4) == 8) {
            // Load/store halfword
            this->decode_thumb[idx] = thumb_ldst_halfword;
            continue;
        }

        if (bits(opcode, 12, 4) == 9) {
            // SP-relative load/store
            this->decode_thumb[idx] = thumb_ldst_sp_relative;
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
                this->decode_thumb[idx] = thumb_push_pop_registers;
            }
            continue;
        }

        if (bits(opcode, 12, 4) == 12) {
            // Multiple load/store
            this->decode_thumb[idx] = thumb_ldst_multiple;
            continue;
        }

        if (bits(opcode, 12, 4) == 13) {
            if (bits(opcode, 8, 4) != 0xF) {
                // Conditional branch
                this->decode_thumb[idx] = thumb_cond_branch;
            } else {
                // Software Interrupt
                this->decode_thumb[idx] = thumb_software_interrupt;
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
            this->decode_thumb[idx] = thumb_branch_link;
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
    u32 offset = bits(opcode, 0, 11);
    if (bit(offset, 10)) {
        offset |= ~(u32)0  << 11;
    }

    reg(15) += offset << 1;
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
    puts("ADD PC");
    u32 offset = bits(opcode, 0, 7) << 2;

    if (bit(opcode, 7)) {
        reg(13) -= offset;
    } else {
        reg(13) += offset;
    }

    return 1;
}

static uint thumb_branch_link(Cpu *this, u16 opcode)
{
    if (bit(opcode, 11) == 0) {
        puts("BLL");
        u32 offset = bits(opcode, 0, 11);
        if (bit(offset, 10)) {
            offset |= ~(u32)0  << 11;
        }
        offset <<= 12;
        reg(14) = reg(15) + offset;
    } else {
        puts("BLH");
        u32 pc = reg(15);

        reg(15) = reg(14) + (bits(opcode, 0, 11) << 1);
        reg(14) = (pc - 2) | 1;
        this->pc_changed = 1;
    }

    return 1;
}

static uint thumb_load_pc_relative(Cpu *this, u16 opcode)
{
    puts("LDR PC");
    u8 rd = bits(opcode, 8, 3);
    u32 offset = bits(opcode, 0, 8) << 2;

    reg(rd) = bus_read32(this->bus, (reg(15) & ~2) + offset);

    return 1;
}

static uint thumb_ldst_register_offset(Cpu *this, u16 opcode)
{
    u8 flag_l = bit(opcode, 11);
    u8 flag_b = bit(opcode, 10);
    u8 rd = bits(opcode, 0, 3);

    u8 rb = bits(opcode, 3, 3);
    u8 ro = bits(opcode, 6, 3);

    u32 addr = reg(rb) + reg(ro);

    // load
    if (flag_l) {
        // byte
        if (flag_b) {
            puts("LDRB");
            reg(rd) = bus_read(this->bus, addr);
        }

        // word
        if (!flag_b) {
            puts("LDR");
            if (addr & 0x3) {
                // read at a word aligned address
                reg(rd) = bus_read32(this->bus, addr & ~0x3);

                // rotate such that lower byte of rd matches the addressed byte
                reg(rd) = ror32(reg(rd), 8 * (addr % 4));
            } else {
                reg(rd) = bus_read32(this->bus, addr);
            }
        }
    }

    // store
    if (!flag_l) {
        // byte
        if (flag_b) {
            puts("STRB");
            bus_write(this->bus, addr, reg(rd));
        }

        // word
        if (!flag_b){
            puts("STR");
            bus_write32(this->bus, addr & ~0x3, reg(rd));
        }
    }

    return 1;
}

static uint thumb_ldst_signed(Cpu *this, u16 opcode)
{
    u8 op = bits(opcode, 10, 2);
    u8 rd = bits(opcode, 0, 3);
    u8 rb = bits(opcode, 3, 3);
    u8 ro = bits(opcode, 6, 3);

    u32 addr = reg(rb) + reg(ro);

    switch (op) {
        // STRH
        case 0:
            puts("STRH");
            bus_write16(this->bus, addr & ~0x1, reg(rd));
        break;

        // LDSB
        case 1:
            puts("LDSB");
            reg(rd) = (i8)bus_read(this->bus, addr);
        break;

        // LDRH
        case 2:
            puts("LDRH");
            if (addr % 2) {
                reg(rd) = ror32(bus_read16(this->bus, addr-1), 8);
            } else {
                reg(rd) = bus_read16(this->bus, addr);
            }
        break;

        // LDSH
        case 3:
            puts("LDSH");
            if (addr % 2) {
                reg(rd) = (i8)bus_read(this->bus, addr);
            } else {
                reg(rd) = (i16)bus_read16(this->bus, addr);
            }
        break;
    }

    return 1;
}

static uint thumb_ldst_immediate(Cpu *this, u16 opcode)
{
    u8 rd = bits(opcode, 0, 3);
    u8 rb = bits(opcode, 3, 3);
    u8 op = bits(opcode, 11, 2);
    u32 addr = bits(opcode, 6, 5);

    if (bit(opcode, 12) == 0)
        addr <<= 2;

    addr += reg(rb);

    switch (op) {
        // STR
        case 0:
            puts("STR");
            bus_write32(this->bus, addr & ~3, reg(rd));
        break;

        // LDR
        case 1:
            puts("LDR");
            if (addr & 0x3) {
                // read at a word aligned address
                reg(rd) = bus_read32(this->bus, addr & ~0x3);

                // rotate such that lower byte of rd matches the addressed byte
                reg(rd) = ror32(reg(rd), 8 * (addr % 4));
            } else {
                reg(rd) = bus_read32(this->bus, addr);
            }
        break;

        // STRB
        case 2:
            puts("STRB");
            bus_write(this->bus, addr, reg(rd));
        break;

        // LDRB
        case 3:
            puts("LDRB");
            reg(rd) = bus_read(this->bus, addr);
        break;
    }

    return 1;
}

static uint thumb_ldst_halfword(Cpu *this, u16 opcode)
{
    u8 rd = bits(opcode, 0, 3);
    u8 rb = bits(opcode, 3, 3);
    u8 flag_l = bit(opcode, 11);

    u32 addr = bits(opcode, 6, 5) << 1;
    addr += reg(rb);

    // Load
    if (flag_l) {
        puts("LDRH");
        if (addr % 2) {
            reg(rd) = ror32(bus_read16(this->bus, addr-1), 8);
        } else {
            reg(rd) = bus_read16(this->bus, addr);
        }
    }

    // Store
    if (!flag_l) {
        puts("STRH");
        bus_write16(this->bus, addr & ~0x1, reg(rd));
    }

    return 1;
}

static uint thumb_ldst_sp_relative(Cpu *this, u16 opcode)
{
    u8 rd = bits(opcode, 8, 3);
    u8 flag_l = bit(opcode, 11);

    u32 addr = bits(opcode, 0, 8) << 2;
    addr += reg(13);

    // Load
    if (flag_l) {
        puts("LDR SP");
        if (addr & 0x3) {
            // read at a word aligned address
            reg(rd) = bus_read32(this->bus, addr & ~0x3);

            // rotate such that lower byte of rd matches the addressed byte
            reg(rd) = ror32(reg(rd), 8 * (addr % 4));
        } else {
            reg(rd) = bus_read32(this->bus, addr);
        }
    }

    // Store
    if (!flag_l) {
        puts("STR SP");
        bus_write32(this->bus, addr & ~3, reg(rd));
    }

    return 1;
}

static uint thumb_push_pop_registers(Cpu *this, u16 opcode)
{
    u8 flag_l = bit(opcode, 11);

    // number of registers to be transfered
    u32 nreg = bit(opcode, 8);
    for (u32 i = 0; i < 8; ++i) {
        nreg += bit(opcode, i);
    }

    u32 sp = reg(13);
    u32 addr = sp;

    // POP
    if (flag_l) {
        puts("POP");
        sp += nreg * 4;
    }

    // PUSH
    if (!flag_l) {
        puts("PUSH");
        sp -= nreg * 4;
        addr = sp;
    }

    for (uint i = 0; i < 8; i++) {
        if (is_clear(opcode, i))
            continue;

        // POP
        if (flag_l) {
            reg(i) = bus_read32(this->bus, addr & ~3);
        }

        // PUSH
        if (!flag_l) {
            bus_write32(this->bus, addr & ~3, reg(i));
        }

        addr += 4;
    }

    if (bit(opcode, 8)) {
        // POP PC
        if (flag_l) {
            reg(15) = bus_read32(this->bus, addr & ~3) & ~1;
            this->pc_changed = 1;
        }

        // PUSH LR
        if (!flag_l) {
            bus_write32(this->bus, addr & ~3, reg(14));
        }
    }

    reg(13) = sp;

    return 1;
}

static uint thumb_ldst_multiple(Cpu *this, u16 opcode)
{
    u8 rb = bits(opcode, 8, 3);
    u8 flag_l = bit(opcode, 11);

    // number of registers to be transfered
    u32 nreg = 0;
    for (u32 i = 0; i < 8; ++i) {
        nreg += bit(opcode, i);
    }

    // strange stuff happen
    if (nreg == 0) {
        if (flag_l) {
            puts("LDM Empty");
            reg(15) = bus_read32(this->bus, reg(rb) & ~3);
            this->pc_changed = 1;
        } else {
            puts("STM Empty");
            bus_write32(this->bus, reg(rb) & ~3, reg(15)+2);
        }
        reg(rb) += 0x40;

        return 1;
    }

    u32 addr = reg(rb);
    u32 base = addr + nreg * 4;

    if (flag_l)
        puts("LDM");
    else
        puts("STM");

    uint cycles = 1;
    for (uint i = 0; i < 8; i++) {
        if (is_clear(opcode, i))
            continue;

        cycles++;

        // Load
        if (flag_l) {
            reg(i) = bus_read32(this->bus, addr & ~3);
        }

        // Store
        if (!flag_l) {
            bus_write32(this->bus, addr & ~3, reg(i));
        }

        // write-back is done at the second cycle
        if (cycles == 2) {
            reg(rb) = base;
        }

        addr += 4;
    }

    return 1;
}

static uint thumb_software_interrupt(Cpu *this, u16 opcode)
{
    this->reg_svc[1] = reg(15) - 2;
    this->spsr[CPU_MODE_SVC] = this->cpsr;

    this->cpsr &= 0xFFFFFFF0;
    this->cpsr |= CPU_MODE_SVC;
    clear_bit(this->cpsr, PSR_BIT_T);
    set_bit(this->cpsr, PSR_BIT_I);

    void cpu_bank_registers(Cpu *);
    cpu_bank_registers(this);

    reg(15) = 0x00000008;
    this->pc_changed = 1;

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
    u8 low = bits(op2, 0, 5);

    if (byte == 0) {
        // unaffected
    } else if (low == 0) {
        carry = bit(op1, 31);
    } else {
        carry = bit(op1, low - 1);
        result = ror32(op1, low);
    }

    if (carry)
        set_bit(*cpsr, PSR_BIT_C);
    else
        clear_bit(*cpsr, PSR_BIT_C);

    return result;
}

static u32 alu_neg(u32 op1, u32 op2, u32 *cpsr)
{
    if (op2 == 0)
        set_bit(*cpsr, PSR_BIT_C);
    else
        clear_bit(*cpsr, PSR_BIT_C);

    if (op2 == ~0)
        set_bit(*cpsr, PSR_BIT_V);
    else
        clear_bit(*cpsr, PSR_BIT_V);

    return -op2;
}

static u32 alu_mul(u32 op1, u32 op2, u32 *cpsr)
{
    return op1 * op2;
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
