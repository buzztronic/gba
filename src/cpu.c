#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include "cpu.h"
#include "thumb.h"
#include "common.h"

// declarations
static uint cpu_step_arm(Cpu *this);
static u32 cpu_fetch_arm(Cpu *this);
static void cpu_reset_pipeline(Cpu *this);

static uint cpu_execute_not_implemented(Cpu *this, u32 opcode);
static uint cpu_execute_branch(Cpu *this, u32 opcode);
static uint cpu_execute_alu(Cpu *this, u32 opcode);
static uint cpu_execute_signed_transfer(Cpu *this, u32 opcode);
static void cpu_build_decode_table(Cpu *this);
static void cpu_build_condition_table(Cpu *this);

static u32 compute_shift(Cpu *this, u32 opcode, u32 rm, u32 *carry);

static u32 alu_and(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_eor(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_sub(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_rsb(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_add(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_adc(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_sbc(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_rsc(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_tst(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_teq(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_cmp(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_cmn(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_orr(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_mov(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_bic(u32 op1, u32 op2, u32 *cpsr);
static u32 alu_mvn(u32 op1, u32 op2, u32 *cpsr);
static u32 (*alu[16])(u32 op1, u32 op2, u32 *cpsr) = {
    alu_and,
    alu_eor,
    alu_sub,
    alu_rsb,
    alu_add,
    alu_adc,
    alu_sbc,
    alu_rsc,
    alu_tst,
    alu_teq,
    alu_cmp,
    alu_cmn,
    alu_orr,
    alu_mov,
    alu_bic,
    alu_mvn
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

    cpu_build_decode_table(cpu);
    cpu_build_condition_table(cpu);

    return cpu;
}

uint cpu_step(Cpu *this)
{
    if (is_clear(this->cpsr, PSR_BIT_T)) {
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

    // compute the index for the condition lookup table
    u8 cond_idx = bits(opcode, 28, 4);
    cond_idx |= bits(this->cpsr, 28, 4) << 4;

    // decode and execute
    if (this->cond_pass[cond_idx]) {
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
    u32 alu_opcode = bits(opcode, 21, 4);
    u32 rn = bits(opcode, 16, 4);
    u32 rd = bits(opcode, 12, 4);
    u32 bit_s = bit(opcode, 20);
    u32 bit_i = bit(opcode, 25);
    u32 rn_val = reg(rn); // might change to PC + 12
    u32 op2 = 0;
    u32 result = 0;

    if (bit_i) {
        u32 imm = bits(opcode, 0, 8);
        u32 rot_imm = bits(opcode, 8, 4);
        op2 = ror32(imm, rot_imm * 2);
    } else {
        u32 rm = bits(opcode, 0, 4);
        u32 rm_val = reg(rm); // might change to PC + 12

        // we might read PC+12 instead of PC+8
        if (bit(opcode, 4)) {
            if (rm == 15)
                rm_val = reg(15) + 12;
            if (rn == 15)
                rn_val = reg(15) + 12;
        }

        u32 shift_carry = 0;
        op2 = compute_shift(this, opcode, rm_val, &shift_carry);

        if (shift_carry) {
            set_bit(this->cpsr, PSR_BIT_C);
        } else {
            clear_bit(this->cpsr, PSR_BIT_C);
        }
    }


    // now we have calculated op2
    u32 cpsr_copy = this->cpsr;
    result = alu[alu_opcode](rn_val, op2, &cpsr_copy);

    // return 0 when the alu_opcode is not implemented yet
    if (alu_opcode != 13 &&
        alu_opcode != 4 &&
        alu_opcode != 10 &&
        alu_opcode != 12 &&
        alu_opcode != 0 &&
        alu_opcode != 8) {
        return 0;
    }

    // update cpsr
    if (bit_s && rd == 15) {
        // TODO: assert we are not in user mode
        this->cpsr = this->spsr[this->cpsr & PSR_MASK_MODE];
    } else if (bit_s) {
        this->cpsr = cpsr_copy;

        // Zero
        if (result == 0) {
            set_bit(this->cpsr, PSR_BIT_Z);
        } else {
            clear_bit(this->cpsr, PSR_BIT_Z);
        }

        // Negative
        if (bit(result, 31)) {
            set_bit(this->cpsr, PSR_BIT_N);
        } else {
            clear_bit(this->cpsr, PSR_BIT_N);
        }
    }

    // store result in Rd
    // TST, TEQ, CMP, CMN don't write the result
    if (alu_opcode < 8 || alu_opcode > 11) {
        reg(rd) = result;
        if (rd == 15) {
            this->pc_changed = 1;
        }
    }

    return 1;
}

static uint cpu_execute_signed_transfer(Cpu *this, u32 opcode)
{
    // page 34
    // review this implemention it is probably bugged
    puts("LD/ST Signed");

    u32 bit_p = bit(opcode, 24);
    u32 bit_u = bit(opcode, 23);
    u32 bit_i = bit(opcode, 22);
    u32 bit_w = bit(opcode, 21);
    u32 bit_l = bit(opcode, 20);
    u32 bits_sh = bits(opcode, 5, 2);
    u32 rn = bits(opcode, 16, 4);
    u32 rd = bits(opcode, 12, 4);
    u32 rm = bits(opcode, 0, 4);
    u32 offset = (bits(opcode, 8, 4) << 4) | bits(opcode, 0, 4);

    // rn = base (including r15 = pc+8)
    // rd = src/dst (including r15 = pc+12)
    // rm = offset register
    // load/store halfword
    // load signed bytes/halfwords
    // address = base +/- offset
    // base = address (if auto-indexing enabled)

    if (!bit_i) {
        offset = reg(rm);
    }

    u32 addr = reg(rn);

    // pre-indexed
    if (bit_p) {
        if (bit_u)
            addr += offset;
        else
            addr -= offset;
    }

    if (bit_l) {
        switch (bits_sh) {
            case 1:
                // unsigned halfword
                reg(rd) = bus_read16(this->bus, addr);
            break;
            case 2:
                // signed byte
                reg(rd) = (i32)bus_read(this->bus, addr);
            break;
            case 3:
                // signed halfword
                reg(rd) = (i32)bus_read16(this->bus, addr);
            break;
        }
    } else {
        // store
        u32 data = reg(rd);
        if (rd == 15)
            data += 4;
        bus_write16(this->bus, addr, data);
    }

    // post-indexed
    if (!bit_p) {
        if (bit_u)
            addr += offset;
        else
            addr -= offset;

        assert(bit_w == 0);
    }

    // write-back
    if (!bit_p || bit_w) {
        // NOTE: write-back is always enabled when P=0 and W bit is not used
        // and must be 0

        reg(rn) = addr;
        if (rn == 15) {
            assert(rn != 15);
        }
    }

    // PC has changed
    if (bit_l && rd == 15) {
        this->pc_changed = 1;
    }
    return 2;
}

static void cpu_build_decode_table(Cpu *this)
{
    // TODO Clean up this mess
    for (uint idx = 0; idx < (1 << 12); ++idx) {
        // unpack idx to make it easier to follow documentation
        // bits[0:3] become bits[4:7]
        // bits[4:12] become bits[20:27]
        u32 opcode = (bits(idx, 4, 8) << 20) | (bits(idx, 0, 4) << 4);
        if (bits(opcode, 25, 3) == 5) {
            this->decode[idx] = cpu_execute_branch;
            continue;
        }
        if (bits(opcode, 26, 2) == 0) {
            u32 alu_opcode = bits(opcode, 21, 4);
            // if bit[25] == 0 bit[4] == 1 bit[7] == 1 then it is not an ALU
            // instruction.
            // if alu opcode is one of CMP/CMN/TST/TEQ then bit[20] must be 1
            // otherwise it is not an ALU instruction.
            if (bit(opcode, 25) == 0 && bit(opcode, 4) && bit(opcode, 7)) {
            } else if (bit(opcode, 20) == 0 && alu_opcode >= 8 && alu_opcode <= 11) {
            } else {
                this->decode[idx] = cpu_execute_alu;
                continue;
            }
        }
        if (bit(opcode, 22) == 0 && bits(opcode, 11, 4) != 0) {
            // not signed transfer
        } else if (bit(opcode, 24) == 0 && bit(opcode, 21) == 1) {
            // not signed transfer
        } else if (bits(opcode, 25, 3) == 0 && bit(opcode, 4) && bit(opcode, 7)) {
            this->decode[idx] = cpu_execute_signed_transfer;
            continue;
        }
        this->decode[idx] = cpu_execute_not_implemented;
    }
}


static void cpu_build_condition_table(Cpu *this)
{
    for (uint idx = 0; idx < (1 << 8); ++idx) {
        u32 cpsr = bits(idx, 4, 8) << 28;
        switch (bits(idx, 0, 4)) {
            case 0x0:
                // EQ
                this->cond_pass[idx] = is_set(cpsr, PSR_BIT_Z);
            break;
            case 0x1:
                // NE
                this->cond_pass[idx] = is_clear(cpsr, PSR_BIT_Z);
            break;
            case 0x2:
                // CS
                this->cond_pass[idx] = is_set(cpsr, PSR_BIT_C);
            break;
            case 0x3:
                // CC
                this->cond_pass[idx] = is_clear(cpsr, PSR_BIT_C);
            break;
            case 0x4:
                // MI
                this->cond_pass[idx] = is_set(cpsr, PSR_BIT_N);
            break;
            case 0x5:
                // PL
                this->cond_pass[idx] = is_clear(cpsr, PSR_BIT_N);
            break;
            case 0x6:
                // VS
                this->cond_pass[idx] = is_set(cpsr, PSR_BIT_V);
            break;
            case 0x7:
                // VC
                this->cond_pass[idx] = is_clear(cpsr, PSR_BIT_V);
            break;
            case 0x8:
                // HI
                this->cond_pass[idx] = is_set(cpsr, PSR_BIT_C) && is_clear(cpsr, PSR_BIT_Z);
            break;
            case 0x9:
                // LS
                this->cond_pass[idx] = is_clear(cpsr, PSR_BIT_C) && is_set(cpsr, PSR_BIT_Z);
            break;
            case 0xA:
                // GE
                this->cond_pass[idx] = bit(cpsr, PSR_BIT_N) == bit(cpsr, PSR_BIT_V);
            break;
            case 0xB:
                // LT
                this->cond_pass[idx] = bit(cpsr, PSR_BIT_N) != bit(cpsr, PSR_BIT_V);
            break;
            case 0xC:
                // GT
                this->cond_pass[idx] = is_clear(cpsr, PSR_BIT_Z) && bit(cpsr, PSR_BIT_N) == bit(cpsr, PSR_BIT_V);
            break;
            case 0xD:
                // LE
                this->cond_pass[idx] = is_clear(cpsr, PSR_BIT_Z) && bit(cpsr, PSR_BIT_N) != bit(cpsr, PSR_BIT_V);
            break;
            case 0xE:
                // AL
                this->cond_pass[idx] = 1;
            break;
        }
    }
}

static u32 alu_and(u32 op1, u32 op2, u32 *cpsr)
{
    puts("AND");
    (void)cpsr;
    return op1 & op2;
}

static u32 alu_eor(u32 op1, u32 op2, u32 *cpsr)
{
    puts("EOR");
}

static u32 alu_sub(u32 op1, u32 op2, u32 *cpsr)
{
    puts("SUB");
}

static u32 alu_rsb(u32 op1, u32 op2, u32 *cpsr)
{
    puts("RSB");
}

static u32 alu_add(u32 op1, u32 op2, u32 *cpsr)
{
    u32 result = op1 + op2;
    u8 result_s = !!(result & BIT_31);
    u8 op1_s = !!(op1 & BIT_31);
    u8 op2_s = !!(op2 & BIT_31);
    puts("ADD");

    if (op1_s == op2_s && result_s != op1_s)
        set_bit(*cpsr, PSR_BIT_V);
    else
        clear_bit(*cpsr, PSR_BIT_V);

    if (result < op1 || result < op2)
        set_bit(*cpsr, PSR_BIT_C);
    else
        clear_bit(*cpsr, PSR_BIT_C);

    return result;
}

static u32 alu_adc(u32 op1, u32 op2, u32 *cpsr)
{
    puts("ADC");
}

static u32 alu_sbc(u32 op1, u32 op2, u32 *cpsr)
{
    puts("SBC");
}

static u32 alu_rsc(u32 op1, u32 op2, u32 *cpsr)
{
    puts("RSC");
}

static u32 alu_tst(u32 op1, u32 op2, u32 *cpsr)
{
    puts("TST");
    return op1 & op2;
}

static u32 alu_teq(u32 op1, u32 op2, u32 *cpsr)
{
    puts("alu_teq");
}

static u32 alu_cmp(u32 op1, u32 op2, u32 *cpsr)
{
    u32 result = op1 - op2;
    u8 result_s = !!(result & BIT_31);
    u8 op1_s = !!(op1 & BIT_31);
    u8 op2_s = !!(op2 & BIT_31);
    puts("CMP");

    if (op1 > op2)
        set_bit(*cpsr, PSR_BIT_C);
    else
        clear_bit(*cpsr, PSR_BIT_C);

    if (op1_s != op2_s && op1_s != result_s)
        set_bit(*cpsr, PSR_BIT_V);
    else
        clear_bit(*cpsr, PSR_BIT_V);

    return result;
}

static u32 alu_cmn(u32 op1, u32 op2, u32 *cpsr)
{
    puts("CMN");
}

static u32 alu_orr(u32 op1, u32 op2, u32 *cpsr)
{
    (void)cpsr;
    puts("ORR");
    return op1 | op2;
}

static u32 alu_mov(u32 op1, u32 op2, u32 *cpsr)
{
    puts("MOV");
    (void)op1;
    (void)cpsr;
    return op2;
}

static u32 alu_bic(u32 op1, u32 op2, u32 *cpsr)
{
    puts("BIC");
}

static u32 alu_mvn(u32 op1, u32 op2, u32 *cpsr)
{
    puts("MVN");
}

// I recommend not reading this thing
static u32 compute_shift(Cpu *this, u32 opcode, u32 rm, u32 *carry)
{
    u32 type = bits(opcode, 5, 2);
    u32 shift_imm = bits(opcode, 7, 5);
    u32 rs = bits(opcode, 8, 4);
    u32 rs_val = bits(reg(rs), 0, 7);
    u32 bit_r = bit(opcode, 4);
    u32 op2 = 0;

    if (rs == 15) {
        assert(0);
    }

    *carry = bit(this->cpsr, PSR_BIT_C);

    if (!bit_r && type == 0) {
        // Logical shift left by immediate
        if (shift_imm == 0) {
            op2 = rm;
        } else {
            op2 = rm << shift_imm;
            *carry = bit(rm, 32 - shift_imm);
        }
    } else if (bit_r && type == 0) {
        // Logical shift left by register
        if (rs_val == 0) {
            op2 = rm;
        } else if (rs_val < 32) {
            op2 = rm << rs_val;
            *carry = bit(rm, 32 - rs_val);
        } else if (rs_val == 32) {
            op2 = 0;
            *carry = bit(rm, 0);
        } else {
            op2 = 0;
            *carry = 0;
        }
    } else if (!bit_r && type == 1) {
        // Logical shift right by immediate
        if (shift_imm == 0) {
            op2 = 0;
            *carry = bit(rm, 31);
        } else {
            op2 = rm >> shift_imm;
            *carry = bit(rm, shift_imm - 1);
        }
    } else if (bit_r && type == 1) {
        // Logical shift right by register

        if (rs_val == 0) {
            op2 = rm;
        } else if (rs_val < 32) {
            op2 = rm >> rs_val;
            *carry = bit(rm, rs_val - 1);
        } else if (rs_val == 32) {
            op2 = 0;
            *carry = bit(rm, 31);
        } else {
            op2 = 0;
            *carry = 0;
        }
    } else if (!bit_r && type == 2) {
        // Arithmetic shift right by immediate
        if (shift_imm == 0) {
            *carry = bit(rm, 31);
            if (*carry) {
                op2 = 0xFFFFFFFF;
            } else {
                op2 = 0;
            }
        } else {
            op2 = asr32(rm, shift_imm);;
            *carry = bit(rm, shift_imm - 1);
        }
    } else if (bit_r && type == 2) {
        // Arithmetic shift right by register

        if (rs_val == 0) {
            op2 = rm;
        } else if (rs_val < 32) {
            op2 = asr32(rm, rs_val);
            *carry = bit(rm, rs_val - 1);
        } else if (bit(rm, 31)) {
            op2 = 0xFFFFFFFF;
            *carry = 1;
        } else {
            op2 = 0;
            *carry = 0;
        }
    } else if (!bit_r && type == 3) {
        // Rotate right by immediate
        if (shift_imm == 0) {
            op2 = (!(this->cpsr & PSR_BIT_C)) << 31;
            op2 |= rm >> 1;
            *carry = bit(rm, 0);
        } else {
            op2 = ror32(rm, shift_imm);
            *carry = bit(rm, shift_imm - 1);
        }
    } else if (bit_r && type == 3) {
        // Rotate right by register
        if (rs_val == 0) {
            op2 = rm;
        } else if (bits(rs_val, 0, 4) == 0) {
            op2 = rm;
            *carry = bit(rm, 31);
        } else {
            op2 = ror32(rm, bits(rs_val, 0, 4));
            *carry = bit(rm, bits(rs_val, 0, 4) - 1);
        }
    } else {
        assert(0);
    }

    return op2;
}
