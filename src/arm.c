#include <assert.h>

#include "cpu.h"
#include "arm.h"
#include "common.h"

static uint (*arm_decode_lut[1 << 12]) (struct Cpu *, u32);

// declarations
static u32 arm_fetch(Cpu *this);

static uint arm_execute_not_implemented(Cpu *this, u32 opcode);
static uint arm_execute_branch(Cpu *this, u32 opcode);
static uint arm_execute_alu(Cpu *this, u32 opcode);
static uint arm_execute_signed_transfer(Cpu *this, u32 opcode);
static uint arm_execute_block_transfer(Cpu *this, u32 opcode);
static uint arm_execute_single_transfer(Cpu *this, u32 opcode);
static uint arm_execute_psr_transfer(Cpu *this, u32 opcode);
static uint arm_execute_branch_exchange(Cpu *this, u32 opcode);
static uint arm_execute_data_swap(Cpu *this, u32 opcode);
static uint arm_execute_multiply(Cpu *this, u32 opcode);
static uint arm_execute_multiply_long(Cpu *this, u32 opcode);

static u32 compute_shift(Cpu *this, u32 opcode, u32 rm, u32 *carry);

static u32 (*alu[16])(u32 op1, u32 op2, u32 *cpsr) = {
    alu_and,
    alu_eor,
    alu_sub,
    alu_rsb,
    alu_add,
    alu_adc,
    alu_sbc,
    alu_rsc,

    alu_and, // tst
    alu_eor, // teq
    alu_sub, // cmp
    alu_add, // cmn

    alu_orr,
    alu_mov,
    alu_bic,
    alu_mvn
};

// functions
uint arm_step(Cpu *this)
{
    u32 opcode = arm_fetch(this);
    uint cycles = 0;

    // compute the index for the condition lookup table
    u8 cond_idx = bits(opcode, 28, 4);
    cond_idx |= bits(this->cpsr, 28, 4) << 4;

    // decode and execute
    if (cpu_cond_lut[cond_idx]) {
        u32 index = (bits(opcode, 20, 8) << 4) | bits(opcode, 4, 4);
        cycles = arm_decode_lut[index](this, opcode);
    } else {
        cycles = 1;
    }

    return cycles;
}

u32 arm_fetch(Cpu *this)
{
    const u32 opcode = this->execute_opcode;
    this->execute_opcode = this->decode_opcode;
    this->decode_opcode = bus_read32(this->bus, reg(15));

    return opcode;
}

void arm_flush_pipeline(Cpu *this)
{
    this->execute_opcode = bus_read32(this->bus, reg(15));
    this->decode_opcode = bus_read32(this->bus, reg(15) + 4);
    reg(15) += 8;
}

uint arm_execute_not_implemented(Cpu *this, u32 opcode)
{
    eprintf("arm: unimplemented opcode: %08X\n", opcode);
    return 0;
}

uint arm_execute_branch(Cpu *this, u32 opcode)
{
    if (bit(opcode, 24)) {
        // branch with link
        reg(14) = reg(15) - 4;
    }

    u32 offset = bits(opcode, 0, 24);
    if (bit(offset, 23)) {
        // sign extention
        offset |= 0xFF000000;
    }
    offset <<= 2;

    reg(15) += (i32)offset;
    this->pc_changed = 1;

    return 1;
}

static uint arm_execute_alu(Cpu *this, u32 opcode)
{
    u32 alu_opcode = bits(opcode, 21, 4);
    u32 rn = bits(opcode, 16, 4);
    u32 rd = bits(opcode, 12, 4);
    u32 bit_s = bit(opcode, 20);
    u32 bit_i = bit(opcode, 25);
    u32 rn_val = reg(rn); // might change to PC + 12
    u32 op2 = 0;
    u32 result = 0;


    u32 shift_carry = bit(this->cpsr, PSR_BIT_C);
    if (bit_i) {
        u32 imm = bits(opcode, 0, 8);
        u32 rot_imm = bits(opcode, 8, 4);
        op2 = ror32(imm, rot_imm * 2);
        if (rot_imm != 0) {
            if (bit(op2, 31))
                shift_carry = 1;
            else
                shift_carry = 0;
        }
    } else {
        u32 rm = bits(opcode, 0, 4);
        u32 rm_val = reg(rm); // might change to PC + 12

        // we might read PC+12 instead of PC+8
        if (bit(opcode, 4)) {
            if (rm == 15)
                rm_val = reg(15) + 4;
            if (rn == 15)
                rn_val = reg(15) + 4;
        }

        op2 = compute_shift(this, opcode, rm_val, &shift_carry);
    }


    // now we have calculated op2
    u32 cpsr_copy = this->cpsr;
    result = alu[alu_opcode](rn_val, op2, &cpsr_copy);

    // update cpsr
    if (bit_s && rd == 15) {
        // TODO: assert we are not in user mode

        // spsr_sys doesn't exist
        if ((this->cpsr & PSR_MASK_MODE) != 0xF) {
            this->cpsr = this->spsr[this->cpsr & PSR_MASK_MODE];
            cpu_bank_registers(this);
        }
    } else if (bit_s) {
        this->cpsr = cpsr_copy;

        switch (alu_opcode) {
            case 0x0: // AND
            case 0x1: // EOR
            case 0x8: // TST
            case 0x9: // TEQ
            case 0xC: // ORR
            case 0xD: // MOV
            case 0xE: // BIC
            case 0xF: // MVN
                if (shift_carry) {
                    set_bit(this->cpsr, PSR_BIT_C);
                } else {
                    clear_bit(this->cpsr, PSR_BIT_C);
                }
            break;
        }

        cpu_update_zn(result, &this->cpsr);
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

static uint arm_execute_signed_transfer(Cpu *this, u32 opcode)
{
    // page 34
    // review this implemention it is probably bugged

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
                if (addr % 2) {
                    reg(rd) = ror32(bus_read16(this->bus, addr-1), 8);
                } else {
                    reg(rd) = bus_read16(this->bus, addr);
                }
            break;
            case 2:
                // signed byte
                reg(rd) = (i8)bus_read(this->bus, addr);
            break;
            case 3:
                // signed halfword
                if (addr % 2) {
                    reg(rd) = (i8)bus_read(this->bus, addr);
                } else {
                    reg(rd) = (i16)bus_read16(this->bus, addr);
                }
            break;
        }
    } else {
        // store
        u32 data = reg(rd);
        if (rd == 15)
            data += 4;
        bus_write16(this->bus, addr & ~0x1, data);
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

        if (!bit_l || rn != rd) {
            reg(rn) = addr;
            if (rn == 15) {
                assert(rn != 15);
            }
        }
    }

    // PC has changed
    if (bit_l && rd == 15) {
        this->pc_changed = 1;
    }
    return 2;
}

static uint arm_execute_block_transfer(Cpu *this, u32 opcode)
{
    u32 flag_p = bit(opcode, 24);
    u32 flag_u = bit(opcode, 23);
    u32 flag_s = bit(opcode, 22);
    u32 flag_w = bit(opcode, 21);
    u32 flag_l = bit(opcode, 20);
    u32 rn = bits(opcode, 16, 4);

    assert(rn != 15);

    // TODO: clean up this implementation

    int use_user_registers = 0;
#define xreg(n) *(use_user_registers ? &this->reg_usr[n] : this->reg[n])

    if (flag_s) {
        // TODO: assert we are not in user mode
        // TODO: can we be in system mode? because system mode doesn't have spsr

        // STM with R15 in transfer list and S bit set (User bank transfer)
        if ((!flag_l && bit(opcode, 15))) {
            // TODO: assert write-back is not used

            // use user register not current mode registers
            use_user_registers = 1;
        }

        // R15 not in list and S bit set (User bank transfer)
        if (!bit(opcode, 15)) {
            // TODO: assert write-back is not used

            // use user register not current mode registers
            use_user_registers = 1;

            // TODO: implemented the cursed behavior that happen when the
            // instruction is LDM and you read from a banked register during
            // the following cycle.
        }
    }

    // number of registers to be transfered
    u32 nreg = 0;
    for (u32 i = 0; i < 16; ++i) {
        nreg += bit(opcode, i);
    }

    // empty Rlist
    if (nreg == 0) {
        opcode |= 1 << 15;
        nreg = 16;
    }

    // calculate the final value for the base register
    u32 base = xreg(rn);
    u32 addr = base;
    if (flag_u) {
        base += nreg * 4;
    } else {
        base -= nreg * 4;
        addr = base;
        flag_p = !flag_p;
    }

    // first register stored at the start of the second cycle
    // base written at the end of the second cycle
    u32 cycle = 1;
    for (u32 i = 0; i < 16; ++i) {
        if (is_clear(opcode, i)) {
            continue;
        }

        cycle ++;

        // pre
        if (flag_p) {
            addr += 4;
        }

        // load
        if (flag_l) {
            xreg(i) = bus_read32(this->bus, addr);
            if (i == 15) {
                if (flag_s) {
                    // CPSR = spsr_<mode>
                    // TODO: what if we are in system mode
                    if ((this->cpsr & PSR_MASK_MODE) != 0xF) {
                        this->cpsr = this->spsr[this->cpsr & PSR_MASK_MODE];
                        cpu_bank_registers(this);
                    }
                }
                this->pc_changed = 1;
            }
        }


        // store
        if (!flag_l) {
            if (i == 15) {
                bus_write32(this->bus, addr, xreg(i)+4);
            } else {
                bus_write32(this->bus, addr, xreg(i));
            }
        }

        // write back at the end of the second cycle
        if (cycle == 2 && flag_w) {
            // if it is a load and loads the base then don't write back
            if (!flag_l || !bit(opcode, rn)) {
                xreg(rn) = base;
            }
        }

        // post
        if (!flag_p) {
            addr += 4;
        }
    }

#undef xreg
    return 1;
}

static uint arm_execute_single_transfer(Cpu *this, u32 opcode)
{

    u32 flag_i = bit(opcode, 25);
    u32 flag_p = bit(opcode, 24);
    u32 flag_u = bit(opcode, 23);
    u32 flag_b = bit(opcode, 22);
    u32 flag_w = bit(opcode, 21);
    u32 flag_l = bit(opcode, 20);
    u32 rn = bits(opcode, 16, 4);
    u32 rd = bits(opcode, 12, 4);

    u32 addr = reg(rn);

    u32 offset = opcode & 0xFFF;

    // compute the offset
    if (flag_i) {
        u32 dummy;
        u32 rm = reg(opcode & 0xF);
        offset = compute_shift(this, opcode, rm, &dummy);
    }

    // pre
    if (flag_p) {
        if (flag_u) {
            addr += offset;
        } else {
            addr -= offset;
        }
    }

    // load
    if (flag_l) {
        if (flag_b) {
            // byte
            reg(rd) = bus_read(this->bus, addr);
        } else {
            // word
            if (addr & 0x3) {
                // read at a word aligned address
                reg(rd) = bus_read32(this->bus, addr & ~0x3);

                // rotate such that lower byte of rd matches the addressed byte
                reg(rd) = ror32(reg(rd), 8 * (addr % 4));
            } else {
                reg(rd) = bus_read32(this->bus, addr);
            }
        }
        if (rd == 15) {
            this->pc_changed = 1;
        }
    }

    // store
    if (!flag_l) {
        u32 data = reg(rd);
        if (rd == 15)
            data += 4;

        if (flag_b) {
            // byte
            bus_write(this->bus, addr, data);
        } else {
            // word
            bus_write32(this->bus, addr & ~0x3, data);
        }
    }

    // post
    if (!flag_p) {
        if (flag_u) {
            addr += offset;
        } else {
            addr -= offset;
        }
    }

    // write-back
    if (flag_w || !flag_p) {
        if (!flag_l || rn != rd) {
            reg(rn) = addr;
            if (rn == 15) {
                this->pc_changed = 1;
            }
        }
    }

    return 1;
}

static uint arm_execute_psr_transfer(Cpu *this, u32 opcode)
{
    u32 flag_i = opcode & BIT_25;
    u32 flag_psr = opcode & BIT_22;
    u32 flag_op = opcode & BIT_21;

    if (flag_op) {
        // MSR : Status <-- Register
        u32 flag_f = opcode & BIT_19;
        u32 flag_s = opcode & BIT_18;
        u32 flag_x = opcode & BIT_17;
        u32 flag_c = opcode & BIT_16;
        u32 rm = opcode & 0xF;
        u32 rot_imm = (opcode >> 8) & 0xF;
        u32 imm = opcode & 0xFF;

        u32 op = reg(rm);
        if (flag_i) {
            op = ror32(imm, rot_imm * 2);
        }

        u32 mask = 0;

        if (flag_c)
            mask |= 0x000000FF;
        if (flag_x)
            mask |= 0x0000FF00;
        if (flag_s)
            mask |= 0x00FF0000;
        if (flag_f)
            mask |= 0xFF000000;

        if (flag_psr) {
            this->spsr[this->cpsr & PSR_MASK_MODE] &= ~mask;
            this->spsr[this->cpsr & PSR_MASK_MODE] |= op & mask;
        } else {
            if ((this->cpsr & PSR_MASK_MODE) == 0x0) {
                // in User Mode only condition flags can be changed
                mask &= 0xFF000000;
            }
            this->cpsr &= ~mask;
            this->cpsr |= op & mask;
            cpu_bank_registers(this);
        }
    } else {
        // MRS : Register <-- Status
        u32 rd = (opcode >> 12) & 0xF;
        if (flag_psr) {
            reg(rd) = this->spsr[this->cpsr & PSR_MASK_MODE];
        } else {
            reg(rd) = this->cpsr;
        }
    }
    return 1;
}

static uint arm_execute_branch_exchange(Cpu *this, u32 opcode)
{

    u32 rn = opcode & 0xF;

    reg(15) = reg(rn);
    this->pc_changed = 1;

    if (is_set(reg(rn), 0)) {
        // Switch to Thumb
        set_bit(this->cpsr, PSR_BIT_T);
    }

    return 1;
}

static uint arm_execute_data_swap(Cpu *this, u32 opcode)
{
    u32 rn = bits(opcode,  16, 4);
    u32 rd = bits(opcode,  12, 4);
    u32 rm = bits(opcode,  0, 4);

    // TODO: assert: rn, rd and rm can't be r15

    u32 ld_opcode = 0xe5900000; // ldr R0, [R0]
    u32 st_opcode = 0xe5800000; // str R0, [R0]

    ld_opcode |= opcode & (1 << 22);
    ld_opcode |= rn << 16;
    ld_opcode |= rd << 12;

    st_opcode |= opcode & (1 << 22);
    st_opcode |= rn << 16;
    st_opcode |= rm << 12;

    if (rm == rd) {
        u32 rd1 = reg(rd);
        arm_execute_single_transfer(this, ld_opcode);

        u32 rd2 = reg(rd);
        reg(rd) = rd1;
        arm_execute_single_transfer(this, st_opcode);

        reg(rd) = rd2;
    } else {
        arm_execute_single_transfer(this, ld_opcode);
        arm_execute_single_transfer(this, st_opcode);
    }

    return 4;
}

static uint arm_execute_multiply(Cpu *this, u32 opcode)
{
    u32 rd = bits(opcode, 16, 4);
    u32 rn = bits(opcode, 12, 4);
    u32 rs = bits(opcode, 8, 4);
    u32 rm = bits(opcode, 0, 4);
    u32 bit_s = bit(opcode, 20);
    u32 bit_a = bit(opcode, 21);

    u32 result = 0;
    if (bit_a) {
        result = reg(rm) * reg(rs) + reg(rn);
    } else {
        result = reg(rm) * reg(rs);
    }

    reg(rd) = result;

    if (!bit_s)
        return 1;

    cpu_update_zn(result, &this->cpsr);

    return 1;
}

static uint arm_execute_multiply_long(Cpu *this, u32 opcode)
{
    u32 rdhi = bits(opcode, 16, 4);
    u32 rdlo = bits(opcode, 12, 4);
    u32 rs = bits(opcode, 8, 4);
    u32 rm = bits(opcode, 0, 4);
    u32 bit_s = bit(opcode, 20);
    u32 bit_a = bit(opcode, 21);
    u32 bit_u = bit(opcode, 22);

    u64 result = 0;

    // TODO: is it bad to use u64/i64?
    // look for a method to do it with only u32/i32
    if (bit_a) {
        if (bit_u) {
            result = (i64)(i32)reg(rm) * (i64)(i32)reg(rs);
            result += ((u64)reg(rdhi) << 32) + reg(rdlo);
        } else {
            result = (u64)reg(rm) * reg(rs);
            result += ((u64)reg(rdhi) << 32) + reg(rdlo);
        }
    } else{
        if (bit_u) {
            result = (i64)(i32)reg(rm) * (i64)(i32)reg(rs);
        } else {
            result = (u64)reg(rm) * reg(rs);
        }
    }

    reg(rdhi) = result >> 32;
    reg(rdlo) = result;

    if (!bit_s)
        return 1;

    // Zero
    if (result == 0) {
        set_bit(this->cpsr, PSR_BIT_Z);
    } else {
        clear_bit(this->cpsr, PSR_BIT_Z);
    }

    // Negative
    if ((i64)result < 0) {
        set_bit(this->cpsr, PSR_BIT_N);
    } else {
        clear_bit(this->cpsr, PSR_BIT_N);
    }

    return 1;
}

void arm_build_decode_table(void)
{
    // TODO Clean up this mess
    for (uint idx = 0; idx < (1 << 12); ++idx) {
        // unpack idx to make it easier to follow documentation
        // bits[0:3] become bits[4:7]
        // bits[4:12] become bits[20:27]
        u32 opcode = (bits(idx, 4, 8) << 20) | (bits(idx, 0, 4) << 4);
        if (bits(opcode, 24, 4) == 0xF) {
            arm_decode_lut[idx] = cpu_software_interrupt;
            continue;
        }
        if (bits(opcode, 22, 6) == 0 && bits(opcode, 4, 4) == 9) {
            arm_decode_lut[idx] = arm_execute_multiply;
            continue;
        }
        if (bits(opcode, 23, 5) == 1 && bits(opcode, 4, 4) == 9) {
            arm_decode_lut[idx] = arm_execute_multiply_long;
            continue;
        }
        if (bits(opcode, 23, 5) == 2 && bits(opcode, 20, 2) == 0 && bits(opcode, 4, 8) == 9) {
            arm_decode_lut[idx] = arm_execute_data_swap;
            continue;
        }
        if (bits(opcode, 20, 8) == 0x12 && bits(opcode, 4, 4) == 1) {
            arm_decode_lut[idx] = arm_execute_branch_exchange;
            continue;
        }
        if (bits(opcode, 25, 3) == 5) {
            arm_decode_lut[idx] = arm_execute_branch;
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
                arm_decode_lut[idx] = arm_execute_psr_transfer;
                continue;
            } else {
                arm_decode_lut[idx] = arm_execute_alu;
                continue;
            }
        }
        if (bits(opcode, 25, 3) == 4) {
            arm_decode_lut[idx] = arm_execute_block_transfer;
            continue;
        }
        if (bits(opcode, 26, 2) == 1) {
            if (bit(opcode, 25) & bit(opcode, 4)) {
                // TODO: this is an undefined instruction
                arm_decode_lut[idx] = arm_execute_not_implemented;
            } else {
                arm_decode_lut[idx] = arm_execute_single_transfer;
            }
            continue;
        }
        if (bit(opcode, 22) == 0 && bits(opcode, 11, 4) != 0) {
            // not signed transfer
        } else if (bit(opcode, 24) == 0 && bit(opcode, 21) == 1) {
            // not signed transfer
        } else if (bits(opcode, 25, 3) == 0 && bit(opcode, 4) && bit(opcode, 7)) {
            arm_decode_lut[idx] = arm_execute_signed_transfer;
            continue;
        }
        arm_decode_lut[idx] = arm_execute_not_implemented;
    }
}

// I recommend not reading this thing
static u32 compute_shift(Cpu *this, u32 opcode, u32 rm, u32 *carry)
{
    u32 type = bits(opcode, 5, 2);
    u32 shift_imm = bits(opcode, 7, 5);
    u32 rs = bits(opcode, 8, 4);
    u32 rs_val = bits(reg(rs), 0, 8);
    u32 bit_r = bit(opcode, 4);
    u32 op2 = 0;

    if (bit_r && rs == 15) {
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
            op2 = bit(this->cpsr, PSR_BIT_C) << 31;
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
        } else if (bits(rs_val, 0, 5) == 0) {
            op2 = rm;
            *carry = bit(rm, 31);
        } else {
            op2 = ror32(rm, bits(rs_val, 0, 5));
            *carry = bit(rm, bits(rs_val, 0, 5) - 1);
        }
    } else {
        assert(0);
    }

    return op2;
}
